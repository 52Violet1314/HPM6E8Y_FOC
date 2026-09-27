/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 诊断状态模块实现
 */

#include <stdio.h>
#include <string.h>

#include "app_diag.h"

volatile app_diag_t g_diag;
volatile app_cal_t  g_cal;

/* ---- 阶段 ---- */
void diag_set_stage(uint32_t stage)
{
    g_diag.stage = stage;
}

/* ---- 心跳 ---- */
void diag_tick(void)
{
    g_diag.loop_cnt++;
}

/* ---- 采样量 ---- */
void diag_set_vbus_mv(uint32_t vbus_mv)
{
    g_diag.vbus_mv = vbus_mv;
}

void diag_set_currents_ma(int32_t iu, int32_t iv, int32_t iw)
{
    g_diag.iu_ma = iu;
    g_diag.iv_ma = iv;
    g_diag.iw_ma = iw;
}

void diag_set_enc(uint32_t angle, uint32_t frames, uint32_t errors,
                  uint32_t raw, uint32_t transfer_fail)
{
    g_diag.enc_angle = angle;
    g_diag.enc_frames = frames;
    g_diag.enc_errors = errors;
    g_diag.enc_raw = raw;
    g_diag.enc_transfer_fail = transfer_fail;
}

/* ---- bit-bang 扫描结果 ---- */
void diag_set_bb(uint32_t hits, uint32_t crc_hits, int32_t best,
                 uint32_t raw, uint32_t miso_static)
{
    g_diag.bb_hits = hits;
    g_diag.bb_crc_hits = crc_hits;
    g_diag.bb_best = best;
    g_diag.bb_raw = raw;
    g_diag.miso_static = miso_static;
}

/* ---- 控制输出 ---- */
void diag_set_control(uint32_t trip,
                      uint32_t ol_voltage_mv,
                      uint32_t ol_elec_angle_mrad,
                      uint32_t duty_u_m, uint32_t duty_v_m, uint32_t duty_w_m)
{
    g_diag.trip = trip;
    g_diag.ol_voltage_mv = ol_voltage_mv;
    g_diag.ol_elec_angle_mrad = ol_elec_angle_mrad;
    g_diag.duty_u_m = duty_u_m;
    g_diag.duty_v_m = duty_v_m;
    g_diag.duty_w_m = duty_w_m;
}

/* ---- 遥测格式化 ---- */
int diag_telemetry(char *buf, uint32_t size, float elec_hz, float mech_rpm,
                   uint32_t vbus_mv)
{
    /* 电频率拆成整数 + 1 位小数, 全部用整数打印避免 %f */
    int elec_hz_x10 = (int)(elec_hz * 10.0f);
    int e_int = elec_hz_x10 / 10;
    int e_dec = elec_hz_x10 % 10;

    if (e_dec < 0) {
        e_dec = -e_dec;
    }

    return snprintf(buf, size,
                    "OL F=%d.%dHz V=%umV D=%u/%u/%u I=%ld/%ld/%ldmA VBUS=%u E=%u RPM=%ld A=%umrad TRIP=%u CNT=%u ERR=%u RAW=%06X TF=%u BB=%u/%u M=%d R=%06X MS=%u",
                    e_int, e_dec,
                    (unsigned)g_diag.ol_voltage_mv,
                    /* 三相占空比 x1000: 用来判断"电压矢量到底是不是静止的" */
                    (unsigned)g_diag.duty_u_m, (unsigned)g_diag.duty_v_m, (unsigned)g_diag.duty_w_m,
                    (long)g_diag.iu_ma, (long)g_diag.iv_ma, (long)g_diag.iw_ma,
                    (unsigned)vbus_mv,
                    (unsigned)g_diag.enc_angle,
                    (long)mech_rpm,
                    (unsigned)g_diag.ol_elec_angle_mrad,
                    (unsigned)g_diag.trip,
                    (unsigned)g_diag.enc_frames,
                    (unsigned)g_diag.enc_errors,
                    (unsigned)g_diag.enc_raw,
                    (unsigned)g_diag.enc_transfer_fail,
                    (unsigned)g_diag.bb_hits,
                    (unsigned)g_diag.bb_crc_hits,
                    (int)g_diag.bb_best,
                    (unsigned)g_diag.bb_raw,
                    (unsigned)g_diag.miso_static);
}

/* ---- 校准遥测 ---- */

/* 映射表: 把编码器一圈分成 CAL_MAP_BINS 箱, 每箱累积 DIFF 的平均值。
 *
 * 【为什么必须在固件里累积】
 *   串口上报是 5Hz, 转速 0.333 圈/s -> 5/0.333 = 15.0 次/圈, 恰好整数!
 *   也就是说每次上报都落在【同一批机械位置】, 一圈只能看到十几个点,
 *   曲线形状完全分辨不出来。改成按控制周期 (1kHz) 累积后, 一转能采约
 *   3000 点, 每箱摊到约 47 个样本, 一次旋转就能把整条曲线填满。
 *
 * 解卷绕: DIFF 在 (-pi, pi] 里, 直接求平均会在 ±pi 附近出错。这里以每箱
 * 第一个样本为参考值记增量, 累加后再加回参考值。 */
#define CAL_MAP_BINS       (64U)
#define CAL_MAP_PER_LINE   (8U)
#define CAL_MAP_LINES      (CAL_MAP_BINS / CAL_MAP_PER_LINE)
#define CAL_MAP_HALF_PI_MRAD  (3141)
#define CAL_MAP_FULL_MRAD     (6283)

static int32_t  s_map_ref[CAL_MAP_BINS];
static int32_t  s_map_acc[CAL_MAP_BINS];
static uint32_t s_map_cnt[CAL_MAP_BINS];

static int32_t cal_map_wrap(int32_t v)
{
    while (v > CAL_MAP_HALF_PI_MRAD) {
        v -= CAL_MAP_FULL_MRAD;
    }
    while (v < -CAL_MAP_HALF_PI_MRAD) {
        v += CAL_MAP_FULL_MRAD;
    }
    return v;
}

void diag_set_cal(uint32_t sw_elec_mrad, uint32_t enc_raw,
                  uint32_t enc_elec_mrad, int32_t diff_mrad)
{
    g_cal.sw_elec_mrad  = sw_elec_mrad;
    g_cal.enc_raw       = enc_raw;
    g_cal.enc_elec_mrad = enc_elec_mrad;
    g_cal.diff_mrad     = diff_mrad;

    /* 按编码器角度分箱累积 */
    {
        uint32_t b = (enc_raw * CAL_MAP_BINS) / 16384U;

        if (b >= CAL_MAP_BINS) {
            b = CAL_MAP_BINS - 1U;
        }
        if (s_map_cnt[b] == 0U) {
            s_map_ref[b] = diff_mrad;
        }
        s_map_acc[b] += cal_map_wrap(diff_mrad - s_map_ref[b]);
        s_map_cnt[b]++;

        /* 数值保护: 定期折半, 保持"滑动平均"语义 */
        if (s_map_cnt[b] >= 2048U) {
            s_map_acc[b] /= 2;
            s_map_cnt[b] = 1024U;
        }
    }
}

int diag_csv_telemetry(char *buf, uint32_t size,
                       int32_t sw_elec_mrad, int32_t enc_elec_mrad,
                       int32_t enc_raw)
{
    /* 【故意不带 "\r\n"】两个日志后端 (console 的 printf("%s\r\n") 与
     * USB 的 write(line)+write("\r\n")) 都已经补了 CRLF。这里再带一次,
     * 串口上每行后面就会多一个空行。 */
    return snprintf(buf, size, "%d,%d,%d",
                    (int)sw_elec_mrad, (int)enc_elec_mrad, (int)enc_raw);
}

int diag_csv_telemetry_lut(char *buf, uint32_t size,
                           int32_t sw_elec_mrad, int32_t enc_elec_mrad,
                           int32_t enc_raw, int32_t lut_mrad)
{
    /* 第 4 列就是"由编码器 raw 查表得到的电角度", 用来和实测 sw 直接对照:
     *   - lut_mrad >= 0 且与 sw 接近  -> 表是对的
     *   - lut_mrad == -1              -> 该 raw 处表项样本不足
     *   - 两者差很多                  -> 表/offset/机械没对齐 */
    return snprintf(buf, size, "%d,%d,%d,%d",
                    (int)sw_elec_mrad, (int)enc_elec_mrad,
                    (int)enc_raw, (int)lut_mrad);
}

int diag_cal_telemetry(char *buf, uint32_t size)
{
    /* 只留校准必需的 4 个数, 尽量短; 单位统一是 mrad (0..6283 = 0..360°) */
    return snprintf(buf, size,
                    "SW=%u ENC=%u ENCEL=%u DIFF=%ld",
                    (unsigned)g_cal.sw_elec_mrad,
                    (unsigned)g_cal.enc_raw,
                    (unsigned)g_cal.enc_elec_mrad,
                    (long)g_cal.diff_mrad);
}

int diag_cal_map_line(char *buf, uint32_t size, uint32_t line)
{
    int n = 0;
    uint32_t i;

    if ((buf == NULL) || (size == 0U) || (line >= CAL_MAP_LINES)) {
        return 0;
    }

    n += snprintf(buf + n, size - (uint32_t)n, "MAP%02u",
                  (unsigned)(line * CAL_MAP_PER_LINE));

    for (i = 0U; i < CAL_MAP_PER_LINE; i++) {
        uint32_t b = line * CAL_MAP_PER_LINE + i;
        int32_t  mean = 0;

        if (s_map_cnt[b] > 0U) {
            mean = cal_map_wrap(s_map_ref[b] +
                                s_map_acc[b] / (int32_t)s_map_cnt[b]);
        }
        n += snprintf(buf + n, size - (uint32_t)n, " %d", (int)mean);
    }

    return n;
}

uint32_t diag_cal_map_count(uint32_t bin)
{
    return (bin < CAL_MAP_BINS) ? s_map_cnt[bin] : 0U;
}

/* ================= 编码器角度查表 (LUT) =================
 *
 * 结构: 每个箱存 "第一个样本的电角度" 作为解卷绕参考, 再累加相对该参考的
 *       增量。这样即使箱内样本跨过 0/6283 边界, 平均值也不会出错。
 * 逐箱圆平均的物理意义: 把转子围绕同步位置的摆动平均掉, 留下该机械位置上
 * 确定的那部分映射 (编码器谐波 + 平均负载角)。 */
static int16_t  s_lut_ref[ENC_LUT_BINS];
static int32_t  s_lut_acc[ENC_LUT_BINS];
static uint16_t s_lut_cnt[ENC_LUT_BINS];
static int16_t  s_lut_lo[ENC_LUT_BINS];    /* 箱内偏差最小值 (相对 ref) */
static int16_t  s_lut_hi[ENC_LUT_BINS];    /* 箱内偏差最大值 */
static uint16_t s_lut_span[ENC_LUT_BINS];  /* 极差 = hi - lo, 判"这箱可不可信" */
static uint32_t s_lut_min_cnt = ENC_LUT_MIN_CNT;  /* 运行时可调 (静态扫描降为 1) */

/* "表项无效"必须用一个不可能和合法角度撞车的值。
 * 合法角度是 (-3141, 3141] mrad, 而负角度是合法的 —— 之前用 -1 同时表示
 * "无效"和"负 1 mrad", 导致查表时把所有负角度箱都当成无效, 打印出一堆 -1。 */
#define ENC_LUT_BIN_INVALID  (0x7FFFFFFF)

static int32_t lut_wrap_mrad(int32_t v)
{
    while (v > 3141)  { v -= 6283; }
    while (v < -3141) { v += 6283; }
    return v;
}

void diag_lut_set_min_cnt(uint32_t n)
{
    s_lut_min_cnt = n;
}

/* 取某箱的平均电角度; 样本不足返回 ENC_LUT_BIN_INVALID */
static int32_t lut_bin_value(uint32_t b)
{
    if ((b >= ENC_LUT_BINS) || (s_lut_cnt[b] < s_lut_min_cnt)) {
        return ENC_LUT_BIN_INVALID;
    }
    return lut_wrap_mrad((int32_t)s_lut_ref[b] +
                         s_lut_acc[b] / (int32_t)s_lut_cnt[b]);
}

/* 运动门控: 环形缓冲里放最近 ENC_LUT_MOVE_WIN 个 raw, 用"当前 - 最老"的
 * 回绕位移判断转子是否真的在转。返回 1 = 在动(样本可用)。 */
static int lut_rotor_moving(uint32_t raw)
{
    static uint32_t hist[ENC_LUT_MOVE_WIN];
    static uint32_t pos;
    static uint32_t fill;
    uint32_t old = hist[pos];
    int32_t  d;

    hist[pos] = raw;
    pos++;
    if (pos >= ENC_LUT_MOVE_WIN) {
        pos = 0U;
    }
    if (fill < ENC_LUT_MOVE_WIN) {
        fill++;
        return 0;                 /* 窗口还没填满, 先不给结论 */
    }

    d = (int32_t)raw - (int32_t)old;
    if (d < 0) { d = -d; }
    if (d > 8192) { d = 16384 - d; }   /* 0/16383 回绕 */

    return (d >= ENC_LUT_MOVE_CNT) ? 1 : 0;
}

/* 【稳态门】见 app_diag.h: 两窗口净行程比。
 * 只认"转子正在以大致恒定的速度跟随磁场"的时刻。 */
static uint32_t s_rate_hist[ENC_LUT_RATE_WIN];
static uint32_t s_rate_pos;
static uint32_t s_rate_fill;

static int lut_rotor_steady(uint32_t raw)
{
    uint32_t old;
    int32_t  sh;
    int32_t  lg;

    old = s_rate_hist[s_rate_pos];
    s_rate_hist[s_rate_pos] = raw;
    s_rate_pos++;
    if (s_rate_pos >= ENC_LUT_RATE_WIN) {
        s_rate_pos = 0U;
    }
    if (s_rate_fill < ENC_LUT_RATE_WIN) {
        s_rate_fill++;
        return 0;
    }

    sh = (int32_t)raw - (int32_t)s_rate_hist[
             (s_rate_pos + ENC_LUT_RATE_WIN - ENC_LUT_RATE_SHORT) % ENC_LUT_RATE_WIN];
    lg = (int32_t)raw - (int32_t)old;
    if (sh < 0) { sh = -sh; }
    if (lg < 0) { lg = -lg; }
    if (sh > 8192) { sh = 16384 - sh; }
    if (lg > 8192) { lg = 16384 - lg; }

    if (lg < (int32_t)ENC_LUT_RATE_MIN) { return 0; }   /* 几乎没走: 卡住/原地抖 */
    if ((sh * 2) < lg)                  { return 0; }   /* 短窗太慢: 刚被顶住 */
    if ((sh * 2) > (lg * 3))            { return 0; }   /* 短窗太快: 正在追赶快跑 */
    return 1;
}

/* 把一个 (编码器角, 磁场电角度) 样本累进对应箱 (圆均值, 跨 0/6283 安全)。
 * 动态学和静态标定共用这一份累加逻辑, 区别只在【要不要过门控】。 */
static void lut_accumulate(uint32_t enc_raw, uint32_t sw_mrad)
{
    uint32_t b = (enc_raw * ENC_LUT_BINS) / 16384U;
    int32_t  d;

    if (b >= ENC_LUT_BINS) {
        b = ENC_LUT_BINS - 1U;
    }

    if (s_lut_cnt[b] == 0U) {
        s_lut_ref[b] = (int16_t)sw_mrad;      /* 解卷绕参考 */
        s_lut_lo[b]  = 0;
        s_lut_hi[b]  = 0;
    }
    d = lut_wrap_mrad((int32_t)sw_mrad - (int32_t)s_lut_ref[b]);
    s_lut_acc[b] += d;
    if (d < (int32_t)s_lut_lo[b]) { s_lut_lo[b] = (int16_t)d; }
    if (d > (int32_t)s_lut_hi[b]) { s_lut_hi[b] = (int16_t)d; }
    s_lut_span[b] = (uint16_t)((int32_t)s_lut_hi[b] - (int32_t)s_lut_lo[b]);
    s_lut_cnt[b]++;

    /* 数值保护: 定期折半, 保持"滑动平均"语义, 防止溢出 */
    if (s_lut_cnt[b] >= 16384U) {
        s_lut_acc[b] /= 2;
        s_lut_cnt[b] = 8192U;
    }
}

/* 【静态标定入口】转子已经吸稳, 磁场角是精确步进值, 不过任何门控。 */
void diag_lut_feed_static(uint32_t enc_raw, uint32_t sw_mrad)
{
    lut_accumulate(enc_raw, sw_mrad);
}

void diag_lut_feed(uint32_t enc_raw, uint32_t sw_mrad)
{
    static int32_t  odo;
    static uint32_t odo_prev;
    static uint8_t  odo_init;
    int32_t  d;

    /* 【起步预热】净行程里程计 (见 app_diag.h): 没真正转过一整圈之前一律不收。
     * 起步暂态里转子原地晃, 净行程≈0, 这里直接把那些样本挡在外面。 */
    if (odo_init == 0U) {
        odo_prev = enc_raw;
        odo_init = 1U;
        return;
    }
    d = (int32_t)enc_raw - (int32_t)odo_prev;
    odo_prev = enc_raw;
    if (d >  8192) { d -= 16384; }
    if (d < -8192) { d += 16384; }
    odo += d;

    if ((odo < (int32_t)ENC_LUT_WARMUP_COUNTS) &&
        (odo > -(int32_t)ENC_LUT_WARMUP_COUNTS)) {
        return;
    }

    /* 【运动门控】转子这一拍没动 -> 样本不可信, 直接丢 (见 app_diag.h 说明) */
    if (lut_rotor_moving(enc_raw) == 0) {
        return;
    }
    /* 【稳态门】只在"稳定跟随"时采样: 卡住/追赶的瞬间一律不要 */
    if (lut_rotor_steady(enc_raw) == 0) {
        return;
    }

    lut_accumulate(enc_raw, sw_mrad);
}

uint32_t diag_lut_valid_bins(void)
{
    uint32_t i;
    uint32_t n = 0U;

    for (i = 0U; i < ENC_LUT_BINS; i++) {
        if (s_lut_cnt[i] >= s_lut_min_cnt) {
            n++;
        }
    }
    return n;
}

uint32_t diag_lut_max_hole(void)
{
    uint32_t best = 0U;
    uint32_t run  = 0U;
    uint32_t i;

    for (i = 0U; i < (2U * ENC_LUT_BINS); i++) {
        uint32_t b = i % ENC_LUT_BINS;
        uint8_t  bad = ((s_lut_cnt[b] < s_lut_min_cnt) ||
                        ((int32_t)s_lut_span[b] > ENC_LUT_SPAN_MAX)) ? 1U : 0U;

        if (bad != 0U) {
            run++;
            if (run > best) { best = run; }
        } else {
            run = 0U;
        }
    }
    return (best > ENC_LUT_BINS) ? ENC_LUT_BINS : best;
}

uint32_t diag_lut_angle(uint32_t enc_raw)
{
    uint32_t b = (enc_raw * ENC_LUT_BINS) / 16384U;
    int32_t  v0;
    int32_t  v1;
    int32_t  vm1;
    int32_t  s_num;
    int32_t  s0_num;
    int32_t  ds;
    int32_t  d;
    int32_t  val;

    if (b >= ENC_LUT_BINS) {
        b = ENC_LUT_BINS - 1U;
    }

    v0 = lut_bin_value(b);
    if (v0 == ENC_LUT_BIN_INVALID) {
        return 0xFFFFFFFFU;         /* 该处没有有效表项 */
    }

    /* ---- 按【箱中心】线性插值 ----
     * 箱宽 W = 16384/BINS; 对 1024 箱正好是 16。
     * (旧代码 800 箱时 W=20.48 不整除, 却用整数 20 去算箱内位置, 箱号越大
     *  偏得越多, 这就是"越到后面查表值差越多"的根源; 现在 BINS=1024 已整除,
     *  这里的整数标度写法仍然通用。)
     * 用整数标度 s = 2*raw*BINS/16384 表示位置, 箱 b 中心对应 s = 2b+1,
     * 相邻箱中心差 2, 所以插值分母用 2*16384。 */
    s_num  = (int32_t)(enc_raw * 2U * ENC_LUT_BINS);       /* s * 16384 */
    s0_num = (int32_t)((2U * b + 1U) * 16384U);            /* 箱 b 中心 */
    ds     = s_num - s0_num;                               /* (-16384, 16384) */

    if (ds >= 0) {
        /* 后半箱: v0 -> v1, t = ds / (2*16384) */
        v1 = lut_bin_value((b + 1U) % ENC_LUT_BINS);
        if (v1 == ENC_LUT_BIN_INVALID) {
            return (uint32_t)v0;
        }
        d   = lut_wrap_mrad(v1 - v0);
        val = v0 + (int32_t)(((int64_t)d * ds) / (2 * 16384));
    } else {
        /* 前半箱: vm1 -> v0, t = (ds + 2*16384) / (2*16384) */
        vm1 = lut_bin_value((b + ENC_LUT_BINS - 1U) % ENC_LUT_BINS);
        if (vm1 == ENC_LUT_BIN_INVALID) {
            return (uint32_t)v0;
        }
        d   = lut_wrap_mrad(v0 - vm1);
        val = vm1 + (int32_t)(((int64_t)d * (ds + 2 * 16384)) / (2 * 16384));
    }

    return (uint32_t)lut_wrap_mrad(val);
}

uint32_t diag_lut_repair(void)
{
    /* 坏箱 = 样本不足, 或箱内极差过大(学习时转子被顶住、磁场还在扫)。
     * 处理: 左右各找最近的【好箱】, 按 bin 距离线性插值 (走回绕), 写回该箱。
     * 必须在"表学满"之后、"切换向"之前调用一次。 */
    static uint8_t bad[ENC_LUT_BINS];
    uint32_t i;
    uint32_t n_fix = 0U;

    for (i = 0U; i < ENC_LUT_BINS; i++) {
        bad[i] = ((s_lut_cnt[i] < s_lut_min_cnt) ||
                  ((int32_t)s_lut_span[i] > ENC_LUT_SPAN_MAX)) ? 1U : 0U;
    }

    for (i = 0U; i < ENC_LUT_BINS; i++) {
        uint32_t p;
        uint32_t n;
        int32_t  vp;
        int32_t  vn;
        int32_t  d;
        int32_t  val;

        if (bad[i] == 0U) {
            continue;
        }

        for (p = 1U; p < ENC_LUT_BINS; p++) {
            if (bad[(i + ENC_LUT_BINS - p) % ENC_LUT_BINS] == 0U) { break; }
        }
        for (n = 1U; n < ENC_LUT_BINS; n++) {
            if (bad[(i + n) % ENC_LUT_BINS] == 0U) { break; }
        }
        if ((p >= ENC_LUT_BINS) || (n >= ENC_LUT_BINS)) {
            continue;               /* 整张表都是坏箱: 没得插, 保持原样 */
        }

        vp = lut_bin_value((i + ENC_LUT_BINS - p) % ENC_LUT_BINS);
        vn = lut_bin_value((i + n) % ENC_LUT_BINS);
        if ((vp == ENC_LUT_BIN_INVALID) || (vn == ENC_LUT_BIN_INVALID)) {
            continue;
        }

        /* 先对两侧差值解卷绕, 再按箱距线性分摊。
         * 【回绕分支必须选对】表沿 raw 是单调递增的 (整圈机械角 = 12 个电周期),
         * 平均每箱前进 ENC_LUT_STEP_NOMINAL ≈ 73.6 mrad。
         * lut_wrap_mrad() 给的是"短支"; 缺口一旦超过半个电周期, 短支就会把斜率
         * 的方向插反, 补出一段"估计角度随 raw 倒退"的表。
         * 实测 (1.5V 学表): 坏箱 539..575 共 37 个, 两侧好箱 538=-1223、
         * 576=-2107, 短支 = -884 mrad -> 补成 -23 mrad/箱 的下降斜坡; 而正确分支
         * 是 -884+6283 = +5399 -> +142 mrad/箱。用错的那版换向后补偿角误差在这
         * 38 箱里扫过 200° 以上, 转子直接焊死在 raw≈8755 (log 里 sw 恒在 150 附近)。
         * 判据: 在 {短支 + k*6283} 里选"缺口平均斜率最接近 73.6 mrad/箱"的一支 ——
         * 这一支唯一且和编码器刻度无关。 */
        d = lut_wrap_mrad(vn - vp);
        {
            int32_t num = (int32_t)ENC_LUT_STEP_NOMINAL * (int32_t)(p + n) - d;
            int32_t k   = (num >= 0) ? ((num + 3141) / 6283)
                                     : -(((-num) + 3141) / 6283);

            d += k * 6283;
        }

        val = vp + (int32_t)(((int64_t)d * (int32_t)p) / (int32_t)(p + n));
        val = lut_wrap_mrad(val);

        s_lut_ref[i]  = 0;
        s_lut_cnt[i]  = (uint16_t)s_lut_min_cnt;
        s_lut_acc[i]  = val * (int32_t)s_lut_min_cnt;
        s_lut_span[i] = 0U;
        bad[i]        = 0U;         /* 修好的箱可以给后面的坏箱当参考 */
        n_fix++;
    }

    /* 回绕处 (0/1023) 也要能互相参考, 上面用取模已经覆盖 */
    for (i = 0U; i < ENC_LUT_BINS; i++) {
        if (s_lut_cnt[i] < s_lut_min_cnt) {
            s_lut_cnt[i] = (uint16_t)s_lut_min_cnt;   /* 兜底: 不留无效箱 */
        }
    }

    return n_fix;
}

static uint32_t s_spd_min[ENC_SPDMAP_SECT];
static uint32_t s_spd_cnt[ENC_SPDMAP_SECT];
static uint8_t  s_spd_done;

int diag_spdmap_feed(uint32_t raw)
{
    static uint32_t prev;
    static uint32_t acc;
    static uint32_t n;
    static uint8_t  started;
    uint32_t sect;
    uint32_t i;
    int32_t  d;

    if (s_spd_done != 0U) {
        return 0;
    }

    if (started == 0U) {
        for (i = 0U; i < ENC_SPDMAP_SECT; i++) {
            s_spd_min[i] = 0xFFFFFFFFU;
        }
        prev    = raw;
        started = 1U;
        return 0;
    }

    d    = (int32_t)raw - (int32_t)prev;
    prev = raw;
    if (d < 0) { d = -d; }
    if (d > 8192) { d = 16384 - d; }
    acc += (uint32_t)d;
    n++;
    if (n < ENC_SPDMAP_WIN) {
        return 0;
    }

    sect = (prev * ENC_SPDMAP_SECT) / 16384U;
    if (sect >= ENC_SPDMAP_SECT) {
        sect = ENC_SPDMAP_SECT - 1U;
    }
    if (acc < s_spd_min[sect]) {
        s_spd_min[sect] = acc;
    }
    if (s_spd_cnt[sect] < 0xFFFFU) {
        s_spd_cnt[sect]++;
    }
    acc = 0U;
    n   = 0U;

    for (i = 0U; i < ENC_SPDMAP_SECT; i++) {
        if (s_spd_cnt[i] < ENC_SPDMAP_MIN) {
            return 0;
        }
    }
    s_spd_done = 1U;
    return 1;
}

uint32_t diag_spdmap_get(uint32_t sect)
{
    if (sect >= ENC_SPDMAP_SECT) {
        return 0U;
    }
    return (s_spd_cnt[sect] == 0U) ? 0U : s_spd_min[sect];
}

int diag_lut_dwell(uint32_t raw)
{
    /* 换向后的卡顿定位。判据与 lut_rotor_moving 完全一样, 但用独立缓冲,
     * 避免和学表阶段的门控互相踩。 */
    static uint32_t hist[ENC_LUT_MOVE_WIN];
    static uint32_t pos;
    static uint32_t fill;
    uint32_t old = hist[pos];
    int32_t  d;

    hist[pos] = raw;
    pos++;
    if (pos >= ENC_LUT_MOVE_WIN) {
        pos = 0U;
    }
    if (fill < ENC_LUT_MOVE_WIN) {
        fill++;
        return 0;
    }

    d = (int32_t)raw - (int32_t)old;
    if (d < 0) { d = -d; }
    if (d > 8192) { d = 16384 - d; }   /* 0/16383 回绕 */

    return (d < ENC_LUT_MOVE_CNT) ? 1 : 0;
}

int32_t diag_lut_health(uint32_t *p_back, int32_t *p_worst_neg, int32_t *p_worst_pos)
{
    /* 只读表, 不改表。每一步在 {短支 + k*6283} 里选最接近名义步长的那一支:
     *   合法回绕 (例: +3094 -> -3074, 短支 -6168) 会被还原成 +115, 不算坏;
     *   真正的倒退 (步长就是 -10 这种) 保持负值, 被 back 抓出来。
     * 这样 sum 才有意义: 绕一圈必须正好是 12 个电周期 = 75396 mrad,
     * 差一个 6283 就说明有整片插反(那种表换向后一定卡死)。 */
    int32_t  sum  = 0;
    int32_t  wneg = 0;
    int32_t  wpos = 0;
    uint32_t back = 0U;
    uint32_t i;

    for (i = 0U; i < ENC_LUT_BINS; i++) {
        int32_t a = lut_bin_value(i);
        int32_t b = lut_bin_value((i + 1U) % ENC_LUT_BINS);
        int32_t d;
        int32_t num;
        int32_t k;

        if ((a == ENC_LUT_BIN_INVALID) || (b == ENC_LUT_BIN_INVALID)) {
            continue;
        }

        d   = lut_wrap_mrad(b - a);
        num = (int32_t)ENC_LUT_STEP_NOMINAL - d;
        k   = (num >= 0) ? ((num + 3141) / 6283) : -(((-num) + 3141) / 6283);
        d  += k * 6283;

        sum += d;
        if (d < 0) {
            back++;
            if (d < wneg) { wneg = d; }
        }
        if (d > wpos) { wpos = d; }
    }

    if (p_back      != NULL) { *p_back      = back; }
    if (p_worst_neg != NULL) { *p_worst_neg = wneg; }
    if (p_worst_pos != NULL) { *p_worst_pos = wpos; }

    return sum;
}

int diag_lut_dump_line(char *buf, uint32_t size, uint32_t line)
{
    int n = 0;
    uint32_t i;

    if ((buf == NULL) || (line >= ENC_LUT_LINES)) {
        return 0;
    }

    n += snprintf(buf + n, size - (uint32_t)n, "LUT%03u",
                  (unsigned)(line * ENC_LUT_PER_LINE));

    for (i = 0U; i < ENC_LUT_PER_LINE; i++) {
        uint32_t b = line * ENC_LUT_PER_LINE + i;
        int32_t  v = lut_bin_value(b);

        if (v == ENC_LUT_BIN_INVALID) {
            v = -1;             /* 打印时仍用 -1 表示"还没学到" */
        }
        n += snprintf(buf + n, size - (uint32_t)n, " %d", (int)v);
    }
    return n;
}
