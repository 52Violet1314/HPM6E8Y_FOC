/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 应用编排层实现
 *
 *   只做"组装": 把 app_diag / app_log / app_encoder / foc_controller /
 *   bsp 串成一个完整应用。具体逻辑都在各自模块内, 本文件保持精简。
 */

#include <stdio.h>
#include <string.h>

#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"

#include "app.h"
#include "app_diag.h"
#include "app_log.h"
#include "app_encoder.h"
#include "app_nv.h"

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_usb.h"
#include "bsp_mt6701.h"
#include "bsp_spi.h"
#include "bsp_spi_bb.h"

#include "foc_controller.h"

/* ================= 应用实例 ================= */
static foc_controller_t s_ctrl;
static app_encoder_t    s_encoder;

/* 校准映射表转储节拍: 每 APP_CAL_MAP_DUMP_TICKS 次上报打印一次整张表。
 * 0 = 关闭。转储会连发 8 次 USB 写, 阻塞控制环、拖慢主循环 —— 这是开环
 * 失步的诱因之一, 默认关闭, 需要看曲线时再打开。 */
static uint32_t s_map_tick;
#define APP_CAL_MAP_DUMP_TICKS  (0U)

#if APP_ENC_LUT_AUTO_SWITCH
/* 0 = 正在学表(电压矢量角度仍用模拟积分角)
 * 1 = 表已学满, 电压矢量角度改用 diag_lut_angle(raw) + 超前角 */
static uint8_t s_lut_full;
/* 1 = 静态扫描造的表被体检否决 (缺口太长 / 整圈不自洽) -> 不换向, 保持开环 VF。
 * 宁可让电机照原样转, 也不能拿一张"编出来"的表去换向 —— 那正是转子在补出来的
 * 那一段里反复弹、堵转发热的根源。 */
static uint8_t s_lut_reject;
/* 换向后"顶住"的连续拍数, 到 APP_ENC_DWELL_LOG_TICKS 就打一行位置 */
static uint32_t s_dwell_ticks;
static uint32_t s_stuck_ms;         /* 换向后连续"顶住"的毫秒数 */
static uint32_t s_kick_ms;          /* 反磁弹簧踢一脚的剩余毫秒数 */
#endif

#if APP_CAL_SWEEP_ENABLE
/* 静态角度扫描标定状态 */
static uint32_t s_sweep_step;
static uint32_t s_sweep_cnt;
static uint32_t s_sweep_ref;        /* 本次保持的 raw 解卷绕参考 */
static int32_t  s_sweep_a1;         /* 前半段(正常电压)增量累加 */
static int32_t  s_sweep_a2;         /* 后半段(弱场)增量累加 */
static uint8_t  s_sweep_done;
static int32_t  s_cal_offset_mrad = (int32_t)(FOC_ELEC_ANGLE_OFFSET * 1000.0f); /* 存档用标定偏移 */
static uint32_t s_sweep_steps;      /* = PHASES × 极对数 × REVS, app_init 里算 */
static uint32_t s_cyc_ref;          /* 上一个电周期边界的 raw, 用来算本周期推进量 */
/* 残差汇总 (用于自动给出推荐偏置) */
static int32_t  s_sweep_res_ref;    /* 第一个残差, 作为解卷绕参考 */
static int32_t  s_sweep_res_acc;    /* 解卷绕后的残差累加 */
static uint32_t s_sweep_res_n;
static int32_t  s_sweep_res_min;
static int32_t  s_sweep_res_max;
/* 强/弱场位移差 dE 的统计: dE = 4·δ_强, 直接反映齿槽/静摩擦把转子拉偏多少 */
static int32_t  s_sweep_de_acc;
static int32_t  s_sweep_de_min;
static int32_t  s_sweep_de_max;
static uint32_t s_sweep_de_n;
static uint32_t s_sweep_de_big;
/* 【整圈行程里程计】扫描的停止条件 = 编码器【净行程走满一整圈(16384 counts)】,
 * 而不是固定步数。固定步数在"极对数不是配置的 12"或"中途丢步"时会留下一段
 * 从没踩到的 raw 区间, 那段只能靠弦插值编出来 —— 换向后就是磁弹簧陷阱。 */
static uint32_t s_sweep_odo_prev;
static int32_t  s_sweep_odo;        /* 净行程 counts, 正 = 沿 raw 增大方向 */
static int32_t  s_sweep_dmax;       /* 单步行程最大/最小值, 看有没有丢步 */
static int32_t  s_sweep_dmin;
static uint32_t s_sweep_stall;      /* 连续"一步没动"的步数 -> 卡住了就提前收尾 */
#define APP_CAL_SWEEP_STALL_MAX  (40U)
static uint32_t s_sweep_pairs;      /* 实测: 走满一整圈磁场转了几个电周期 = 极对数 */
static uint32_t s_sweep_pre;        /* 预吸稳已经过的主循环数 */

/* 把 14bit raw 差值折算到 (-8192, 8192] —— 抗 16384->0 回绕 */
static int32_t cal_wrap_raw(int32_t v)
{
    while (v > 8192)  { v -= 16384; }
    while (v < -8192) { v += 16384; }
    return v;
}

/* 把 mrad 折算到 (-pi, pi] */
static int32_t cal_wrap_mrad(int32_t v)
{
    while (v > 3141) {
        v -= 6283;
    }
    while (v < -3141) {
        v += 6283;
    }
    return v;
}
#endif

/* 当前三相电流缓存 (mA), 供中断回调与主循环共享 */
static volatile int32_t s_iu_ma;
static volatile int32_t s_iv_ma;
static volatile int32_t s_iw_ma;

#if APP_FOC_CLOSED_LOOP_ENABLE
/* 闭环过流保护标志: 一旦触发就永久关断输出, 必须复位重跑才恢复 */
static uint8_t s_oc_tripped;

/* 只用比较实现, 避免依赖 math.h / libm */
static float app_abs_f(float v)
{
    return (v < 0.0f) ? -v : v;
}
#endif

/* ================= 日志后端 ================= */
/* printf 调试口后端 */
static void log_backend_console(const char *line)
{
    printf("%s\r\n", line);
}

/* USB 虚拟串口后端 (仅主机已连接时输出) */
static void log_backend_usb(const char *line)
{
    if (bsp_usb_connected()) {
        (void)bsp_usb_write((const uint8_t *)line, (uint32_t)strlen(line));
        (void)bsp_usb_write((const uint8_t *)"\r\n", 2);
    }
}

/* ================= 中断回调 (由 BSP 在中断上下文调用) ================= */

/* ADC 采样完成: 缓存电流并更新诊断 */
static void app_on_adc_conv(void)
{
    bsp_adc_get_currents_ma((int32_t *)&s_iu_ma,
                            (int32_t *)&s_iv_ma,
                            (int32_t *)&s_iw_ma);
    diag_set_currents_ma(s_iu_ma, s_iv_ma, s_iw_ma);
}

/* MT6701 新帧到达 (SPI 后端在主循环里 poll, 属线程上下文, 不是中断) */
static void app_on_mt6701_frame(const mt6701_frame_t *frame, uint32_t raw)
{
    (void)frame;
    (void)raw;
    /* 编码器诊断在主循环 poll 后统一刷新 (见 app_run), 这里预留
     * 给未来需要在"新帧到达"时刻做额外处理的逻辑。 */
}

/* USB 主机下发数据 */
static void app_on_usb_rx(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
    /* 预留: 可在此解析主机命令 (启停/调速等) */
}

/* ================= 初始化 ================= */
void app_init(void)
{
    float dt = (float)APP_MAIN_LOOP_MS * 0.001f;

    /* 诊断状态清零 */
    memset((void *)&g_diag, 0, sizeof(g_diag));

    /* 日志: 注册 console + USB 两个后端 */
    app_log_add_backend(log_backend_console);
    app_log_add_backend(log_backend_usb);

    LOG_I("APP", "YeeZi HPM6EMotor open-loop demo");

    /* 外设初始化顺序: PWM -> ADC -> USB.
     * PWM 先初始化很重要 —— ADC 触发源来自 PWM TRGO */
    bsp_pwm_init();
    diag_set_stage(APP_STAGE_PWM_INIT);
    bsp_adc_init();
    diag_set_stage(APP_STAGE_ADC_INIT);
    bsp_usb_init();
    diag_set_stage(APP_STAGE_USB_INIT);

    /* MT6701 (编码器) 初始化: SPI2 @ J11 扩展排针
     *   PC26=SCLK(脚31) / PC28=MISO(脚10) / PC24=CS(脚11)
     * 走普通 SPI 主机, 没有硬件周期触发, 由主循环主动 poll。 */
    bsp_mt6701_register_frame_cb(app_on_mt6701_frame);
    bsp_mt6701_init();
    diag_set_stage(APP_STAGE_SEI_INIT);
    LOG_I("ENC", "MT6701 SPI2 @ J11: SCLK=PC26(pin31) MISO=PC28(pin10) CS=PC24(pin11)");
    /* 时钟/分频诊断: 确认源时钟、实际 SCLK、以及时序初始化是否成功
     * (timing=0 表示 status_success) */
    LOG_I("ENC", "SPI2 src_clk=%u Hz sclk=%u Hz timing=%d",
          (unsigned)bsp_spi_get_src_clk(),
          (unsigned)bsp_spi_get_sclk(),
          bsp_spi_get_timing_stat());

    /* ---- 编码器接线 bit-bang 排查 ----
     * 用 GPIO 软件模拟 SPI, 把 16 种 CPOL/CPHA/CS方式/位序 组合全试一遍。
     * 目的: 把"SPI2 外设"和"模式配对"这两个变量一起排除掉。
     *   - 有组合读到变化数据  -> 编码器是活的, 只是模式/接线不对;
     *   - 16 种全部恒定不变   -> DOUT 上没有任何器件在响应, 纯硬件问题。 */
    bsp_bb_scan();
    diag_set_bb(bsp_bb_hit_count(), bsp_bb_crc_hit_count(),
                bsp_bb_best_index(), bsp_bb_best_raw(),
                (uint32_t)bsp_bb_static_level());

    LOG_I("ENC", "bitbang: MISO static=%u, hit=%u/16, crc_hit=%u/16, best=%d",
          (unsigned)bsp_bb_static_level(),
          (unsigned)bsp_bb_hit_count(),
          (unsigned)bsp_bb_crc_hit_count(),
          (int)bsp_bb_best_index());

    /* 把 CRC 通过 / 读到变化数据的组合逐条打出来, 便于复盘 */
    {
        char line[192];
        uint32_t i;

        for (i = 0U; i < BSP_BB_MODE_COUNT; i++) {
            const bsp_bb_result_t *r = bsp_bb_result_at(i);

            if ((r != NULL) && ((r->crc_ok > 0U) || (r->distinct >= 2U))) {
                (void)bsp_bb_format(line, sizeof(line), i);
                LOG_I("ENC", "%s", line);
            }
        }
    }

    /* 只要扫描找到了 CRC 可信的组合, 就直接启用它。
     * 实测硬件 SPI2 这条路 (即使已改成 Mode 0) 仍然读不出数据, 而 bit-bang
     * 已被 CRC6 证明能读出 100% 合法的帧 —— 先用能工作的这条把编码器跑起来。
     * idx=8 => CS 每帧翻转 + CPOL=0 + CPHA=0 + MSB first。 */
    if (bsp_bb_best_index() >= 0) {
        (void)bsp_bb_activate((uint32_t)bsp_bb_best_index());
        LOG_I("ENC", "encoder reader -> bitbang idx=%d (CRC-validated)",
              (int)bsp_bb_best_index());
    }
    /* 【BUILD 标记】日志里一眼看出跑的是哪一版, 免得再来回猜 */
    LOG_I("ENC", "BUILD: sweep=%d presetttle=%u mincnt=%u kick_ms=%u",
          (int)APP_CAL_SWEEP_ENABLE, (unsigned)APP_CAL_SWEEP_PRESETTLE,
          (unsigned)ENC_LUT_MIN_CNT, (unsigned)APP_ENC_STUCK_KICK_MS);
    {
    }

    /* 统一 FOC 控制器: 默认开环模式 */
    foc_controller_init(&s_ctrl, dt);
    foc_controller_set_voltage(&s_ctrl, APP_OPENLOOP_VOLTAGE_V);
    foc_controller_set_current_limit(&s_ctrl, APP_OPENLOOP_CURRENT_LIMIT);

    /* ---- 非易失校准数据 (Flash) ----
     * 有合法存档就直接查表换向, 跳过 3 分钟静态扫描。
     * 【不调 app_nv_init()】上电时 CPU 正在这片 NOR 上取指, 任何 XPI 重配置
     * 都会把启动流程卡死 (症状: BUILD 行之后无输出)。读档是纯 XIP 直读,
     * 不需要配置; 只有"写存档"时才在 RAM 里懒加载配置。 */
    {
        static int16_t nv_tbl[ENC_LUT_BINS];
        int32_t        nv_off = 0;

        if (app_nv_load(nv_tbl, &nv_off)) {
            diag_lut_load(nv_tbl);
            app_encoder_set_elec_offset_rad((float)nv_off * 0.001f);
            s_lut_full   = 1U;
            s_sweep_done = 1U;
            foc_controller_set_voltage(&s_ctrl, APP_ENC_LUT_RUN_VOLTAGE_V);
            foc_controller_set_current_limit(&s_ctrl, APP_ENC_LUT_RUN_CURRENT_LIMIT_A);
            LOG_I("ENC", "NV: loaded LUT + offset from flash (offset=%ld mrad) -> LUT commutation, skip cal",
                  (long)nv_off);
        } else {
            LOG_I("ENC", "NV: no valid table in flash -> run calibration");
        }
    }

    s_ctrl.openloop.elec_accel = APP_OPENLOOP_ACCEL;
#if APP_FOC_CLOSED_LOOP_ENABLE
    /* ---- 闭环: 用编码器角度换向 ----
     * 电角度不再来自内部积分, 而是 app_encoder_get_elec_rad()
     * = normalize(mech×POLE_PAIRS + FOC_ELEC_ANGLE_OFFSET)。
     * 这是 FOC_ELEC_ANGLE_OFFSET 第一次真正参与控制。 */
    foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_current);
    foc_controller_set_current_ref(&s_ctrl, APP_FOC_ID_REF_A, APP_FOC_IQ_REF_A);
    foc_controller_set_gains(&s_ctrl, APP_FOC_CURRENT_KP, APP_FOC_CURRENT_KI);
    LOG_I("FOC", "CLOSED LOOP on: id=%d mA iq=%d mA kp=%d ki=%d rpm_limit=%d",
          (int)(APP_FOC_ID_REF_A * 1000.0f), (int)(APP_FOC_IQ_REF_A * 1000.0f),
          (int)APP_FOC_CURRENT_KP, (int)APP_FOC_CURRENT_KI,
          (int)APP_FOC_RPM_LIMIT);
#else
#  if APP_CAL_SWEEP_ENABLE
    if (s_lut_full == 0U) {
    /* 静态扫描标定: 磁场从 0° 开始分步固定, 由 app_run 推进 */
    /* 这里算的是【步数上限】: 正常会在"净行程走满一整圈"时提前收尾 (见 app_run
     * 里的里程计)。给 1.5 倍余量, 极对数比配置值大 (实测配 12 时 12 个电周期
     * 只走了 0.95 圈) 或中途丢过步也能扫完整圈。 */
    /* 【步数上限从 ×1.5 提到 ×2.5】(3600 -> 6000 步)
     * 真正收尾靠下面的净行程里程计 (odo 满 16384 = 走满整圈), 这里只是兜底上限。
     * 原来只给 1.5 倍余量: 弱场下转子打滑时 (实测每电周期只走 1026/640 counts,
     * 理想 1365), 3600 步只覆盖约 91% 的圈 -> 缺口 > 16 箱 -> 直接被否掉。
     * 给到 2.5 倍后即使按 3 counts/步 的严重打滑也能走完 (6000×3 = 18000 > 16384),
     * 而正常跟得住时里程计会在 ~2400 步就收尾, 时间不变。 */
    s_sweep_steps = APP_CAL_SWEEP_PHASES
                    * ((uint32_t)FOC_MOTOR_POLE_PAIRS * 5U / 2U)
                    * APP_CAL_SWEEP_REVS;
    if (s_sweep_steps < APP_CAL_SWEEP_PHASES) {
        s_sweep_steps = APP_CAL_SWEEP_PHASES;
    }
    s_ctrl.openloop.voltage_slew = APP_CAL_SWEEP_SLEW;   /* 扫描期间电压要能快速切换 */
    /* 【拉入】先用 APP_OPENLOOP_VOLTAGE_V(1.5V) 把转子从上电位置可靠拽到 0°,
     * 预吸稳那 1.5s 里再经斜坡降到标定电压 0.65V。一上来就用 0.65V 有风险:
     * 转子若停在齿槽谷底可能拽不动, 整场扫描直接废掉。1.5V 只维持 1.5 秒,
     * 发热可以忽略。 */
    foc_controller_set_voltage(&s_ctrl, APP_OPENLOOP_VOLTAGE_V);
    foc_controller_set_align_angle(&s_ctrl, 0.0f);
    /* 静态每箱只有一个吸稳+外推样本, 有效门限用 3 会把整圈都扫完的表误拒
     * (实测 missed=664 maxhole=43 -> REJECTED)。降到 1: 实测缺口最长 11 箱,
     * 弦插值在本地线性的表上完全够用。 */
    diag_lut_set_min_cnt(1U);
    LOG_I("ENC", "CAL SWEEP start: <=%u steps x %u loops, V=%d/%d mV, mincnt=1 (走到净行程满整圈为止)",
          (unsigned)s_sweep_steps, (unsigned)APP_CAL_SWEEP_HOLD,
          (int)(APP_CAL_SWEEP_HIGH_V * 1000.0f),
          (int)(APP_CAL_SWEEP_LOW_V * 1000.0f));
    }
#  elif APP_OPENLOOP_ALIGN_ONLY
    foc_controller_set_align_angle(&s_ctrl, APP_OPENLOOP_ALIGN_DEG * (FOC_TWO_PI / 360.0f));
#  elif APP_ENC_ANGLE_DRIVE
    /* 编码器换向: 角度由 app_run 每周期写入 (enc_elec + 超前角),
     * 所以这里【不设开环速度】—— 内部积分保持不动。 */
    LOG_I("FOC", "ENCODER COMMUTATION on: lead=%d deg, V=%u mV",
          (int)APP_ENC_LEAD_DEG, (unsigned)(APP_OPENLOOP_VOLTAGE_V * 1000.0f));
#  else
    foc_controller_set_speed_rpm(&s_ctrl, APP_OPENLOOP_TARGET_RPM);
#  endif
#endif
#if APP_OPENLOOP_DEMO_ENABLE
    foc_controller_enable(&s_ctrl, true);
#endif
    diag_set_stage(APP_STAGE_FOC_INIT);

    /* 编码器封装: 速度估算低通系数 0.1 */
    app_encoder_init(&s_encoder, dt, 0.1f);

    /* 注册中断回调: 硬件中断留在 BSP, 应用逻辑经此上抛
     * (编码器回调已在上方 bsp_mt6701_init() 之前注册) */
    bsp_adc_register_conv_cb(app_on_adc_conv);
    bsp_usb_register_rx_cb(app_on_usb_rx);

    /* 初始 50% 占空比, 打开三相输出 */
    bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
    bsp_pwm_enable_output(true);
    diag_set_stage(APP_STAGE_PWM_ENABLE);

    LOG_I("PWM", "reload=%u, freq=%u Hz, poles=%u, target=%u RPM, V=%u mV",
          (unsigned)bsp_pwm_get_reload(), (unsigned)BSP_PWM_FREQUENCY_HZ,
          (unsigned)FOC_MOTOR_POLE_PAIRS,
          (unsigned)APP_OPENLOOP_TARGET_RPM,
          (unsigned)(APP_OPENLOOP_VOLTAGE_V * 1000.0f));
    LOG_I("APP", "Tips: watch g_diag in debugger; vbus_mv must be > %u mV",
          (unsigned)(FOC_OPENLOOP_VBUS_MIN_V * 1000.0f));
}

/* ================= 主循环 ================= */
void app_run(void)
{
    static uint32_t tick = 0;
    int32_t iu_ma = 0, iv_ma = 0, iw_ma = 0;
    float ia, ib, ic, vbus;
    float du, dv, dw;

    diag_set_stage(APP_STAGE_LOOP_TOP);
    diag_tick();

    /* ---- 采集: 编码器 / 电流 / 母线电压 ---- */
    /* MT6701 走 SPI, 没有硬件周期触发, 由控制周期主动读一帧 (1kHz) */
    (void)bsp_mt6701_poll();
    /* 无论本帧成败都刷新诊断: 只靠成功回调的话, 编码器没接/没供电时
     * (poll 一直失败) CNT/ERR 会永远停在 0, 看不出任何信息。 */
    diag_set_enc(bsp_mt6701_get_raw_angle(),
                 bsp_mt6701_get_frame_count(),
                 bsp_mt6701_get_error_count(),
                 bsp_mt6701_get_last_raw(),
                 bsp_mt6701_get_transfer_fail());
    (void)app_encoder_update(&s_encoder);

    /* ---- 【编码器/校准体检】每 1s 一行, 串口直接看 ----
     * 编码器没接/没供电/线断/被烤坏时, 不接调试器也能一眼判定:
     *   frames 恒 0 + errs 每拍都涨 + miso=0  ->  编码器根本没有响应,
     *   此时固件看到的"转子不动"是真的读不到角度, 不是电机的问题。 */
    {
        static uint32_t enc_log_tick = 0U;
        static uint32_t enc_no_frame  = 0U;

        enc_log_tick += APP_MAIN_LOOP_MS;
        if (enc_log_tick >= 1000U) {
            uint32_t fr = bsp_mt6701_get_frame_count();
            uint32_t er = bsp_mt6701_get_error_count();
            uint32_t tf = bsp_mt6701_get_transfer_fail();
            uint8_t  mi = bsp_bb_miso_live();

            enc_log_tick = 0U;

            LOG_I("ENC",
                  "health: reader=%s bb=%d miso=%u scan(hit=%u crc=%u best=%d static=%u) "
                  "frames=%u errs=%u tfail=%u raw=%u last=0x%06x | sweep=%u/%u odo=%ld stall=%u lutfull=%u reject=%u",
                  bsp_bb_is_active() ? "bitbang" : "spi2",
                  (int)bsp_bb_best_index(),
                  (unsigned)mi,
                  (unsigned)bsp_bb_hit_count(),
                  (unsigned)bsp_bb_crc_hit_count(),
                  (int)bsp_bb_best_index(),
                  (unsigned)bsp_bb_static_level(),
                  (unsigned)fr, (unsigned)er, (unsigned)tf,
                  (unsigned)bsp_mt6701_get_raw_angle(),
                  (unsigned)bsp_mt6701_get_last_raw(),
                  (unsigned)s_sweep_step, (unsigned)s_sweep_steps,
                  (long)s_sweep_odo, (unsigned)s_sweep_stall,
                  (unsigned)s_lut_full, (unsigned)s_lut_reject);

            if (fr == 0U) {
                enc_no_frame++;
                if (enc_no_frame >= 5U) {
                    enc_no_frame = 0U;
                    LOG_I("ENC",
                          "!! 编码器一帧都没收到: MISO 空闲=%u (正常应为 1)。"
                          "查 J11: PC26=SCLK / PC28=MISO / PC24=CS, 以及编码器 3.3V/GND",
                          (unsigned)mi);
                }
            } else {
                enc_no_frame = 0U;
            }
        }
    }

    diag_set_stage(APP_STAGE_LOOP_VBUS);
    vbus = (float)bsp_adc_get_vbus_mv() * 0.001f;
    diag_set_vbus_mv((uint32_t)(vbus * 1000.0f));

    diag_set_stage(APP_STAGE_LOOP_CURRENT);
    bsp_adc_get_currents_ma(&iu_ma, &iv_ma, &iw_ma);
    ia = (float)iu_ma * 0.001f;
    ib = (float)iv_ma * 0.001f;
    ic = (float)iw_ma * 0.001f;
    diag_set_currents_ma(iu_ma, iv_ma, iw_ma);

    /* ---- 实测控制周期 -> 交给控制器作为积分步长 ----
     * 上报是阻塞式的 (USB 写 + printf), 提高上报频率会拉长主循环。开环电角度
     * 是 elec_angle += elec_speed * dt 积分出来的, 若仍用名义 1ms, 磁场的
     * 实际频率就会随循环负载漂移 (负载重 -> 频率低), 导致转子失步。
     * 这里用 mcycle 实测, 保证磁场频率始终等于指令值。 */
    {
        static uint64_t s_prev_mcycle;
        uint64_t now_cycle = hpm_csr_get_core_mcycle();

        if (s_prev_mcycle != 0U) {
            float dt_meas = (float)(now_cycle - s_prev_mcycle)
                            / (float)hpm_core_clock;

            /* 限幅: 排除首次采样、调试器暂停等异常大值 */
            if ((dt_meas > 0.00005f) && (dt_meas < 0.05f)) {
                foc_controller_set_dt(&s_ctrl, dt_meas);
            }
        }
        s_prev_mcycle = now_cycle;
    }

    /* ---- 控制: 统一控制器 (开环 or 电流环) ---- */
    diag_set_stage(APP_STAGE_LOOP_CONTROL);
    {
        float elec_rad = app_encoder_get_elec_rad(&s_encoder);

#if APP_ENC_ANGLE_DRIVE
        /* ---- 编码器换向 (开环电压幅值) ----
         * 电压矢量的角度 = 编码器电角度 + 超前角。
         * 超前 90° 让矢量落在 q 轴 -> 单位电压出最大转矩;
         * 若直接填 enc_elec, 矢量落在 d 轴, 转矩为 0 (电机只会发热不转)。
         * 幅值仍走开环给定, 所以电压斜坡 + 过流折叠保护照常生效。 */
        {
            float lead = APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f);

            foc_controller_set_external_angle(&s_ctrl, elec_rad + lead);
        }
#endif

#if APP_ENC_LUT_AUTO_SWITCH || APP_ENC_LUT_LEARN || APP_ENC_LUT_USE
        /* ---- 编码器角度查表 (LUT) ----
         * 学习: 把 (编码器原始角, 当前软件电角度) 喂进对应箱做圆平均。
         *       LUT[bin] 逐渐收敛成"该机械位置对应的平均电角度"。
         * 使用: 电角度改从表里取 (相邻箱插值), 表项不足处自动退回线性公式。
         * 注意学习只在【开环旋转】时才有意义。 */
        {
            uint32_t raw = (uint32_t)app_encoder_get_raw_angle(&s_encoder);
            uint32_t sw_mrad =
                (uint32_t)(foc_normalize_angle(s_ctrl.openloop.elec_angle) * 1000.0f);

#if APP_ENC_LUT_AUTO_SWITCH
            if ((s_lut_reject != 0U) && (s_lut_full == 0U)) {
                /* 表被体检否决 (缺口太长 / 整圈不自洽): 保持开环 VF 旋转,
                 * 不换向。拿"编出来"的表换向 = 转子在补出来的那段里反复弹 + 堵转发热。 */
            } else if (s_lut_full == 0U) {
                /* 阶段 1: 学表。此时电压矢量角度仍然由内部积分的模拟电角度决定。 */
#if APP_CAL_SWEEP_ENABLE
                /* 表由【静态扫描】造 (见 app_diag.h / app.h 的说明):
                 * 动态学法会把负载角 δ 积分进表, 累积误差每次都能到 60~100°,
                 * 必然在某个位置卡死。这里把动态喂表彻底关掉。 */
                (void)sw_mrad;
#else
                if (s_ctrl.mode == foc_ctrl_mode_openloop) {
                    diag_lut_feed(raw, sw_mrad);
                }
#endif

                if (diag_lut_valid_bins() >= ENC_LUT_BINS) {
                    uint32_t fixed;
                    uint32_t di;
                    char     dline[256];

                    s_lut_full = 1U;

                    /* 一次性整表转储: 此刻转子还没换向(开环慢转), 阻塞打印无所谓,
                     * 而且转储只发生这一次, 不会像周期转储那样把电机"刹"住。
                     * 先打【学习原始结果】(A), 再打【修复后】(B), 两段对照即可定位
                     * "斜率坏"的箱 —— 也就是换向后会被焊住的那个机械位置。 */
                    LOG_I("ENC", "LUT dump A: learned (before repair)");
                    for (di = 0U; di < ENC_LUT_LINES; di++) {
                        (void)diag_lut_dump_line(dline, sizeof(dline), di);
                        app_log_write_raw(dline);
                    }

                    /* 切换向前先修表: 把"学习时转子被顶住"的坏箱用邻域插值顶掉。
                     * 这些箱斜率可能反向, 不修就是换向后的"磁弹簧"陷阱。 */
                    fixed = diag_lut_repair();
                    {
                        uint32_t h_back = 0U;
                        int32_t  h_neg  = 0;
                        int32_t  h_pos  = 0;
                        int32_t  h_sum  = diag_lut_health(&h_back, &h_neg, &h_pos);

                        LOG_I("ENC",
                              "LUT repair: %u/%u bins interpolated (span>%d mrad)",
                              (unsigned)fixed, (unsigned)ENC_LUT_BINS,
                              (int)ENC_LUT_SPAN_MAX);
                        LOG_I("ENC",
                              "LUT health: sum=%d (want 75396=12rev) back=%u worst=%d..%d",
                              (int)h_sum, (unsigned)h_back, (int)h_neg, (int)h_pos);
                    }

                    LOG_I("ENC", "LUT dump B: repaired");
                    for (di = 0U; di < ENC_LUT_LINES; di++) {
                        (void)diag_lut_dump_line(dline, sizeof(dline), di);
                        app_log_write_raw(dline);
                    }

                    /* 换向后把电压/电流限幅都降下来:
                     *   学表阶段用 0.7V / 4A 保证转子锁得住;
                     *   换向后矢量跟着转子走, 转速由 V/Ke 自限,
                     *   "手能不能停住"由转矩 ≈ Kt×I 决定, 所以电流限幅
                     *   (APP_ENC_LUT_RUN_CURRENT_LIMIT_A) 比电压更直接。
                     * 电压会按 FOC_OPENLOOP_VOLTAGE_SLEW 平滑爬过去。 */
                    foc_controller_set_voltage(&s_ctrl,
                                               APP_ENC_LUT_RUN_VOLTAGE_V);
                    foc_controller_set_current_limit(&s_ctrl,
                                                     APP_ENC_LUT_RUN_CURRENT_LIMIT_A);

                    /* 【存档】表已被接受, 立即写 Flash, 以后上电免校准 */
                    {
                        static int16_t nv_tbl[ENC_LUT_BINS];

                        diag_lut_snapshot(nv_tbl);
                        if (app_nv_save(nv_tbl, s_cal_offset_mrad)) {
                            LOG_I("ENC", "NV: LUT + offset saved to flash (offset=%ld mrad)",
                                  (long)s_cal_offset_mrad);
                        } else {
                            LOG_I("ENC", "NV: save FAILED");
                        }
                    }

                    LOG_I("ENC",
                          "LUT %u/%u full -> LUT commutation ON, lead=%d deg, V=%d mV Ilim=%d mA",
                          (unsigned)ENC_LUT_BINS, (unsigned)ENC_LUT_BINS,
                          (int)APP_ENC_LEAD_DEG,
                          (int)(APP_ENC_LUT_RUN_VOLTAGE_V * 1000.0f),
                          (int)(APP_ENC_LUT_RUN_CURRENT_LIMIT_A * 1000.0f));
                }
            } else {
                /* 阶段 2: 学满, 电角度改用查表值 + 超前角。
                 * 查表值就是"该机械位置对应的磁场电角度", 反过来说就是
                 * 转子电角度 + 平均负载角; 再加超前角即可换向。
                 *
                 * 【必须取回 int32_t】diag_lut_angle 返回的是 (uint32_t)(int32_t):
                 * 表里约一半箱是负角度, 直接 (float)uint32_t 会变成 4.29e9,
                 * 乘 0.001f 后落在 float32 间距 0.5rad 的台阶上 —— 电压矢量
                 * 角度变成每步 500mrad 的乱数, 换向自然抖/卡。
                 * 症状: CSV 第 1 列(sw)只出现间隔 500mrad 的几个值。 */
                uint32_t lut = diag_lut_angle(raw);

                if (lut != 0xFFFFFFFFU) {
                    float lead = APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f);
                    int   dwell = diag_lut_dwell(raw);

                    /* 【反磁弹簧踢一脚】表里只要有一小段误差落在 ~90°, 那里就是
                     * 零转矩假平衡点: 转矩 = Kt·I·cos(误差) ≈ 0, 转子顶在那儿
                     * 一动不动却一直通着电流(发热), 自己出不来。
                     * 静止超过 APP_ENC_STUCK_KICK_MS 就把超前角临时抬到
                     * APP_ENC_KICK_LEAD_DEG: 误差 90° 时转矩从 0 变成 cos(-60°)=0.5,
                     * 转子被拽出去后表就恢复正常工作了。 */
                    if (dwell != 0) {
                        if (s_stuck_ms < 0xFFFFU) { s_stuck_ms += APP_MAIN_LOOP_MS; }
                    } else {
                        s_stuck_ms = 0U;
                    }
                    if (s_stuck_ms >= APP_ENC_STUCK_KICK_MS) {
                        s_stuck_ms = 0U;
                        s_kick_ms  = APP_ENC_KICK_HOLD_MS;
                        LOG_I("ENC", "KICK raw=%u bin=%u lut=%d (顶住>%ums, 超前角临时 %d deg)",
                              (unsigned)raw, (unsigned)(raw / 16U), (int)(int32_t)lut,
                              (unsigned)APP_ENC_STUCK_KICK_MS,
                              (int)APP_ENC_KICK_LEAD_DEG);
                    }
                    if (s_kick_ms > 0U) {
                        s_kick_ms -= APP_MAIN_LOOP_MS;
                        lead = APP_ENC_KICK_LEAD_DEG * (FOC_TWO_PI / 360.0f);
                    }

                    foc_controller_set_external_angle(
                        &s_ctrl, ((float)(int32_t)lut * 0.001f) + lead);

                    /* 【卡顿定位】换向后转子被齿槽/转矩凹陷顶住时, 把位置打出来。
                     * raw 直接对上 LUT dump 的行: bin = raw/16, 例如 raw=11200
                     * -> LUT 行 LUT700 的第 0 个数。 */
                    if (dwell != 0) {
                        s_dwell_ticks++;
                        if (s_dwell_ticks >= APP_ENC_DWELL_LOG_TICKS) {
                            s_dwell_ticks = 0U;
                            LOG_I("ENC", "STALL raw=%u bin=%u lut=%d",
                                  (unsigned)raw, (unsigned)(raw / 16U),
                                  (int)(int32_t)lut);
                        }
                    } else {
                        s_dwell_ticks = 0U;
                    }

                    /* 【转速图】采满一次就把 64 个扇区的最小 4 拍位移打出来:
                     * 数值明显低的扇区就是"卡在哪个 raw"的答案。 */
                    if (diag_spdmap_feed(raw) != 0) {
                        uint32_t si;

                        for (si = 0U; si < ENC_SPDMAP_SECT; si += 16U) {
                            LOG_I("ENC",
                                  "SPDMAP%02u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                                  (unsigned)si,
                                  (unsigned)diag_spdmap_get(si + 0U),
                                  (unsigned)diag_spdmap_get(si + 1U),
                                  (unsigned)diag_spdmap_get(si + 2U),
                                  (unsigned)diag_spdmap_get(si + 3U),
                                  (unsigned)diag_spdmap_get(si + 4U),
                                  (unsigned)diag_spdmap_get(si + 5U),
                                  (unsigned)diag_spdmap_get(si + 6U),
                                  (unsigned)diag_spdmap_get(si + 7U),
                                  (unsigned)diag_spdmap_get(si + 8U),
                                  (unsigned)diag_spdmap_get(si + 9U),
                                  (unsigned)diag_spdmap_get(si + 10U),
                                  (unsigned)diag_spdmap_get(si + 11U),
                                  (unsigned)diag_spdmap_get(si + 12U),
                                  (unsigned)diag_spdmap_get(si + 13U),
                                  (unsigned)diag_spdmap_get(si + 14U),
                                  (unsigned)diag_spdmap_get(si + 15U));
                        }
                    }
                }
            }
#else
#  if APP_ENC_LUT_LEARN
            if (s_ctrl.mode == foc_ctrl_mode_openloop) {
                diag_lut_feed(raw, sw_mrad);
            }
#  endif

#  if APP_ENC_LUT_USE
            {
                uint32_t lut = diag_lut_angle(raw);

                if (lut != 0xFFFFFFFFU) {
                    /* 表里的角度 = 该位置的【磁场】电角度。
                     * 要用它换向, 仍需叠加超前角 (原理同编码器换向)。
                     * lut 是 (uint32_t)(int32_t), 负角度必须先取回 int32_t。 */
                    float lead = APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f);

                    foc_controller_set_external_angle(
                        &s_ctrl, ((float)(int32_t)lut * 0.001f) + lead);
                }
            }
#  endif
#endif
        }
#endif

#if APP_FOC_CLOSED_LOOP_ENABLE
        /* ================= 闭环安全保护 =================
         * 电流环这条路径【本身没有任何保护】(foc_controller_get_trip 在电流环
         * 模式下恒返回 0), 所以这里必须自己补两道:
         *   1) 过流硬保护: 任一相瞬时值超过阈值 -> 立即关断输出并锁死。
         *      没有它的话, PI 参数不合适时电流可能冲到 限幅电压/R ≈ 数十安。
         *   2) 软转速限幅: 闭环只给转矩、没有速度环, 自由转子会一直加速。
         *      超速就把 iq 清零 (纯限速, 不做速度环, 与转向无关, 不会正反馈)。 */
        {
            float imax = app_abs_f(ia);
            float rpm  = app_encoder_get_mech_rpm(&s_encoder);
            float iq   = APP_FOC_IQ_REF_A;

            if (app_abs_f(ib) > imax) { imax = app_abs_f(ib); }
            if (app_abs_f(ic) > imax) { imax = app_abs_f(ic); }

            if ((s_oc_tripped == 0U) && (imax > APP_FOC_OC_LIMIT_A)) {
                s_oc_tripped = 1U;
                foc_controller_enable(&s_ctrl, false);
                bsp_pwm_enable_output(false);
                LOG_I("FOC", "OVERCURRENT %d mA -> output OFF (锁死)",
                      (int)(imax * 1000.0f));
            }

            if (s_oc_tripped != 0U) {
                iq = 0.0f;
            } else if (app_abs_f(rpm) > APP_FOC_RPM_LIMIT) {
                iq = 0.0f;                                   /* 超速: 撤转矩 */
            } else if (app_abs_f(rpm) > (APP_FOC_RPM_LIMIT * 0.9f)) {
                iq *= 0.3f;                                  /* 接近限速: 提前减弱 */
            }

            foc_controller_set_current_ref(&s_ctrl, APP_FOC_ID_REF_A, iq);
        }
#endif

        foc_controller_run(&s_ctrl, ia, ib, ic, elec_rad, vbus);
        foc_controller_get_duty(&s_ctrl, &du, &dv, &dw);
        bsp_pwm_set_duty_uvw(du, dv, dw);

        diag_set_control(foc_controller_get_trip(&s_ctrl),
                         (uint32_t)(s_ctrl.openloop.voltage * 1000.0f),
                         (uint32_t)(s_ctrl.openloop.elec_angle * 1000.0f),
                         (uint32_t)(du * 1000.0f),
                         (uint32_t)(dv * 1000.0f),
                         (uint32_t)(dw * 1000.0f));
    }
    diag_set_stage(APP_STAGE_LOOP_DONE);

    /* ---- 安装偏置校准量 / 校准验证 ----
     * foc.h 的换算关系:  elec = normalize(mech * POLE_PAIRS + offset)
     *
     * 【DIFF 的含义】这里刻意把 FOC_ELEC_ANGLE_OFFSET 也算进去, 所以
     *     DIFF = sw_elec - (mech * POLE_PAIRS + offset)
     *          = 磁场电角度 与 编码器电角度 之差 = 【残差 / 负载角】
     *   - 偏置标定正确 且 转子吸稳对齐 -> DIFF ≈ 0   (校准验证通过)
     *   - 旋转中 -> DIFF 就是负载角, 反映失步/喘振程度
     *   - 若 DIFF 稳定在某个非零常数 -> 那个常数就是还差的偏置, 补进配置即可
     *
     * 之前这里没加 offset, 于是 DIFF 永远是"待标定的原始值"(实测 2061),
     * 偏置写对与否都看不出来 —— 现在可以闭环验证了。 */
    {
        float sw_rad   = foc_normalize_angle(s_ctrl.openloop.elec_angle);
        float mech_rad = app_encoder_get_mech_rad(&s_encoder);
        float enc_elec = foc_mech_to_elec(mech_rad, app_encoder_get_elec_offset_rad());
        float diff_rad = sw_rad - enc_elec;

        while (diff_rad > (FOC_TWO_PI * 0.5f)) {
            diff_rad -= FOC_TWO_PI;
        }
        while (diff_rad < -(FOC_TWO_PI * 0.5f)) {
            diff_rad += FOC_TWO_PI;
        }

        diag_set_cal((uint32_t)(sw_rad * 1000.0f),
                     (uint32_t)app_encoder_get_raw_angle(&s_encoder),
                     (uint32_t)(enc_elec * 1000.0f),
                     (int32_t)(diff_rad * 1000.0f));
    }

#if APP_CAL_SWEEP_ENABLE
    /* ---- 静态角度扫描标定 ----
     * 每步把磁场钉在第 k 步的电角度, 等转子吸稳 (APP_CAL_SWEEP_HOLD 个循环),
     * 取编码器角度平均后打印一行。扫完一遍自动关断输出, 避免持续堵转发热。
     * 静态下没有动态滞后, 所以得到的是真正的磁场电角度 <-> 编码器机械角关系:
     * 斜率均匀 = 编码器正常; 斜率不均 = 编码器有周期性误差。 */
    if (s_sweep_done == 0U) {
        if (s_sweep_pre < APP_CAL_SWEEP_PRESETTLE) {
            /* 预吸稳: 磁场先钉在 0°, 等转子从上电位置被拽过来的摆动彻底停住。
             * 这期间【不采样】—— 否则摆动过程会被当成"磁场在动、转子不动",
             * 把开头那几十个箱填成平表, 换向后正好停在里面出不来。 */
            s_sweep_pre++;
            foc_controller_set_align_angle(&s_ctrl, 0.0f);
            foc_controller_set_voltage(&s_ctrl, APP_CAL_SWEEP_HIGH_V);  /* 预吸稳里降到标定电压 */
        } else {
        {
            uint32_t raw_now = (uint32_t)app_encoder_get_raw_angle(&s_encoder);

            if (s_sweep_cnt == APP_CAL_SWEEP_DROP) {      /* 暂态结束 -> 取参考 */
                s_sweep_ref = raw_now;
                s_sweep_a1 = 0;
                s_sweep_a2 = 0;
            }
            if (s_sweep_cnt >= APP_CAL_SWEEP_DROP) {
                int32_t d = cal_wrap_raw((int32_t)raw_now - (int32_t)s_sweep_ref);

                if (s_sweep_cnt < APP_CAL_SWEEP_MID) {
                    s_sweep_a1 += d;                      /* 前半段: 正常电压 */
                } else {
                    s_sweep_a2 += d;                      /* 后半段: 弱场 */
                }
            }
            if (s_sweep_cnt == APP_CAL_SWEEP_MID) {
                foc_controller_set_voltage(&s_ctrl, APP_CAL_SWEEP_LOW_V);
            }
        }
        s_sweep_cnt++;

        if (s_sweep_cnt >= APP_CAL_SWEEP_HOLD) {
            char line[128];
            int32_t resid = g_cal.diff_mrad;      /* 已含当前 offset, 所以是残差 */

            /* 解卷绕平均: 丢掉前 1/3 暂态, 余下相对参考累加, 可跨 16384->0 */
            /* E1=正常电压段平均, E2=弱场段平均; dE 直接反映"转子是否真停在磁场角度上" */
            int32_t e1 = (int32_t)s_sweep_ref +
                         s_sweep_a1 / (int32_t)(APP_CAL_SWEEP_MID - APP_CAL_SWEEP_DROP);
            int32_t e2 = (int32_t)s_sweep_ref +
                         s_sweep_a2 / (int32_t)(APP_CAL_SWEEP_HOLD - APP_CAL_SWEEP_MID);
            uint32_t raw_avg =
                (uint32_t)(((int32_t)s_sweep_ref +
                            (s_sweep_a1 + s_sweep_a2) /
                            (int32_t)(APP_CAL_SWEEP_HOLD - APP_CAL_SWEEP_DROP)) & 0x3FFF);

            /* 【整圈里程计】净行程 = 各步平均位置的解卷绕累加。
             * 它是扫描的停止依据: 净行程到 16384 就说明编码器走过整整一圈,
             * 每个 bin 都被真实测量过 —— 表不再有"编出来"的段落。 */
            if (s_sweep_step == 0U) {
                s_sweep_odo_prev = raw_avg;
                s_sweep_odo      = 0;
                s_sweep_dmax     = 0;
                s_sweep_dmin     = 0;
            } else {
                int32_t dd = cal_wrap_raw((int32_t)raw_avg -
                                          (int32_t)s_sweep_odo_prev);

                s_sweep_odo += dd;
                s_sweep_odo_prev = raw_avg;
                if (dd > s_sweep_dmax) { s_sweep_dmax = dd; }
                if (dd < s_sweep_dmin) { s_sweep_dmin = dd; }
                /* 一步几乎没动 -> 转子被齿槽/失步顶住了。连着 APP_CAL_SWEEP_STALL_MAX
                 * 步不动就提前收尾, 不能拿 1.5V 一直顶着烧。 */
                if ((dd < 2) && (dd > -2)) { s_sweep_stall++; }
                else                       { s_sweep_stall = 0U; }
            }
            /* 每走完一个电周期, 打印这一周期转子实际推进了多少计数。
             * 理想值恒为 16384/极对数 = 1365。
             * 若在整圈里忽大忽小(0.5x~2.4x) -> 编码器刻度不均匀(磁环偏心/倾斜);
             * 换过磁环后应当落在 1365±130 之内。 */
            if ((s_sweep_step % APP_CAL_SWEEP_PHASES) == 0U) {
                if (s_sweep_step > 0U) {
                    (void)snprintf(line, sizeof(line),
                                   "CALCYC %2u ADV=%ld (ideal %ld)",
                                   (unsigned)(s_sweep_step / APP_CAL_SWEEP_PHASES),
                                   (long)cal_wrap_raw((int32_t)raw_avg - (int32_t)s_cyc_ref),
                                   (long)(16384U / (uint32_t)FOC_MOTOR_POLE_PAIRS));
                    app_log_write_raw(line);
                }
                s_cyc_ref = raw_avg;
            }

            /* 磁场每步走 6283/PHASES mrad; 与下面的 align 公式严格一致 */
            uint32_t sw_mrad = (s_sweep_step * 6283U / APP_CAL_SWEEP_PHASES) % 6283U;

            /* ---- 把这个静态点记进 LUT (这是本模式存在的意义) ----
             * 【按 1/I 外推掉齿槽偏移】静平衡时 Kt·I·sinδ = 齿槽/摩擦转矩,
             * 所以 δ ∝ 1/I: 弱场段(0.30V, 1/5 电流)的偏移是强场段的 5 倍。
             * 两段位移差 dE = 4·δ_强, 减掉 dE/4 就得到"无限电流"下真正的
             * 对齐点 —— 这样标出来的表连静态齿槽偏移都没有。
             * |dE| 过大 = 弱场下转子已经跟不住磁场(齿槽太强), 这时不修正,
             * 只统计出来给人看。 */
            {
                int32_t d_e = cal_wrap_raw(e2 - e1);
                int32_t raw_use;
                int32_t vh  = (int32_t)(APP_CAL_SWEEP_HIGH_V * 1000.0f);
                int32_t vl  = (int32_t)(APP_CAL_SWEEP_LOW_V * 1000.0f);

                /* 修正量 = dE × Vl/(Vh - Vl) —— 由电压比算出来, 不再写死 /4;
                 * 改了电压而忘了改系数, 外推就会多补/少补 (方向还是对的, 量错)。 */
                if ((d_e >= -12) && (d_e <= 12)) {
                    raw_use = e1 - (int32_t)(((int32_t)d_e * vl) / (vh - vl));
                } else {
                    raw_use = e1;
                    s_sweep_de_big++;
                }
                diag_lut_feed_static((uint32_t)(raw_use & 0x3FFF), sw_mrad);

                if (s_sweep_de_n == 0U) {
                    s_sweep_de_min = d_e;
                    s_sweep_de_max = d_e;
                }
                if (d_e < s_sweep_de_min) { s_sweep_de_min = d_e; }
                if (d_e > s_sweep_de_max) { s_sweep_de_max = d_e; }
                s_sweep_de_acc += d_e;
                s_sweep_de_n++;

                if ((s_sweep_step % 4U) == 0U) {
                    (void)snprintf(line, sizeof(line),
                                   "CALSTEP %4u SW=%u ENC=%u E1=%ld E2=%ld dE=%ld DIFF=%ld",
                                   (unsigned)s_sweep_step,
                                   (unsigned)sw_mrad,
                                   (unsigned)raw_avg,
                                   (long)(e1 & 0x3FFF), (long)(e2 & 0x3FFF),
                                   (long)d_e,
                                   (long)resid);
                    app_log_write_raw(line);
                }
            }

            /* 残差汇总: 相对第一个样本解卷绕后累加, 避免 ±pi 附近出错 */
            if (s_sweep_res_n == 0U) {
                s_sweep_res_ref = resid;
                s_sweep_res_min = resid;
                s_sweep_res_max = resid;
            }
            s_sweep_res_acc += cal_wrap_mrad(resid - s_sweep_res_ref);
            s_sweep_res_n++;
            if (resid < s_sweep_res_min) { s_sweep_res_min = resid; }
            if (resid > s_sweep_res_max) { s_sweep_res_max = resid; }

            s_sweep_a1   = 0;
            s_sweep_a2   = 0;
            s_sweep_cnt  = 0U;
            s_sweep_step++;
            foc_controller_set_voltage(&s_ctrl, APP_CAL_SWEEP_HIGH_V);  /* 恢复标定电压(低压) */

            if ((s_sweep_step >= s_sweep_steps) || (s_sweep_odo >= 16384) ||
                (s_sweep_stall >= APP_CAL_SWEEP_STALL_MAX)) {
                /* ---- 自动给出结论 ----
                 * 残差均值 = 当前 offset 还差多少。推荐偏置 = 当前 + 残差均值。
                 * 若推荐值与当前值几乎相同 -> 校准已被验证通过。 */
                int32_t mean_res = s_sweep_res_ref +
                                   s_sweep_res_acc / (int32_t)s_sweep_res_n;
                int32_t rec = (int32_t)(app_encoder_get_elec_offset_rad() * 1000.0f) + mean_res;

                while (rec < 0)     { rec += 6283; }
                while (rec >= 6283) { rec -= 6283; }

                s_cal_offset_mrad = rec;   /* 存档用 */
                s_sweep_done = 1U;

                /* 【扫完自动恢复旋转, 不再关断输出】
                 * 这里原来写的是 foc_controller_enable(&s_ctrl, false),
                 * 目的是防止"堵转+顶限幅"持续发热。但副作用是【每次标定完
                 * 电机都停着不动】, 极易被误判成"校准把电机搞坏了"。
                 * 扫描全程只有 STEPS × HOLD ≈ 3.6 秒, 堵转时间很短,
                 * 直接恢复 20 RPM 旋转既安全又不产生困惑。 */
                foc_controller_set_speed_rpm(&s_ctrl, APP_OPENLOOP_TARGET_RPM);

                (void)snprintf(line, sizeof(line),
                    "CAL RESULT n=%u residual_mean=%ld spread=%ld..%ld",
                    (unsigned)s_sweep_res_n, (long)mean_res,
                    (long)s_sweep_res_min, (long)s_sweep_res_max);
                app_log_write_raw(line);

                /* 【齿槽偏移体检】dE = 强场段与弱场段的编码器位置差 (counts)。
                 * 静平衡: Kt·I·sinδ = 齿槽/摩擦转矩 => δ ∝ 1/I, 所以
                 *   dE = 4·δ_强(1.5V/0.3V 电流比 5:1)。
                 * dE 只有 ±1~2 counts -> 齿槽很弱, 静态标定非常干净;
                 * dE 到 ±5 counts 以上 -> 齿槽已经明显把转子拉偏, 换向误差里
                 *   有这部分"负载角", 它随位置变化, 就是残余的停顿来源;
                 * big = 弱场下转子跟不住(|dE|>12)的步数, 这些步没有做外推修正。 */
                (void)snprintf(line, sizeof(line),
                    "CAL DEOFF n=%u mean=%ld min=%ld max=%ld big=%u",
                    (unsigned)s_sweep_de_n,
                    (long)((s_sweep_de_n != 0U) ? (s_sweep_de_acc / (int32_t)s_sweep_de_n) : 0),
                    (long)s_sweep_de_min, (long)s_sweep_de_max,
                    (unsigned)s_sweep_de_big);
                app_log_write_raw(line);

                /* 【整圈收尾汇报】走了多少步 = 磁场转了几个电周期。转子吸稳时
                 * "磁场转 P 个电周期 = 编码器走一整圈", 所以 fieldrev 就是这台
                 * 电机【极对数的实测值】(配 12 而实际只走了 11.x 圈 -> 配置偏小
                 * 或中途丢步)。odo 目标 16384 = 一整圈; dmax/dmin 是单步行程的
                 * 极值, 出现大负数说明转子倒退/失步过。 */
                {
                    uint32_t steps = (s_sweep_step != 0U) ? s_sweep_step : 1U;
                    uint32_t pairs = (steps + (APP_CAL_SWEEP_PHASES / 2U))
                                     / APP_CAL_SWEEP_PHASES;

                    s_sweep_pairs = pairs;
                    (void)snprintf(line, sizeof(line),
                        "CAL SWEEP end: steps=%u fieldrev=%u odo=%ld dmax=%ld dmin=%ld",
                        (unsigned)s_sweep_step, (unsigned)pairs,
                        (long)s_sweep_odo, (long)s_sweep_dmax, (long)s_sweep_dmin);
                    app_log_write_raw(line);
                }

                if (s_sweep_stall >= APP_CAL_SWEEP_STALL_MAX) {
                    /* 转子顶着不动: 这次扫出来的表右半边是空的, 直接否决换向 */
                    s_lut_reject = 1U;
                    (void)snprintf(line, sizeof(line),
                        "CAL SWEEP ABORT: 转子连续 %u 步不动 (raw~%u) -> 不换向",
                        (unsigned)s_sweep_stall,
                        (unsigned)app_encoder_get_raw_angle(&s_encoder));
                    app_log_write_raw(line);
                }

                /* 【补空缺 + 体检】走满整圈后, 缺口应只剩"刻度太疏跳过 1~2 箱",
                 * 用左右精确点弦插值补上 (邻居是静态精确点, 误差极小)。补完
                 * bins=1024, 主循环里"表学满 -> 换向"那段既有逻辑会自动接手。
                 * 【否决条件】缺口一旦长于 1/4 电周期(16 箱), 弦插值的 6283 分支
                 * 就判不出来 (端点的短支 + 名义斜率给不出真值: 实测 46 箱的缺口
                 * 被补成多整整一个电周期的斜坡, 换向后转子正好在那段里反复弹),
                 * 此时【不换向】, 保持开环 VF, 而不是拿假表把转子焊住。 */
                {
                    uint32_t missed  = ENC_LUT_BINS - diag_lut_valid_bins();
                    uint32_t maxhole = diag_lut_max_hole();
                    uint32_t fixed   = diag_lut_repair();
                    uint32_t hb = 0U;
                    int32_t  hn = 0;
                    int32_t  hp = 0;
                    int32_t  hs = diag_lut_health(&hb, &hn, &hp);
                    int32_t  want = (int32_t)s_sweep_pairs * 6283;

                    (void)snprintf(line, sizeof(line),
                        "CAL LUT: missed=%u repaired=%u valid=%u maxhole=%u",
                        (unsigned)missed, (unsigned)fixed,
                        (unsigned)diag_lut_valid_bins(), (unsigned)maxhole);
                    app_log_write_raw(line);

                    /* sum 应 = 极对数 × 6283 (整圈恰好那么多电周期), back 越小越好 */
                    (void)snprintf(line, sizeof(line),
                        "CAL LUT health: sum=%ld (want %ld = %u rev) back=%u worst=%ld..%ld",
                        (long)hs, (long)want, (unsigned)s_sweep_pairs,
                        (unsigned)hb, (long)hn, (long)hp);
                    app_log_write_raw(line);

                    if ((maxhole > 16U) ||
                        ((hs - want) > 6283) || ((hs - want) < -6283)) {
                        s_lut_reject = 1U;
                        (void)snprintf(line, sizeof(line),
                            "CAL LUT REJECTED: maxhole=%u gap_sum=%ld -> 不换向, 保持开环 VF",
                            (unsigned)maxhole, (long)(hs - want));
                        app_log_write_raw(line);
                    }
                }

                /* 不用 %f (工程刻意避开浮点打印), 拆成整数+小数 */
                (void)snprintf(line, sizeof(line),
                    "CAL -> 写进 foc_config.h:  #define FOC_ELEC_ANGLE_OFFSET (%ld.%03ldf)",
                    (long)(rec / 1000), (long)(rec % 1000));
                app_log_write_raw(line);

                app_log_write_raw("CALSWEEP done -> 已恢复旋转 (20 RPM)");
            } else {
                /* 磁场绝对累加: 每步 2π/PHASES 电角度, 整圈正好走完 极对数 个电周期 */
                foc_controller_set_align_angle(&s_ctrl,
                    (float)s_sweep_step * FOC_TWO_PI / (float)APP_CAL_SWEEP_PHASES);
            }
        }
        }   /* else: 预吸稳结束, 开始记表 */
    }
#endif

    /* ---- 周期性上报 ---- */
    if (++tick >= APP_REPORT_TICKS) {
        char msg[256];

        tick = 0;

#if APP_TELEMETRY_PAIR_ONLY
        /* ---- 极简 CSV: 三个整数 ----
         *   1) 软件(模拟)电角度      mrad, 0..6283
         *   2) 编码器【电角度】       mrad, 0..6283 (含已标定的偏置)
         *   3) 编码器【原始 14bit】   0..16383   <-- 判断编码器本身是否正常
         * 转子匀速转时, 第 3 列应当单调、均匀地推进, 一圈正好走完 0..16383。
         * 若它出现回退/跳变/长时间不动, 就是编码器或磁环的问题。 */
        {
            int32_t sw = (int32_t)(foc_normalize_angle(s_ctrl.openloop.elec_angle)
                                   * 1000.0f);
            int32_t enc_elec =
                (int32_t)(app_encoder_get_elec_rad(&s_encoder) * 1000.0f);
            int32_t enc_raw =
                (int32_t)app_encoder_get_raw_angle(&s_encoder);

#if APP_TELEMETRY_LUT
            /* 第 4 列: 由编码器 raw 查表得到的电角度。
             * 表项样本不足时 diag_lut_angle 返回 0xFFFFFFFF, 这里折算成 -1,
             * 方便串口侧直接看出"这个位置还没学到". */
            {
                uint32_t lut = diag_lut_angle((uint32_t)enc_raw);
                int32_t  lut_mrad;

                if (lut == 0xFFFFFFFFU) {
                    lut_mrad = -1;          /* 表项样本不足 */
                } else {
                    lut_mrad = (int32_t)lut;
                    if (lut_mrad < 0) {
                        lut_mrad += 6283;   /* 统一到 0..6283, 便于和 sw 对照 */
                    }
                }

                (void)diag_csv_telemetry_lut(msg, sizeof(msg), sw, enc_elec,
                                             enc_raw, lut_mrad);
            }
#else
            (void)diag_csv_telemetry(msg, sizeof(msg), sw, enc_elec, enc_raw);
#endif
        }
#elif APP_TELEMETRY_CAL_ONLY
        /* 校准模式: 只输出 SW / ENC / ENCEL / DIFF */
        (void)diag_cal_telemetry(msg, sizeof(msg));
#else
        (void)diag_telemetry(msg, sizeof(msg),
                             s_ctrl.openloop.elec_speed / FOC_TWO_PI,
                             app_encoder_get_mech_rpm(&s_encoder),
                             bsp_adc_get_vbus_mv());
#endif

        /* 遥测原样输出, 复用日志后端 (console + USB) */
        app_log_write_raw(msg);
    }

    /* ---- 校准映射表转储 ----
     * 串口上报 (5Hz) 与转速 (0.333圈/s) 恰好成 15:1, 每次上报都落在同一批
     * 机械位置 —— 只看上面那行日志是【看不到曲线形状】的。这张表由 1kHz 的
     * 控制循环累积, 一次旋转就能把整圈 64 箱填满, 每 5 秒打印一次。 */
    if ((APP_CAL_MAP_DUMP_TICKS > 0U) && (++s_map_tick >= APP_CAL_MAP_DUMP_TICKS)) {
        char line[192];
        uint32_t i;

        s_map_tick = 0;

        /* 只在所有箱都收到过样本后才打印, 避免输出半张表 */
        for (i = 0U; i < CAL_MAP_BINS; i++) {
            if (diag_cal_map_count(i) == 0U) {
                break;
            }
        }
        if (i >= CAL_MAP_BINS) {
            for (i = 0U; i < CAL_MAP_LINES; i++) {
                (void)diag_cal_map_line(line, sizeof(line), i);
                app_log_write_raw(line);
            }
        }
    }

    /* ---- 编码器角度查表 (LUT) 整表转储 ----
     * APP_ENC_LUT_DUMP_TICKS = 每 N 次上报打印一次整张表; 0 = 关闭。
     * 64 行会阻塞控制环, 和校准映射表一样, 看完就关。
     * 每行: "LUTxxx v0 v1 ... v15", 单位 mrad, -1 表示该箱样本不足。 */
#if (APP_ENC_LUT_DUMP_TICKS > 0U)
    {
        static uint32_t s_lut_tick;

        if (++s_lut_tick >= APP_ENC_LUT_DUMP_TICKS) {
            char line[256];
            uint32_t valid = diag_lut_valid_bins();
            uint32_t i;

            s_lut_tick = 0;

            (void)snprintf(line, sizeof(line),
                           "LUTBINS %u/%u err=%u tfail=%u lastraw=0x%06lx",
                           (unsigned)valid, (unsigned)ENC_LUT_BINS,
                           (unsigned)bsp_mt6701_get_error_count(),
                           (unsigned)bsp_mt6701_get_transfer_fail(),
                           (unsigned long)bsp_mt6701_get_last_raw());
            app_log_write_raw(line);

            if (valid >= ENC_LUT_BINS) {
                for (i = 0U; i < ENC_LUT_LINES; i++) {
                    (void)diag_lut_dump_line(line, sizeof(line), i);
                    app_log_write_raw(line);
                }
            }
        }
    }
#endif
}
