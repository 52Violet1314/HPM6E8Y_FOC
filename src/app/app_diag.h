/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 诊断状态模块
 *
 *   把调试器可观察的运行状态收敛到一个模块。各功能模块通过 diag_set_xxx()
 *   更新状态, 而不是直接改写 g_diag 的字段; 上报格式由 diag_telemetry()
 *   统一生成, 避免格式化逻辑散落在 app.c 里。
 *
 *   g_diag 供 J-Link/Cortex-Debug 的 WATCH 窗口直接观察: 卡在哪一步、VBUS、
 *   占空比等一目了然, 即使没有串口也能定位问题。
 */
#ifndef APP_DIAG_H
#define APP_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 运行阶段 (g_diag.stage, 调试器据此判断"卡在哪一步") */
#define APP_STAGE_BOOT              (1U)
#define APP_STAGE_PWM_INIT          (2U)
#define APP_STAGE_ADC_INIT          (3U)
#define APP_STAGE_USB_INIT          (4U)
#define APP_STAGE_SEI_INIT          (5U)
#define APP_STAGE_FOC_INIT          (6U)
#define APP_STAGE_PWM_ENABLE        (8U)
#define APP_STAGE_LOOP_TOP          (10U)
#define APP_STAGE_LOOP_VBUS         (11U)
#define APP_STAGE_LOOP_CURRENT      (12U)
#define APP_STAGE_LOOP_CONTROL      (13U)
#define APP_STAGE_LOOP_DONE         (14U)
/* 主循环里的"耗时嫌疑段": 标出来是为了让 ADC 中断在被拖长时能记录
 * "当时主循环正在干什么"(见 app.c 的 s_isr_gap_stage), 用来定位是谁堵的中断。 */
#define APP_STAGE_LOOP_ENCODER      (15U)   /* bit-bang 读 MT6701 */
#define APP_STAGE_LOOP_LCD          (16U)   /* LVGL 任务 + 触摸 + SPI 刷屏 */
#define APP_STAGE_LOOP_LOG          (17U)   /* 日志格式化/入队 */
#define APP_STAGE_LOOP_USB          (18U)   /* USB CDC 提交 */

typedef struct {
    uint32_t stage;
    uint32_t loop_cnt;
    uint32_t vbus_mv;
    int32_t  iu_ma;
    int32_t  iv_ma;
    int32_t  iw_ma;
    uint32_t enc_angle;
    uint32_t enc_frames;
    uint32_t enc_errors;
    uint32_t enc_raw;            /* 最近一次读到的编码器原始 24bit */
    uint32_t enc_transfer_fail;  /* SPI 传输失败/超时次数 */
    uint32_t trip;
    uint32_t ol_voltage_mv;
    uint32_t ol_elec_angle_mrad;
    uint32_t duty_u_m;
    uint32_t duty_v_m;
    uint32_t duty_w_m;
    /* bit-bang 扫描结果 (编码器接线排查用, 见 bsp_spi_bb.h) */
    uint32_t bb_hits;        /* 读到"变化数据"的组合数 (满 16)   */
    uint32_t bb_crc_hits;    /* 其中 CRC6 有帧通过的组合数       */
    int32_t  bb_best;        /* 最佳组合下标, -1 = 一个都没有     */
    uint32_t bb_raw;         /* 最佳组合的代表 raw               */
    uint32_t miso_static;    /* 不打时钟时 MISO 静态电平 (0/1)   */
} app_diag_t;

extern volatile app_diag_t g_diag;     /* 调试器直接 watch 这个变量 */

/* ---- 校准状态 (开环同步旋转时整定 FOC_ELEC_ANGLE_OFFSET 用) ----
 *
 * 开环驱动时转子被电压矢量"拖着"同步旋转, 于是:
 *     软件电角度 (sw_elec)  ≈  编码器换算出的电角度 (enc_elec) + 安装偏置
 * 这个"安装偏置"就是需要写进 foc_config.h 的 FOC_ELEC_ANGLE_OFFSET。
 * 判据: 电机稳定同步旋转时 DIFF 应当几乎不变; 变动说明不同步或极对数不对。 */
typedef struct {
    uint32_t sw_elec_mrad;    /* 软件(开环)电角度, mrad, 0..6283 */
    uint32_t enc_raw;         /* 编码器原始角度, 14bit, 0..16383 */
    uint32_t enc_elec_mrad;   /* 编码器换算的电角度, mrad, 不含偏置 */
    int32_t  diff_mrad;       /* sw_elec - enc_elec, 折算到 ±3141 mrad */
} app_cal_t;

extern volatile app_cal_t g_cal;       /* 调试器直接 watch */

/* ---- 校准映射表 (DIFF 随编码器角度的曲线) ----
 *
 * 串口上报 5Hz 与转速 0.333 圈/s 恰好成 15:1, 每次上报都落在同一批机械位置,
 * 所以【日志里看不到曲线形状】。固件按 1kHz 把一圈分成 64 箱累积 DIFF 均值,
 * 一次旋转即可填满整条曲线 —— 用来判别:
 *   - 锯齿状 (每圈整数量个齿)  -> 极对数不匹配;
 *   - 光滑曲线 (有界)          -> 负载角随位置变化 (转矩/同步问题)。 */
#define CAL_MAP_BINS       (64U)
#define CAL_MAP_PER_LINE   (8U)   /* 每行打印 8 个箱 */
#define CAL_MAP_LINES      (CAL_MAP_BINS / CAL_MAP_PER_LINE)

/**
 * @brief  格式化映射表的一行
 * @param  line  0 .. CAL_MAP_LINES-1
 * @return 写入长度 (不含 '\0')
 */
int diag_cal_map_line(char *buf, uint32_t size, uint32_t line);

/** @return 指定箱累积到的样本数 (0 表示该箱还没数据) */
uint32_t diag_cal_map_count(uint32_t bin);

/* ================= 编码器角度查表 (LUT) =================
 *
 * 用途: 把「编码器原始角 -> 电角度」从线性公式改成查表, 用来补偿编码器/磁环
 *       真实存在的【位置相关谐波误差】。
 *
 * 【先看清楚它能修什么、不能修什么】
 *   实测: 编码器平均刻度是对的 (转子机械转速 21.1 RPM vs 同步 21.3 RPM,
 *   比值 0.99)。所以查表不是为了修"刻度"。
 *   能修: 编码器在整圈内某个位置偏了几度这样的【确定性谐波】。
 *   修不了: 转子围绕同步位置的【摆动】(负载角摆动 ±250° 电角度, 会让瞬时
 *   转速波动 ±75%)。逐箱圆平均能把摆动平均掉, 但残留会留在表里。
 *
 * 【建表条件】学习期间电机要在开环下稳定旋转。每箱累积 sw 的圆均值:
 *      LUT[bin(raw)] ≈ 该机械位置对应的平均电角度
 *   有效前提是该箱样本数 >= ENC_LUT_MIN_CNT。 */
/* 【表密度】1024 箱 -> 16384/1024 = 16 counts/箱, 正好整除。
 * 之前用 800 箱时 16384/800 = 20.48 不是整数, 查表插值按 20 算, 箱号越大
 * 偏得越多 (第 799 箱能偏 300+ counts ≈ 1.7 rad), 表现为"越到后面差越大"。
 * 改成 1024 后箱宽是整数, 从根上消掉这个累积误差; 分辨率也从 20.5 提到 16。 */
#define ENC_LUT_BINS        (1024U)  /* 一圈 1024 份 (16 counts/份, 精确整除) */
#define ENC_LUT_PER_LINE    (16U)    /* 打印时每行的个数 */
#define ENC_LUT_LINES       (ENC_LUT_BINS / ENC_LUT_PER_LINE)  /* = 64 行 */
/* 【为什么从 16 降到 3】
 *   一圈 16384 计数、4RPM 下一圈约 1500 个采样点 -> 正常情况下每箱每圈
 *   只有 16 x 1500 / 16384 ≈ 1.47 个样本。而本机编码器刻度很不均匀
 *   (局部增益 0.54~2.34 倍), 在增益 2.34 那一段每箱每圈只剩 0.63 个样本
 *   -> 要 25 圈(6 分钟)才够 16 个, 实测表现为 LUTBINS 停在 966/1024 不动。
 *   16 -> 3: 高增益段 5 圈(约 75s)即可填满。
 *   注: 每箱每圈样本数与转速无关(转慢则停留时间变长、采样数同比变多),
 *   所以只能降门槛, 不能靠降转速解决。 */
/* 【3 -> 1】静态扫描造表时, 2400 步摊到 1024 箱, 每箱大约 1~5 个样本;
 *   但静态样本本身就是"吸稳后 30 拍的平均", 单点精度 < 1 count, 所以 1 个就够。
 *   动态学法才需要 3 个以上(那时同一箱的样本来自不同圈, 抖动要靠平均压掉)。
 *   没被踩到的箱(cnt=0) 仍然会被 diag_lut_repair 用邻居插值补上。 */
#define ENC_LUT_MIN_CNT     (3U)     /* 每箱最少样本数; 开环边转边学时靠它滤掉个别坏点 */

/* 【运动门控】只有"转子确实在动"的样本才允许进表。
 *   开环学表时若转子被齿槽/静摩擦顶住不动, 而磁场仍按 4RPM 在扫, 那一小段
 *   机械位置就会收到一堆"磁场在扫、转子不动"的样本 —— 圆均值把它们平成一个
 *   中间值, 该处【斜率符号都可能反掉】。换向后就表现为"磁弹簧"陷阱:
 *   转子停在那里, 手拨开又弹回去 (换向失效)。
 *   判据: 最近 ENC_LUT_MOVE_WIN 个控制周期内 raw 的位移 >= ENC_LUT_MOVE_CNT。
 *   4RPM = 16384 counts/15s = 1093 counts/s, 16ms 约 17 counts (理想刻度下与
 *   增益无关: 增益只改"每 count 多少电角度")。
 *   【4 -> 10】原来的 4 counts 太松: 换到 1.5V 学表后, 转子在齿槽/磁弹簧处
 *   原地来回抖就能凑够 4 counts, 门控形同虚设, 于是"磁场在扫、转子没动"的
 *   样本混进那一小段机械位置 —— 实测坏箱从 2 个暴涨到 40 个, 换向后卡死。
 *   10 counts/16ms ≈ 625 counts/s ≈ 2.3RPM, 抖动过不去, 正常 4RPM 仍有 1.7 倍余量。 */
#define ENC_LUT_MOVE_WIN    (16U)
#define ENC_LUT_MOVE_CNT    (10)
/* 【稳态门】学表只在"转子稳定跟随磁场"时采样: 用两个窗口的净行程比判断。
 *   卡住(短窗≈0) / 追赶(短窗远大于长窗) 都不采 —— 这两种时刻转子都不在
 *   磁场角对应的位置上, 采进去就是把滞后平均进表里。实测: 开环强推学法在
 *   raw 1024..1535 那一段, 每格(16 counts)里磁场角的圆跨度中位数就有 1291 mrad
 *   (74°), 平均后表在那里偏了 37°, 换向后转子正好在那顿/卡。
 *   改用弦插值补这些格 (邻居是稳态精确点), 比采一个偏 37° 的平均值好得多。 */
#define ENC_LUT_RATE_WIN    (256U)   /* 长窗拍数 */
#define ENC_LUT_RATE_SHORT  (64U)    /* 短窗拍数 */
#define ENC_LUT_RATE_MIN    (64U)    /* 长窗净行程下限 (counts): 保证确实在转 */

/* 【起步预热】上电时转子静止、磁场已经在按 4RPM 扫 —— 拉入同步的过程里转子
 *   只会在原地来回晃, 而运动门控只看"动没动", 于是这一小段机械位置的样本被
 *   污染得很厉害 (圆均值变成一个没意义的中间值)。
 *   实测两次: 污染区都紧挨着上电瞬间的 raw (一次 raw≈11200, 一次 raw≈11550),
 *   而真正转起来以后同一张表在别处是干净的 —— 所以不是编码器问题, 是起步暂态。
 *   判据: 净行程里程计 (只累加回绕校正后的有符号位移), 转子必须真正净转过
 *   ENC_LUT_WARMUP_COUNTS 个计数才开始收样本。原地晃动的净行程≈0, 过不了。 */
#define ENC_LUT_WARMUP_COUNTS (16384U)   /* 一整圈机械角 */

/* 【箱内极差】一箱样本的 max-min。正常学习时同箱样本来自不同圈、负载角基本
 *   一致, 极差只有几十 mrad; 掺进"转子没动"的样本后极差会到几 rad。
 *   diag_lut_repair() 用它挑出坏箱, 用左右最近好箱线性插值补掉。 */
#define ENC_LUT_SPAN_MAX    (800)

/* 【每箱平均前进量】整圈机械角 = 12 个电周期 = 12*6283 mrad, 1024 箱,
 *   所以平均每箱 73.6 mrad, 这是与编码器刻度无关的硬约束。
 *   修表时用它决定"回绕差值取哪一支"(见 diag_lut_repair)。 */
#define ENC_LUT_STEP_NOMINAL (73)

/**
 * @brief  喂一个样本 (每个控制周期调用一次, 仅在开环旋转时)
 * @param  enc_raw   编码器原始角 (14bit)
 * @param  sw_mrad   该时刻的软件电角度 (mrad)
 */
void diag_lut_feed(uint32_t enc_raw, uint32_t sw_mrad);

/**
 * @brief  静态标定专用: 把一个【已吸稳】的样本记进表。
 *         与 diag_lut_feed 的区别是【不走运动门控/预热里程计】—— 静态时转子
 *         本来就不动, 那两个门会把它全部丢掉。磁场角是精确已知的步进值,
 *         编码器角是吸稳后的平均值, 所以单点就是映射上的一个精确点。
 * @param  enc_raw   编码器原始角 (14bit)
 * @param  sw_mrad   该步钉住的磁场电角度 (mrad)
 */
void diag_lut_feed_static(uint32_t enc_raw, uint32_t sw_mrad);

/**
 * @brief  设置"多少样本算有效箱"的门限 (运行时, 默认 ENC_LUT_MIN_CNT)。
 *         静态扫描每箱只有一个已吸稳+外推的样本, 用 3 会把 2/3 的箱判成
 *         "从未学到"然后整体拒表 (实测 missed=664 maxhole=43 -> 被拒)。
 *         静态路径在启动扫描前把它降到 1; 开环边转边学仍保持默认 3。
 */
void diag_lut_set_min_cnt(uint32_t n);

/** @return 已经有效的箱数 (>= 当前 min_cnt 个样本) */
uint32_t diag_lut_valid_bins(void);

/**
 * @brief  修表: 把"箱内极差过大"(学习时转子没跟上)和"从未学到"的箱,
 *         用左右最近的好箱线性插值补上。表学满后、切换向之前调用一次。
 * @return 被替换掉的箱数 (打印出来看表有多脏)
 * @note   这类箱不是"角度偏几十度"那么简单: 它们的斜率可能反向, 换向后就是
 *         一个"磁弹簧"陷阱(转子停死、拨开弹回)。不修的话换向必然卡在某个位置。
 */
uint32_t diag_lut_repair(void);

/**
 * @brief  最长"坏箱"连续段 (环形, 不修表)。
 *         坏箱 = 样本不足, 或箱内极差过大。走满整圈后缺口应只剩 1~2 箱;
 *         缺口一长 (>16 箱 = 1/4 电周期), diag_lut_repair 的弦插值就无法
 *         判定 6283 分支 —— 端点的"短支 + 名义斜率"给不出真值, 补出来的是
 *         多/少整整一个电周期的假表, 换向后转子必在那段里弹跳并堵转发热。
 * @return 最长连续坏箱数 (0 = 全表都踩到过)
 */
uint32_t diag_lut_max_hole(void);

/** @brief 把当前整张表拷出来 (mrad), 存档用。须在修表之后调用。 */
void diag_lut_snapshot(int16_t *vals_mrad);

/** @brief 用存档覆盖整张表 (mrad), 每箱标记为有效。上电直接查表换向用。 */
void diag_lut_load(const int16_t *vals_mrad);

/**
 * @brief  表自洽性体检 (不修表)。
 *         沿 raw 走一圈, 每一步取【最接近 ENC_LUT_STEP_NOMINAL 的回绕分支】
 *         —— 只站在"这张表该长什么样"的物理约束上, 不被 ±pi 的表示法骗到。
 *         健康的表: 回绕和 = 75396 mrad (正好 12 个电周期), back = 0。
 *           - back 不为 0 且只有几十 mrad: 只是学习噪声(每箱样本太少), 无害;
 *             若是几百 mrad 以上, 那就是换向后会顿一下的位置。
 *           - 回绕和差一个 6283: 某一段整片插反了, 换向后必然卡死。
 * @param  p_back      输出: "倒退"的箱数, 可为 NULL
 * @param  p_worst_neg 输出: 最负的一步 (mrad, <= 0), 可为 NULL
 * @param  p_worst_pos 输出: 最大的一步 (mrad), 可为 NULL
 * @return 整圈累积电角度 (mrad), 期望 75396
 */
int32_t diag_lut_health(uint32_t *p_back, int32_t *p_worst_neg, int32_t *p_worst_pos);

/**
 * @brief  卡顿定位 (换向后用): 判"这一拍转子是不是被顶住了"。
 *         判据与学习门控相同 (最近 ENC_LUT_MOVE_WIN 拍 raw 位移 < MOVE_CNT),
 *         但用【独立】的环形缓冲, 不影响学表。
 * @return 1 = 顶住, 0 = 在动 (窗口还没填满时也返回 0)
 */
int diag_lut_dwell(uint32_t raw);

/* ================= 换向后的"转速图" (定位到底卡在哪) =================
 *   64 个扇区 (每扇区 256 counts = 机械 5.6°), 每个扇区记录【最慢的一个
 *   ENC_SPDMAP_WIN 拍滑动位移】(counts/WIN拍)。
 *   换向正常时各扇区速度差不多; 某个扇区明显低 = 换向误差/齿槽真正顶住的位置,
 *   直接对应 raw: 扇区 n 覆盖 raw [n*256, n*256+255)。
 *   每扇区攒够 ENC_SPDMAP_MIN 个窗口就整表打一次 (SPDMAP..), 然后停止采集。 */
#define ENC_SPDMAP_SECT   (64U)
#define ENC_SPDMAP_WIN    (4U)
#define ENC_SPDMAP_MIN    (60U)

/** 喂一拍 (换向后每拍调用); @return 1 = 刚好采满, 该打印了 */
int      diag_spdmap_feed(uint32_t raw);
/** 取某扇区的最小 WIN 拍位移 (counts), 没数据返回 0 */
uint32_t diag_spdmap_get(uint32_t sect);

/**
 * @brief  查表取电角度 (相邻箱线性插值)
 * @param  enc_raw  编码器原始角 (14bit)
 * @return 电角度 mrad; 若该处表项无效则返回 0xFFFFFFFF (调用者应退回线性公式)
 */
uint32_t diag_lut_angle(uint32_t enc_raw);

/** @brief 格式化 LUT 的一行 (line: 0..ENC_LUT_LINES-1, 每行 16 个值) */
int diag_lut_dump_line(char *buf, uint32_t size, uint32_t line);

/* ---- 阶段 ---- */
void diag_set_stage(uint32_t stage);

/* ---- 心跳 ---- */
void diag_tick(void);                          /* loop_cnt++ */

/* ---- 采样量 ---- */
void diag_set_vbus_mv(uint32_t vbus_mv);
void diag_set_currents_ma(int32_t iu, int32_t iv, int32_t iw);
void diag_set_enc(uint32_t angle, uint32_t frames, uint32_t errors,
                  uint32_t raw, uint32_t transfer_fail);

/* ---- bit-bang 扫描结果 (写一次即固定, 不随控制环刷新) ---- */
void diag_set_bb(uint32_t hits, uint32_t crc_hits, int32_t best,
                 uint32_t raw, uint32_t miso_static);

/* ---- 控制输出 ---- */
void diag_set_control(uint32_t trip,
                      uint32_t ol_voltage_mv,
                      uint32_t ol_elec_angle_mrad,
                      uint32_t duty_u_m, uint32_t duty_v_m, uint32_t duty_w_m);

/**
 * @brief  生成一条遥测上报字符串 (周期上报用)
 * @param  buf     输出缓冲
 * @param  size    缓冲大小
 * @param  elec_hz 当前电频率 (Hz)
 * @param  mech_rpm 当前机械转速 (RPM)
 * @param  vbus_mv  母线电压 (mV)
 * @return 写入的字节数 (不含结尾 '\0'), 缓冲不足时返回所需长度
 * @note   内部只使用整数, 避免 %f 带来的尺寸/性能开销。
 */
int diag_telemetry(char *buf, uint32_t size, float elec_hz, float mech_rpm,
                   uint32_t vbus_mv);

/**
 * @brief  极简 CSV 遥测: 三个整数, 无任何标签, 逗号分隔
 * @param  buf           输出缓冲
 * @param  size          缓冲大小
 * @param  sw_elec_mrad  软件(模拟)电角度, mrad, 0..6283
 * @param  enc_elec_mrad 编码器【电角度】, mrad, 0..6283 (含已标定偏置)
 * @param  enc_raw       编码器【原始角度】, 14bit, 0..16383  <-- 判断编码器
 *                       本身是否正常就看这一列: 转子匀速转时它应当单调、
 *                       均匀地推进, 一圈正好 0..16383 走一遍。
 * @return 写入的字节数
 * @note   格式就是 "%d,%d,%d"。【不带 '\r\n'】: console/USB 两个日志后端都会
 *         自己补 CRLF, 这里再带一次串口上就会多出空行。
 */
int diag_csv_telemetry(char *buf, uint32_t size,
                       int32_t sw_elec_mrad, int32_t enc_elec_mrad,
                       int32_t enc_raw);

/**
 * @brief  极简 CSV 遥测 (带查表值): 四个整数, 逗号分隔
 * @param  buf           输出缓冲
 * @param  size          缓冲大小
 * @param  sw_elec_mrad  软件(模拟)电角度, mrad, 0..6283
 * @param  enc_elec_mrad 编码器电角度, mrad, 0..6283
 * @param  enc_raw       编码器原始角度, 14bit, 0..16383
 * @param  lut_mrad      查表法(diag_lut_angle)得到的电角度, mrad;
 *                       表项无效时为 -1
 * @return 写入的字节数
 * @note   格式 "%d,%d,%d,%d"
 */
int diag_csv_telemetry_lut(char *buf, uint32_t size,
                           int32_t sw_elec_mrad, int32_t enc_elec_mrad,
                           int32_t enc_raw, int32_t lut_mrad);

/* ---- 校准遥测 ---- */

/** @brief 写入校准状态 */
void diag_set_cal(uint32_t sw_elec_mrad, uint32_t enc_raw,
                  uint32_t enc_elec_mrad, int32_t diff_mrad);

/**
 * @brief  生成一行校准遥测: SW=软件电角度 ENC=编码器角度 ENCEL=编码器电角度 DIFF=偏置
 * @return 写入的字节数 (不含结尾 '\0')
 */
int diag_cal_telemetry(char *buf, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* APP_DIAG_H */
