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
    uint32_t trip;
    uint32_t ol_voltage_mv;
    uint32_t ol_elec_angle_mrad;
    uint32_t duty_u_m;
    uint32_t duty_v_m;
    uint32_t duty_w_m;
} app_diag_t;

extern volatile app_diag_t g_diag;     /* 调试器直接 watch 这个变量 */

/* ---- 阶段 ---- */
void diag_set_stage(uint32_t stage);

/* ---- 心跳 ---- */
void diag_tick(void);                          /* loop_cnt++ */

/* ---- 采样量 ---- */
void diag_set_vbus_mv(uint32_t vbus_mv);
void diag_set_currents_ma(int32_t iu, int32_t iv, int32_t iw);
void diag_set_enc(uint32_t angle, uint32_t frames, uint32_t errors);

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

#ifdef __cplusplus
}
#endif

#endif /* APP_DIAG_H */
