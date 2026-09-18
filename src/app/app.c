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

#include "app.h"
#include "app_diag.h"
#include "app_log.h"
#include "app_encoder.h"

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_usb.h"
#include "bsp_sei.h"

#include "foc_controller.h"

/* ================= 应用实例 ================= */
static foc_controller_t s_ctrl;
static app_encoder_t    s_encoder;

/* 当前三相电流缓存 (mA), 供中断回调与主循环共享 */
static volatile int32_t s_iu_ma;
static volatile int32_t s_iv_ma;
static volatile int32_t s_iw_ma;

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

/* SEI 新帧到达: 由 app_encoder_update 在主循环统一处理, 这里只做轻量标记 */
static void app_on_sei_frame(const mt6701_frame_t *frame, uint32_t raw)
{
    (void)raw;
    diag_set_enc(frame->angle, bsp_sei_get_frame_count(), bsp_sei_get_error_count());
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

    /* SEI (MT6701 编码器) 初始化 */
    bsp_sei_init();
    diag_set_stage(APP_STAGE_SEI_INIT);
    LOG_I("ENC", "MT6701 SEI (ctrl%d) init done", BSP_SEI_CTRL_INDEX);

    /* 统一 FOC 控制器: 默认开环模式 */
    foc_controller_init(&s_ctrl, dt);
    foc_controller_set_voltage(&s_ctrl, APP_OPENLOOP_VOLTAGE_V);
    foc_controller_set_current_limit(&s_ctrl, APP_OPENLOOP_CURRENT_LIMIT);
    s_ctrl.openloop.elec_accel = APP_OPENLOOP_ACCEL;
#if APP_OPENLOOP_ALIGN_ONLY
    foc_controller_set_align_angle(&s_ctrl, APP_OPENLOOP_ALIGN_DEG * (FOC_TWO_PI / 360.0f));
#else
    foc_controller_set_speed_rpm(&s_ctrl, APP_OPENLOOP_TARGET_RPM);
#endif
#if APP_OPENLOOP_DEMO_ENABLE
    foc_controller_enable(&s_ctrl, true);
#endif
    diag_set_stage(APP_STAGE_FOC_INIT);

    /* 编码器封装: 速度估算低通系数 0.1 */
    app_encoder_init(&s_encoder, dt, 0.1f);

    /* 注册中断回调: 硬件中断留在 BSP, 应用逻辑经此上抛 */
    bsp_adc_register_conv_cb(app_on_adc_conv);
    bsp_sei_register_frame_cb(app_on_sei_frame);
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
    (void)app_encoder_update(&s_encoder);

    diag_set_stage(APP_STAGE_LOOP_VBUS);
    vbus = (float)bsp_adc_get_vbus_mv() * 0.001f;
    diag_set_vbus_mv((uint32_t)(vbus * 1000.0f));

    diag_set_stage(APP_STAGE_LOOP_CURRENT);
    bsp_adc_get_currents_ma(&iu_ma, &iv_ma, &iw_ma);
    ia = (float)iu_ma * 0.001f;
    ib = (float)iv_ma * 0.001f;
    ic = (float)iw_ma * 0.001f;
    diag_set_currents_ma(iu_ma, iv_ma, iw_ma);

    /* ---- 控制: 统一控制器 (开环 or 电流环) ---- */
    diag_set_stage(APP_STAGE_LOOP_CONTROL);
    {
        float elec_rad = app_encoder_get_elec_rad(&s_encoder);
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

    /* ---- 周期性上报 ---- */
    if (++tick >= APP_REPORT_TICKS) {
        char msg[256];
        float elec_hz = s_ctrl.openloop.elec_speed / FOC_TWO_PI;
        float mech_rpm = app_encoder_get_mech_rpm(&s_encoder);

        tick = 0;
        (void)diag_telemetry(msg, sizeof(msg), elec_hz, mech_rpm,
                             bsp_adc_get_vbus_mv());

        /* 遥测原样输出, 复用日志后端 (console + USB) */
        app_log_write_raw(msg);
    }
}
