/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * YeeZi HPM6EMotor V2.0 (HPM6E8Y) BSP 示例入口
 *   - 三相互补 PWM (PD0~PD5 -> STDRIVE101 IN_UH/UL/VH/VL/WH/WL)
 *   - 相电流 / 母线电压 / 温度 ADC 采集
 *   - MT6701 磁编码器 (SSI 绝对角度, 经 SEI0 读取)
 *   - 开环电压矢量驱动 (默认) / FOC 电流环 (可选)
 *   - USB0 虚拟串口 CDC ACM, 周期性上报状态
 *
 * 【调试提示】
 *   本文件定义了一个全局 volatile 变量 g_diag, 用 J-Link/Cortex-Debug 的
 *   WATCH 窗口添加 g_diag 即可实时看到"卡在哪一步 / VBUS 多少 / 占空比多少"。
 *   即使没有串口也能定位问题。
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "hpm_debug_console.h"

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_usb.h"

#include "mt6701_sei.h"
#include "foc.h"
#include "foc_openloop.h"

/* ================= 运行模式 ================= */
/* 开环驱动: 电机相电阻/电感未知时的首选, 不依赖电流环参数。
 *   1 = 运行开环驱动 (默认)
 *   0 = 不运行, PWM 固定 50% */
#define OPENLOOP_DEMO_ENABLE    (1)

/* FOC 电流环演示: 需要先知道相电阻/电感并整定好 PI。与开环互斥。 */
#define FOC_DEMO_ENABLE         (0)

/* 调试用: 只把电压矢量固定在一个电角度上(转子被"吸住"/对齐), 不旋转。
 *   1 = 固定角度(用于验证 功率级/接线/PWM 是否正常, 不依赖能否起转)
 *   0 = 正常旋转开环
 * 用 1 时: 若电机会"咯噔"一下吸到某个位置并保持有劲, 说明功率级和接线是好的。
 */
#define OPENLOOP_ALIGN_ONLY     (0)
#define OPENLOOP_ALIGN_DEG      (0.0f)

/* ================= 开环调试参数 ================= */
/* 目标机械转速 (RPM)。12 极对 -> 电频率 = RPM/60*12。
 * 先用低速 (60 RPM -> 12Hz 电频率) 让转子能跟得上, 稳定后再往上加。 */
#define OPENLOOP_TARGET_RPM     (60.0f)
/* 电压矢量幅值 (V)。相电阻未知, 先从 2V 试; 转不动就加到 3~5V,
 * 同时盯住电流(现在电流换算已修正, 能真实显示)。 */
#define OPENLOOP_VOLTAGE_V      (2.0f)
/* 相电流限幅 (A)。超过后自动折叠降压, 相电阻未知时这是兜底保护。 */
#define OPENLOOP_CURRENT_LIMIT  (2.0f)
/* 电角加速度 (rad/s^2)。越小起步越柔, 转子越容易同步;
 * 0->12Hz(75rad/s) 用 20rad/s^2 大约 3.7s。 */
#define OPENLOOP_ACCEL          (20.0f)

/* 主循环周期 (ms) */
#define MAIN_LOOP_MS            (1U)
/* 上报周期 (主循环次数) */
#define REPORT_TICKS            (200U)

/* ================= 调试器可观察的诊断信息 =================
 * 用 J-Link 调试时在 WATCH 里添加 g_diag:
 *   stage     : 当前执行到哪一步(见下面 STAGE_* 定义), 卡住不动就是问题点
 *   loop_cnt  : 主循环心跳, 一直增加说明程序在跑
 *   vbus_mv   : 母线电压(mV), 正常应接近实际电源电压; 若为 0 说明采样/供电有问题
 *   trip      : 开环保护标志, bit0=母线欠压, bit1=过流折叠
 *   duty_*_m  : 三相占空比 x1000, 应该在 500 附近来回摆(旋转)或偏离 500(固定角度)
 */
#define STAGE_BOOT              (1U)
#define STAGE_PWM_INIT          (2U)
#define STAGE_ADC_INIT          (3U)
#define STAGE_USB_INIT          (4U)
#define STAGE_SEI_INIT          (5U)
#define STAGE_FOC_INIT          (6U)
#define STAGE_OPENLOOP_INIT     (7U)
#define STAGE_PWM_ENABLE        (8U)
#define STAGE_LOOP_TOP          (10U)
#define STAGE_LOOP_VBUS         (11U)
#define STAGE_LOOP_CURRENT      (12U)
#define STAGE_LOOP_CONTROL      (13U)
#define STAGE_LOOP_DONE         (14U)

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

volatile app_diag_t g_diag;     /* 调试器直接 watch 这个变量 */

int main(void)
{
    foc_t foc;
    foc_openloop_t openloop;
    foc_speed_est_t enc_speed;
    float dt = (float)MAIN_LOOP_MS * 0.001f;
    uint32_t tick = 0;
    char msg[224];

    memset((void *)&g_diag, 0, sizeof(g_diag));

    /* 板级初始化: 时钟树、引脚复用等, 必须最先调用 */
    board_init();
    g_diag.stage = STAGE_BOOT;

    printf("\nYeeZi HPM6EMotor open-loop demo\n");

    /* 外设初始化顺序: PWM -> ADC -> USB.
     * PWM 先初始化很重要 —— 因为 ADC 的触发源来自 PWM TRGO */
    bsp_pwm_init();
    g_diag.stage = STAGE_PWM_INIT;
    bsp_adc_init();
    g_diag.stage = STAGE_ADC_INIT;
    bsp_usb_init();
    g_diag.stage = STAGE_USB_INIT;

    /* MT6701 编码器: SEI0 同步主机模式, 周期读取 SSI 绝对角度 */
    mt6701_sei_init();
    g_diag.stage = STAGE_SEI_INIT;
    printf("MT6701 SEI (ctrl%d) init done\n", MT6701_SEI_CTRL_INDEX);

    /* FOC 电流环控制器 (仅 FOC_DEMO_ENABLE 时使用) */
    foc_init(&foc);
    foc_set_current_ref(&foc, 0.0f, 0.0f);
    g_diag.stage = STAGE_FOC_INIT;

    /* 开环驱动器: 设定目标转速 / 电压 / 限流 */
    foc_openloop_init(&openloop, dt);
    foc_openloop_set_voltage(&openloop, OPENLOOP_VOLTAGE_V);
    foc_openloop_set_current_limit(&openloop, OPENLOOP_CURRENT_LIMIT);
    openloop.elec_accel = OPENLOOP_ACCEL;
#if OPENLOOP_ALIGN_ONLY
    openloop.elec_angle = OPENLOOP_ALIGN_DEG * (FOC_TWO_PI / 360.0f);
    foc_openloop_set_speed_elec(&openloop, 0.0f);     /* 不转, 固定角度 */
#else
    foc_openloop_set_speed_rpm(&openloop, OPENLOOP_TARGET_RPM);
#endif
#if OPENLOOP_DEMO_ENABLE
    foc_openloop_enable(&openloop, true);
#endif
    g_diag.stage = STAGE_OPENLOOP_INIT;

    /* 编码器速度估算 (用于显示实际机械转速) */
    foc_speed_est_init(&enc_speed, dt, 0.1f);

    /* 初始 50% 占空比, 打开三相输出 */
    bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
    bsp_pwm_enable_output(true);
    g_diag.stage = STAGE_PWM_ENABLE;

    printf("PWM reload=%u, freq=%u Hz, poles=%u, target=%u RPM, V=%u mV\n",
           (unsigned)bsp_pwm_get_reload(), (unsigned)BSP_PWM_FREQUENCY_HZ,
           (unsigned)FOC_MOTOR_POLE_PAIRS,
           (unsigned)OPENLOOP_TARGET_RPM,
           (unsigned)(OPENLOOP_VOLTAGE_V * 1000.0f));
    printf("Tips: watch g_diag in debugger; vbus_mv must be > %u mV\n",
           (unsigned)(FOC_OPENLOOP_VBUS_MIN_V * 1000.0f));

    while (1) {
        int32_t iu_ma = 0, iv_ma = 0, iw_ma = 0;
        mt6701_frame_t enc = {0};
        float ia, ib, ic, vbus, mech_rad;
        float mech_rpm;

        g_diag.stage = STAGE_LOOP_TOP;
        g_diag.loop_cnt++;

        /* ---- 采集: 编码器 / 电流 / 母线电压 ---- */
        (void)mt6701_sei_get_latest(&enc);
        mech_rad = FOC_ENCODER_DIRECTION * mt6701_angle_to_rad(enc.angle);
        mech_rpm = foc_speed_est_update(&enc_speed, mech_rad) * (60.0f / FOC_TWO_PI);

        g_diag.enc_angle = enc.angle;

        g_diag.stage = STAGE_LOOP_VBUS;
        vbus = (float)bsp_adc_get_vbus_mv() * 0.001f;
        g_diag.vbus_mv = (uint32_t)(vbus * 1000.0f);

        g_diag.stage = STAGE_LOOP_CURRENT;
        bsp_adc_get_currents_ma(&iu_ma, &iv_ma, &iw_ma);
        ia = (float)iu_ma * 0.001f;
        ib = (float)iv_ma * 0.001f;
        ic = (float)iw_ma * 0.001f;
        g_diag.iu_ma = iu_ma;
        g_diag.iv_ma = iv_ma;
        g_diag.iw_ma = iw_ma;

        /* ---- 控制: 开环优先, 其次 FOC 电流环 ---- */
        g_diag.stage = STAGE_LOOP_CONTROL;
#if OPENLOOP_DEMO_ENABLE
        {
            float du, dv, dw;
            foc_openloop_update(&openloop, dt, vbus, ia, ib, ic);
            foc_openloop_get_duty(&openloop, &du, &dv, &dw);
            bsp_pwm_set_duty_uvw(du, dv, dw);
            g_diag.trip = openloop.trip;
            g_diag.ol_voltage_mv = (uint32_t)(openloop.voltage * 1000.0f);
            g_diag.ol_elec_angle_mrad = (uint32_t)(openloop.elec_angle * 1000.0f);
            g_diag.duty_u_m = (uint32_t)(du * 1000.0f);
            g_diag.duty_v_m = (uint32_t)(dv * 1000.0f);
            g_diag.duty_w_m = (uint32_t)(dw * 1000.0f);
        }
#elif FOC_DEMO_ENABLE
        {
            float du, dv, dw;
            float elec_rad = foc_mech_to_elec(mech_rad, FOC_ELEC_ANGLE_OFFSET);
            foc_update(&foc, ia, ib, ic, elec_rad, vbus);
            foc_get_duty(&foc, &du, &dv, &dw);
            bsp_pwm_set_duty_uvw(du, dv, dw);
            g_diag.duty_u_m = (uint32_t)(du * 1000.0f);
            g_diag.duty_v_m = (uint32_t)(dv * 1000.0f);
            g_diag.duty_w_m = (uint32_t)(dw * 1000.0f);
        }
#else
        (void)foc;
        (void)openloop;
#endif
        g_diag.stage = STAGE_LOOP_DONE;

        /* ---- 周期性上报 (避免 %f, 全部用整数) ---- */
        if (++tick >= REPORT_TICKS) {
            int n;
            int elec_hz_x10 = (int)((openloop.elec_speed / FOC_TWO_PI) * 10.0f);
            int e_int = elec_hz_x10 / 10;
            int e_dec = elec_hz_x10 % 10;

            if (e_dec < 0) {
                e_dec = -e_dec;
            }

            tick = 0;
            g_diag.enc_frames = mt6701_sei_get_frame_count();
            g_diag.enc_errors = mt6701_sei_get_error_count();

            n = snprintf(msg, sizeof(msg),
                         "OL F=%d.%dHz V=%umV I=%ld/%ld/%ldmA VBUS=%u E=%u RPM=%ld A=%umrad TRIP=%u CNT=%u ERR=%u SEI=%08X\r\n",
                         e_int, e_dec,
                         (unsigned)(openloop.voltage * 1000.0f),
                         (long)iu_ma, (long)iv_ma, (long)iw_ma,
                         (unsigned)bsp_adc_get_vbus_mv(),
                         (unsigned)enc.angle,
                         (long)mech_rpm,
                         (unsigned)(openloop.elec_angle * 1000.0f),
                         (unsigned)openloop.trip,
                         (unsigned)mt6701_sei_get_frame_count(),
                         (unsigned)mt6701_sei_get_error_count(),
                         (unsigned)mt6701_sei_peek_raw());

            printf("%s", msg);
            if (bsp_usb_connected()) {
                (void)bsp_usb_write((const uint8_t *)msg, (uint32_t)n);
            }
        }

        board_delay_ms(MAIN_LOOP_MS);
    }

    return 0;
}
