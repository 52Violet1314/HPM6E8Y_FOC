/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 统一控制器实现
 */

#include "foc_controller.h"

void foc_controller_init(foc_controller_t *ctrl, float ts)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->mode = foc_ctrl_mode_current;
    ctrl->duty_u = 0.5f;
    ctrl->duty_v = 0.5f;
    ctrl->duty_w = 0.5f;
    ctrl->elec_angle = 0.0f;
    ctrl->vbus = 0.0f;
    ctrl->ts = (ts > 0.0f) ? ts : 1e-3f;
    ctrl->enabled = false;

    foc_init(&ctrl->foc);
    foc_openloop_init(&ctrl->openloop, ctrl->ts);

    /* 【必须修正】foc_init() 里把 foc.ts 设成了 1/FOC_CURRENT_LOOP_FREQ_HZ
     * (=50us, 按 20kHz 电流环假设), 但本工程实际是在 1kHz 主循环里调用
     * foc_update() 的。PI 的积分项是 integral += ki * ts * error, ts 差 20 倍
     * 就意味着积分速度差 20 倍 —— 环路会严重迟滞、几乎不起作用。
     * 这里统一成真实的调用周期。 */
    ctrl->foc.ts = ctrl->ts;
}

void foc_controller_set_mode(foc_controller_t *ctrl, foc_ctrl_mode_t mode)
{
    if (ctrl == NULL) {
        return;
    }
    ctrl->mode = mode;

    /* 切换时复位输出到零矢量, 避免两种算法状态跳变造成电流冲击 */
    ctrl->duty_u = 0.5f;
    ctrl->duty_v = 0.5f;
    ctrl->duty_w = 0.5f;
}

foc_ctrl_mode_t foc_controller_get_mode(const foc_controller_t *ctrl)
{
    return (ctrl != NULL) ? ctrl->mode : foc_ctrl_mode_openloop;
}

void foc_controller_set_dt(foc_controller_t *ctrl, float dt)
{
    if ((ctrl != NULL) && (dt > 0.0f)) {
        ctrl->ts          = dt;     /* 开环/电流环都用它作为积分步长 */
        ctrl->foc.ts      = dt;
        ctrl->openloop.ts = dt;
    }
}

void foc_controller_enable(foc_controller_t *ctrl, bool enable)
{
    if (ctrl == NULL) {
        return;
    }
    ctrl->enabled = enable;

    /* 同时下发到两个子控制器, 由它们各自处理零矢量输出 */
    foc_enable(&ctrl->foc, enable);
    foc_openloop_enable(&ctrl->openloop, enable);

    if (!enable) {
        ctrl->duty_u = 0.5f;
        ctrl->duty_v = 0.5f;
        ctrl->duty_w = 0.5f;
    }
}

/* ---- 开环模式参数 ---- */
void foc_controller_set_voltage(foc_controller_t *ctrl, float volts)
{
    if (ctrl != NULL) {
        foc_openloop_set_voltage(&ctrl->openloop, volts);
    }
}

void foc_controller_set_speed_rpm(foc_controller_t *ctrl, float mech_rpm)
{
    if (ctrl != NULL) {
        foc_openloop_set_speed_rpm(&ctrl->openloop, mech_rpm);
    }
}

void foc_controller_set_speed_elec(foc_controller_t *ctrl, float elec_rad_s)
{
    if (ctrl != NULL) {
        foc_openloop_set_speed_elec(&ctrl->openloop, elec_rad_s);
    }
}

void foc_controller_set_current_limit(foc_controller_t *ctrl, float amps)
{
    if (ctrl != NULL) {
        foc_openloop_set_current_limit(&ctrl->openloop, amps);
    }
}

void foc_controller_set_align_angle(foc_controller_t *ctrl, float elec_rad)
{
    if (ctrl != NULL) {
        ctrl->openloop.elec_angle = elec_rad;
        foc_openloop_set_speed_elec(&ctrl->openloop, 0.0f);
    }
}

void foc_controller_set_external_angle(foc_controller_t *ctrl, float elec_rad)
{
    /* 与 set_align_angle 的内部动作相同(写死角度 + 停掉积分), 但语义不同:
     * 这个函数是【每个控制周期都调】的, 用来把开环电压矢量的角度交给外部
     * (编码器) 决定。命名分开是为了让调用点一眼看出"这是编码器换向"。 */
    foc_controller_set_align_angle(ctrl, elec_rad);
}

/* ---- 电流环模式参数 ---- */
void foc_controller_set_current_ref(foc_controller_t *ctrl, float id_ref, float iq_ref)
{
    if (ctrl != NULL) {
        foc_set_current_ref(&ctrl->foc, id_ref, iq_ref);
    }
}

void foc_controller_set_gains(foc_controller_t *ctrl, float kp, float ki)
{
    if (ctrl != NULL) {
        foc_set_gains(&ctrl->foc, kp, ki);
    }
}

void foc_controller_run(foc_controller_t *ctrl,
                        float ia, float ib, float ic,
                        float elec_angle, float vbus)
{
    float du, dv, dw;

    if (ctrl == NULL) {
        return;
    }

    ctrl->elec_angle = elec_angle;
    ctrl->vbus = vbus;

    if (!ctrl->enabled) {
        ctrl->duty_u = 0.5f;
        ctrl->duty_v = 0.5f;
        ctrl->duty_w = 0.5f;
        return;
    }

    switch (ctrl->mode) {
    case foc_ctrl_mode_current:
        /* 电流环: 需要编码器电角度 */
        foc_update(&ctrl->foc, ia, ib, ic, elec_angle, vbus);
        foc_get_duty(&ctrl->foc, &du, &dv, &dw);
        break;

    case foc_ctrl_mode_openloop:
    default:
        /* 开环: 电角度由内部积分, 电流仅用于过流保护 */
        foc_openloop_update(&ctrl->openloop, ctrl->ts, vbus, ia, ib, ic);
        foc_openloop_get_duty(&ctrl->openloop, &du, &dv, &dw);
        break;
    }

    ctrl->duty_u = du;
    ctrl->duty_v = dv;
    ctrl->duty_w = dw;
}

void foc_controller_get_duty(const foc_controller_t *ctrl,
                             float *du, float *dv, float *dw)
{
    if (ctrl == NULL) {
        return;
    }
    if (du != NULL) {
        *du = ctrl->duty_u;
    }
    if (dv != NULL) {
        *dv = ctrl->duty_v;
    }
    if (dw != NULL) {
        *dw = ctrl->duty_w;
    }
}

uint32_t foc_controller_get_trip(const foc_controller_t *ctrl)
{
    if (ctrl == NULL) {
        return 0U;
    }
    if (ctrl->mode == foc_ctrl_mode_openloop) {
        return ctrl->openloop.trip;
    }
    return 0U;
}
