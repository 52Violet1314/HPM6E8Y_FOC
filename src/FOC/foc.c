/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 磁场定向控制实现
 */

#include "foc.h"

/* --------------------------------------------------------------------------
 * 通用 PI (带积分限幅 / 饱和时停止积分, 即抗积分饱和)
 * -------------------------------------------------------------------------- */
void foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_limit)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_limit = out_limit;
    pi->out = 0.0f;
}

void foc_pi_reset(foc_pi_t *pi)
{
    pi->integral = 0.0f;
    pi->out = 0.0f;
}

float foc_pi_run(foc_pi_t *pi, float error, float ts)
{
    float out = pi->kp * error + pi->integral;
    bool saturated = false;

    if (out > pi->out_limit) {
        out = pi->out_limit;
        saturated = true;
    } else if (out < -pi->out_limit) {
        out = -pi->out_limit;
        saturated = true;
    }

    if (!saturated) {
        pi->integral += pi->ki * ts * error;
        pi->integral = foc_clampf(pi->integral, -pi->out_limit, pi->out_limit);
    }

    pi->out = out;
    return out;
}

/* --------------------------------------------------------------------------
 * FOC
 * -------------------------------------------------------------------------- */
void foc_init(foc_t *foc)
{
    if (foc == NULL) {
        return;
    }

    foc_math_init();    /* 初始化 sin/cos 查表 (FOC_USE_HPM_MATH=1 时有效) */

    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->id = 0.0f;
    foc->iq = 0.0f;
    foc->ud = 0.0f;
    foc->uq = 0.0f;
    foc->duty_u = 0.5f;
    foc->duty_v = 0.5f;
    foc->duty_w = 0.5f;
    foc->elec_angle = 0.0f;
    foc->vdc = 0.0f;
    foc->ts = 1.0f / (float)FOC_CURRENT_LOOP_FREQ_HZ;
    foc->v_limit = 0.0f;
    foc->enable = false;

    foc_pi_init(&foc->id_pi, FOC_DEFAULT_CURRENT_KP, FOC_DEFAULT_CURRENT_KI, FOC_VOLTAGE_LIMIT_V);
    foc_pi_init(&foc->iq_pi, FOC_DEFAULT_CURRENT_KP, FOC_DEFAULT_CURRENT_KI, FOC_VOLTAGE_LIMIT_V);
}

void foc_reset(foc_t *foc)
{
    if (foc == NULL) {
        return;
    }
    foc_pi_reset(&foc->id_pi);
    foc_pi_reset(&foc->iq_pi);
    foc->ud = 0.0f;
    foc->uq = 0.0f;
}

void foc_set_current_ref(foc_t *foc, float id_ref, float iq_ref)
{
    if (foc == NULL) {
        return;
    }
    foc->id_ref = foc_clampf(id_ref, -FOC_CURRENT_LIMIT_A, FOC_CURRENT_LIMIT_A);
    foc->iq_ref = foc_clampf(iq_ref, -FOC_CURRENT_LIMIT_A, FOC_CURRENT_LIMIT_A);
}

void foc_set_gains(foc_t *foc, float kp, float ki)
{
    if (foc == NULL) {
        return;
    }
    foc->id_pi.kp = kp;
    foc->id_pi.ki = ki;
    foc->iq_pi.kp = kp;
    foc->iq_pi.ki = ki;
}

void foc_enable(foc_t *foc, bool enable)
{
    if (foc == NULL) {
        return;
    }
    if (!enable) {
        foc_reset(foc);
        foc->duty_u = 0.5f;
        foc->duty_v = 0.5f;
        foc->duty_w = 0.5f;
    }
    foc->enable = enable;
}

void foc_update(foc_t *foc, float ia, float ib, float ic, float elec_angle, float vdc)
{
    float ialpha, ibeta;
    float sin_t, cos_t;
    float ualpha, ubeta;
    float vmax;

    if (foc == NULL) {
        return;
    }

    foc->elec_angle = elec_angle;
    foc->vdc = vdc;

    /* SVPWM 线性区最大相电压 = vdc/√3。
     *
     * 【为什么只取 85%, 而不是 95%】这 15% 不是给电压留的, 是给【电流采样】留的。
     *   低边采样只能在"三相下桥同时导通"的窗口里做, 该窗口以 PWM 计数器回绕点为
     *   中心, 半宽 = reload/2×(1-0.98×duty_max)。占空比越极端窗口越窄:
     *       向量=0.85×线性极限 -> duty 约 0.075~0.925 -> 窗口半宽约 465 counts = 2.3us ✓
     *       向量=0.95×线性极限 -> duty 约 0.03 ~0.97  -> 窗口半宽约 190 counts = 0.95us
     *   后者再扣掉死区, 采样点几乎贴在开关沿上 —— 实测就是这样采到过 10A 的假值
     *   (12V+3.1Ω 下即使相间短路也只有 1.9A, 物理上不可能), 把整个运行打断。
     *   标准做法就是留"调制余量", 代价只是最高电压少 10%。 */
    vmax = vdc * FOC_ONE_BY_SQRT3 * 0.85f;
    foc->v_limit = vmax;
    foc->id_pi.out_limit = vmax;
    foc->iq_pi.out_limit = vmax;

    if (!foc->enable || vdc < 1.0f) {
        foc->duty_u = 0.5f;
        foc->duty_v = 0.5f;
        foc->duty_w = 0.5f;
        return;
    }

    /* 1) Clarke + Park */
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    foc_sin_cos(elec_angle, &sin_t, &cos_t);
    foc_park(ialpha, ibeta, sin_t, cos_t, &foc->id, &foc->iq);

    /* 2) 电流环 PI */
    foc->ud = foc_pi_run(&foc->id_pi, foc->id_ref - foc->id, foc->ts);
    foc->uq = foc_pi_run(&foc->iq_pi, foc->iq_ref - foc->iq, foc->ts);

    /* 3) 电压矢量限幅 (保证在 SVPWM 线性区) */
    {
        float mag = sqrtf(foc->ud * foc->ud + foc->uq * foc->uq);
        if (mag > vmax && mag > 1e-6f) {
            float scale = vmax / mag;
            foc->ud *= scale;
            foc->uq *= scale;
        }
    }

    /* 4) 反 Park + SVPWM */
    foc_inv_park(foc->ud, foc->uq, sin_t, cos_t, &ualpha, &ubeta);
    foc_svpwm(ualpha, ubeta, vdc, &foc->duty_u, &foc->duty_v, &foc->duty_w);
}

void foc_get_duty(const foc_t *foc, float *du, float *dv, float *dw)
{
    if (foc == NULL) {
        return;
    }
    if (du != NULL) {
        *du = foc->duty_u;
    }
    if (dv != NULL) {
        *dv = foc->duty_v;
    }
    if (dw != NULL) {
        *dw = foc->duty_w;
    }
}

void foc_open_loop(float ud, float uq, float elec_angle, float vdc,
                   float *du, float *dv, float *dw)
{
    float sin_t, cos_t;
    float ualpha, ubeta;

    foc_sin_cos(elec_angle, &sin_t, &cos_t);
    foc_inv_park(ud, uq, sin_t, cos_t, &ualpha, &ubeta);
    foc_svpwm(ualpha, ubeta, vdc, du, dv, dw);
}

/* --------------------------------------------------------------------------
 * 速度估算
 * -------------------------------------------------------------------------- */
void foc_speed_est_init(foc_speed_est_t *est, float ts, float lpf_alpha)
{
    if (est == NULL) {
        return;
    }
    est->prev_angle = 0.0f;
    est->speed = 0.0f;
    est->ts = (ts > 0.0f) ? ts : 1e-3f;
    est->lpf_alpha = foc_clampf(lpf_alpha, 0.001f, 1.0f);
    est->started = false;
}

float foc_speed_est_update(foc_speed_est_t *est, float mech_angle)
{
    float delta;
    float raw;

    if (est == NULL) {
        return 0.0f;
    }

    if (!est->started) {
        est->prev_angle = mech_angle;
        est->started = true;
        return 0.0f;
    }

    /* 角度差分, 做 ±π 过零处理 */
    delta = foc_normalize_angle_pm_pi(mech_angle - est->prev_angle);
    est->prev_angle = mech_angle;

    raw = delta / est->ts;
    est->speed += est->lpf_alpha * (raw - est->speed);

    return est->speed;
}

void foc_speed_est_set_ts(foc_speed_est_t *est, float ts)
{
    if ((est != NULL) && (ts > 0.0f)) {
        est->ts = ts;
    }
}
