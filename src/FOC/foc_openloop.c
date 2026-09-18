/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 开环电压矢量驱动实现
 */

#include <stddef.h>

#include "foc_openloop.h"

void foc_openloop_init(foc_openloop_t *ol, float ts)
{
    if (ol == NULL) {
        return;
    }

    ol->elec_angle = 0.0f;
    ol->elec_speed = 0.0f;
    ol->voltage = 0.0f;
    ol->duty_u = 0.5f;
    ol->duty_v = 0.5f;
    ol->duty_w = 0.5f;
    ol->enabled = false;
    ol->output_on = false;
    ol->trip = 0;

    ol->elec_speed_target = 0.0f;
    ol->elec_accel = FOC_OPENLOOP_ACCEL_RAD_S2;
    ol->voltage_target = FOC_OPENLOOP_VOLTAGE_V;
    ol->voltage_slew = FOC_OPENLOOP_VOLTAGE_SLEW;
    ol->current_limit = FOC_OPENLOOP_CURRENT_LIMIT_A;
    ol->vbus_min = FOC_OPENLOOP_VBUS_MIN_V;
    ol->ts = (ts > 0.0f) ? ts : 1e-3f;
}

void foc_openloop_enable(foc_openloop_t *ol, bool enable)
{
    if (ol == NULL) {
        return;
    }
    ol->enabled = enable;
    if (!enable) {
        ol->elec_speed = 0.0f;
        ol->voltage = 0.0f;
        ol->duty_u = 0.5f;
        ol->duty_v = 0.5f;
        ol->duty_w = 0.5f;
    }
}

void foc_openloop_set_speed_elec(foc_openloop_t *ol, float elec_speed_rad_s)
{
    if (ol != NULL) {
        ol->elec_speed_target = elec_speed_rad_s;
    }
}

void foc_openloop_set_speed_hz(foc_openloop_t *ol, float elec_hz)
{
    if (ol != NULL) {
        ol->elec_speed_target = elec_hz * FOC_TWO_PI;
    }
}

void foc_openloop_set_speed_rpm(foc_openloop_t *ol, float mech_rpm)
{
    if (ol != NULL) {
        ol->elec_speed_target = mech_rpm / 60.0f * FOC_TWO_PI * FOC_MOTOR_POLE_PAIRS;
    }
}

void foc_openloop_set_voltage(foc_openloop_t *ol, float volts)
{
    if (ol != NULL) {
        ol->voltage_target = (volts >= 0.0f) ? volts : 0.0f;
    }
}

void foc_openloop_set_current_limit(foc_openloop_t *ol, float amps)
{
    if (ol != NULL) {
        ol->current_limit = amps;
    }
}

void foc_openloop_clear_trip(foc_openloop_t *ol)
{
    if (ol != NULL) {
        ol->trip = 0;
    }
}

void foc_openloop_update(foc_openloop_t *ol, float dt, float vbus,
                         float ia, float ib, float ic)
{
    float v_eff;
    float v_step;
    float w_step;
    float imax;
    float sin_t;
    float cos_t;

    if (ol == NULL) {
        return;
    }
    if (dt <= 0.0f) {
        dt = ol->ts;
    }

    /* ---- 保护 1: 母线欠压 -> 立即封锁输出 ---- */
    if (vbus < ol->vbus_min) {
        ol->trip |= FOC_OL_TRIP_VBUS_LOW;
        ol->output_on = false;
        ol->elec_speed = 0.0f;
        ol->voltage = 0.0f;
        ol->duty_u = 0.5f;
        ol->duty_v = 0.5f;
        ol->duty_w = 0.5f;
        return;
    }
    ol->trip &= ~FOC_OL_TRIP_VBUS_LOW;

    /* ---- 未使能: 输出零矢量 ---- */
    if (!ol->enabled) {
        ol->output_on = false;
        ol->elec_speed = 0.0f;
        ol->voltage = 0.0f;
        ol->duty_u = 0.5f;
        ol->duty_v = 0.5f;
        ol->duty_w = 0.5f;
        return;
    }
    ol->output_on = true;

    /* ---- 电压 / 电角速度斜坡 ---- */
    v_step = ol->voltage_slew * dt;
    if (ol->voltage < ol->voltage_target) {
        ol->voltage += v_step;
        if (ol->voltage > ol->voltage_target) {
            ol->voltage = ol->voltage_target;
        }
    } else if (ol->voltage > ol->voltage_target) {
        ol->voltage -= v_step;
        if (ol->voltage < ol->voltage_target) {
            ol->voltage = ol->voltage_target;
        }
    }

    w_step = ol->elec_accel * dt;
    if (ol->elec_speed < ol->elec_speed_target) {
        ol->elec_speed += w_step;
        if (ol->elec_speed > ol->elec_speed_target) {
            ol->elec_speed = ol->elec_speed_target;
        }
    } else if (ol->elec_speed > ol->elec_speed_target) {
        ol->elec_speed -= w_step;
        if (ol->elec_speed < ol->elec_speed_target) {
            ol->elec_speed = ol->elec_speed_target;
        }
    }

    /* ---- 保护 2: 过流折叠 ---- */
    v_eff = ol->voltage;
    if (ol->current_limit > 0.0f) {
        imax = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
        if (imax > ol->current_limit) {
            v_eff = ol->voltage * (ol->current_limit / imax);
            ol->voltage = v_eff;                /* 立即压低给定, 避免持续过流 */
            ol->trip |= FOC_OL_TRIP_OVERCURRENT;
        } else {
            ol->trip &= ~FOC_OL_TRIP_OVERCURRENT;
        }
    }

    /* ---- 积分电角度 ---- */
    ol->elec_angle = foc_normalize_angle(ol->elec_angle + ol->elec_speed * dt);

    /* ---- 电压矢量跟随电角度旋转: uα = V·cosθ, uβ = V·sinθ ---- */
    foc_sin_cos(ol->elec_angle, &sin_t, &cos_t);
    foc_svpwm(v_eff * cos_t, v_eff * sin_t, vbus,
              &ol->duty_u, &ol->duty_v, &ol->duty_w);
}

void foc_openloop_get_duty(const foc_openloop_t *ol, float *du, float *dv, float *dw)
{
    if (ol == NULL) {
        return;
    }
    if (du != NULL) {
        *du = ol->duty_u;
    }
    if (dv != NULL) {
        *dv = ol->duty_v;
    }
    if (dw != NULL) {
        *dw = ol->duty_w;
    }
}
