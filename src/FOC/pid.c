/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 通用 PID 实现 (说明见 pid.h; 算法与 F405_M3508_FOC 的 PID.c 一致)
 */
#include "pid.h"

#include <stddef.h>     /* NULL */

static float pid_clamp(float v, float lo, float hi)
{
    if (v > hi) {
        return hi;
    }
    if (v < lo) {
        return lo;
    }
    return v;
}

void PID_Init(PID_Controller_t *pid, float kp, float ki, float dt,
              float out_min, float out_max,
              float int_min, float int_max)
{
    if (pid == NULL) {
        return;
    }
    pid->kp         = kp;
    pid->ki         = ki;
    pid->dt         = dt;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->int_min    = int_min;
    pid->int_max    = int_max;
    pid->integrator = 0.0f;
}

void PID_InitSym(PID_Controller_t *pid, float kp, float ki, float dt,
                  float out_limit, float int_limit)
{
    PID_Init(pid, kp, ki, dt, -out_limit, out_limit, -int_limit, int_limit);
}

void PID_SetGains(PID_Controller_t *pid, float kp, float ki)
{
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
}

void PID_Reset(PID_Controller_t *pid)
{
    if (pid != NULL) {
        pid->integrator = 0.0f;
    }
}

float PID_Step(PID_Controller_t *pid, float error)
{
    float unsaturated;
    float output;
    uint8_t integrate;

    if (pid == NULL) {
        return 0.0f;
    }

    /* 1) 积分器先夹一次: 防止外部改过限幅后带着越界的旧值进来 */
    pid->integrator = pid_clamp(pid->integrator, pid->int_min, pid->int_max);

    /* 2) 先算一次"没限幅会是多少", 只用它来判断饱和方向 */
    unsaturated = pid->kp * error + pid->integrator;
    output      = pid_clamp(unsaturated, pid->out_min, pid->out_max);

    /* 3) 条件积分 (抗饱和): 没饱和, 或者饱和了但误差正在把输出往回拉 */
    integrate = (uint8_t)(((output > pid->out_min) && (output < pid->out_max)) ||
                          ((output >= pid->out_max) && (error < 0.0f)) ||
                          ((output <= pid->out_min) && (error > 0.0f)));
    if (integrate != 0U) {
        pid->integrator += pid->ki * error * pid->dt;
        pid->integrator = pid_clamp(pid->integrator, pid->int_min, pid->int_max);
    }

    /* 4) 用更新后的积分器出最终结果 */
    output = pid->kp * error + pid->integrator;

    return pid_clamp(output, pid->out_min, pid->out_max);
}

float PID_GetIntegrator(const PID_Controller_t *pid)
{
    return (pid != NULL) ? pid->integrator : 0.0f;
}
