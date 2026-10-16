/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 位置环实现
 */
#include "app_motion.h"
#include "app.h"            /* APP_MOTION_* */
#include "foc_math.h"       /* FOC_PI / FOC_TWO_PI */

/* 方向统一系数。
 *
 * 【它统一的是谁】本模块内部的"位置正方向"要和【速度环/电流环的正方向】
 * 一致, 也就是"正 iq -> 位置读数增大"。而所用的机械角是
 *      mech_rad = FOC_ENCODER_DIRECTION * mt6701_angle_to_rad(raw)
 * 所以:
 *      FOC_ENCODER_DIRECTION = +1  ->  正 iq 让 mech_rad 增大 -> 这里取 +1
 *      FOC_ENCODER_DIRECTION = -1  ->  正 iq 让 mech_rad 减小 -> 这里取 -1
 * 写成表达式而不是写死, 就是为了避免"改了 foc_config.h 却忘了改这里"。
 *
 * 【注意它现在必须和速度环同号】速度环 (app.c) 是拿 app_encoder_get_mech_rpm()
 * 和给定比较的, 而那个 rpm 已经含了 FOC_ENCODER_DIRECTION。位置环如果单独
 * 反号, 两个环就在两个坐标系里, 一使能必然往一个方向冲到底(实测就是这样)。
 * 改方向只需要改 foc_config.h 的 FOC_ENCODER_DIRECTION, 这里自动跟随。 */
#define APP_MOTION_DIR_SIGN   (FOC_ENCODER_DIRECTION)

static float motion_wrap_2pi(float a)
{
    while (a >= FOC_TWO_PI) {
        a -= FOC_TWO_PI;
    }
    while (a < 0.0f) {
        a += FOC_TWO_PI;
    }
    return a;
}

static float motion_clamp(float v, float lim)
{
    if (v > lim) {
        return lim;
    }
    if (v < -lim) {
        return -lim;
    }
    return v;
}

/* 单圈角 -> 连续多圈角: 每拍算增量, 过零补 ±2π */
static float motion_unwrap(app_motion_t *m, float mech_rad)
{
    float d;

    if (!m->unwrap_valid) {
        m->prev_mech_rad = mech_rad;
        m->turns_rad     = mech_rad;
        m->unwrap_valid  = true;
        return m->turns_rad;
    }

    d = mech_rad - m->prev_mech_rad;
    if (d > FOC_PI) {
        d -= FOC_TWO_PI;            /* 正向过零 (~2π -> ~0) */
    } else if (d < -FOC_PI) {
        d += FOC_TWO_PI;            /* 反向过零 */
    }
    m->turns_rad    += d;
    m->prev_mech_rad = mech_rad;

    return m->turns_rad;
}

void app_motion_init(app_motion_t *m)
{
    if (m == NULL) {
        return;
    }
    m->pos_ref      = 0.0f;
    m->pos_fb       = 0.0f;
    m->speed_ref    = 0.0f;
    m->pos_kp       = APP_MOTION_POS_KP;
    m->speed_limit  = APP_MOTION_SPEED_LIMIT_RPM;
    m->turns_rad    = 0.0f;
    m->prev_mech_rad = 0.0f;
    m->unwrap_valid = false;
    m->enable       = false;
}

void app_motion_enable(app_motion_t *m, bool enable, float mech_rad)
{
    if (m == NULL) {
        return;
    }
    if (enable && !m->enable) {
        m->unwrap_valid = false;                      /* 重新建立展开基准 */
        m->pos_fb       = APP_MOTION_DIR_SIGN * motion_unwrap(m, motion_wrap_2pi(mech_rad));
        m->pos_ref      = m->pos_fb;                  /* 原地保持 */
        m->speed_ref    = 0.0f;
    }
    m->enable = enable;
    if (!enable) {
        m->speed_ref = 0.0f;
    }
}

void app_motion_set_target(app_motion_t *m, float pos_rad)
{
    if (m != NULL) {
        m->pos_ref = pos_rad;
    }
}

void app_motion_set_gain(app_motion_t *m, float kp)
{
    if (m != NULL) {
        m->pos_kp = kp;
    }
}

float app_motion_run(app_motion_t *m, float mech_rad)
{
    float err;

    if ((m == NULL) || (!m->enable)) {
        if (m != NULL) {
            m->speed_ref = 0.0f;
        }
        return 0.0f;
    }

    m->pos_fb = APP_MOTION_DIR_SIGN * motion_unwrap(m, motion_wrap_2pi(mech_rad));

    /* P 控制: 位置误差 -> 速度给定, 并限速(同时限制最大跟踪速度) */
    err          = m->pos_ref - m->pos_fb;
    m->speed_ref = motion_clamp(m->pos_kp * err, m->speed_limit);

    return m->speed_ref;
}

float app_motion_get_pos(const app_motion_t *m)
{
    return (m != NULL) ? m->pos_fb : 0.0f;
}

float app_motion_get_err(const app_motion_t *m)
{
    return (m != NULL) ? (m->pos_ref - m->pos_fb) : 0.0f;
}
