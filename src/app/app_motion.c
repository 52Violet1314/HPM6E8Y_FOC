/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 位置环实现
 */
#include "app_motion.h"
#include "app.h"            /* APP_MOTION_* */
#include "foc_math.h"       /* FOC_PI / FOC_TWO_PI */

/* ================= 方向统一: 只认一个来源 =================
 *
 * 【规则】位置环和速度环必须【同号】。速度环在 app.c 里用的是
 *      给定 = s_speed_target_rpm * s_speed_dir;   误差 = 给定 - rpm
 * 而这里的位置反馈也必须用同一套"正方向", 否则两个环各自在自己的坐标系里,
 * 一使能就是正反馈: 位置环输出正速度 -> 速度环按正速度出力 -> 电机往
 * 负方向跑 -> 位置误差更大 -> 输出更大的正速度 ...... 实测就是这个:
 *      pos 从 4746 漂到 -8500000 mrad, err=8500000, spdref=+60 饱和, rpm=-553
 *
 * 【所以不再自己乘 FOC_ENCODER_DIRECTION】
 *   以前这里硬编码 APP_MOTION_DIR_SIGN(-1), 而速度环默认 +1, 两边反号 ——
 *   这才是"位置环一开就往一边冲"的直接原因。现在位置环直接用
 *   app_speed_get_dir() 取速度环那同一个系数, 从根上保证同号。
 *   要多圈展开的方向感也是同一件事: mech_rad 里已经含了
 *   FOC_ENCODER_DIRECTION (见 app_encoder.c), 速度环也是用它, 所以这里
 *   只要"直接用它、不再二次取反"即可。 */
static float motion_dir(void)
{
    /* 速度环的方向系数 (由 app_init 自检/静态配置决定) */
    return app_speed_get_dir();
}

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

    /* 【防呆: 增益还是 0 就先自愈】
     * 实测坑: 延迟自检那条路径直接调 app_motion_enable(), 却没先调
     * app_motion_init() —— pos_kp/speed_limit 停在 .bss 的 0, P 律
     * clamp(kp*err, 0) 恒等于 0: 环"使能"了却永远不出力(日志: err 涨到
     * 20~85 rad 而 spdref 恒为 0, 电机只剩速度环自己守 0 转速 -> 漂走 -> 过流)。
     * 这里补一道自愈, 让"忘了 init"不再能悄悄地废掉位置环。 */
    if ((m->speed_limit <= 0.0f) || (m->pos_kp <= 0.0f)) {
        app_motion_init(m);
    }

    if (enable && !m->enable) {
        m->unwrap_valid = false;                      /* 重新建立展开基准 */
        m->pos_fb       = motion_dir() * motion_unwrap(m, motion_wrap_2pi(mech_rad));
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

    m->pos_fb = motion_dir() * motion_unwrap(m, motion_wrap_2pi(mech_rad));

    /* P 控制: 位置误差 -> 速度给定, 并限速(同时限制最大跟踪速度)
     *
     * 【死区】|误差| <= APP_MOTION_DEADBAND_RAD 时直接给 0 速度。
     * 编码器 14bit, 1 count ≈ 0.38 mrad, 位置环在目标附近会一直追最后几个
     * count —— 表现为"到位后轻微嗡嗡"。4 mrad(0.23°)的死区让它真正停住,
     * 死区外仍是纯 P, 不引入额外滞后。 */
    err = m->pos_ref - m->pos_fb;
    if ((err > -APP_MOTION_DEADBAND_RAD) && (err < APP_MOTION_DEADBAND_RAD)) {
        m->speed_ref = 0.0f;
    } else {
        m->speed_ref = motion_clamp(m->pos_kp * err, m->speed_limit);
        /* 【最小逼近速度】纯 P 律在接近目标时输出趋近 0, 而电机几 RPM 以下转不动,
         * 最后一段就变成"爬"。误差还在死区外, 就给一个最小速度把它顶过去;
         * 进了死区上面那个分支会干净地停住。见 app.h 的说明。 */
        if ((APP_MOTION_APPROACH_MIN_RPM < m->speed_limit) &&
            (m->speed_ref > -APP_MOTION_APPROACH_MIN_RPM) &&
            (m->speed_ref < APP_MOTION_APPROACH_MIN_RPM)) {
            m->speed_ref = (err > 0.0f) ? APP_MOTION_APPROACH_MIN_RPM
                                        : -APP_MOTION_APPROACH_MIN_RPM;
        }
    }

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
