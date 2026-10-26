/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 一阶线性自抗扰控制器 (LADRC) 实现
 *
 *   算法/离散化/参数含义见 LADRC.h。这里只强调三个实现要点:
 *     1) ESO 的模型项必须用【限幅后】的 u —— 否则执行器饱和时观测器会按
 *        "不存在的输入"推算, z2 反而把饱和补偿回去, 形成闭环发散;
 *     2) z1/z2 用前向欧拉、步长 ts 显式积分, ts 每拍可更新 (RTOS 下主循环
 *        周期会抖, 写死 1ms 会让观测器带宽随负载漂移);
 *     3) 控制律用上一拍的 z1/z2, 再更新观测器 —— 单步内不存在代数环。
 */
#include "LADRC.h"

static float ladrc_clamp_sym(float v, float lim)
{
    if (lim <= 0.0f) {
        return v;
    }
    if (v > lim) {
        return lim;
    }
    if (v < -lim) {
        return -lim;
    }
    return v;
}

void LADRC_Init(LADRC_typedef *l, float b0, float wc, float wo,
                float ts, float u_max)
{
    if (l == NULL) {
        return;
    }

    l->ts    = (ts > 0.0f) ? ts : 0.001f;
    l->u_max = (u_max > 0.0f) ? u_max : 0.0f;

    LADRC_SetB0(l, b0);
    LADRC_SetBandwidth(l, wc, wo);
    LADRC_Reset(l);
}

void LADRC_Reset(LADRC_typedef *l)
{
    if (l == NULL) {
        return;
    }
    l->z1  = 0.0f;
    l->z2  = 0.0f;
    l->u   = 0.0f;
    l->y   = 0.0f;
    l->ref = 0.0f;
}

void LADRC_SetTs(LADRC_typedef *l, float ts)
{
    if ((l != NULL) && (ts > 0.0f)) {
        l->ts = ts;
    }
}

void LADRC_SetBandwidth(LADRC_typedef *l, float wc, float wo)
{
    if (l == NULL) {
        return;
    }

    if (wc < 0.0f) {
        wc = 0.0f;
    }
    if (wo < 0.0f) {
        wo = 0.0f;
    }

    l->Kp     = wc;
    l->beta_1 = 2.0f * wo;
    l->beta_2 = wo * wo;
}

void LADRC_SetB0(LADRC_typedef *l, float b0)
{
    if (l == NULL) {
        return;
    }

    l->b0     = b0;
    /* b0 = 0 时输出恒为 0 (inv_b0 = 0), 比除零安全; 方向允许为负,
     * 但本工程会在应用层把 y/ref 都折算到"正增益"坐标系, 见 app.c。 */
    l->inv_b0 = (b0 != 0.0f) ? (1.0f / b0) : 0.0f;
}

float LADRC_Run(LADRC_typedef *l, float ref, float y)
{
    float err;
    float u0;
    float eo;

    if (l == NULL) {
        return 0.0f;
    }

    l->ref = ref;
    l->y   = y;

    /* ---- 1) 误差反馈 + 扰动补偿 ----
     * u0 = wc·(ref - z1)          (z1 而不是 y: 用观测器的"无噪"状态做反馈)
     * u  = (u0 - z2) / b0         (z2 把负载/摩擦/反电动势/b0 误差一次补掉) */
    err = ref - l->z1;
    u0  = l->Kp * err;
    l->u = ladrc_clamp_sym((u0 - l->z2) * l->inv_b0, l->u_max);

    /* ---- 2) 扩张状态观测器 (LESO) ----
     * 用【限幅后的 l->u】作为模型输入, 用实测 y 校正。
     * 前向欧拉: z += ts·z' */
    eo = y - l->z1;
    {
        float z1_dot = l->z2 + (l->b0 * l->u) + (l->beta_1 * eo);
        float z2_dot = l->beta_2 * eo;

        l->z1 += l->ts * z1_dot;
        l->z2 += l->ts * z2_dot;
    }

    return l->u;
}
