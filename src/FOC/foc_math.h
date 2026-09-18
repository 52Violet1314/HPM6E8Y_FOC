/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 基础数学 (Clarke / Park / 反 Park / SVPWM)
 *   - 纯内联函数, 无硬件依赖, 可单独测试
 *   - 采用 "幅值不变" 变换 (2/3 系数), 电流/电压幅值与三相一致
 *
 * 对照 STM32: 相当于把电机库里的 Clarke/Park 变换和 SVPWM 抽成独立的
 *             math 头文件, 不涉及任何寄存器。
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <math.h>
#include <stdint.h>

#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TWO_PI          (6.283185307179586f)
#define FOC_PI              (3.141592653589793f)
#define FOC_SQRT3           (1.732050807568877f)
#define FOC_ONE_BY_SQRT3    (0.5773502691896257f)
#define FOC_TWO_BY_SQRT3    (1.1547005383792515f)

/* ---- 角度工具 ---- */

/** @brief 归一化到 [0, 2π) */
static inline float foc_normalize_angle(float angle)
{
    angle = fmodf(angle, FOC_TWO_PI);
    if (angle < 0.0f) {
        angle += FOC_TWO_PI;
    }
    return angle;
}

/** @brief 归一化到 [-π, π) */
static inline float foc_normalize_angle_pm_pi(float angle)
{
    angle = foc_normalize_angle(angle);
    if (angle >= FOC_PI) {
        angle -= FOC_TWO_PI;
    }
    return angle;
}

/** @brief 限幅 */
static inline float foc_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/** @brief 同时求 sin/cos
 *
 * FOC_USE_HPM_MATH=1: 调用 HPM Math 库的查表 + 二次插值实现 (见 foc_math.c),
 *                     调用前需先执行 foc_math_init();
 * FOC_USE_HPM_MATH=0: 直接使用标准 libm。
 */
#if FOC_USE_HPM_MATH
void foc_math_init(void);
void foc_sin_cos(float angle, float *sin_v, float *cos_v);
#else
#define foc_math_init() ((void)0)
static inline void foc_sin_cos(float angle, float *sin_v, float *cos_v)
{
    *sin_v = sinf(angle);
    *cos_v = cosf(angle);
}
#endif

/* ---- Clarke 变换 (幅值不变) ----
 * 输入三相电流 ia/ib/ic, 输出静止坐标系 iα/iβ。
 *   iα = (2/3) * (ia - ib/2 - ic/2)
 *   iβ = (ib - ic) / √3
 * 在 ia+ib+ic=0 时等价于 iα=ia, iβ=(ia+2*ib)/√3。
 */
static inline void foc_clarke(float ia, float ib, float ic, float *ialpha, float *ibeta)
{
    *ialpha = (2.0f / 3.0f) * (ia - 0.5f * ib - 0.5f * ic);
    *ibeta  = (ib - ic) * FOC_ONE_BY_SQRT3;
}

/* ---- Park 变换 ----
 * 输入 iα/iβ 和电角度 sin/cos, 输出旋转坐标系 id/iq。
 *   id =  iα*cos + iβ*sin
 *   iq = -iα*sin + iβ*cos
 */
static inline void foc_park(float ialpha, float ibeta, float sin_t, float cos_t,
                            float *id, float *iq)
{
    *id =  ialpha * cos_t + ibeta * sin_t;
    *iq = -ialpha * sin_t + ibeta * cos_t;
}

/* ---- 反 Park 变换 ----
 * 输入 ud/uq 和电角度 sin/cos, 输出 uα/uβ。
 */
static inline void foc_inv_park(float ud, float uq, float sin_t, float cos_t,
                                float *ualpha, float *ubeta)
{
    *ualpha = ud * cos_t - uq * sin_t;
    *ubeta  = ud * sin_t + uq * cos_t;
}

/* ---- SVPWM ----
 * 输入 uα/uβ (V) 与母线电压 vdc (V), 输出三相占空比 [0,1]。
 *
 * 实现方式: 先反 Clarke 得到三相参考电压, 再注入零序分量
 *           voff = -(vmax+vmin)/2, 等价于标准七段式 SVPWM,
 *           比查表法更简洁且数值稳定。
 *           占空比 0.5 对应零电压。
 */
static inline void foc_svpwm(float ualpha, float ubeta, float vdc,
                             float *du, float *dv, float *dw)
{
    float va = ualpha;
    float vb = -0.5f * ualpha + 0.5f * FOC_SQRT3 * ubeta;
    float vc = -0.5f * ualpha - 0.5f * FOC_SQRT3 * ubeta;
    float vmax = fmaxf(va, fmaxf(vb, vc));
    float vmin = fminf(va, fminf(vb, vc));
    float voff = -0.5f * (vmax + vmin);
    float inv_vdc;

    if (vdc < 1.0f) {
        vdc = 1.0f;     /* 防止除零 */
    }
    inv_vdc = 1.0f / vdc;

    *du = foc_clampf((va + voff) * inv_vdc + 0.5f, 0.0f, 1.0f);
    *dv = foc_clampf((vb + voff) * inv_vdc + 0.5f, 0.0f, 1.0f);
    *dw = foc_clampf((vc + voff) * inv_vdc + 0.5f, 0.0f, 1.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
