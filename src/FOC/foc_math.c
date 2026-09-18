/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: sin/cos 实现
 *
 *   FOC_USE_HPM_MATH = 1 时使用 HPM Math 库 (hpm_math.h) 的
 *   "查表 + 二次插值" 快速三角函数:
 *     - hpm_math_fast_trig_init()        上电生成 sin 表
 *     - hpm_math_fast_sincosf_quadratic() 一次求出 sin/cos
 *   相比 libm 的 sinf/cosf, 在 20kHz 电流环里可显著降低 CPU 占用。
 *
 *   查表需要一块用户提供的缓冲区, 这里用静态数组 (FOC_TRIG_TABLE_SIZE 个 float)。
 */

#include "foc_math.h"

#if FOC_USE_HPM_MATH

#include "hpm_math.h"

static float s_foc_sin_table[FOC_TRIG_TABLE_SIZE];
static hpm_math_fast_trig_context_t s_foc_trig_ctx;

void foc_math_init(void)
{
    (void)hpm_math_fast_trig_init(&s_foc_trig_ctx, FOC_TRIG_TABLE_SIZE, s_foc_sin_table);
}

void foc_sin_cos(float angle, float *sin_v, float *cos_v)
{
    hpm_math_fast_sincosf_quadratic(&s_foc_trig_ctx, angle, sin_v, cos_v);
}

#endif /* FOC_USE_HPM_MATH */
