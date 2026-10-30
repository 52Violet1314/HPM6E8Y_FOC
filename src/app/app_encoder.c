/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 编码器封装实现 (锐鹰 SRMA35, 17bit 单圈 + 16bit 多圈, RS485)
 *
 * 【LUT 为什么关掉了】主线的 APP_ENC_USE_BUILTIN_LUT 那张表是按 MT6701(14bit)
 * 的安装零位离线标定的, 换 SRMA35 后零位和分辨率都变了, 整张表作废。
 * 现在用 elec = mech × 极对数 + offset, 其中 offset 由静态扫描标定并已存 Flash
 * (实测 CALCYC 误差 ±0.4%, 残差 spread 142 mrad)。
 * 若要恢复查表换向, 需要用 17bit 索引重标一张 (tools/enc_lut_build.py 的索引也要改)。
 */

#include "app_encoder.h"

#include <string.h>

#include "app.h"            /* APP_ENC_USE_BUILTIN_LUT */
#include "app_enc_lut.h"    /* 离线标定的 raw -> 电角度表 (当前未启用) */
#include "bsp_srma35.h"

static float s_elec_offset_rad = FOC_ELEC_ANGLE_OFFSET;

void app_encoder_init(app_encoder_t *enc, float ts, float lpf_alpha)
{
    if (enc == NULL) {
        return;
    }

    memset(enc, 0, sizeof(*enc));
    foc_speed_est_init(&enc->speed_est, ts, lpf_alpha);
}

bool app_encoder_update(app_encoder_t *enc)
{
    bsp_srma35_frame_t f;
    float speed_rad_s;

    if (enc == NULL) {
        return false;
    }

    if (bsp_srma35_get_latest(&f) && f.valid) {
        /* 只有校验通过的帧才更新角度。丢帧时保持上一帧的 raw ——
         * 校准/换向看到的是"角度冻结", 而不是跳到 0。 */
        enc->raw_angle  = f.abs;
        enc->multi_turn = f.abm;
        enc->sf         = f.sf;
        enc->almc       = f.almc;
        enc->frame      = f;
    }

    enc->frame_count = bsp_srma35_get_frame_count();
    enc->error_count = bsp_srma35_get_error_count();

    /* 机械角度 = raw/一圈 * 2π, 再乘方向系数 */
    enc->mech_rad = FOC_ENCODER_DIRECTION *
                    ((float)enc->raw_angle * (FOC_TWO_PI / (float)APP_ENC_RAW_MAX));

    /* 速度估算 (rad/s) -> RPM */
    speed_rad_s = foc_speed_est_update(&enc->speed_est, enc->mech_rad);
    enc->mech_rpm = speed_rad_s * (60.0f / FOC_TWO_PI);

#if APP_ENC_USE_BUILTIN_LUT
    /* 17bit -> 表用的 14bit 索引。注意: 这张表是 MT6701 的安装标定, 换编码器后
     * 未重标, 只作为"回到旧硬件"时的兼容路径保留。 */
    enc->elec_rad = app_enc_lut_elec_rad((uint16_t)(enc->raw_angle >> 3));
#else
    /* 电角度 = 机械角度 * 极对数 + 安装偏置 (偏置由静态扫描标定) */
    enc->elec_rad = foc_mech_to_elec(enc->mech_rad, s_elec_offset_rad);
#endif

    return (enc->frame_count != 0U);
}

float app_encoder_get_mech_rad(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->mech_rad : 0.0f;
}

float app_encoder_get_mech_rpm(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->mech_rpm : 0.0f;
}

float app_encoder_get_elec_rad(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->elec_rad : 0.0f;
}

void app_encoder_set_elec_offset_rad(float offset_rad)
{
    s_elec_offset_rad = offset_rad;
}

float app_encoder_get_elec_offset_rad(void)
{
    return s_elec_offset_rad;
}

uint32_t app_encoder_get_raw_angle(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->raw_angle : 0U;
}

uint32_t app_encoder_get_frame_count(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->frame_count : 0U;
}

uint32_t app_encoder_get_error_count(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->error_count : 0U;
}
