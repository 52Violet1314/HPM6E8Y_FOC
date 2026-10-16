/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 编码器封装实现
 */

#include "app_encoder.h"

#include <string.h>

#include "app.h"            /* APP_ENC_USE_BUILTIN_LUT */
#include "app_enc_lut.h"    /* 离线标定的 3200 组 raw -> 电角度表 */
#include "bsp_mt6701.h"

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
    bool is_new;

    if (enc == NULL) {
        return false;
    }

    is_new = bsp_mt6701_get_latest(&enc->frame);
    enc->raw_angle = enc->frame.angle;
    enc->frame_count = bsp_mt6701_get_frame_count();
    enc->error_count = bsp_mt6701_get_error_count();

    /* 机械角度 = 原始角度 -> rad, 再乘方向系数 */
    enc->mech_rad = FOC_ENCODER_DIRECTION * mt6701_angle_to_rad(enc->raw_angle);

    /* 速度估算 (rad/s) -> RPM */
    float speed_rad_s = foc_speed_est_update(&enc->speed_est, enc->mech_rad);
    enc->mech_rpm = speed_rad_s * (60.0f / FOC_TWO_PI);

    /* 电角度 = 机械角度 * 极对数 + 安装偏置 (偏置可被校准存档覆盖) */
#if APP_ENC_USE_BUILTIN_LUT
    /* 【查表法】电角度直接由离线标定的表给出。
     * 表值是"编码器读到这个 raw 时, 能让转子停在这个位置的磁场电角度",
     * 因此它已经包含了: 极对数换算、安装偏置、以及编码器自身的周期误差
     * (实测按电周期重复, 幅度约 ±300 mrad)。
     * 【方向已含在表里】本安装下 raw 增大时电角度是【下降】的, 不要再乘
     * FOC_ENCODER_DIRECTION —— 那个系数只作用于 mech_rad (速度估算用)。 */
    enc->elec_rad = app_enc_lut_elec_rad(enc->raw_angle);
#else
    enc->elec_rad = foc_mech_to_elec(enc->mech_rad, s_elec_offset_rad);
#endif

    return is_new;
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

uint16_t app_encoder_get_raw_angle(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->raw_angle : 0;
}

uint32_t app_encoder_get_frame_count(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->frame_count : 0;
}

uint32_t app_encoder_get_error_count(const app_encoder_t *enc)
{
    return (enc != NULL) ? enc->error_count : 0;
}
