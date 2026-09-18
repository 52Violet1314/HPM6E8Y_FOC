/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 编码器封装实现
 */

#include "app_encoder.h"

#include <string.h>

#include "bsp_sei.h"

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

    is_new = bsp_sei_get_latest(&enc->frame);
    enc->raw_angle = enc->frame.angle;
    enc->frame_count = bsp_sei_get_frame_count();
    enc->error_count = bsp_sei_get_error_count();

    /* 机械角度 = 原始角度 -> rad, 再乘方向系数 */
    enc->mech_rad = FOC_ENCODER_DIRECTION * mt6701_angle_to_rad(enc->raw_angle);

    /* 速度估算 (rad/s) -> RPM */
    float speed_rad_s = foc_speed_est_update(&enc->speed_est, enc->mech_rad);
    enc->mech_rpm = speed_rad_s * (60.0f / FOC_TWO_PI);

    /* 电角度 = 机械角度 * 极对数 + 安装偏置 */
    enc->elec_rad = foc_mech_to_elec(enc->mech_rad, FOC_ELEC_ANGLE_OFFSET);

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
