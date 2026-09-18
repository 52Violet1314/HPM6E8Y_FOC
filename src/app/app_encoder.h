/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 编码器封装模块
 *
 *   把 MT6701 编码器的角度获取、方向修正、速度估算收敛到一个模块。
 *   应用层不再直接调 bsp_sei_get_latest() + mt6701_angle_to_rad() +
 *   foc_speed_est_update() 这一串, 而只面对:
 *     encoder_get_mech_rad()  /  encoder_get_mech_rpm()  /  encoder_get_elec_rad()
 *
 *   速度估算复用 FOC 的 foc_speed_est_t (角度差分 + 一阶低通)。
 */
#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "foc.h"            /* foc_speed_est_t / FOC_ELEC_ANGLE_OFFSET */
#include "mt6701.h"         /* mt6701_frame_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    foc_speed_est_t speed_est;      /* 机械速度估算器 */
    mt6701_frame_t  frame;          /* 最近一帧编码器数据 */
    uint16_t        raw_angle;      /* 最近原始角度 (14bit) */
    float           mech_rad;       /* 最近机械角度 (rad) */
    float           mech_rpm;       /* 最近机械转速 (RPM) */
    float           elec_rad;       /* 最近电角度 (rad) */
    uint32_t        frame_count;    /* 帧计数 */
    uint32_t        error_count;    /* 错误计数 */
} app_encoder_t;

/**
 * @brief  初始化编码器封装 (速度估算器 + 方向参数)
 * @param  enc        实例
 * @param  ts         更新周期 (s)
 * @param  lpf_alpha  速度低通系数 (建议 0.05~0.3)
 */
void app_encoder_init(app_encoder_t *enc, float ts, float lpf_alpha);

/**
 * @brief  拉取最新编码器数据并更新速度/角度 (建议按控制周期调用)
 * @param  enc  实例
 * @return true = 自上次调用以来收到新帧
 */
bool app_encoder_update(app_encoder_t *enc);

/** @return 机械角度 (rad) */
float app_encoder_get_mech_rad(const app_encoder_t *enc);

/** @return 机械转速 (RPM) */
float app_encoder_get_mech_rpm(const app_encoder_t *enc);

/** @return 电角度 (rad, 含极对数与安装偏置) */
float app_encoder_get_elec_rad(const app_encoder_t *enc);

/** @return 原始角度 (14bit) */
uint16_t app_encoder_get_raw_angle(const app_encoder_t *enc);

/** @return 帧计数 */
uint32_t app_encoder_get_frame_count(const app_encoder_t *enc);

/** @return 错误计数 */
uint32_t app_encoder_get_error_count(const app_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_H */
