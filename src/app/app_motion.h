/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 位置环 (串在速度环外面)
 *
 * 【分层】
 *     位置环(本模块)    位置误差 --P--> 速度给定 [RPM]
 *     速度环(app.c 的 APP_FOC_SPEED_LOOP_ENABLE)  速度误差 --PI--> iq [A]
 *     电流环(ADC 中断 20kHz)                          -> ud/uq
 *
 *   【为什么位置环不能直接给 iq】iq -> 转矩 -> 加速度 -> 速度 -> 位置 是"双积分",
 *   只加 P 必然振荡(没有阻尼)。所以必须串一个速度环做内环并提供阻尼。
 *   也正因如此, 用本模块做位置控制时 APP_FOC_SPEED_LOOP_ENABLE 必须为 1 ——
 *   否则速度给定只会被当成"软限速上限", 位置环等于没有内环, 会一直冲过目标。
 *
 * 【编码器只给单圈角度, 所以本模块负责两件事】
 *   1) 多圈展开: 0~2π 锯齿 -> 连续角度(过零补 ±2π), 否则跨 0/2π 时会跳变;
 *   2) 方向统一: 实测 iq>0 时编码器机械角减小(FOC_ENCODER_DIRECTION=-1),
 *      统一成"正 iq -> 位置/速度增大"。若上电表现为正反馈(一使能就冲),
 *      把 APP_MOTION_DIR_SIGN 改成 +1 即可(只此一处)。
 */
#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pos_ref;          /* 目标机械角 (rad, 连续多圈) */
    float pos_fb;           /* 反馈机械角 (rad, 连续多圈) */
    float speed_ref;        /* 位置环输出 = 速度给定 (RPM), 交给速度环 */
    float pos_kp;           /* (rad 误差) -> RPM */
    float speed_limit;      /* 速度给定限幅 (RPM) */

    float turns_rad;        /* 多圈展开累计角 (rad) */
    float prev_mech_rad;    /* 上一拍单圈角, 用于展开 */
    bool  unwrap_valid;
    bool  enable;
} app_motion_t;

/** @brief 初始化 (增益/限幅取 app.h 的 APP_MOTION_* 宏) */
void app_motion_init(app_motion_t *m);

/**
 * @brief 使能/关闭。
 * @note  使能瞬间把"当前位置"设为目标(原地保持): 否则目标与实际差得远时
 *        (比如目标 0、实际 5rad), 一使能就是满速冲过去。
 */
void app_motion_enable(app_motion_t *m, bool enable, float mech_rad);

/** @brief 设置目标机械角 (rad, 连续多圈) */
void app_motion_set_target(app_motion_t *m, float pos_rad);

/** @brief 直接改位置环增益 (rad 误差 -> RPM), 供调试 */
void app_motion_set_gain(app_motion_t *m, float kp);

/**
 * @brief 按控制周期调用: 位置误差 -> 速度给定
 * @param  m         实例
 * @param  mech_rad  单圈机械角 (rad, 0~2π, 直接来自编码器)
 * @return 速度给定 (RPM); 未使能时返回 0
 */
float app_motion_run(app_motion_t *m, float mech_rad);

/** @return 最近展开后的连续位置 (rad) */
float app_motion_get_pos(const app_motion_t *m);

/** @return 最近的位置误差 (rad, 目标-反馈) */
float app_motion_get_err(const app_motion_t *m);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTION_H */
