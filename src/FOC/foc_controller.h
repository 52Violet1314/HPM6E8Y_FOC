/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 统一控制器 (编排层)
 *
 *   把 foc.c (电流环) 与 foc_openloop.c (开环电压矢量) 封装成一个统一的
 *   控制器实例, 应用层只面对一个 run() 接口, 不再直接操作二者的内部字段。
 *   内部按当前模式把三相电流/电角度/母线电压送入对应算法, SVPWM 在各算法
 *   内部 (foc_math.h 的 foc_svpwm) 完成, 最终统一输出三相占空比。
 *
 *   依赖方向: 本模块依赖 foc.h / foc_openloop.h (同层算法), 不依赖 bsp。
 *   输出为占空比 [0,1], 由上层 (app) 写入 PWM (bsp_pwm_set_duty_uvw)。
 */
#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "foc.h"
#include "foc_openloop.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 控制器运行模式 */
typedef enum {
    foc_ctrl_mode_openloop = 0,     /* 开环电压矢量 (默认, 相参数未知时用) */
    foc_ctrl_mode_current,          /* FOC 电流环 */
} foc_ctrl_mode_t;

/* 统一控制器 */
typedef struct {
    foc_ctrl_mode_t mode;           /* 当前运行模式 */

    foc_t           foc;            /* 电流环控制器 */
    foc_openloop_t  openloop;       /* 开环驱动器 */

    /* 统一输出 (占空比 [0,1]) */
    float duty_u;
    float duty_v;
    float duty_w;

    /* 运行上下文 */
    float elec_angle;               /* 最近一次电角度 (rad) */
    float vbus;                     /* 最近一次母线电压 (V) */
    float ts;                       /* 控制周期 (s) */
    bool  enabled;                  /* 总使能 */
} foc_controller_t;

/**
 * @brief  初始化统一控制器 (默认开环模式)
 * @param  ctrl  控制器实例
 * @param  ts    控制周期 (s), 例如 0.001 = 1kHz
 */
void foc_controller_init(foc_controller_t *ctrl, float ts);

/**
 * @brief  切换运行模式 (开环 / 电流环)
 * @note   切换时会把目标输出复位到零矢量, 避免跳变。
 */
void foc_controller_set_mode(foc_controller_t *ctrl, foc_ctrl_mode_t mode);

/** @return 当前模式 */
foc_ctrl_mode_t foc_controller_get_mode(const foc_controller_t *ctrl);

/**
 * @brief  每周期告知【实测控制周期】, 作为内部算法的时间步长
 * @param  ctrl  控制器
 * @param  dt    实测周期 (s), 必须 > 0
 * @note   开环电角度是积分出来的 (elec_angle += elec_speed * dt)。若用名义
 *         的固定 dt, 一旦主循环因日志/串口写被拉长, 磁场实际频率就会低于
 *         指令值并随负载抖动 —— 这是开环失步的主要诱因之一。
 *         调用者应在每个控制周期用 mcycle 实测 dt 后调用本函数。
 */
void foc_controller_set_dt(foc_controller_t *ctrl, float dt);

/**
 * @brief  总使能 / 关闭 (关闭时三相 50% 零矢量)
 */
void foc_controller_enable(foc_controller_t *ctrl, bool enable);

/* ---- 开环模式参数 ---- */
void foc_controller_set_voltage(foc_controller_t *ctrl, float volts);
void foc_controller_set_speed_rpm(foc_controller_t *ctrl, float mech_rpm);
void foc_controller_set_speed_elec(foc_controller_t *ctrl, float elec_rad_s);
void foc_controller_set_current_limit(foc_controller_t *ctrl, float amps);
void foc_controller_set_align_angle(foc_controller_t *ctrl, float elec_rad);

/**
 * @brief  用【外部电角度】驱动开环电压矢量 (编码器换向)
 * @param  elec_rad  本周期要用的电角度 (rad), 例如编码器电角度 + 超前角
 * @note   每控制周期调用一次。内部会把 elec_speed_target 置 0, 所以开环的
 *         角度积分不再推进 —— 角度完全由外部决定, 而电压幅值的斜坡与
 *         过流折叠保护仍然照常工作。
 *         注意: 若直接把转子电角度填进来, 电压矢量会落在转子 d 轴上,
 *         转矩为零; 必须【超前】一个角度 (通常 90°) 才能出力矩。
 */
void foc_controller_set_external_angle(foc_controller_t *ctrl, float elec_rad);

/* ---- 电流环模式参数 ---- */
void foc_controller_set_current_ref(foc_controller_t *ctrl, float id_ref, float iq_ref);
void foc_controller_set_gains(foc_controller_t *ctrl, float kp, float ki);

/**
 * @brief  执行一次控制 (建议按 ts 周期调用)
 * @param  ctrl       控制器
 * @param  ia,ib,ic   三相电流 (A)
 * @param  elec_angle 电角度 (rad)
 * @param  vbus       母线电压 (V)
 * @note   执行后通过 foc_controller_get_duty() 取占空比写入 PWM。
 */
void foc_controller_run(foc_controller_t *ctrl,
                        float ia, float ib, float ic,
                        float elec_angle, float vbus);

/** @brief 取三相占空比 [0,1] */
void foc_controller_get_duty(const foc_controller_t *ctrl,
                             float *du, float *dv, float *dw);

/** @return 开环保护标志位 (电流环模式下为 0) */
uint32_t foc_controller_get_trip(const foc_controller_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROLLER_H */
