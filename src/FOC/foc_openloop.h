/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 开环电压矢量驱动 (V/F 式)
 *
 *   用途: 电机相电阻/电感未知、或电流环还没调好时, 先用开环把电机转起来,
 *         验证: PWM -> 栅极驱动 -> 功率级 -> 电机 接线是否正常, 转向是否正确。
 *
 *   原理:
 *     电角度 θ 由内部按目标电角速度积分得到 (不需要编码器):
 *         θ += ω * dt
 *     电压矢量固定跟随 θ 旋转 (相当于 ud = V, uq = 0):
 *         uα = V * cosθ
 *         uβ = V * sinθ
 *     再经 SVPWM 变成三相占空比。
 *     电机转子会被这个旋转磁场"拖着走", 所以叫开环(无位置反馈)驱动。
 *
 *   保护:
 *     - 母线欠压: vbus < vbus_min 时封锁输出
 *     - 过流折叠: 相电流超过 current_limit 时按比例压低输出电压
 *       (相电阻未知时这是必要的兜底, 避免堵转烧管)
 *
 *   注意: 开环启动瞬间转子与磁场不同步, 可能抖动/反转一小步属正常;
 *         电压应从很小开始, 慢慢加, 加到能稳定跟随为止。
 */
#ifndef FOC_OPENLOOP_H
#define FOC_OPENLOOP_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_config.h"
#include "foc_math.h"

/* 默认参数在 foc_config.h 里集中配置; 这里用 #ifndef 兜底,
 * 保证即使配置文件缺失/被改动, 本模块也能独立编译。 */
#ifndef FOC_OPENLOOP_VOLTAGE_V
#define FOC_OPENLOOP_VOLTAGE_V          (1.0f)
#endif
#ifndef FOC_OPENLOOP_VOLTAGE_SLEW
#define FOC_OPENLOOP_VOLTAGE_SLEW       (2.0f)
#endif
#ifndef FOC_OPENLOOP_ACCEL_RAD_S2
#define FOC_OPENLOOP_ACCEL_RAD_S2       (100.0f)
#endif
#ifndef FOC_OPENLOOP_CURRENT_LIMIT_A
#define FOC_OPENLOOP_CURRENT_LIMIT_A    (0.5f)
#endif
#ifndef FOC_OPENLOOP_VBUS_MIN_V
#define FOC_OPENLOOP_VBUS_MIN_V         (5.0f)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 保护触发标志 */
#define FOC_OL_TRIP_VBUS_LOW    (1U << 0)   /* 母线欠压 */
#define FOC_OL_TRIP_OVERCURRENT (1U << 1)   /* 过流折叠 */

typedef struct {
    /* --- 运行状态 --- */
    float elec_angle;           /* 当前电角度 (rad, [0,2π)) */
    float elec_speed;           /* 当前电角速度 (rad/s) */
    float voltage;              /* 当前电压矢量幅值 (V) */
    float duty_u, duty_v, duty_w;
    bool  enabled;              /* 用户使能 */
    bool  output_on;            /* 实际是否有输出(经保护判断后) */
    uint32_t trip;              /* 保护标志位 */

    /* --- 目标 / 参数 --- */
    float elec_speed_target;    /* 目标电角速度 (rad/s) */
    float elec_accel;           /* 电角加速度 (rad/s^2), 决定升速快慢 */
    float voltage_target;       /* 目标电压幅值 (V) */
    float voltage_slew;         /* 电压上升速率 (V/s) */
    float current_limit;        /* 相电流限幅 (A), <=0 表示不启用 */
    float vbus_min;             /* 母线欠压阈值 (V) */
    float ts;                   /* 控制周期 (s) */
} foc_openloop_t;

/**
 * @brief  初始化开环驱动器 (参数取 foc_config.h 默认值)
 * @param  ol  实例
 * @param  ts  调用 update 的周期 (s), 例如 0.001 表示 1kHz
 */
void foc_openloop_init(foc_openloop_t *ol, float ts);

/** @brief 使能 / 关闭开环输出 (关闭时三相 50%, 电机自由停车) */
void foc_openloop_enable(foc_openloop_t *ol, bool enable);

/** @brief 设置目标电角速度 (rad/s) */
void foc_openloop_set_speed_elec(foc_openloop_t *ol, float elec_speed_rad_s);

/** @brief 设置目标电角频率 (Hz) */
void foc_openloop_set_speed_hz(foc_openloop_t *ol, float elec_hz);

/** @brief 设置目标机械转速 (RPM), 使用 FOC_MOTOR_POLE_PAIRS 换算 */
void foc_openloop_set_speed_rpm(foc_openloop_t *ol, float mech_rpm);

/** @brief 设置目标电压矢量幅值 (V) */
void foc_openloop_set_voltage(foc_openloop_t *ol, float volts);

/** @brief 设置相电流限幅 (A), <=0 关闭限流 */
void foc_openloop_set_current_limit(foc_openloop_t *ol, float amps);

/** @brief 清除保护标志并恢复输出 (排除故障后调用) */
void foc_openloop_clear_trip(foc_openloop_t *ol);

/**
 * @brief  执行一次开环更新 (建议 1kHz 调用)
 * @param  ol        实例
 * @param  dt        距上次调用的时间 (s), <=0 时用初始化的 ts
 * @param  vbus      母线电压 (V)
 * @param  ia,ib,ic  三相电流 (A), 用于过流保护; 无采样可传 0
 * @note   执行后用 foc_openloop_get_duty() 取占空比写入 PWM。
 */
void foc_openloop_update(foc_openloop_t *ol, float dt, float vbus,
                         float ia, float ib, float ic);

/** @brief 取三相占空比 [0,1] */
void foc_openloop_get_duty(const foc_openloop_t *ol, float *du, float *dv, float *dw);

#ifdef __cplusplus
}
#endif

#endif /* FOC_OPENLOOP_H */
