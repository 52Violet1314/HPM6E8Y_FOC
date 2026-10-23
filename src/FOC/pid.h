/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 通用 PID 控制器
 *
 * 【来源】语义照搬 C:\Users\14420\Desktop\Sensor_FOC\F405_M3508_FOC\FOC\src\PID.c
 *         (PID_Controller_t / PID_Init / PID_Reset / PID_Step),
 *         只做了两件事: 命名统一成本工程的风格, 加一个对称限幅的便捷 init。
 *
 * 【为什么要有这个文件】原来本工程有三套各不相同的"PI":
 *   - 电流环 foc.c 里的 foc_pi_run(): 只有一个 out_limit, 积分限幅被硬编成
 *     ±out_limit; 抗饱和是"饱和就完全不积分";
 *   - 速度环 app.c 里手写的一段: 自己一个 s_spd_integ, 先积分再限幅(更容易
 *     积到天上), 积分限幅又是 IQ_LIMIT;
 *   - 位置环 app_motion.c: 纯 P, 另一套写法。
 *   三处规则不同 -> 同一组 Kp/Ki 在不同环上手感不一样, 调参没法复用经验。
 *   现在统一到这一个实现, 和 F405 那份例程完全一致。
 *
 * 【算法】out = clamp(kp·e + ∫, out_min, out_max), 积分增量 ki·e·dt。
 *   积分采用【条件积分】(抗积分饱和):
 *       - 输出没饱和                        -> 积分
 *       - 输出顶在上限, 而误差 e < 0        -> 仍然积分(误差正在把它往回拉)
 *       - 输出顶在下限, 而误差 e > 0        -> 仍然积分
 *       - 其余情况(饱和且误差还在往外推)    -> 不积分
 *   和"饱和就一律不积分"相比, 多了第二条: 饱和边界上不会卡住, 退出更快。
 *   积分另外单独夹在 [int_min, int_max] —— 和输出限幅解耦, 这是本实现的关键,
 *   参考例程里也是分开给的两个参数。
 */
#ifndef FOC_PID_H
#define FOC_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;               /* 比例增益 */
    float ki;               /* 积分增益 (单位 /s, 内部乘 dt) */
    float dt;               /* 采样周期 (s) —— 存进结构体, 每拍不必再传 */
    float out_min;          /* 输出下限 */
    float out_max;          /* 输出上限 */
    float int_min;          /* 积分下限 */
    float int_max;          /* 积分上限 */
    float integrator;       /* 积分累加器 */
} PID_Controller_t;

/** @brief 完整初始化 (限幅可以不对称) */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float dt,
              float out_min, float out_max,
              float int_min, float int_max);

/** @brief 对称限幅的便捷初始化 (本工程三个环都是对称的) */
void PID_InitSym(PID_Controller_t *pid, float kp, float ki, float dt,
                  float out_limit, float int_limit);

/** @brief 只改增益, 保留 dt/限幅/积分器 (在线调参用) */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki);

/** @brief 清积分器 (过流封锁、重新使能、模式切换时必须调) */
void PID_Reset(PID_Controller_t *pid);

/** @brief 走一拍: 输入误差, 返回输出 */
float PID_Step(PID_Controller_t *pid, float error);

/** @brief 读积分器/输出 (遥测用) */
float PID_GetIntegrator(const PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PID_H */
