/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 磁场定向控制 (电流环 + SVPWM)
 *
 *   控制链路: 三相电流 --Clarke--> iα/iβ --Park--> id/iq
 *              id/iq 与参考值做 PI --反Park--> uα/uβ --SVPWM--> 三相占空比
 *
 *   输出为占空比 [0,1], 由调用者写入 PWM (见 bsp_pwm_set_duty_uvw),
 *   因此本模块不依赖任何硬件, 便于复用与测试。
 *
 * 对照 STM32: 相当于把 MC_Lib 的 FOC 电流环 + SVPWM 单独抽出来,
 *             输入是采样电流和电角度, 输出是三相占空比。
 */
#ifndef FOC_H
#define FOC_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_config.h"
#include "foc_math.h"
#include "pid.h"        /* 通用 PID: 电流环/速度环/位置环共用同一个实现 */

#ifdef __cplusplus
extern "C" {
#endif

/* 电流环用的 PI 就是 pid.h 里的 PID_Controller_t (原来这里有个 foc_pi_t, 已合并:
 * 它的"积分限幅=输出限幅、饱和即停积分"与速度环/位置环的写法不一致,
 * 三处规则不同会让同一组 Kp/Ki 在不同环上手感不一样, 调参经验没法复用)。 */

/* ---- FOC 控制器 ---- */
typedef struct {
    /* 参考值 */
    float id_ref;
    float iq_ref;

    /* 反馈值 */
    float id;
    float iq;

    /* 相电流一阶低通的状态 (系数见 FOC_CURRENT_LPF_ALPHA)
     * 放在结构体里而不是 ISR 局部静态: 控制器重新 init 时会随结构体一起清零。 */
    float ia_f;
    float ib_f;
    float ic_f;

    /* 电流环 PI (通用 PID_Controller_t) */
    PID_Controller_t id_pi;
    PID_Controller_t iq_pi;

    /* 输出 */
    float ud;
    float uq;
    float duty_u;
    float duty_v;
    float duty_w;

    /* 运行上下文 */
    float elec_angle;       /* 电角度 (rad) */
    float vdc;              /* 母线电压 (V) */
    float ts;               /* 控制周期 (s) */
    float v_limit;          /* SVPWM 线性区电压限幅 (V) */
    bool  enable;           /* false = 输出零矢量 (三相 50%) */
} foc_t;

/**
 * @brief  初始化 FOC 控制器 (使用 foc_config.h 默认参数)
 * @param  foc  控制器实例
 */
void foc_init(foc_t *foc);

/**
 * @brief  复位 PI 积分与输出 (重新使能前调用)
 */
void foc_reset(foc_t *foc);

/**
 * @brief  设置 d/q 电流参考值 (A)
 * @note   id_ref 一般给 0 (表贴式 PMSM), iq_ref 决定转矩。
 */
void foc_set_current_ref(foc_t *foc, float id_ref, float iq_ref);

/**
 * @brief  设置电流环 PI 参数
 */
void foc_set_gains(foc_t *foc, float kp, float ki);

/**
 * @brief  使能 / 关闭 FOC 输出
 * @param  enable  false 时输出三相 50% (零电压), 电机自由停车
 */
void foc_enable(foc_t *foc, bool enable);

/**
 * @brief  执行一次电流环
 * @param  foc         控制器
 * @param  ia,ib,ic    三相电流 (A), 方向以流入电机为正
 * @param  elec_angle  电角度 (rad)
 * @param  vdc         母线电压 (V)
 * @note   执行后通过 foc_get_duty() 取三相占空比。
 */
void foc_update(foc_t *foc, float ia, float ib, float ic, float elec_angle, float vdc);

/**
 * @brief  取三相占空比 [0,1]
 */
void foc_get_duty(const foc_t *foc, float *du, float *dv, float *dw);

/**
 * @brief  开环电压模式 (没有电流采样时用来验证 PWM/驱动/编码器)
 * @param  ud,uq       期望 d/q 电压 (V)
 * @param  du,dv,dw    输出占空比
 */
void foc_open_loop(float ud, float uq, float elec_angle, float vdc,
                   float *du, float *dv, float *dw);

/** @brief 机械角度 -> 电角度 (rad), 含极对数和安装偏置 */
static inline float foc_mech_to_elec(float mech_angle, float offset)
{
    return foc_normalize_angle(mech_angle * FOC_MOTOR_POLE_PAIRS + offset);
}

/* ---- 速度估算 (基于编码器角度差分 + 一阶低通) ---- */
typedef struct {
    float prev_angle;
    float speed;        /* 机械角速度 (rad/s) */
    float ts;           /* 更新周期 (s) */
    float lpf_alpha;    /* 低通系数 0~1, 越小越平滑 */
    bool  started;
} foc_speed_est_t;

/**
 * @brief  初始化速度估算器
 * @param  ts         更新周期 (s)
 * @param  lpf_alpha  低通系数 (建议 0.05 ~ 0.3)
 */
void  foc_speed_est_init(foc_speed_est_t *est, float ts, float lpf_alpha);

/**
 * @brief  输入机械角度 (rad), 返回滤波后的机械角速度 (rad/s)
 */
float foc_speed_est_update(foc_speed_est_t *est, float mech_angle);

/**
 * @brief  更新速度估算用的时间步长 (每拍调用, 传【实测】周期)
 * @param  ts  本次采样的实际间隔 (s), 必须 > 0
 * @note   speed = Δangle / ts。ts 是名义值而实际周期变长时, 算出的转速会
 *         按比例偏大 (1ms 的名义值 + 5ms 的实际周期 = 5 倍假转速)。上 RTOS 后
 *         控制任务的周期会被显示/日志任务挤长, 所以这个 ts 必须每拍刷新。
 *         取值合理性由调用者负责 (本函数只挡 <= 0 的非法值)。
 */
void  foc_speed_est_set_ts(foc_speed_est_t *est, float ts);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
