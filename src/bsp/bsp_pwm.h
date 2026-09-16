/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 三相互补 PWM (HPM6E8Y / hpm6e00YZ)
 *   - PWM0, 3 对互补通道 + 死区, 中心对齐
 *   - 引脚: UH/UL=PD00/PD01, VH/VL=PD02/PD03, WH/WL=PD04/PD05
 *   - TRGO 经 TRGM 触发 ADC 采样
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PWM 载波频率 / 死区 (tick)
 * BSP_PWM_FREQUENCY_HZ  : 目标载波频率, FOC 常用 10~20kHz, 这里 20kHz。
 *                         对应 STM32 里 TIM_Period 的倒数。
 * BSP_PWM_DEAD_TIME_TICK: 死区时间(单位=计数 tick), 对应 STM32 的 BDTR.DTG。
 *                         实际死区时间 = tick / PWM 时钟频率, 需按栅极驱动手册取值。 */
#define BSP_PWM_FREQUENCY_HZ     (20000U)
#define BSP_PWM_DEAD_TIME_TICK   (100U)

/* 相选择枚举, 对应 STM32 里区分 TIM_CH1/CH2/CH3 的 channel 参数 */
typedef enum {
    bsp_pwm_phase_u = 0,    /* U 相 */
    bsp_pwm_phase_v,        /* V 相 */
    bsp_pwm_phase_w,        /* W 相 */
} bsp_pwm_phase_t;

/**
 * @brief  初始化 PWM0 (3 对互补 + 死区 + 中心对齐 + TRGO 触发 ADC)
 * @note   初始化完成后 6 路输出处于关闭状态(全低), 需再调
 *         bsp_pwm_enable_output(true) 才会输出波形。
 *         对照 STM32: 相当于 TIM1 的 RCC/GPIO/TIM_TimeBase/TIM_OC/
 *         TIM_BDTR/TRGO 一整串初始化的合成。
 */
void bsp_pwm_init(void);

/**
 * @brief  使能 / 关闭 6 路 PWM 输出
 * @param  enable  true=正常输出; false=强制全低(电机自由停车)
 * @note   对照 STM32: enable=true 相当于置 BDTR.MOE;
 *         enable=false 相当于清 BDTR.MOE, OCx 立即变无效电平。
 */
void bsp_pwm_enable_output(bool enable);

/**
 * @brief  设置单相占空比
 * @param  phase  相: bsp_pwm_phase_u / _v / _w
 * @param  duty   占空比, 0.0f ~ 1.0f (0.5f = 50%)
 * @note   值会在下一次载波 reload 点生效(影子寄存器机制)。
 */
void bsp_pwm_set_duty(bsp_pwm_phase_t phase, float duty);

/**
 * @brief  一次设置三相占空比 (FOC/SVPWM 常用入口)
 * @param  du,dv,dw  三相占空比, 各 0.0f ~ 1.0f
 */
void bsp_pwm_set_duty_uvw(float du, float dv, float dw);

/**
 * @brief  获取 PWM 计数重载值 (载波周期 tick 数)
 * @return reload 值; 对照 STM32: 相当于 TIMx->ARR
 */
uint32_t bsp_pwm_get_reload(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
