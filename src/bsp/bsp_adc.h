/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: ADC 采集 (HPM6E8Y / hpm6e00YZ)
 *   - 相电流 (PWM 经 TRGM 触发 + 抢占模式 + DMA)
 *       I_U = ADC2_IN05 (PF28)
 *       I_V = ADC3_IN04 (PF29)
 *       I_W 由 I_U + I_V 计算 (2 电阻采样)
 *   - 母线电压 VBUS = ADC3_IN07 (PF31, 分压 200K/10K)
 *   - 温度        TEMP = ADC1_IN10 (PF08, MCP9700)
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 电流换算参数(按实际硬件修改): 采样电阻 / 运放增益 */
#define BSP_ADC_CURRENT_SHUNT_MOHM   (10U)      /* 采样电阻 10 mΩ */
#define BSP_ADC_CURRENT_GAIN_X10     (100U)     /* 运放增益 *10 = 100/10 */

/* 母线分压电阻 */
#define BSP_ADC_VBUS_TOP_OHM         (200000U)
#define BSP_ADC_VBUS_BOTTOM_OHM      (10000U)

/**
 * @brief  初始化相电流(PWM 触发+DMA) / 母线电压 / 温度 三路 ADC
 * @note   对照 STM32: 等价于 MX_ADCx_Init() + HAL_ADC_Start_DMA() 的组合。
 */
void bsp_adc_init(void);

/**
 * @brief  查询是否已有一次新的电流采样完成
 * @return true = 至少有了一次电流数据
 */
bool bsp_adc_current_ready(void);

/**
 * @brief  取"ADC 采样完成"的累计帧数 (只在 ADC 中断里自增)
 * @return 帧计数
 * @note   【用途】判断电流采样通路到底活没活。这个数不涨 = ADC 没有在转换,
 *         此时 bsp_adc_get_currents_ma() 返回的仍是上一帧(或初始的 0/32767),
 *         电流环拿它闭环必然跑飞。
 *         【预期速率】= PWM 载波频率 (20kHz), 即每毫秒约 20 帧。
 */
uint32_t bsp_adc_get_frame_count(void);

/**
 * @brief  读取相电流原始码值 (16bit)
 * @param  iu,iv,iw  输出指针, 可传 NULL; iw 由 iu/iv 反推得到
 */
void bsp_adc_get_currents_raw(uint16_t *iu, uint16_t *iv, uint16_t *iw);

/**
 * @brief  读取相电流 (mA), 已扣除零点并按采样电阻/增益换算
 * @param  iu,iv,iw  输出指针(mA), 可传 NULL
 */
void bsp_adc_get_currents_ma(int32_t *iu, int32_t *iv, int32_t *iw);

/**
 * @brief  把"相对零点的 ADC 码值"换算成电流 (mA)
 * @param  counts  相对零点的码值 (已是 raw - offset)
 * @note   与 bsp_adc_get_currents_ma 同一套标定参数; 1 LSB ≈ 0.504 mA
 */
int32_t bsp_adc_counts_to_ma(int32_t counts);

/**
 * @brief  读取母线电压
 * @return 母线电压 mV (按 200k/10k 分压换算); 读取失败返回 0
 */
uint32_t bsp_adc_get_vbus_mv(void);

/**
 * @brief  读取板载 MCP9700 温度
 * @return 温度, 单位 0.001 ℃(毫摄氏度); 读取失败返回 0
 */
int32_t bsp_adc_get_temperature_mc(void);

/**
 * @brief  相电流零点校准
 * @note   必须在电机静止、PWM 输出封锁时调用; 内部会等 PWM 触发的 256 个
 *         【新】样本求平均(约 12.8ms @20kHz), 作为零点 offset。
 *         调用前请不要先使能电机 —— 否则零点里会混进真实电流。
 */
void bsp_adc_calibrate(void);

/**
 * @brief  读取当前使用的相电流零点 (原始码值, 理论中点 = 65535/2)
 * @param  u,v  输出指针, 可传 NULL
 * @note   换算: 1 LSB ≈ 0.504 mA (10mΩ 采样电阻 × 10 倍增益, Vref=3.3V, 16bit)
 */
void bsp_adc_get_offsets(uint16_t *u, uint16_t *v);

/** @brief 电流采样完成回调类型 (在 ADC 中断上下文调用, 应尽量简短) */
typedef void (*bsp_adc_conv_cb_t)(void);

/**
 * @brief  注册电流采样完成回调
 * @param  cb  回调; 传 NULL 表示注销
 * @note   硬件中断 bsp_adc_isr 保留在本模块内; 应用层/FOC 通过此函数注册
 *         回调, 采样完成后由中断调用。对照 STM32: 类似 HAL_ADC_ConvCpltCallback(),
 *         但用运行时注册替代弱符号覆盖, 便于分层解耦。
 */
void bsp_adc_register_conv_cb(bsp_adc_conv_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
