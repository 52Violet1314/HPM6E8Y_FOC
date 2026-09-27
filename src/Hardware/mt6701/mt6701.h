/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Hardware: MT6701 磁编码器 —— 协议层 (与 MCU 外设无关)
 *   - 解析 MT6701 的 SSI 绝对角度帧
 *   - 帧格式: 14bit 角度 + 4bit 状态 + 6bit CRC = 24bit
 *   - 只做数据解析, 不依赖任何寄存器/外设, 便于单元测试和移植
 *
 * MT6701 SSI 数据帧 (MSB 先出):
 *   bit[23:10]  ANGLE  : 14bit 绝对角度, 0 ~ 16383 对应 0 ~ 360°
 *   bit[9:6]    STATUS : 4bit 状态
 *                          [1:0] 磁场状态 0=正常 1=过强 2=过弱 3=错误
 *                          [2]   磁按钮按下 (部分封装)
 *                          [3]   跟踪丢失 (track loss)
 *   bit[5:0]    CRC6   : 6bit CRC, 多项式 x^6 + x + 1 (0x43), 初值 0
 *
 * 对照 STM32: 相当于一个纯软件的传感器协议解析文件,
 *             硬件接口部分见 src/bsp/bsp_mt6701.c (SPI2 读取) /
 *             src/bsp/bsp_sei.c (板载 SEI 通道读取)。
 */
#ifndef MT6701_H
#define MT6701_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 帧格式常量 ---- */
#define MT6701_SSI_FRAME_BITS   (24U)              /* 一帧总位数 */
#define MT6701_SSI_FRAME_MASK   (0x00FFFFFFUL)     /* 一帧掩码 */
#define MT6701_ANGLE_BITS       (14U)              /* 角度位数 */
#define MT6701_ANGLE_MAX        (16383U)           /* 角度最大值 */
#define MT6701_CPR              (16384U)           /* 每圈计数 (2^14) */

#define MT6701_TWO_PI           (6.28318530717958647692f)
#define MT6701_DEG_PER_COUNT    (360.0f / (float)MT6701_CPR)

/* 磁场状态 (STATUS[1:0]) */
typedef enum {
    mt6701_field_normal = 0,    /* 磁场正常 */
    mt6701_field_strong = 1,    /* 磁场过强, 建议拉开磁铁 */
    mt6701_field_weak   = 2,    /* 磁场过弱, 建议靠近磁铁 */
    mt6701_field_error  = 3,    /* 磁场错误/丢失 */
} mt6701_field_status_t;

/* 解析后的一帧数据 */
typedef struct {
    uint16_t angle;                     /* 14bit 原始角度 0 ~ 16383 */
    uint8_t  status;                    /* 4bit 原始状态 */
    uint8_t  crc;                       /* 帧内收到的 CRC6 */
    uint8_t  crc_calc;                  /* 本地计算的 CRC6 */
    bool     crc_ok;                    /* CRC 是否一致 */
    mt6701_field_status_t field;        /* 磁场状态 */
    bool     button;                    /* 磁按钮是否按下 */
    bool     track_loss;                /* 是否跟踪丢失 */
} mt6701_frame_t;

/**
 * @brief  计算 MT6701 SSI 的 CRC6
 * @param  data18  前 18bit 数据 (角度 14bit << 4 | 状态 4bit)
 * @return 6bit CRC 值
 * @note   多项式 x^6 + x + 1 = 0b1000011 (0x43), 初值 0, MSB 先入。
 *         CRC 校验默认关闭 (见 src/bsp/bsp_mt6701.c 的 BSP_MT6701_CRC_CHECK),
 *         待用真实编码器波形确认多项式后再打开。
 */
uint8_t mt6701_crc6(uint32_t data18);

/**
 * @brief  解析一帧 24bit 原始数据
 * @param  raw24  原始帧 (bit[23:0], MSB = 角度最高位)
 * @param  frame  解析结果输出 (不可为 NULL)
 */
void mt6701_decode_frame(uint32_t raw24, mt6701_frame_t *frame);

/* ---- 便捷换算 ---- */

/** @return 原始角度 -> 弧度 [0, 2π) */
static inline float mt6701_angle_to_rad(uint16_t angle)
{
    return (float)(angle & MT6701_ANGLE_MAX) * (MT6701_TWO_PI / (float)MT6701_CPR);
}

/** @return 原始角度 -> 角度制 [0, 360) */
static inline float mt6701_angle_to_deg(uint16_t angle)
{
    return (float)(angle & MT6701_ANGLE_MAX) * MT6701_DEG_PER_COUNT;
}

/** @return 原始角度 -> 机械圈数 [0, 1) */
static inline float mt6701_angle_to_rev(uint16_t angle)
{
    return (float)(angle & MT6701_ANGLE_MAX) / (float)MT6701_CPR;
}

/**
 * @brief  计算两次角度之间的最短增量 (圈数, 已过零处理)
 * @param  prev  上一次原始角度
 * @param  now   当前原始角度
 * @return 增量, 范围 [-0.5, 0.5) 圈; 正 = 正转
 * @note   用于 FOC 速度估算 / 多圈累计。
 */
static inline float mt6701_angle_delta_rev(uint16_t prev, uint16_t now)
{
    int32_t d = (int32_t)(now & MT6701_ANGLE_MAX) - (int32_t)(prev & MT6701_ANGLE_MAX);
    if (d > (int32_t)(MT6701_CPR / 2U)) {
        d -= (int32_t)MT6701_CPR;
    } else if (d < -(int32_t)(MT6701_CPR / 2U)) {
        d += (int32_t)MT6701_CPR;
    }
    return (float)d / (float)MT6701_CPR;
}

#ifdef __cplusplus
}
#endif

#endif /* MT6701_H */
