/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: MT6701 磁编码器读取后端 —— SPI2 @ J11 扩展排针
 *
 *   与 bsp_sei.c (板载 SEI 通道) 是同一个角色的两种后端: 都是"读 MT6701 ->
 *   解析 -> 缓存最新角度", 应用层通过回调拿到新帧。本模块走普通 SPI 主机,
 *   由主循环主动 apoll, 不占用中断, 也不依赖 SEI 的指令表/状态机。
 *
 *   分层约定与 SEI 后端一致: 硬件细节留在 BSP, 应用逻辑经注册回调上抛。
 *   协议解析仍由 Hardware/mt6701/mt6701.c 这个纯协议层完成, 本模块只负责
 *   取 24bit 原始帧。
 *
 * 【SPI 方式与 SSI 方式的区别】
 *   SEI 后端: 编码器接成 SSI, 由 SEI 硬件按周期自动收发并产生中断 —— 相当于
 *             "SPI + 定时触发 + DMA" 的合体。
 *   SPI 后端: 就是最普通的主机 SPI。片选拉低, 敲 24 个时钟, 读回 3 字节,
 *             片选拉高。对应 STM32 上的 HAL_SPI_Receive()。
 */
#ifndef BSP_MT6701_H
#define BSP_MT6701_H

#include <stdbool.h>
#include <stdint.h>

#include "mt6701.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MT6701 一帧 = 24bit = 3 字节 */
#define BSP_MT6701_FRAME_BYTES  (3U)

/* 是否启用 CRC6 校验。默认关闭: 先确认能稳定读到数据, 再打开做完整性检查。 */
#define BSP_MT6701_CRC_CHECK    (0)

/* ---- 状态 ---- */
typedef struct {
    uint32_t       raw;           /* 最近一帧原始 24bit (成功) */
    mt6701_frame_t frame;         /* 最近一帧解析结果 */
    uint32_t       last_raw;      /* 最近一次读到的原始 24bit (含无效帧, 诊断用) */
    uint32_t       frame_count;   /* 成功帧数 */
    uint32_t       error_count;   /* 数据无效次数 (全 0/全 1/CRC) */
    uint32_t       transfer_fail; /* SPI 传输失败/超时次数 */
    bool           new_frame;     /* 自上次 get_latest 后是否收到新帧 */
} bsp_mt6701_state_t;

/** "新帧到达"回调 (在主循环上下文被调用, 可做稍重的处理) */
typedef void (*bsp_mt6701_frame_cb_t)(const mt6701_frame_t *frame, uint32_t raw);

/**
 * @brief  初始化 SPI2 + 片选, 并清空状态
 * @note   依赖 board_init() 已完成系统时钟初始化
 */
void bsp_mt6701_init(void);

/**
 * @brief  主动读一帧并解析 (SPI 没有硬件周期触发, 由上层按需调用)
 * @return true = 本帧有效
 * @note   在本工程里由 app_run() 每次控制周期调用一次, 对应 1kHz 采样率。
 */
bool bsp_mt6701_poll(void);

/** 注册"新帧到达"回调; 传 NULL 注销 */
void bsp_mt6701_register_frame_cb(bsp_mt6701_frame_cb_t cb);

/**
 * @brief  读取最新一帧 (poll 时已解析好)
 * @param  frame  输出解析结果, 可为 NULL
 * @return true = 自上次调用以来收到了新帧
 */
bool bsp_mt6701_get_latest(mt6701_frame_t *frame);

/** @return 最近一帧原始 24bit */
uint32_t bsp_mt6701_get_raw(void);

/** @return 最近一帧原始角度 (14bit) */
uint16_t bsp_mt6701_get_raw_angle(void);

/** @return 编码器角度 (弧度) */
float bsp_mt6701_get_angle_rad(void);

/** @return 成功接收的帧数 */
uint32_t bsp_mt6701_get_frame_count(void);

/** @return 错误计数 */
uint32_t bsp_mt6701_get_error_count(void);

/** @return 最近一次读到的原始 24bit (诊断用, 失败时也能看) */
uint32_t bsp_mt6701_get_last_raw(void);

/** @return SPI 传输失败/超时次数 (诊断用: 与"数据无效"分开) */
uint32_t bsp_mt6701_get_transfer_fail(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MT6701_H */
