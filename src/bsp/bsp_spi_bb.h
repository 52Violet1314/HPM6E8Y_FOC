/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: GPIO 软件模拟 SPI (bit-bang) —— 编码器接线排查专用
 *
 * 目的: 把 SPI2 外设完全排除在外, 直接用软件翻转 PC26(SCLK) / 读 PC28(MISO) /
 * 控制 PC24(CS), 并自动把下面 16 种组合全部试一遍:
 *
 *       CPOL (2) × CPHA (2) × CS 方式 (2) × 位序 (2) = 16
 *
 * 判据:
 *   - distinct >= 2   该组合读到了"会变化的数据" -> 编码器确实在响应
 *   - crc_ok  > 0     该组合有帧通过 MT6701 的 CRC6 -> 基本可以确定找对了
 *   若 16 种组合的 distinct 全部 <= 1, 就只能得出一个结论:
 *       DOUT 上没有任何器件在对任何时钟方案做出响应 —— 纯硬件问题。
 *
 * 另外还会单独记录"一个时钟都不打时 MISO 的静态电平", 这是区分
 * "悬空(被上拉拉高)" 与 "被外部硬拉低" 的最直接证据。
 *
 * @note 扫描会临时把 PC26/PC28 从 SPI2 复用切成 GPIO, 结束时自动调用
 *       bsp_spi_init() 恢复, 对正常流程无影响。
 */
#ifndef BSP_SPI_BB_H
#define BSP_SPI_BB_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 扫描的组合总数: CPOL(2) × CPHA(2) × CS方式(2) × 位序(2) */
#define BSP_BB_MODE_COUNT       (16U)

/* 每个组合采样的帧数 */
#define BSP_BB_FRAMES_PER_MODE  (8U)

/* MT6701 一帧 24bit: [23:10] 角度, [9:6] 状态, [5:0] CRC6 */
#define BSP_BB_FRAME_BITS       (24U)

/* ---- 引脚 (与硬件 SPI 完全同一组脚, 定义见 bsp_spi.h) ---- */
#define BSP_BB_GPIO_CTRL        BSP_SPI_CS_GPIO_CTRL   /* HPM_GPIO0            */
#define BSP_BB_GPIO_PORT        BSP_SPI_CS_GPIO_PORT   /* GPIO_DI_GPIOC        */
#define BSP_BB_SCLK_GPIO_PIN    (26U)                  /* PC26 -> J11 脚 31    */
#define BSP_BB_MISO_GPIO_PIN    (28U)                  /* PC28 -> J11 脚 10    */
#define BSP_BB_CS_GPIO_PIN      BSP_SPI_CS_GPIO_PIN    /* PC24 -> J11 脚 11    */

/* SCLK 半周期 (us)。2us 半周期 -> 4us/bit -> 250kHz, 与硬件 SPI 目标一致;
 * 对 22k 上拉 × 约 50pF 的约 1.1us 上升时间有充足余量。 */
#define BSP_BB_HALF_US          (2U)

/** 一种扫描组合 */
typedef struct {
    uint8_t cpol;        /* 0 = 时钟空闲低, 1 = 空闲高         */
    uint8_t cpha;        /* 0 = 第 1 个沿采样, 1 = 第 2 个沿采样 */
    uint8_t cs_toggle;   /* 1 = 每帧翻转 CS, 0 = CS 一直保持低    */
    uint8_t lsb_first;   /* 1 = 低位在前                        */
} bsp_bb_mode_t;

/** 一种组合的扫描结果 */
typedef struct {
    uint32_t raw_first;   /* 本轮第 1 帧           */
    uint32_t raw_last;    /* 本轮最后一帧          */
    uint32_t raw_min;     /* 本轮最小值            */
    uint32_t raw_max;     /* 本轮最大值            */
    uint32_t distinct;    /* 出现过的不同 raw 个数  */
    uint32_t crc_ok;      /* CRC6 校验通过的帧数    */
} bsp_bb_result_t;

/* ---- 执行 ---- */

/**
 * @brief  跑一次全组合扫描 (阻塞, 约十几 ms, 建议只在初始化时调用一次)
 * @note   内部会临时切 GPIO, 返回前自动恢复 SPI2 复用。
 */
void bsp_bb_scan(void);

/* ---- 结果查询 ---- */

/** @return 一个时钟都没打时 MISO 的静态电平 (0/1); 未扫描过时返回 0xFF */
uint8_t bsp_bb_static_level(void);

/** @return 【此刻】MISO 引脚电平 (0/1), 不打时钟、不改引脚配置 */
uint8_t bsp_bb_miso_live(void);

/** @return 读到"变化数据"(distinct>=2) 的组合数, 0..16 */
uint32_t bsp_bb_hit_count(void);

/** @return 其中有帧通过 CRC6 的组合数, 0..16 */
uint32_t bsp_bb_crc_hit_count(void);

/**
 * @return 最佳组合下标 —— 按 **CRC6 通过帧数** 选 (要求 >= 半数帧通过),
 *         -1 表示没有任何组合达到可信的 CRC 通过率。
 * @note   判据必须用 CRC, 不能用 "distinct>=2": 转子静止时正确的组合读出
 *         恒定值, distinct 反而只有 1。
 */
int32_t bsp_bb_best_index(void);

/** @return 最佳组合的代表 raw (raw_last); 无最佳时返回 0 */
uint32_t bsp_bb_best_raw(void);

/** @return 指定组合的描述; idx 越界返回 NULL */
const bsp_bb_mode_t *bsp_bb_mode_at(uint32_t idx);

/** @return 指定组合的结果; idx 越界返回 NULL */
const bsp_bb_result_t *bsp_bb_result_at(uint32_t idx);

/**
 * @brief  把某个组合的结果格式化成一行文本 (供日志逐条打印)
 * @return 写入长度 (不含 '\0')
 */
int bsp_bb_format(char *buf, uint32_t size, uint32_t idx);

/**
 * @brief  用指定组合现场读一帧 (会临时切 GPIO, 读完恢复 SPI2)
 * @param  idx  组合下标, 越界返回 0
 * @return 读到的 24bit 原始值
 */
uint32_t bsp_bb_read_raw(uint32_t idx);

/* ---- 把 bit-bang 变成正式读取后端 (兜底用) ---- */

/**
 * @brief  激活某个组合: 把 PC26/PC28 切到 GPIO 并【保持】,
 *         之后 bsp_bb_read() 就可以低成本地反复调用。
 * @return true = 激活成功
 * @note   激活后引脚不再属于 SPI2, 要走回硬件 SPI 需重新 bsp_spi_init()。
 */
bool bsp_bb_activate(uint32_t idx);

/** @return 当前 bit-bang 后端是否已激活 */
bool bsp_bb_is_active(void);

/**
 * @brief  用已激活的组合读一帧, 按大端拆成 3 字节
 * @param  rx   接收缓冲 (长度须 >= 3)
 * @param  len  缓冲长度
 * @return true = 读成功
 * @note   字节序与硬件 SPI 路径一致 (rx[0] = 最高字节), 便于两条路互换。
 */
bool bsp_bb_read(uint8_t *rx, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_BB_H */
