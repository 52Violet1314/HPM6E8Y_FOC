/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: MT6701 专用 SPI 主机 (HPM6E8Y / hpm6e00yz)
 *
 *   走 J11 扩展排针上的 SPI2, 片选由 GPIO 软件控制。
 *
 * 【为什么是 SPI2 + 这三根脚】
 *   板上对外引出的排针一共两处: J11(2x22 扩展排针) 和 J10(16 脚显示接口)。
 *   逐个核对后, 能在 J11 上拼出完整硬件 SPI 的只有这一组:
 *       PC26 = SPI2.SCLK   -> J11 脚 31
 *       PC28 = SPI2.MISO   -> J11 脚 10
 *       PC24 = GPIO 输出   -> J11 脚 11   (片选, 低有效)
 *   说明:
 *     - SPI2.MOSI = PC29 没有引到 J11, 且 MT6701 是只读器件, 不需要 MOSI;
 *     - SPI2.CS_0 = PC27 也没引出, 所以片选只能用 GPIO 软件拉;
 *     - J11 上的 PF26/PF27 虽然名字是 SPI7.SCLK/CS0, 但 SPI7 的
 *       MISO/MOSI(PF28/PF29) 在本板上被相电流采样 ADC 占用了, 无法使用。
 *
 * 【注意: 板级 pinmux 会把这几个脚复用成 FEMC】
 *   init_femc_pins() 里 PC24/PC25/PC26/PC27/PC28 分别被设成
 *   FEMC_A_09/FEMC_A_11/FEMC_A_12/FEMC_CLK_0/FEMC_CKE。本工程不使用 FEMC,
 *   bsp_spi_init() 会重新写 IOC 复用寄存器把它们抢回来。
 *
 * 【SPI 模式】实测为 Mode 0: CPOL = 0 (空闲低) / CPHA = 0 (第 1 个边沿采样) /
 *   MSB first, 且 CS 必须每帧翻转。
 *   注意: 3 个 STM32 参考工程写的都是 Mode 1 (CPHA=1), 照抄会恒读 0xFFFFFF。
 *   结论来自 bsp_spi_bb.c 的 16 组合 bit-bang 扫描 —— 只有 idx=8
 *   (CS 翻转 + CPOL=0 + CPHA=0 + MSB) 能读出 CRC6 合法的帧。
 */
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "hpm_common.h"
#include "hpm_gpio_drv.h"
#include "hpm_iomux.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 外设 / 时钟 ---- */
#define BSP_SPI_BASE          HPM_SPI2
#define BSP_SPI_CLOCK_NAME    clock_spi2

/* SCLK 目标频率 (bring-up 阶段取慢速)。
 *
 * 【注意驱动有一条下限约束, 不是上限】
 *   spi_master_timing_init 要求 div = clk_src/sclk 满足:
 *       ① 整除  ② 偶数  ③ <= 510
 *   即 SCLK 的下限约 clk_src/510 (源时钟 200MHz 时约 392kHz)。
 *   若传入非法频率, 驱动返回 status_invalid_argument 且不写 TIMING 寄存器。
 *   bsp_spi_init() 会自动在合法值里挑最接近本目标的频率, 不会踩这个坑。
 *
 * 目标值取 250kHz: 24bit 一帧约 96us, 对 1kHz 控制环绰绰有余; 慢速对
 * 长杜邦线的信号完整性、以及"DOUT 若为开漏输出"的上升时间都更宽容。 */
#define BSP_SPI_SCLK_FREQ_HZ  (250000U)

/* ---- 引脚 (J11 扩展排针) ---- */
#define BSP_SPI_PAD_SCLK      IOC_PAD_PC26   /* -> J11 脚 31 */
#define BSP_SPI_PAD_MISO      IOC_PAD_PC28   /* -> J11 脚 10 */
#define BSP_SPI_PAD_CS        IOC_PAD_PC24   /* -> J11 脚 11 (GPIO 片选) */

#define BSP_SPI_CS_GPIO_CTRL  HPM_GPIO0      /* 本 SoC 所有端口都在 GPIO0 */
#define BSP_SPI_CS_GPIO_PORT  GPIO_DI_GPIOC
#define BSP_SPI_CS_GPIO_PIN   (24U)

/* ---- 接口 ---- */

/**
 * @brief  配置 SPI2 引脚复用 + 主机模式 (CPOL=0 / CPHA=1 / MSB first)
 * @note   依赖 board_init() 已完成系统时钟初始化; 需在 app 初始化阶段调用。
 */
void bsp_spi_init(void);

/**
 * @brief  片选拉低 -> 全双工传 len 字节 -> 片选拉高
 * @param  rx   接收缓冲, 长度 >= len
 * @param  len  字节数 (发送侧统一补 0xFF, MT6701 不接 MOSI)
 * @return true = 传输成功
 */
bool bsp_spi_read(uint8_t *rx, uint32_t len);

/* ---- 诊断查询 (确认时钟/分频/时序初始化结果) ---- */

/** @return SPI2 源时钟频率 (Hz) */
uint32_t bsp_spi_get_src_clk(void);

/** @return 实际使用的 SCLK 频率 (Hz) */
uint32_t bsp_spi_get_sclk(void);

/** @return spi_master_timing_init 的返回值 (0 = status_success) */
int bsp_spi_get_timing_stat(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
