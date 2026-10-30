/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 锐鹰 SRMA35/46 分体式多圈绝对值编码器 —— RS485 后端 (UART11)
 *
 *   协议: 主机发 1 字节 CF(0x1A) -> 编码器回 11 字节
 *         CF SF ABS0 ABS1 ABS2 ENID ABM0 ABM1 ABM2 ALMC CRC
 *         CRC = 除 CRC 外所有字节异或
 *
 *   引脚 (与 SEI2 复用同一组, 换后端不用改线):
 *     PD15 = UART11_TXD (ALT2)
 *     PD14 = UART11_RXD (ALT2)
 *     PD13 = GPIO 输出, 控 485 收发器 DE (高=发, 低=收)
 *
 *   一帧的时序 (每一步都在代码里, 可读可测):
 *     DE=1 -> 等 1us(收发器建立) -> 发 CF(4us) -> 等 TEMT(字节出线,含 stop 位)
 *     -> DE=0 -> 编码器 2.3~5.5us 后回 11 字节(44us) -> RX 中断收集 -> 校验
 *
 *   【两个必须保留的细节, 都是实测踩出来的】
 *   1) DE 释放沿必须早于编码器开口(最快 2.3us)。20kHz 电流环中断(PLIC 7)只要
 *      落在这 5us 窗口里, DE 就晚放, 编码器前几个字节被两个驱动器对打糊掉。
 *      实测不保护时坏帧 0.44%, 关中断保护后 0.047%。
 *   2) 收帧窗口必须在【放开总线之后】再打开, 并且先掏空 RX FIFO。否则上一帧
 *      的残字节会被当成新帧第一个字节, 整帧右移一格(实测出现过错位帧通过 CRC)。
 */
#ifndef BSP_SRMA35_H
#define BSP_SRMA35_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 硬件配置 ---- */
#define BSP_SRMA35_BAUDRATE         (2500000U)   /* 编码器额定 2.5Mbps */
#define BSP_SRMA35_PERIOD_MS        (1U)         /* 读帧周期 1ms = 1kHz */
#define BSP_SRMA35_RX_TIMEOUT_MS    (2U)         /* 回帧超时 (11 字节只要 44us) */
#define BSP_SRMA35_DE_SETUP_US      (1U)         /* DE 拉高到发首位的建立时间 */

/* 发 CF + 释放 DE 这一段是否关中断 (见文件头第 1 条)。1 = 关 (默认) */
#ifndef BSP_SRMA35_DE_CRITICAL
#define BSP_SRMA35_DE_CRITICAL      (1)
#endif

/* ---- 协议常量 ---- */
#define BSP_SRMA35_CF_ALL           (0x1AU)      /* ID3: 位置+多圈+ID+故障 */
#define BSP_SRMA35_RSP_LEN          (11U)
#define BSP_SRMA35_ABS_MAX          (131072U)    /* 单圈 17bit */
#define BSP_SRMA35_ABM_MAX          (65536U)     /* 多圈 16bit */

/* ---- 故障位 (ALMC) ---- */
#define BSP_SRMA35_ALMC_OVERSPEED   (1U << 0)
#define BSP_SRMA35_ALMC_COUNT_ERR   (1U << 2)
#define BSP_SRMA35_ALMC_MULTITURN   (1U << 5)
#define BSP_SRMA35_ALMC_BATT_ERR    (1U << 6)
#define BSP_SRMA35_ALMC_BATT_ALARM  (1U << 7)

/** @brief 一帧解析结果 */
typedef struct {
    uint8_t  raw[BSP_SRMA35_RSP_LEN];
    uint8_t  cf;         /* 回显的命令字, 正常 = 0x1A */
    uint8_t  sf;         /* 状态字 */
    uint8_t  enid;       /* 编码器 ID, 默认 0x11 */
    uint8_t  almc;       /* 故障标志 */
    uint8_t  crc_rx;
    uint8_t  crc_calc;
    uint32_t abs;        /* 单圈位置 17bit (0..131071) */
    uint32_t abm;        /* 多圈位置 16bit */
    uint32_t pos32;      /* abm<<17 | abs */
    bool     crc_ok;
    bool     valid;      /* cf 回显正确且 CRC 通过 */
} bsp_srma35_frame_t;

/** @brief 初始化 (UART11 + PD13 控 DE), 启动周期读帧 */
void bsp_srma35_init(void);

/** @brief 每 1ms 调一次: 到点发 CF, 并判断回帧超时 */
void bsp_srma35_poll(void);

/** @brief 取最近一帧; 返回 true 表示这次拿到了新的、校验通过的帧 */
bool bsp_srma35_get_latest(bsp_srma35_frame_t *out);

/** @brief 取最近一帧的副本 (不论是否校验通过) */
void bsp_srma35_peek(bsp_srma35_frame_t *out);

uint32_t bsp_srma35_get_frame_count(void);
uint32_t bsp_srma35_get_error_count(void);
bool     bsp_srma35_is_silent(void);

/** @brief 每 1ms 调一次, 内部每 1s 打一行状态 + 坏帧现场 */
void bsp_srma35_poll_log(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SRMA35_H */
