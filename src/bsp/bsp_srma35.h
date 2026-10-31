/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 锐鹰 SRMA35 —— RS485 后端 (SEI_CTRL_2 异步模式, 10kHz)
 *
 *   协议: 发 1 字节 CF(0x1A) -> 回 11 字节
 *         CF SF ABS0 ABS1 ABS2 ENID ABM0 ABM1 ABM2 ALMC CRC  (CRC = 异或)
 *   引脚: PD15=SEI2_TX / PD14=SEI2_RX / PD13=SEI2_DE  (ALT22)
 *
 * 【为什么用 SEI 而不是 UART】DE 的释放沿必须落在编码器开口之前(它最快 2.3us
 *   就回包)。20kHz 电流环中断只要落进那 5us 窗口, 软件控 DE 就会晚放, 回包前
 *   几个字节被两个驱动器对打糊掉(实测 UART 版 0.44% 坏帧, 关中断保护后 0.047%)。
 *   SEI 的 DE 由外设硬件在 SCLK 域翻转, 不受中断延迟影响 —— 这是能跑 10kHz 的
 *   前提(10kHz 下软件关中断窗口会占 5~6% 占空比)。
 *
 * 【10kHz 时序余量】一帧 = 发 4us + 回帧延迟 2.3~5.5us + 收 11 字节 44us
 *   ≈ 50~55us; 周期 100us -> 总线占空比 ~50%, 也满足编码器 16kHz 上限。
 */
#ifndef BSP_SRMA35_H
#define BSP_SRMA35_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 硬件 ---- */
#define BSP_SRMA35_SEI_CTRL_INDEX   (2)          /* 2 = SEI_CTRL_2 (PD13/14/15) */
#define BSP_SRMA35_BAUDRATE         (2500000U)
#define BSP_SRMA35_PERIOD_US        (100U)       /* 10kHz */
#define BSP_SRMA35_CF_ALL           (0x1AU)
#define BSP_SRMA35_RSP_LEN          (11U)
#define BSP_SRMA35_ABS_MAX          (131072U)    /* 单圈 17bit */
#define BSP_SRMA35_ABM_MAX          (65536U)

/* ---- 故障位 (ALMC) ---- */
#define BSP_SRMA35_ALMC_OVERSPEED   (1U << 0)
#define BSP_SRMA35_ALMC_COUNT_ERR   (1U << 2)
#define BSP_SRMA35_ALMC_MULTITURN   (1U << 5)
#define BSP_SRMA35_ALMC_BATT_ERR    (1U << 6)
#define BSP_SRMA35_ALMC_BATT_ALARM  (1U << 7)

typedef struct {
    uint8_t  raw[BSP_SRMA35_RSP_LEN];
    uint8_t  cf, sf, enid, almc, crc_rx, crc_calc;
    uint32_t abs;        /* 单圈 17bit (0..131071) */
    uint32_t abm;        /* 多圈 16bit */
    uint32_t pos32;      /* abm<<17 | abs */
    bool     crc_ok;
    bool     valid;      /* cf 回显正确且 CRC 通过 */
} bsp_srma35_frame_t;

/** @brief "新帧到达"回调: 在 SEI 中断上下文调用, 10kHz 一次, 必须短 */
typedef void (*bsp_srma35_frame_cb_t)(const bsp_srma35_frame_t *f);

/** @brief 注册回调 —— 10kHz 级联(位置/速度环)就挂在这里 */
void bsp_srma35_register_frame_cb(bsp_srma35_frame_cb_t cb);

/** @brief 初始化 SEI2 异步模式, 启动 10kHz 周期触发 */
void bsp_srma35_init(void);

/** @brief 兼容接口: SEI 后端由硬件触发, 本函数是空操作 */
void bsp_srma35_poll(void);

bool     bsp_srma35_get_latest(bsp_srma35_frame_t *out);
void     bsp_srma35_peek(bsp_srma35_frame_t *out);
uint32_t bsp_srma35_get_frame_count(void);
uint32_t bsp_srma35_get_error_count(void);
bool     bsp_srma35_is_silent(void);

/** @brief 每 1ms 调一次, 内部每 1s 打一行状态 + 坏帧现场 */
void bsp_srma35_poll_log(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SRMA35_H */
