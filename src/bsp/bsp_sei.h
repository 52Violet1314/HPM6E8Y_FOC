/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: MT6701 磁编码器 —— HPM SEI 硬件读取后端 (HPM6E8Y / hpm6e00YZ)
 *
 *   本质是一个 SEI 外设的中断驱动采集模块, 故归入 BSP 层。硬件中断服务
 *   函数 bsp_sei_isr() 保留在本模块内; 应用层/FOC 不直接接触中断, 而是通过
 *   bsp_sei_register_frame_cb() 注册一个"新帧到达"回调, 由本模块在中断里
 *   解析完帧后调用。这符合"硬件中断留在 BSP, 应用逻辑经注册回调上抛"的分层约定。
 *
 *   依赖: Hardware/mt6701/mt6701.h 提供纯协议层 (帧解析/CRC, 无硬件依赖)。
 *
 *   MT6701 配置为 SSI 模式: 主机(SEI)输出 SSI 时钟, 编码器在时钟上升沿
 *   串行输出 24bit 数据。SEI 硬件按周期自动发起一帧, 收完后产生中断,
 *   中断里解析并缓存角度, 应用层随时读取最新值。
 *
 * 【本板引脚 (见 pinmux.c / 原理图 encoder 页)】
 *   SEI_CTRL_1  PE16=TX  PE17=RX  PE18=DE  PE19=CK
 *   实际只用到 CK(输出时钟) 和 RX(编码器数据输入)。
 *   注意: 本板 pinmux.c 只实现 CTRL_1 的引脚 (PE 口); CTRL_0(PC 口) 与 QEI0 复用。
 *
 * 【对照 STM32】
 *   SEI 相当于 STM32 的 SPI(主机) + 自动帧 + 位置采样单元,
 *   一次配置后由硬件周期性收发, 类似 "SPI + DMA + 定时触发" 的合体。
 */
#ifndef BSP_SEI_H
#define BSP_SEI_H

#include <stdbool.h>
#include <stdint.h>

#include "mt6701.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 硬件配置 (按需修改) ---- */

/* SEI 控制块, 必须与本板 board.h 的 BOARD_SEI_CTRL 一致。
 *
 *   1 = SEI_CTRL_1: PE16=TX / PE17=RX / PE18=DE / PE19=CK  <-- 本板使用
 *   0 = SEI_CTRL_0: PC07=TX / PC06=RX / PC05=DE / PF04=CK  (与 QEI0 复用)
 *
 * 本板 pinmux.c 只实现了 CTRL_1 的引脚初始化 (init_sei_ctrl1_pins),
 * 且 board.h 定义 BOARD_SEI_CTRL = SEI_CTRL_1 / BOARD_SEI_IRQn = IRQn_SEI0_1,
 * 故此处用 1。若换板后编码器改走 SEI_CTRL_0(PC 口), 改回 0 即可。 */
#define BSP_SEI_CTRL_INDEX   (1)

/* 与已验证的 SPI2 配置保持一致：250kHz，先用慢速确认时序和电平。 */
#define BSP_SEI_BAUDRATE     (250000U)

/* 采样周期 (us)。200us = 5kHz, 满足常规 FOC 速度/位置环需求。 */
#define BSP_SEI_PERIOD_US    (200U)

/* ---- SSI 时序选项 (读不到数据时可在此切换) ----
 *
 * MT6701 有两种常见读法, 时钟极性相反:
 *   A. SPI 模式 (本机 3 个 MT6701 参考工程 G474EtherCAT_FOC / iRDT / F405
 *      均用 CPOL=0 + CPHA=1): 时钟【空闲低】, 第 2 边沿(下降沿)采样  <-- 默认
 *   B. SSI 标准 / HPM SDK samples/drivers/sei/master/ssi 例程:
 *      时钟【空闲高】, 第 1 边沿(下降沿)采样
 *
 * 当前按 SPI2 实测成功的 Mode 0：CPOL=0、CPHA=0，即空闲低、第一边沿采样。
 * 若后续逻辑分析仪确认你的 MT6701 接成标准 SSI 空闲高，再切换到 B。
 */

/* 时钟空闲电平: 0 = 空闲低(SPI CPOL=0, MT6701 参考工程), 1 = 空闲高(SSI 标准) */
#define BSP_SEI_CLK_IDLE_HIGH    (0)

/* 采样边沿: 1 = 第 2 个时钟边沿(SPI CPHA=1), 0 = 第 1 个时钟边沿 */
#define BSP_SEI_SAMPLE_2ND_EDGE  (0)

/* 是否启用 CRC6 校验 (默认关闭, 用真实波形确认多项式后再开) */
#define BSP_SEI_SSI_CRC_CHECK (0)

/* ---- 状态 ---- */
typedef struct {
    uint32_t      raw;          /* 最近一帧原始 24bit */
    mt6701_frame_t frame;       /* 最近一帧解析结果 */
    uint32_t      frame_count;  /* 成功接收帧数 */
    uint32_t      error_count;  /* 传输错误 / CRC 错误次数 */
    bool          new_frame;    /* 自上次 get_latest 后是否收到新帧 */
} bsp_sei_state_t;

/**
 * @brief  "新帧到达"回调类型 (在 SEI 中断上下文被调用, 应尽量简短)
 * @param  frame  解析后的帧
 * @param  raw    原始 24bit
 */
typedef void (*bsp_sei_frame_cb_t)(const mt6701_frame_t *frame, uint32_t raw);

/**
 * @brief  注册"新帧到达"回调
 * @param  cb  回调; 传 NULL 表示注销
 * @note   由应用层在初始化阶段注册, 中断里解析完帧后回调。
 */
void bsp_sei_register_frame_cb(bsp_sei_frame_cb_t cb);

/**
 * @brief  初始化 SEI0 引脚 + SEI 控制器, 并启动周期性 SSI 采样
 * @note   依赖 board_init() 已完成系统时钟初始化;
 *         函数内部会打开 SEI 时钟、配置引脚、装载指令表、使能中断。
 */
void bsp_sei_init(void);

/**
 * @brief  仅配置 SEI 引脚复用 (CK/RX/TX/DE)
 */
void bsp_sei_init_pins(void);

/**
 * @brief  读取最新一帧 (中断中已解析好)
 * @param  frame  输出解析结果, 可为 NULL
 * @return true = 自上次调用以来收到了新帧; false = 无新数据(沿用缓存)
 */
bool bsp_sei_get_latest(mt6701_frame_t *frame);

/** @return 最近一帧原始 24bit 数据 */
uint32_t bsp_sei_get_raw(void);

/**
 * @brief  直接读取 SEI 硬件数据寄存器(绕过中断缓存), 用于诊断
 * @return 由 DAT_2(角度14bit) + DAT_3(状态/CRC) 拼出的 24bit 原始值;
 *         若 SEI 引擎没在跑, 读回的是复位默认值
 */
uint32_t bsp_sei_peek_raw(void);

/** @return 最近一帧原始角度 (14bit) */
uint16_t bsp_sei_get_raw_angle(void);

/** @return 编码器角度 (弧度) */
float bsp_sei_get_angle_rad(void);

/** @return 成功接收的帧数 */
uint32_t bsp_sei_get_frame_count(void);

/** @return 错误计数 (SEI 传输错误 + CRC 错误) */
uint32_t bsp_sei_get_error_count(void);

/** @brief 软件触发一次采样 (调试/单次读取用, 正常情况下周期触发已足够) */
void bsp_sei_soft_trigger(void);

/* ---- 诊断 ---- */

/**
 * @brief  SEI 诊断信息 (用于确认配置是否真的生效)
 * @note   配置快照在 bsp_sei_init() 时记录; 计数/寄存器为读取时实时值。
 */
typedef struct {
    /* --- 初始化时的配置快照 --- */
    uint32_t ctrl_index;    /* SEI_CTRL_x                        */
    uint32_t clk_freq;      /* 源时钟 Hz; 为 0 说明 SEI 时钟没拿到 */
    uint32_t baud_div;      /* 波特率分频值 (实际写入寄存器)      */
    uint32_t ck0_point;     /* 时钟第 1 边沿点                    */
    uint32_t ck1_point;     /* 时钟第 2 边沿点                    */
    uint32_t trig_period;   /* 周期触发时间 (源时钟 tick)         */

    /* --- 读取时的实时状态 --- */
    uint32_t frame_count;   /* 成功收帧数                         */
    uint32_t error_count;   /* 传输/CRC 错误数                    */
    uint32_t irq_enable;    /* 中断使能寄存器回读                 */
    uint32_t irq_flag;      /* 当前中断标志回读                   */
    uint32_t peek_raw;      /* 直接读硬件 DAT 寄存器拼出的 24bit   */
} bsp_sei_diag_t;

/**
 * @brief  读取 SEI 诊断信息
 * @param  diag  输出结构, 不可为 NULL
 */
void bsp_sei_get_diag(bsp_sei_diag_t *diag);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SEI_H */
