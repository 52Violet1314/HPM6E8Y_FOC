/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 内置 EtherCAT 从站控制器 (ESC) 硬件层 —— 只做"把硬件点亮 + 读状态",
 *      不做协议栈, 不打印(日志留给 app 层, 与其它 BSP 模块一致)。
 *
 * 硬件事实 (HPM6E8Y, 板级宏见 board.h 的 BOARD_ECAT_* / pinmux.c 的 init_esc_pins):
 *   - ESC 基地址 0xF1700000, PDI 是 "On-chip bus"(0x80), 寄存器/过程数据 RAM 直接内存映射;
 *   - 本板只引出 P0/P1 两个口, MDI 直接接 RJ45 变压器 (片内集成 2 个 100M PHY);
 *   - 板上没有 ESC EEPROM -> 必须用 GPR_CFG0.EEPROM_EMU 模拟 (SII 由软件喂);
 *   - PHY 复位 PV12/PW12 (低有效), 且 board_init_ethercat() 之后默认仍停在复位态;
 *   - 端口的 LINK 信号不是寄存器而是 PHY 的 LED 输出经 ESC0_CTR_x 引脚回灌
 *     (PA28=CTR_1->LINK0, PA25=CTR_0->LINK1, 低有效 -> 需要 invert)。
 *
 * 回环自检能证明什么、不能证明什么:
 *   能: PHY 复位时序 / 25M 参考时钟 / MDI+变压器+RJ45 / MDIO 通路 / LINK 极性。
 *   不能: 报文处理。ESC 不会自己发帧, 没有主站就永远没有流量, CP(communication) 位
 *         会一直是 0 —— 那是正常的, 不是故障。
 */
#ifndef BSP_ECAT_H
#define BSP_ECAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 本板引出的端口数 (P0/P1) */
#define BSP_ECAT_PORT_CNT   (2U)

/* 一次采样得到的完整状态 */
typedef struct {
    /* ---- 通过 ESC 的 MII 管理接口 (MDIO/MDC) 从 PHY 读回来的 ---- */
    bool     mdio_ok[BSP_ECAT_PORT_CNT];    /* PHY 有应答 (读到合法 PHYID) */
    uint16_t phy_id1[BSP_ECAT_PORT_CNT];    /* 寄存器 2 */
    uint16_t phy_id2[BSP_ECAT_PORT_CNT];    /* 寄存器 3 */
    uint16_t bmcr[BSP_ECAT_PORT_CNT];       /* 寄存器 0 (控制) */
    uint16_t bmsr[BSP_ECAT_PORT_CNT];       /* 寄存器 1 (状态) */
    bool     phy_link[BSP_ECAT_PORT_CNT];   /* BMSR.2  链路 */
    bool     phy_100m[BSP_ECAT_PORT_CNT];   /* BMSR.13/14 协商结果 100M */
    bool     phy_fd[BSP_ECAT_PORT_CNT];     /* BMSR.14(100M FD) / .12(10M FD) */

    /* ---- 片内 PHY 状态: ESC 私有寄存器 PHY_STAT[port] (0x518+y) ----
     * 【重要】板上的 PHY 是集成在芯片里的, 不挂在 PDI 侧那对 MDIO/MDC 上,
     * 所以 mdio_ok 为 false 是正常的 —— 链路状态以这一组为准。 */
    uint8_t  phy_stat[BSP_ECAT_PORT_CNT];   /* 原始寄存器值 */
    bool     phy_pls[BSP_ECAT_PORT_CNT];    /* bit0 PLS: 物理 link (PHY 寄存器 1.2) */
    bool     phy_ls[BSP_ECAT_PORT_CNT];     /* bit1 LS : link (100M 全双工 自协商) */
    bool     phy_lse[BSP_ECAT_PORT_CNT];    /* bit2 LSE: link 错误 / link 被抑制 */
    bool     phy_re[BSP_ECAT_PORT_CNT];     /* bit3 RE : 读 PHY 状态出错(粘滞位) */
    bool     phy_lpe[BSP_ECAT_PORT_CNT];    /* bit4 LPE: link partner 错误 */

    /* ---- ESC 自己的链路状态 (ESC_DL_STAT, 0x0110) ---- */
    uint16_t dl_stat;
    bool     esc_link[BSP_ECAT_PORT_CNT];   /* PLP0/PLP1: 物理 link (来自 CTR 引脚) */
    bool     esc_loop[BSP_ECAT_PORT_CNT];   /* LP0/LP1:   1 = 该口内部环回(无 link) */
    bool     esc_comm[BSP_ECAT_PORT_CNT];   /* CP0/CP1:   1 = 该口通信稳定 */
    bool     eeprom_loaded;                 /* EPLC: EEPROM 已正确加载(本板恒 0, 走模拟) */

    /* ---- 计数器 ---- */
    uint16_t rx_err_cnt[BSP_ECAT_PORT_CNT];
    uint8_t  lost_link_cnt[BSP_ECAT_PORT_CNT];
} bsp_ecat_status_t;

/**
 * @brief ESC 硬件初始化(幂等)
 *
 * 顺序: board_init_ethercat()(时钟+54 根引脚+PHY 复位脚) -> EEPROM 模拟+时钟
 *       -> 释放 PHY 复位 -> 配置 LINK 信号来自 IO(PHY LED) 及极性。
 *
 * @note 会占用 PA16~PA31 / PV00~PV15 / PW00~PW21 —— 电机应用(PWM=PD, ADC=PF、
 *       编码器=PC/PE)不冲突; 但不要同时调用 init_tsw_pins()(那组脚的另一套复用)。
 */
void bsp_ecat_hw_init(void);

/**
 * @brief 采样一次完整状态(不自检、不打印)
 * @param st 输出; 不能为 NULL
 */
void bsp_ecat_read_status(bsp_ecat_status_t *st);

/**
 * @brief 清掉 PHY_STAT 里的粘滞标志(PCU/RE)
 *
 * 复位释放期间 ESC 读 PHY 会失败并置 RE, 那是历史事件。采样前调一次, 之后
 * phy_re 就只反映"清完之后"的情况。
 */
void bsp_ecat_clear_phy_stat(void);

/**
 * @brief ESC 能力寄存器一行文本 (Type/Rev/FMMU/SM/RAM/端口描述/PDI/DL_STAT)
 * @return snprintf 的返回值 (>=len 表示被截断)
 */
int bsp_ecat_format_info(char *buf, size_t len);

/**
 * @brief 单个端口的 PHY + ESC 链路状态一行文本
 * @param port 0 / 1
 * @return snprintf 的返回值
 */
int bsp_ecat_format_port(const bsp_ecat_status_t *st, uint32_t port, char *buf, size_t len);

/**
 * @brief 扫一遍 MDIO 地址 0~31, 报告哪些地址上的 PHY 有应答
 *
 * 用于 board.h 里假设的 PHY 地址(0/1)不成立时定位实际地址。
 * @param found_mask 输出位图, bit n = 地址 n 有应答
 * @return 有应答的 PHY 个数
 */
uint32_t bsp_ecat_scan_mdio(uint32_t *found_mask);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ECAT_H */
