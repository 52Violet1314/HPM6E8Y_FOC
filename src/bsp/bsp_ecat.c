/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 内置 EtherCAT(ESC) 硬件层实现 —— 见 bsp_ecat.h 顶部的硬件说明。
 */

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "hpm_esc_drv.h"
#include "hpm_gpio_drv.h"

#include "bsp_ecat.h"

/* board.h 里两个口的 PHY 地址 (实际地址 = 偏移 + 端口地址) */
#define BSP_ECAT_PHY_ADDR(p)   ((uint8_t)(BOARD_ECAT_PHY_ADDR_OFFSET + \
                                ((p) == 0U ? BOARD_ECAT_PORT0_PHY_ADDR : BOARD_ECAT_PORT1_PHY_ADDR)))

/* PHY 标准寄存器 (IEEE 802.3 clause 22) */
#define BSP_ECAT_PHY_REG_BMCR  (0U)
#define BSP_ECAT_PHY_REG_BMSR  (1U)
#define BSP_ECAT_PHY_REG_ID1   (2U)
#define BSP_ECAT_PHY_REG_ID2   (3U)

/* BMSR 位 (IEEE 802.3) */
#define BMSR_LINK_STATUS       (1U << 2)
#define BMSR_10M_HALF          (1U << 11)
#define BMSR_10M_FULL          (1U << 12)
#define BMSR_100M_HALF         (1U << 13)
#define BMSR_100M_FULL         (1U << 14)

static bool s_inited;

static void bsp_ecat_set_phy_reset(bool assert_reset)
{
    uint8_t level = assert_reset ? BOARD_ECAT_PHY_RESET_LEVEL
                                 : (uint8_t)(!BOARD_ECAT_PHY_RESET_LEVEL);

    /* 复位脚就是普通 GPIO (PHY0=PV12, PHY1=PW12), 板级宏给的是引脚号 */
    gpio_set_pin_output_with_initial(BOARD_ECAT_PHY0_RESET_GPIO,
                                     BOARD_ECAT_PHY0_RESET_GPIO_PORT_INDEX,
                                     BOARD_ECAT_PHY0_RESET_PIN_INDEX, level);
    gpio_set_pin_output_with_initial(BOARD_ECAT_PHY1_RESET_GPIO,
                                     BOARD_ECAT_PHY1_RESET_GPIO_PORT_INDEX,
                                     BOARD_ECAT_PHY1_RESET_PIN_INDEX, level);
}

void bsp_ecat_hw_init(void)
{
    esc_eeprom_clock_config_t cfg;

    if (s_inited) {
        return;
    }

    /* 1) 外设时钟 + ESC 全部引脚 + 两个 PHY 复位脚。
     *    【注意】这一步结束时 PHY 仍停在复位态(板级代码刻意如此)。 */
    board_init_ethercat(HPM_ESC);

    /* 2) EEPROM 模拟(板上没有 EEPROM, 必须模拟) + ESC 内核 100M + PHY 25M 参考时钟。
     *    不做这一步: PHY 没参考时钟, 永远不会有 link。 */
    cfg.eeprom_emulation      = true;
    cfg.eeprom_size_over_16kbit = false;
    cfg.core_clock_en         = true;
    cfg.phy_refclk_en         = true;
    esc_config_eeprom_and_clock(HPM_ESC, &cfg);

    /* 3) 释放 PHY 复位 */
    bsp_ecat_set_phy_reset(false);

    /* 4) 两个口的 LINK 信号来自 IO(PHY 的 LED 输出经 ESC0_CTR_x 回灌),
     *    不是 GPR 寄存器里的固定值; 极性由板级宏给出(LED 低有效 -> invert)。 */
    esc_config_nmii_link_source(HPM_ESC, true, true, false);
    esc_config_ctrl_signal_function(HPM_ESC, (uint8_t)BOARD_ECAT_NMII_LINK0_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link0,
                                    BOARD_ECAT_PORT0_LINK_INVERT);
    esc_config_ctrl_signal_function(HPM_ESC, (uint8_t)BOARD_ECAT_NMII_LINK1_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link1,
                                    BOARD_ECAT_PORT1_LINK_INVERT);

    s_inited = true;
}

void bsp_ecat_read_status(bsp_ecat_status_t *st)
{
    uint16_t dl;
    uint32_t p;

    if (st == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));

    for (p = 0U; p < BSP_ECAT_PORT_CNT; p++) {
        uint8_t addr = BSP_ECAT_PHY_ADDR(p);
        uint16_t id1 = 0U, id2 = 0U;

        /* 片内 PHY 的状态: ESC 自己一直在轮询 PHY 并缓存到这里, PDI 侧读 MDIO
         * 是读不到片内 PHY 的(它不在那条总线上), 所以这一组才是权威判据。 */
        st->phy_stat[p] = HPM_ESC->PHY_STAT[p];
        st->phy_pls[p]  = ESC_PHY_STAT_PLS_GET(st->phy_stat[p]) != 0U;
        st->phy_ls[p]   = ESC_PHY_STAT_LS_GET(st->phy_stat[p]) != 0U;
        st->phy_lse[p]  = ESC_PHY_STAT_LSE_GET(st->phy_stat[p]) != 0U;
        st->phy_re[p]   = ESC_PHY_STAT_RE_GET(st->phy_stat[p]) != 0U;
        st->phy_lpe[p]  = ESC_PHY_STAT_LPE_GET(st->phy_stat[p]) != 0U;

        (void)esc_mdio_read(HPM_ESC, addr, BSP_ECAT_PHY_REG_BMCR, &st->bmcr[p]);
        (void)esc_mdio_read(HPM_ESC, addr, BSP_ECAT_PHY_REG_BMSR, &st->bmsr[p]);
        (void)esc_mdio_read(HPM_ESC, addr, BSP_ECAT_PHY_REG_ID1, &id1);
        (void)esc_mdio_read(HPM_ESC, addr, BSP_ECAT_PHY_REG_ID2, &id2);

        st->phy_id1[p] = id1;
        st->phy_id2[p] = id2;
        /* PHYID 全 0 或全 1 视为"没有器件应答"(悬空 MDIO 的典型读数) */
        st->mdio_ok[p] = ((id1 != 0U) || (id2 != 0U)) && (id1 != 0xFFFFU) && (id2 != 0xFFFFU);

        st->phy_link[p] = (st->bmsr[p] & BMSR_LINK_STATUS) != 0U;
        st->phy_100m[p] = (st->bmsr[p] & (BMSR_100M_HALF | BMSR_100M_FULL)) != 0U;
        if (st->phy_100m[p]) {
            st->phy_fd[p] = (st->bmsr[p] & BMSR_100M_FULL) != 0U;
        } else {
            st->phy_fd[p] = (st->bmsr[p] & BMSR_10M_FULL) != 0U;
        }

        st->rx_err_cnt[p]    = HPM_ESC->RX_ERR_CNT[p];
        st->lost_link_cnt[p] = HPM_ESC->LOST_LINK_CNT[p];
    }

    /* ESC 自己的链路/环回/通信状态 */
    dl = HPM_ESC->ESC_DL_STAT;
    st->dl_stat = dl;

    st->esc_link[0] = ESC_ESC_DL_STAT_PLP0_GET(dl) != 0U;
    st->esc_link[1] = ESC_ESC_DL_STAT_PLP1_GET(dl) != 0U;
    st->esc_loop[0] = ESC_ESC_DL_STAT_LP0_GET(dl) != 0U;
    st->esc_loop[1] = ESC_ESC_DL_STAT_LP1_GET(dl) != 0U;
    st->esc_comm[0] = ESC_ESC_DL_STAT_CP0_GET(dl) != 0U;
    st->esc_comm[1] = ESC_ESC_DL_STAT_CP1_GET(dl) != 0U;
    st->eeprom_loaded = ESC_ESC_DL_STAT_EPLC_GET(dl) != 0U;
}

int bsp_ecat_format_info(char *buf, size_t len)
{
    if ((buf == NULL) || (len == 0U)) {
        return 0;
    }

    return snprintf(buf, len,
                    "ESC type=0x%02X rev=0x%02X FMMU=%u SM=%u RAM=%uKB port_desc=0x%02X PDI=0x%02X "
                    "| DL_STAT=0x%04X EEPROM_loaded=%u | NLINK0_padsel=%u NLINK1_padsel=%u",
                    (unsigned)HPM_ESC->TYPE,
                    (unsigned)HPM_ESC->REVISION,
                    (unsigned)ESC_FMMU_NUM_NUM_GET(HPM_ESC->FMMU_NUM),
                    (unsigned)ESC_SYNCM_NUM_NUM_GET(HPM_ESC->SYNCM_NUM),
                    (unsigned)ESC_RAM_SIZE_SIZE_GET(HPM_ESC->RAM_SIZE),
                    (unsigned)HPM_ESC->PORT_DESC,
                    (unsigned)ESC_PDI_CTRL_PDI_GET(HPM_ESC->PDI_CTRL),
                    (unsigned)HPM_ESC->ESC_DL_STAT,
                    (unsigned)ESC_ESC_DL_STAT_EPLC_GET(HPM_ESC->ESC_DL_STAT),
                    (unsigned)ESC_GPR_STATUS_NLINK0_PADSEL_GET(HPM_ESC->GPR_STATUS),
                    (unsigned)ESC_GPR_STATUS_NLINK1_PADSEL_GET(HPM_ESC->GPR_STATUS));
}

int bsp_ecat_format_port(const bsp_ecat_status_t *st, uint32_t port, char *buf, size_t len)
{
    if ((st == NULL) || (buf == NULL) || (len == 0U) || (port >= BSP_ECAT_PORT_CNT)) {
        return 0;
    }

    if (st->mdio_ok[port]) {
        /* 外挂 PHY 的场合: MDIO 能读到, 信息更全 */
        return snprintf(buf, len,
                        "P%u: PHY_STAT=0x%02X[pls=%u ls=%u lse=%u re=%u lpe=%u] "
                        "ESC[plp=%u lp=%u cp=%u] | MDIO ok phyid=0x%04X%04X bmsr=0x%04X "
                        "| rxerr=%u lost=%u eeprom=%u",
                        (unsigned)port, (unsigned)st->phy_stat[port],
                        st->phy_pls[port] ? 1U : 0U, st->phy_ls[port] ? 1U : 0U,
                        st->phy_lse[port] ? 1U : 0U, st->phy_re[port] ? 1U : 0U,
                        st->phy_lpe[port] ? 1U : 0U,
                        st->esc_link[port] ? 1U : 0U, st->esc_loop[port] ? 1U : 0U,
                        st->esc_comm[port] ? 1U : 0U,
                        (unsigned)st->phy_id1[port], (unsigned)st->phy_id2[port],
                        (unsigned)st->bmsr[port],
                        (unsigned)st->rx_err_cnt[port], (unsigned)st->lost_link_cnt[port],
                        st->eeprom_loaded ? 1U : 0U);
    }

    /* 片内 PHY: MDIO 无应答是常态。PHY_STAT 也读不到(恒 0)时, 说明这颗芯片的
     * 片内 PHY 没有对外暴露寄存器通道 —— 链路只能看 ESC 的 PLP(LINK 由 PHY 的
     * LED 输出经 ESC0_CTR_x 回灌)。这不是故障, 也不影响 EtherCAT 工作。 */
    return snprintf(buf, len,
                    "P%u: PHY_STAT=0x%02X[pls=%u ls=%u lse=%u re=%u lpe=%u]%s "
                    "ESC[plp=%u lp=%u cp=%u] | MDIO 无应答(片内 PHY, 正常) "
                    "| rxerr=%u lost=%u eeprom=%u",
                    (unsigned)port, (unsigned)st->phy_stat[port],
                    st->phy_pls[port] ? 1U : 0U, st->phy_ls[port] ? 1U : 0U,
                    st->phy_lse[port] ? 1U : 0U, st->phy_re[port] ? 1U : 0U,
                    st->phy_lpe[port] ? 1U : 0U,
                    (st->phy_stat[port] == 0U) ? " (无 PHY 寄存器通道)" : "",
                    st->esc_link[port] ? 1U : 0U, st->esc_loop[port] ? 1U : 0U,
                    st->esc_comm[port] ? 1U : 0U,
                    (unsigned)st->rx_err_cnt[port], (unsigned)st->lost_link_cnt[port],
                    st->eeprom_loaded ? 1U : 0U);
}

void bsp_ecat_clear_phy_stat(void)
{
    uint32_t p;

    /* 写任意值即可清 PCU/RE 两个粘滞位 */
    for (p = 0U; p < 4U; p++) {
        HPM_ESC->PHY_STAT[p] = 0U;
    }
}

uint32_t bsp_ecat_scan_mdio(uint32_t *found_mask)
{
    uint32_t addr;
    uint32_t count = 0U;
    uint32_t mask  = 0U;

    for (addr = 0U; addr < 32U; addr++) {
        uint16_t id1 = 0U, id2 = 0U;

        (void)esc_mdio_read(HPM_ESC, (uint8_t)addr, BSP_ECAT_PHY_REG_ID1, &id1);
        (void)esc_mdio_read(HPM_ESC, (uint8_t)addr, BSP_ECAT_PHY_REG_ID2, &id2);

        if (((id1 != 0U) || (id2 != 0U)) && (id1 != 0xFFFFU) && (id2 != 0xFFFFU)) {
            mask |= (1UL << addr);
            count++;
        }
    }

    if (found_mask != NULL) {
        *found_mask = mask;
    }
    return count;
}
