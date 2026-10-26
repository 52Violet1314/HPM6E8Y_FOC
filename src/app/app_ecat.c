/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 内置 EtherCAT(ESC) 开箱自检 + 回环链路测试 (实现)
 *
 * 这一层只负责"跑一遍 + 打日志 + 给结论", 硬件细节都在 bsp_ecat.c。
 */

#include <stdio.h>

#include "board.h"          /* board_delay_ms */
#include "bsp_ecat.h"
#include "app_log.h"

#include "app_ecat.h"

#define ECAT_TAG            "ECAT"

/* 释放 PHY 复位后等待自协商的时间 (100BASE-TX 一般 200~800ms) */
#define ECAT_AUTONEG_MS     (500U)

/* 把一次采样的两个口都打出来 */
static void ecat_log_ports(const bsp_ecat_status_t *st)
{
    char line[224];
    uint32_t p;

    for (p = 0U; p < BSP_ECAT_PORT_CNT; p++) {
        (void)bsp_ecat_format_port(st, p, line, sizeof(line));
        LOG_I(ECAT_TAG, "%s", line);
    }
}

/* 状态指纹: 只关心"变没变" */
static uint32_t ecat_sig(const bsp_ecat_status_t *st)
{
    return ((st->phy_ls[0] ? 1UL : 0UL) | (st->phy_ls[1] ? 2UL : 0UL)
            | (st->esc_link[0] ? 4UL : 0UL) | (st->esc_link[1] ? 8UL : 0UL)
            | (st->esc_loop[0] ? 16UL : 0UL) | (st->esc_loop[1] ? 32UL : 0UL)
            | (st->esc_comm[0] ? 64UL : 0UL) | (st->esc_comm[1] ? 128UL : 0UL));
}

/* 判定"这个口有没有 link": ESC 的 PLP 和片内 PHY 的 PHY_STAT 任一为真就算。
 * 【不要用 MDIO 做判据】片内 PHY 不挂在 PDI 侧的 MDIO 上, 永远读不到。 */
static uint32_t ecat_port_linked(const bsp_ecat_status_t *st, uint32_t p)
{
    return (st->esc_link[p] || st->phy_ls[p] || st->phy_pls[p]) ? 1UL : 0UL;
}

static void ecat_verdict(const bsp_ecat_status_t *st)
{
    uint32_t links = ecat_port_linked(st, 0U) + ecat_port_linked(st, 1U);

    if (links == 2U) {
        LOG_I(ECAT_TAG, "结论: 两个口都 link —— 回环线接法正确");
        LOG_I(ECAT_TAG, "      判据: ESC DL_STAT plp=%u/%u lp=%u/%u cp=%u/%u (DL_STAT=0x%04X);"
                        " LINK 是 PHY 的 LED 输出经 ESC0_CTR_x 回灌进 ESC 的, 这条链通了",
              st->esc_link[0] ? 1U : 0U, st->esc_link[1] ? 1U : 0U,
              st->esc_loop[0] ? 1U : 0U, st->esc_loop[1] ? 1U : 0U,
              st->esc_comm[0] ? 1U : 0U, st->esc_comm[1] ? 1U : 0U,
              (unsigned)st->dl_stat);
        if ((st->phy_stat[0] == 0U) && (st->phy_stat[1] == 0U)) {
            LOG_I(ECAT_TAG, "      片内 PHY 没有寄存器通道(MDIO 与 PHY_STAT 都读不到),"
                            " 所以看不到速率/双工寄存器值 —— 不影响 EtherCAT 工作");
        }
        LOG_I(ECAT_TAG, "      还没有主站发帧: 报文(PDO)通路要接主站才算验证;"
                        " EEPROM_loaded=0 是没跑协议栈的正常状态");
        return;
    }

    if (links == 0U) {
        LOG_W(ECAT_TAG, "结论: 两个口都没有 link");
        if ((st->phy_re[0] || st->phy_re[1]) && (!st->phy_pls[0] && !st->phy_pls[1])) {
            LOG_W(ECAT_TAG, "      PHY_STAT re=1 且无物理 link: 优先查 PHY 复位(PV12/PW12 是否已拉高)"
                            " 和 25M 参考时钟");
        } else if (st->phy_lse[0] || st->phy_lse[1]) {
            LOG_W(ECAT_TAG, "      PHY_STAT lse=1(link 被抑制/出错): 查网线质量与对端口");
        } else {
            LOG_W(ECAT_TAG, "      查: 网线是否真的接在 J1<->J2 两个 EtherCAT 口上");
        }
        return;
    }

    LOG_W(ECAT_TAG, "结论: 只有 %u 个口 link —— 大概率有一根线/一个口不通", (unsigned)links);
}

void app_ecat_dump_status(void)
{
    char line[224];
    bsp_ecat_status_t st;

    bsp_ecat_format_info(line, sizeof(line));
    LOG_I(ECAT_TAG, "%s", line);

    bsp_ecat_read_status(&st);
    ecat_log_ports(&st);
}

void app_ecat_selftest(void)
{
    char line[224];
    bsp_ecat_status_t st;
    uint32_t mask = 0U;
    uint32_t found;

    LOG_I(ECAT_TAG, "---- 内置 EtherCAT(ESC) 自检 ----");
    LOG_I(ECAT_TAG, "回环测试接线: 网线把 J1 <-> J2 (P0 <-> P1) 对接");

    /* 1) 硬件点亮: 时钟 + 引脚 + EEPROM 模拟 + 放 PHY 复位 + 配 LINK 来源 */
    bsp_ecat_hw_init();

    /* 2) 等自协商 */
    board_delay_ms(ECAT_AUTONEG_MS);

    /* 3) 清掉复位期间留下的粘滞标志(PHY_STAT.re), 之后的 re=1 才是"现在读不到" */
    bsp_ecat_clear_phy_stat();
    board_delay_ms(20U);

    /* 4) ESC 能力寄存器 */
    (void)bsp_ecat_format_info(line, sizeof(line));
    LOG_I(ECAT_TAG, "%s", line);

    /* 5) 两个口的状态快照 */
    bsp_ecat_read_status(&st);
    ecat_log_ports(&st);

    /* 6) PDI 侧 MDIO 扫一遍(片内 PHY 不在这条总线上, 0 个是正常的; 留着是为了
     *    将来外挂 PHY 时能一眼看出地址) */
    if ((!st.mdio_ok[0]) && (!st.mdio_ok[1])) {
        found = bsp_ecat_scan_mdio(&mask);
        LOG_I(ECAT_TAG, "MDIO(PDI 侧)0~31 扫描: %u 个器件 —— 片内 PHY 不走这条总线, 0 个属正常",
              (unsigned)found);
    }

#if (APP_ECAT_LINK_WATCH_MS > 0)
    {
        uint32_t t;
        uint32_t last = 0xFFFFFFFFUL;

        LOG_I(ECAT_TAG, "链路监视 %u ms (每 %u ms 采样, 变化才打印) —— 现在可插拔网线",
              (unsigned)APP_ECAT_LINK_WATCH_MS, (unsigned)APP_ECAT_LINK_WATCH_STEP_MS);

        for (t = 0U; t <= APP_ECAT_LINK_WATCH_MS; t += APP_ECAT_LINK_WATCH_STEP_MS) {
            uint32_t sig;

            bsp_ecat_read_status(&st);
            sig = ecat_sig(&st);
            if (sig != last) {
                last = sig;
                LOG_I(ECAT_TAG, "t=%4ums PHYls=[%u %u] ESClink=[%u %u] ESCloop=[%u %u] ESCcp=[%u %u]",
                      (unsigned)t,
                      st.phy_ls[0] ? 1U : 0U, st.phy_ls[1] ? 1U : 0U,
                      st.esc_link[0] ? 1U : 0U, st.esc_link[1] ? 1U : 0U,
                      st.esc_loop[0] ? 1U : 0U, st.esc_loop[1] ? 1U : 0U,
                      st.esc_comm[0] ? 1U : 0U, st.esc_comm[1] ? 1U : 0U);
                ecat_log_ports(&st);
            }
            board_delay_ms(APP_ECAT_LINK_WATCH_STEP_MS);
        }

        bsp_ecat_read_status(&st);
    }
#endif

    /* 7) 结论只出一次(放在监视之后, 免得链路还没起来就下结论) */
    ecat_verdict(&st);

    LOG_I(ECAT_TAG, "---- 自检结束 ----");
}
