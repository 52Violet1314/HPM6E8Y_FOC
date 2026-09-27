/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: MT6701 专用 SPI 主机实现 (SPI2 @ J11 扩展排针)
 *
 * 【与 STM32 对照】
 *   STM32                                        HPM6E8Y
 *   ----------------------------------------------------------------
 *   使能 SPI 时钟                                  clock_add_to_group(clock_spi2, 0)
 *   GPIO 初始化 (AF 复用 SCK/MISO, 推挽 CS)        HPM_IOC->PAD[...].FUNC_CTL/PAD_CTL
 *   SPI_Init (CPOL/CPHA/FirstBit/Prescaler)       spi_format_config_t + spi_format_init()
 *                                                 spi_timing_config_t + spi_master_timing_init()
 *   HAL_SPI_Receive (软件 NSS 拉低/拉高)           gpio_write_pin() + spi_transfer()
 */
#include "bsp_spi.h"
#include "bsp_spi_bb.h"      /* bit-bang 兜底后端 */

#include "hpm_clock_drv.h"
#include "hpm_spi_drv.h"

/* 单帧最大字节数 (MT6701 只需 3) */
#define BSP_SPI_MAX_FRAME_BYTES  (8U)

/* --------------------------------------------------------------------------
 * 引脚复用
 * -------------------------------------------------------------------------- */
static void bsp_spi_init_pins(void)
{
    /* SCLK: SPI2 主机输出 */
    HPM_IOC->PAD[BSP_SPI_PAD_SCLK].FUNC_CTL = IOC_PC26_FUNC_CTL_SPI2_SCLK;
    HPM_IOC->PAD[BSP_SPI_PAD_SCLK].PAD_CTL  = IOC_PAD_PAD_CTL_SR_SET(1) |
                                              IOC_PAD_PAD_CTL_SPD_SET(3);

    /* MISO: 编码器数据输入。
     * 上拉加强到 22k (PRS=0b10): 若 MT6701 的 DOUT 是开漏输出, 这是唯一的
     * 上升沿来源; 100k 太弱, 在 SCLK 边沿上根本抬不起来。 */
    HPM_IOC->PAD[BSP_SPI_PAD_MISO].FUNC_CTL = IOC_PC28_FUNC_CTL_SPI2_MISO;
    HPM_IOC->PAD[BSP_SPI_PAD_MISO].PAD_CTL  = IOC_PAD_PAD_CTL_SR_SET(1) |
                                              IOC_PAD_PAD_CTL_SPD_SET(3) |
                                              IOC_PAD_PAD_CTL_PRS_SET(2) |
                                              IOC_PAD_PAD_CTL_PE_SET(1) |
                                              IOC_PAD_PAD_CTL_PS_SET(1);

    /* CS: 当普通 GPIO 输出用 (低有效), 上电先拉高 -> 不选中 */
    HPM_IOC->PAD[BSP_SPI_PAD_CS].FUNC_CTL = IOC_PC24_FUNC_CTL_GPIO_C_24;
    HPM_IOC->PAD[BSP_SPI_PAD_CS].PAD_CTL  = IOC_PAD_PAD_CTL_SR_SET(1) |
                                            IOC_PAD_PAD_CTL_SPD_SET(3) |
                                            IOC_PAD_PAD_CTL_PE_SET(1) |
                                            IOC_PAD_PAD_CTL_PS_SET(1);

    gpio_set_pin_output_with_initial(BSP_SPI_CS_GPIO_CTRL,
                                     BSP_SPI_CS_GPIO_PORT,
                                     (uint8_t)BSP_SPI_CS_GPIO_PIN,
                                     1U);
}

/* --------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------- */

/* 运行期诊断: 供 app 打一行日志, 确认源时钟/实际 SCLK/时序初始化结果 */
static uint32_t   s_spi_src_clk;
static uint32_t   s_spi_sclk;
static hpm_stat_t s_spi_timing_stat;

/* 驱动对分频的硬性要求 (见 spi_master_timing_init):
 *     div = clk_src / sclk 必须 ① 整除 ② 偶数 ③ <= 510
 * 即 sclk = clk_src / div, div 是 (0,510] 内的偶数因子。
 * 换句话说 SCLK 不能随便设低 —— 下限约 clk_src/510 (源时钟 200MHz 时
 * 约 392kHz)。若直接把非法频率丢给驱动, 它会返回 status_invalid_argument
 * 且【不写 TIMING 寄存器】, 结果是一个无关的分频值留在寄存器里。
 * 这里在合法值里挑一个最接近目标的, 保证一定合法。 */
static uint32_t bsp_spi_pick_sclk(uint32_t clk_src_hz, uint32_t target_hz)
{
    uint32_t best = 0U;
    uint32_t best_err = 0xFFFFFFFFU;

    if ((clk_src_hz == 0U) || (target_hz == 0U)) {
        return 0U;
    }

    for (uint32_t div = 2U; div <= 510U; div += 2U) {
        uint32_t sclk;
        uint32_t err;

        if ((clk_src_hz % div) != 0U) {
            continue;
        }
        sclk = clk_src_hz / div;
        err  = (sclk > target_hz) ? (sclk - target_hz) : (target_hz - sclk);
        if (err < best_err) {
            best_err = err;
            best     = sclk;
        }
    }
    return best;
}

void bsp_spi_init(void)
{
    spi_timing_config_t timing_config;
    spi_format_config_t format_config;

    /* [1] SPI2 时钟挂到 CPU0 组, 使能该外设时钟 */
    clock_add_to_group(BSP_SPI_CLOCK_NAME, 0);

    /* [2] 引脚复用 */
    bsp_spi_init_pins();

    /* [3] 时序: 源时钟 -> 挑一个驱动能接受的 SCLK -> 写 TIMING */
    s_spi_src_clk = clock_get_frequency(BSP_SPI_CLOCK_NAME);
    s_spi_sclk    = bsp_spi_pick_sclk(s_spi_src_clk, BSP_SPI_SCLK_FREQ_HZ);

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = s_spi_src_clk;
    timing_config.master_config.sclk_freq_in_hz    = s_spi_sclk;
    s_spi_timing_stat = spi_master_timing_init(BSP_SPI_BASE, &timing_config);

    /* [4] 格式: 8bit/帧, MSB first, 主机, CPOL=0 / CPHA=0  (SPI Mode 0)
     *
     * 【注意: 这里曾经写成 CPHA=1 (Mode 1), 是错的】
     *   3 个 STM32 参考工程都写的是 Mode 1, 照抄后现象是硬件 SPI 恒读
     *   0xFFFFFF, 但 GPIO bit-bang 扫描 (bsp_spi_bb.c) 把 16 种组合全试了一遍,
     *   唯一能读出 CRC6 合法帧的是 idx=8:
     *       CS 每帧翻转 + CPOL=0 + CPHA=0 + MSB first
     *   (读回 0xAF7888, 低 6 位 CRC=8, 按 poly 0x43 对高 18 位算出的 CRC6 也是 8)
     *   所以实测这个 MT6701 要的是 Mode 0, 不是 Mode 1。 */
    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = 8;
    format_config.common_config.mode             = spi_master_mode;
    format_config.common_config.cpol             = spi_sclk_low_idle;
    format_config.common_config.cpha             = spi_sclk_sampling_odd_clk_edges;
    format_config.common_config.lsb              = false;
    spi_format_init(BSP_SPI_BASE, &format_config);
}

/* --------------------------------------------------------------------------
 * 收发 —— 阻塞读 (等价于 STM32 的 HAL_SPI_Receive)
 *
 *   对应关系:
 *     STM32: HAL_SPI_Receive(&hspi, buf, len, timeout)   同步轮询, 有界超时
 *     HPM  : spi_transfer(base, cfg, NULL, NULL, tx, len, rx, len)
 *   两者都是"收满即返回 / 超时即报错"的同步阻塞调用, 驱动内部用
 *   HPM_SPI_DRV_DEFAULT_RETRY_COUNT 做有界重试, 不会死等。
 *
 *   HPM 主机读必须自己写 dummy 才会产生 SCLK (STM32 的 receive-only 是硬件
 *   自动补 dummy, HPM 没有等价自动机制), 所以发送侧统一补 0xFF。
 *
 *   【教训】不要在这里自己手写 "写一个字节 -> 忙等 RX" 的紧循环: 那种无界
 *   忙等在 SPI 收不满时会空转上千万次, 而本函数每个 1kHz 控制周期都被调用,
 *   会把整个主循环拖垮 (现象就是"卡死")。
 * -------------------------------------------------------------------------- */
bool bsp_spi_read(uint8_t *rx, uint32_t len)
{
    spi_control_config_t ctrl_config;
    uint8_t    tx_dummy[BSP_SPI_MAX_FRAME_BYTES];
    hpm_stat_t stat;
    uint32_t   i;

    /* 若 bit-bang 后端已被激活 (扫描找到了能工作的组合, 或硬件 SPI 被判定
     * 走不通), 就直接交给软件 SPI —— 两条路读出来都是 3 字节大端, 可互换。 */
    if (bsp_bb_is_active()) {
        return bsp_bb_read(rx, len);
    }

    if ((rx == NULL) || (len == 0U) || (len > BSP_SPI_MAX_FRAME_BYTES)) {
        return false;
    }

    /* MT6701 只读, MOSI 上没有真实数据, 统一送 0xFF 换取 SCLK */
    for (i = 0U; i < len; i++) {
        tx_dummy[i] = 0xFFU;
    }

    spi_master_get_default_control_config(&ctrl_config);
    ctrl_config.common_config.trans_mode     = spi_trans_write_read_together;
    ctrl_config.common_config.data_phase_fmt = spi_single_io_mode;
    ctrl_config.common_config.tx_dma_enable  = false;
    ctrl_config.common_config.rx_dma_enable  = false;

    /* 软件片选: 整个 24bit 帧期间保持低电平 */
    gpio_write_pin(BSP_SPI_CS_GPIO_CTRL, BSP_SPI_CS_GPIO_PORT,
                   (uint8_t)BSP_SPI_CS_GPIO_PIN, 0U);

    stat = spi_transfer(BSP_SPI_BASE, &ctrl_config, NULL, NULL,
                        tx_dummy, len, rx, len);

    gpio_write_pin(BSP_SPI_CS_GPIO_CTRL, BSP_SPI_CS_GPIO_PORT,
                   (uint8_t)BSP_SPI_CS_GPIO_PIN, 1U);

    return (stat == status_success);
}

/* --------------------------------------------------------------------------
 * 诊断查询
 * -------------------------------------------------------------------------- */
uint32_t bsp_spi_get_src_clk(void)
{
    return s_spi_src_clk;
}

uint32_t bsp_spi_get_sclk(void)
{
    return s_spi_sclk;
}

int bsp_spi_get_timing_stat(void)
{
    return (int)s_spi_timing_stat;
}
