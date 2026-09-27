/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: GPIO 软件模拟 SPI (bit-bang) 实现 —— 见 bsp_spi_bb.h
 */

#include "bsp_spi_bb.h"

#include <stdio.h>

/* --------------------------------------------------------------------------
 * 低层引脚操作
 * -------------------------------------------------------------------------- */

static inline void bb_sclk_write(uint8_t high)
{
    gpio_write_pin(BSP_BB_GPIO_CTRL, BSP_BB_GPIO_PORT,
                   (uint8_t)BSP_BB_SCLK_GPIO_PIN, (high != 0U) ? 1U : 0U);
}

static inline uint8_t bb_miso_read(void)
{
    return (gpio_read_pin(BSP_BB_GPIO_CTRL, BSP_BB_GPIO_PORT,
                          (uint8_t)BSP_BB_MISO_GPIO_PIN) != 0U) ? 1U : 0U;
}

static inline void bb_cs_write(uint8_t high)
{
    gpio_write_pin(BSP_BB_GPIO_CTRL, BSP_BB_GPIO_PORT,
                   (uint8_t)BSP_BB_CS_GPIO_PIN, (high != 0U) ? 1U : 0U);
}

/** 把三根脚从 SPI2 复用切成普通 GPIO */
static void bb_pins_to_gpio(void)
{
    /* SCLK: 推挽输出, 起始低 */
    HPM_IOC->PAD[BSP_SPI_PAD_SCLK].FUNC_CTL = IOC_PC26_FUNC_CTL_GPIO_C_26;
    HPM_IOC->PAD[BSP_SPI_PAD_SCLK].PAD_CTL  = IOC_PAD_PAD_CTL_SR_SET(1) |
                                              IOC_PAD_PAD_CTL_SPD_SET(3);
    gpio_set_pin_output_with_initial(BSP_BB_GPIO_CTRL, BSP_BB_GPIO_PORT,
                                     (uint8_t)BSP_BB_SCLK_GPIO_PIN, 0U);

    /* MISO: 输入 + 22k 上拉 (与硬件 SPI 路径保持一致, 便于对比) */
    HPM_IOC->PAD[BSP_SPI_PAD_MISO].FUNC_CTL = IOC_PC28_FUNC_CTL_GPIO_C_28;
    HPM_IOC->PAD[BSP_SPI_PAD_MISO].PAD_CTL  = IOC_PAD_PAD_CTL_SR_SET(1) |
                                              IOC_PAD_PAD_CTL_SPD_SET(3) |
                                              IOC_PAD_PAD_CTL_PRS_SET(2) |
                                              IOC_PAD_PAD_CTL_PE_SET(1) |
                                              IOC_PAD_PAD_CTL_PS_SET(1);
    gpio_set_pin_input(BSP_BB_GPIO_CTRL, BSP_BB_GPIO_PORT,
                       (uint8_t)BSP_BB_MISO_GPIO_PIN);

    /* CS: 推挽输出, 先不选中 */
    HPM_IOC->PAD[BSP_SPI_PAD_CS].FUNC_CTL = IOC_PC24_FUNC_CTL_GPIO_C_24;
    HPM_IOC->PAD[BSP_SPI_PAD_CS].PAD_CTL  = IOC_PAD_PAD_CTL_SR_SET(1) |
                                            IOC_PAD_PAD_CTL_SPD_SET(3);
    gpio_set_pin_output_with_initial(BSP_BB_GPIO_CTRL, BSP_BB_GPIO_PORT,
                                     (uint8_t)BSP_BB_CS_GPIO_PIN, 1U);
}

/* --------------------------------------------------------------------------
 * CRC6 (MT6701: 多项式 0x43, 覆盖帧内高 18 位, 低 6 位是校验值)
 * -------------------------------------------------------------------------- */
static uint8_t bb_crc6(uint32_t data18)
{
    uint8_t crc = 0U;
    int32_t i;

    for (i = 17; i >= 0; i--) {
        uint8_t in_bit = (uint8_t)((data18 >> (uint32_t)i) & 1U);
        uint8_t fb     = (uint8_t)(((crc >> 5) & 1U) ^ in_bit);

        crc = (uint8_t)((crc << 1) & 0x3FU);
        if (fb != 0U) {
            crc ^= 0x03U;
        }
    }
    return crc;
}

/* --------------------------------------------------------------------------
 * 软件 SPI 读一帧
 *
 *   边沿约定:
 *     leading  edge = 空闲电平 -> 有效电平
 *     trailing edge = 有效电平 -> 空闲电平
 *     CPHA = 0 -> 在 leading  edge 之后采样
 *     CPHA = 1 -> 在 trailing edge 之后采样
 * -------------------------------------------------------------------------- */
static uint32_t bb_read_frame(const bsp_bb_mode_t *m)
{
    uint32_t raw = 0U;
    uint32_t i;

    if (m->cs_toggle != 0U) {
        bb_cs_write(0U);              /* 选中 */
        board_delay_us(2U);
    }

    bb_sclk_write(m->cpol);           /* 先回到空闲电平 */
    board_delay_us(BSP_BB_HALF_US);

    for (i = 0U; i < BSP_BB_FRAME_BITS; i++) {
        uint8_t bit = 0U;

        /* leading edge */
        bb_sclk_write((m->cpol != 0U) ? 0U : 1U);
        board_delay_us(BSP_BB_HALF_US);
        if (m->cpha == 0U) {
            bit = bb_miso_read();
        }

        /* trailing edge */
        bb_sclk_write(m->cpol);
        board_delay_us(BSP_BB_HALF_US);
        if (m->cpha != 0U) {
            bit = bb_miso_read();
        }

        if (m->lsb_first != 0U) {
            raw |= ((uint32_t)bit << i);
        } else {
            raw = (raw << 1) | (uint32_t)bit;
        }
    }

    if (m->cs_toggle != 0U) {
        board_delay_us(2U);
        bb_cs_write(1U);              /* 释放 */
    }

    return (raw & 0xFFFFFFU);
}

/* --------------------------------------------------------------------------
 * 扫描
 * -------------------------------------------------------------------------- */

static bsp_bb_mode_t   s_modes[BSP_BB_MODE_COUNT];
static bsp_bb_result_t s_results[BSP_BB_MODE_COUNT];
static uint8_t         s_static_level = 0xFFU;
static uint32_t        s_hit_count;
static uint32_t        s_crc_hit_count;
static int32_t         s_best_index = -1;
static uint8_t         s_scanned;

static void bb_build_modes(void)
{
    uint32_t n = 0U;
    uint8_t cs, cpol, cpha, lsb;

    for (cs = 0U; cs < 2U; cs++) {
        for (cpol = 0U; cpol < 2U; cpol++) {
            for (cpha = 0U; cpha < 2U; cpha++) {
                for (lsb = 0U; lsb < 2U; lsb++) {
                    s_modes[n].cs_toggle = cs;
                    s_modes[n].cpol      = cpol;
                    s_modes[n].cpha      = cpha;
                    s_modes[n].lsb_first = lsb;
                    n++;
                }
            }
        }
    }
}

void bsp_bb_scan(void)
{
    uint32_t i;
    uint32_t f;

    bb_build_modes();
    bb_pins_to_gpio();

    /* --- 静态电平: 一个时钟都不打, 直接读 MISO ---
     * 这是最有说服力的一条: 被上拉拉高 vs 被外部硬拉低, 一眼区分。 */
    bb_sclk_write(0U);
    bb_cs_write(1U);
    board_delay_us(10U);
    s_static_level = bb_miso_read();

    s_hit_count     = 0U;
    s_crc_hit_count = 0U;
    s_best_index    = -1;

    for (i = 0U; i < BSP_BB_MODE_COUNT; i++) {
        bsp_bb_result_t *r = &s_results[i];
        uint32_t seen[BSP_BB_FRAMES_PER_MODE];
        uint32_t nseen = 0U;

        r->raw_first = 0U;
        r->raw_last  = 0U;
        r->raw_min   = 0xFFFFFFFFU;
        r->raw_max   = 0U;
        r->distinct  = 0U;
        r->crc_ok    = 0U;

        for (f = 0U; f < BSP_BB_FRAMES_PER_MODE; f++) {
            uint32_t raw = bb_read_frame(&s_modes[i]);
            uint32_t k;
            uint8_t  dup = 0U;

            if (f == 0U) {
                r->raw_first = raw;
            }
            r->raw_last = raw;
            if (raw < r->raw_min) { r->raw_min = raw; }
            if (raw > r->raw_max) { r->raw_max = raw; }

            if ((raw & 0x3FU) == (uint32_t)bb_crc6(raw >> 6)) {
                r->crc_ok++;
            }

            for (k = 0U; k < nseen; k++) {
                if (seen[k] == raw) {
                    dup = 1U;
                    break;
                }
            }
            if ((dup == 0U) && (nseen < BSP_BB_FRAMES_PER_MODE)) {
                seen[nseen++] = raw;
            }
        }

        r->distinct = nseen;

        if (nseen >= 2U) {
            s_hit_count++;
        }
        if (r->crc_ok > 0U) {
            s_crc_hit_count++;
        }
    }

    /* 选最佳组合: 按 CRC6 通过帧数, 且要求至少一半帧通过才算数。
     *
     * 【这里曾经写错过】早先的判据是"第一个 distinct>=2 的组合", 这在
     * 转子静止时是反的: 正确的组合读出来是恒定值 (distinct=1) 反而选不上,
     * 而位序错误的组合因为抖动 distinct>=2 就被选中了 —— 实测就挑中了
     * idx=9 (LSB-first), 导致整机拿到的帧全部位反转。
     * 正确判据只能是 CRC: 错误的位序不可能持续通过 CRC6。 */
    {
        uint32_t best_crc = 0U;

        for (i = 0U; i < BSP_BB_MODE_COUNT; i++) {
            if (s_results[i].crc_ok > best_crc) {
                best_crc     = s_results[i].crc_ok;
                s_best_index = (int32_t)i;
            }
        }

        if (best_crc < (BSP_BB_FRAMES_PER_MODE / 2U)) {
            s_best_index = -1;      /* 没有任何组合达到可信的 CRC 通过率 */
        }
    }

    s_scanned = 1U;

    /* 恢复 SPI2 复用 (bsp_spi_init 内部会重写 IOC 并重配时序/格式) */
    bsp_spi_init();
}

/* --------------------------------------------------------------------------
 * 结果查询
 * -------------------------------------------------------------------------- */

uint8_t bsp_bb_static_level(void)
{
    return s_static_level;
}

uint32_t bsp_bb_hit_count(void)
{
    return s_hit_count;
}

uint32_t bsp_bb_crc_hit_count(void)
{
    return s_crc_hit_count;
}

int32_t bsp_bb_best_index(void)
{
    return s_best_index;
}

uint32_t bsp_bb_best_raw(void)
{
    if ((s_best_index < 0) || (s_scanned == 0U)) {
        return 0U;
    }
    return s_results[(uint32_t)s_best_index].raw_last;
}

const bsp_bb_mode_t *bsp_bb_mode_at(uint32_t idx)
{
    return (idx < BSP_BB_MODE_COUNT) ? &s_modes[idx] : NULL;
}

const bsp_bb_result_t *bsp_bb_result_at(uint32_t idx)
{
    return (idx < BSP_BB_MODE_COUNT) ? &s_results[idx] : NULL;
}

int bsp_bb_format(char *buf, uint32_t size, uint32_t idx)
{
    const bsp_bb_mode_t   *m;
    const bsp_bb_result_t *r;

    if ((buf == NULL) || (idx >= BSP_BB_MODE_COUNT)) {
        return 0;
    }

    m = &s_modes[idx];
    r = &s_results[idx];

    return snprintf(buf, size,
                    "bb[%02u] CPOL=%u CPHA=%u CS=%-3s LSB=%u "
                    "first=%06X last=%06X min=%06X max=%06X distinct=%u crc_ok=%u",
                    (unsigned)idx,
                    (unsigned)m->cpol, (unsigned)m->cpha,
                    (m->cs_toggle != 0U) ? "tog" : "low",
                    (unsigned)m->lsb_first,
                    (unsigned)r->raw_first, (unsigned)r->raw_last,
                    (unsigned)r->raw_min, (unsigned)r->raw_max,
                    (unsigned)r->distinct, (unsigned)r->crc_ok);
}

uint32_t bsp_bb_read_raw(uint32_t idx)
{
    uint32_t raw;

    if (idx >= BSP_BB_MODE_COUNT) {
        return 0U;
    }

    bb_pins_to_gpio();
    raw = bb_read_frame(&s_modes[idx]);
    bsp_spi_init();                    /* 恢复 SPI2 复用 */

    return raw;
}

/* --------------------------------------------------------------------------
 * bit-bang 作为正式读取后端 (兜底)
 * -------------------------------------------------------------------------- */

static uint8_t  s_active;
static uint32_t s_active_mode;

bool bsp_bb_activate(uint32_t idx)
{
    if (idx >= BSP_BB_MODE_COUNT) {
        return false;
    }

    bb_pins_to_gpio();
    s_active_mode = idx;
    s_active      = 1U;

    return true;
}

bool bsp_bb_is_active(void)
{
    return (s_active != 0U);
}

bool bsp_bb_read(uint8_t *rx, uint32_t len)
{
    uint32_t raw;

    if ((rx == NULL) || (len < 3U) || (s_active == 0U)) {
        return false;
    }

    raw = bb_read_frame(&s_modes[s_active_mode]);

    /* 与硬件 SPI 路径同样的字节序: MSB first, rx[0] 是最高字节 */
    rx[0] = (uint8_t)(raw >> 16);
    rx[1] = (uint8_t)(raw >> 8);
    rx[2] = (uint8_t)raw;

    return true;
}
