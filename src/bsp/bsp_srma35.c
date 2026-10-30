/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 锐鹰 SRMA35 —— RS485 (UART11) 后端实现
 *      协议/引脚/时序说明见 bsp_srma35.h 文件头。
 */

#include "bsp_srma35.h"

#include <stdio.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_uart_drv.h"

#define SRMA35_UART        HPM_UART11
#define SRMA35_UART_CLK    clock_uart11
#define SRMA35_UART_IRQn   IRQn_UART11
#define SRMA35_DE_PORT     GPIO_DI_GPIOD
#define SRMA35_DE_PIN      (13U)

/* ---- 状态 ---- */
static bsp_srma35_frame_t     s_frame;
static volatile bool          s_new_frame;
static volatile uint32_t      s_frame_count;
static volatile uint32_t      s_error_count;
static volatile uint32_t      s_trx_err_count;   /* 超时/没回够字节 */
static volatile uint32_t      s_crc_err_count;   /* 收满 11 字节但校验不过 */
static volatile uint32_t      s_overrun_count;
static volatile uint32_t      s_frameerr_count;

/* ---- 接收 ---- */
static volatile uint8_t  s_rx_buf[BSP_SRMA35_RSP_LEN];
static volatile uint8_t  s_rx_idx;
static volatile bool     s_waiting;
static uint32_t          s_rx_start_ms;
static uint32_t          s_tick_ms;
static uint32_t          s_next_slot_ms;
static uint32_t          s_ticks_per_us;

/* ---- 坏帧现场 ---- */
static volatile uint8_t  s_bad_raw[BSP_SRMA35_RSP_LEN];
static volatile uint8_t  s_bad_len;
static volatile uint8_t  s_bad_reason;    /* 1=超时 2=校验不过 */

static uint32_t s_cfg_clk_freq;

static inline void srma35_de(bool on)
{
    gpio_write_pin(HPM_GPIO0, SRMA35_DE_PORT, SRMA35_DE_PIN, on ? 1U : 0U);
}

static void srma35_delay_us(uint32_t us)
{
    uint64_t t0 = mchtmr_get_count(HPM_MCHTMR);
    uint64_t need = (uint64_t)us * (uint64_t)s_ticks_per_us;

    while ((mchtmr_get_count(HPM_MCHTMR) - t0) < need) {
        /* busy wait */
    }
}

/* 11 字节 -> 一帧 (CRC 校验 + 字段拆解) */
static void srma35_decode(const uint8_t *raw, bsp_srma35_frame_t *f)
{
    uint8_t  crc = 0U;
    uint32_t i;

    for (i = 0U; i < BSP_SRMA35_RSP_LEN; i++) {
        f->raw[i] = raw[i];
    }
    f->cf     = raw[0];
    f->sf     = raw[1];
    f->abs    = (uint32_t)raw[2] | ((uint32_t)raw[3] << 8) | ((uint32_t)raw[4] << 16);
    f->enid   = raw[5];
    f->abm    = (uint32_t)raw[6] | ((uint32_t)raw[7] << 8) | ((uint32_t)raw[8] << 16);
    f->almc   = raw[9];
    f->crc_rx = raw[10];

    for (i = 0U; i < (BSP_SRMA35_RSP_LEN - 1U); i++) {
        crc ^= raw[i];
    }
    f->crc_calc = crc;
    f->crc_ok   = (crc == f->crc_rx);

    f->abs  &= 0x1FFFFU;
    f->abm  &= 0xFFFFU;
    f->pos32 = (f->abm << 17) | f->abs;

    /* 有效判据: CF 回显 + CRC。只查 CRC 不够 —— 总线断开时全 0 也"通过" */
    f->valid = (f->cf == BSP_SRMA35_CF_ALL) && f->crc_ok;
}

static void srma35_publish(const uint8_t *raw)
{
    bsp_srma35_frame_t f;
    uint32_t i;

    srma35_decode(raw, &f);
    s_frame = f;

    if (f.valid) {
        s_frame_count++;
        s_new_frame = true;
    } else {
        s_error_count++;
        if (!f.crc_ok) {
            s_crc_err_count++;
        }
        for (i = 0U; i < BSP_SRMA35_RSP_LEN; i++) {
            s_bad_raw[i] = raw[i];
        }
        s_bad_len    = BSP_SRMA35_RSP_LEN;
        s_bad_reason = 2U;
    }
}

/* -------------------------------------------------------------------------- */
void bsp_srma35_init(void)
{
    uart_config_t cfg;
    hpm_stat_t st;

    clock_add_to_group(SRMA35_UART_CLK, 0);

    /* PD15=TXD / PD14=RXD (ALT2=UART11), PD13 当 GPIO 控 DE */
    HPM_IOC->PAD[IOC_PAD_PD15].FUNC_CTL = IOC_PD15_FUNC_CTL_UART11_TXD;
    HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_UART11_RXD;
    HPM_IOC->PAD[IOC_PAD_PD13].FUNC_CTL = IOC_PD13_FUNC_CTL_GPIO_D_13;

    HPM_IOC->PAD[IOC_PAD_PD15].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                          IOC_PAD_PAD_CTL_SPD_SET(3);
    HPM_IOC->PAD[IOC_PAD_PD14].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                          IOC_PAD_PAD_CTL_SPD_SET(3) |
                                          IOC_PAD_PAD_CTL_PE_SET(1) |
                                          IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PD13].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                          IOC_PAD_PAD_CTL_SPD_SET(3);

    (void)gpio_set_pin_output(HPM_GPIO0, SRMA35_DE_PORT, SRMA35_DE_PIN);
    srma35_de(false);

    s_ticks_per_us = clock_get_frequency(clock_mchtmr0) / 1000000U;
    if (s_ticks_per_us == 0U) {
        s_ticks_per_us = 24U;
    }
    s_cfg_clk_freq = clock_get_frequency(SRMA35_UART_CLK);

    uart_default_config(SRMA35_UART, &cfg);
    cfg.src_freq_in_hz   = s_cfg_clk_freq;
    cfg.baudrate         = BSP_SRMA35_BAUDRATE;
    cfg.num_of_stop_bits = stop_bits_1;
    cfg.word_length      = word_length_8_bits;
    cfg.parity           = parity_none;
    cfg.fifo_enable      = true;
    cfg.dma_enable       = false;
    cfg.rx_fifo_level    = uart_rx_fifo_trg_not_empty;
    cfg.tx_fifo_level    = uart_tx_fifo_trg_not_full;
    st = uart_init(SRMA35_UART, &cfg);

    uart_enable_irq(SRMA35_UART, uart_intr_rx_data_avail_or_timeout);
    intc_m_enable_irq_with_priority(SRMA35_UART_IRQn, 1);

    printf("[SRMA35] UART11 %u bps: clk=%uHz div*osc=%u TX=PD15 RX=PD14 DE=PD13 init=%d\n",
           (unsigned)BSP_SRMA35_BAUDRATE, (unsigned)s_cfg_clk_freq,
           (unsigned)(s_cfg_clk_freq / BSP_SRMA35_BAUDRATE), (int)st);
}

void bsp_srma35_poll(void)
{
    s_tick_ms++;

    if (s_waiting && ((s_tick_ms - s_rx_start_ms) >= BSP_SRMA35_RX_TIMEOUT_MS)) {
        uint8_t i;

        s_waiting = false;
        s_error_count++;
        s_trx_err_count++;

        for (i = 0U; i < BSP_SRMA35_RSP_LEN; i++) {
            s_bad_raw[i] = (i < s_rx_idx) ? s_rx_buf[i] : 0U;
        }
        s_bad_len    = s_rx_idx;
        s_bad_reason = 1U;
    }

    if (!s_waiting && ((s_tick_ms - s_next_slot_ms) >= BSP_SRMA35_PERIOD_MS)) {
        uint32_t guard = 0U;
#if BSP_SRMA35_DE_CRITICAL
        uint32_t lock;
#endif

        s_next_slot_ms = s_tick_ms;
        s_rx_start_ms  = s_tick_ms;

#if BSP_SRMA35_DE_CRITICAL
        lock = disable_global_irq(CSR_MSTATUS_MIE_MASK);
#endif

        srma35_de(true);
        srma35_delay_us(BSP_SRMA35_DE_SETUP_US);

        (void)uart_send_byte(SRMA35_UART, BSP_SRMA35_CF_ALL);

        /* 先等 1us 让它真正开始发 (TEMT 由 1 变 0), 再等发完 */
        srma35_delay_us(1);
        while (!uart_check_status(SRMA35_UART, uart_stat_transmitter_empty)) {
            if (++guard > 100000U) {
                break;
            }
        }

        srma35_de(false);

        /* 见文件头第 2 条: 先掏空 FIFO, 再开收帧窗口 */
        {
            uint8_t junk;
            while (uart_check_status(SRMA35_UART, uart_stat_data_ready)) {
                if (uart_try_receive_byte(SRMA35_UART, &junk) != status_success) {
                    break;
                }
            }
        }
        s_rx_idx  = 0U;
        s_waiting = true;

#if BSP_SRMA35_DE_CRITICAL
        restore_global_irq(lock);
#endif
    }
}

SDK_DECLARE_EXT_ISR_M(SRMA35_UART_IRQn, bsp_srma35_uart_isr)
void bsp_srma35_uart_isr(void)
{
    uint8_t  c;
    uint32_t lsr;

    /* 先取错误标志 (读 LSR 会清掉; 下面的 uart_check_status 也会读 LSR) */
    lsr = SRMA35_UART->LSR;
    if ((lsr & UART_LSR_OE_MASK) != 0U) {
        s_overrun_count++;
    }
    if ((lsr & (UART_LSR_FE_MASK | UART_LSR_PE_MASK | UART_LSR_LBREAK_MASK)) != 0U) {
        s_frameerr_count++;
    }

    while (uart_check_status(SRMA35_UART, uart_stat_data_ready)) {
        if (uart_try_receive_byte(SRMA35_UART, &c) != status_success) {
            break;
        }
        if (!s_waiting) {
            continue;                    /* 空闲期杂散字节: 丢 */
        }
        if (s_rx_idx < BSP_SRMA35_RSP_LEN) {
            s_rx_buf[s_rx_idx++] = c;
        }
        if (s_rx_idx >= BSP_SRMA35_RSP_LEN) {
            s_waiting = false;
            srma35_publish((const uint8_t *)s_rx_buf);
        }
    }
}

/* -------------------------------------------------------------------------- */
bool bsp_srma35_get_latest(bsp_srma35_frame_t *out)
{
    uint32_t lock = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    bool is_new = s_new_frame;

    s_new_frame = false;
    if (out != NULL) {
        *out = s_frame;
    }
    restore_global_irq(lock);

    return is_new;
}

void bsp_srma35_peek(bsp_srma35_frame_t *out)
{
    uint32_t lock;

    if (out == NULL) {
        return;
    }
    lock = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    *out = s_frame;
    restore_global_irq(lock);
}

uint32_t bsp_srma35_get_frame_count(void)
{
    return s_frame_count;
}

uint32_t bsp_srma35_get_error_count(void)
{
    return s_error_count;
}

bool bsp_srma35_is_silent(void)
{
    return (s_frame_count == 0U);
}

void bsp_srma35_poll_log(void)
{
    static uint32_t ms_acc;
    static uint32_t last_err;
    bsp_srma35_frame_t f;

    if (++ms_acc < 1000U) {
        return;
    }
    ms_acc = 0U;

    bsp_srma35_peek(&f);

    if (bsp_srma35_is_silent()) {
        printf("[SRMA35] !! 无有效帧 e=%u(trx=%u crc=%u) raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
               (unsigned)s_error_count, (unsigned)s_trx_err_count, (unsigned)s_crc_err_count,
               f.raw[0], f.raw[1], f.raw[2], f.raw[3], f.raw[4], f.raw[5],
               f.raw[6], f.raw[7], f.raw[8], f.raw[9], f.raw[10]);
    } else {
        printf("[SRMA35] f=%u e=%u(t%u c%u) abs=%u abm=%u sf=%02x almc=%02x enid=%02x %s\n",
               (unsigned)s_frame_count, (unsigned)s_error_count,
               (unsigned)s_trx_err_count, (unsigned)s_crc_err_count,
               (unsigned)f.abs, (unsigned)f.abm,
               (unsigned)f.sf, (unsigned)f.almc, (unsigned)f.enid,
               f.crc_ok ? "crc=OK" : "crc=BAD");
    }

    if (s_error_count != last_err) {
        last_err = s_error_count;
        printf("[SRMA35] 坏帧: reason=%u len=%u ovr=%u fe=%u raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
               (unsigned)s_bad_reason, (unsigned)s_bad_len,
               (unsigned)s_overrun_count, (unsigned)s_frameerr_count,
               s_bad_raw[0], s_bad_raw[1], s_bad_raw[2], s_bad_raw[3], s_bad_raw[4],
               s_bad_raw[5], s_bad_raw[6], s_bad_raw[7], s_bad_raw[8], s_bad_raw[9],
               s_bad_raw[10]);
    }
}
