/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 锐鹰 SRMA35 —— SEI_CTRL_2 异步模式后端 (10kHz)
 *      协议/引脚/时序说明见 bsp_srma35.h 文件头。
 *
 * 工作流程 (全部由 SEI 硬件按指令表走, CPU 每帧只进一次中断):
 *   1. 引脚复用 PD15=SEI2_TX / PD14=SEI2_RX / PD13=SEI2_DE
 *   2. 收发器配成异步模式 2.5Mbps 8 数据位 无校验 空闲高
 *   3. 指令表: SEND(CF=0x1A) -> RECV×7 (11 字节) -> HALT
 *   4. 周期触发每 100us 发起一帧; DE 由硬件在 SEND 期间自动拉高
 *   5. 收完 CRC 那一字节 -> latch1 事件 -> 中断里解析 + CRC 校验 + 回调
 *
 * 参考: HPM SDK samples/drivers/sei/master/tamagawa (同一套协议/同一种收发器模式)
 */

#include "bsp_srma35.h"

#include <stdio.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_sei_drv.h"

#define BSP_SRMA35_BASE      HPM_SEI
#define BSP_SRMA35_CLK_NAME  clock_sei0

#if (BSP_SRMA35_SEI_CTRL_INDEX == 0)
#define BSP_SRMA35_CTRL      SEI_CTRL_0
#define BSP_SRMA35_IRQn      IRQn_SEI0_0
#elif (BSP_SRMA35_SEI_CTRL_INDEX == 1)
#define BSP_SRMA35_CTRL      SEI_CTRL_1
#define BSP_SRMA35_IRQn      IRQn_SEI0_1
#elif (BSP_SRMA35_SEI_CTRL_INDEX == 2)
#define BSP_SRMA35_CTRL      SEI_CTRL_2
#define BSP_SRMA35_IRQn      IRQn_SEI0_2
#elif (BSP_SRMA35_SEI_CTRL_INDEX == 3)
#define BSP_SRMA35_CTRL      SEI_CTRL_3
#define BSP_SRMA35_IRQn      IRQn_SEI0_3
#else
#error "BSP_SRMA35_SEI_CTRL_INDEX must be 0..3"
#endif

/* INSTR[128] / DAT[18] 是整个 SEI 外设共享的, 必须与 bsp_sei.c(MT6701) 错开 */
#define BSP_SRMA35_INSTR_BASE   (32U)
#define BSP_SRMA35_INSTR_LAST   (BSP_SRMA35_INSTR_BASE + 8U)
#define BSP_SRMA35_INSTR_CRC    (BSP_SRMA35_INSTR_BASE + 7U)

#define BSP_SRMA35_DAT_TX       (SEI_DAT_4)
#define BSP_SRMA35_DAT_CF       (SEI_DAT_5)
#define BSP_SRMA35_DAT_SF       (SEI_DAT_6)
#define BSP_SRMA35_DAT_ABS      (SEI_DAT_7)
#define BSP_SRMA35_DAT_ENID     (SEI_DAT_8)
#define BSP_SRMA35_DAT_ABM      (SEI_DAT_9)
#define BSP_SRMA35_DAT_ALMC     (SEI_DAT_10)
#define BSP_SRMA35_DAT_CRC      (SEI_DAT_11)
#define BSP_SRMA35_DAT_FIRST    (BSP_SRMA35_DAT_TX)
#define BSP_SRMA35_DAT_LAST     (BSP_SRMA35_DAT_CRC)

static bsp_srma35_frame_t      s_frame;
static volatile bool           s_new_frame;
static volatile uint32_t       s_frame_count;
static volatile uint32_t       s_error_count;
static volatile uint32_t       s_trx_err_count;
static volatile uint32_t       s_wdg_count;
static volatile uint32_t       s_crc_err_count;

static bsp_srma35_frame_cb_t   s_frame_cb;

static uint32_t s_cfg_clk_freq;
static uint32_t s_cfg_baud_div;
static uint32_t s_cfg_trig_ticks;

/* -------------------------------------------------------------------------- */
static void bsp_srma35_init_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PD15].FUNC_CTL = IOC_PD15_FUNC_CTL_SEI2_TX;
    HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_SEI2_RX;
    HPM_IOC->PAD[IOC_PAD_PD13].FUNC_CTL = IOC_PD13_FUNC_CTL_SEI2_DE;

    HPM_IOC->PAD[IOC_PAD_PD15].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                          IOC_PAD_PAD_CTL_SPD_SET(3);
    HPM_IOC->PAD[IOC_PAD_PD14].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                          IOC_PAD_PAD_CTL_SPD_SET(3) |
                                          IOC_PAD_PAD_CTL_PE_SET(1) |
                                          IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PD13].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                          IOC_PAD_PAD_CTL_SPD_SET(3);
}

static void srma35_cfg_dat(uint8_t dat_idx, uint8_t word_len)
{
    sei_data_format_config_t cfg = {0};

    cfg.mode        = sei_data_mode;
    cfg.signed_flag = false;
    cfg.bit_order   = sei_bit_lsb_first;      /* 协议: 低位在前 */
    cfg.word_order  = sei_word_nonreverse;
    cfg.word_len    = word_len;
    cfg.first_bit   = 0;
    cfg.last_bit    = (uint8_t)(word_len - 1U);
    cfg.max_bit     = (uint8_t)(word_len - 1U);
    cfg.min_bit     = 0;
    (void)sei_cmd_data_format_config_init(BSP_SRMA35_BASE, SEI_SELECT_DATA,
                                          dat_idx, &cfg);
}

/* 11 字节 -> 一帧 */
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

    f->abs   &= 0x1FFFFU;
    f->abm   &= 0xFFFFU;
    f->pos32  = (f->abm << 17) | f->abs;

    /* 有效判据: CF 回显 + CRC (只查 CRC 不够, 断线时全 0 也会"通过") */
    f->valid  = (f->cf == BSP_SRMA35_CF_ALL) && f->crc_ok;
}

static void srma35_publish(const uint8_t *raw)
{
    bsp_srma35_frame_t f;
    bsp_srma35_frame_cb_t cb = s_frame_cb;

    srma35_decode(raw, &f);
    s_frame = f;

    if (f.valid) {
        s_frame_count++;
        s_new_frame = true;
        if (cb != NULL) {
            cb(&f);          /* 10kHz: 位置环/速度环挂在这里 */
        }
    } else {
        s_error_count++;
        if (!f.crc_ok) {
            s_crc_err_count++;
        }
    }
}

/* -------------------------------------------------------------------------- */
void bsp_srma35_init(void)
{
    sei_transceiver_config_t transceiver_config = {0};
    sei_engine_config_t engine_config = {0};
    sei_trigger_input_config_t trigger_input_config = {0};
    sei_state_transition_config_t state_transition_config = {0};
    sei_state_transition_latch_config_t latch_config = {0};
    sei_sample_config_t sample_config = {0};
    sei_update_config_t update_config = {0};
    uint32_t clk_freq;
    uint32_t tick_per_us;

    clock_add_to_group(BSP_SRMA35_CLK_NAME, 0);
    bsp_srma35_init_pins();

    sei_set_engine_enable(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, false);

    clk_freq = clock_get_frequency(BSP_SRMA35_CLK_NAME);
    s_cfg_clk_freq = clk_freq;

    /* [1] 收发器: 异步模式 2.5Mbps 8 数据位 无校验 空闲高 */
    transceiver_config.mode = sei_asynchronous_mode;
    transceiver_config.tri_sample = false;
    transceiver_config.src_clk_freq = clk_freq;
    transceiver_config.asynchronous_config.baudrate = BSP_SRMA35_BAUDRATE;
    transceiver_config.asynchronous_config.data_len = 8;
    transceiver_config.asynchronous_config.parity_enable = false;
    transceiver_config.asynchronous_config.parity = sei_asynchronous_parity_even;
    transceiver_config.asynchronous_config.data_idle_high_z = false;
    transceiver_config.asynchronous_config.data_idle_state = sei_idle_high_state;
    transceiver_config.asynchronous_config.wait_len = 0;
    (void)sei_transceiver_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, &transceiver_config);

    /* [2] 数据寄存器 */
    srma35_cfg_dat(BSP_SRMA35_DAT_TX, 8);
    srma35_cfg_dat(BSP_SRMA35_DAT_CF, 8);
    srma35_cfg_dat(BSP_SRMA35_DAT_SF, 8);
    srma35_cfg_dat(BSP_SRMA35_DAT_ABS, 24);
    srma35_cfg_dat(BSP_SRMA35_DAT_ENID, 8);
    srma35_cfg_dat(BSP_SRMA35_DAT_ABM, 24);
    srma35_cfg_dat(BSP_SRMA35_DAT_ALMC, 8);
    srma35_cfg_dat(BSP_SRMA35_DAT_CRC, 8);

    sei_set_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_TX, BSP_SRMA35_CF_ALL);

    /* [3] 指令表 */
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 0U, SEI_INSTR_OP_SEND,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_TX, 8);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 1U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_CF, 8);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 2U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_SF, 8);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 3U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_ABS, 24);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 4U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_ENID, 8);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 5U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_ABM, 24);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 6U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_ALMC, 8);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_BASE + 7U, SEI_INSTR_OP_RECV_WDG,
                  0, SEI_DAT_0, BSP_SRMA35_DAT_CRC, 8);
    sei_set_instr(BSP_SRMA35_BASE, BSP_SRMA35_INSTR_LAST, SEI_INSTR_OP_HALT,
                  0, SEI_DAT_0, SEI_DAT_0, 0);

    /* [4] 采样/更新单元 (保持与参考实现一致) */
    sample_config.latch_select = SEI_LATCH_0;
    (void)sei_sample_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, &sample_config);

    update_config.latch_select = SEI_LATCH_1;
    update_config.update_on_err = false;
    (void)sei_update_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, &update_config);

    /* [5] latch0: 收发切换点 (DE 由这个 latch 驱动, 0.48us 提前量) */
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = false;
    state_transition_config.instr_ptr_cfg = sei_state_tran_condition_fall_leave;
    state_transition_config.instr_ptr_value = BSP_SRMA35_INSTR_BASE;
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_0_1, &state_transition_config);

    state_transition_config.disable_instr_ptr_check = true;
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_1_2, &state_transition_config);
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_2_3, &state_transition_config);
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_3_0, &state_transition_config);

    latch_config.enable = true;
    latch_config.output_select = SEI_CTRL_LATCH_TRAN_0_1;
    latch_config.delay = (uint16_t)((48U * (clk_freq / 1000000U)) / 100U);
    (void)sei_state_transition_latch_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL,
                                                 SEI_LATCH_0, &latch_config);

    /* [6] latch1: 收完 CRC -> 中断 */
    state_transition_config.disable_instr_ptr_check = false;
    state_transition_config.instr_ptr_cfg = sei_state_tran_condition_fall_leave;
    state_transition_config.instr_ptr_value = BSP_SRMA35_INSTR_CRC;
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_0_1, &state_transition_config);

    state_transition_config.disable_instr_ptr_check = true;
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_1_2, &state_transition_config);
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_2_3, &state_transition_config);
    (void)sei_state_transition_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_3_0, &state_transition_config);

    latch_config.enable = true;
    latch_config.output_select = SEI_CTRL_LATCH_TRAN_0_1;
    latch_config.delay = 0;
    (void)sei_state_transition_latch_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL,
                                                 SEI_LATCH_1, &latch_config);

    /* [7] 引擎 */
    engine_config.arming_mode = sei_arming_wait_trigger;
    engine_config.data_cdm_idx = 0;
    engine_config.data_base_idx = 0;
    engine_config.init_instr_idx = BSP_SRMA35_INSTR_BASE;
    engine_config.wdg_enable = true;
    engine_config.wdg_action = sei_wdg_exec_exception_instr;
    engine_config.wdg_instr_idx = BSP_SRMA35_INSTR_LAST;
    engine_config.wdg_time = 0xFFFE;
    (void)sei_engine_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, &engine_config);
    sei_set_engine_enable(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, true);

    /* [8] 中断 */
    sei_clear_irq_flag(BSP_SRMA35_BASE, BSP_SRMA35_CTRL,
                       sei_irq_latch1_event | sei_irq_trx_err_event | sei_irq_wdog_event);
    sei_set_irq_enable(BSP_SRMA35_BASE, BSP_SRMA35_CTRL,
                       sei_irq_latch1_event | sei_irq_trx_err_event | sei_irq_wdog_event,
                       true);
    /* 优先级 6: 比电流环(7)低、比 RTOS 可屏蔽线(4)高 —— 10kHz 下不能压过电流环 */
    intc_m_enable_irq_with_priority(BSP_SRMA35_IRQn, 6);

    /* [9] 周期触发 100us */
    tick_per_us = clk_freq / 1000000U;
    trigger_input_config.trig_period_enable = true;
    trigger_input_config.trig_period_arming_mode = sei_arming_direct_exec;
    trigger_input_config.trig_period_sync_enable = false;
    trigger_input_config.trig_period_time = BSP_SRMA35_PERIOD_US * tick_per_us;
    (void)sei_trigger_input_config_init(BSP_SRMA35_BASE, BSP_SRMA35_CTRL,
                                        &trigger_input_config);

    s_cfg_baud_div   = sei_get_xcvr_baud_div(BSP_SRMA35_BASE, BSP_SRMA35_CTRL);
    s_cfg_trig_ticks = trigger_input_config.trig_period_time;

    printf("[SRMA35] SEI_CTRL_%u async %u bps @ %uHz: clk=%uHz baud_div=%u period=%utick DE=PD13 TX=PD15 RX=PD14\n",
           (unsigned)BSP_SRMA35_SEI_CTRL_INDEX, (unsigned)BSP_SRMA35_BAUDRATE,
           (unsigned)(1000000U / BSP_SRMA35_PERIOD_US),
           (unsigned)clk_freq, (unsigned)s_cfg_baud_div, (unsigned)s_cfg_trig_ticks);
}

void bsp_srma35_poll(void)
{
    /* SEI 硬件自己触发, 无需轮询 */
}

SDK_DECLARE_EXT_ISR_M(BSP_SRMA35_IRQn, bsp_srma35_isr)
void bsp_srma35_isr(void)
{
    uint32_t irq_flag;

    irq_flag = sei_get_irq_flag(BSP_SRMA35_BASE, BSP_SRMA35_CTRL);
    sei_clear_irq_flag(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, irq_flag);
    irq_flag &= sei_get_irq_enable_status(BSP_SRMA35_BASE, BSP_SRMA35_CTRL);

    if ((irq_flag & sei_irq_latch1_event) != 0U) {
        uint8_t  raw[BSP_SRMA35_RSP_LEN];
        uint32_t abs = sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_ABS) & 0xFFFFFFU;
        uint32_t abm = sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_ABM) & 0xFFFFFFU;

        raw[0]  = (uint8_t)sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_CF);
        raw[1]  = (uint8_t)sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_SF);
        raw[2]  = (uint8_t)(abs & 0xFFU);
        raw[3]  = (uint8_t)((abs >> 8) & 0xFFU);
        raw[4]  = (uint8_t)((abs >> 16) & 0xFFU);
        raw[5]  = (uint8_t)sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_ENID);
        raw[6]  = (uint8_t)(abm & 0xFFU);
        raw[7]  = (uint8_t)((abm >> 8) & 0xFFU);
        raw[8]  = (uint8_t)((abm >> 16) & 0xFFU);
        raw[9]  = (uint8_t)sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_ALMC);
        raw[10] = (uint8_t)sei_get_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_CRC);

        srma35_publish(raw);
    }

    if ((irq_flag & sei_irq_wdog_event) != 0U) {
        s_wdg_count++;
        s_error_count++;

        sei_set_engine_enable(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, false);
        sei_set_engine_rewind(BSP_SRMA35_BASE, BSP_SRMA35_CTRL);
        sei_set_command_rewind(BSP_SRMA35_BASE, BSP_SRMA35_CTRL);
        for (uint8_t d = BSP_SRMA35_DAT_FIRST; d <= BSP_SRMA35_DAT_LAST; d++) {
            sei_set_data_rewind(BSP_SRMA35_BASE, d);
        }
        sei_set_data_value(BSP_SRMA35_BASE, BSP_SRMA35_DAT_TX, BSP_SRMA35_CF_ALL);
        sei_set_engine_enable(BSP_SRMA35_BASE, BSP_SRMA35_CTRL, true);
    }

    if ((irq_flag & sei_irq_trx_err_event) != 0U) {
        s_trx_err_count++;
        s_error_count++;
    }
}

/* -------------------------------------------------------------------------- */
void bsp_srma35_register_frame_cb(bsp_srma35_frame_cb_t cb)
{
    s_frame_cb = cb;
}

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
        printf("[SRMA35] !! 无有效帧 e=%u(trx=%u wdg=%u crc=%u) raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
               (unsigned)s_error_count, (unsigned)s_trx_err_count,
               (unsigned)s_wdg_count, (unsigned)s_crc_err_count,
               f.raw[0], f.raw[1], f.raw[2], f.raw[3], f.raw[4], f.raw[5],
               f.raw[6], f.raw[7], f.raw[8], f.raw[9], f.raw[10]);
    } else {
        printf("[SRMA35] f=%u e=%u(trx%u wdg%u crc%u) abs=%u abm=%u sf=%02x almc=%02x enid=%02x\n",
               (unsigned)s_frame_count, (unsigned)s_error_count,
               (unsigned)s_trx_err_count, (unsigned)s_wdg_count, (unsigned)s_crc_err_count,
               (unsigned)f.abs, (unsigned)f.abm,
               (unsigned)f.sf, (unsigned)f.almc, (unsigned)f.enid);
    }

    if (s_error_count != last_err) {
        last_err = s_error_count;
    }
}
