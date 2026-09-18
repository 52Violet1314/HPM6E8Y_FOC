/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Hardware: MT6701 磁编码器 —— HPM SEI 后端实现 (HPM6E8Y / hpm6e00YZ)
 *
 * 工作流程:
 *   1. 引脚复用: SEI0 的 CK/RX/TX/DE
 *   2. 收发器配置为 "同步主机" 模式, 产生 SSI 时钟
 *   3. 装载指令: 收 14bit 角度 + 收 10bit(状态+CRC) + 超时 + 等待 + 停机
 *   4. 周期触发器每 MT6701_SEI_PERIOD_US 触发一帧
 *   5. 收帧完成产生 latch1 中断, 在中断里解析并缓存角度
 *
 * 参考: HPM SDK samples/drivers/sei/master/ssi
 */

#include "mt6701_sei.h"

#include <stdio.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_sei_drv.h"

/* ---- 实例选择 ---- */
#define MT6701_SEI_BASE     HPM_SEI
#define MT6701_SEI_CLK_NAME clock_sei0

#if (MT6701_SEI_CTRL_INDEX == 0)
#define MT6701_SEI_CTRL     SEI_CTRL_0
#define MT6701_SEI_IRQn     IRQn_SEI0_0
#elif (MT6701_SEI_CTRL_INDEX == 1)
#define MT6701_SEI_CTRL     SEI_CTRL_1
#define MT6701_SEI_IRQn     IRQn_SEI0_1
#else
#error "MT6701_SEI_CTRL_INDEX only supports 0 (SEI0) or 1 (SEI1)"
#endif

/* 数据寄存器分配: DAT_2 = 角度, DAT_3 = 状态+CRC */
#define MT6701_SEI_DAT_ANGLE SEI_DAT_2
#define MT6701_SEI_DAT_STAT  SEI_DAT_3

/* 状态+CRC 在 DAT_3 中共 10bit: [9:6]=状态, [5:0]=CRC */
#define MT6701_SEI_STAT_MASK (0x3FFU)

static volatile mt6701_sei_state_t s_mt6701;

/* --------------------------------------------------------------------------
 * 引脚复用
 * -------------------------------------------------------------------------- */
void mt6701_sei_init_pins(void)
{
#if (MT6701_SEI_CTRL_INDEX == 0)
    /* 本板 encoder 接口 (见 pinmux.c 顶部注释):
     *   PC07 = SEI0_TX   PC06 = SEI0_RX   PC05 = SEI0_DE   PF04 = SEI0_CK
     * SSI 实际只用 CK(时钟输出) 与 RX(数据输入), TX/DE 预留 RS485 模式。 */
    HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PC07_FUNC_CTL_SEI0_TX;
    HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PC06_FUNC_CTL_SEI0_RX;
    HPM_IOC->PAD[IOC_PAD_PC05].FUNC_CTL = IOC_PC05_FUNC_CTL_SEI0_DE;
    HPM_IOC->PAD[IOC_PAD_PF04].FUNC_CTL = IOC_PF04_FUNC_CTL_SEI0_CK;
#else
    /* SEI1 备选引脚 */
    HPM_IOC->PAD[IOC_PAD_PE16].FUNC_CTL = IOC_PE16_FUNC_CTL_SEI1_TX;
    HPM_IOC->PAD[IOC_PAD_PE17].FUNC_CTL = IOC_PE17_FUNC_CTL_SEI1_RX;
    HPM_IOC->PAD[IOC_PAD_PE18].FUNC_CTL = IOC_PE18_FUNC_CTL_SEI1_DE;
    HPM_IOC->PAD[IOC_PAD_PE19].FUNC_CTL = IOC_PE19_FUNC_CTL_SEI1_CK;
#endif
}

/* --------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------- */
void mt6701_sei_init(void)
{
    sei_transceiver_config_t transceiver_config = {0};
    sei_data_format_config_t data_format_config = {0};
    sei_engine_config_t engine_config = {0};
    sei_trigger_input_config_t trigger_input_config = {0};
    sei_state_transition_config_t state_transition_config = {0};
    sei_state_transition_latch_config_t state_transition_latch_config = {0};
    sei_sample_config_t sample_config = {0};
    sei_update_config_t update_config = {0};
    uint8_t instr_idx;
    uint32_t clk_freq;
    uint32_t tick_per_us;

    clock_add_to_group(MT6701_SEI_CLK_NAME, 0);
    mt6701_sei_init_pins();

    /* [0] 关闭引擎, 进入配置态 */
    sei_set_engine_enable(MT6701_SEI_BASE, MT6701_SEI_CTRL, false);

    clk_freq = clock_get_frequency(MT6701_SEI_CLK_NAME);

    /* [1] 收发器: 同步主机 (SEI 产生 SSI 时钟) */
    transceiver_config.mode = sei_synchronous_master_mode;
    transceiver_config.tri_sample = false;
    transceiver_config.src_clk_freq = clk_freq;
    transceiver_config.synchronous_master_config.data_idle_high_z = false;
    transceiver_config.synchronous_master_config.data_idle_state = sei_idle_high_state;
    transceiver_config.synchronous_master_config.clock_idle_high_z = false;
    transceiver_config.synchronous_master_config.clock_idle_state = sei_idle_high_state;
    transceiver_config.synchronous_master_config.baudrate = MT6701_SEI_BAUDRATE;
    (void)sei_transceiver_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, &transceiver_config);

    /* SSI 时序: 时钟下降沿编码器换数据, 上升沿主机采样。
     * rx_point = ck0_point, tx_point = ck1_point (与 SDK SSI 例程一致)。
     * 若实测数据错位, 改为 rx=ck1_point / tx=ck0_point 试试。 */
    sei_set_xcvr_rx_point(MT6701_SEI_BASE, MT6701_SEI_CTRL,
                          sei_get_xcvr_ck0_point(MT6701_SEI_BASE, MT6701_SEI_CTRL));
    sei_set_xcvr_tx_point(MT6701_SEI_BASE, MT6701_SEI_CTRL,
                          sei_get_xcvr_ck1_point(MT6701_SEI_BASE, MT6701_SEI_CTRL));

    /* [2] 数据格式 */

    /* DAT_2: 14bit 角度, MSB 先出, 右对齐 */
    data_format_config.mode = sei_data_mode;
    data_format_config.signed_flag = false;
    data_format_config.bit_order = sei_bit_msb_first;
    data_format_config.word_order = sei_word_nonreverse;
    data_format_config.word_len = 14;
    data_format_config.last_bit = 0;
    data_format_config.first_bit = 13;
    data_format_config.max_bit = 13;
    data_format_config.min_bit = 0;
    (void)sei_cmd_data_format_config_init(MT6701_SEI_BASE, SEI_SELECT_DATA,
                                          MT6701_SEI_DAT_ANGLE, &data_format_config);

    /* DAT_3: 10bit = 4bit 状态 + 6bit CRC, MSB 先出, 右对齐 */
    data_format_config.word_len = 10;
    data_format_config.last_bit = 0;
    data_format_config.first_bit = 9;
    data_format_config.max_bit = 9;
    data_format_config.min_bit = 0;
    (void)sei_cmd_data_format_config_init(MT6701_SEI_BASE, SEI_SELECT_DATA,
                                          MT6701_SEI_DAT_STAT, &data_format_config);

    /* [3] 指令序列 */
    instr_idx = 0;
    /* 收 14bit 角度 */
    sei_set_instr(MT6701_SEI_BASE, instr_idx++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_FALL_RISE,
                  SEI_DAT_0, MT6701_SEI_DAT_ANGLE, 14);
    /* 收 10bit 状态 + CRC */
    sei_set_instr(MT6701_SEI_BASE, instr_idx++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_FALL_RISE,
                  SEI_DAT_0, MT6701_SEI_DAT_STAT, 10);
    /* 超时指令: 时钟保持高, 数据用常量 0 */
    sei_set_instr(MT6701_SEI_BASE, instr_idx++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_HIGH,
                  SEI_DAT_0, SEI_DATA_CONST_0, 0);
    /* 等待指令: 时钟保持高, 数据用常量 1 */
    sei_set_instr(MT6701_SEI_BASE, instr_idx++, SEI_INSTR_OP_WAIT, SEI_INSTR_M_CK_HIGH,
                  SEI_DAT_0, SEI_DATA_CONST_1, 0);
    /* 停机: 等待下一次周期触发 */
    sei_set_instr(MT6701_SEI_BASE, instr_idx++, SEI_INSTR_OP_HALT, 0,
                  SEI_DAT_0, SEI_DAT_0, 0);

    /* [4] 采样 / 更新配置 (用 latch0 采样, latch1 表示收帧完成) */
    sample_config.latch_select = SEI_LATCH_0;
    (void)sei_sample_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, &sample_config);

    update_config.pos_data_idx = MT6701_SEI_DAT_ANGLE;
    update_config.pos_data_use_rx = true;
    update_config.update_on_err = false;
    update_config.latch_select = SEI_LATCH_1;
    update_config.data_register_select = BIT2_MASK;    /* 仅 DAT_2 参与更新 */
    (void)sei_update_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, &update_config);

    /* [5] 状态机: latch0 做超时保护, latch1 产生收帧完成中断 */
    /* latch0 */
    state_transition_config.disable_clk_check = false;
    state_transition_config.clk_cfg = sei_state_tran_condition_fall_leave;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true;
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_0_1, &state_transition_config);

    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true;
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_1_2, &state_transition_config);
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_2_3, &state_transition_config);

    state_transition_config.disable_instr_ptr_check = false;
    state_transition_config.instr_ptr_cfg = sei_state_tran_condition_fall_leave;
    state_transition_config.instr_ptr_value = (uint8_t)(instr_idx - 2);
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_0,
                                           SEI_CTRL_LATCH_TRAN_3_0, &state_transition_config);

    state_transition_latch_config.enable = true;
    state_transition_latch_config.output_select = SEI_CTRL_LATCH_TRAN_0_1;
    state_transition_latch_config.delay = 0;
    (void)sei_state_transition_latch_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_0,
                                                 &state_transition_latch_config);

    /* latch1: 当指令指针走到超时指令(索引 3)时置位 -> 表示一帧收完 */
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = false;
    state_transition_config.instr_ptr_cfg = sei_state_tran_condition_fall_leave;
    state_transition_config.instr_ptr_value = 3;
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_0_1, &state_transition_config);

    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true;
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_1_2, &state_transition_config);
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_2_3, &state_transition_config);
    (void)sei_state_transition_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_1,
                                           SEI_CTRL_LATCH_TRAN_3_0, &state_transition_config);

    state_transition_latch_config.enable = true;
    state_transition_latch_config.output_select = SEI_CTRL_LATCH_TRAN_0_1;
    state_transition_latch_config.delay = 0;
    (void)sei_state_transition_latch_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, SEI_LATCH_1,
                                                 &state_transition_latch_config);

    /* [6] 引擎: 等待触发, 从指令 0 开始 */
    engine_config.arming_mode = sei_arming_wait_trigger;
    engine_config.data_cdm_idx = 0;
    engine_config.data_base_idx = 0;
    engine_config.init_instr_idx = 0;
    engine_config.wdg_enable = false;
    (void)sei_engine_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, &engine_config);
    sei_set_engine_enable(MT6701_SEI_BASE, MT6701_SEI_CTRL, true);

    /* [7] 中断: 收帧完成 + 传输错误 */
    sei_clear_irq_flag(MT6701_SEI_BASE, MT6701_SEI_CTRL,
                       sei_irq_latch1_event | sei_irq_trx_err_event);
    sei_set_irq_enable(MT6701_SEI_BASE, MT6701_SEI_CTRL,
                       sei_irq_latch1_event | sei_irq_trx_err_event, true);
    intc_m_enable_irq_with_priority(MT6701_SEI_IRQn, 1);

    /* [8] 周期触发: 每 PERIOD_US 自动发起一帧 */
    tick_per_us = clk_freq / 1000000U;
    trigger_input_config.trig_period_enable = true;
    trigger_input_config.trig_period_arming_mode = sei_arming_direct_exec;
    trigger_input_config.trig_period_sync_enable = false;
    trigger_input_config.trig_period_time = (uint32_t)MT6701_SEI_PERIOD_US * tick_per_us;
    (void)sei_trigger_input_config_init(MT6701_SEI_BASE, MT6701_SEI_CTRL, &trigger_input_config);

    /* 诊断: 如果 clk 打印为 0, 说明 SEI 时钟没拿到, 周期触发时间会变成 0,
     * 引擎就永远不会被触发(现象: CNT=0, E=0)。 */
    printf("[SEI] ctrl=%d clk=%u Hz, baud=%u, period=%u ticks\n",
           (int)MT6701_SEI_CTRL_INDEX, (unsigned)clk_freq,
           (unsigned)MT6701_SEI_BAUDRATE,
           (unsigned)trigger_input_config.trig_period_time);
}

/* --------------------------------------------------------------------------
 * 中断: 一帧收完后解析并缓存
 * -------------------------------------------------------------------------- */
SDK_DECLARE_EXT_ISR_M(MT6701_SEI_IRQn, mt6701_sei_isr)
void mt6701_sei_isr(void)
{
    uint32_t irq_flag;
    uint32_t angle;
    uint32_t stat;
    uint32_t raw;
    mt6701_frame_t frame;

    irq_flag = sei_get_irq_flag(MT6701_SEI_BASE, MT6701_SEI_CTRL);
    sei_clear_irq_flag(MT6701_SEI_BASE, MT6701_SEI_CTRL, irq_flag);
    irq_flag &= sei_get_irq_enable_status(MT6701_SEI_BASE, MT6701_SEI_CTRL);

    if ((irq_flag & sei_irq_latch1_event) != 0U) {
        angle = sei_get_data_value(MT6701_SEI_BASE, MT6701_SEI_DAT_ANGLE) & MT6701_ANGLE_MAX;
        stat  = sei_get_data_value(MT6701_SEI_BASE, MT6701_SEI_DAT_STAT) & MT6701_SEI_STAT_MASK;
        raw   = (angle << 10) | stat;    /* 还原 24bit: 角度[23:10] + 状态/CRC[9:0] */

        mt6701_decode_frame(raw, &frame);

#if MT6701_SSI_CRC_CHECK
        if (!frame.crc_ok) {
            s_mt6701.error_count++;
            return;
        }
#endif
        s_mt6701.raw = raw;
        s_mt6701.frame = frame;      /* 结构体整体拷贝 */
        s_mt6701.frame_count++;
        s_mt6701.new_frame = true;
    }

    if ((irq_flag & sei_irq_trx_err_event) != 0U) {
        s_mt6701.error_count++;
    }
}

/* --------------------------------------------------------------------------
 * 应用层接口
 * -------------------------------------------------------------------------- */
bool mt6701_sei_get_latest(mt6701_frame_t *frame)
{
    uint32_t lock = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    bool is_new = s_mt6701.new_frame;

    s_mt6701.new_frame = false;
    if (frame != NULL) {
        *frame = s_mt6701.frame;
    }
    restore_global_irq(lock);

    return is_new;
}

uint32_t mt6701_sei_get_raw(void)
{
    return s_mt6701.raw;
}

uint32_t mt6701_sei_peek_raw(void)
{
    uint32_t angle = sei_get_data_value(MT6701_SEI_BASE, MT6701_SEI_DAT_ANGLE) & MT6701_ANGLE_MAX;
    uint32_t stat  = sei_get_data_value(MT6701_SEI_BASE, MT6701_SEI_DAT_STAT) & MT6701_SEI_STAT_MASK;

    return (angle << 10) | stat;
}

uint16_t mt6701_sei_get_raw_angle(void)
{
    return s_mt6701.frame.angle;
}

float mt6701_sei_get_angle_rad(void)
{
    return mt6701_angle_to_rad(s_mt6701.frame.angle);
}

uint32_t mt6701_sei_get_frame_count(void)
{
    return s_mt6701.frame_count;
}

uint32_t mt6701_sei_get_error_count(void)
{
    return s_mt6701.error_count;
}

void mt6701_sei_soft_trigger(void)
{
    sei_set_trig_input_soft_enable(MT6701_SEI_BASE, MT6701_SEI_CTRL);
}
