/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Hardware: MT6701 磁编码器 —— 协议解析实现
 */

#include <stddef.h>

#include "mt6701.h"

uint8_t mt6701_crc6(uint32_t data18)
{
    /*
     * CRC-6: G(x) = x^6 + x^1 + 1, 即多项式 0x43, 初值 0, 不反转。
     * 6bit 寄存器逐位处理, 每来一位先左移, 最高位与数据位异或为 1 时
     * 再异或多项式低 6bit (0x03), x^6 项由移出位体现。
     *
     * 对照 STM32: 与硬件 CRC 外设无关, 纯软件查表/移位实现。
     */
    uint8_t crc = 0x00U;

    for (int i = 17; i >= 0; i--) {
        uint8_t in_bit = (uint8_t)((data18 >> i) & 0x01U);
        uint8_t top    = (uint8_t)((crc >> 5) & 0x01U);

        crc = (uint8_t)((crc << 1) & 0x3FU);
        if ((top ^ in_bit) != 0U) {
            crc ^= 0x03U;
        }
    }

    return crc;
}

void mt6701_decode_frame(uint32_t raw24, mt6701_frame_t *frame)
{
    uint32_t data18;

    if (frame == NULL) {
        return;
    }

    raw24 &= MT6701_SSI_FRAME_MASK;

    frame->angle  = (uint16_t)((raw24 >> 10) & MT6701_ANGLE_MAX);   /* bit[23:10] */
    frame->status = (uint8_t)((raw24 >> 6) & 0x0FU);                /* bit[9:6]   */
    frame->crc    = (uint8_t)(raw24 & 0x3FU);                       /* bit[5:0]   */

    /* CRC 覆盖 14bit 角度 + 4bit 状态, 共 18bit */
    data18 = ((uint32_t)frame->angle << 4) | (uint32_t)frame->status;
    frame->crc_calc = mt6701_crc6(data18);
    frame->crc_ok   = (frame->crc_calc == frame->crc);

    frame->field      = (mt6701_field_status_t)(frame->status & 0x03U);
    frame->button     = ((frame->status & 0x04U) != 0U);
    frame->track_loss = ((frame->status & 0x08U) != 0U);
}
