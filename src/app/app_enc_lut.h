/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 磁编码器 (MT6701 14bit) -> 电角度 查表 (离线标定, 3200 组)
 *
 * 【这组表从哪来】tools/enc_lut_build.py 从静态角度扫描的串口日志生成:
 *   固件把磁场一步步钉在各个电角度上, 转子吸稳后读编码器, 于是每个 raw
 *   都对应一个"能让转子停在这个位置的磁场电角度" —— 这正是换向要的角度。
 *   本次数据: 实测箱 2716/3200, 其余 483 箱由电周期折叠补;
 *             表 vs 实测残差 RMS 17 mrad, 最大 89 mrad。
 *
 * 【注意方向】表值随 raw 增大而【下降】(本安装下编码器方向与磁场正方向
 *   相反)。方向已经含在表里, 不要再乘 FOC_ENCODER_DIRECTION。
 *
 * 【重新生成】python tools/enc_lut_build.py "<静态扫描日志>"
 */
#ifndef APP_ENC_LUT_H
#define APP_ENC_LUT_H

#include <stdint.h>

#define APP_ENC_LUT_BINS    (3200U)     /* = 5.12 编码器 counts / 组 */

/** @brief 编码器原始角 (0..16383) -> 磁场电角度 (mrad, 0..6282) */
uint16_t app_enc_lut_mrad(uint16_t raw);

/** @brief 编码器原始角 (0..16383) -> 磁场电角度 (rad, 0..2pi) */
float app_enc_lut_elec_rad(uint16_t raw);

#endif /* APP_ENC_LUT_H */
