/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: MT6701 SPI 读取后端实现
 *
 *   一次 poll 的完整流程 (对应 STM32 上的一次 HAL_SPI_Receive):
 *     片选拉低 -> 敲 24 个时钟收 3 字节 -> 片选拉高 -> 拼成 24bit -> 解析
 *   MT6701 帧内已经是完整角度信息, 不需要发命令/地址, 所以 SPI 侧只要
 *   "读 3 个字节" 就够了。
 */
#include "bsp_mt6701.h"

#include <string.h>

#if BSP_MT6701_USE_SEI
#include "bsp_sei.h"
#else
#include "bsp_spi.h"
#endif

/* ---- 模块状态 ---- */
static bsp_mt6701_state_t   s_mt6701;
static bsp_mt6701_frame_cb_t s_frame_cb;

void bsp_mt6701_init(void)
{
    memset(&s_mt6701, 0, sizeof(s_mt6701));
    /* 注意: 这里【不要】把 s_frame_cb 清成 NULL。应用层可能在 init 之前
     * 就注册了回调 (与 bsp_sei 的约定一致); 若 init 顺手清掉, 成功帧就
     * 没人处理, 现象是 CNT/ERR 恒为 0 (之前就栽在这里)。 */

#if BSP_MT6701_USE_SEI
    bsp_sei_init();
#else
    /* SPI2 + J11 引脚 (PC26=SCLK / PC28=MISO / PC24=CS) */
    bsp_spi_init();
#endif
}

bool bsp_mt6701_poll(void)
{
#if BSP_MT6701_USE_SEI
    mt6701_frame_t frame;

    /* SEI 已由周期触发器自动收帧；这里把最新帧搬到兼容缓存，
     * 让原有 app_encoder 层继续使用 bsp_mt6701_* API。 */
    if (!bsp_sei_get_latest(&frame)) {
        return false;
    }

    s_mt6701.frame = frame;
    s_mt6701.raw = bsp_sei_get_raw();
    s_mt6701.last_raw = s_mt6701.raw;
    s_mt6701.new_frame = true;

    if (s_frame_cb != NULL) {
        s_frame_cb(&frame, s_mt6701.raw);
    }
    return true;
#else
    uint8_t  rx[BSP_MT6701_FRAME_BYTES];
    mt6701_frame_t frame;
    uint32_t raw;

    /* [1] 读 3 字节 */
    if (!bsp_spi_read(rx, BSP_MT6701_FRAME_BYTES)) {
        s_mt6701.transfer_fail++;
        return false;
    }

    /* [2] 拼成 24bit, MSB 在前; 无论是否有效都先记录, 便于诊断 */
    raw = ((uint32_t)rx[0] << 16) |
          ((uint32_t)rx[1] << 8)  |
          ((uint32_t)rx[2]);
    s_mt6701.last_raw = raw;

    /* [3] 全 0 / 全 1 基本可以判定是总线异常 (编码器没接、没供电、
     *     或者 MISO 被拉死), 直接计错, 避免把垃圾角度喂给速度环。
     *     注意全 0 帧的 CRC 恰好是合法的, 所以必须单独判。 */
    if ((raw == 0x000000U) || (raw == MT6701_SSI_FRAME_MASK)) {
        s_mt6701.error_count++;
        return false;
    }

    /* [4] 纯协议层解析 */
    mt6701_decode_frame(raw, &frame);

#if (BSP_MT6701_CRC_CHECK != 0)
    if (!frame.crc_ok) {
        s_mt6701.error_count++;
        return false;
    }
#endif

    /* [5] 缓存 + 上抛 */
    s_mt6701.raw         = raw;
    s_mt6701.frame       = frame;
    s_mt6701.new_frame   = true;
    s_mt6701.frame_count++;

    if (s_frame_cb != NULL) {
        s_frame_cb(&frame, raw);
    }

    return true;
#endif
}

void bsp_mt6701_register_frame_cb(bsp_mt6701_frame_cb_t cb)
{
    s_frame_cb = cb;
}

bool bsp_mt6701_get_latest(mt6701_frame_t *frame)
{
    bool is_new = s_mt6701.new_frame;

    if (frame != NULL) {
        *frame = s_mt6701.frame;
    }
    s_mt6701.new_frame = false;

    return is_new;
}

uint32_t bsp_mt6701_get_raw(void)
{
    return s_mt6701.raw;
}

uint16_t bsp_mt6701_get_raw_angle(void)
{
    return s_mt6701.frame.angle;
}

uint8_t bsp_mt6701_get_status(void)
{
    return s_mt6701.frame.status;
}

bool bsp_mt6701_get_crc_ok(void)
{
    return s_mt6701.frame.crc_ok;
}

bool bsp_mt6701_get_track_loss(void)
{
    return s_mt6701.frame.track_loss;
}

float bsp_mt6701_get_angle_rad(void)
{
    return mt6701_angle_to_rad(s_mt6701.frame.angle);
}

uint32_t bsp_mt6701_get_frame_count(void)
{
#if BSP_MT6701_USE_SEI
    return bsp_sei_get_frame_count();
#else
    return s_mt6701.frame_count;
#endif
}

uint32_t bsp_mt6701_get_error_count(void)
{
#if BSP_MT6701_USE_SEI
    return bsp_sei_get_error_count();
#else
    return s_mt6701.error_count;
#endif
}

uint32_t bsp_mt6701_get_last_raw(void)
{
#if BSP_MT6701_USE_SEI
    /* SEI 无 SPI transfer；peek_raw 用于尚未收帧时的硬件诊断。 */
    return bsp_sei_peek_raw();
#else
    return s_mt6701.last_raw;
#endif
}

uint32_t bsp_mt6701_get_transfer_fail(void)
{
#if BSP_MT6701_USE_SEI
    return 0U;
#else
    return s_mt6701.transfer_fail;
#endif
}
