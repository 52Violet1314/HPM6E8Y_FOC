/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: USB0 设备 (CherryUSB CDC ACM 虚拟串口)
 *   引脚: USB0 DP/DN 由 USB PHY 固定, ID/OC/PWR = PF22/PF23/PF19
 */
#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 USB0 设备(CDC ACM), 上电枚举为虚拟串口
 * @note   对照 STM32: 相当于 MX_USB_DEVICE_Init()。
 */
void bsp_usb_init(void);

/**
 * @brief  主机是否已打开串口
 * @return true = 主机已置 DTR(串口已打开), 可以发送数据
 */
bool bsp_usb_connected(void);

/**
 * @brief  发送数据到主机
 * @param  data  数据指针(建议静态/全局缓冲, 因为是异步提交)
 * @param  len   字节数
 * @return 实际提交的字节数
 * @note   对照 STM32: 相当于 CDC_Transmit_FS(); 但为非阻塞提交。
 */
uint32_t bsp_usb_write(const uint8_t *data, uint32_t len);

/**
 * @brief  收到主机下发数据的回调 (弱定义, 用户可重写)
 * @param  data  收到的数据
 * @param  len   字节数
 * @note   在 USB 中断上下文被调用, 应保持简短。
 *         对照 STM32: 相当于 CDC_Receive_FS() 回调。
 */
void bsp_usb_rx_callback(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USB_H */
