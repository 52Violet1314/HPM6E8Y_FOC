/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: USB0 设备实现 (CherryUSB CDC ACM)
 *
 * 【CDC ACM 是什么?】
 *   CDC = Communication Device Class, ACM = Abstract Control Model。
 *   通俗讲就是"用 USB 模拟一个串口": 设备插到 PC 上会出现一个 COM 口,
 *   不需要装驱动(Windows 10+ 自带 usbser)。本 BSP 用它来做调试/数据上报,
 *   相当于把 UART 调试口换成了 USB。
 *
 * 【对照 STM32】
 *   STM32 里通常用 STM32CubeMX 生成 USB_DEVICE (CDC) 中间件 +
 *   USBD_CDC_Receive_FS / CDC_Transmit_FS 收发。
 *   HPM 这里用的是 CherryUSB 协议栈, 对应关系:
 *       MX_USB_DEVICE_Init()      <-> cdc_acm_init()
 *       CDC_Transmit_FS(buf,len)  <-> cdc_acm_send()  (本文件 bsp_usb_write())
 *       CDC_Receive_FS 回调       <-> usbd_cdc_acm_bulk_out() -> bsp_usb_rx_callback()
 *       CDC_Control_FS(SET_LINE_CODING/DTR) <-> usbd_cdc_acm_set_dtr()
 *   注意: HPM 的 USB 收发是"异步提交"语义 —— 提交后立即返回, 真正的传输
 *   由 USB 中断/DMA 完成, 这点和 STM32 HAL 的阻塞式发送也不同。
 */
#include "bsp_usb.h"
#include "board.h"
#include "usb_config.h"
#include "hpm_interrupt.h"

/* 下面三个来自 usb_cdc_acm.c (CherryUSB 设备层) */
extern void cdc_acm_init(uint8_t busid, uint32_t reg_base);              /* 初始化 USB 设备/枚举 */
extern void cdc_acm_send(uint8_t busid, const uint8_t *data, uint32_t len); /* 向主机发数据 */
extern volatile bool dtr_enable;                                        /* 主机是否"打开了串口"(DTR=1) */

/* 应用层注册的主机下发数据回调 (USB 中断上下文调用) */
static bsp_usb_rx_cb_t s_rx_cb;

/**
 * @brief  初始化 USB0 设备并枚举成 CDC ACM 虚拟串口
 * @note   三步:
 *           1) board_init_usb()   -> 配置 USB PHY/时钟/引脚(板级相关);
 *           2) 设置 USB 中断优先级;
 *           3) cdc_acm_init()     -> 注册描述符/接口/端点并启动协议栈。
 *         对照 STM32: 相当于 MX_USB_DEVICE_Init()。
 */
void bsp_usb_init(void)
{
    board_init_usb((USB_Type *)CONFIG_HPM_USBD_BASE);    /* 板级 USB 硬件初始化 */
    /* USB 中断优先级 = 5。
     * 【为什么必须是 5, 而不是原来的 2】CherryUSB 的 osal 会在中断里调用 FreeRTOS
     * 的 API (give semaphore 等), 这类中断的优先级必须 >= configMAX_SYSCALL_INTERRUPT_PRIORITY
     * (=4, 见 src/rtos/FreeRTOSConfig.h), 否则内核关中断期间它的数据被改坏,
     * 表现为偶发死机。而电流环(1)/串口TX(3) 高于该阈值, 所以永不被 RTOS 屏蔽。
     * 优先级表见 src/rtos/app_rtos.h。 */
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 5);
    cdc_acm_init(0, CONFIG_HPM_USBD_BASE);               /* busid=0, 启动协议栈 */
}

/**
 * @brief  查询主机是否已打开本虚拟串口
 * @return true = 主机应用(串口助手等)已打开端口, 可以发送数据
 * @note   判断依据是 USB 控制传输里的 DTR 信号。很多上位机不打开串口时
 *         也会收到数据, 但会被丢弃, 所以发送前先判断可减少无效提交。
 *         对照 STM32: 相当于检查 CDC 的 line coding 状态。
 */
bool bsp_usb_connected(void)
{
    return dtr_enable;
}

/**
 * @brief  向主机发送数据
 * @param  data  待发送数据指针
 * @param  len   字节数
 * @return 实际提交的字节数(等于 len)
 * @note   内部调用 cdc_acm_send() -> usbd_ep_start_write(), 属于"异步提交":
 *         函数返回不代表数据已经发完, 只是排进了端点发送队列。
 *         因此不能传入栈上的临时缓冲区(除非确认本次调用会立即拷贝), 建议
 *         使用静态/全局缓冲区。这也是与 STM32 HAL_UART_Transmit(阻塞)最大的区别。
 */
uint32_t bsp_usb_write(const uint8_t *data, uint32_t len)
{
    cdc_acm_send(0, data, len);
    return len;
}

/**
 * @brief  注册主机下发数据回调
 * @param  cb  回调; NULL 表示注销
 * @note   在 USB 中断上下文被调用, 不要在这里做耗时操作。
 */
void bsp_usb_register_rx_cb(bsp_usb_rx_cb_t cb)
{
    s_rx_cb = cb;
}

/**
 * @brief  主机下发数据的回调分发 (由 usb_cdc_acm.c 在 USB 中断上下文调用)
 * @param  data  收到的数据
 * @param  len   字节数
 * @note   转发给应用层通过 bsp_usb_register_rx_cb() 注册的回调。
 */
void bsp_usb_rx_callback(const uint8_t *data, uint32_t len)
{
    if (s_rx_cb != NULL) {
        s_rx_cb(data, len);
    }
}
