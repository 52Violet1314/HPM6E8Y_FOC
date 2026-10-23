/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1 独立控制台(UART14) 的接口 —— 见 console_uart14.c
 *
 * 一句话: 让 core1 的日志走一条【自己的】物理串口, 不再依赖 core0 的日志任务。
 */
#ifndef CORE1_CONSOLE_UART14_H
#define CORE1_CONSOLE_UART14_H

#include <stdint.h>

/* 波特率: 必须和电脑端 USB-TTL 适配器一致 */
#ifndef CORE1_CONSOLE_BAUD
#define CORE1_CONSOLE_BAUD          (921600U)
#endif

/* 发送用 HDMA 还是 TX FIFO 中断。
 *   1 = HDMA 通道 1(默认)。中断全屏蔽, 由 core1_console_uart_poll() 续传;
 *       任何一步失败会自动退回 0 那条路。
 *   0 = TX FIFO 中断(和 core0 控制台同一实现)。
 * 【什么时候该改回 0】如果串口收到的是乱码/半截行, 先用 0 排除 DMA 这一层。 */
#ifndef CORE1_CONSOLE_USE_DMA
#define CORE1_CONSOLE_USE_DMA       (1)
#endif

/* 【总开关】1 = core1 用自己的 UART14 打印; 0 = 退回原来的共享内存环
 * (退回时 core1 的日志仍然只有 core0 的日志任务能搬, 也就是"core0 一忙就没日志") */
#ifndef CORE1_CONSOLE_OWN_UART
#define CORE1_CONSOLE_OWN_UART      (1)
#endif

/* 初始化 UART14(PF24/PF25 = J11 40/37)。必须在任何 LOG_I / printf 之前调用。
 * 失败也不致命: ready() 会返回 0, 日志自动退回共享环。 */
void core1_console_uart_init(void);

/* 1 = UART14 就绪, 可以往这里写; 0 = 还没初始化/初始化失败 */
int  core1_console_uart_ready(void);

/* 非阻塞写: 返回真正入队的字节数。'\n' 会自动补成 "\r\n"。放不下的丢弃。 */
int  core1_console_uart_write(const char *data, int size);

/* 写整行(自动补换行) */
void core1_console_uart_write_line(const char *line);

/* 【必须周期调用】由显示任务每拍(5ms)调一次: 收掉已完成的 DMA 并续传。
 * FIFO 模式下是空操作。 */
void core1_console_uart_poll(void);

/* 把 core0 的日志(共享内存的反向环 log0)取出来写到自己这条串口上。
 * 由显示任务周期调用 —— 于是【两个核的日志都在同一根线上】。
 * UART14 没就绪时什么都不取(数据留在环里, 环满由 core0 计数并丢弃)。 */
void core1_console_forward_core0(void);

/* 因环形缓冲满而丢掉的字节数(只增不减, 用于判断"日志被丢了") */
uint32_t core1_console_uart_dropped(void);

/* DMA 失败/退回 FIFO 的次数(正常应恒为 0) */
uint32_t core1_console_uart_dma_errors(void);

#endif /* CORE1_CONSOLE_UART14_H */
