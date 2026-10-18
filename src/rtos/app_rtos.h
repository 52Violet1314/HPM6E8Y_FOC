/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RTOS: 任务划分入口 (core0 = 电机控制核)
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "FreeRTOS.h"

/* ---------------- 中断优先级表 (PLIC, 数值越小优先级越高) ----------------
 * 改任何中断优先级前先看这张表, 尤其别把 USB 提到 4 以上:
 *   configMAX_SYSCALL_INTERRUPT_PRIORITY = 4 (见 FreeRTOSConfig.h)
 *     - 优先级 1/2/3 的中断【永远不会被 RTOS 临界区屏蔽】, 但不能调用 RTOS API;
 *     - 优先级 >=4 的中断可以被屏蔽, 也【必须】这样 —— 因为只有它会在中断里
 *       调用 RTOS API (CherryUSB 的 osal 会 give semaphore), 否则内核数据结构
 *       在关中断期间被改坏, 表现为偶发死机。
 * 现状(core0): 电流环=1 (绝对实时), 串口TX=3 (不调 RTOS API), USB=5 (调 RTOS API)。
 * 【这张表只说 core0 自己的 PLIC】PLIC 每核独立: core1 使能的 LCD 像素 DMA
 * 中断(优先级 6)在 core1 的 PLIC 里, 不在本表范围内, 见 core1/rtos/app_rtos.h。 */
#define APP_IRQ_PRIO_CURRENT    (1U)
#define APP_IRQ_PRIO_CONSOLE    (3U)
#define APP_IRQ_PRIO_USB        (5U)

/* ---------------- 任务优先级 (数值越大越高) ---------------- */
#define APP_TASK_PRIO_CONTROL   (configMAX_PRIORITIES - 1)   /* 控制: 最高 */
#define APP_TASK_PRIO_LOG       (configMAX_PRIORITIES - 3)   /* 日志 */
/* 显示任务不在这里: 它跑在 core1 上, 见 core1/rtos/app_rtos.h */

/**
 * @brief  创建控制/日志任务并启动 core0 的调度器 (不返回)
 * @note   调用前必须完成所有外设与应用的初始化 (board_init / app_init /
 *         core1_start 等), 因为调度器一旦启动, 后续代码就不会再执行。
 */
void app_rtos_start(void);

#endif /* APP_RTOS_H */
