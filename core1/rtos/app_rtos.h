/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核) 的 RTOS 入口与优先级表
 *
 * 【为什么显示核也要 RTOS】不是为了多任务, 而是为了沿用原设计:
 *   lcd_app.c 的 LVGL 互斥锁、LCDHw.c 的 DMA 完成信号量都是 FreeRTOS 对象,
 *   把它们改成裸机轮询既没必要也更容易出错。core1 上只有一个显示任务,
 *   "绑定到 Core1"就体现在: 这套调度器、这个任务, 整个跑在 core1 上。
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "FreeRTOS.h"

/* ---------------- 中断优先级 (core1 自己的 PLIC) ----------------
 * PLIC 是每核独立的: core0 上给电流环 ADC 使能的中断, core1 这边根本看不到;
 * 反过来 core1 使能的 LCD DMA 中断也不会打扰 core0 的 1kHz 控制任务 ——
 * 这正是把显示搬到 core1 的收益: 刷屏再慢也抢不到控制核的中断与总线优先级。
 *
 * LCD 像素 DMA 完成 = 6: 它在中断里 give 一个信号量, 所以必须 >=
 * configMAX_SYSCALL_INTERRUPT_PRIORITY(4)。 */
#define APP_IRQ_PRIO_LCD_DMA    (6U)

/* ---------------- 任务优先级 (数值越大越高) ---------------- */
#define APP_TASK_PRIO_DISPLAY   (configMAX_PRIORITIES - 2)

/* 显示任务周期: 5ms = 200Hz, 与原 core0 上的显示任务一致 */
#define CORE1_DISP_PERIOD_MS    (5U)

/**
 * @brief  创建显示任务并启动 core1 的调度器 (不返回)
 * @note   调用前必须完成 LCD/触摸/LVGL 的初始化(与 core0 原来的顺序一致)。
 */
void core1_rtos_start(void);

#endif /* APP_RTOS_H */
