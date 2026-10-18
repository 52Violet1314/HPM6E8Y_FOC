/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeRTOS 配置 —— core1(显示核)
 *
 * 与 core0 的 src/rtos/FreeRTOSConfig.h 是【两份独立配置】: FreeRTOS 的内核数据
 * 结构在各自核的内存里, 两个核各跑一套调度器, 互不相干(不是 SMP)。
 * 相同点保留(便于两边对照), 不同点都是为了显示核这个"只有 GUI 的小系统":
 *
 *   - 优先级只留 8 级 (core0 是 16): core1 只有显示任务 + 空闲任务, 用不着那么多。
 *   - 不做定时器任务 (configUSE_TIMERS 0): core1 没有任何软件定时器需求, 省一份栈。
 *   - 中断优先级门槛同样设 4: 显示用的 LCD 像素 DMA 中断(6)会在中断里 give 信号量,
 *     必须 >= 4(可被内核临界区屏蔽), 否则内核数据结构会被改坏。
 *
 * 【最要紧的一条: tick 用本核自己的 MCHTMR】
 *   HPM6E80 每个核有独立的 MCHTMR(地址相同, 互相独立), 所以这里的
 *   configMTIME_BASE_ADDRESS / configMTIMECMP_BASE_ADDRESS 和 core0 写同一个值
 *   就对了 —— 它取到的是 core1 自己的计数器, 不会和 core0 的 tick 抢同一个
 *   mtimecmp 导致两边互相打断。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "board.h"

/* tick 源 = 本核的机器定时器 mchtmr (24MHz, 见 board_init_clock):
 * 0xE6000000 = MTIME, 0xE6000008 = MTIMECMP (每核一份) */
#define configMTIME_BASE_ADDRESS                (HPM_MCHTMR_BASE)
#define configMTIMECMP_BASE_ADDRESS             (HPM_MCHTMR_BASE + 8UL)

#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      ((uint32_t) 24000000)  /* mchtmr 频率, 非 CPU 频率 */
#define configTICK_RATE_HZ                      ((TickType_t) 1000)
#define configMAX_PRIORITIES                    (8)
#define configMINIMAL_STACK_SIZE                (256)
#define configMAX_TASK_NAME_LEN                 (16)
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0

#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_MUTEXES                       1       /* lcd_app.c 的 LVGL 互斥锁 */
#define configUSE_RECURSIVE_MUTEXES             1

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
/* 堆: 显示任务栈 8KB + 空闲任务 + TCB。DLM 有 256KB, 16KB 够用且留余量。 */
#define configTOTAL_HEAP_SIZE                   ((size_t) (16 * 1024))

#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskAbortDelay                 1

#define configUSE_CO_ROUTINES                   0
/* core1 不需要软件定时器任务(没有任何 xTimer 调用)。真要用时改成 1 即可。
 * 但 configTIMER_TASK_STACK_DEPTH 仍然必须定义: HPM 的 RISC-V port.c 里
 * vApplicationGetTimerTaskMemory() 是无条件编译的, 它要拿这个宏声明数组,
 * 不给就报 "configTIMER_TASK_STACK_DEPTH undeclared"(实测)。 */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE)
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                (8)

/* 高于(数值小于)这个优先级的中断不受内核临界区影响; >= 本值的中断才可以调
 * RTOS API。LCD 像素 DMA 中断用的是 6(见 core1/rtos/app_rtos.h)。 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    4

/* 断言: 出错就断点+死循环, 便于调试器直接抓到出错现场 */
#define configASSERT(x) \
    if ((x) == 0) { taskDISABLE_INTERRUPTS(); __asm volatile("ebreak"); for (;;) {} }

/* 栈溢出/堆失败钩子的函数体在 core1/rtos/app_rtos.c; 声明由 FreeRTOS 的 task.h
 * 给出, 这里不要自己再声明(类型不一致会和 task.h 冲突)。 */

#endif /* FREERTOS_CONFIG_H */
