/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeRTOS 配置 (HPM6E8Y + 本工程)
 *
 * 【为什么上 RTOS】把"显示"和"打印"从 1kHz 控制循环里挪出去, 各跑一个低优先级任务:
 *   - 控制任务(最高优先级)只做采集/保护/换向, 周期稳定在 1ms (原来是 0.96~1.2ms 漂);
 *   - 日志任务负责格式化+写串口/USB, 显示任务负责 LVGL 刷屏。
 *
 * 【最要紧的一条: 电流环 ISR 不能被内核挡住】
 *   电流环跑在 ADC 完成中断里(PLIC 优先级 1)。FreeRTOS 的临界区会关中断, 如果
 *   电流环落在这个范围内, 20kHz 环路就会被拖长 —— 那正是之前"高速时顿挫"的
 *   根源。所以这里把 configMAX_SYSCALL_INTERRUPT_PRIORITY 设成 4:
 *       优先级 1/2/3 的中断(电流环=1, 串口TX=3) 永远不被屏蔽;
 *       优先级 >=4 的中断(USB=5) 会被屏蔽 —— 但只有它会调用 RTOS API, 必须如此。
 *   【改动中断优先级时务必对照这张表, 见 app_rtos.h 的说明。】
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "board.h"

/* tick 源 = 机器定时器 mchtmr (24MHz, 见上电时钟摘要)。
 * 不套 portasmHAS_MTIME 判断: 本芯片一定有 mtime/mtimecmp, 而 portasm.h 要更晚
 * 才被包含, 用它判断会让这里误判成 0 -> tick 根本不产生。 */
#define configMTIME_BASE_ADDRESS                (HPM_MCHTMR_BASE)
#define configMTIMECMP_BASE_ADDRESS             (HPM_MCHTMR_BASE + 8UL)

#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      ((uint32_t) 24000000)  /* mchtmr 频率, 非 CPU 频率 */
#define configTICK_RATE_HZ                      ((TickType_t) 1000)
#define configMAX_PRIORITIES                    (16)
#define configMINIMAL_STACK_SIZE                (256)
#define configMAX_TASK_NAME_LEN                 (16)
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0

#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
/* 堆: 三个任务的 TCB+栈、日志队列等。AXI_SRAM 有 768KB, 给 32KB 很宽裕。 */
#define configTOTAL_HEAP_SIZE                   ((size_t) (32 * 1024))

#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2   /* 上电阶段就抓栈溢出 */
#define configUSE_MALLOC_FAILED_HOOK            1   /* 堆不够立刻发现, 不要装作没事 */
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
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE)

/* 高于(数值小于)这个优先级的中断不受内核临界区影响。
 * 1=电流环ADC  3=串口TX  → 永不屏蔽, 实时性不受 RTOS 影响
 * 5=USB(会在中断里调 RTOS API) → 必须 >= 本值 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    4

/* 断言: 出错就断点+死循环, 便于调试器直接抓到出错现场 */
#define configASSERT(x) \
    if ((x) == 0) { taskDISABLE_INTERRUPTS(); __asm volatile("ebreak"); for (;;) {} }

/* 栈溢出钩子 / 堆失败钩子的声明由 FreeRTOS 的 task.h 给出 (类型是 TaskHandle_t),
 * 这里不要自己再声明一遍 —— 类型不一致会和 task.h 冲突(实测)。函数体在
 * src/rtos/app_rtos.c。 */

#endif /* FREERTOS_CONFIG_H */
