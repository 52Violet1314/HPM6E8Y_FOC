/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RTOS: 任务划分入口 (core0 = 电机控制核)
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "FreeRTOS.h"

/* ---------------- 中断优先级表 (PLIC, 【数值越大】优先级越高) ----------------
 * 【这张表原来整段写反了, 2026-10-18 修正】HPM 的 PLIC 与 RISC-V PLIC 规范一致:
 *   - 数值【越大】优先级越高 (7 最高, 0 = 不中断);
 *   - 只有 "优先级 > 阈值" 的中断才会被投递;
 *   - FreeRTOS 临界区把阈值设成 configMAX_SYSCALL_INTERRUPT_PRIORITY = 4
 *     (见 FreeRTOSConfig.h), 于是【优先级 ≤4 的中断在临界区里被屏蔽】。
 *
 * 于是规则是:
 *   - 必须在临界区里照常响应的 -> 优先级【>4】(5~7), 且它不能调用 RTOS API;
 *   - 会在中断里调用 RTOS API 的 -> 优先级【≤4】, 让它被临界区屏蔽住,
 *     否则内核数据结构在关中断期间被改坏, 表现为偶发死机。
 *
 * 现状(core0):
 *   电流环 ADC   = 7  (最高, >4: 永不被 RTOS 屏蔽; 不调用 RTOS API)
 *   串口 TX      = 3  (≤4: 可屏蔽; 不调用 RTOS API)
 *   USB(Cherry)  = 4  (≤4: 必须可屏蔽, 它的 osal 会 give semaphore)
 *
 * 【历史教训】原来写的是 电流环=1 / USB=5, 正好反了: 电流环在每次 RTOS 临界区里
 * 被屏蔽 -> 日志里每秒出现 "ADC 中断被拖长 260~300us"(5~6 个 PWM 周期占空比冻结)
 * -> 电机"跳一下", 300Hz 带宽的电流环被这种抖动打成自激振荡。
 *
 * 【这张表只说 core0 自己的 PLIC】PLIC 每核独立: core1 使能的 LCD 像素 DMA
 * 中断在 core1 的 PLIC 里, 不在本表范围内, 见 core1/rtos/app_rtos.h。 */
#define APP_IRQ_PRIO_CURRENT    (7U)    /* >4: 电流环, 永不被临界区屏蔽 */
#define APP_IRQ_PRIO_CONSOLE    (3U)    /* ≤4: 串口 TX */
#define APP_IRQ_PRIO_USB        (4U)    /* ≤4: CherryUSB 会调 RTOS API */

/* ---------------- 任务优先级 (数值越大越高) ----------------
 *
 * 【为什么日志比控制还高 —— 2026-10-24 实测】
 *   控制任务实测 979Hz(每拍 1.021ms > 1ms 周期), 于是 vTaskDelayUntil() 发现
 *   "已经超期"就【不再阻塞】—— 控制任务永远处于就绪态, 比它低的任务【一次都轮不到】。
 *   实测证据: 共享内存里 log_tail 冻在 7、2 秒内日志任务搬运 0 次, log_dropped 一路涨。
 *   结果就是"屏还在刷, 但串口一行日志都没有", 排错时彻底瞎掉。
 *
 *   把日志任务的优先级放到控制任务之上就解决了: 它 99% 的时间阻塞在
 *   xQueueReceive(10ms), 只有在真有日志时才抢占控制任务【几微秒】, 而且三个后端
 *   都是非阻塞的(控制台是环形缓冲+TX中断; USB 只在主机开了端口时提交;
 *   共享内存后端只是一次 memcpy)。
 *
 * 【为什么不担心它拖慢电流环】电流环 ADC 中断是优先级 7(见上面的表), 而 FreeRTOS
 *   临界区只屏蔽到 4 —— 日志任务里的队列临界区【碰不到】电流环中断。
 *   这一点是这个方案能成立的前提。 */
#define APP_TASK_PRIO_LOG       (configMAX_PRIORITIES - 1)   /* 日志: 最高(理由见上) */
#define APP_TASK_PRIO_CONTROL   (configMAX_PRIORITIES - 2)   /* 控制 */
/* 显示任务不在这里: 它跑在 core1 上, 见 core1/rtos/app_rtos.h */

/**
 * @brief  创建控制/日志任务并启动 core0 的调度器 (不返回)
 * @note   调用前必须完成所有外设与应用的初始化 (board_init / app_init /
 *         core1_start 等), 因为调度器一旦启动, 后续代码就不会再执行。
 */
void app_rtos_start(void);

#endif /* APP_RTOS_H */
