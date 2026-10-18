/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核) 的任务与启动
 *
 * 分工: 只有一个显示任务, 每 5ms 跑一次 lcd_app_task_handler()
 *       (内部 = 拉遥测 + LVGL 定时器/重绘 + 触摸扫描), 周期与原来 core0 上的
 *       显示任务完全相同, 所以界面手感不变, 变的是它现在占的是 core1 的时间。
 */
#include "FreeRTOS.h"
#include "task.h"

#include "app_log.h"
#include "lcd_app.h"
#include "shm_client.h"
#include "shm_core.h"
#include "app_rtos.h"

/* 8KB: LVGL 刷屏 + 触摸 + 日志格式化都在这一个任务里, 比 core0 的控制任务吃栈 */
#define CORE1_DISP_TASK_STACK   (2048U)

static void core1_disp_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    (void)arg;

    shm_client_stage(CORE1_STAGE_RTOS_OK);

    for (;;) {
        /* 1) 先拿一帧一致的遥测快照: 界面这一拍读到的都是同一时刻的数据 */
        shm_client_pull();

        /* 2) LVGL: 刷新文字/滑条 + 定时器 + 重绘 + 触摸扫描 (与 core0 同一个入口) */
        lcd_app_task_handler();

        /* 3) 心跳: core0 靠它判断显示核是否还在跑, 停了会在串口报警 */
        shm_client_beat();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(CORE1_DISP_PERIOD_MS));
    }
}

void core1_rtos_start(void)
{
    TaskHandle_t h = NULL;

    /* 【返回值不是句柄】FreeRTOS 10.5.1 的 xTaskCreate 返回 BaseType_t(pdPASS/pdFAIL),
     * 句柄从第 6 个参数带出来 —— 写成 `h = xTaskCreate(...)` 再判 NULL 是永远不成立
     * 的(等于没检查), core0 那边原来就是这么写的, 这里一并按正确用法来。 */
    if (xTaskCreate(core1_disp_task, "disp", CORE1_DISP_TASK_STACK, NULL,
                    APP_TASK_PRIO_DISPLAY, &h) != pdPASS) {
        /* 堆不够: 显示核就没有任务了。报给 core0(它会打到串口), 然后停住 ——
         * 继续跑一个没有显示任务的调度器只会让人以为"屏坏了"。 */
        shm_client_fault(CORE1_FAULT_TASK_CREATE);
        for (;;) {
        }
    }

    vTaskStartScheduler();      /* 不返回 */

    /* 走到这里 = 空闲任务都建不起来(堆不足), 同样是致命错误 */
    shm_client_fault(CORE1_FAULT_TASK_CREATE);
    for (;;) {
    }
}

/* ---------------- FreeRTOS 钩子 ----------------
 * core1 的钩子不能像 core0 那样直接写控制台(它没有控制台), 只能:
 *   1) 把错误码写进共享内存 -> core0 的日志任务打出来;
 *   2) 顺手往日志环塞一行(能塞进去就塞), 这样现场信息更全;
 *   3) 停在原地, 让 core0 的心跳检查报"core1 停摆"。 */
static void core1_die(uint32_t fault, const char *why, const char *task_name)
{
    shm_client_fault(fault);
    LOG_E("CORE1", "%s%s%s", why,
          (task_name != NULL) ? ": " : "",
          (task_name != NULL) ? task_name : "");
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    core1_die(CORE1_FAULT_STACK_OVERFLOW, "任务栈溢出", pcTaskName);
}

void vApplicationMallocFailedHook(void)
{
    core1_die(CORE1_FAULT_MALLOC, "FreeRTOS 堆不足(configTOTAL_HEAP_SIZE)", NULL);
}
