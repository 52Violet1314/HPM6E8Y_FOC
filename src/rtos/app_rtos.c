/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RTOS: 任务划分与启动 (core0 = 电机控制核)
 *
 * 【任务与中断的分工】
 *   ADC 完成中断 (PLIC 1)  电流环 20kHz —— 保持在中断里, 不进任务。
 *                          理由: 被控对象 τ=0.42ms, 只有硬件中断能给到确定的延迟;
 *                          而且它的优先级高于 configMAX_SYSCALL_INTERRUPT_PRIORITY,
 *                          RTOS 关中断时它照样进得来。
 *   控制任务  (最高)       采集(编码器/电流) + 保护(过流/限速) + 遥测 + 换向角计算,
 *                          用 vTaskDelayUntil 锁 1ms 硬周期。原来这些都挤在主循环里,
 *                          一遇到 LCD/USB 就会漂到 1.2ms。
 *                          每拍还要 core1_sync(): 和显示核换一次遥测/命令。
 *   日志任务  (中低)       从队列取日志行 -> 格式化好的字符串写串口/USB,
 *                          顺带把 core1 攒在共享内存里的日志搬出来、查它的心跳。
 *   显示任务  —— 已经【整个搬到 core1】了(见 core1/rtos/app_rtos.c)。
 *                 刷屏是 SPI + LVGL 的大活儿(几十 ms), 放在同一个核上无论怎么调
 *                 优先级都会干扰 1ms 控制节拍; 搬到 core1 之后 core0 这边连它的
 *                 任务都不存在, "抢不到时间"从调度问题变成了物理隔离。
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>        /* strlen: 钩子/致命错误里直接写控制台用 */

#include "app.h"
#include "app_log.h"
#include "app_rtos.h"
#include "core1_boot.h"    /* core1_sync: 每拍与显示核交换数据 */
#include "bsp_pwm.h"        /* bsp_pwm_enable_output: 出致命错误时封锁输出 */

/* 任务栈 (单位: word = 4 字节) */
#define APP_CTRL_TASK_STACK     (1024U)   /* 4KB: app_run 里有 snprintf 等 */
#define APP_LOG_TASK_STACK      (1024U)   /* 4KB: 只做格式化后的写出 */

static void app_ctrl_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    (void)arg;
    /* 【直写控制台, 绕过日志队列】取证用。
     * 调度器启动后 LOG_I 只进 16 行队列, 由本文件里的日志任务(优先级-3)搬运;
     * 日志任务若因任何原因不跑, 队列满后所有日志静默丢弃 ——
     * 那时串口"在调度器启动后一行都没有", 无法区分"系统卡住"和"日志丢了"。
     * 所以任务入口先用 bsp_console_write() 直接打一行, 证明调度器真的切到了它。 */
    (void)bsp_console_write("[RTOS] ctrl task running\r\n", (int)sizeof("[RTOS] ctrl task running\r\n") - 1);

    for (;;) {
        app_run();
        /* 与显示核换数据: 上行遥测快照, 下行界面设的目标转速(见 core1_boot.c)。
         * 全是无锁的标量读写, 不阻塞, 放在 1ms 节拍里不影响周期。 */
        core1_sync();
        /* 锁 1ms 硬周期: 控制节拍不再受日志/显示影响 */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_MAIN_LOOP_MS));
    }
}

static void app_log_task(void *arg)
{
    (void)arg;
    (void)bsp_console_write("[RTOS] log task running\r\n", (int)sizeof("[RTOS] log task running\r\n") - 1);   /* 直写取证 */
    app_log_task_body();      /* 内部是死循环, 不返回 */
}

/* 任务建不起来的统一处理 (定义在 app_rtos_start 之前, 后者会调它) */
static void app_rtos_create_failed(const char *name);

void app_rtos_start(void)
{
    /* 【必须检查返回值】xTaskCreate 失败(堆不够)时返回 pdFAIL, 任务不会建立也不会
     * 报错 —— 现象是"某个功能整个不工作", 很难往堆上想。这里失败就直接报出来并
     * 封锁电机输出。
     * 【注意返回值不是句柄】FreeRTOS 10.5.1 的 xTaskCreate 返回 BaseType_t, 句柄
     * 从第 6 个参数(pxCreatedTask)带出来。原来写成 `h = xTaskCreate(..., NULL);`
     * 再判 `h == NULL` —— 那个条件永远不成立(句柄被 pdPASS=1 覆盖), 等于没检查。 */
    TaskHandle_t h = NULL;

    if (xTaskCreate(app_ctrl_task, "ctrl", APP_CTRL_TASK_STACK, NULL,
                    APP_TASK_PRIO_CONTROL, &h) != pdPASS) {
        app_rtos_create_failed("ctrl");
    }
    h = NULL;
    if (xTaskCreate(app_log_task, "log", APP_LOG_TASK_STACK, NULL,
                    APP_TASK_PRIO_LOG, &h) != pdPASS) {
        app_rtos_create_failed("log");
    }

    vTaskStartScheduler();     /* 不返回 */

    /* 走到这里说明堆不够, 调度器起不来 —— 这是致命错误, 直接封锁输出 */
    bsp_pwm_enable_output(false);
    (void)bsp_console_write("\r\n[FATAL] vTaskStartScheduler returned: heap too small\r\n", (int)sizeof("\r\n[FATAL] vTaskStartScheduler returned: heap too small\r\n") - 1);
    for (;;) {
    }
}

/* 任务建不起来的统一处理: 报名字 + 封锁输出 + 停在这里 (堆不够是致命错误,
 * 继续跑等于"电机没有控制任务", 绝不能装作没事) */
static void app_rtos_create_failed(const char *name)
{
    (void)bsp_console_write("\r\n[FATAL] xTaskCreate 失败(堆不足): ", 34);
    if (name != NULL) {
        (void)bsp_console_write(name, (int)strlen(name));
    }
    (void)bsp_console_write("\r\n", 2);
    bsp_pwm_enable_output(false);
    for (;;) {
    }
}

/* ---------------- FreeRTOS 钩子 ---------------- */

/* 栈溢出: 直接写控制台(不走日志队列, 避免递归), 然后封锁电机输出 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)bsp_console_write("\r\n[FATAL] FreeRTOS 任务栈溢出: ", 30);
    if (pcTaskName != NULL) {
        (void)bsp_console_write(pcTaskName, (int)strlen(pcTaskName));
    }
    (void)bsp_console_write("\r\n", 2);
    bsp_pwm_enable_output(false);
    for (;;) {
    }
}

/* 堆不足: 同上 */
void vApplicationMallocFailedHook(void)
{
    (void)bsp_console_write("\r\n[FATAL] FreeRTOS 堆不足 (configTOTAL_HEAP_SIZE)\r\n", 48);
    bsp_pwm_enable_output(false);
    for (;;) {
    }
}
