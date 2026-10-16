/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RTOS: 任务划分与启动
 *
 * 【任务与中断的分工】
 *   ADC 完成中断 (PLIC 1)  电流环 20kHz —— 保持在中断里, 不进任务。
 *                          理由: 被控对象 τ=0.42ms, 只有硬件中断能给到确定的延迟;
 *                          而且它的优先级高于 configMAX_SYSCALL_INTERRUPT_PRIORITY,
 *                          RTOS 关中断时它照样进得来。
 *   控制任务  (最高)       采集(编码器/电流) + 保护(过流/限速) + 遥测 + 换向角计算,
 *                          用 vTaskDelayUntil 锁 1ms 硬周期。原来这些都挤在主循环里,
 *                          一遇到 LCD/USB 就会漂到 1.2ms。
 *   日志任务  (中低)       从队列取日志行 -> 格式化好的字符串写串口/USB。
 *                          格式化(vsnprintf)也从控制任务挪到这里了。
 *   显示任务  (最低)       lv_timer_handler + 触摸扫描。SPI 刷屏很慢(几十 ms),
 *                          放在最低优先级, 抢不到控制任务的时间。
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>        /* strlen: 钩子/致命错误里直接写控制台用 */

#include "app.h"
#include "app_log.h"
#include "app_rtos.h"
#include "lcd_app.h"
#include "bsp_pwm.h"        /* bsp_pwm_enable_output: 出致命错误时封锁输出 */

/* 任务栈 (单位: word = 4 字节) */
#define APP_CTRL_TASK_STACK     (1024U)   /* 4KB: app_run 里有 snprintf 等 */
#define APP_LOG_TASK_STACK      (1024U)   /* 4KB: 只做格式化后的写出 */
#define APP_DISP_TASK_STACK     (2048U)   /* 8KB: LVGL 刷屏比较吃栈 */

static void app_ctrl_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    (void)arg;
    for (;;) {
        app_run();
        /* 锁 1ms 硬周期: 控制节拍不再受日志/显示影响 */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_MAIN_LOOP_MS));
    }
}

static void app_log_task(void *arg)
{
    (void)arg;
    app_log_task_body();      /* 内部是死循环, 不返回 */
}

static void app_disp_task(void *arg)
{
    (void)arg;
    for (;;) {
        lcd_app_task_handler();
        vTaskDelay(pdMS_TO_TICKS(5));   /* 约 200Hz, 与原来 GUI 分频一致 */
    }
}

/* 任务建不起来的统一处理 (定义在 app_rtos_start 之前, 后者会调它) */
static void app_rtos_create_failed(const char *name);

void app_rtos_start(void)
{
    TaskHandle_t h;

    /* 【必须检查返回值】xTaskCreate 失败(堆不够)时返回 NULL, 任务不会建立也不会
     * 报错 —— 现象是"某个功能整个不工作"(例如屏一直停在初始画面), 很难往堆上想。
     * 这里失败就直接报出来并封锁电机输出。 */
    h = xTaskCreate(app_ctrl_task, "ctrl", APP_CTRL_TASK_STACK, NULL,
                    APP_TASK_PRIO_CONTROL, NULL);
    if (h == NULL) {
        app_rtos_create_failed("ctrl");
    }
    h = xTaskCreate(app_log_task, "log", APP_LOG_TASK_STACK, NULL,
                    APP_TASK_PRIO_LOG, NULL);
    if (h == NULL) {
        app_rtos_create_failed("log");
    }
    h = xTaskCreate(app_disp_task, "disp", APP_DISP_TASK_STACK, NULL,
                    APP_TASK_PRIO_DISPLAY, NULL);
    if (h == NULL) {
        app_rtos_create_failed("disp");
    }

    vTaskStartScheduler();     /* 不返回 */

    /* 走到这里说明堆不够, 调度器起不来 —— 这是致命错误, 直接封锁输出 */
    bsp_pwm_enable_output(false);
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
