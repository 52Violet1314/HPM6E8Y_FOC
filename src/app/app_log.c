/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 日志模块实现
 *
 * 【RTOS 下的分工】
 *   调用 LOG_* 的地方(控制任务等)只做: 格式化 -> 塞进队列 -> 返回。
 *   真正的写出(串口环形缓冲 + USB 提交)由日志任务做。
 *   这样控制任务里不再有 vsnprintf / USB 相关逻辑, 1kHz 节拍更稳。
 *
 *   【双核之后的额外一件事】core1(显示核)没有控制台, 它的日志写在共享内存的
 *   槽位环里(见 core1/src/app_log_ring.c)。日志任务顺带负责把那些行搬出来转发
 *   (core1_log_drain), 以及监控 core1 的心跳(core1_health_check) —— 于是串口上
 *   看到的是两个核混在一起的完整时间线, 不需要第二个串口。
 *
 *   队列满 -> 整行丢弃, 绝不阻塞: 日志永远不该拖慢控制。
 *   调度器还没启动(上电初始化早期) -> 直接写后端, 否则早期日志会卡在队列里出不来。
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "app_log.h"
#include "core1_boot.h"     /* core1_log_drain / core1_health_check */

#define APP_LOG_MAX_BACKENDS   (4)
#define APP_LOG_LINE_LEN       (256)
#define APP_LOG_QUEUE_LEN      (16)     /* 16 行 x 256B = 4KB: 够扛住一次突发 */
/* 延迟格式化记录的队列深度: 热路径每秒最多几条, 8 足够 */
#define APP_LOG_REC_QUEUE_LEN  (8)

static app_log_backend_fn s_backends[APP_LOG_MAX_BACKENDS];
static QueueHandle_t      s_log_q;
static QueueHandle_t      s_rec_q;      /* app_log_rec_t 队列(控制任务 -> 日志任务) */
static app_log_fmt_fn_t   s_fmt;        /* 延迟记录的格式化回调 */

int app_log_add_backend(app_log_backend_fn fn)
{
    for (int i = 0; i < APP_LOG_MAX_BACKENDS; i++) {
        if (s_backends[i] == NULL) {
            s_backends[i] = fn;
            return i;
        }
    }
    return -1;
}

/* 队列懒创建: 这样初始化早期(还没建队列时)也能正常打日志 */
static void app_log_ensure_queue(void)
{
    if (s_log_q == NULL) {
        s_log_q = xQueueCreate(APP_LOG_QUEUE_LEN, APP_LOG_LINE_LEN);
    }
}

/* 后端直写 (初始化早期 / 日志任务里都用它) */
static void app_log_call_backends(const char *line)
{
    for (int i = 0; i < APP_LOG_MAX_BACKENDS; i++) {
        if (s_backends[i] != NULL) {
            s_backends[i](line);
        }
    }
}

static void app_log_dispatch(const char *line)
{
    app_log_ensure_queue();

    /* 调度器没起来时队列没人消费, 直接写, 避免日志堆在队列里 */
    if ((s_log_q == NULL) || (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)) {
        app_log_call_backends(line);
        return;
    }
    (void)xQueueSend(s_log_q, line, 0);     /* 满则丢弃, 绝不阻塞 */
}

/* ================= 延迟格式化日志 (控制任务热路径) =================
 * 见 app_log.h 顶部的说明: 把 vsnprintf 从控制任务挪到日志任务。 */

void app_log_set_formatter(app_log_fmt_fn_t fn)
{
    s_fmt = fn;
}

/* 没有队列/调度器还没起来: 退化成"当场格式化", 宁可慢一点也不能丢日志 */
static void app_log_post_now(const app_log_rec_t *rec)
{
    char line[APP_LOG_LINE_LEN];

    if (s_fmt == NULL) {
        return;
    }
    s_fmt(rec, line, sizeof(line));
    app_log_call_backends(line);
}

int app_log_post(const app_log_rec_t *rec)
{
    if ((rec == NULL) || (s_fmt == NULL)) {
        return 0;
    }

    if (s_rec_q == NULL) {
        s_rec_q = xQueueCreate(APP_LOG_REC_QUEUE_LEN, sizeof(app_log_rec_t));
    }
    if ((s_rec_q == NULL) || (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)) {
        app_log_post_now(rec);
        return 0;
    }

    /* 【这里就是全部开销】一次定长结构体拷贝 + 入队, 没有 snprintf。
     * 拿不到锁/队列满都直接丢 —— 日志永远不该让控制任务等。 */
    return (xQueueSend(s_rec_q, rec, 0) == pdTRUE) ? 1 : 0;
}

void app_log_write(app_log_level_t level, const char *tag, const char *fmt, ...)
{
    /* 【必须是 static】USB 后端是"异步提交"(见 bsp_usb.c): 函数返回后端点仍在
     * 读这块内存。放栈上会被下次调用覆盖, 主机收到错乱日志。
     * (控制任务和日志任务各自调用本函数, 但日志行是送进队列后由日志任务统一
     *  写出的, 所以这里的暂存只需要在"入队那一刻"有效。) */
    static char line[APP_LOG_LINE_LEN];
    va_list ap;
    int n;
    int used = 0;

    static const char *const prefix[] = {
        "[DEBUG] ", "[INFO] ", "[WARN] ", "[ERROR] ",
    };

    /* 1) 级别 TAG */
    n = snprintf(line, sizeof(line), "%s", prefix[level]);
    if (n < 0 || n >= (int)sizeof(line)) {
        return;
    }
    used = n;

    /* 2) 模块 TAG (可选) */
    if (tag != NULL && tag[0] != '\0') {
        n = snprintf(line + used, sizeof(line) - (size_t)used, "[%s] ", tag);
        if (n < 0 || n >= (int)(sizeof(line) - (size_t)used)) {
            return;
        }
        used += n;
    }

    /* 3) 正文 (尾部不加 '\n', 由后端决定换行) */
    va_start(ap, fmt);
    (void)vsnprintf(line + used, sizeof(line) - (size_t)used, fmt, ap);
    va_end(ap);

    app_log_dispatch(line);
}

void app_log_write_raw(const char *line)
{
    if (line == NULL) {
        return;
    }
    app_log_dispatch(line);
}

void app_log_write_direct(const char *line)
{
    if (line == NULL) {
        return;
    }
    app_log_call_backends(line);
}

/* 日志任务: 从队列取行并写出 (不死循环返回)
 *
 * 【为什么 xQueueReceive 要带超时】显示核(core1)的日志也归这个任务转发:
 *   它把日志行写进共享内存的槽位环(见 src/dualcore/shm_core.h), core0 这边
 *   定期来搬。如果这里用 portMAX_DELAY 死等队列, 没有本地日志时任务就永远躺在
 *   阻塞态, core1 的日志会一直积在环里出不来(表现出来就是"屏上的日志串口看不到")。
 *   10ms 的超时既不影响本地日志的实时性(有日志立刻被唤醒), 又保证空转时每 10ms
 *   去看一眼 core1 那边。 */
void app_log_task_body(void)
{
    static char line[APP_LOG_LINE_LEN];
    app_log_rec_t rec;

    for (;;) {
        if ((s_log_q != NULL) &&
            (xQueueReceive(s_log_q, line, pdMS_TO_TICKS(10)) == pdTRUE)) {
            app_log_call_backends(line);
        }

        /* ---- 延迟格式化的记录: 【snprintf 在这里跑, 不在控制任务里】 ---- */
        while ((s_rec_q != NULL) && (xQueueReceive(s_rec_q, &rec, 0) == pdTRUE)) {
            app_log_post_now(&rec);
        }

        /* core1 的日志 + 它的心跳检查(内部 1s 节流) */
        core1_log_drain();
        core1_health_check();
    }
}
