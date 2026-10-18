/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1 侧: 日志模块实现 —— 写进共享内存, 由 core0 的日志任务搬到串口/USB
 *
 * 【为什么不直接用 printf】core1 上没有控制台: 控制台 UART0 归 core0, 而且
 * core1 的 board.h 视角下控制台会变成 UART1(另一根线, 本板没接调试口)。两个核
 * 同时抢同一个 UART 还会把行打碎。所以 core1 的日志统一走共享内存里的槽位环,
 * 由 core0 唯一的写者负责真正输出 —— 和 core0 内部"日志任务统一写出"的思路一致。
 *
 * 【和 core0 的 app_log.c 的关系】接口完全一样(同一个 app_log.h), 只是"后端"
 * 从 串口+USB 换成了 共享内存。所以 src/lcd 下的 LOG_I("LCD", ...) 不用改。
 *
 * 【会不会阻塞刷屏】不会: 入队是纯内存操作, 环满直接丢并计数(shm->log_dropped),
 * 绝不等 core0。刷屏卡一帧的代价比丢一行日志大得多。
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "shm_core.h"
#include "app_log.h"

#define APP_LOG_LINE_LEN    (256)

/* 入队实现 (三个对外入口共用) */
static void core1_log_emit(const char *line)
{
    if (line == NULL) {
        return;
    }
    (void)shm_log_push(shm_get(), line);
}

void app_log_write(app_log_level_t level, const char *tag, const char *fmt, ...)
{
    /* 栈上格式化: core1 只有显示任务会打日志, 而且入队是同步拷贝,
     * 不需要像 core0 那样留 static 缓冲给异步 USB 后端。 */
    char line[APP_LOG_LINE_LEN];
    static const char *const prefix[] = {
        "[DEBUG] ", "[INFO] ", "[WARN] ", "[ERROR] ",
    };
    va_list ap;
    int n;
    int used = 0;

    n = snprintf(line, sizeof(line), "%s", prefix[level]);
    if ((n < 0) || (n >= (int)sizeof(line))) {
        return;
    }
    used = n;

    if ((tag != NULL) && (tag[0] != '\0')) {
        n = snprintf(line + used, sizeof(line) - (size_t)used, "[%s] ", tag);
        if ((n < 0) || (n >= (int)(sizeof(line) - (size_t)used))) {
            return;
        }
        used += n;
    }

    va_start(ap, fmt);
    (void)vsnprintf(line + used, sizeof(line) - (size_t)used, fmt, ap);
    va_end(ap);

    core1_log_emit(line);
}

void app_log_write_raw(const char *line)
{
    core1_log_emit(line);
}

void app_log_write_direct(const char *line)
{
    /* core1 上没有"队列"这一层, 直接入队即等价于 core0 的"立刻写后端" */
    core1_log_emit(line);
}
