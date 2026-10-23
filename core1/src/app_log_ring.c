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
#include "console_uart14.h"     /* core1 独立控制台(UART14): 不再吊在 core0 的日志任务上 */

#define APP_LOG_LINE_LEN    (256)

/* 【core1 的日志走哪条路】
 *   1) UART14(自己的串口) —— 只要接上 USB-TTL 就能看到, 和 core0 死活无关;
 *   2) 共享内存正向环(log) —— core0 的日志任务搬出去转发。
 *
 * 【2026-10-24: 两者不再同时用】因为 core0 的日志现在会经"反向环(log0)"交给 core1
 * 从 UART14 输出 —— 如果 core1 还同时往正向环里推, 那条路会绕一圈又回到 core1,
 * 同一条日志在串口上出现两遍。所以改成二选一:
 *   UART14 就绪 -> 只走 1;  否则(极早期/初始化失败) -> 退回 2, 由 core0 转发。
 *
 * 把 CORE1_LOG_KEEP_SHM_RING 改成 1 = "永远两条都发"(会重复, 只在排查转发链路时用)。 */
#ifndef CORE1_LOG_KEEP_SHM_RING
#define CORE1_LOG_KEEP_SHM_RING (0)
#endif

static void core1_log_emit(const char *line)
{
    if (line == NULL) {
        return;
    }
#if CORE1_CONSOLE_OWN_UART
    if (core1_console_uart_ready() != 0) {
        core1_console_uart_write_line(line);
#if CORE1_LOG_KEEP_SHM_RING
        (void)shm_log_push(shm_get(), line);
#endif
        return;                     /* 已经写出去了, 不再绕 core0 */
    }
#endif
    /* UART14 还没就绪: 退回共享环, 由 core0 的日志任务转发(绝不静默丢日志) */
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
