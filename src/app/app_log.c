/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 日志模块实现
 */

#include <stdarg.h>
#include <stdio.h>

#include "app_log.h"

#define APP_LOG_MAX_BACKENDS   (4)
#define APP_LOG_LINE_LEN       (256)

static app_log_backend_fn s_backends[APP_LOG_MAX_BACKENDS];

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

void app_log_write(app_log_level_t level, const char *tag, const char *fmt, ...)
{
    char line[APP_LOG_LINE_LEN];
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

    for (int i = 0; i < APP_LOG_MAX_BACKENDS; i++) {
        if (s_backends[i] != NULL) {
            s_backends[i](line);
        }
    }
}

void app_log_write_raw(const char *line)
{
    if (line == NULL) {
        return;
    }
    for (int i = 0; i < APP_LOG_MAX_BACKENDS; i++) {
        if (s_backends[i] != NULL) {
            s_backends[i](line);
        }
    }
}
