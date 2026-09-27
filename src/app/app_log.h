/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 日志模块 (分级 + 模块 TAG + 多后端)
 *
 *   提供分级日志 (DEBUG/INFO/WARN/ERROR) 与多后端输出 (printf 调试口 + USB 虚拟串口),
 *   运行时通过 app_log_add_backend() 注册后端。应用代码统一用 LOG_I/LOG_W 等宏,
 *   不再直接调 printf / bsp_usb_write, 便于:
 *     - 统一日志格式: [级别][模块] 正文, 如 [INFO][FOC] PWM init done
 *     - 按级别全局开关 (改 APP_LOG_LEVEL 即可静默调试日志)
 *     - 后端解耦 (换输出口只改注册, 不动业务代码)
 *
 * 用法:
 *     LOG_I("APP",  "系统启动完成, 版本 %d.%d", 2, 0);
 *     LOG_W("FOC",  "过流折叠, imax=%d mA", imax);
 *     LOG_E("BSP",  "ADC 初始化失败");
 *     LOG_D("ENC",  "angle=%u", angle);   // 调试级, 可整体静默
 */
#ifndef APP_LOG_H
#define APP_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 日志级别 (数值越大越严重) ----
 * 同时提供数字宏 (供 #if 预处理求值) 与枚举 (供运行时使用)。 */
#define APP_LOG_LVL_DEBUG   (0)
#define APP_LOG_LVL_INFO    (1)
#define APP_LOG_LVL_WARN    (2)
#define APP_LOG_LVL_ERROR   (3)

typedef enum {
    app_log_debug = APP_LOG_LVL_DEBUG,
    app_log_info  = APP_LOG_LVL_INFO,
    app_log_warn  = APP_LOG_LVL_WARN,
    app_log_error = APP_LOG_LVL_ERROR,
} app_log_level_t;

/* 编译期全局日志级别: 用上面的数字宏, 低于此级别的 LOG_x 调用被完全裁剪掉。
 * 例: -DAPP_LOG_LEVEL=APP_LOG_LVL_INFO 可静默所有 DEBUG 日志。 */
#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL   APP_LOG_LVL_DEBUG
#endif

/* ---- 后端 ---- */
/** 后端输出回调: 输出一行文本 (不含结尾 '\n') */
typedef void (*app_log_backend_fn)(const char *line);

/**
 * @brief  注册一个日志后端
 * @param  fn  输出回调; 传 NULL 表示注销该槽位
 * @return 后端槽位索引 (0~APP_LOG_MAX_BACKENDS-1); 已满返回 -1
 */
int  app_log_add_backend(app_log_backend_fn fn);

/**
 * @brief  输出一条日志 (由 LOG_* 宏调用, 一般不需直接使用)
 * @param  level  级别
 * @param  tag    模块名 (如 "APP"/"FOC"/"BSP"/"ENC"), 可为 NULL 表示无模块名
 * @param  fmt    printf 风格格式串
 * @param  ...    参数
 */
void app_log_write(app_log_level_t level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * @brief  原样输出一行 (不追加级别/模块前缀, 用于遥测等固定格式上报)
 * @param  line  已格式化好的文本 (不含结尾 '\n')
 */
void app_log_write_raw(const char *line);

#ifdef __cplusplus
}
#endif

/* ---- 便捷宏: LOG_[级别](模块TAG, 格式串, ...) ---- */
#if APP_LOG_LEVEL <= APP_LOG_LVL_DEBUG
#define LOG_D(tag, fmt, ...)  app_log_write(app_log_debug, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_D(tag, fmt, ...)  ((void)0)
#endif

#if APP_LOG_LEVEL <= APP_LOG_LVL_INFO
#define LOG_I(tag, fmt, ...)  app_log_write(app_log_info, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_I(tag, fmt, ...)  ((void)0)
#endif

#if APP_LOG_LEVEL <= APP_LOG_LVL_WARN
#define LOG_W(tag, fmt, ...)  app_log_write(app_log_warn, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_W(tag, fmt, ...)  ((void)0)
#endif

#if APP_LOG_LEVEL <= APP_LOG_LVL_ERROR
#define LOG_E(tag, fmt, ...)  app_log_write(app_log_error, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_E(tag, fmt, ...)  ((void)0)
#endif

#endif /* APP_LOG_H */
