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

/**
 * @brief  绕开日志队列, 立刻写所有后端
 * @param  line  已格式化好的文本 (不含结尾 '\n')
 * @note   只有两处该用它:
 *           1) core0 的日志任务把 core1 攒在共享内存里的日志搬出来转发
 *              (见 src/dualcore/core1_boot.c 的 core1_log_drain) —— 它本身就跑在
 *              日志任务里, 再入队等于自己给自己发消息, 白白多绕一圈还可能把队列塞满;
 *           2) 调度器起来之前的致命错误路径。
 *         其它场合请用 LOG_* / app_log_write_raw(), 别在控制任务里直接调它 ——
 *         那会把 vsnprintf 之后的串口写出重新拉回控制任务。
 */
void app_log_write_direct(const char *line);

/**
 * @brief  日志任务的循环体 (由 RTOS 的日志任务调用, 内部不返回)
 *
 * 【为什么要有它】LOG_* 现在只把"格式化好的行"塞进队列, 真正的后写出
 * (串口环形缓冲 + USB 提交) 由这个任务做。这样控制任务里不再有 vsnprintf
 * 和 USB 相关逻辑, 1kHz 节拍更稳。
 * 队列满时整行丢弃(绝不阻塞), 因为日志永远不该拖慢控制。
 */
void app_log_task_body(void);

#ifdef __cplusplus
}
#endif

/**
 * @brief  非阻塞控制台写出 (printf / newlib 的 _write 最终都落到这里)
 * @param  data  要写出的字节(可以不是一整行)
 * @param  size  字节数
 * @return 实际"收下"的字节数; 缓冲满时只收一部分(丢弃剩余), 绝不等待
 *
 * @note   SDK 的 hpm_debug_console.c 里对它是【弱定义】(阻塞式轮询 UART),
 *         链接时由本工程选强定义, printf 于是也走非阻塞路径:
 *           core0: src/app/app.c —— 串口环形缓冲 + TX 中断;
 *           core1: core1/src/console_ring.c —— 塞进共享内存日志环, 由 core0 输出
 *                  (core1 上既没有控制台 UART, 也绝不能在 UART 上死等)。
 *         这里声明一次, 免得调用方靠隐式声明(返回类型默认 int, 容易埋雷)。
 */
int bsp_console_write(const char *data, int size);

/* ================= 延迟格式化日志 (控制任务热路径专用) =================
 *
 * 【为什么需要】LOG_*() 的 vsnprintf 是在【调用者上下文】里跑的 —— 控制任务自己
 * 打日志时, 这笔开销就记在控制任务头上(见 app_log.c 里 app_log_write 的实现)。
 * 实测每秒那几条长日志(ctrl period / ADC ISR / 电流环峰峰值 / FOC 状态行, 合计
 * 约 500 字符、25 个转换)会在【同一拍】里连续格式化, 让那一拍多花 100~200us,
 * 占 1ms 周期的 10~20% —— 外环(速度/位置)晚一拍, 现象就是"打印一下电机顿一下"。
 *
 * 用法: 控制任务只把【已经算好的整数】塞进 app_log_rec_t 投递(约 0.3us),
 *       真正的 snprintf 交给日志任务。格式串仍然留在消息所在的那份文件中
 *       (用 app_log_set_formatter 注册一个回调即可)。
 *
 * 注意: 浮点参数请先自己定标成整数(例如 (int)(id * 1000.0f)), 因为这个结构体
 *       只装 int32_t —— 一次软件浮点乘法约 50ns, 和 vsnprintf 比可以忽略。
 */
#define APP_LOG_REC_MAX_ARGS   (16)

typedef struct {
    uint16_t id;                        /* 消息编号, 由格式化回调解释 */
    uint8_t  n;                         /* 有效参数个数(<= APP_LOG_REC_MAX_ARGS) */
    int32_t  a[APP_LOG_REC_MAX_ARGS];   /* 参数(整数) */
} app_log_rec_t;

/** 格式化回调: 由【日志任务】调用, 把 rec 拼成一整行(含 "[INFO] [TAG] " 前缀, 不含换行) */
typedef void (*app_log_fmt_fn_t)(const app_log_rec_t *rec, char *out, uint32_t out_sz);

/** 注册格式化回调 (启动时调一次) */
void app_log_set_formatter(app_log_fmt_fn_t fn);

/**
 * @brief  投递一条"延迟格式化"的日志 —— 可以从控制任务里调, 开销约 0.3us
 * @return 1 = 已入队; 0 = 队列满被丢弃(或退化成立刻格式化)
 * @note   非阻塞。队列满直接丢, 绝不等待。
 */
int  app_log_post(const app_log_rec_t *rec);

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
