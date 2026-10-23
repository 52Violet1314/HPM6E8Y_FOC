/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核): 把"控制台"接管到共享内存日志环
 *
 * 【为什么必须有这个文件】SDK 的 hpm_debug_console.c 里对 bsp_console_write() 是
 * 弱定义, printf/_write 最终都走到它。而 core1 上这条路是危险的:
 *   1) 控制台 UART 是 core0 的(UART0): core1 的 board.h 视角里
 *      BOARD_CONSOLE_UART_BASE 会变成 UART1(board.h 按 BOARD_RUNNING_CORE 选),
 *      本板没把 UART1 接到调试口;
 *   2) 更糟的是 UART1 的时钟/引脚根本没初始化 —— SDK 的写出实现是"等 TX FIFO 有
 *      空位"的轮询循环, 一个卡在复位态的 UART 会让它【死等下去】, 直接挂死显示核。
 *   3) 两个核同时往一个 UART 写还会把行打碎。
 *
 * 谁会用 printf? LVGL 的日志(lv_conf.h 里 LV_LOG_PRINTF=1, LV_USE_LOG=1)、
 * 断言、以及 SDK 里零星的 printf。它们在 core1 上不该消失(那是排错信息), 也不该
 * 挂住显示核, 所以这里把弱定义覆盖掉: 按行攒好, 塞进共享内存的日志环, 由 core0
 * 的日志任务搬到串口 —— 于是串口上两个核的日志是一条完整的时间线, 而且
 * core1 这边【永不阻塞】。
 *
 * 注意: 日志环满了就丢(见 shm_log_push), 这里同样不等待。
 */
#include <string.h>

#include "board.h"
#include "app_log.h"        /* bsp_console_write 的声明: 与 SDK 的弱定义同一签名 */
#include "shm_core.h"
#include "console_uart14.h" /* core1 独立控制台(UART14) */

#define CORE1_CONSOLE_LINE_MAX  (SHM_LOG_LINE_LEN)

static char s_line[CORE1_CONSOLE_LINE_MAX];
static unsigned s_used;

/* 把攒好的一行送进环 */
static void core1_console_flush_line(void)
{
    if (s_used > 0U) {
        s_line[s_used] = '\0';
        (void)shm_log_push(shm_get(), s_line);
        s_used = 0U;
    }
}

/* 覆盖 SDK 的弱定义 (签名见 hpm_debug_console.h / board.h 的用法) */
int bsp_console_write(const char *data, int size)
{
    int i;

    if ((data == NULL) || (size <= 0)) {
        return 0;
    }

#if CORE1_CONSOLE_OWN_UART
    /* UART14 就绪就直接从自己的串口出去(printf / LVGL 日志 / SDK 断言都走这里)。
     * 这条路径和 core0 完全无关, 所以 core0 的控制环再忙, 日志也不会消失。 */
    if (core1_console_uart_ready() != 0) {
        return core1_console_uart_write(data, size);
    }
#endif
    /* 还没初始化(上电最早期)或没接适配器: 退回共享内存环, 由 core0 转发 */
    for (i = 0; i < size; i++) {
        char c = data[i];

        if (c == '\n') {
            core1_console_flush_line();
            continue;                       /* 换行由 core0 的后端补, 不留空行 */
        }
        if (c == '\r') {
            continue;
        }
        if (s_used >= (CORE1_CONSOLE_LINE_MAX - 1U)) {
            /* 一行太长: 先把它发出去, 剩下的另起一行, 绝不丢内容也绝不等待 */
            core1_console_flush_line();
        }
        s_line[s_used++] = c;
    }
    return size;    /* 全部"收下"了(收进环, 不是收进 UART) */
}
