/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核) 程序入口 —— LVGL / ILI9341 / XPT2046
 *
 * 【它在哪、怎么起来的】core1 不是从 flash 复位的: 它的镜像被 core0 的构建链接到
 * flash 的 0x80100000 起 1MB(core0 的 elf 里带着这一段), 由 core0 在
 * src/dualcore/core1_boot.c 里检查镜像头、写入口地址(0x80100010)、再清掉
 * SYSCTL->CPU[1].LP.HALT 释放 core1。core1 从那里开始【就地执行(XIP)】。
 * 为什么不像 SDK 的 multicore 例程那样拷进 ILM: LVGL 太大, 256KB ILM 装不下,
 * 详见 core1/linker/core1_flash.ld 顶部。
 *
 * 所以:
 *   - 时钟树 / 电源 / PLL / 引脚复用 已经由 core0 的 board_init() 配好了;
 *     这里【绝对不能】再调 board_init(): 那会重新配 PLL, 把正在跑的 core0 带崩。
 *   - 但有两样东西是【每个核独立的】, 必须自己补:
 *       clock_update_core_clock(): 本核 mhartid=1 -> hpm_core_clock = CPU1 频率,
 *                                  所有 board_delay_xxx / 超时换算都依赖它;
 *       board_init_pmp()          : PMP/PMA 是每核独立的寄存器(见 SDK 的 multicore
 *                                  文档), SHARE_RAM 必须在本核也被配成"非缓存",
 *                                  否则从共享内存读到的是本核 cache 里的旧值。
 *   - 也不要调 board_init_console(): 那是 core0 的控制台(UART0/PA00,PA01)。
 *     【core1 现在有自己的控制台了】UART14 = PF24(TXD)/PF25(RXD) = J11 的 40/37 脚,
 *     见 console_uart14.c: core1 的日志直接走这条线, 不再依赖 core0 的日志任务。
 *     (原来那套"全部写进共享内存环、由 core0 转发"的路仍然保留着, 作为
 *      上电最早期和没接适配器时的退路。)
 */
#include "board.h"

#include "app_log.h"
#include "console_uart14.h"     /* core1 独立控制台: UART14 = PF24/PF25 = J11 40/37 */
#include "core1_boot_info.h"    /* CORE1_IMG_BASE / CORE1_IMG_ENTRY: 本镜像的落点 */
#include "lcd_app.h"
#include "shm_client.h"
#include "shm_core.h"
#include "app_rtos.h"

int main(void)
{
    /* ---- 1) 补齐"每核独立"的初始化 (理由见文件头) ---- */
    clock_update_core_clock();
    board_init_pmp();

    /* 【必须放在第一行日志之前】把 core1 的日志从"共享内存 -> core0 的日志任务"
     * 改到自己的 UART14 上。之后 core0 的控制环再忙, 这边的日志照样出得来。
     * 初始化失败也不影响启动: ready() 返回 0, 日志自动退回共享环。 */
    core1_console_uart_init();

    shm_client_stage(CORE1_STAGE_ENTER);

    /* ---- 2) 和 core0 的共享内存握手 ----
     * core0 是"先写好共享区再释放 core1"的顺序, 所以这里正常一次就过。
     * 校验 magic/version 是刻意的: 万一两边镜像对共享区布局的理解不一致
     * (改结构体只编了其中一个核), 宁可停在这里让 core0 报"阶段 225",
     * 也不要在错误的地基上刷屏、再花几个小时怀疑硬件。 */
    if (shm_client_attach() == 0) {
        shm_client_stage(CORE1_STAGE_SHM_BAD);
        for (;;) {
        }
    }
    shm_client_stage(CORE1_STAGE_SHM_OK);

    LOG_I("CORE1", "显示核启动: CPU1 %uMHz, 代码 @flash 0x%08X (入口 0x%08X), 共享区 0x%08X (%u 字节)",
          (unsigned)(clock_get_frequency(clock_cpu1) / 1000000U),
          (unsigned)CORE1_IMG_BASE,
          (unsigned)CORE1_IMG_ENTRY,
          (unsigned)(uintptr_t)shm_get(),
          (unsigned)sizeof(shm_t));

    /* ---- 3) 面板 + 触摸 (SPI1 轮询/DMA + 软件 SPI 触摸) ---- */
    lcd_app_hardware_init();
    shm_client_stage(CORE1_STAGE_PANEL_OK);

    /* ---- 4) LVGL: 显示/输入设备 + 电机界面 ---- */
    lcd_app_ui_init();
    shm_client_stage(CORE1_STAGE_UI_OK);

    /* ---- 5) 交给 core1 自己的 FreeRTOS: 只有显示任务 (不返回) ---- */
    LOG_I("CORE1", "界面就绪, 启动显示任务 (5ms/拍)");
    core1_rtos_start();

    return 0;   /* 不会到这里 */
}
