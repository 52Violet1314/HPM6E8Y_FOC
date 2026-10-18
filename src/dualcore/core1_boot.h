/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 双核: core1(显示核) 的启动与运行时协作 —— core0 侧接口
 *
 * 【分工】
 *   core0 = 电机控制核: FOC 电流环(ADC 中断) + 1ms 控制任务 + 日志。
 *   core1 = 显示核:     LVGL + ILI9341 + XPT2046 触摸, 独立镜像、独立 FreeRTOS。
 *
 * 【core1 是怎么起来的】HPM6E80 上电永远从 core0 开始, core1 自己不会启动。
 * 硬件给了一条固定的路(与 SDK 的 multicore 例程一致):
 *   1) core0(其实是链接期)把 core1 的镜像放进 flash 的 0x80100000 起 1MB 里;
 *   2) core0 先读镜像头的 magic/version 确认这段 flash 里确实是 core1 的镜像;
 *   3) core0 调 sysctl_set_cpu_entry() 写入口地址(0x80100010, 同时写 release key);
 *   4) core0 调 sysctl_release_cpu() 清掉 SYSCTL->CPU[1].LP.HALT, core1 从入口取指。
 * 没有这一步, core1 永远停在 halt, 屏不会有任何反应。
 *
 * 【为什么是 flash 就地执行, 而不是像 SDK 例程那样拷进 ILM】
 *   SDK 的例程把镜像拷进 core1 的 ILM(256KB) 再跑, 但本工程显示核带 LVGL 9,
 *   实测 .text + .rodata ≈ 700KB, ILM 装不下(详见 core1/linker/core1_flash.ld 的
 *   说明)。所以改成: core1 的代码留在 flash 里就地执行, 数据/栈用自己的
 *   AXI_SRAM + DLM。好处是 core0 不需要在启动时搬 700KB, 也没有 cache 一致性问题。
 */
#ifndef CORE1_BOOT_H
#define CORE1_BOOT_H

#include <stdint.h>

/**
 * @brief  检查 core1 镜像头(magic/version)并释放 core1 (不等待它跑起来)
 * @note   必须在 board_init()/app_init() 之后调: core1 依赖 core0 配好的时钟树、
 *         电源和引脚复用; 它自己只补"每核独立"的那部分(时钟计数/PMP-PMA)。
 *         调度器起来之前调最省事(主循环还没开始, 不会和 1ms 任务抢总线)。
 *         失败时只报错, 不封锁电机输出 —— 显示是附属功能, 电机该照常能转,
 *         但镜像大小越界这种情况会【拒绝释放】, 免得 core1 跑去执行垃圾。
 */
void core1_start(void);

/**
 * @brief  和 core1 交换一次数据 (控制任务每拍调, 1kHz)
 * @note   上行: 把 app 层的转速/状态快照写进共享内存;
 *         下行: 把显示核(滑条)下发的目标转速交给 app_speed_target_set()。
 *         两个方向都是无锁的, 单写者, 耗时 ~1us 量级, 不阻塞。
 */
void core1_sync(void);

/**
 * @brief  把 core1 攒在共享内存里的日志行转发到 core0 的日志后端
 * @note   必须由【日志任务】调用(core1 没有控制台, 它的日志只能借 core0 的
 *         串口/USB 出去)。每条日志一次, 每拍有上限, 不会把日志任务堵住。
 */
void core1_log_drain(void);

/**
 * @brief  检查 core1 心跳, 变了就报一行 (由日志任务周期调用, 内部 1s 节流)
 * @note   目的很实际: 双核第一次跑起来时, 最常见的问题是"core1 没起来"或
 *         "core1 卡在某一步" —— 这个函数把 core1_stage 直接翻译成中文日志。
 */
void core1_health_check(void);

#endif /* CORE1_BOOT_H */
