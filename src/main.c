/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * YeeZi HPM6EMotor V2.0 (HPM6E8Y) 程序入口 —— core0(电机控制核)
 *
 * 双核分工:
 *   core0 (本文件): 电机控制 —— ADC 中断里的电流环 + 1ms 控制任务 + 日志/遥测。
 *   core1 (core1/): 显示 —— LVGL + ILI9341 + XPT2046, 独立镜像、独立 FreeRTOS。
 *   两个核通过 SHARE_RAM 交换数据, 协议见 src/dualcore/shm_core.h;
 *   core1 的镜像在 flash 的 0x80100000 起(core1 就地执行), core0 只负责检查
 *   镜像头并把入口地址写给硬件, 见 src/dualcore/core1_boot.c。
 *
 * 分层架构:
 *   main  -> 只做 board_init + app_init + 启动 core1 + 启动 RTOS
 *   app   -> 采集(BSP) + 控制(FOC) + 上报 的编排 (src/app)
 *   FOC   -> SVPWM / 开环 / 电流环 控制算法 (src/FOC)
 *   BSP   -> PWM/ADC/USB/SEI 硬件驱动, 中断留在 BSP, 经注册回调上抛 (src/bsp)
 *   Hardware -> 传感器协议层(纯解析, 无硬件) (src/Hardware/mt6701)
 *   dualcore -> 跨核启动与共享内存 (src/dualcore, core1/)
 */

#include "board.h"

#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"

#include "app.h"
#include "app_diag.h"       /* diag_set_stage: 给中断诊断标记"主循环在干什么" */
#include "app_log.h"
#include "app_rtos.h"
#include "core1_boot.h"     /* core1_start: 把显示核拉起来 */

int main(void)
{
    /* 板级初始化: 时钟树、引脚复用等, 必须最先调用 */

    board_init();

    /* 应用初始化: 外设 + FOC + 回调注册 */
    app_init();

    /* LCD/LVGL 的初始化和显示任务现在都在 core1 上(见 core1/src/main.c)。
     * core0 这边只剩一件事: 检查 core1 的镜像头, 然后把入口地址写给硬件、释放 core1。
     * 必须放在 app_init() 之后 —— core1 依赖 core0 配好的时钟/电源/引脚;
     * 放在调度器之前最省事, 主循环还没开始, 不会和 1ms 控制任务抢总线。 */
    core1_start();

    /* 交给 RTOS: 控制任务(最高优先级, 硬 1ms) + 日志任务 + 双核数据交换。
     *
     * 【为什么换掉原来的 while(1) 补周期写法】
     *   原写法用 mcycle 把周期精确补到 1ms —— 那解决了"周期漂移"的一部分问题,
     *   但 app_run 里只要遇上 LCD 刷屏/USB 提交, 整拍就会超过 1ms(实测 0.96~1.2ms
     *   来回变): 编码器采样率、遥测时序、限速判断都跟着漂。
     *   现在控制任务用 vTaskDelayUntil 锁 1ms 硬周期, 日志降为低优先级任务;
     *   而最占时间的刷屏(GUI + SPI + 触摸)整个搬到了 core1 —— 它连 core0 的
     *   调度器都进不来, 更抢不走 1ms 控制任务的时间。
     *
     *   注意: 电流环仍然在 ADC 中断里(优先级 1, 高于 configMAX_SYSCALL_INTERRUPT_PRIORITY),
     *   RTOS 的临界区关中断也不会挡住它 —— 见 FreeRTOSConfig.h 的说明。 */
    app_rtos_start();

    return 0;   /* 不会到这里: app_rtos_start 不返回 */
}
