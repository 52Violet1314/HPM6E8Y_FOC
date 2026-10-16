/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * YeeZi HPM6EMotor V2.0 (HPM6E8Y) 程序入口
 *
 * 分层架构:
 *   main  -> 只做 board_init + app_init + 主循环外壳
 *   app   -> 采集(BSP) + 控制(FOC) + 上报 的编排 (src/app)
 *   FOC   -> SVPWM / 开环 / 电流环 控制算法 (src/FOC)
 *   BSP   -> PWM/ADC/USB/SEI 硬件驱动, 中断留在 BSP, 经注册回调上抛 (src/bsp)
 *   Hardware -> 传感器协议层(纯解析, 无硬件) (src/Hardware/mt6701)
 */

#include "board.h"

#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"

#include "app.h"
#include "app_diag.h"       /* diag_set_stage: 给中断诊断标记"主循环在干什么" */
#include "app_log.h"
#include "app_rtos.h"
#include "lcd_app.h"

int main(void)
{
    /* 板级初始化: 时钟树、引脚复用等, 必须最先调用 */

    board_init();

    /* 应用初始化: 外设 + FOC + 回调注册 */
    app_init();

    /* LCD/LVGL: ILI9341 + XPT2046 board example port */
    lcd_app_hardware_init();
    lcd_app_ui_init();

    /* 交给 RTOS: 控制任务(最高优先级, 硬 1ms) + 日志任务 + 显示任务。
     *
     * 【为什么换掉原来的 while(1) 补周期写法】
     *   原写法用 mcycle 把周期精确补到 1ms —— 那解决了"周期漂移"的一部分问题,
     *   但 app_run 里只要遇上 LCD 刷屏/USB 提交, 整拍就会超过 1ms(实测 0.96~1.2ms
     *   来回变): 编码器采样率、遥测时序、限速判断都跟着漂。
     *   现在控制任务用 vTaskDelayUntil 锁 1ms 硬周期, 显示/日志降为低优先级任务,
     *   抢不走控制任务的时间; 日志的格式化与 USB 提交也不再占用控制任务。
     *
     *   注意: 电流环仍然在 ADC 中断里(优先级 1, 高于 configMAX_SYSCALL_INTERRUPT_PRIORITY),
     *   RTOS 的临界区关中断也不会挡住它 —— 见 FreeRTOSConfig.h 的说明。 */
    app_rtos_start();

    return 0;   /* 不会到这里: app_rtos_start 不返回 */
}
