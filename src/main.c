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

#include "app.h"

int main(void)
{
    /* 板级初始化: 时钟树、引脚复用等, 必须最先调用 */
    board_init();

    /* 应用初始化: 外设 + FOC + 回调注册 */
    app_init();

    /* 主循环: 采集 -> 控制 -> 上报 */
    while (1) {
        app_run();
        board_delay_ms(APP_MAIN_LOOP_MS);
    }

    return 0;
}
