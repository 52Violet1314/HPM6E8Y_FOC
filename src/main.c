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

int main(void)
{
    /* 板级初始化: 时钟树、引脚复用等, 必须最先调用 */

    board_init();

    /* 应用初始化: 外设 + FOC + 回调注册 */
    app_init();

    /* 主循环: 用 mcycle 做【精确】周期节拍。
     *
     * 【为什么不用 board_delay_ms】
     *   原写法是 `app_run(); board_delay_ms(1);` —— 这是"固定延时", 不是
     *   "固定周期"。app_run() 本身要花时间, 于是真实周期 = app_run 耗时 + 1ms。
     *   实测约 1.3ms, 也就是 FOC 内部按 dt=1ms 积分、真实时间却是 1.3ms,
     *   结果: 指令的 12Hz 电频率实际只有约 9Hz, 转速偏低约 25%。
     *   改成"等到下一个周期起点"后, 周期严格 = 1ms, 且 app_run 超时也能自恢复。 */
    {
        uint32_t period_cycles = hpm_core_clock / (1000U / APP_MAIN_LOOP_MS);

        if (period_cycles == 0U) {
            period_cycles = 1U;
        }

        while (1) {
            uint64_t t0 = hpm_csr_get_core_mcycle();

            app_run();

            /* 补足到本周期结束 (app_run 超时就不补, 自然降速) */
            while ((hpm_csr_get_core_mcycle() - t0) < (uint64_t)period_cycles) {
                /* busy wait */
            }
        }
    }

    return 0;
}
