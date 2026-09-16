/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * YeeZi HPM6EMotor V2.0 (HPM6E8Y) BSP 示例入口
 *   - 三相互补 PWM 输出 (50%)
 *   - 相电流 / 母线电压 / 温度 ADC 采集
 *   - USB0 虚拟串口 CDC ACM, 周期性上报状态
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "hpm_debug_console.h"

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_usb.h"

/**
 * @brief  程序入口: 外设初始化 + 周期性上报运行状态
 * @return 不会返回(死循环)
 *
 * 【执行流程 / 对照 STM32 的 main()】
 *   1. board_init()          板级初始化(时钟、引脚, 相当于 HAL_Init+SystemClock_Config+MspInit)
 *   2. bsp_pwm_init()        三相互补 PWM(相当于 TIM1 初始化)
 *   3. bsp_adc_init()        电流/母线/温度 ADC(相当于 ADC+DMA 初始化)
 *   4. bsp_usb_init()        USB CDC 虚拟串口(相当于 USB Device 初始化)
 *   5. 主循环                 采集 -> 组包 -> 串口/USB 上报
 *
 * 【注意】
 *   示例直接给了 50% 占空比并输出, 三相电压矢量叠加后电机不会转,
 *   但上电瞬间会有冲击。接真实电机调试前建议先空载/低压, 或先关闭输出。
 */
int main(void)
{
    /* 板级初始化: 时钟树、引脚复用等, 必须最先调用 */
    board_init();

    printf("\nYeeZi HPM6EMotor BSP demo\n");

    /* 外设初始化顺序: PWM -> ADC -> USB.
     * PWM 先初始化很重要 —— 因为 ADC 的触发源来自 PWM TRGO */
    bsp_pwm_init();
    bsp_adc_init();
    bsp_usb_init();

    /* 50% 占空比, 打开三相输出 (注意: 接电机前先悬空/降低母线电压) */
    bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
    bsp_pwm_enable_output(true);

    printf("PWM reload=%u, freq=%u Hz\n",
           (unsigned)bsp_pwm_get_reload(), (unsigned)BSP_PWM_FREQUENCY_HZ);

    /* 主循环: 每 500ms 采样一次并上报 */
    while (1) {
        int32_t iu_ma = 0, iv_ma = 0, iw_ma = 0;
        char msg[160];
        int n;

        /* 读取三相电流(mA)。真实 FOC 应在 ADC 中断回调里做这件事,
         * 这里主循环读取只是为了演示 */
        bsp_adc_get_currents_ma(&iu_ma, &iv_ma, &iw_ma);

        /* 组包: 母线电压 / 温度 / 三相电流 */
        n = snprintf(msg, sizeof(msg),
                     "VBUS=%u mV, TEMP=%ld.%03ld C, I=%ld/%ld/%ld mA\r\n",
                     (unsigned)bsp_adc_get_vbus_mv(),
                     (long)(bsp_adc_get_temperature_mc() / 1000),      /* 整数部分 (℃) */
                     (long)(bsp_adc_get_temperature_mc() % 1000),      /* 小数部分 (0.001℃) */
                     (long)iu_ma, (long)iv_ma, (long)iw_ma);

        /* 串口调试输出(板载 debug console, 相当于 STM32 的 ITM/UART printf 重定向) */
        printf("%s", msg);

        /* USB 虚拟串口输出(主机打开串口后可见) */
        if (bsp_usb_connected()) {
            (void)bsp_usb_write((const uint8_t *)msg, (uint32_t)n);
        }

        board_delay_ms(500);
    }

    return 0;
}
