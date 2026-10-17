/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: J11 排针上的按键输入 (按键调速/位置点动)
 *
 * 【为什么不用 SDK 的 init_butn_pins()】那个函数配的是 PB24/PB25, 是 SDK 板级
 * 例程预留的按键脚; 本板按键没焊、也不在 J11 上, 而且 PB24/PB25 在这里根本没被
 * board_init() 调过。本工程按"用户自己往 J11 排针上接按键"来做, 所以脚位在下面
 * 用宏定义, 换脚只改这三行。
 *
 * 【接线】每个按键: 一端接引脚, 另一端接 GND。内部上拉已开, 所以
 *   松开 = 高电平, 按下 = 低电平 (低有效)。不需要外部电阻。
 *
 * 【为什么在 GPIO 上做而不是用中断】按键要消抖 + 要支持"长按连发", 这些逻辑在
 * 1kHz 控制任务里做最省事: 不需要额外中断, 也不会在 ISR 里碰任何共享状态。
 * 采样周期 1ms, 消抖窗 20ms —— 机械抖动(典型 5~10ms)完全覆盖, 而且按住到
 * 第一次上报只慢 20ms, 手感无差别。
 */
#ifndef BSP_BUTTON_H
#define BSP_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 脚位定义 (J11 排针) =================
 *
 * 这三个脚在 pinmux.c 里【只出现在注释/未被调用的 init_femc_pins()】中
 * (FEMC_CS_1 / FEMC_A_11 / FEMC_DQ_10), 而本工程 board_init() 并没有调
 * init_femc_pins(), 也就是说它们没有任何外设占用, 可以安全当 GPIO 用。
 *
 *   BTN_UP   : PD8   J11 脚 42
 *   BTN_DOWN : PC25  J11 脚 27
 *   BTN_HOME : PD16  J11 脚 12
 *
 * ================= 为什么不用板上的 KEYA/KEYB/KEYC =================
 *
 * 原理图(page 8)上有三个实体按键 SW4/SW5/SW6, 分别接:
 *     SW4(Key1) -> KEYA -> **PC14**
 *     SW5(Key2) -> KEYB -> **PC15**
 *     SW6(Key3) -> KEYC -> **PC19**
 * 每个都是"10K 上拉到 +3V3 + 100nF 对地 + 按键到 GND"的标准低有效接法。
 *
 * 但【这三个脚都不在 J11 排针上】, 而本工程现在唯一可靠的外接点就是 J11:
 *   - 要用板上实体按键, 就必须飞线/改接到这三个脚, 并且要重新确认这些脚在
 *     pinmux.c 里没有被别的被调用函数占用 (它们只出现在 FEMC/PPI 的配置函数里,
 *     而本工程 board_init() 没调 init_femc_pins());
 *   - 所以默认走 J11 外接按键 —— 换一个不坏的实体键、或者直接外接微动开关,
 *     接到下面三个 J11 脚即可, 不需要改代码。
 *
 * 【改脚位时注意】别选 J11 上已被占用或已被配置成模拟输入的脚:
 *   编码器 = PC26/PC28/PC24 (SPI2), LCD = PC10..PC13 + PC03/PC04/PC08,
 *   触摸 = PC20..PC23 + PC09, 电机 PWM = PD00..PD05, 电流/母线采样 = 其它 ADC 脚。 */
typedef enum {
    BSP_BTN_UP = 0,     /* 位置 + (或速度 +) */
    BSP_BTN_DOWN,       /* 位置 - (或速度 -) */
    BSP_BTN_HOME,       /* 回起点 / 停止 */
    BSP_BTN_COUNT
} bsp_btn_id_t;

/* 按键事件 (由 bsp_button_poll 每拍返回) */
#define BSP_BTN_EV_NONE     (0U)
#define BSP_BTN_EV_DOWN     (1U)    /* 刚按下 (消抖后) */
#define BSP_BTN_EV_REPEAT   (2U)    /* 长按连发 */
#define BSP_BTN_EV_UP       (3U)    /* 松开 */

/** @brief 初始化引脚 (内部上拉 + 施密特) 与消抖状态; 必须在 board_init() 之后调 */
void bsp_button_init(void);

/**
 * @brief  采样一次并返回【刚发生的事件】(按控制周期调用, 建议 1kHz)
 * @param  which  哪一个按键 (bsp_btn_id_t)
 * @return BSP_BTN_EV_* (顺序调用 up/down/home 各一次即可)
 * @note   事件在上升/下降沿各返回一次; 长按超过 BSP_BTN_LONG_MS 后每
 *         BSP_BTN_REPEAT_MS 返回一次 REPEAT。
 */
uint8_t bsp_button_poll(bsp_btn_id_t which);

/**
 * @brief  一次性取三个按键的事件位掩码 (省得调三次)
 * @return bit0/1/2 = UP/DOWN/HOME 的事件值 (每 2 位一个事件, 见实现)
 */
uint32_t bsp_button_poll_all(void);

/** @return 当前电平 (消抖后): 1 = 按下 */
uint8_t bsp_button_is_down(bsp_btn_id_t which);

/** @brief 取某个按键的原始电平 (不消抖), 接线排查用: 松开应为 1 */
uint8_t bsp_button_raw_level(bsp_btn_id_t which);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUTTON_H */
