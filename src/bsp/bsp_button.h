/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 按键输入 (板上实体键 Key2/Key3 + J11 外接键)
 *
 * 【本板按键的真实接线 —— 已用 PDF 渲染成图逐条核对】
 *   原理图 page 8 上三个实体键 SW4/SW5/SW6, 每个都是
 *       +3V3 -- 10K 上拉 --+-- KEYx --→ MCU 引脚
 *                          |
 *                       100nF 对地
 *       按键一端接 KEYx, 另一端接 GND            (低有效)
 *       SW4 (Key1) -> KEYA -> PC14   (本板已坏)
 *       SW5 (Key2) -> KEYB -> PC15
 *       SW6 (Key3) -> KEYC -> PC19
 *
 * 【用哪几个】Key1/PC14 已坏, 所以:
 *     UP   = 板上 Key2 (PC15)  -> 位置 +
 *     DOWN = 板上 Key3 (PC19)  -> 位置 -
 *     HOME = J11 PD8 (脚 42)   -> 回起点 (低频操作, 接一个外接微动开关)
 * 板上两个好键留给最常用的调速, 不需要飞线; HOME 不想接也不影响另两个键。
 *
 * 【PC15/PC19 为什么能当 GPIO】核查过本工程【真正会执行到】的初始化路径:
 *   board_init() 只做: clock / console(UART0 PA00/PA01) / pmp / encoder_init(),
 *   而 encoder_init() 配的是 PD6/PD7/PD13。两个脚在 pinmux.c 里只出现在
 *   init_femc_pins() / init_ppi_pins() 里, 而 board_init() 并没有调用它们
 *   (board_init_sdram_pins() 才调, 本工程没调)。ADC 用的是 PF07/PF10/PF20/
 *   PF27/PF28/PF29/PF30, PWM0 是 PD00~PD05, 都不与它们相交。
 *   【但要注意】这两个脚不在 J11 排针上, 想外接就得找到板上 KEY2/KEY3 焊盘;
 *   这也是为什么 HOME 仍留在 J11。
 *
 * 【上拉】板上已有 10K 外部上拉, PC15/PC19 不再开内部上拉; J11 的 PD8 没有
 *   外部上拉, 必须开内部上拉 —— 每个按键单独配置 (见 bsp_button.c 的 s_cfg 表)。
 *
 * 【为什么在 GPIO 上做而不是用中断】按键要消抖 + 要支持"长按连发", 这些逻辑在
 * 1kHz 控制任务里做最省事: 不需要额外中断, 也不会在 ISR 里碰任何共享状态。
 * 采样周期 1ms, 消抖窗 20ms, 加上板上 100nF 的硬件滤波, 手感稳定。
 */
#ifndef BSP_BUTTON_H
#define BSP_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 按键映射 (脚位/功能在 bsp_button.c 的 s_cfg 表里) =================
 *
 *   BTN_UP   : 板上 Key2 / PC15   -> 位置 +
 *   BTN_DOWN : 板上 Key3 / PC19   -> 位置 -
 *   BTN_HOME : J11 PD8 (脚 42)    -> 回起点(重新定零), 外接键, 可不接
 *
 * 【改脚位/改功能】只改 bsp_button.c 顶部那张 s_cfg[] 表, 一项一行:
 *   端口 / 位号 / PAD / FUNC / 是否开内部上拉 / 日志名
 *
 * 【改脚位时注意】别选已被占用或已被配置成模拟输入的脚:
 *   编码器 = PC26/PC28/PC24 (SPI2), LCD = PC10..PC13 + PC03/PC04/PC08,
 *   触摸 = PC20..PC23 + PC09, 电机 PWM = PD00..PD05,
 *   电流/母线采样 = PF07/PF10/PF20/PF27/PF28/PF29/PF30,
 *   编码器方向检测 = PD6/PD7/PD13, 控制台 = PA00/PA01。 */
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

/** @brief 取某个按键的原始电平 (不消抖), 接线排查用: 松开应为 0 */
uint8_t bsp_button_raw_level(bsp_btn_id_t which);

/** @return 该按键的日志名 (例如 "Key2/PC15"), 只用于打印 */
const char *bsp_button_name(bsp_btn_id_t which);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUTTON_H */
