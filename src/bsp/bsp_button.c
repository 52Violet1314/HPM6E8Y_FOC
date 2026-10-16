/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: J11 按键输入实现 (消抖 + 长按连发)
 */
#include "board.h"
#include "hpm_gpio_drv.h"
/* IOC 的 PAD/PAD_CTL 宏在 hpm_ioc_regs.h 里 (经 board.h -> hpm_soc.h 间接包含);
 * 这里不写 #include "hpm_ioc_drv.h" —— 本 SDK 里没有那个头文件。 */
#include "hpm_soc.h"

#include "app.h"            /* APP_MAIN_LOOP_MS: 采样周期 (ms) */
#include "bsp_button.h"

/* ---- 脚位 (改脚只改这三组) ----
 * 注意 HPM 的 GPIO 驱动签名是 gpio_read_pin(base, port_index, pin):
 *   port_index 用 GPIO_DI_GPIOx (见 hpm_gpio_regs.h), pin 是组内位号。 */
#define BSP_BTN_GPIO        HPM_GPIO0
#define BSP_BTN_PORT_UP     GPIO_DI_GPIOD   /* PD8  */
#define BSP_BTN_PORT_DOWN   GPIO_DI_GPIOC   /* PC25 */
#define BSP_BTN_PORT_HOME   GPIO_DI_GPIOD   /* PD16 */

#define BSP_BTN_PIN_UP      (8U)
#define BSP_BTN_PIN_DOWN    (25U)
#define BSP_BTN_PIN_HOME    (16U)

#define BSP_BTN_PAD_UP      IOC_PAD_PD08
#define BSP_BTN_PAD_DOWN    IOC_PAD_PC25
#define BSP_BTN_PAD_HOME    IOC_PAD_PD16

#define BSP_BTN_FUNC_UP     IOC_PD08_FUNC_CTL_GPIO_D_08
#define BSP_BTN_FUNC_DOWN   IOC_PC25_FUNC_CTL_GPIO_C_25
#define BSP_BTN_FUNC_HOME   IOC_PD16_FUNC_CTL_GPIO_D_16

/* ---- 消抖 / 连发参数 (单位: 控制周期数, = ms) ---- */
#define BSP_BTN_DEBOUNCE_MS (20U)   /* 连续稳定的时间才算一次真实变化 */
#define BSP_BTN_LONG_MS     (600U)  /* 按住多久开始连发 */
#define BSP_BTN_REPEAT_MS   (150U)  /* 连发间隔 */

typedef struct {
    uint32_t port;              /* GPIO_DI_GPIOx */
    uint8_t  pin;               /* 组内位号 */
    uint8_t  stable;            /* 消抖后的状态: 1 = 按下 */
    uint8_t  last_raw;          /* 上一次原始采样 */
    uint16_t cnt;               /* 当前原始状态已经持续的拍数 */
    uint16_t hold_ms;           /* 按下持续的时间 (ms) */
    uint16_t repeat_ms;         /* 距上一次连发的时间 (ms) */
} btn_state_t;

static btn_state_t s_btn[BSP_BTN_COUNT];

static void btn_pad_init(uint32_t pad, uint32_t func)
{
    /* 上拉 + 施密特: 按键一端接引脚、另一端接 GND, 松开时必须被稳稳拉到高电平。
     * 不加上拉的话浮空脚会被旁边的 PWM/SPI 噪声耦合出假按键。 */
    HPM_IOC->PAD[pad].FUNC_CTL = func;
    HPM_IOC->PAD[pad].PAD_CTL  = IOC_PAD_PAD_CTL_PE_SET(1)   /* 上拉使能 */
                               | IOC_PAD_PAD_CTL_PS_SET(1)   /* 上拉(非下拉) */
                               | IOC_PAD_PAD_CTL_HYS_SET(1); /* 施密特 */
}

void bsp_button_init(void)
{
    btn_pad_init(BSP_BTN_PAD_UP,   BSP_BTN_FUNC_UP);
    btn_pad_init(BSP_BTN_PAD_DOWN, BSP_BTN_FUNC_DOWN);
    btn_pad_init(BSP_BTN_PAD_HOME, BSP_BTN_FUNC_HOME);

    s_btn[BSP_BTN_UP].port   = BSP_BTN_PORT_UP;
    s_btn[BSP_BTN_UP].pin    = BSP_BTN_PIN_UP;
    s_btn[BSP_BTN_DOWN].port = BSP_BTN_PORT_DOWN;
    s_btn[BSP_BTN_DOWN].pin  = BSP_BTN_PIN_DOWN;
    s_btn[BSP_BTN_HOME].port = BSP_BTN_PORT_HOME;
    s_btn[BSP_BTN_HOME].pin  = BSP_BTN_PIN_HOME;

    for (uint32_t i = 0U; i < (uint32_t)BSP_BTN_COUNT; i++) {
        (void)gpio_set_pin_input(BSP_BTN_GPIO, s_btn[i].port, s_btn[i].pin);
        s_btn[i].stable    = 0U;
        s_btn[i].last_raw  = 0U;
        s_btn[i].cnt       = 0U;
        s_btn[i].hold_ms   = 0U;
        s_btn[i].repeat_ms = 0U;
    }
}

uint8_t bsp_button_raw_level(bsp_btn_id_t which)
{
    /* 低有效: 按下读到 0 -> 返回 1 */
    return (gpio_read_pin(BSP_BTN_GPIO, s_btn[which].port, s_btn[which].pin) == 0U)
           ? 1U : 0U;
}

uint8_t bsp_button_is_down(bsp_btn_id_t which)
{
    return s_btn[which].stable;
}

static uint8_t btn_update(btn_state_t *b)
{
    uint8_t raw = (gpio_read_pin(BSP_BTN_GPIO, b->port, b->pin) == 0U) ? 1U : 0U;   /* 低有效 */
    uint8_t ev  = BSP_BTN_EV_NONE;

    if (raw != b->last_raw) {
        b->last_raw = raw;
        b->cnt      = 0U;               /* 电平变了, 消抖计数重新开始 */
    } else if (b->cnt < 0xFFFFU) {
        b->cnt++;
    }

    if ((b->cnt >= BSP_BTN_DEBOUNCE_MS) && (b->stable != raw)) {
        b->stable = raw;                /* 认定一次真实的状态变化 */
        if (raw != 0U) {
            ev          = BSP_BTN_EV_DOWN;
            b->hold_ms  = 0U;
            b->repeat_ms = 0U;
        } else {
            ev = BSP_BTN_EV_UP;
        }
        b->cnt = BSP_BTN_DEBOUNCE_MS;   /* 防止再次触发 */
    }

    /* 长按连发: 按住超过 BSP_BTN_LONG_MS 之后, 每 BSP_BTN_REPEAT_MS 再报一次 */
    if (b->stable != 0U) {
        b->hold_ms += APP_MAIN_LOOP_MS;
        if (b->hold_ms >= BSP_BTN_LONG_MS) {
            b->repeat_ms += APP_MAIN_LOOP_MS;
            if (b->repeat_ms >= BSP_BTN_REPEAT_MS) {
                b->repeat_ms = 0U;
                ev = BSP_BTN_EV_REPEAT;
            }
        }
    } else {
        b->hold_ms = 0U;
    }

    return ev;
}

uint8_t bsp_button_poll(bsp_btn_id_t which)
{
    if ((uint32_t)which >= (uint32_t)BSP_BTN_COUNT) {
        return BSP_BTN_EV_NONE;
    }
    return btn_update(&s_btn[which]);
}

uint32_t bsp_button_poll_all(void)
{
    uint32_t mask = 0U;

    for (uint32_t i = 0U; i < (uint32_t)BSP_BTN_COUNT; i++) {
        mask |= ((uint32_t)btn_update(&s_btn[i]) << (i * 4U));
    }
    return mask;
}
