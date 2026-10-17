/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 按键输入实现 (板上实体键 + J11 外接键, 消抖 + 长按连发)
 *
 * 【本板实际接线 —— 已按原理图核对, 不是猜的】
 *   原理图 page 8 上三个实体键, 每个都是
 *       +3V3 -- 10K 上拉 --+-- KEYx --→ MCU 引脚
 *                          |
 *                       100nF 对地
 *       按键 SWx 一端接 KEYx, 另一端接 GND          (低有效)
 *   对应关系(用 PDF 渲染成图逐条核对的):
 *       SW4 (Key1) -> KEYA -> PC14   R165 10K / C136 100nF
 *       SW5 (Key2) -> KEYB -> PC15   R170 10K / C137 100nF
 *       SW6 (Key3) -> KEYC -> PC19   R115 10K / C148 100nF
 *
 * 【为什么用 PC15/PC19 而不是 J11 外接】
 *   Key1(SW4/PC14) 在本板已损坏; Key2/Key3 完好, 而且板上已有 10K 上拉 +
 *   100nF 硬件滤波, 比外接更省事。这两个脚在本工程里没有被任何外设占用
 *   (见 bsp_button.h 的核查清单), 可以安全当 GPIO 输入。
 *
 * 【上拉怎么配】板上已经有 10K 外部上拉, 所以这里【不再开内部上拉】:
 *   内部上拉和外部上拉并联只会让"松开"更硬一点, 没坏处, 但按下时
 *   按键要把两者一起拉低, 没必要多花电流。J11 外接按键(没有外部上拉)
 *   则必须开内部上拉 —— 所以每个按键单独配。
 */
#include "board.h"
#include "hpm_gpio_drv.h"
/* IOC 的 PAD/PAD_CTL 宏在 hpm_ioc_regs.h 里 (经 board.h -> hpm_soc.h 间接包含);
 * 这里不写 #include "hpm_ioc_drv.h" —— 本 SDK 里没有那个头文件。 */
#include "hpm_soc.h"

#include "app.h"            /* APP_MAIN_LOOP_MS: 采样周期 (ms) */
#include "bsp_button.h"

#define BSP_BTN_GPIO        HPM_GPIO0

/* ---- 消抖 / 连发参数 (单位: 控制周期数, = ms) ---- */
#define BSP_BTN_DEBOUNCE_MS (20U)   /* 连续稳定的时间才算一次真实变化 */
#define BSP_BTN_LONG_MS     (600U)  /* 按住多久开始连发 */
#define BSP_BTN_REPEAT_MS   (150U)  /* 连发间隔 */

/* 一个按键的静态配置 */
typedef struct {
    uint32_t port;          /* GPIO_DI_GPIOx */
    uint8_t  pin;           /* 组内位号 */
    uint32_t pad;           /* IOC_PAD_xxx */
    uint32_t func;          /* IOC_Pxx_FUNC_CTL_GPIO_x_xx */
    uint8_t  pull_up;       /* 1 = 开内部上拉(外接键); 0 = 板上已有外部上拉 */
    const char *name;       /* 日志/自检用 */
} btn_cfg_t;

/* ---- 按键映射: 改脚位/改功能只改这张表 ----
 *
 *   UP   = 板上 Key2 (SW5 / KEYB / PC15)  -> 位置 + (每步 5°, 长按连发)
 *   DOWN = 板上 Key3 (SW6 / KEYC / PC19)  -> 位置 -
 *   HOME = J11 PD8 (脚 42, 外接)          -> 回起点(重新定零)
 *
 * 【pull_up 全部填 1 —— 这是踩过坑的】
 *   原理图上 PC15/PC19 有 10K 外部上拉, 所以第一版这里填 0(不开内部上拉)。
 *   但实测上电后电机会自己跑: 位置目标被改成了 0, 而转子实际停在非零角度。
 *   原因就是这两个脚【浮空】—— 板子上的外部上拉不一定焊了(Key1 已经坏了,
 *   说明这几个按键的硬件本身未必齐全), 浮空脚被噪声/漏电拉低后
 *   bsp_button_raw_level() 读成"按下" -> 触发 HOME -> 目标归零 -> 位置环
 *   看到几弧度的误差 -> 满力去追。
 *   开内部上拉是安全的: 有外部 10K 时两者并联, 只是"松开"更硬一点;
 *   没有外部上拉时它就是唯一的上拉。所以【这里一律开】。 */
static const btn_cfg_t s_cfg[BSP_BTN_COUNT] = {
    /* port                          pin                    pad                func                              pull name   */
    { GPIO_DI_GPIOC,                 15U, IOC_PAD_PC15, IOC_PC15_FUNC_CTL_GPIO_C_15, 1U, "Key2/PC15" },
    { GPIO_DI_GPIOC,                 19U, IOC_PAD_PC19, IOC_PC19_FUNC_CTL_GPIO_C_19, 1U, "Key3/PC19" },
    { GPIO_DI_GPIOD,                  8U, IOC_PAD_PD08, IOC_PD08_FUNC_CTL_GPIO_D_08, 1U, "J11/PD8"   },
};

typedef struct {
    uint8_t  stable;            /* 消抖后的状态: 1 = 按下 */
    uint8_t  last_raw;          /* 上一次原始采样 */
    uint16_t cnt;               /* 当前原始状态已经持续的拍数 */
    uint16_t hold_ms;           /* 按下持续的时间 (ms) */
    uint16_t repeat_ms;         /* 距上一次连发的时间 (ms) */
} btn_state_t;

static btn_state_t s_btn[BSP_BTN_COUNT];

void bsp_button_init(void)
{
    for (uint32_t i = 0U; i < (uint32_t)BSP_BTN_COUNT; i++) {
        /* 复用成 GPIO + 施密特(抗噪); 上拉按表决定 */
        HPM_IOC->PAD[s_cfg[i].pad].FUNC_CTL = s_cfg[i].func;
        HPM_IOC->PAD[s_cfg[i].pad].PAD_CTL  = IOC_PAD_PAD_CTL_HYS_SET(1)
                                            | ((s_cfg[i].pull_up != 0U)
                                               ? (IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1))
                                               : 0U);

        (void)gpio_set_pin_input(BSP_BTN_GPIO, s_cfg[i].port, s_cfg[i].pin);
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
    return (gpio_read_pin(BSP_BTN_GPIO, s_cfg[which].port, s_cfg[which].pin) == 0U)
           ? 1U : 0U;
}

uint8_t bsp_button_is_down(bsp_btn_id_t which)
{
    return s_btn[which].stable;
}

const char *bsp_button_name(bsp_btn_id_t which)
{
    return s_cfg[which].name;
}

static uint8_t btn_update(btn_state_t *b, const btn_cfg_t *c)
{
    uint8_t raw = (gpio_read_pin(BSP_BTN_GPIO, c->port, c->pin) == 0U) ? 1U : 0U;   /* 低有效 */
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
            ev           = BSP_BTN_EV_DOWN;
            b->hold_ms   = 0U;
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
    return btn_update(&s_btn[which], &s_cfg[which]);
}

uint32_t bsp_button_poll_all(void)
{
    uint32_t mask = 0U;

    for (uint32_t i = 0U; i < (uint32_t)BSP_BTN_COUNT; i++) {
        mask |= ((uint32_t)btn_update(&s_btn[i], &s_cfg[i]) << (i * 4U));
    }
    return mask;
}
