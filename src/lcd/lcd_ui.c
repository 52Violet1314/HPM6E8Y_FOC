/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * LCD: LVGL 电机界面实现 (转速显示 + 速度条设定目标转速)
 *
 * 布局与数据流见 lcd_ui.h。
 *
 * 【RTOS 下这是谁在跑】上 RTOS 之后本模块只由【显示任务】调用
 * (app_rtos.c 的 app_disp_task -> lcd_app_task_handler -> lcd_ui_refresh),
 * 约 200Hz。滑条回调也在这个任务里, 所以这里碰 LVGL 不需要自己加锁 ——
 * 锁由调用者 (lcd_app_task_handler) 持有, 见 lcd_app.h 的说明。
 * 从本文件【只能】调 app 的跨任务安全接口 (app_speed_*), 绝不能直接读
 * s_ctrl / s_encoder 这类控制任务的内部状态。
 *
 * 【为什么刷新要节流】lv_label_set_text_fmt() 会重建文本并触发重绘。显示任务
 * 5ms 来一次, 全量刷新会白白吃掉 SPI 刷屏带宽 (刷屏很慢, 是显示任务里最大头
 * 的开销)。所以:
 *   - 外层 20ms 节流 (转速本来就是低通估算值, 50Hz 显示足够);
 *   - 文本内容没变就不调 set_text (LVGL 内部对相同字符串其实有短路, 但显式跳过
 *     更省事, 也避免每拍走一遍 snprintf)。
 */
#include <string.h>
#include <stdio.h>

#include "lvgl.h"

#include "lcd_ui.h"
#include "app.h"            /* app_speed_get_* / APP_FOC_CLOSED_LOOP_ENABLE */

/* 颜色 */
#define UI_BG            0x0E1621
#define UI_PANEL         0x16212E
#define UI_TRACK         0x22303F
#define UI_ACCENT        0x35C8F0
#define UI_TEXT          0xE8F1F8
#define UI_DIM           0x8FA3B4
#define UI_WARN          0xFFB300
#define UI_BAD           0xFF5252
#define UI_OK            0x5BE37D

/* 刷新节流 (ms) */
#define UI_REFRESH_MS    20U

typedef struct {
    lv_obj_t *lbl_rpm;          /* 大字: 实测转速数值 */
    lv_obj_t *lbl_unit;         /* "RPM" 单位 */
    lv_obj_t *lbl_target;       /* "目标 300 RPM" */
    lv_obj_t *slider;           /* 速度条: 目标转速 */
    lv_obj_t *lbl_fb;           /* 反馈行: 电机状态 / 使用说明 */
    lv_obj_t *lbl_readback;     /* 实际下发值 (只在"未接管"时提示) */

    int32_t   target_rpm;       /* 滑条设定值 (RPM, 无符号) */
    uint32_t  tick_ms;          /* 节流累计 */
    uint32_t  last_rpm_text;    /* 上次显示的数值 (去重: 0xFFFFFFFF = 还没显示过) */

    /* 【文本缓存: 必须的, 不是优化】
     * lv_label_set_text_fmt() 进函数第一件事就是 lv_obj_invalidate(obj), 且它会
     * 释放旧字符串再重新格式化 —— 也就是说【内容没变也照样标脏 + 走一遍
     * vsnprintf + 触发一轮重绘】。上 RTOS 之后重绘意味着显示任务吃掉几毫秒 CPU
     * (SPI 是轮询写的), 那是会直接影响电机控制节拍的。所以每次写文本前先跟自己
     * 缓存的上一次内容比一下, 一样就整个跳过。 */
    char      fb[40];
    char      readback[40];
    char      target[20];
    char      rpm[12];
} ui_t;

static ui_t s_ui;

/* ---- 文本写入: 内容没变就什么都不做 ---- */
static void ui_text(lv_obj_t *lbl, char *cache, uint32_t cache_sz, const char *txt)
{
    if (strcmp(cache, txt) == 0) {
        return;
    }
    strncpy(cache, txt, cache_sz - 1U);
    cache[cache_sz - 1U] = '\0';
    lv_label_set_text(lbl, txt);
}

static void ui_text_fmt(lv_obj_t *lbl, char *cache, uint32_t cache_sz,
                        const char *fmt, int32_t v1, int32_t v2)
{
    char tmp[40];

    if (v2 >= 0) {
        (void)snprintf(tmp, sizeof(tmp), fmt, (int)v1, (int)v2);
    } else {
        (void)snprintf(tmp, sizeof(tmp), fmt, (int)v1);
    }
    ui_text(lbl, cache, cache_sz, tmp);
}

/* 滑条回调 (跑在显示任务里): 把新值交给 app。
 *
 * 【为什么只调 app_speed_target_set() 就够了】那个函数只写一个 32 位对齐的
 * volatile float, 是"跨任务安全"的 (见 app.h 的跨任务约定)。真正的下发
 * (写控制器/斜坡/软限速) 由控制任务在 app_speed_apply() 里做。
 *
 * 【千万不要在这里写硬件或控制器】LVGL 回调跑在显示任务上下文, 直接改 s_ctrl
 * 会和控制任务的 1ms 计算打架; 而且拖手指时回调一次能来好几遍。 */
static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t   v      = lv_slider_get_value(slider);

    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        s_ui.target_rpm = v;
        app_speed_target_set((float)v);
    }
}

int32_t lcd_ui_get_target_rpm(void)
{
    return s_ui.target_rpm;
}

/* ---- 小工具: 建一个标签 ---- */
static lv_obj_t *ui_label(lv_obj_t *parent, const char *text, const lv_font_t *font,
                          uint32_t color)
{
    lv_obj_t *lbl = lv_label_create(parent);

    lv_label_set_text(lbl, text);
    if (font != NULL) {
        lv_obj_set_style_text_font(lbl, font, 0);
    }
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
    return lbl;
}

void lcd_ui_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *title;
    lv_obj_t *lbl_cap;
    lv_obj_t *lbl_min;
    lv_obj_t *lbl_max;
    int32_t   init_rpm;

    s_ui.tick_ms       = 0U;
    s_ui.last_rpm_text = 0xFFFFFFFFU;

    /* 上电时滑条位置 = 应用层当前的目标转速, 两边一上来就是一致的 */
    init_rpm = (int32_t)app_speed_get_target();
    if (init_rpm < LCD_UI_RPM_MIN) {
        init_rpm = LCD_UI_RPM_MIN;
    }
    if (init_rpm > LCD_UI_RPM_MAX) {
        init_rpm = LCD_UI_RPM_MAX;
    }
    s_ui.target_rpm = init_rpm;

    lv_obj_set_style_bg_color(screen, lv_color_hex(UI_BG), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(screen, 0, 0);

    /* ---- 标题 ----
     * 【全部界面文本必须是 ASCII】LVGL 只编译了 Montserrat, 它没有中文字形
     * (lv_conf.h 里 LV_FONT_SOURCE_HAN_SANS_SC_*_CJK 都是 0)。写中文进去在屏上
     * 就是空框/缺字 —— 这正是之前"转速那一块画面不规则"的一部分原因, 不是排版
     * 问题。所以要中文界面必须先把 CJK 字体编进去(阈值 LV_MEM_SIZE 也得跟着涨),
     * 在那之前这里只用英文+数字。 */
    title = ui_label(screen, "SPEED", &lv_font_montserrat_20, UI_TEXT);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 8);
    {
        lv_obj_t *en = ui_label(screen, "MOTOR", &lv_font_montserrat_12, UI_DIM);

        lv_obj_align(en, LV_ALIGN_TOP_RIGHT, -10, 13);
    }

    /* ---- 大字转速 ---- */
    s_ui.lbl_rpm = ui_label(screen, "--", &lv_font_montserrat_40, UI_ACCENT);
    lv_obj_align(s_ui.lbl_rpm, LV_ALIGN_TOP_LEFT, 12, 30);

    s_ui.lbl_unit = ui_label(screen, "RPM", &lv_font_montserrat_20, UI_DIM);
    lv_obj_align(s_ui.lbl_unit, LV_ALIGN_TOP_LEFT, 150, 50);

    /* ---- 速度条 (目标转速) ----
     * 【纵向排布的依据】滑条本体占 y 106..132, 但 knob 设成 34px 高,
     * 会从 y=102 突出到 y=140 —— 所以刻度 0/600 必须放到 144 以下,
     * 否则旋钮压到数字上(之前就是这个问题)。 */
    lbl_cap = ui_label(screen, "TARGET", &lv_font_montserrat_14, UI_TEXT);
    lv_obj_align(lbl_cap, LV_ALIGN_TOP_LEFT, 10, 86);

    s_ui.lbl_target = ui_label(screen, "--- RPM", &lv_font_montserrat_16, UI_ACCENT);
    lv_obj_align(s_ui.lbl_target, LV_ALIGN_TOP_RIGHT, -10, 84);

    lbl_min = ui_label(screen, "0", &lv_font_montserrat_12, UI_DIM);
    lv_obj_align(lbl_min, LV_ALIGN_TOP_LEFT, 10, 146);

    lbl_max = ui_label(screen, "600", &lv_font_montserrat_12, UI_DIM);
    lv_obj_align(lbl_max, LV_ALIGN_TOP_RIGHT, -10, 146);

    s_ui.slider = lv_slider_create(screen);
    lv_obj_set_size(s_ui.slider, 268, 26);
    lv_obj_align(s_ui.slider, LV_ALIGN_TOP_MID, 0, 106);
    lv_slider_set_range(s_ui.slider, LCD_UI_RPM_MIN, LCD_UI_RPM_MAX);
    lv_slider_set_value(s_ui.slider, init_rpm, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui.slider, lv_color_hex(UI_TRACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ui.slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ui.slider, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui.slider, lv_color_hex(UI_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui.slider, 8, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_ui.slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_ui.slider, 0, LV_PART_KNOB);
    /* 旋钮做得比滑轨高一点, 手指好按 (电阻屏 + 无校准也能拖得住) */
    lv_obj_set_style_width(s_ui.slider, 20, LV_PART_KNOB);
    lv_obj_set_style_height(s_ui.slider, 34, LV_PART_KNOB);
    lv_obj_add_event_cb(s_ui.slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---- 反馈行 ---- */
    s_ui.lbl_fb = ui_label(screen, "INIT...", &lv_font_montserrat_14, UI_DIM);
    lv_obj_align(s_ui.lbl_fb, LV_ALIGN_TOP_LEFT, 10, 164);

    s_ui.lbl_readback = ui_label(screen, "", &lv_font_montserrat_12, UI_WARN);
    lv_obj_align(s_ui.lbl_readback, LV_ALIGN_TOP_LEFT, 10, 182);

    {
        lv_obj_t *hint = ui_label(screen,
            "SLIDE = SET TARGET\n"
            "APPLIED AT 1kHz",
            &lv_font_montserrat_12, UI_DIM);

        lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 10, -6);
    }
}

void lcd_ui_refresh(float rpm, float target, float target_eff,
                    uint8_t running, uint8_t busy, uint8_t pending)
{
    static uint32_t last_rpm_color = 0xFFFFFFFFU;   /* 0xFFFFFFFF = 还没设过颜色 */
    int32_t  rpm_i;
    int32_t  tgt_i;
    int32_t  eff_i;
    uint32_t color;
    uint8_t  dragging;

    if (s_ui.lbl_rpm == NULL) {
        return;                     /* 界面还没建好 */
    }

    s_ui.tick_ms += APP_MAIN_LOOP_MS;
    if (s_ui.tick_ms < UI_REFRESH_MS) {
        return;
    }
    s_ui.tick_ms = 0U;

    /* ---- 实测转速 ----
     *
     * 【超速/反转用颜色提示】正转青, 反转或标定中黄, 接近限幅红。 */
    rpm_i = (int32_t)((rpm >= 0.0f) ? (rpm + 0.5f) : (rpm - 0.5f));

    if ((busy != 0U) || (rpm_i < 0)) {
        color = UI_WARN;
    } else if ((float)rpm_i > APP_FOC_RPM_LIMIT * 0.9f) {
        color = UI_BAD;
    } else {
        color = UI_ACCENT;
    }

    /* 【为什么必须限幅再显示】大字是 montserrat_40, x 从 12 开始, 屏宽只有 320。
     * 40px 字体一个数字约 22~24px 宽, 所以**最多只能放 4 个字符**(约 96~110px),
     * 加上负号就是 "-999"。
     * 而实测转速在符号错/堵转时会冲到几百甚至上千 RPM —— "-1042" 或编码器异常
     * 时的 "-2147483" 会直接画到屏幕外面, 看起来就是"转速那块什么都没显示"。
     * 所以这里把显示值夹在 ±999。【注意】这只影响显示, 不影响控制。 */
    {
        int32_t shown = rpm_i;

        if (shown > 999) {
            shown = 999;
        } else if (shown < -999) {
            shown = -999;
        }

        /* 去重用的键要把"显示值"和"是否越界"都算进去, 否则 999/1000 看起来一样 */
        {
            uint32_t key = (uint32_t)shown ^ (((rpm_i != shown) ? 1U : 0U) << 20);

            if ((key != s_ui.last_rpm_text) || (color != last_rpm_color)) {
                char tmp[16];

                s_ui.last_rpm_text = key;
                (void)snprintf(tmp, sizeof(tmp), "%d", (int)shown);
                ui_text(s_ui.lbl_rpm, s_ui.rpm, sizeof(s_ui.rpm), tmp);

                if (color != last_rpm_color) {
                    last_rpm_color = color;
                    lv_obj_set_style_text_color(s_ui.lbl_rpm, lv_color_hex(color), 0);
                }
            }
        }
    }

    /* ---- 目标转速文本 ---- */
    tgt_i = s_ui.target_rpm;
    ui_text_fmt(s_ui.lbl_target, s_ui.target, sizeof(s_ui.target), "%d RPM", tgt_i, -1);

    /* ---- 滑条位置跟随 ---- */
    /* 只有"用户没在拖"且"滑条值与外部目标不一致"时才回写 —— 覆盖正在拖动的
     * 滑条会和手指打架 (手指往右、代码把它拉回来)。 */
    dragging = lv_obj_has_state(s_ui.slider, LV_STATE_PRESSED) ? 1U : 0U;
    if (dragging == 0U) {
        int32_t want = (int32_t)((target >= 0.0f) ? (target + 0.5f) : (target - 0.5f));

        if (want < LCD_UI_RPM_MIN) { want = LCD_UI_RPM_MIN; }
        if (want > LCD_UI_RPM_MAX) { want = LCD_UI_RPM_MAX; }
        if ((want != tgt_i) && (lv_slider_get_value(s_ui.slider) != want)) {
            lv_slider_set_value(s_ui.slider, want, LV_ANIM_OFF);
            s_ui.target_rpm = want;
            ui_text_fmt(s_ui.lbl_target, s_ui.target, sizeof(s_ui.target),
                        "%d RPM", want, -1);
        }
    }

    /* ---- 状态行 ---- */
    if (busy != 0U) {
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "CALIBRATING");
        lv_obj_set_style_text_color(s_ui.lbl_fb, lv_color_hex(UI_WARN), 0);
        ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                "TARGET IGNORED");
    } else if (running == 0U) {
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "MOTOR OFF");
        lv_obj_set_style_text_color(s_ui.lbl_fb, lv_color_hex(UI_BAD), 0);
        ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback), "");
    } else {
#if APP_FOC_CLOSED_LOOP_ENABLE
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "FOC RUNNING");
#else
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "OPENLOOP RUN");
#endif
        lv_obj_set_style_text_color(s_ui.lbl_fb, lv_color_hex(UI_OK), 0);

#if APP_FOC_SPEED_LOOP_ENABLE
        ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                "SPEED LOOP OWNS IQ");
#else
        /* 没有速度环时, 目标转速只作为软限速/开环频率给定 —— 老实告诉用户,
         * 免得看到"实际转速不等于目标"以为是 bug。 */
        eff_i = (int32_t)((target_eff >= 0.0f) ? (target_eff + 0.5f)
                                               : (target_eff - 0.5f));
        if ((pending != 0U) || (eff_i != tgt_i)) {
            /* 目标刚被拖过 / 斜坡还没爬到: 用"当前值 -> 目标值"显示出来 */
            ui_text_fmt(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                        "RAMP %d > %d RPM", eff_i, tgt_i);
        } else if (tgt_i >= LCD_UI_RPM_MAX) {
            /* 拉到底 = 不额外限速 (软限速上限仍是 app 里的 APP_FOC_RPM_LIMIT)。
             * 这是刻意的: 600 RPM 低于默认限速 1000 RPM, 若照字面限到 600,
             * 用户一拖到底反而把转速上限压低了。 */
            ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                    "FULL = NO EXTRA LIMIT");
        } else if (tgt_i == 0) {
            ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                    "0 = NO LIMIT");
        } else {
            ui_text_fmt(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                        "LIMIT %d RPM", tgt_i, -1);
        }
#endif
    }
}
