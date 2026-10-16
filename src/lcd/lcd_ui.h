/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * LCD: LVGL 电机界面
 *
 *   布局 (320x240, ILI9341):
 *     ┌──────────────────────────────┐
 *     │ 电机转速  MOTOR SPEED        │  标题
 *     │         320 RPM              │  大字: 实测转速 (app 的编码器估算值)
 *     │  目标转速 ────────            │
 *     │  0 [========o────] 600       │  速度条: 拖动设定目标转速 (RPM)
 *     │  目标 300 RPM                 │
 *     │  FOC 电流环 / 校准中 / 已停机  │  电机状态
 *     └──────────────────────────────┘
 *
 *   【数据流】本模块不直接读电机内部状态, 全部经 app.h 暴露的接口:
 *     app_speed_get_rpm()      -> 显示用实测转速
 *     app_speed_get_target()   -> 回读目标转速 (用于刷新滑条位置)
 *     app_speed_set_target()   -> 滑条回调写目标转速
 *   这样 UI 层与 FOC/编码器解耦, 也不会在中断上下文里碰 LVGL。
 */
#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 速度条量程 (RPM)。上限取 600: 12V 母线 / KV316 的机械转速上限约 400 RPM,
 * 再留一点余量。想改量程只改这两个宏 (lcd_ui.c 与 app.h 各自有对应的默认值)。 */
#define LCD_UI_RPM_MIN      (0)
#define LCD_UI_RPM_MAX      (600)

/** @brief 建立电机界面 (在 lv_init + display/indev 创建之后调用一次) */
void lcd_ui_create(void);

/**
 * @brief  刷新界面 (由显示任务周期调用, 内部有 20ms 节流)
 * @param  rpm       实测机械转速 (RPM, 带符号)
 * @param  target    目标转速 (RPM, 用于在界面里显示"目标 xxx RPM"并回写滑条)
 * @param  target_eff 经斜坡后真正下发的值 (RPM); 与 target 不等就说明还在爬
 * @param  running   电机是否受控运行
 * @param  busy      是否处于标定/扫描阶段 (此时目标转速不被采用)
 * @param  pending   目标值刚被改过 (app_speed_target_pending 的返回值)
 * @note   只应由显示任务调用 (LVGL 不是线程安全的)。
 *         滑条位置【只在用户没在拖动时】才被 target 覆盖, 否则会和手指打架。
 */
void lcd_ui_refresh(float rpm, float target, float target_eff,
                    uint8_t running, uint8_t busy, uint8_t pending);

/** @return 界面上滑条当前设定的目标转速 (RPM, 无符号 0..600) */
int32_t lcd_ui_get_target_rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_UI_H */
