/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 应用编排层
 *
 *   职责: 把各功能模块组装成一个完整的电机应用。
 *     - app_diag     诊断状态 (debugger 可 watch 的 g_diag)
 *     - app_log      分级日志 + 多后端 (printf + USB)
 *     - app_encoder  编码器封装 (角度/方向/速度估算)
 *     - foc_controller 控制算法 (开环/电流环)
 *     - bsp          PWM/ADC/USB/SEI 硬件驱动
 *
 *   main.c 只负责 board_init + app_init + 主循环外壳(调 app_run)。
 *   依赖方向: app -> (app_*) + FOC + bsp, 不反向; bsp 不依赖 app。
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 运行模式 ================= */
/* 开环驱动: 电机相电阻/电感未知时的首选, 不依赖电流环参数。
 *   1 = 运行开环驱动 (默认)
 *   0 = 不运行, PWM 固定 50% */
#define APP_OPENLOOP_DEMO_ENABLE    (1)

/* FOC 电流环演示: 需要先知道相电阻/电感并整定好 PI。与开环互斥。 */
#define APP_FOC_DEMO_ENABLE         (0)

/* 调试用: 只把电压矢量固定在一个电角度上(转子被"吸住"/对齐), 不旋转。
 *   1 = 固定角度(用于验证 功率级/接线/PWM 是否正常)
 *   0 = 正常旋转开环 */
#define APP_OPENLOOP_ALIGN_ONLY     (0)
#define APP_OPENLOOP_ALIGN_DEG      (0.0f)

/* ================= 开环调试参数 ================= */
#define APP_OPENLOOP_TARGET_RPM     (60.0f)   /* 目标机械转速 (RPM) */
#define APP_OPENLOOP_VOLTAGE_V      (2.0f)    /* 电压矢量幅值 (V) */
#define APP_OPENLOOP_CURRENT_LIMIT  (2.0f)    /* 相电流限幅 (A) */
#define APP_OPENLOOP_ACCEL          (20.0f)   /* 电角加速度 (rad/s^2) */

/* 主循环周期 (ms) */
#define APP_MAIN_LOOP_MS            (1U)
/* 上报周期 (主循环次数) */
#define APP_REPORT_TICKS            (200U)

/**
 * @brief  初始化整个应用 (外设 + FOC + 编码器 + 日志 + 回调注册)
 * @note   内部完成 PWM->ADC->USB->SEI->FOC 的初始化与启动顺序,
 *         并注册 ADC/USB/SEI 中断回调。调用前需先 board_init()。
 */
void app_init(void);

/**
 * @brief  执行一次应用主循环 (采集 -> 控制 -> 上报)
 * @note   建议每 APP_MAIN_LOOP_MS(1ms) 调用一次。
 */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
