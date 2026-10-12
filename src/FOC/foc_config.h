/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 参数配置
 *   所有可调参数集中在这里, 按实际电机/硬件修改。
 */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* ---- 电机参数 ---- */
#define FOC_MOTOR_POLE_PAIRS            (11.0f)     /* 4015 中空电机极对数 */
/* 相电阻/相电感未知: 先用开环驱动 (foc_openloop) 调试, 不依赖这两个值;
 * 等实际测量(万用表测线电阻/电感表)后再填入, 用于电流环 PI 整定。 */
#define FOC_MOTOR_RS_OHM                (5.4f)      /* 相电阻 (Ω); 若 5.4Ω 是线间值，需改为 2.7Ω */
#define FOC_MOTOR_LS_HENRY              (0.0013f)   /* 相电感 (H); 若 1.3mH 是线间值，需改为 0.65mH */
#define FOC_MOTOR_FLUX_WB               (0.0f)      /* 磁链 (Wb), 未知填 0 */

/* ---- 编码器安装 ---- */
#define FOC_ENCODER_DIRECTION           (1.0f)      /* +1 或 -1, 编码器正方向与电机正方向关系 */
/* 电角度偏置 (rad)。
 *
 * 【怎么来的】用静态角度扫描标定 (app.h 的 APP_CAL_SWEEP_ENABLE):
 *   把磁场按电角度分 12 步钉死, 每步等转子吸稳后读编码器角度, 由
 *       offset = normalize(sw_elec - mech * POLE_PAIRS)
 *   算出 12 个值: 2153 1870 1939 1833 1910 1934 2021 2116 2141 2228
 *                 2254 2331  ->  均值 2061 mrad
 *   散布 ±250 mrad 是齿槽转矩造成的静态平衡误差 (转子没完全对齐磁场)。
 *
 * 【为什么不能用旋转中的数据标定】旋转时负载角在整圈里摆动 (实测负载角
 *   可摆 ±250°), DIFF 会扫遍全场, 任何一个采样值都不代表安装偏置。
 *
 * 【验证方法】改回标定模式重跑一次, CALSTEP 各行算出的 DIFF 应当接近 0。 */
#define FOC_ELEC_ANGLE_OFFSET           (2.061f)

/* ---- 控制频率 ---- */
#define FOC_PWM_FREQ_HZ                 (20000U)    /* 载波频率, 与 bsp_pwm 一致 */
#define FOC_CURRENT_LOOP_FREQ_HZ        (20000U)    /* 电流环频率, 通常 = PWM */
#define FOC_SPEED_LOOP_FREQ_HZ          (1000U)     /* 速度环频率 */
#define FOC_OPENLOOP_LOOP_FREQ_HZ       (1000U)     /* 开环驱动更新频率 */

/* ---- 开环驱动默认参数 (相电阻/电感未知时的安全初值) ----
 * 调试步骤建议:
 *   1. 先给很小电压 (0.5~1V), 看电机会不会抖/转, 同时盯住相电流;
 *   2. 电流大而转不动 -> 降低 FOC_OPENLOOP_VOLTAGE_V;
 *   3. 能稳定跟随后再逐步升压/升速。 */
#define FOC_OPENLOOP_VOLTAGE_V          (2.0f)      /* 目标电压矢量幅值 (V) */
#define FOC_OPENLOOP_VOLTAGE_SLEW       (1.0f)      /* 电压上升速率 (V/s), 防止冲击 */
#define FOC_OPENLOOP_ACCEL_RAD_S2       (20.0f)     /* 电角加速度 (rad/s^2), 起步要柔 */
#define FOC_OPENLOOP_CURRENT_LIMIT_A    (1.0f)      /* 相电流限幅 (A), <=0 关闭 */
#define FOC_OPENLOOP_VBUS_MIN_V         (3.0f)      /* 母线欠压保护阈值 (V); 太低会误封锁 */

/* ---- 数学库选择 ----
 * 1 = 使用 HPM Math 库 (hpm_math.h) 的查表快速 sin/cos, 速度远快于 libm 的
 *     sinf/cosf, 适合在 20kHz 电流环里调用;
 * 0 = 使用标准 libm (sinf/cosf), 精度最高但较慢。
 * 使用 1 时需保证工程已打开 CONFIG_HPM_MATH (见 CMakeLists.txt)。 */
#define FOC_USE_HPM_MATH                (1)
#define FOC_TRIG_TABLE_SIZE             (2048U)     /* sin 表点数, 2 的幂; 2048 -> 8KB */

/* ---- 电流环 PI (按带宽法预估: Kp = Ls*ωc, Ki = Rs*ωc) ---- */
#define FOC_DEFAULT_CURRENT_KP          (5.0f)
#define FOC_DEFAULT_CURRENT_KI          (200.0f)

/* ---- 限幅 ---- */
#define FOC_CURRENT_LIMIT_A             (2.0f)      /* d/q 电流限幅 (A) */
#define FOC_VOLTAGE_LIMIT_V             (12.0f)     /* 输出电压限幅 (V) */
#define FOC_SPEED_LIMIT_RAD_S           (200.0f)    /* 速度限幅 (rad/s) */

/* ---- 速度环 PI ---- */
#define FOC_SPEED_KP                    (0.02f)
#define FOC_SPEED_KI                    (0.5f)

#endif /* FOC_CONFIG_H */
