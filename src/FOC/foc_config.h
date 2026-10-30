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
/* 极对数: 换过电机 (10 极 = 5 对极)。
 * 【怎么确认的】静态扫描时磁场每转一个电周期, 编码器实测前进 26053~26323 counts,
 * 一圈 131072 -> 131072/26157 = 5.01; 且 5 个电周期正好走满一个机械圈
 * (CAL SWEEP end: fieldrev=5 odo=131156)。配 12 时残差铺满整个电周期
 * (spread ±3125 ≈ ±π), 标出来的 offset 是垃圾值, 闭环会顶住-踢-再顶住地抖。 */
#define FOC_MOTOR_POLE_PAIRS            (5.0f)
/* 相电阻/相电感: 【旧电机 (11 极对) 的实测值, 换电机后尚未复测】。
 * 保留它们是给电流环 PI 一个量级正确的起点; 若新电机电阻差一倍以上,
 * 电流环会偏软/偏硬, 需要用万用表(相间/2)重测后更新。 */
#define FOC_MOTOR_RS_OHM                (2.7f)      /* 相电阻 (Ω) —— 按【相】值, 待复测 */
#define FOC_MOTOR_LS_HENRY              (0.0013f)   /* 相电感 (H) = 1.3mH, 待复测 */
#define FOC_MOTOR_FLUX_WB               (0.0f)      /* 磁链 (Wb), 未知填 0 */

/* ---- 编码器安装 ----
 *
 * 【方向 = -1, 这是实测确定的, 不是猜的】
 * 判据来自一整段连续日志(不是单点): 正 iq 时电机物理上在正转, 但编码器
 * 原始计数【单调减小】。证据链:
 *    iq_ref=+1500mA, uq 顶到 5660mV(电流环在往正方向满力推)
 *    rpm 报 -558, 位置报 -2000000 mrad(一路变负)
 *    vbus 有反电势 -> 电机确实在转
 *   => 正 iq 让 raw 减小, 所以"raw 增大"对应的是【电机负方向】。
 * mech_rad = FOC_ENCODER_DIRECTION * raw, 要让它随电机正转增大, 必须取 -1。
 *
 * 【这一处错了会导致什么】mech_rpm 和位置反馈的符号会一起反:
 *    速度环: 给定 +60 -> 实测 -558 -> 误差 +618 -> iq 顶正限幅
 *            -> 正 iq 让电机正转 -> 编码器报更负 -> 正反馈 -> 飞车
 *    位置环: 目标 5887 -> 反馈一路变负 -> 误差涨到 2e6 -> 速度给定拉满
 *  表现就是"上电电机一直转、iq 为正、uq 顶限幅、pos 无限漂移"。
 * 即 FOC_ENCODER_DIRECTION 用错时, 任何下游的符号修正都只是把错误搬到别处。
 *
 * 【它不影响换向】本工程用内置离线 LUT 换向 (APP_ENC_USE_BUILTIN_LUT=1),
 * 电角度由 app_enc_lut_elec_rad(raw) 直接给出, 与这个宏无关;
 * 这个宏只作用于 mech_rad / mech_rpm(速度估算与位置反馈)。
 * 【改它之后要同步】app.h 的 APP_SPEED_DIR_SIGN 应保持 +1(此时
 * "正 iq -> 正 rpm" 已经成立), 不要再叠一次取反。 */
#define FOC_ENCODER_DIRECTION           (-1.0f)     /* 实测: 正 iq 让 raw 减小 -> -1 */
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
#define FOC_ELEC_ANGLE_OFFSET           (0.420f)

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

/* ---- 电流环 PI (带宽法: Kp = Ls*ωc, Ki = Rs*ωc) ----
 *
 * 参数: Ls = 1.3 mH; R 用【实测的视在值】—— 开机自检里 500mV 对应约 160mA,
 *   反推 Rs ≈ 3.1 Ω (含电流换算误差, 但它正是环路实际"看到"的电阻)。
 *
 * 【关键前提: 电流环跑在 20kHz 的 ADC 中断里】(见 app.h 的 APP_CURRENT_LOOP_IN_ISR)
 *   本电机 τ = Ls/Rs = 0.42ms, 比 1kHz 主循环周期还短 —— 在 1kHz 里调它,
 *   无论怎么整定都在悬崖边(实测: ωc=40Hz 低速勉强, 100RPM 以上就失控)。
 *   搬进 ADC 完成中断后 ts = 50us, 采样→计算→更新占空比在同一载波周期内完成,
 *   延迟只剩约 1 个 PWM 周期, 带宽可以放到几百 Hz。
 *
 * ωc = 2π×300 = 1885 rad/s
 *     Kp = Ls×ωc = 0.0013 × 1885 = 2.45   V/A
 *     Ki = Rs×ωc = 3.1    × 1885 = 5844   V/(A·s)
 *   延迟相位 ≈ ωc×50us = 5.4°, 加采样保持 2.7° -> 相位裕度约 82°, 很稳。
 *   (若想更快可以按同样公式提到 500Hz: Kp=4.08/Ki=9740, 但先确认这版稳定。)
 *
 * PI 实现见 pid.c (通用 PID): out = kp·e + 积分, 积分 += ki·dt·e, 故 Ki 单位 V/(A·s)。
 * 【2026-10-18】原来这里指向 foc_pi_run(), 那份实现只有一个 out_limit、积分限幅
 * 被硬编成 ±out_limit, 且抗饱和看的是单轴限幅 —— 而真正的饱和在矢量限幅里,
 * 于是积分器被钉死在限幅上(实测 ud/uq 恒 ±4200mV、id 差 1A)。现统一到 pid.c,
 * 并在矢量限幅处按比例退积分(反算抗饱和)。 */
/* 【2026-10-18 带宽 300Hz -> 100Hz -> 60Hz: 啸叫 = 电流环高频自激】
 * 现象: 位置环守住之后电机发出啸叫, 而 CSV 里 iq_ref 只有 40~70mA 时
 *       iq 实测却在 ±500~900mA 来回甩、ud/uq 满摆 ±6V —— 环路用 2~6V 去补一个
 *       自己造出来的 1~2kHz 振荡, PWM 载波把它变成声频啸叫。
 * 机理: 带宽 300Hz(=1885rad/s) 的相位裕度被"20kHz 采样 + 实测每 2 拍丢 1 拍的
 *       100~120us 抖动 + 任务抢 CPU"吃光。100Hz 仍可闻, 再降到 60Hz。
 * 【关键: Kp/Ki 必须同比例缩放】一阶 R-L 对象的标准整定是
 *       Kp = Ls·ωc,  Ki = Rs·ωc
 *   零点 Ki/Kp = Rs/Ls 与对象极点对消; 只动一个会让零点偏离 Rs/Ls, 相位更差。
 *   所以 100->60Hz 是 Kp 和 Ki 都乘 0.6:
 *       0.817×0.6 = 0.49,  1948×0.6 = 1169
 * 【还想更低就 40Hz: Kp 0.33 / Ki 779 —— 再低就带不动负载了】 */
#define FOC_CURRENT_BW_HZ               (60.0f)
#define FOC_DEFAULT_CURRENT_KP          (0.49f)     /* = Ls·ωc = 1.3mH × 377 rad/s */
#define FOC_DEFAULT_CURRENT_KI          (1169.0f)   /* = Rs·ωc = 2.7Ω × 377 rad/s  */

/* 电流环【积分】限幅 = 输出限幅(vmax) × 这个比例。
 *   1.0 = 积分器可以独自顶满电压(两轴合成可到 √2·vmax) —— 【会自激, 别用】
 *   0.5 = 积分器合成矢量最多占一半电压, 一半留给 Kp 项
 *
 * 【2026-10-18 从 1.0 改到 0.5, 这是本次啸叫的直接病根】
 * 电流环隔离实验(只给 d 轴 0.3A、外环全关)实测: idpp≈2100mA / iqpp≈1500mA 自激,
 * 而 sqrt(ud²+uq²) 恒等于 5960mV(=vmax) 还在转 —— 积分器被钉在限幅圆上出不来。
 * 配合 foc.c 里新增的"积分器矢量限幅", 现在积分矢量被硬压到 vmax×0.5=2.95V,
 * Kp 项永远留有余量, 极限环消失。
 * 【代价】积分器给的稳态电压上限 2.95V -> 稳态电流上限约 1.1A(2.95/2.7);
 * 高速时要靠反电动势+Kp 项, 若发现高速带不动可以把比例往 0.6~0.7 提,
 * 但别回到 1.0(那就回到自激)。根本解法是加反电动势前馈/解耦。 */
#define FOC_CURRENT_INT_LIMIT_RATIO     (0.5f)

/* ---- 相电流一阶低通 (削开关尖峰/ADC 抖动) ----
 * 20kHz 采样下:  τ = ts/α,  fc = α/(2π·ts)
 *   α=1.0  -> 关闭(直通)
 *   α=0.5  -> fc≈1.6kHz, 群延迟≈100us
 *   α=0.3  -> fc≈1.0kHz, 群延迟≈160us
 *   α=0.2  -> fc≈0.64kHz
 * 【为什么加】开关瞬间的共模 dv/dt 会经运放输入电容耦合到差分端(1k+27pF 只滤
 *   ~27ns, 挡不住), 采样点上偶尔就带毛刺; 再加上 ADC 的 ±1LSB 抖动。这些噪声
 *   会被 Kp 放大成电压抖动 -> 绕组可闻啸叫。
 * 【为什么低通比降 P 好】降 P 只是让环路对噪声不敏感, 噪声还在; 低通从源头削。
 * 【代价】相位滞后: 电流环 100Hz(628rad/s) 时 1.6kHz 低通附加滞后 <10°, 可接受;
 *   若把带宽提回 300Hz 必须重新评估(那时滞后会吃掉裕度)。 */
#define FOC_CURRENT_LPF_ALPHA           (0.5f)

/* ---- 限幅 ---- */
#define FOC_CURRENT_LIMIT_A             (2.0f)      /* d/q 电流限幅 (A) */
#define FOC_VOLTAGE_LIMIT_V             (12.0f)     /* 输出电压限幅 (V) */
#define FOC_SPEED_LIMIT_RAD_S           (200.0f)    /* 速度限幅 (rad/s) */

/* ---- 速度环 PI ---- */
#define FOC_SPEED_KP                    (0.02f)
#define FOC_SPEED_KI                    (0.5f)

#endif /* FOC_CONFIG_H */
