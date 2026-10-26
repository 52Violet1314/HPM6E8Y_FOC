/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FOC: 一阶线性自抗扰控制器 (First-Order LADRC)
 *
 *   被控对象按"一阶 + 集总扰动"建模:
 *       dy/dt = b0·u + f          (f = 内部未建模动态 + 外部负载/摩擦/反电动势)
 *   控制器由三部分组成:
 *       1) 线性扩张状态观测器 (LESO): 用实测 y 观测 z1≈y、z2≈f
 *              z1' = z2 + b0·u + β1·(y - z1)
 *              z2' =              β2·(y - z1)
 *       2) 误差反馈:      u0 = Kp·(ref - z1)
 *       3) 扰动补偿:      u  = (u0 - z2) / b0
 *   带宽参数化 (整定只需两个频率):
 *              β1 = 2·wo,  β2 = wo²,  Kp = wc
 *       wc = 控制器带宽 (决定跟踪快慢), wo = 观测器带宽 (通常 3~5 倍 wc)。
 *
 * 【为什么 LADRC 适合这里的速度环】
 *   速度环最麻烦的是"负载/摩擦/反电动势/转动惯量都说不准"。一阶 ADRC 把所有
 *   没建模的东西全塞进 z2, 实时反相抵消; b0 只决定"前馈出多大力", 取偏了
 *   ESO 也会把差额补回来 —— 这正是本机"KV 已知、转动惯量靠猜"的场合。
 *
 * 【离散化】前向欧拉, 步长 ts 每个周期可调 (本项目主循环是 1ms, 但 RTOS 下
 *   实测周期会抖动, 调用方应把实测 dt 通过 LADRC_SetTs() 喂进来)。
 *   注意: 原实现 LADRC_Run(Ladrc, Target) 里没有测量 y, 且 z += dot 等于隐式
 *   假设 ts = 1s —— 接到 1kHz 环上观测器会完全失效。这里按标准形式重写,
 *   并保留旧名字 LADARC_Init 的兼容宏 (原文件拼写少了一个 C)。
 */
#ifndef __LADRC_H__
#define __LADRC_H__

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ---- 整定参数 ---- */
    float ts;           /* 离散步长 (s), 即调用周期 */
    float b0;           /* 控制增益: 输入 u 对 y 的导数的影响, 单位 [y/s / u] */
    float inv_b0;       /* 1/b0 (预先算好, 省一次除法) */
    float beta_1;       /* ESO 增益 β1 = 2·wo */
    float beta_2;       /* ESO 增益 β2 = wo² */
    float Kp;           /* 误差反馈增益 = wc */
    float u_max;        /* 输出对称限幅 ±u_max; <= 0 表示不限幅 */

    /* ---- 状态 ---- */
    float z1;           /* ESO: y 的估计 */
    float z2;           /* ESO: 集总扰动 f 的估计 (单位 [y/s]) */
    float u;            /* 上一拍实际输出 (已限幅) */

    /* ---- 遥测快照 (只读, 调参/日志用) ---- */
    float y;            /* 上一拍实测值 */
    float ref;          /* 上一拍给定值 */
} LADRC_typedef;

/**
 * @brief  初始化一阶 LADRC
 * @param  l      控制器实例
 * @param  b0     控制增益 [y/s / u]; 例如速度环里 y=RPM、u=A, 则 b0=RPM/(s·A)
 * @param  wc     控制器带宽 (rad/s)
 * @param  wo     观测器带宽 (rad/s), 一般取 3~5·wc
 * @param  ts     采样周期 (s), 例如 0.001
 * @param  u_max  输出限幅 (对称), <=0 表示不限幅
 */
void LADRC_Init(LADRC_typedef *l, float b0, float wc, float wo,
                float ts, float u_max);

/** @brief 清空观测器/输出状态 (使能、过流解锁、模式切换时必须调) */
void LADRC_Reset(LADRC_typedef *l);

/** @brief 每拍更新实测步长 (s); 非法值 (<=0) 被忽略 */
void LADRC_SetTs(LADRC_typedef *l, float ts);

/** @brief 在线改带宽 (会重算 β1/β2/Kp, 不清状态) */
void LADRC_SetBandwidth(LADRC_typedef *l, float wc, float wo);

/** @brief 在线改控制增益 b0 (会重算 1/b0, 不清状态) */
void LADRC_SetB0(LADRC_typedef *l, float b0);

/**
 * @brief  走一拍
 * @param  l    控制器实例
 * @param  ref  给定值 (与 y 同量纲)
 * @param  y    实测值
 * @return 本拍输出 u (已限幅); 可直接当电流/电压给定
 */
float LADRC_Run(LADRC_typedef *l, float ref, float y);

/* 兼容旧拼写: 原文件里叫 LADARC_Init (少一个 C), 保留宏避免旧调用点报错。
 * 参数顺序也统一成 (b0, wc, wo, ts, u_max) —— 旧代码若按 (b0, wo, Kp, u_max)
 * 调用会少一个参数而编译失败, 这是故意的: 旧签名无法表达采样周期, 必须改。 */
#define LADARC_Init(l, b0, wc, wo, ts, u_max) \
    LADRC_Init((l), (b0), (wc), (wo), (ts), (u_max))

#ifdef __cplusplus
}
#endif

#endif /* __LADRC_H__ */
