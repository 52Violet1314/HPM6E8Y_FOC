/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 电流环录波 (自己调 PI 用的"示波器")
 *
 * 【为什么需要它】电流环跑在 20kHz 的 ADC 中断里, 而串口是 115200 baud
 *   (11.5 字符/ms): 一条 45 字符的 CSV 就要 4ms 的串口时间。想"每拍都打"
 *   是不可能的 —— 打出来也会被日志环形缓冲整行丢掉, 曲线全是洞。
 *
 * 【做法】ISR 里只做"往 RAM 数组里写一组 int16"(几十个周期, 对 20kHz 无感),
 *   数组按环形一直覆盖; 命中触发条件后【再采一半窗口】才冻结, 然后由控制任务
 *   按 APP_SCOPE_DUMP_MS 节流转储成 CSV。于是拿到的是【触发点前后居中】的
 *   一段波形(默认 ±25.6ms, 10kHz 抽点 512 行), 既能看阶跃响应, 也能看
 *   失控瞬间的起点。
 *
 * 【触发源】
 *   1) 自动: |Δiq_ref| > APP_SCOPE_TRIG_DIQ_MA —— 速度环/位置环一改给定就是
 *      一次阶跃, 于是每次"扰动"都会被自动录下来(包括飞车的头几十毫秒);
 *   2) 过流保护触发时(app.c 的 OVERCURRENT 那一行)会强制录一次;
 *   3) APP_SCOPE_AUTO_MS > 0 时, 每 N ms 无条件录一次(看稳态/连续行为用)。
 *
 * 【输出格式】每行:
 *      SCOPE,<t_us>,<id_ref>,<id>,<iq_ref>,<iq>,<ud>,<uq>
 *   单位: t_us = 相对窗口起点的微秒; 电流 mA; 电压 mV。
 *   转储开头先打一行 "#SCOPE t_us,id_ref,id,iq_ref,iq,ud,uq" 给脚本当表头。
 *   直接喂给 Python/gnuplot 就能画: iq_ref 与 iq 的差就是电流环的误差,
 *   ud/uq 顶到母线幅值 = PI 饱和(限幅/解耦没做好)。
 *
 * 【调参怎么用】
 *   - 看阶跃: 波形里 iq 追上 iq_ref 的时间 ≈ 电流环带宽; 有明显过冲/振荡 ->
 *     减 Kp 或减 Ki;
 *   - 看饱和: ud/uq 长期贴住 Vbus/√3 就是电压不够(或角度错), 这时再调 gains
 *     也没用, 先解决角度/限幅;
 *   - 看静差: 稳态 iq_ref - iq 不为 0 且长时间不收敛 -> Ki 太小或积分被限幅。
 */
#ifndef APP_SCOPE_H
#define APP_SCOPE_H

#include <stdint.h>

/* ================= 配置 ================= */

/* 0 = 整个录波不编译(省 24KB RAM, 也不占串口带宽)
 * 【为什么默认关】转储期间串口被 SCOPE 数据占满, 实测把 "ADC 中断被拖长"
 * 从 220us 抬到 300~440us —— 调电流环要的是干净的数据, 不是这个。
 * 想看 20kHz 瞬态时再置 1。 */
#define APP_SCOPE_ENABLE        (0)

/* 环形窗口长度(拍)。1024 拍 @20kHz = 51.2ms */
#define APP_SCOPE_SAMPLES       (1024U)

/* 转储抽点: 2 -> 等效 10kHz, 512 行; 1 -> 20kHz 全量(串口要慢一倍) */
#define APP_SCOPE_DECIM         (2U)

/* 转储节流(ms/行)。10ms/行 ≈ 45 字符/10ms ≈ 4.5 字符/ms, 串口(11.5)有余量。
 * 【别调太小】实测 6ms/行时 "ADC 中断被拖长" 从 220us 涨到 450us —— 串口
 * 负担会经日志任务传导到电流环的中断延迟上, 录波数据本身也会被污染。 */
#define APP_SCOPE_DUMP_MS       (10U)

/* 触发条件: |Δiq_ref| 超过它(mA)就录。速度环输出一变就会超过 */
#define APP_SCOPE_TRIG_DIQ_MA   (200)

/* >0 = 每 N ms 无条件录一次; 0 = 只靠阶跃/过流触发 */
#define APP_SCOPE_AUTO_MS       (0U)

/* 两次录波之间的最小间隔(ms)。不加冷却的话: 失控时 iq_ref 每几毫秒就大幅跳变,
 * 会一路触发 -> 串口被 SCOPE 数据淹掉(正常日志全被挤掉)。 */
#define APP_SCOPE_COOLDOWN_MS   (5000U)

/* 触发后再采多少拍才冻结(窗口居中)。默认半个窗口 = 前 25.6ms + 后 25.6ms */
#define APP_SCOPE_POST_N        (APP_SCOPE_SAMPLES / 2U)

/* 采样周期(us) —— 必须和 APP_ADC_ISR_FREQ_HZ 一致(20kHz -> 50us) */
#define APP_SCOPE_TS_US         (50U)

/* ================= 接口 ================= */

/* 在 20kHz 电流环中断里每拍调用一次(只写数组, 不做浮点/不做打印) */
void app_scope_sample(int32_t id_ref_ma, int32_t iq_ref_ma,
                      int32_t id_ma, int32_t iq_ma,
                      int32_t ud_mv, int32_t uq_mv);

/* 立刻安排一次录波(过流保护等处调用; 中断里也可调用) */
void app_scope_trigger(void);

/* 在控制任务(1kHz)里每拍调用: 推进自动触发计时 + 节流转储 */
void app_scope_poll(void);

/* 1 = 正在等后置采样或正在转储 */
uint8_t app_scope_busy(void);

#endif /* APP_SCOPE_H */
