/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 电流环录波实现 (说明见 app_scope.h)
 */
#include "app_scope.h"
#include "app_log.h"        /* app_log_write_raw(): 原样打一行, 自动补 CRLF */

#include <stdio.h>          /* snprintf */
#include <string.h>         /* memcpy */

#if APP_SCOPE_ENABLE

/* 一组样本: 12 字节, 1024 组 = 12KB (AXI_SRAM 里, 不占 flash) */
typedef struct {
    int16_t id_ref;         /* mA */
    int16_t id;
    int16_t iq_ref;
    int16_t iq;
    int16_t ud;             /* mV */
    int16_t uq;
} app_scope_samp_t;

#define APP_SCOPE_MASK      (APP_SCOPE_SAMPLES - 1U)

/* 状态机:
 *   0 IDLE     采样中, 顺便判阶跃触发
 *   1 WAITPOST 已触发, 再采 APP_SCOPE_POST_N 拍(窗口居中)后冻结
 *   2 FROZEN   已冻结: ISR【停止写入】, 等控制任务把窗口拷进快照
 *   3 DUMPING  快照已就绪, 正在按 APP_SCOPE_DUMP_MS 节流转储; ISR 恢复采样
 *
 * 【为什么必须有 FROZEN 这一档】第一版没有: 冻结后 ISR 照写不误, 而 512 行
 * 转储要 5 秒 —— 这 5 秒里 20kHz 的 ISR 把环形缓冲绕了 100 圈, 转储读到的
 * "窗口"其实是 100 段不同时间的样本交织在一起。症状: 波形里 iq_ref 每 100us
 * 就在 ±1500 之间翻一次(1kHz 的速度环根本做不到), 那是别名, 不是真波形。
 * 现在冻结即停写, 由任务侧拷到 s_snap, 转储期间 ISR 立刻恢复采样 ——
 * 采样只盲 ~1ms(一拍控制周期), 而不是 5 秒。 */
#define APP_SCOPE_IDLE      (0U)
#define APP_SCOPE_WAITPOST  (1U)
#define APP_SCOPE_FROZEN    (2U)
#define APP_SCOPE_DUMPING   (3U)

static app_scope_samp_t s_buf[APP_SCOPE_SAMPLES];    /* 环形, ISR 写 */
static app_scope_samp_t s_snap[APP_SCOPE_SAMPLES];   /* 快照, 任务写/读 */

/* 【为什么这些是 volatile】写在 ISR 里, 读在控制任务里;
 * 32 位对齐访问在 RV32 上是原子的, 单生产者单消费者不需要额外加锁。 */
static volatile uint32_t s_w;               /* 累计写入拍数(环形) */
static volatile uint32_t s_frozen_end;      /* 冻结时的 s_w = 窗口右端 */
static volatile uint32_t s_post;            /* 后置窗口剩余拍数 */
static volatile uint8_t  s_state;
static volatile int16_t  s_prev_iq_ref;     /* 上一拍的 iq_ref: 判阶跃 */
static volatile uint8_t  s_have_prev;

static uint32_t s_dump_i;                   /* 转储进度(窗口内偏移) */
static uint32_t s_dump_acc;                 /* 转储节流累加(ms) */
static uint32_t s_cooldown_ms;              /* 冷却: 两次录波的最小间隔 */
static uint8_t  s_head_done;                /* 表头是否已打 */
#if (APP_SCOPE_AUTO_MS > 0U)
static uint32_t s_auto_acc;                 /* 自动触发累加(ms) */
#endif

static int16_t app_scope_clamp16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return (int16_t)v;
}

void app_scope_trigger(void)
{
    if ((s_state == APP_SCOPE_FROZEN) || (s_state == APP_SCOPE_DUMPING)) {
        return;                             /* 上一份还没转储完, 别覆盖 */
    }
    if (s_cooldown_ms != 0U) {
        return;                             /* 冷却中: 防止失控时把串口刷爆 */
    }
    s_post  = APP_SCOPE_POST_N;
    s_state = APP_SCOPE_WAITPOST;
}

void app_scope_sample(int32_t id_ref_ma, int32_t iq_ref_ma,
                      int32_t id_ma, int32_t iq_ma,
                      int32_t ud_mv, int32_t uq_mv)
{
    app_scope_samp_t *p;

    if (s_state == APP_SCOPE_FROZEN) {
        return;                             /* 已冻结: 停写, 保住窗口(见状态机说明) */
    }

    p = &s_buf[s_w & APP_SCOPE_MASK];
    p->id_ref = app_scope_clamp16(id_ref_ma);
    p->id     = app_scope_clamp16(id_ma);
    p->iq_ref = app_scope_clamp16(iq_ref_ma);
    p->iq     = app_scope_clamp16(iq_ma);
    p->ud     = app_scope_clamp16(ud_mv);
    p->uq     = app_scope_clamp16(uq_mv);
    s_w++;

    if (s_state == APP_SCOPE_WAITPOST) {
        /* 后置窗口采够 -> 冻结。窗口 = 最后 APP_SCOPE_SAMPLES 拍 */
        if (s_post == 0U) {
            s_frozen_end = s_w;
            s_state      = APP_SCOPE_FROZEN;
        } else {
            s_post--;
        }
    } else if (s_state == APP_SCOPE_IDLE) {
        /* 自动触发: iq_ref 出现阶跃。首次不判(避免上电第一拍误触发) */
        if (s_have_prev != 0U) {
            int32_t d = iq_ref_ma - (int32_t)s_prev_iq_ref;

            if ((d > APP_SCOPE_TRIG_DIQ_MA) || (d < -APP_SCOPE_TRIG_DIQ_MA)) {
                app_scope_trigger();
            }
        } else {
            s_have_prev = 1U;
        }
    }
    /* DUMPING: 照常采样(转储读的是快照, 互不干扰), 但不再判触发 */
    s_prev_iq_ref = app_scope_clamp16(iq_ref_ma);
}

void app_scope_poll(void)
{
    char line[96];

    /* 冷却是按"控制任务每拍 = 1ms"推进的 */
    if (s_cooldown_ms != 0U) {
        s_cooldown_ms--;
    }

    /* ---- 冻结 -> 拷进快照, 立刻放 ISR 回去采样 ---- */
    if (s_state == APP_SCOPE_FROZEN) {
        uint32_t start = s_frozen_end - APP_SCOPE_SAMPLES;
        uint32_t head  = APP_SCOPE_SAMPLES - (start & APP_SCOPE_MASK);

        /* 环形窗口可能跨过回绕点, 分两段拷 */
        (void)memcpy(&s_snap[0], &s_buf[start & APP_SCOPE_MASK],
                     head * sizeof(app_scope_samp_t));
        if (head < APP_SCOPE_SAMPLES) {
            (void)memcpy(&s_snap[head], &s_buf[0],
                         (APP_SCOPE_SAMPLES - head) * sizeof(app_scope_samp_t));
        }
        s_dump_i    = 0U;
        s_dump_acc  = APP_SCOPE_DUMP_MS;    /* 下一拍就开始打, 不额外等 */
        s_head_done = 0U;
        s_state     = APP_SCOPE_DUMPING;
        return;
    }

#if (APP_SCOPE_AUTO_MS > 0U)
    if ((s_state == APP_SCOPE_IDLE) && (s_cooldown_ms == 0U)) {
        s_auto_acc++;
        if (s_auto_acc >= APP_SCOPE_AUTO_MS) {
            s_auto_acc = 0U;
            app_scope_trigger();
        }
    }
#endif

    if (s_state != APP_SCOPE_DUMPING) {
        return;
    }

    /* 节流转储: 串口只有 11.5 字符/ms, 一次全打出去必然被环形缓冲丢掉 */
    s_dump_acc++;
    if (s_dump_acc < APP_SCOPE_DUMP_MS) {
        return;
    }
    s_dump_acc = 0U;

    if (s_head_done == 0U) {
        app_log_write_raw("#SCOPE t_us,id_ref,id,iq_ref,iq,ud,uq");
        s_head_done = 1U;
    }

    {
        const app_scope_samp_t *p = &s_snap[s_dump_i];

        (void)snprintf(line, sizeof(line),
                       "SCOPE,%u,%d,%d,%d,%d,%d,%d",
                       (unsigned)(s_dump_i * APP_SCOPE_TS_US),
                       (int)p->id_ref, (int)p->id,
                       (int)p->iq_ref, (int)p->iq,
                       (int)p->ud, (int)p->uq);
        app_log_write_raw(line);
    }

    s_dump_i += APP_SCOPE_DECIM;
    if (s_dump_i >= APP_SCOPE_SAMPLES) {
        s_dump_i      = 0U;
        s_head_done   = 0U;
        s_state       = APP_SCOPE_IDLE;     /* 重新武装 */
        s_cooldown_ms = APP_SCOPE_COOLDOWN_MS;
    }
}

uint8_t app_scope_busy(void)
{
    return (s_state != APP_SCOPE_IDLE) ? 1U : 0U;
}

#else   /* !APP_SCOPE_ENABLE: 全部空实现, 调用点不用加 #if */

void app_scope_sample(int32_t id_ref_ma, int32_t iq_ref_ma,
                      int32_t id_ma, int32_t iq_ma,
                      int32_t ud_mv, int32_t uq_mv)
{
    (void)id_ref_ma; (void)iq_ref_ma; (void)id_ma;
    (void)iq_ma;     (void)ud_mv;     (void)uq_mv;
}

void app_scope_trigger(void) { }
void app_scope_poll(void) { }
uint8_t app_scope_busy(void) { return 0U; }

#endif
