/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 双核共享内存: core0(电机控制) <-> core1(LVGL 显示) 之间唯一的数据通路
 *
 * 【为什么需要它】两个核跑的是两份独立的镜像, 各有自己的链接地址空间: core1
 * 根本链接不到 core0 的 app.c 符号, core0 也读不到 core1 的 LVGL 状态。以前
 * "显示任务读 app_speed_get_rpm()" 这种同进程函数调用, 现在必须变成"跨核读一块
 * 双方约定好的内存"。
 *
 * 【为什么放在 SHARE_RAM】两块内存谁都能访问, 又不能各链接一份:
 *   SHARE_RAM = 0x012FC000, 16KB, 两个链接脚本(flash_xip.ld / ram_core1.ld)里
 *   都叫 SHARE_RAM, 都定义为 NOLOAD。这里用链接脚本给的符号
 *   __share_mem_start__ 来定位, 而不是各自定义一个 .sh_mem 变量 —— 后者一旦两个
 *   镜像的段顺序/对齐有一点差别, 就会变成"两个核各写各的、永远对不上"这种最难查的
 *   故障; 用同一个链接符号则物理上必然是同一块内存。
 *
 * 【不需要 cache 维护】board_init_pmp() 把 [__share_mem_start__, __share_mem_end__)
 * 配成 MEM_TYPE_MEM_NON_CACHE_BUF(非缓存)。注意 PMA/PMP 是【每个核独立】的,
 * 所以两个核都必须调 board_init_pmp() —— core1 忘了调, 就会看到过期数据。
 *
 * 【并发模型】全部是"单生产者-单消费者"的无锁结构, 没有自旋锁:
 *   - 遥测 (core0 写 -> core1 读): 32 位对齐的标量 + 一个序号, core1 靠序号判断
 *     "这一帧是新的还是旧的"(读到撕裂的一帧也无所谓, 显示差一帧不影响控制)。
 *   - 命令 (core1 写 -> core0 读): 同上一套, 只是方向相反。
 *   - 日志 (core1 写 -> core0 读): 定长槽位环形缓冲, 槽位状态字做握手, 满了就丢。
 * 每个核都是自己那部分数据的唯一写者, 这是不需要锁的前提。
 */
#ifndef SHM_CORE_H
#define SHM_CORE_H

#include <stdint.h>
#include <string.h>

/* 放在文件里的标识: core1 启动时先校验这两个值, 不匹配说明两边镜像对共享区布局的
 * 理解不一致(改结构体忘了同步编译两边/链接脚本被改过), 必须停下来, 不能装作没事。 */
#define SHM_MAGIC           (0x4432484DUL)   /* 'M','H','2','D' */
/* 【v4】增加"位置滑条"命令(cmd_pos_mrad/cmd_pos_seq): 屏幕上直接拖目标位置。
 * 结构体变了 -> 版本必须跟着变, 否则两边镜像不一致时会带着错的偏移跑。 */
#define SHM_VERSION         (4U)

/* ---------------- core1 启动阶段码 (shm->core1_stage) ----------------
 * core1 上电后没有控制台(见 core1/src/main.c), 所以它把"走到哪一步"写进共享内存,
 * core0 的日志任务负责打出来。显示不出来时先看这个数:
 *   一直在 1 -> core1 卡在最早的启动代码(时钟/PMP)
 *   2 -> 共享内存握手过了, 卡在 LCD_Init()/触摸
 *   3 -> 面板好了, 卡在 lv_init/建界面
 *   4 -> 界面好了, 卡在 FreeRTOS 调度器
 *   6 -> 显示任务在跑(正常) */
#define CORE1_STAGE_ENTER       (1U)    /* 进了 core1 的 main */
#define CORE1_STAGE_SHM_OK      (2U)    /* 共享内存握手成功 */
#define CORE1_STAGE_PANEL_OK    (3U)    /* LCD/触摸初始化完成 */
#define CORE1_STAGE_UI_OK       (4U)    /* LVGL 界面建好 */
#define CORE1_STAGE_RTOS_OK     (5U)    /* 调度器已启动 */
#define CORE1_STAGE_RUNNING     (6U)    /* 显示任务在刷屏(每拍刷 core1_alive) */
#define CORE1_STAGE_SHM_BAD     (0xE1U) /* 共享内存 magic/version 不符: 死等 */
#define CORE1_STAGE_FATAL       (0xEFU) /* 致命错误, 细节见 core1_fault */

/* core1 致命错误码 (shm->core1_fault) */
#define CORE1_FAULT_NONE            (0U)
#define CORE1_FAULT_TASK_CREATE     (1U)    /* xTaskCreate 失败(堆不够) */
#define CORE1_FAULT_STACK_OVERFLOW  (2U)    /* 任务栈溢出 */
#define CORE1_FAULT_MALLOC          (3U)    /* FreeRTOS 堆耗尽 */
#define CORE1_FAULT_LCD_INIT        (4U)    /* 面板初始化失败 */

/* core0 状态位 (shm->status) */
#define SHM_STATUS_RUNNING      (1UL << 0)  /* 电机闭环受控运行中 */
#define SHM_STATUS_CALIBRATING  (1UL << 1)  /* 正在标定/静态扫描 */
#define SHM_STATUS_MOTION       (1UL << 2)  /* 位置环已使能(屏幕上的开关状态, 由 core0 回报) */

/* ---------------- 日志环 ----------------
 * 定长槽位: 每条日志一个槽位, 槽位自己带状态字(空/满)。这样生产者不需要处理
 * "一条消息跨过环尾"的折返拷贝, 消费者也不需要拼包 —— 少一半出错机会。
 * 32 槽 x 192 字节 ≈ 6KB, 对"几秒才几行"的显示日志绰绰有余; 满了直接丢并计数,
 * 绝不阻塞 core1 的刷屏(刷屏卡住的代价比丢一行日志大得多)。 */
#define SHM_LOG_SLOTS       (32U)
#define SHM_LOG_LINE_LEN    (192U)

typedef struct {
    volatile uint32_t state;                    /* 0=空 1=满; 生产/消费靠它握手 */
    char              text[SHM_LOG_LINE_LEN];   /* 以 '\0' 结尾的整行(不含换行) */
} shm_log_slot_t;

typedef struct {
    /* ---- 头部: 握手用 ---- */
    volatile uint32_t magic;
    volatile uint32_t version;

    /* ---- core0 -> core1: 遥测快照 ----
     * 顺序有讲究: 先写数据, 最后写 telemetry_seq(RELEASE), core1 先读 seq
     * 再读数据(ACQUIRE), 这样"seq 变了"就一定能看到配套的新数据。 */
    volatile uint32_t telemetry_seq;
    volatile float    rpm;              /* 实测机械转速 (RPM, 带符号) */
    volatile float    target;           /* 目标转速 (RPM) */
    volatile float    target_eff;       /* 经斜坡后真正下发的值 (RPM) */
    volatile float    dir;              /* 速度环方向系数 (+1/-1) */
    volatile uint32_t status;           /* SHM_STATUS_* 位 */
    volatile uint32_t target_apply_seq; /* 每把目标下发到控制环一次就 +1 */
    volatile uint32_t core0_alive;      /* core0 控制任务每拍 +1 (1kHz) */

    /* ---- core1 -> core0: 命令 ---- */
    volatile float    cmd_target_rpm;   /* 显示滑条设定的目标转速 */
    volatile uint32_t cmd_seq;          /* 每写一次 +1; core0 靠它判断"有新命令" */
    /* 屏幕上的"位置环开/关"(1 = 位置模式, 0 = 速度模式)。
     * 【为什么单独一个 seq】cmd_seq 是"值变了就重发"的语义, 而位置环使能是
     * 【边沿事件】: app_motion_enable(true) 会把"当前位置"重新锁存成目标。
     * 如果和滑条共用一个 seq, 那么每拖一次滑条都会重新锁存一次位置目标,
     * 位置环就再也守不住(等于每次都被重置)。所以给它独立的序号, core0 只在
     * 这个序号变化时才动使能位。 */
    volatile uint32_t cmd_motion;       /* 0/1: 显示核要的位置环使能 */
    volatile uint32_t cmd_motion_seq;   /* 每写一次 +1 */

    /* 【位置滑条】屏幕上的位置目标 (mrad, 0..6283 = 0..360° 机械角)。
     * 【为什么又单独一个 seq】拖手指时这个值会连着变很多次。如果和 cmd_motion
     * 共用一个序号, 那每拖一次滑条都会被 core0 当成"使能边沿"处理一次 ——
     * 而使能边沿会把当前位置重新锁存成目标(见上面), 于是位置环每拖一下就重置,
     * 根本守不住。所以三个命令各用各的序号。 */
    volatile int32_t  cmd_pos_mrad;     /* 0..6283 mrad */
    volatile uint32_t cmd_pos_seq;      /* 每写一次 +1 */

    /* ---- core1 存活/诊断 ---- */
    volatile uint32_t core1_alive;      /* core1 显示任务每拍 +1 (200Hz) */
    volatile uint32_t core1_stage;      /* CORE1_STAGE_* */
    volatile uint32_t core1_fault;      /* CORE1_FAULT_* */
    volatile uint32_t core1_img_bytes;  /* core0 填: core1 镜像(flash 里那段)的字节数 */

    /* ---- 日志环 (core1 生产, core0 消费) ---- */
    volatile uint32_t log_head;         /* 只有 core1 写 */
    volatile uint32_t log_tail;         /* 只有 core0 写 */
    volatile uint32_t log_dropped;      /* core1 因环满丢弃的行数 */
    shm_log_slot_t    log[SHM_LOG_SLOTS];

    /* ---- 反向日志环 (core0 生产, core1 消费) ----------------
     * 【为什么需要】core0 自己的两个输出口都指望不上: USB CDC 一断电就没了,
     * UART0 接的是板上调试口(不一定接出来)。而 core1 现在有一条独立的 UART14。
     * 于是 core0 的日志经这个环交给 core1 转发出去 —— 两个核的日志最终都出现在
     * 同一个串口上, core0 那边一根线都不用接。 */
    volatile uint32_t log0_head;        /* 只有 core0 写 */
    volatile uint32_t log0_tail;        /* 只有 core1 写 */
    volatile uint32_t log0_dropped;     /* core0 因环满丢弃的行数 */
    shm_log_slot_t    log0[SHM_LOG_SLOTS];
} shm_t;

/* 链接脚本符号: 两个镜像里都存在, 都指向 SHARE_RAM 的起点(0x012FC000) */
extern uint8_t __share_mem_start__[];

/** @brief 取共享区指针 (任何时刻、任何核都能调, 只是取个地址) */
static inline shm_t *shm_get(void)
{
    return (shm_t *)(void *)__share_mem_start__;
}

/* 编译期护栏: 结构体必须装得进 16KB 的 SHARE_RAM。装不下就在这里编译报错,
 * 而不是运行起来把隔壁的 AXI_SRAM 踩了。 */
typedef char shm_size_must_fit_in_share_ram[(sizeof(shm_t) <= (16U * 1024U)) ? 1 : -1];

/* ---------------- 环形槽位的通用实现 (两个方向各用一套) ----------------
 * 单生产者/单消费者: 两个指针各自只有一个核写, 32 位对齐访问在 RV32 上是原子的。 */
static inline int shm_ring_push(volatile uint32_t *head_p, volatile uint32_t *dropped_p,
                                shm_log_slot_t *slots, const char *line)
{
    uint32_t head;
    shm_log_slot_t *slot;

    if ((head_p == NULL) || (slots == NULL) || (line == NULL)) {
        return 0;
    }
    head = __atomic_load_n(head_p, __ATOMIC_RELAXED);
    slot = &slots[head % SHM_LOG_SLOTS];

    if (__atomic_load_n(&slot->state, __ATOMIC_ACQUIRE) != 0U) {
        /* 消费者还没腾出这个槽位: 丢, 并记账。绝不等待。 */
        if (dropped_p != NULL) {
            (void)__atomic_add_fetch(dropped_p, 1U, __ATOMIC_RELAXED);
        }
        return 0;
    }

    {
        size_t n = strlen(line);
        if (n >= SHM_LOG_LINE_LEN) {
            n = SHM_LOG_LINE_LEN - 1U;      /* 超长截断: 槽位定长, 不留半个字符 */
        }
        memcpy(slot->text, line, n);
        slot->text[n] = '\0';
    }

    __atomic_store_n(&slot->state, 1U, __ATOMIC_RELEASE);
    __atomic_store_n(head_p, head + 1U, __ATOMIC_RELEASE);
    return 1;
}

static inline int shm_ring_pop(volatile uint32_t *tail_p, shm_log_slot_t *slots,
                               char *out, uint32_t out_sz)
{
    uint32_t tail;
    shm_log_slot_t *slot;

    if ((tail_p == NULL) || (slots == NULL) || (out == NULL) || (out_sz == 0U)) {
        return 0;
    }
    tail = __atomic_load_n(tail_p, __ATOMIC_RELAXED);
    slot = &slots[tail % SHM_LOG_SLOTS];

    if (__atomic_load_n(&slot->state, __ATOMIC_ACQUIRE) == 0U) {
        return 0;
    }

    {
        size_t n = strlen(slot->text);
        if (n >= out_sz) {
            n = out_sz - 1U;
        }
        memcpy(out, slot->text, n);
        out[n] = '\0';
    }

    __atomic_store_n(&slot->state, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(tail_p, tail + 1U, __ATOMIC_RELEASE);
    return 1;
}

/* ---------------- 日志环: 单生产者入队 (core1) ----------------
 * @return 1=已入队, 0=环满(丢弃) */
static inline int shm_log_push(shm_t *s, const char *line)
{
    if (s == NULL) {
        return 0;
    }
    return shm_ring_push(&s->log_head, &s->log_dropped, s->log, line);
}

/* ---------------- 日志环: 单消费者出队 (core0) ----------------
 * @return 1=取到一行(已写进 out), 0=环空 */
static inline int shm_log_pop(shm_t *s, char *out, uint32_t out_sz)
{
    if (s == NULL) {
        return 0;
    }
    return shm_ring_pop(&s->log_tail, s->log, out, out_sz);
}

/* ---------------- 反向环: core0 生产 (日志) ---------------- */
static inline int shm_log0_push(shm_t *s, const char *line)
{
    if (s == NULL) {
        return 0;
    }
    return shm_ring_push(&s->log0_head, &s->log0_dropped, s->log0, line);
}

/* ---------------- 反向环: core1 消费 (转发到 UART14) ---------------- */
static inline int shm_log0_pop(shm_t *s, char *out, uint32_t out_sz)
{
    if (s == NULL) {
        return 0;
    }
    return shm_ring_pop(&s->log0_tail, s->log0, out, out_sz);
}

/* SD 卡式的"距离"运算: 两个单调递增的序号不相等就说明有变化(不比较大小,
 * 这样计数器回绕也不会误判)。 */
static inline int shm_seq_changed(uint32_t a, uint32_t b)
{
    return (a != b) ? 1 : 0;
}

#endif /* SHM_CORE_H */
