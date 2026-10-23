/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1 侧: 共享内存客户端实现
 */
#include "board.h"              /* board_delay_us */
#include "shm_core.h"
#include "shm_client.h"

static core1_telem_t s_telem;           /* 本核私有的快照, 只有显示任务碰它 */
static uint32_t s_last_apply_seq;       /* 上次看到的"已下发"计数器 */

int shm_client_attach(void)
{
    shm_t *shm = shm_get();
    uint32_t retry;

    /* core0 是共享区的初始化者, 而它一定在释放 core1 之前写完 magic/version,
     * 所以正常情况第一次判断就过。留重试只是为了"以后有人改了启动顺序"时
     * 不至于直接死等 —— 但也不会无限等: 200ms 之后仍然不对就认输。 */
    for (retry = 0U; retry < 200U; retry++) {
        if ((shm->magic == SHM_MAGIC) && (shm->version == SHM_VERSION)) {
            return 1;
        }
        board_delay_us(1000U);
    }
    return 0;
}

void shm_client_stage(uint32_t stage)
{
    shm_get()->core1_stage = stage;
}

void shm_client_fault(uint32_t fault)
{
    shm_t *shm = shm_get();

    shm->core1_fault = fault;
    shm->core1_stage = CORE1_STAGE_FATAL;
}

void shm_client_beat(void)
{
    shm_t *shm = shm_get();

    if (shm->core1_stage != CORE1_STAGE_RUNNING) {
        shm->core1_stage = CORE1_STAGE_RUNNING;   /* 第一拍: 告诉 core0 "我在刷屏了" */
    }
    (void)__atomic_add_fetch(&shm->core1_alive, 1U, __ATOMIC_RELAXED);
}

void shm_client_pull(void)
{
    shm_t *shm = shm_get();
    uint32_t seq0;
    uint32_t seq1;

    /* seqlock: core0 只在两次序号自增之间改数据, 所以"两次读到的序号相同"
     * 就说明中间没有发生过写入, 这一帧是自洽的。最多试 4 次, 拿不到就用
     * 手上这份(显示差一帧完全无所谓, 不能在这里耗时间)。 */
    for (int attempt = 0; attempt < 4; attempt++) {
        seq0 = __atomic_load_n(&shm->telemetry_seq, __ATOMIC_ACQUIRE);
        s_telem.rpm        = shm->rpm;
        s_telem.target     = shm->target;
        s_telem.target_eff = shm->target_eff;
        s_telem.dir        = shm->dir;
        s_telem.status     = shm->status;
        __asm volatile("fence r, r" ::: "memory");
        seq1 = __atomic_load_n(&shm->telemetry_seq, __ATOMIC_ACQUIRE);
        if (seq0 == seq1) {
            s_telem.seq = seq1;
            return;
        }
    }
    s_telem.seq = seq1;
}

const core1_telem_t *shm_client_telem(void)
{
    return &s_telem;
}

void shm_client_set_target(float rpm)
{
    shm_t *shm = shm_get();
    uint32_t seq = __atomic_load_n(&shm->cmd_seq, __ATOMIC_RELAXED);

    /* 先写数据, 再写序号(release): core0 看到序号变了才会去读这个值 */
    shm->cmd_target_rpm = rpm;
    __atomic_store_n(&shm->cmd_seq, seq + 1U, __ATOMIC_RELEASE);
}

uint8_t shm_client_target_apply_pending(void)
{
    shm_t *shm = shm_get();
    uint32_t seq = __atomic_load_n(&shm->target_apply_seq, __ATOMIC_ACQUIRE);
    uint8_t pending = 0U;

    if (shm_seq_changed(seq, s_last_apply_seq) != 0) {
        s_last_apply_seq = seq;
        pending = 1U;
    }
    return pending;
}

/* 屏幕开关 -> core0 的位置环使能。边沿事件, 用独立序号。 */
static volatile uint8_t s_motion_req;

void shm_client_set_motion(uint8_t on)
{
    shm_t *shm = shm_get();
    uint32_t seq;
    uint8_t  v = (on != 0U) ? 1U : 0U;

    if (v == s_motion_req) {
        return;                 /* 值没变: 不占序号, 免得 core0 反复重新锁存目标 */
    }
    s_motion_req = v;

    seq = __atomic_load_n(&shm->cmd_motion_seq, __ATOMIC_RELAXED);
    shm->cmd_motion = v;        /* 先写数据 */
    __atomic_store_n(&shm->cmd_motion_seq, seq + 1U, __ATOMIC_RELEASE);   /* 再写序号 */
}

/* 屏幕上的位置滑条: 目标位置 (mrad)。语义和 set_motion 一样 ——
 * 值没变就不占序号(拖动时同一像素来回蹭不该反复下发), 变了才发。 */
void shm_client_set_pos_mrad(int32_t mrad)
{
    shm_t   *shm = shm_get();
    uint32_t seq;
    static int32_t s_pos_req;

    if (mrad == s_pos_req) {
        return;                 /* 值没变: 不占序号 */
    }
    s_pos_req = mrad;

    seq = __atomic_load_n(&shm->cmd_pos_seq, __ATOMIC_RELAXED);
    shm->cmd_pos_mrad = mrad;   /* 先写数据 */
    __atomic_store_n(&shm->cmd_pos_seq, seq + 1U, __ATOMIC_RELEASE);      /* 再写序号 */
}

uint8_t shm_client_motion_requested(void)
{
    return s_motion_req;
}
