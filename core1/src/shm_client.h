/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1 侧: 共享内存客户端
 *
 * 把 src/dualcore/shm_core.h 里那块内存包装成 core1 能用的几件事:
 *   - 启动握手(magic/version)
 *   - 上报启动阶段/致命错误/心跳 (core1 没有控制台, 这就是它的"串口")
 *   - 以"整帧一致"的方式取 core0 发布的遥测快照
 *   - 把界面改的目标转速写回给 core0
 */
#ifndef SHM_CLIENT_H
#define SHM_CLIENT_H

#include <stdint.h>

/* core1 本地的一份遥测快照 (shm_client_pull() 填) */
typedef struct {
    float    rpm;
    float    target;
    float    target_eff;
    float    dir;
    uint32_t status;
    uint32_t seq;           /* 快照对应的 core0 遥测序号(调试用) */
} core1_telem_t;

/**
 * @brief  和 core0 完成共享内存握手
 * @return 1=成功, 0=失败(magic/version 不对)
 * @note   带有限次重试: core0 是先写共享区、再释放 core1 的, 正常一次就过。
 */
int shm_client_attach(void);

/** @brief 上报启动阶段 (CORE1_STAGE_*, 定义在 shm_core.h) */
void shm_client_stage(uint32_t stage);

/** @brief 上报致命错误码 (CORE1_FAULT_*) */
void shm_client_fault(uint32_t fault);

/** @brief 心跳: 显示任务每拍调一次 (core0 靠它判断显示核还活着) */
void shm_client_beat(void);

/**
 * @brief  拉取一帧"一致"的遥测快照
 * @note   用序号做 seqlock: 读序号 -> 读数据 -> 再读序号, 两次序号不一样就重读。
 *         显示差一帧无所谓, 但"同一帧里一半是新值、一半是旧值"会让界面数字乱跳。
 *         每帧调一次即可(显示任务 5ms 一拍)。
 */
void shm_client_pull(void);

/** @brief 取本地快照 (必须先在当前任务里调过 shm_client_pull) */
const core1_telem_t *shm_client_telem(void);

/** @brief 把界面设定的目标转速发给 core0 (滑条回调里调, 无锁, 立即返回) */
void shm_client_set_target(float rpm);

/**
 * @brief  "目标转速刚被 core0 下发到控制环" ? (读走即清, 语义同 core0 的
 *         app_speed_target_pending())
 */
uint8_t shm_client_target_apply_pending(void);

#endif /* SHM_CLIENT_H */
