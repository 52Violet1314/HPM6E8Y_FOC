/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1 侧: app 接口的"替身"(遥测客户端)
 *
 * 【为什么要有这个文件】src/lcd 下的界面代码原来跟电机控制在同一个镜像里,
 * 直接调 app_speed_get_rpm() 等函数读共享变量。双核之后界面跑在 core1 上,
 * 那些变量在 core0 的地址空间里, 链接都链接不到。
 *
 * 解决办法不是去改界面代码(界面并不知道自己在哪个核上跑), 而是在 core1 这边
 * 提供【同名同签名】的实现, 内部改成读共享内存快照:
 *
 *   界面 (lcd_app.c / lcd_ui.c)
 *        |  app_speed_get_rpm() / app_speed_target_set() ...   <- 语义不变
 *        v
 *   本文件 (core1)  --共享内存-->  core0 的 src/dualcore/core1_boot.c
 *        ^                                    |
 *        +---- 快照 (shm_client_pull) --------+
 *
 * 写入方向只有一个: 目标转速(app_speed_target_set -> 共享内存命令区),
 * 由 core0 的控制任务在 app_speed_apply() 里真正下发, 仍然是"写者唯一"。
 * 也就是说: 界面代码一行都不用为双核改动, 跨核这件事被关在这两个文件里。
 */
#include "app.h"            /* 与 core0 共用同一份声明: 签名不一致会在这里编译报错 */
#include "shm_core.h"
#include "shm_client.h"

float app_speed_get_rpm(void)
{
    return shm_client_telem()->rpm;
}

float app_speed_get_target(void)
{
    return shm_client_telem()->target;
}

float app_speed_get_target_eff(void)
{
    return shm_client_telem()->target_eff;
}

void app_speed_target_set(float rpm)
{
    /* 夹到与 core0 相同的量程: 界面上滑条本来就限幅了, 这里再兜一次,
     * 免得以后加了别的输入源(按键/串口)把越界值送进控制环。 */
    if (rpm > APP_SPEED_TARGET_MAX_RPM) {
        rpm = APP_SPEED_TARGET_MAX_RPM;
    } else if (rpm < -APP_SPEED_TARGET_MAX_RPM) {
        rpm = -APP_SPEED_TARGET_MAX_RPM;
    }
    shm_client_set_target(rpm);
}

uint8_t app_speed_target_pending(void)
{
    return shm_client_target_apply_pending();
}

uint8_t app_speed_is_running(void)
{
    return ((shm_client_telem()->status & SHM_STATUS_RUNNING) != 0U) ? 1U : 0U;
}

uint8_t app_speed_is_calibrating(void)
{
    return ((shm_client_telem()->status & SHM_STATUS_CALIBRATING) != 0U) ? 1U : 0U;
}

/* ---- 位置环开关 (屏幕上的那个 switch) ----
 * core1 只能"发命令", 真正的使能位在 core0 的 s_motion.enable 里。
 * 所以这里做成: 写 = 发命令(cmd_motion), 读 = 看 core0 回报的状态位。 */
void app_motion_set_enable(uint8_t enable)
{
    shm_client_set_motion(enable);
}

/* 屏幕上的位置滑条 -> core0 的位置目标。
 * 【为什么这里只发一个整数】显示核不能直接改位置环状态(那是控制任务的数据),
 * 只能把目标值经共享内存发过去, 由 core0 在 core1_sync() 里按序号锁存
 * (见 src/dualcore/core1_boot.c 对 cmd_pos_seq 的处理)。 */
void app_motion_set_target_rad(float pos_rad)
{
    shm_client_set_pos_mrad((int32_t)(pos_rad * 1000.0f));
}

uint8_t app_motion_is_enabled(void)
{
    return ((shm_client_telem()->status & SHM_STATUS_MOTION) != 0U) ? 1U : 0U;
}
