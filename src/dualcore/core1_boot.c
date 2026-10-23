/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 双核: core1(显示核) 的启动与运行时协作 —— core0 侧实现
 * 设计说明见 core1_boot.h / src/dualcore/core1_boot_info.h / src/dualcore/shm_core.h。
 */
#include <string.h>

#include "board.h"
#include "hpm_misc.h"           /* HPM_CORE1 */
#include "hpm_sysctl_drv.h"     /* sysctl_set_cpu_entry / sysctl_release_cpu */

#include "FreeRTOS.h"
#include "task.h"

#include "app.h"                /* app_speed_*: 控制任务侧的读写都从这里走 */
#include "app_log.h"
#include "core1_boot_info.h"    /* CORE1_IMG_*: 镜像头约定(两个核共用) */
#include "shm_core.h"
#include "core1_boot.h"

/* core1 镜像: 由 cmake/gen_core1_img.py 从 core1.bin 生成, 链接时被
 * --section-start 钉在 CORE1_IMG_BASE(0x80100000) —— 也就是 core1 就地执行的那段
 * flash。core0 自己不执行它, 只做两件事: 检查头 + 把入口告诉硬件。
 * sec_core_img_size 用于日志和上界检查。 */
extern const uint8_t  sec_core_img[];
extern const uint32_t sec_core_img_size;

/* ---- 运行时状态 ---- */
static uint32_t s_last_cmd_seq;         /* 最近一次处理过的显示核命令序号 */
static uint32_t s_last_motion_seq;      /* 同上, 位置环开关命令(边沿事件) */
static uint32_t s_last_pos_seq;         /* 同上, 位置滑条命令(值变了就锁存) */
static uint32_t s_last_alive;           /* 最近一次看到的 core1 心跳值 */
static uint32_t s_last_stage;           /* 最近一次看到的 core1 启动阶段 */
static uint32_t s_last_health_tick;     /* 心跳检查节流 (1s) */
static uint8_t  s_core1_state;          /* 0=还没释放 1=在跑 2=停摆 */
static uint8_t  s_started;

/* 启动阶段 -> 人能读的名字 (core1 没有控制台, 这是它唯一的"喊话"方式) */
static const char *core1_stage_name(uint32_t stage)
{
    switch (stage) {
    case 0U:                    return "还没进 main";
    case CORE1_STAGE_ENTER:     return "已进 main";
    case CORE1_STAGE_SHM_OK:    return "共享内存握手完成";
    case CORE1_STAGE_PANEL_OK:  return "面板/触摸初始化完成";
    case CORE1_STAGE_UI_OK:     return "LVGL 界面建立完成";
    case CORE1_STAGE_RTOS_OK:   return "FreeRTOS 调度器已启动";
    case CORE1_STAGE_RUNNING:   return "显示任务正在刷屏";
    case CORE1_STAGE_SHM_BAD:   return "共享内存 magic/version 不符(两边镜像不一致?)";
    case CORE1_STAGE_FATAL:     return "致命错误";
    default:                    return "未知阶段";
    }
}

void core1_start(void)
{
    shm_t *shm = shm_get();
    /* core1 的镜像头就在它的入口之前 16 字节(见 core1_boot_info.h) */
    const core1_boot_info_t *info = (const core1_boot_info_t *)(uintptr_t)CORE1_IMG_BASE;

    /* 1) 共享区清零 + 握手头。core0 是这块内存的唯一初始化者(它先跑),
     *    清完再放 core1, core1 一进来就能看到合法的 magic。 */
    (void)memset(shm, 0, sizeof(*shm));
    shm->magic = SHM_MAGIC;
    shm->version = SHM_VERSION;
    shm->core1_img_bytes = sec_core_img_size;

    /* 2) 链接期自检: 镜像数组必须正好被放在 CORE1_IMG_BASE。
     *    它由根 CMakeLists 的 -Wl,--section-start 决定; 万一有人动了那条链接选项,
     *    core1 就会在别的地方执行一段"以为自己在这个地址"的代码 —— 必须挡住。
     *    顺带: core0 里对 sec_core_img 的这次引用, 也让这段不会被 --gc-sections 丢掉。 */
    if ((uint32_t)(uintptr_t)sec_core_img != CORE1_IMG_BASE) {
        LOG_E("DUAL", "core1 镜像被链接到 0x%08X, 不是约定的 0x%08X -> 不释放 core1",
              (unsigned)(uintptr_t)sec_core_img, (unsigned)CORE1_IMG_BASE);
        return;
    }
    if ((sec_core_img_size == 0U) || (sec_core_img_size > CORE1_IMG_MAX_SIZE)) {
        LOG_E("DUAL", "core1 镜像大小异常: %u 字节(上限 %u) -> 不释放 core1",
              (unsigned)sec_core_img_size, (unsigned)CORE1_IMG_MAX_SIZE);
        return;
    }

    /* 3) 运行期检查: 镜像头。这一步专门抓"忘了烧 / 烧了旧版本"——
     *    没有它, core0 会把 core1 扔进空白 flash(0xFF 0xFF ...)里执行, 现象是随机跑飞。 */
    if ((info->magic != CORE1_IMG_MAGIC) ||
        (info->version != CORE1_IMG_VERSION) ||
        (info->entry != CORE1_IMG_ENTRY)) {
        LOG_E("DUAL", "core1 镜像头无效(magic=0x%08X ver=%u entry=0x%08X) -> 不释放 core1",
              (unsigned)info->magic, (unsigned)info->version, (unsigned)info->entry);
        return;
    }

    if (sysctl_is_cpu_released(HPM_SYSCTL, HPM_CORE1)) {
        /* 正常上电不会走到这里(复位后 CPU1 是 halt 的)。会走到只有两种可能:
         * 调试器用 dual-core 配置 attach 过(它 examine 时会 release core1),
         * 或者 core0 软复位过而 core1 没复位。两种情况下 core1 都在跑一份
         * 来路不明的代码, 所以下面先把它按住再重新放一次。 */
        LOG_W("DUAL", "core1 已处于释放状态(调试器 attach 过?), 先按住再重新释放");
    }

    /* 4) 先按住 core1, 让"写入口 -> 释放"从一个确定的状态开始 */
    HPM_SYSCTL->CPU[HPM_CORE1].LP |= SYSCTL_CPU_LP_HALT_MASK;
    __asm volatile("fence" ::: "memory");

    /* 5) 写入口地址 + 释放: 清掉 LP.HALT, core1 就从入口开始取指(flash 就地执行)。
     *    入口由链接脚本固定为 0x80100010(镜像头之后), 见 core1/linker/core1_flash.ld。 */
    if (sysctl_set_cpu_entry(HPM_SYSCTL, HPM_CORE1, CORE1_IMG_ENTRY) != status_success) {
        LOG_E("DUAL", "sysctl_set_cpu_entry 失败 -> core1 不启动");
        return;
    }
    sysctl_release_cpu(HPM_SYSCTL, HPM_CORE1);

    s_started = 1U;
    s_core1_state = 1U;
    LOG_I("DUAL", "core1 镜像 %u 字节 @ flash 0x%08X, 入口 0x%08X, 已释放 core1",
          (unsigned)sec_core_img_size, (unsigned)CORE1_IMG_BASE, (unsigned)CORE1_IMG_ENTRY);
}

void core1_sync(void)
{
    shm_t *shm = shm_get();
    uint32_t status = 0U;
    uint32_t cmd_seq;

    if ((s_started == 0U) || (shm->magic != SHM_MAGIC)) {
        return;     /* 共享区还没建好(极早期), 本拍不交换 */
    }

    /* ---- 1) 下行: 显示核(滑条)写下的目标转速 ----
     * 先读序号(acquire), 序号变了才读数据 —— 这样读到的一定是"配套这一版序号"
     * 的值, 而不是写者改到一半的中间态。多读一次(序号又变了)也无害。 */
    cmd_seq = __atomic_load_n(&shm->cmd_seq, __ATOMIC_ACQUIRE);
    if (shm_seq_changed(cmd_seq, s_last_cmd_seq) != 0) {
        s_last_cmd_seq = cmd_seq;
        app_speed_target_set(shm->cmd_target_rpm);
    }

    /* ---- 1c) 下行: 屏幕位置滑条的目标位置 (mrad) ----
     * 【和上面两个的区别】它是"值变了就锁存"的语义(拖动时连续变), 不是边沿事件 ——
     * 所以拖到哪就守到哪, 不会像使能边沿那样把当前位置重置成目标。
     * app_motion_set_target_rad() 内部只写 pos_ref, 不动使能, 随便连续调都安全。 */
    {
        uint32_t pseq = __atomic_load_n(&shm->cmd_pos_seq, __ATOMIC_ACQUIRE);

        if (shm_seq_changed(pseq, s_last_pos_seq) != 0) {
            int32_t mrad = shm->cmd_pos_mrad;

            s_last_pos_seq = pseq;
            app_motion_set_target_rad((float)mrad * 0.001f);
            /* 显示用: mrad -> 度 (1 deg = 17.4533 mrad), 用整数算免得引入浮点打印 */
            LOG_I("MOT", "屏幕滑条: 目标位置 %d mrad (%d.%02d deg)",
                  (int)mrad, (int)(mrad / 17), (int)(((mrad % 17) * 100) / 17));
        }
    }

    /* ---- 1b) 下行: 屏幕上的"位置环开/关" ----
     * 【必须用独立序号】app_motion_set_enable(1) 会把"当前位置"重新锁存成目标
     * (见 app_motion_enable 的说明), 所以它只能在【边沿】上执行一次 —— 要是和
     * 滑条共用一个序号, 每拖一次滑条就会把位置目标重置一次, 位置环永远守不住。 */
    {
        uint32_t mseq = __atomic_load_n(&shm->cmd_motion_seq, __ATOMIC_ACQUIRE);

        if (shm_seq_changed(mseq, s_last_motion_seq) != 0) {
            s_last_motion_seq = mseq;
            app_motion_set_enable((shm->cmd_motion != 0U) ? 1U : 0U);
            LOG_I("MOT", "屏幕开关: 位置环 %s (速度模式/位置模式已切换)",
                  (shm->cmd_motion != 0U) ? "ON (位置模式)" : "OFF (速度模式)");
        }
    }

    /* ---- 2) 上行: 遥测快照 ----
     * 先写数据, 最后写序号(release): core1 看到序号变了, 就一定看得到这些数据。 */
    if (app_speed_is_running() != 0U) {
        status |= SHM_STATUS_RUNNING;
    }
    if (app_speed_is_calibrating() != 0U) {
        status |= SHM_STATUS_CALIBRATING;
    }
    /* 把位置环的【实际】使能状态回报给屏幕: 界面据此把开关画成真实状态,
     * 而不是"我按了什么就显示什么"(使能可能因为未运行/故障没生效)。 */
    if (app_motion_is_enabled() != 0U) {
        status |= SHM_STATUS_MOTION;
    }
    shm->rpm = app_speed_get_rpm();
    shm->target = app_speed_get_target();
    shm->target_eff = app_speed_get_target_eff();
    shm->dir = app_speed_get_dir();
    shm->status = status;

    /* "目标值刚被下发到控制环" —— 这个标志原本是显示任务读走即清的, 现在显示任务
     * 在另一个核上, 读走即清会把"一瞬间的脉冲"丢掉(显示核 5ms 才看一次)。
     * 所以改成计数器: core0 读走标志后 +1, core1 自己记住上次看到的值,
     * 只要不相等就说明"有新目标被下发"。 */
    if (app_speed_target_pending() != 0U) {
        (void)__atomic_add_fetch(&shm->target_apply_seq, 1U, __ATOMIC_RELAXED);
    }

    (void)__atomic_add_fetch(&shm->telemetry_seq, 1U, __ATOMIC_RELEASE);
    (void)__atomic_add_fetch(&shm->core0_alive, 1U, __ATOMIC_RELAXED);
}

void core1_log_drain(void)
{
    shm_t *shm = shm_get();
    char line[SHM_LOG_LINE_LEN];
    uint32_t budget = 8U;       /* 每拍最多转发 8 行, 别把日志任务堵在 core1 上 */

    if ((s_started == 0U) || (shm->magic != SHM_MAGIC)) {
        return;
    }

    while (budget > 0U) {
        budget--;
        if (shm_log_pop(shm, line, sizeof(line)) == 0) {
            break;
        }
        app_log_write_direct(line);
    }
}

void core1_health_check(void)
{
    shm_t *shm = shm_get();
    uint32_t tick = (uint32_t)xTaskGetTickCount();
    uint32_t alive;
    uint32_t stage;

    if ((s_started == 0U) || (shm->magic != SHM_MAGIC)) {
        return;
    }
    if ((tick - s_last_health_tick) < 1000U) {
        return;                 /* 1s 一次就够, 心跳是 200Hz */
    }
    s_last_health_tick = tick;

    /* ---- 启动阶段变化: 逐级打出来, 这是 core1 唯一的"串口" ---- */
    stage = shm->core1_stage;
    if (stage != s_last_stage) {
        s_last_stage = stage;
        LOG_I("DUAL", "core1 阶段: %u (%s)", (unsigned)stage, core1_stage_name(stage));
    }

    /* ---- 致命错误 ---- */
    if (shm->core1_fault != CORE1_FAULT_NONE) {
        LOG_E("DUAL", "core1 报致命错误 fault=%u (1=任务建不起 2=栈溢出 3=堆耗尽 4=面板初始化)",
              (unsigned)shm->core1_fault);
        shm->core1_fault = CORE1_FAULT_NONE;    /* 只报一次 */
    }

    /* ---- 心跳 ---- */
    alive = shm->core1_alive;
    if (alive != s_last_alive) {
        s_last_alive = alive;
        if (s_core1_state != 1U) {
            s_core1_state = 1U;
            LOG_I("DUAL", "core1 心跳恢复 (alive=%u)", (unsigned)alive);
        }
    } else if (s_core1_state != 2U) {
        /* 1s 没动: core1 要么卡住了, 要么根本没起来 —— 报阶段码帮定位 */
        s_core1_state = 2U;
        LOG_W("DUAL", "core1 1s 无心跳(阶段 %u: %s), 显示可能停更",
              (unsigned)stage, core1_stage_name(stage));
    }

    /* core1 丢日志说明它刷屏压力大或 core0 转发太慢, 报一次(有计数就够定位) */
    if (shm->log_dropped != 0U) {
        LOG_W("DUAL", "core1 日志因环满丢弃 %u 行", (unsigned)shm->log_dropped);
        shm->log_dropped = 0U;
    }
}
