/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 应用编排层实现
 *
 *   只做"组装": 把 app_diag / app_log / app_encoder / foc_controller /
 *   bsp 串成一个完整应用。具体逻辑都在各自模块内, 本文件保持精简。
 */

#include <stdio.h>
#include <string.h>

#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"      /* SDK_DECLARE_EXT_ISR_M / intc_m_enable_irq_with_priority */
#include "hpm_uart_drv.h"       /* 控制台 UART 非阻塞发送 (TX FIFO + 中断) */
#include "board.h"              /* BOARD_CONSOLE_UART_BASE / _IRQ */

#include "app.h"
#include "app_diag.h"
#include "app_log.h"
#include "app_encoder.h"
#include "app_enc_lut.h"    /* 离线标定的 3200 组 raw -> 电角度表 */
#include "app_motion.h"     /* 位置环 (外环, 输出速度给定) */
#include "app_nv.h"

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_usb.h"
#include "bsp_button.h"     /* J11 按键 (位置点动) */
#include "bsp_mt6701.h"
#if BSP_MT6701_USE_SEI
#include "bsp_sei.h"
#else
#include "bsp_spi.h"
#include "bsp_spi_bb.h"
#endif

#include "foc_controller.h"

/* ================= 应用实例 ================= */
static foc_controller_t s_ctrl;
static app_encoder_t    s_encoder;
#if APP_MOTION_ENABLE
/* 位置环: 每拍算出"速度给定"写进 s_speed_target_rpm, 由速度环(内环)去追。
 * 详见 src/app/app_motion.c 顶部的分层说明。 */
static app_motion_t     s_motion;
#endif

/* ================= 转速给定 (LCD 速度条用) =================
 *
 * 【为什么是 volatile】写它的是 LVGL 回调 (lcd_ui.c 的 slider_event_cb),
 * 读它的是 app_run —— 同一线程, 但跨越 lcd_app_task_handler() 的调用边界,
 * 加 volatile 防止编译器把读取提到调用之前缓存起来。
 *
 * 【量纲】机械 RPM, 带符号。上电 = APP_SPEED_TARGET_DEFAULT_RPM。
 * 相关开关 (APP_FOC_SPEED_LOOP_ENABLE / APP_SPEED_OPENLOOP_DIRECT) 见 app.h。 */
static volatile float s_speed_target_rpm = APP_SPEED_TARGET_DEFAULT_RPM;

/* ---- 速度环反馈方向 (见 app_init 里的"速度环方向自检") ----
 * 速度环必须是负反馈: 给定 + 时必须看到实测转速也 + 。这个系数把
 * "速度环坐标系" 和 "编码器/iq 坐标系" 对齐, 由开机自检实测决定;
 * 不想自检就把它固定成 APP_SPEED_DIR_SIGN (app.h)。 */
static float s_speed_dir = APP_SPEED_DIR_SIGN;

/* ---- 位置环状态快照 (RTOS 下显示/其它任务只能读这些, 不能碰 s_motion) ---- */
#if APP_MOTION_ENABLE
static volatile uint8_t s_motion_running;    /* 1 = 位置环在跑 */
static volatile uint8_t s_motion_takeover;   /* 1 = 这一拍位置环改写了速度给定 */
#endif

/* ---- 跨任务状态快照 (只有控制任务里的 app_speed_apply() 会写, 显示任务只读) ----
 * RTOS 起来后显示任务是独立任务, 所以"控制任务正在改的结构体"一律不许给显示
 * 任务直接读 —— 这三个量就是给它的只读快照。详见 app.c 末尾转速接口的注释。 */
static volatile uint8_t s_speed_ui_running;      /* 1 = 电机受控运行中 */
static volatile uint8_t s_speed_ui_pending;      /* 1 = 目标值刚被改过 (读走即清) */
static float            s_speed_ui_target_eff;   /* 实际下发值 (经斜坡), 便于界面显示 */
static float            s_speed_ui_last_seen;    /* 上一拍看到的目标值, 用来判"变了" */

#if APP_FOC_SPEED_LOOP_ENABLE
/* 速度环 PI 积分项 (单位 A) 与上一次的 iq 给定 */
static float s_spd_integ;
#endif

/* 校准映射表转储节拍: 每 APP_CAL_MAP_DUMP_TICKS 次上报打印一次整张表。
 * 0 = 关闭。转储会连发 8 次 USB 写, 阻塞控制环、拖慢主循环 —— 这是开环
 * 失步的诱因之一, 默认关闭, 需要看曲线时再打开。 */
static uint32_t s_map_tick;
#define APP_CAL_MAP_DUMP_TICKS  (0U)

#if APP_ENC_LUT_AUTO_SWITCH
/* 0 = 正在学表(电压矢量角度仍用模拟积分角)
 * 1 = 表已学满, 电压矢量角度改用 diag_lut_angle(raw) + 超前角 */
static uint8_t s_lut_full;
/* 1 = 静态扫描造的表被体检否决 (缺口太长 / 整圈不自洽) -> 不换向, 保持开环 VF。
 * 宁可让电机照原样转, 也不能拿一张"编出来"的表去换向 —— 那正是转子在补出来的
 * 那一段里反复弹、堵转发热的根源。 */
static uint8_t s_lut_reject;
/* 换向后"顶住"的连续拍数, 到 APP_ENC_DWELL_LOG_TICKS 就打一行位置 */
static uint32_t s_dwell_ticks;
static uint32_t s_stuck_ms;         /* 换向后连续"顶住"的毫秒数 */
static uint32_t s_kick_ms;          /* 反磁弹簧踢一脚的剩余毫秒数 */
#endif

#if APP_CAL_SWEEP_ENABLE
/* 静态角度扫描标定状态 */
static uint32_t s_sweep_step;
static uint32_t s_sweep_cnt;
static uint32_t s_sweep_ref;        /* 本次保持的 raw 解卷绕参考 */
static int32_t  s_sweep_a1;         /* 前半段(正常电压)增量累加 */
static int32_t  s_sweep_a2;         /* 后半段(弱场)增量累加 */
static uint8_t  s_sweep_done;
static int32_t  s_cal_offset_mrad = (int32_t)(FOC_ELEC_ANGLE_OFFSET * 1000.0f); /* 存档用标定偏移 */
static uint32_t s_sweep_steps;      /* = PHASES × 极对数 × REVS, app_init 里算 */
static uint32_t s_cyc_ref;          /* 上一个电周期边界的 raw, 用来算本周期推进量 */
/* 残差汇总 (用于自动给出推荐偏置) */
static int32_t  s_sweep_res_ref;    /* 第一个残差, 作为解卷绕参考 */
static int32_t  s_sweep_res_acc;    /* 解卷绕后的残差累加 */
static uint32_t s_sweep_res_n;
static int32_t  s_sweep_res_min;
static int32_t  s_sweep_res_max;
/* 强/弱场位移差 dE 的统计: dE = 4·δ_强, 直接反映齿槽/静摩擦把转子拉偏多少 */
static int32_t  s_sweep_de_acc;
static int32_t  s_sweep_de_min;
static int32_t  s_sweep_de_max;
static uint32_t s_sweep_de_n;
static uint32_t s_sweep_de_big;
/* 【整圈行程里程计】扫描的停止条件 = 编码器【净行程走满一整圈(16384 counts)】,
 * 而不是固定步数。固定步数在"极对数不是配置的 12"或"中途丢步"时会留下一段
 * 从没踩到的 raw 区间, 那段只能靠弦插值编出来 —— 换向后就是磁弹簧陷阱。 */
static uint32_t s_sweep_odo_prev;
static int32_t  s_sweep_odo;        /* 净行程 counts, 正 = 沿 raw 增大方向 */
static int32_t  s_sweep_dmax;       /* 单步行程最大/最小值, 看有没有丢步 */
static int32_t  s_sweep_dmin;
static uint32_t s_sweep_stall;      /* 连续"一步没动"的步数 -> 卡住了就提前收尾 */
#define APP_CAL_SWEEP_STALL_MAX  (40U)
static uint32_t s_sweep_pairs;      /* 实测: 走满一整圈磁场转了几个电周期 = 极对数 */
static uint32_t s_sweep_pre;        /* 预吸稳已经过的主循环数 */

/* 把 14bit raw 差值折算到 (-8192, 8192] —— 抗 16384->0 回绕 */
static int32_t cal_wrap_raw(int32_t v)
{
    while (v > 8192)  { v -= 16384; }
    while (v < -8192) { v += 16384; }
    return v;
}

/* 把 mrad 折算到 (-pi, pi] */
static int32_t cal_wrap_mrad(int32_t v)
{
    while (v > 3141) {
        v -= 6283;
    }
    while (v < -3141) {
        v += 6283;
    }
    return v;
}
#endif

/* 当前三相电流缓存 (mA), 供中断回调与主循环共享 */
static volatile int32_t s_iu_ma;
static volatile int32_t s_iv_ma;
static volatile int32_t s_iw_ma;

/* 相电流符号 (低边采样电阻的方向): +1 / -1。
 * 默认 +1; 开机自检 (APP_CURRENT_SIGN_AUTOTEST) 会按实测结果改写它,
 * 也可以用 APP_CURRENT_SIGN=+1/-1 强制指定。
 * 只在初始化阶段写一次, 之后只读 —— 所以中断里直接乘没有竞争问题。 */
static float s_cur_sign = 1.0f;

/* 相电流通道映射: FOC 的 ia/ib/ic 分别取 ADC 返回的哪一路。
 * 默认 {0,1,2} = 直连。板子若把"电流采样通道 <-> 三相"的对应关系接错
 * (哪怕只是 V/W 互换), 闭环反馈就会变成镜像 -> 2θ 调制的坏反馈 -> 电压顶到限幅
 * -> 过流锁死。所以这一项和极性一样, 开机自检必须实测出来 (见 app_init 第二步)。 */
static uint8_t s_phase_map[3] = {0U, 1U, 2U};

#if APP_CURRENT_LOOP_IN_ISR
/* 20kHz 电流环挂在 ADC 完成中断里, 它需要主循环(约1kHz)刷新的量:
 *   - 转子电角度: 编码器读一次要几十微秒(bit-bang), 不可能在 50us 中断里做,
 *     所以主循环读完表算好角度后放进这里。
 *   - 电角速度: 【关键】主循环会被日志打印阻塞几十毫秒, 这期间角度若冻住,
 *     电机在 150RPM 下 20ms 就偏了约 200 度电角度 —— 电流矢量跑飞, 电机顿挫。
 *     所以中断里按这个速度自己外推角度(见 app_on_adc_conv), 主循环只负责
 *     每次拿到新编码器值时"重同步"一次。
 *   - 母线电压: 同上。
 * s_isr_closed_loop = 1 才真正在中断里跑闭环 (初始化阶段的自检要用主循环直接驱动)。 */
static volatile float  s_isr_elec_rad;
static volatile float  s_isr_elec_speed;    /* rad/s (电角度), 中断里外推用 */
static volatile float  s_isr_vbus;
static volatile uint8_t s_isr_closed_loop;
static volatile uint32_t s_isr_cnt;      /* ADC 中断次数: 用来核对是不是 20kHz */
static volatile uint32_t s_isr_dt_max;   /* 本窗口内两次中断的最大间隔(mcycle) */
static volatile uint64_t s_isr_last_mcycle;
/* 【定位"谁堵了中断"】中断被拖长时, 记下当时主循环处于哪个阶段(见 app_diag.h 的
 * APP_STAGE_LOOP_*): 11=VBUS 12=电流 15=编码器 16=LCD 17=日志 18=USB。
 * 用途: 15ms 级的中断停摆会让高速下的电机顿挫, 但光看现象猜不出是谁;
 * 这个值能直接指到人。 */
static volatile uint32_t s_isr_gap_stage;
static volatile uint32_t s_isr_gap_us;
#endif

/* 主循环实测周期(s), 供角度外推估计电角速度用 */
static float s_dt_main = 0.001f;

/* 初始化阶段用的粗延时 (mcycle 计时, 不占外设, 不受中断影响) */
static void app_delay_ms(uint32_t ms)
{
    uint64_t t0 = hpm_csr_get_core_mcycle();
    uint64_t n  = ((uint64_t)hpm_core_clock * ms) / 1000U;

    while ((hpm_csr_get_core_mcycle() - t0) < n) {
        /* busy wait */
    }
}

/* 遥测用的"电压矢量角度"(rad):
 *   开环 = 内部积分角 (= 表值 + 超前角, 每周期被 set_external_angle 写入);
 *   闭环 = 表值本身 (Park/反 Park 用的转子电角度, ud/uq 由 PI 给)。
 * 不区分的话, 闭环时 openloop.elec_angle 停在上电初值, 遥测第 1 列会一直是 0。 */
static float app_vec_angle_rad(void)
{
    return (s_ctrl.mode == foc_ctrl_mode_current)
           ? app_encoder_get_elec_rad(&s_encoder)
           : s_ctrl.openloop.elec_angle;
}

/* 初始化阶段跑 n 个控制周期 (1kHz), 走的是与 app_run 完全一样的控制路径:
 *   读编码器 -> 采电流(按实测映射+极性) -> 控制器 -> 写占空比
 * 开环四角度探测和闭环方向验证都用它。 */
static void app_init_run_cycles(uint32_t n, float vbus)
{
    uint32_t i;

    for (i = 0U; i < n; i++) {
        int32_t im[3] = {0, 0, 0};
        float   ia, ib, ic, du, dv, dw;

        (void)bsp_mt6701_poll();
        (void)app_encoder_update(&s_encoder);

        bsp_adc_get_currents_ma(&im[0], &im[1], &im[2]);
        ia = (float)im[s_phase_map[0]] * 0.001f * s_cur_sign;
        ib = (float)im[s_phase_map[1]] * 0.001f * s_cur_sign;
        ic = (float)im[s_phase_map[2]] * 0.001f * s_cur_sign;

        foc_controller_run(&s_ctrl, ia, ib, ic,
                           app_encoder_get_elec_rad(&s_encoder), vbus);
        foc_controller_get_duty(&s_ctrl, &du, &dv, &dw);
        bsp_pwm_set_duty_uvw(du, dv, dw);
        app_delay_ms(1U);
    }
}

#if APP_FOC_CLOSED_LOOP_ENABLE
/* 闭环过流保护标志: 一旦触发就永久关断输出, 必须复位重跑才恢复 */
static uint8_t s_oc_tripped;

/* 只用比较实现, 避免依赖 math.h / libm */
static float app_abs_f(float v)
{
    return (v < 0.0f) ? -v : v;
}
#endif

/* ================= 日志后端 ================= */

/* ---- 控制台: 非阻塞环形缓冲 + UART 发送中断 ----
 *
 * 【为什么必须改】原先直接用 printf -> SDK 的 console_send, 那是"一个字符一个字符
 *   轮询等待"的阻塞写法。115200 baud 只有 11.5 字符/ms:
 *       一行 185 字符的 [ENC] health  ->  主循环被卡住约 16ms
 *       [FOC] 日志(约80字符) x2/秒    ->  每次约 7ms
 *   于是每秒都有一次十几毫秒的停顿: 编码器少采一拍、转速限幅反应变慢, 而且
 *   (在角度自走之前)电流矢量指错方向 -> 电机顿挫。实测现象就是"打印一大串数据
 *   时电机突然顿一下"。
 *
 * 【现在】日志只做"往环形缓冲里塞"(塞不下就整行丢弃, 绝不等待), 由 UART 的
 *   TX FIFO 可用中断慢慢喂硬件。对照 STM32: 相当于 __HAL_UART_ENABLE_IT(UART_IT_TXE)
 *   或 HAL_UART_Transmit_DMA(), 只是这里靠 FIFO 深度轮询推进。
 *   中断优先级设 3 —— 比电流环的 ADC 中断(优先级 1)低, 保证电流环永远优先响应。
 *
 * 环形缓冲是单生产者(主循环)/单消费者(中断)模型, 只在两端各自推进读写指针,
 * 32 位对齐访问在 RV32 上是原子的, 不需要额外加锁。 */
#define APP_CON_LOG_BUF_SZ   (4096U)                 /* 必须是 2 的幂 */
#define APP_CON_LOG_MASK     (APP_CON_LOG_BUF_SZ - 1U)
#define APP_CON_UART_IRQ_PRIO (3U)

static volatile uint8_t  s_con_buf[APP_CON_LOG_BUF_SZ];
static volatile uint32_t s_con_head;                 /* 生产: 主循环写 */
static volatile uint32_t s_con_tail;                 /* 消费: 中断读 */

SDK_DECLARE_EXT_ISR_M(BOARD_CONSOLE_UART_IRQ, app_console_uart_isr)

/* 中断上下文: 尽量把环形缓冲喂进 TX FIFO; 喂空了就关掉这个中断 */
void app_console_uart_isr(void)
{
    UART_Type *base = BOARD_CONSOLE_UART_BASE;

    while (s_con_tail != s_con_head) {
        if (uart_send_byte(base, s_con_buf[s_con_tail]) != status_success) {
            return;                                  /* FIFO 满: 等下一次中断, 不等待 */
        }
        s_con_tail = (s_con_tail + 1U) & APP_CON_LOG_MASK;
    }
    uart_disable_irq(base, uart_intr_tx_slot_avail);
}

/**
 * @brief  非阻塞控制台输出 (供项目日志和 SDK 的 printf/_write 共用)
 *
 * @note   SDK 的 hpm_debug_console.c 里对它有【弱定义】(原来的阻塞实现),
 *         这里的强定义会覆盖它 —— 于是 printf 也走这条非阻塞路径,
 *         既不会阻塞主循环, 也不会和环形缓冲抢同一个 UART 造成字符交错。
 *         返回实际入队的字节数; 缓冲满时只入队一部分(丢弃剩余), 绝不等待。
 */
int bsp_console_write(const char *data, int size)
{
    UART_Type *base = BOARD_CONSOLE_UART_BASE;
    uint32_t   head = s_con_head;
    uint32_t   used = (head - s_con_tail) & APP_CON_LOG_MASK;
    uint32_t   room = APP_CON_LOG_BUF_SZ - used - 1U;
    int        n    = 0;

    while ((n < size) && (data != NULL)) {
        char     c    = data[n];
        uint32_t need = (c == '\n') ? 2U : 1U;      /* '\n' 自动补 '\r' */

        if (room < need) {
            break;                                   /* 满了: 丢弃剩余, 绝不等硬件 */
        }
        if (c == '\n') {
            s_con_buf[head] = (uint8_t)'\r';
            head = (head + 1U) & APP_CON_LOG_MASK;
        }
        s_con_buf[head] = (uint8_t)c;
        head = (head + 1U) & APP_CON_LOG_MASK;
        room -= need;
        n++;
    }
    s_con_head = head;

    uart_enable_irq(base, uart_intr_tx_slot_avail);   /* 让中断把数据推出去 */
    return n;
}

/* 线程上下文: 交给非阻塞路径 */
static void log_backend_console(const char *line)
{
    diag_set_stage(APP_STAGE_LOOP_LOG);   /* 给中断诊断标记: 正在打日志 */
    (void)bsp_console_write(line, (int)strlen(line));
    (void)bsp_console_write("\n", 1);
}

/* 控制台输出初始化: 使能 TX 中断 (优先级低于电流环) */
static void app_console_init(void)
{
    uart_disable_irq(BOARD_CONSOLE_UART_BASE, uart_intr_tx_slot_avail);
    intc_m_enable_irq_with_priority(BOARD_CONSOLE_UART_IRQ, APP_CON_UART_IRQ_PRIO);
}

/* USB 虚拟串口后端 (仅主机已连接时输出) */
static void log_backend_usb(const char *line)
{
    if (bsp_usb_connected()) {
        diag_set_stage(APP_STAGE_LOOP_USB);   /* 给中断诊断标记: 正在写 USB */
        (void)bsp_usb_write((const uint8_t *)line, (uint32_t)strlen(line));
        (void)bsp_usb_write((const uint8_t *)"\r\n", 2);
    }
}

/* ================= 中断回调 (由 BSP 在中断上下文调用) ================= */

#if APP_CURRENT_LOOP_IN_ISR
/* 【物理可及限幅】一个采样周期内相电流最多能变多少?
 *     Δi_max = Vmax × ts / Ls
 *   12V 母线时 Vmax ≈ 6.9V, ts = 1/20kHz, Ls = 1.3mH -> 约 0.27A。
 * 实测出现过 10A 的采样值 —— 而 12V + 3.1Ω 下即使相间短路也不过 1.9A,
 * 那个值物理上不可能, 是开关瞬间把运放打飞了。这种假值直接夹掉,
 * 否则它会立刻触发过流锁死(实测就是这么停的)。 */
static void app_current_rate_limit(float *ia, float *ib, float *ic)
{
    static float pia, pib, pic;
    const float lim = ((APP_FOC_VMAX_V / FOC_MOTOR_LS_HENRY) / (float)APP_ADC_ISR_FREQ_HZ);
    float a = *ia, b = *ib, c = *ic;

    if (a > (pia + lim)) { a = pia + lim; } else if (a < (pia - lim)) { a = pia - lim; }
    if (b > (pib + lim)) { b = pib + lim; } else if (b < (pib - lim)) { b = pib - lim; }
    if (c > (pic + lim)) { c = pic + lim; } else if (c < (pic - lim)) { c = pic - lim; }

    pia = a; pib = b; pic = c;
    *ia = a; *ib = b; *ic = c;
}
#endif

/* ADC 采样完成中断 (20kHz, 由 PWM 触发):
 *   取相电流(按实测相序/极性) -> 物理限幅 -> 刷新诊断;
 *   若电流环跑在 ISR 里, 紧接着算 PI 并更新 PWM ——
 *   于是"采样 -> 计算 -> 更新占空比"严格发生在同一个载波周期内, 没有主循环延迟。 */
static void app_on_adc_conv(void)
{
    int32_t im[3] = {0, 0, 0};
    float   ia, ib, ic;

    bsp_adc_get_currents_ma(&im[0], &im[1], &im[2]);
    s_isr_cnt++;
    /* 先按实测映射重排相序, 再统一到 FOC 的相电流正方向 */
    ia = (float)im[s_phase_map[0]] * 0.001f * s_cur_sign;
    ib = (float)im[s_phase_map[1]] * 0.001f * s_cur_sign;
    ic = (float)im[s_phase_map[2]] * 0.001f * s_cur_sign;

#if APP_CURRENT_LOOP_IN_ISR
    app_current_rate_limit(&ia, &ib, &ic);
#endif

    s_iu_ma = (int32_t)(ia * 1000.0f);
    s_iv_ma = (int32_t)(ib * 1000.0f);
    s_iw_ma = (int32_t)(ic * 1000.0f);
    diag_set_currents_ma(s_iu_ma, s_iv_ma, s_iw_ma);

#if APP_CURRENT_LOOP_IN_ISR
    if (s_isr_closed_loop != 0U) {
        float    du, dv, dw;
        float    ang;
        uint64_t now = hpm_csr_get_core_mcycle();
        float    dt;

        /* 【角度自走 + 按实际间隔推进】
         *   1) 主循环会被日志/USB 阻塞几十毫秒, 所以角度必须在中断里自己外推;
         *   2) 更关键的是【推进量用 mcycle 实测的实际间隔】, 不是固定 50us ——
         *      中断偶尔被别的临界区拖长(实测见过 15ms)时, 固定步长等于漏掉
         *      15ms 的转角: 566RPM 下电频率 104Hz, 15ms = 1.5 个电周期,
         *      恢复瞬间矢量方向完全错, 要好几拍才追回来 —— 表现就是"顿一下"。
         *      用实际间隔, 中断一恢复角度立刻就是对的。 */
        if (s_isr_last_mcycle != 0U) {
            uint32_t d = (uint32_t)(now - s_isr_last_mcycle);

            if (d > s_isr_dt_max) {
                s_isr_dt_max = d;
            }
            /* 间隙 > 200us 就记下"当时主循环在哪": 这就是堵住中断的嫌疑人 */
            if (d > (uint32_t)(hpm_core_clock / 5000U)) {
                s_isr_gap_stage = g_diag.stage;
                s_isr_gap_us    = (uint32_t)(((uint64_t)d * 1000000U)
                                             / (uint64_t)hpm_core_clock);
            }
            dt = (float)d / (float)hpm_core_clock;
            if (dt > 0.05f) {
                dt = 0.05f;                         /* 上限 50ms: 防调试器暂停等异常 */
            }
        } else {
            dt = 1.0f / (float)APP_ADC_ISR_FREQ_HZ;
        }
        s_isr_last_mcycle = now;

        ang = s_isr_elec_rad + (s_isr_elec_speed * dt);
        if (ang >= FOC_TWO_PI) {
            ang -= FOC_TWO_PI;
        } else if (ang < 0.0f) {
            ang += FOC_TWO_PI;
        }
        s_isr_elec_rad = ang;

        foc_controller_run(&s_ctrl, ia, ib, ic, ang, s_isr_vbus);
        foc_controller_get_duty(&s_ctrl, &du, &dv, &dw);
        bsp_pwm_set_duty_uvw(du, dv, dw);
    }
#endif
}

/* MT6701 新帧到达 (SPI 后端在主循环里 poll, 属线程上下文, 不是中断) */
static void app_on_mt6701_frame(const mt6701_frame_t *frame, uint32_t raw)
{
    (void)frame;
    (void)raw;
    /* 编码器诊断在主循环 poll 后统一刷新 (见 app_run), 这里预留
     * 给未来需要在"新帧到达"时刻做额外处理的逻辑。 */
}

/* USB 主机下发数据 */
static void app_on_usb_rx(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
    /* 预留: 可在此解析主机命令 (启停/调速等) */
}

/* ================= 初始化 ================= */
void app_init(void)
{
    float dt = (float)APP_MAIN_LOOP_MS * 0.001f;

    /* 诊断状态清零 */
    memset((void *)&g_diag, 0, sizeof(g_diag));

    /* 日志: 注册 console + USB 两个后端。
     * 控制台走"环形缓冲 + TX 中断"的非阻塞路径, 先把它初始化好, 否则第一行
     * 日志之后每次打印都会阻塞主循环(详见 log_backend_console 上面的说明)。 */
    app_console_init();
    app_log_add_backend(log_backend_console);
    app_log_add_backend(log_backend_usb);

    LOG_I("APP", "YeeZi HPM6EMotor open-loop demo");

    /* 外设初始化顺序: PWM -> ADC -> USB.
     * PWM 先初始化很重要 —— ADC 触发源来自 PWM TRGO */
    bsp_pwm_init();
    diag_set_stage(APP_STAGE_PWM_INIT);
    bsp_adc_init();
    diag_set_stage(APP_STAGE_ADC_INIT);
    bsp_usb_init();
    diag_set_stage(APP_STAGE_USB_INIT);

    /* MT6701 编码器初始化 */
    bsp_mt6701_register_frame_cb(app_on_mt6701_frame);
    bsp_mt6701_init();
    diag_set_stage(APP_STAGE_SEI_INIT);
#if BSP_MT6701_USE_SEI
    {
        bsp_sei_diag_t sd;
        bsp_sei_get_diag(&sd);
        LOG_I("ENC", "MT6701 SEI SSI Master: SEI_CTRL_%u CK=PE19 RX=PE17",
              (unsigned)sd.ctrl_index);
        LOG_I("SEI", "clk=%uHz baud_div=%u ck0=%u ck1=%u period=%u irq_en=0x%08x",
              (unsigned)sd.clk_freq, (unsigned)sd.baud_div,
              (unsigned)sd.ck0_point, (unsigned)sd.ck1_point,
              (unsigned)sd.trig_period, (unsigned)sd.irq_enable);
    }
#else
    LOG_I("ENC", "MT6701 SPI2 @ J11: SCLK=PC26(pin31) MISO=PC28(pin10) CS=PC24(pin11)");
    /* 时钟/分频诊断: 确认源时钟、实际 SCLK、以及时序初始化是否成功
     * (timing=0 表示 status_success) */
    LOG_I("ENC", "SPI2 src_clk=%u Hz sclk=%u Hz timing=%d",
          (unsigned)bsp_spi_get_src_clk(),
          (unsigned)bsp_spi_get_sclk(),
          bsp_spi_get_timing_stat());
#endif

#if !BSP_MT6701_USE_SEI
    /* ---- 编码器接线 bit-bang 排查 ----
     * 用 GPIO 软件模拟 SPI, 把 16 种 CPOL/CPHA/CS方式/位序 组合全试一遍。
     * 目的: 把"SPI2 外设"和"模式配对"这两个变量一起排除掉。
     *   - 有组合读到变化数据  -> 编码器是活的, 只是模式/接线不对;
     *   - 16 种全部恒定不变   -> DOUT 上没有任何器件在响应, 纯硬件问题。 */
    bsp_bb_scan();
    diag_set_bb(bsp_bb_hit_count(), bsp_bb_crc_hit_count(),
                bsp_bb_best_index(), bsp_bb_best_raw(),
                (uint32_t)bsp_bb_static_level());

    LOG_I("ENC", "bitbang: MISO static=%u, hit=%u/16, crc_hit=%u/16, best=%d",
          (unsigned)bsp_bb_static_level(),
          (unsigned)bsp_bb_hit_count(),
          (unsigned)bsp_bb_crc_hit_count(),
          (int)bsp_bb_best_index());

    /* 把 CRC 通过 / 读到变化数据的组合逐条打出来, 便于复盘 */
    {
        char line[192];
        uint32_t i;

        for (i = 0U; i < BSP_BB_MODE_COUNT; i++) {
            const bsp_bb_result_t *r = bsp_bb_result_at(i);

            if ((r != NULL) && ((r->crc_ok > 0U) || (r->distinct >= 2U))) {
                (void)bsp_bb_format(line, sizeof(line), i);
                LOG_I("ENC", "%s", line);
            }
        }
    }

    /* 只要扫描找到了 CRC 可信的组合, 就直接启用它。
     * 实测硬件 SPI2 这条路 (即使已改成 Mode 0) 仍然读不出数据, 而 bit-bang
     * 已被 CRC6 证明能读出 100% 合法的帧 —— 先用能工作的这条把编码器跑起来。
     * idx=8 => CS 每帧翻转 + CPOL=0 + CPHA=0 + MSB first。 */
    if (bsp_bb_best_index() >= 0) {
        (void)bsp_bb_activate((uint32_t)bsp_bb_best_index());
        LOG_I("ENC", "encoder reader -> bitbang idx=%d (CRC-validated)",
              (int)bsp_bb_best_index());
    }
    /* 【BUILD 标记】日志里一眼看出跑的是哪一版, 免得再来回猜 */
    LOG_I("ENC", "BUILD: sweep=%d presetttle=%u mincnt=%u kick_ms=%u",
          (int)APP_CAL_SWEEP_ENABLE, (unsigned)APP_CAL_SWEEP_PRESETTLE,
          (unsigned)ENC_LUT_MIN_CNT, (unsigned)APP_ENC_STUCK_KICK_MS);
    {
    }
#endif

    /* 统一 FOC 控制器: 默认开环模式 */
    foc_controller_init(&s_ctrl, dt);
    foc_controller_set_voltage(&s_ctrl, APP_OPENLOOP_VOLTAGE_V);
    foc_controller_set_current_limit(&s_ctrl, APP_OPENLOOP_CURRENT_LIMIT);

    /* 转速给定复位。
     * 【注意】这里不能调 app_speed_target_set(): 它会顺手把开环频率给定也改掉,
     * 而本函数后面那几段 (#elif) 才决定最终的工作点 (开环 V/F / 静态扫描 / 查表
     * 换向), 顺序反了会把它们的设定覆盖掉。只复位目标值, 下发交给 app_speed_apply()。
     *
     * 【为什么用位置环时初值必须是 0, 而不是 600】
     *   速度环在 app_init 里就已经使能了, 而位置环要等到本函数后面才 enable()。
     *   如果这里留 600, 上电到"位置环第一次跑起来"之间, 速度给定就是 600 RPM ——
     *   这段时间短, 但足以让速度环把 iq 顶上去、电机抽一下; 更糟的是如果位置环
     *   那次 enable 失败(参数/编码器异常), 电机就一直追 600。默认给 0 才是
     *   "谁都没让它动, 它就不动"。 */
#if APP_MOTION_ENABLE
    s_speed_target_rpm = 0.0f;
#else
    s_speed_target_rpm = APP_SPEED_TARGET_DEFAULT_RPM;
#endif
    LOG_I("APP", "speed setpoint: target=%d RPM, ramp=%d RPM/s, speed_loop=%d, motion_loop=%d",
          (int)s_speed_target_rpm, (int)APP_SPEED_RAMP_RPM_S,
          (int)APP_FOC_SPEED_LOOP_ENABLE, (int)APP_MOTION_ENABLE);


    /* ---- 非易失校准数据 (Flash) ----
     * 有合法存档就直接查表换向, 跳过 3 分钟静态扫描。
     * 【不调 app_nv_init()】上电时 CPU 正在这片 NOR 上取指, 任何 XPI 重配置
     * 都会把启动流程卡死 (症状: BUILD 行之后无输出)。读档是纯 XIP 直读,
     * 不需要配置; 只有"写存档"时才在 RAM 里懒加载配置。
     *
     * 【用内置离线表时不读档】读档会把电压改成 APP_ENC_LUT_RUN_VOLTAGE_V 并把
     * s_lut_full 置 1 (=准备交给运行时 diag_lut), 而这时候角度已经由内置表给了,
     * 那份存档只是多余的。 */
#if !APP_ENC_USE_BUILTIN_LUT
    {
        static int16_t nv_tbl[ENC_LUT_BINS];
        int32_t        nv_off = 0;

        if (app_nv_load(nv_tbl, &nv_off)) {
            diag_lut_load(nv_tbl);
            app_encoder_set_elec_offset_rad((float)nv_off * 0.001f);
            s_lut_full   = 1U;
            s_sweep_done = 1U;
            foc_controller_set_voltage(&s_ctrl, APP_ENC_LUT_RUN_VOLTAGE_V);
            foc_controller_set_current_limit(&s_ctrl, APP_ENC_LUT_RUN_CURRENT_LIMIT_A);
            LOG_I("ENC", "NV: loaded LUT + offset from flash (offset=%ld mrad) -> LUT commutation, skip cal",
                  (long)nv_off);
        } else {
            LOG_I("ENC", "NV: no valid table in flash -> run calibration");
        }
    }
#endif /* !APP_ENC_USE_BUILTIN_LUT */

    s_ctrl.openloop.elec_accel = APP_OPENLOOP_ACCEL;
#if APP_FOC_CLOSED_LOOP_ENABLE
    /* ---- 闭环: 用编码器角度换向 ----
     * 电角度不再来自内部积分, 而是 app_encoder_get_elec_rad()
     * = normalize(mech×POLE_PAIRS + FOC_ELEC_ANGLE_OFFSET)。
     * 这是 FOC_ELEC_ANGLE_OFFSET 第一次真正参与控制。 */
    foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_current);
    foc_controller_set_current_ref(&s_ctrl, APP_FOC_ID_REF_A, APP_FOC_IQ_REF_A);
    foc_controller_set_gains(&s_ctrl, APP_FOC_CURRENT_KP, APP_FOC_CURRENT_KI);
    LOG_I("FOC", "CLOSED LOOP on: id=%d mA iq=%d mA kp=%d ki=%d rpm_limit=%d",
          (int)(APP_FOC_ID_REF_A * 1000.0f), (int)(APP_FOC_IQ_REF_A * 1000.0f),
          (int)APP_FOC_CURRENT_KP, (int)APP_FOC_CURRENT_KI,
          (int)APP_FOC_RPM_LIMIT);
#else
#  if APP_ENC_USE_BUILTIN_LUT
    /* ---- 内置离线查表换向 ----
     * 电角度由 app_encoder 查表给出 (见 app_encoder.c), app_run 每周期把
     * "表值 + 超前角"写进开环电压矢量。这里只需要把工作点设好:
     *   - 电压用 APP_ENC_LUT_RUN_VOLTAGE_V (换向后转子跟着矢量走, 转速由 V/Ke 自限);
     *   - 电流限幅用 APP_ENC_LUT_RUN_CURRENT_LIMIT_A (转矩 ≈ Kt×I, 手能不能按住看它);
     *   - 不设速度 —— 内部积分被 set_external_angle 停掉了, 不需要它。 */
    foc_controller_set_voltage(&s_ctrl, APP_ENC_LUT_RUN_VOLTAGE_V);
    foc_controller_set_current_limit(&s_ctrl, APP_ENC_LUT_RUN_CURRENT_LIMIT_A);
    LOG_I("FOC", "BUILTIN LUT commutation: %u bins (%.2f counts/bin), V=%u mV, Ilim=%u mA, lead=%d deg",
          (unsigned)APP_ENC_LUT_BINS, (float)16384.0f / (float)APP_ENC_LUT_BINS,
          (unsigned)(APP_ENC_LUT_RUN_VOLTAGE_V * 1000.0f),
          (unsigned)(APP_ENC_LUT_RUN_CURRENT_LIMIT_A * 1000.0f),
          (int)APP_ENC_LEAD_DEG);
#  elif APP_CAL_SWEEP_ENABLE
    if (s_lut_full == 0U) {
    /* 静态扫描标定: 磁场从 0° 开始分步固定, 由 app_run 推进 */
    /* 这里算的是【步数上限】: 正常会在"净行程走满一整圈"时提前收尾 (见 app_run
     * 里的里程计)。给 1.5 倍余量, 极对数比配置值大 (实测配 12 时 12 个电周期
     * 只走了 0.95 圈) 或中途丢过步也能扫完整圈。 */
    /* 【步数上限从 ×1.5 提到 ×2.5】(3600 -> 6000 步)
     * 真正收尾靠下面的净行程里程计 (odo 满 16384 = 走满整圈), 这里只是兜底上限。
     * 原来只给 1.5 倍余量: 弱场下转子打滑时 (实测每电周期只走 1026/640 counts,
     * 理想 1365), 3600 步只覆盖约 91% 的圈 -> 缺口 > 16 箱 -> 直接被否掉。
     * 给到 2.5 倍后即使按 3 counts/步 的严重打滑也能走完 (6000×3 = 18000 > 16384),
     * 而正常跟得住时里程计会在 ~2400 步就收尾, 时间不变。 */
    s_sweep_steps = APP_CAL_SWEEP_PHASES
                    * ((uint32_t)FOC_MOTOR_POLE_PAIRS * 5U / 2U)
                    * APP_CAL_SWEEP_REVS;
    if (s_sweep_steps < APP_CAL_SWEEP_PHASES) {
        s_sweep_steps = APP_CAL_SWEEP_PHASES;
    }
    s_ctrl.openloop.voltage_slew = APP_CAL_SWEEP_SLEW;   /* 扫描期间电压要能快速切换 */
    /* 【拉入】先用 APP_OPENLOOP_VOLTAGE_V(1.5V) 把转子从上电位置可靠拽到 0°,
     * 预吸稳那 1.5s 里再经斜坡降到标定电压 0.65V。一上来就用 0.65V 有风险:
     * 转子若停在齿槽谷底可能拽不动, 整场扫描直接废掉。1.5V 只维持 1.5 秒,
     * 发热可以忽略。 */
    foc_controller_set_voltage(&s_ctrl, APP_OPENLOOP_VOLTAGE_V);
    foc_controller_set_align_angle(&s_ctrl, 0.0f);
    /* 静态每箱只有一个吸稳+外推样本, 有效门限用 3 会把整圈都扫完的表误拒
     * (实测 missed=664 maxhole=43 -> REJECTED)。降到 1: 实测缺口最长 11 箱,
     * 弦插值在本地线性的表上完全够用。 */
    diag_lut_set_min_cnt(1U);
    LOG_I("ENC", "CAL SWEEP start: <=%u steps x %u loops, V=%d/%d mV, mincnt=1 (走到净行程满整圈为止)",
          (unsigned)s_sweep_steps, (unsigned)APP_CAL_SWEEP_HOLD,
          (int)(APP_CAL_SWEEP_HIGH_V * 1000.0f),
          (int)(APP_CAL_SWEEP_LOW_V * 1000.0f));
    }
#  elif APP_OPENLOOP_ALIGN_ONLY
    foc_controller_set_align_angle(&s_ctrl, APP_OPENLOOP_ALIGN_DEG * (FOC_TWO_PI / 360.0f));
#  elif APP_ENC_ANGLE_DRIVE
    /* 编码器换向: 角度由 app_run 每周期写入 (enc_elec + 超前角),
     * 所以这里【不设开环速度】—— 内部积分保持不动。 */
    LOG_I("FOC", "ENCODER COMMUTATION on: lead=%d deg, V=%u mV",
          (int)APP_ENC_LEAD_DEG, (unsigned)(APP_OPENLOOP_VOLTAGE_V * 1000.0f));
#  else
    foc_controller_set_speed_rpm(&s_ctrl, APP_OPENLOOP_TARGET_RPM);
#  endif
#endif
    /* 【注意】控制器不在这里使能 —— 必须先做电流零点校准(甚至极性自检),
     * 之后才 bsp_pwm_enable_output(true) + foc_controller_enable(true)。
     * 见本函数末尾的"初始校准 -> 使能电机"两段。 */
    diag_set_stage(APP_STAGE_FOC_INIT);

    /* 编码器封装: 速度估算低通系数 0.1 */
    app_encoder_init(&s_encoder, dt, 0.1f);

    /* 注册中断回调: 硬件中断留在 BSP, 应用逻辑经此上抛
     * (编码器回调已在上方 bsp_mt6701_init() 之前注册) */
    bsp_adc_register_conv_cb(app_on_adc_conv);
    bsp_usb_register_rx_cb(app_on_usb_rx);

    /* ================= 第一步: 初始电流校准 (必须在使能电机之前) =================
     *
     * 【顺序为什么不能反】校准采的是"相电流为 0 时运放输出是多少"。
     *   先使能电机 -> 相电流不为 0 -> 零点里混进真实电流 -> 之后整个电流环都在
     *   一条偏掉的基线上工作 (d/q 电流恒有偏置 -> 转矩脉动, 严重时环路自己跑飞)。
     *
     * 【为什么零矢量就够】三相占空比都写 50% = 三相下桥同时导通、相间电压为 0,
     *   转子静止时相电流必然是 0 (没有反电势就没有环流)。这样初始化只走"设置
     *   占空比"这条老路径, 不必去动 PWM 的软件强制 —— 而"封锁后恢复"这条路
     *   原代码是恢复不回来的 (见 bsp_pwm_enable_output 里的说明)。
     *   代价: 若上电时转子还在转(手拨过), 零矢量会把绕组短接产生制动电流,
     *   所以务必在电机静止时上电。 */
    bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);      /* 零矢量 */
    bsp_pwm_enable_output(true);                 /* 打开输出 (零矢量下相电流仍为 0) */
    diag_set_stage(APP_STAGE_PWM_ENABLE);
#if APP_ADC_OFFSET_CAL_ENABLE
    bsp_adc_calibrate();                         /* 等 256 个新样本求平均 (约 12.8ms @20kHz) */
#endif
    {
        uint16_t uo = 0U;
        uint16_t vo = 0U;
        const int32_t mid = (int32_t)(65535U / 2U);

        bsp_adc_get_offsets(&uo, &vo);
        LOG_I("ADC", "offset cal: u=%u v=%u (mid=%d, 1LSB=0.504mA, FS=+-16.5A)",
              (unsigned)uo, (unsigned)vo, (int)mid);
        LOG_I("ADC", "offset error: u=%d mA v=%d mA",
              (int)bsp_adc_counts_to_ma((int32_t)uo - mid),
              (int)bsp_adc_counts_to_ma((int32_t)vo - mid));
    }

    /* ================= 第二步: 电流传感自检 (可选, 但闭环必须做) =================
     *
     * 【为什么不是只测一个角度】
     *   只把矢量钉在 0°(U 轴) 只能定"整体极性": 那个角度下 V/W 两相电流本来就相等,
     *   所以"V/W 通道互换"这类错位在那个角度【完全看不出来】。但这种错位在数学上
     *   是一次"镜像"而不是旋转 —— 闭环的电流反馈会因此变成按 2θ 调制的坏反馈:
     *   PI 永远等不到误差收敛 -> 积分一路顶到电压限幅 -> 电流冲到 Vmax/R -> 过流锁死。
     *   (现象正是"转一下就停"。)
     *
     *   所以这里在 0/90/180/270° 四个角度各钉一次, 记录三相电流, 然后穷举
     *   【6 种相序排列 × 2 种极性】, 用"测得的电流矢量是否与施加的电压矢量同向"打分,
     *   取最高分作为电流传感的正确接法 —— 用测量代替假设。 */
#if APP_CURRENT_SIGN_AUTOTEST
    {
        /* 四个探测角度与对应的施加电压单位矢量 (cos, sin), 不引入三角函数 */
        static const float   ang[4]     = {0.0f, 1.5707963f, 3.1415927f, 4.7123890f};
        static const float   vdir[4][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}, {0.0f, -1.0f}};
        float    pat[4][3];
        float    vbus_f = (float)bsp_adc_get_vbus_mv() * 0.001f;
        float    vtest  = (float)APP_CURRENT_SIGN_TEST_MV * 0.001f;
        uint32_t n = (APP_CURRENT_SIGN_TEST_MS > 0U) ? APP_CURRENT_SIGN_TEST_MS : 1U;
        uint32_t k;

        s_ctrl.openloop.voltage_slew = 100.0f;   /* 默认 1V/s 爬 500mV 要 0.5s, 太慢 */
        foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_openloop);
        foc_controller_set_voltage(&s_ctrl, vtest);
        foc_controller_enable(&s_ctrl, true);

        for (k = 0U; k < 4U; k++) {
            uint32_t i;
            uint32_t nsum = 0U;
            float    sa = 0.0f, sb = 0.0f, sc = 0.0f;

            foc_controller_set_align_angle(&s_ctrl, ang[k]);
            for (i = 0U; i < n; i++) {
                int32_t a = 0, b = 0, c = 0;
                float   ia, ib, ic, du, dv, dw;

                bsp_adc_get_currents_ma(&a, &b, &c);
                ia = (float)a * 0.001f;
                ib = (float)b * 0.001f;
                ic = (float)c * 0.001f;

                if (i >= (n / 2U)) {        /* 前半段等电压爬到位/电流稳定 */
                    sa += ia; sb += ib; sc += ic; nsum++;
                }

                foc_controller_run(&s_ctrl, ia, ib, ic, 0.0f, vbus_f);
                foc_controller_get_duty(&s_ctrl, &du, &dv, &dw);
                bsp_pwm_set_duty_uvw(du, dv, dw);
                app_delay_ms(1U);
            }
            if (nsum == 0U) { nsum = 1U; }
            pat[k][0] = sa / (float)nsum;
            pat[k][1] = sb / (float)nsum;
            pat[k][2] = sc / (float)nsum;

            LOG_I("ADC", "probe %3d deg @%d mV: ia=%d ib=%d ic=%d mA",
                  (int)(ang[k] * 57.2957795f), (int)APP_CURRENT_SIGN_TEST_MV,
                  (int)(pat[k][0] * 1000.0f), (int)(pat[k][1] * 1000.0f),
                  (int)(pat[k][2] * 1000.0f));
        }

        /* 自检做完: 关控制器、矢量归零、恢复电压斜坡; 输出一直开着 */
        foc_controller_enable(&s_ctrl, false);
        bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
        s_ctrl.openloop.voltage_slew = FOC_OPENLOOP_VOLTAGE_SLEW;

        /* ---- 打分: 6 种排列 × 2 种极性 ---- */
        {
            static const uint8_t perm[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                                               {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
            float best = -1.0e9f;
            int   best_p = 0;
            int   best_s = 1;
            int   p;
            int   s;

            for (p = 0; p < 6; p++) {
                for (s = -1; s <= 1; s += 2) {
                    float score = 0.0f;

                    for (k = 0U; k < 4U; k++) {
                        float ia = (float)s * pat[k][perm[p][0]];
                        float ib = (float)s * pat[k][perm[p][1]];
                        float ic = (float)s * pat[k][perm[p][2]];
                        float ialpha = ia;
                        float ibeta  = (ib - ic) * 0.5773503f;

                        /* 电流矢量在施加电压方向上的投影 (四个角度电压幅值相同,
                         * 所以不必归一化, 直接比投影大小即可) */
                        score += ialpha * vdir[k][0] + ibeta * vdir[k][1];
                    }
                    if (score > best) {
                        best   = score;
                        best_p = p;
                        best_s = s;
                    }
                }
            }

            if (APP_CURRENT_SIGN != 0.0f) {
                best_s = (APP_CURRENT_SIGN > 0.0f) ? 1 : -1;
            }

            s_cur_sign    = (float)best_s;
            s_phase_map[0] = perm[best_p][0];
            s_phase_map[1] = perm[best_p][1];
            s_phase_map[2] = perm[best_p][2];

            LOG_I("ADC", "=> sign=%d, ia/ib/ic <- adc[%d/%d/%d], score=%d mA (>0 表示电流跟着电压走)",
                  best_s, (int)s_phase_map[0], (int)s_phase_map[1], (int)s_phase_map[2],
                  (int)(best * 250.0f));
            if ((best_p != 0) || (best_s != 1)) {
                LOG_I("ADC", "!! 注意: 电流传感接法与默认不同 (排列 %d, 极性 %d) —— 这是实测结果, 已自动采用",
                      best_p, best_s);
            }
        }
    }
#endif

    /* ================= 第二步之一: ADC 采样点定标 =================
     *
     * 【为什么必须管这个】相电流只能在"三相下桥同时导通"的窗口里采, 而且要离两侧
     *   开关沿越远越好 —— 否则运放正被开关瞬变打着, 采到的是垃圾(实测出现过 10A,
     *   而 12V+3.1Ω 下即使相间短路也只有 1.9A, 物理上不可能)。
     *
     *   本 PWM 的比较值是对称构造的 (cmp = reload/2 ∓ cmp_half, 见 bsp_pwm.c),
     *   所以下桥导通窗口以计数器回绕点为中心, 半宽 = reload/2×(1-0.98×duty_max):
     *       duty=0.5 -> 12.8us    duty=0.9 -> 2.95us    duty=0.98 -> 1.0us(再扣死区)
     *   调制越深窗口越窄 —— 电流环顶限幅时(实测正是那时出事)采样点就可能踩在沿上。
     *
     * 【做法: 先让转子吸稳, 再在回绕点附近细扫】
     *   候选点只在回绕点 ±APP_ADC_TRIG_WIN counts 内取 —— 因为深调制时上面的窗口
     *   会收缩到几百 counts, 采样点必须贴着回绕点; 让它到周期别处去"找安静的地方"
     *   是错的(浅调制下那里确实也安静, 但调制一加深就掉出窗口, 又会采到垃圾)。
     *
     *   【上一版的教训】第一版是整个周期粗扫, 结果抖动从 195mA 单调降到 96mA,
     *   看上去"周期末尾最干净", 其实是【转子还在摆动】—— 每个点只等 19ms, 对齐
     *   暂态根本没结束。所以现在先空跑 400ms 让转子吸稳, 测到的才是采样时刻的噪声。 */
#if APP_ADC_TRIG_SCAN
    {
        const uint32_t reload = bsp_pwm_get_reload();
        /* 10 个候选: 0/100/…/400 与 reload-400/…/reload —— 回绕点两侧都试 */
        const uint32_t ncand  = 2U * (APP_ADC_TRIG_WIN / 100U);
        uint32_t k;
        uint32_t best_cmp = 1U;
        uint32_t best_pp  = 0xFFFFFFFFU;
        float    vbus_f = (float)bsp_adc_get_vbus_mv() * 0.001f;
        float    vtest  = (float)APP_CURRENT_SIGN_TEST_MV * 0.001f;

        s_ctrl.openloop.voltage_slew = 100.0f;
        foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_openloop);
        foc_controller_set_align_angle(&s_ctrl, 0.0f);      /* 矢量钉在 U 轴 */
        foc_controller_set_voltage(&s_ctrl, vtest);
        foc_controller_enable(&s_ctrl, true);

        app_init_run_cycles(400U, vbus_f);                  /* 等转子吸稳, 见上 */

        for (k = 0U; k < ncand; k++) {
            uint32_t cmp = (k < (ncand / 2U)) ? (k * 100U)
                                              : (reload - (ncand - 1U - k) * 100U);
            int32_t  mn[3] = {0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF};
            int32_t  mx[3] = {-0x7FFFFFFF, -0x7FFFFFFF, -0x7FFFFFFF};
            uint32_t pp = 0U;
            uint32_t i;
            int      j;

            bsp_pwm_set_adc_trig_count(cmp);
            app_init_run_cycles(5U, vbus_f);                /* 换点后稳定 */

            for (i = 0U; i < 16U; i++) {
                int32_t im[3] = {0, 0, 0};

                app_init_run_cycles(1U, vbus_f);
                bsp_adc_get_currents_ma(&im[0], &im[1], &im[2]);
                for (j = 0; j < 3; j++) {
                    if (im[j] < mn[j]) { mn[j] = im[j]; }
                    if (im[j] > mx[j]) { mx[j] = im[j]; }
                }
            }
            for (j = 0; j < 3; j++) {
                uint32_t d = (uint32_t)(mx[j] - mn[j]);
                if (d > pp) { pp = d; }
            }
            LOG_I("ADC", "  trig cmp=%5u -> 抖动 %d mA", (unsigned)cmp, (int)pp);
            if (pp < best_pp) {
                best_pp  = pp;
                best_cmp = cmp;
            }
        }

        bsp_pwm_set_adc_trig_count(best_cmp);
        foc_controller_enable(&s_ctrl, false);
        bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
        s_ctrl.openloop.voltage_slew = FOC_OPENLOOP_VOLTAGE_SLEW;

        LOG_I("ADC", "sample-point scan: best cmp=%u (抖动 %d mA), reload=%u",
              (unsigned)best_cmp, (int)best_pp, (unsigned)reload);
    }
#endif

    /* ================= 第二步之二: 闭环方向验证 =================
     *
     * 【为什么还要这一步】上面四角度探测是【开环】做的, 判据只用 iα/iβ, 根本没经过
     *   Park 变换; 而闭环的反馈路径是  电流 → Clarke → Park(表角度) → PI。
     *   如果镜像发生在 Park 这一环 (或表角度/约定有别的问题), 开环探测看不见。
     *
     * 【怎么验】用电流环自己去试方向, 而且做成本质安全的:
     *   只开 P (Ki=0) -> 电压 = Kp×误差, 天然有界, 不可能 windup 到限幅;
     *   给定只有 0.5A -> 即使方向反了, 电流也停在 100mA 量级 (实测见下面注释)。
     *   看测到的 iq 是正还是负: 负 -> 反馈里还有一次镜像 -> 极性取反再来一遍。 */
#if APP_CURRENT_LOOP_VERIFY && APP_FOC_CLOSED_LOOP_ENABLE
    {
        float vbus_f = (float)bsp_adc_get_vbus_mv() * 0.001f;
        int   pass;
        int   flipped = 0;

        foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_current);
        /* 验证用固定 P 增益 (见 app.h), 与正式 Kp 无关 */
        foc_controller_set_gains(&s_ctrl, APP_CURRENT_VERIFY_KP, 0.0f);   /* P-only */
        foc_controller_set_current_ref(&s_ctrl, 0.0f, APP_CURRENT_LOOP_VERIFY_A);

        for (pass = 0; pass < 2; pass++) {
            float    iq_s = 0.0f;
            uint32_t isr_before;
            uint32_t isr_used;
            int      i;

            /* 【采样存活检查】这一步必须在"电流读数可信"的前提下做。
             * 实测教训: 上一步定标把 ADC 触发点设成了死点, 采样冻结, 于是这里
             * 看到的是冻结值, 反而把正确的极性(-1)改成了错误(+1), 电机彻底不转。
             * 所以先记录中断计数, 跑完确认它确实涨了。 */
            isr_before = s_isr_cnt;

            foc_controller_enable(&s_ctrl, true);      /* 会清 PI 状态 */
            app_init_run_cycles(APP_CURRENT_LOOP_VERIFY_MS, vbus_f);
            for (i = 0; i < 8; i++) {                  /* 再取 8 拍平均 */
                app_init_run_cycles(1U, vbus_f);
                iq_s += s_ctrl.foc.iq;
            }
            iq_s /= 8.0f;
            isr_used = s_isr_cnt - isr_before;

            foc_controller_enable(&s_ctrl, false);
            bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);

            LOG_I("ADC", "loop verify pass%d: iq=%d mA (ref %d mA), sign=%d, map=%d/%d/%d, adc_isr=%u",
                  pass, (int)(iq_s * 1000.0f), (int)(APP_CURRENT_LOOP_VERIFY_A * 1000.0f),
                  (int)s_cur_sign, (int)s_phase_map[0], (int)s_phase_map[1],
                  (int)s_phase_map[2], (unsigned)isr_used);

            if (isr_used < (APP_CURRENT_LOOP_VERIFY_MS * 10U)) {
                LOG_I("ADC", "!! ADC 中断只跑了 %u 次 (该有 %u) —— 采样已停, 本次验证结果不可信, 不改极性",
                      (unsigned)isr_used, (unsigned)((APP_CURRENT_LOOP_VERIFY_MS + 8U) * 20U));
                break;
            }

            if (iq_s >= 0.0f) {
                break;                                  /* 方向对了 */
            }
            s_cur_sign = -s_cur_sign;                   /* 反向: 取反重试 */
            flipped    = 1;
        }

        if (flipped != 0) {
            LOG_I("ADC", "!! 闭环方向验证发现反馈反向, 已把极性取反 -> sign=%d",
                  (int)s_cur_sign);
        }

        foc_controller_set_gains(&s_ctrl, APP_FOC_CURRENT_KP, APP_FOC_CURRENT_KI);
        foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_current);
    }
#endif

    /* ================= 第三步: 使能电机 ================= */
    foc_controller_set_mode(&s_ctrl,
                            (APP_FOC_CLOSED_LOOP_ENABLE != 0) ? foc_ctrl_mode_current
                                                              : foc_ctrl_mode_openloop);
    bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
    diag_set_stage(APP_STAGE_PWM_ENABLE);
#if APP_OPENLOOP_DEMO_ENABLE
    foc_controller_enable(&s_ctrl, true);        /* 输出在前面已经打开, 这里只是放开控制器 */
#endif
#if APP_CURRENT_LOOP_IN_ISR
    /* 把控制权交给 20kHz 中断: 积分步长固定成 1/20000, 然后打开中断里的闭环。
     * (自检阶段 s_isr_closed_loop=0, 那时由主循环直接驱动控制器) */
    foc_controller_set_dt(&s_ctrl, 1.0f / (float)APP_ADC_ISR_FREQ_HZ);
    s_isr_closed_loop = 1U;
    LOG_I("FOC", "current loop -> ADC ISR @%u Hz (ts=%u us), ADC trig cmp=%u/%u",
          (unsigned)APP_ADC_ISR_FREQ_HZ, (unsigned)(1000000U / APP_ADC_ISR_FREQ_HZ),
          (unsigned)bsp_pwm_get_adc_trig_count(), (unsigned)bsp_pwm_get_reload());
#endif
#if APP_MOTION_ENABLE
    /* 位置环: 初始化 + 使能。
     * 使能瞬间会把"当前位置"设为目标(原地保持), 所以不会一上电就冲。
     * 注意内环速度环由 APP_FOC_SPEED_LOOP_ENABLE 打开(见 app.h 的说明)。 */
    app_motion_init(&s_motion);
    app_motion_enable(&s_motion, true,
                      app_encoder_get_mech_rad(&s_encoder));
    LOG_I("MOT", "position loop ON: kp=%d mRPM/rad, 速度限幅=%d RPM",
          (int)(APP_MOTION_POS_KP * 1000.0f), (int)APP_MOTION_SPEED_LIMIT_RPM);
#endif

    /* ================= 第四步: 速度环反馈方向自检 =================
     *
     * 【为什么必须做】速度环是"负反馈", 但反馈回来的 rpm 是否和 iq 同号,
     * 取决于三件互相独立的事: 编码器计数方向、相电流极性(自检出来是 ±1)、
     * 以及 FOC 的电角度约定。任何一处反了, 速度环就成了【正反馈】:
     *     正给定 -> 正的 iq -> 实测转速却是负的 -> 误差更大 -> iq 更大 ...
     * iq 会一路顶到 APP_FOC_SPEED_IQ_LIMIT_A 饱和, 转子被"反向拖住"。
     * 实测日志里的现场就是: iq=+147mA(顶到限幅) 而 rpm=-553 —— 正是这个。
     *
     * 【怎么测】把速度环给定设成 +APP_SPEED_DIR_TEST_RPM, 只跑
     * APP_SPEED_DIR_TEST_MS, 然后看实测 rpm 朝哪边走。因为
     *      |给定| = 测试转速, e ≈ 测试转速, iq ≈ Kp·e
     * 所以只要把测试转速选得让 Kp·e 落在安全电流内就行 (见 app.h 的宏),
     * 而且这段时间速度环自己的 PI 会限制电流, 不会失控。
     * 测出来是负的就把 s_speed_dir 翻成 -1 (之后所有给定/反馈都过这个系数)。
     *
     * 【前提】电机必须【空载】—— 带载时转子被负载拖住/反转, 判据不可信。
     * 不需要这个自检时把 APP_SPEED_DIR_AUTOTEST 置 0, 直接信
     * APP_SPEED_DIR_SIGN 的静态值。 */
#if APP_FOC_CLOSED_LOOP_ENABLE && APP_FOC_SPEED_LOOP_ENABLE && APP_SPEED_DIR_AUTOTEST
    {
        float    rpm_before;
        float    rpm_after;
        float    vbus_f = (float)bsp_adc_get_vbus_mv() * 0.001f;
        uint32_t i;

        s_speed_dir = APP_SPEED_DIR_SIGN;
        s_spd_integ = 0.0f;

        rpm_before = app_encoder_get_mech_rpm(&s_encoder);
        s_speed_target_rpm = APP_SPEED_DIR_TEST_RPM;

        /* 用与运行期完全相同的控制路径跑: 这样"实测速度"就是运行期看到的那个量 */
        for (i = 0U; i < (APP_SPEED_DIR_TEST_MS / APP_MAIN_LOOP_MS); i++) {
            app_init_run_cycles(1U, vbus_f);
            app_speed_apply();
        }

        rpm_after = app_encoder_get_mech_rpm(&s_encoder);

        s_speed_target_rpm = 0.0f;      /* 回到停止, 交给位置环接管 */
        s_spd_integ        = 0.0f;

        if (rpm_after < rpm_before) {
            s_speed_dir = -s_speed_dir;
            LOG_I("FOC", "速度环方向自检: 给定 +%d RPM -> 实测 %d -> %d RPM, 判定反馈反向, s_speed_dir=%d",
                  (int)APP_SPEED_DIR_TEST_RPM, (int)rpm_before, (int)rpm_after,
                  (int)s_speed_dir);
        } else {
            LOG_I("FOC", "速度环方向自检: 给定 +%d RPM -> 实测 %d -> %d RPM, 方向正常, s_speed_dir=%d",
                  (int)APP_SPEED_DIR_TEST_RPM, (int)rpm_before, (int)rpm_after,
                  (int)s_speed_dir);
        }
    }
#endif
#if APP_MOTION_ENABLE
    LOG_I("FOC", "speed loop: dir=%d, target=%d RPM (位置环每拍会重写它)",
          (int)s_speed_dir, (int)s_speed_target_rpm);
#endif

    /* ================= 第五步: 按键 =================
     * 只做引脚复用/上拉和消抖状态初始化, 采样与事件处理在 app_run 里
     * (控制任务, 1kHz)。脚位/上拉/映射表见 src/bsp/bsp_button.c 的 s_cfg[]。 */
    bsp_button_init();
    LOG_I("BTN", "按键就绪: %s=位置+  %s=位置-  %s=回起点",
          bsp_button_name(BSP_BTN_UP),
          bsp_button_name(BSP_BTN_DOWN),
          bsp_button_name(BSP_BTN_HOME));
#if APP_FOC_CLOSED_LOOP_ENABLE
    LOG_I("APP", "motor ENABLED: CURRENT LOOP, iq_ref=%d mA, id_ref=%d mA, Kp=0.%02d V/A, Ki=%d V/(A*s), OC=%d mA",
          (int)(APP_FOC_IQ_REF_A * 1000.0f), (int)(APP_FOC_ID_REF_A * 1000.0f),
          (int)(APP_FOC_CURRENT_KP * 100.0f), (int)APP_FOC_CURRENT_KI,
          (int)(APP_FOC_OC_LIMIT_A * 1000.0f));
#else
    LOG_I("APP", "motor ENABLED: open-loop V/F @ %d RPM, %d mV",
          (unsigned)APP_OPENLOOP_TARGET_RPM, (unsigned)(APP_OPENLOOP_VOLTAGE_V * 1000.0f));
#endif

    LOG_I("PWM", "reload=%u, freq=%u Hz, poles=%u, target=%u RPM, V=%u mV",
          (unsigned)bsp_pwm_get_reload(), (unsigned)BSP_PWM_FREQUENCY_HZ,
          (unsigned)FOC_MOTOR_POLE_PAIRS,
          (unsigned)APP_OPENLOOP_TARGET_RPM,
          (unsigned)(APP_OPENLOOP_VOLTAGE_V * 1000.0f));
    LOG_I("APP", "Tips: watch g_diag in debugger; vbus_mv must be > %u mV",
          (unsigned)(FOC_OPENLOOP_VBUS_MIN_V * 1000.0f));
}

/* ================= 转速给定接口 (LCD 速度条 / 上位机) =================
 *
 * 【RTOS 下的跨任务约定 —— 改这个文件前务必先读这一段】
 *   FreeRTOS 起来以后, 显示任务 (lcd_app_task_handler -> lcd_ui_refresh ->
 *   滑条回调) 和控制任务 (app_run, 1ms) 是两个任务。所以这里严格分成两半:
 *
 *     app_speed_target_set()  —— 允许【任何任务】调用。它只写一个 32 位对齐的
 *        volatile float (RV32 上是单条指令, 读写天然原子), 不碰 s_ctrl, 不碰
 *        FOC, 不碰日志。所以显示任务的滑条回调可以放心调它。
 *
 *     app_speed_apply()       —— 只允许【控制任务】调用 (app_run 里)。
 *       所有对控制器的写入 (开环频率给定) 和所有状态快照都发生在这里。
 *
 *   【为什么不能把 foc_controller_set_speed_rpm() 放进 setter】
 *     那是往 s_ctrl.openloop.elec_speed_target 里写, 而控制任务每 1ms 都在读它
 *     做积分。显示任务在任意时刻抢占进来写一半, 控制任务读到的就是撕裂值
 *     或者"这一拍的给定来自上一拍、下一拍又变回去"的抖动。
 *     写者唯一化 (single writer) 是这里唯一的同步手段 —— 比加互斥量更简单,
 *     也不会让显示任务把控制任务的锁抢走。
 *
 * 【为什么目标值要斜坡】速度条是可以一手指拖到底的。环路前馈直接跳变时,
 * 开环电频率会一步跳到目标 (转子跟不上 -> 失步/过流折叠), 速度环也会给一个
 * 巨大的阶跃误差。所以默认按 APP_SPEED_RAMP_RPM_S (见 app.h) 限速爬升/下降。
 * 调试时想看阶跃响应, 把这个宏改成很大的值即可。 */

float app_speed_get_rpm(void)
{
    return app_encoder_get_mech_rpm(&s_encoder);
}

float app_speed_get_target(void)
{
    return s_speed_target_rpm;
}

float app_speed_get_target_eff(void)
{
    /* 经斜坡之后真正作用到控制环的值 (由 app_speed_apply 每拍更新)。
     * 显示任务读它用来显示"斜坡中 x/y RPM" —— 必须读快照, 不能去读 s_ctrl。 */
    return s_speed_ui_target_eff;
}

void app_speed_target_set(float rpm)
{
    if (rpm > APP_SPEED_TARGET_MAX_RPM) {
        rpm = APP_SPEED_TARGET_MAX_RPM;
    } else if (rpm < -APP_SPEED_TARGET_MAX_RPM) {
        rpm = -APP_SPEED_TARGET_MAX_RPM;
    }
    s_speed_target_rpm = rpm;
    /* 只有这一条写操作 —— 见上面"跨任务约定"。 */
}

void app_speed_apply(void)
{
    static float ramp = APP_SPEED_TARGET_DEFAULT_RPM;   /* 实际作用到控制环的值 */

    float want = s_speed_target_rpm;
    float step = APP_SPEED_RAMP_RPM_S * ((float)APP_MAIN_LOOP_MS * 0.001f);

    /* ---- 1) 斜坡 ---- */
    if (ramp < want) {
        ramp += step;
        if (ramp > want) { ramp = want; }
    } else if (ramp > want) {
        ramp -= step;
        if (ramp < want) { ramp = want; }
    }

    /* ---- 2) 下发 (控制任务上下文) ---- */
#if !APP_FOC_SPEED_LOOP_ENABLE && APP_SPEED_OPENLOOP_DIRECT
    /* 开环频率给定。已做编码器查表换向时, 矢量角度由 set_external_angle 每拍
     * 写死, 内部积分被停掉, 这里的值不产生作用 —— 那时它只影响遥测读数。 */
    if (s_ctrl.mode == foc_ctrl_mode_openloop) {
        foc_controller_set_speed_rpm(&s_ctrl, ramp);
    }
#endif
    /* 闭环 (电流环) 模式下: 由 app_run 的闭环段读 s_speed_target_rpm 用作速度环
     * 给定或软限速上限。那里用【未斜坡】的目标值是有意的 —— 速度环自己有积分器
     * 和输出限幅, 再叠一层斜坡只会让响应更慢; 软限速上限也不该被斜坡拖住
     * (否则"限速"在拉低滑条的瞬间形同虚设)。 */

    /* ---- 3) 状态快照 (给显示任务读; 它们只在这里被写, 所以显示任务读到的一定
     *         是一个"完整的"状态, 不会看到控制任务改到一半的中间态) ---- */
    s_speed_ui_running   = (s_ctrl.enabled) ? 1U : 0U;
#if APP_FOC_CLOSED_LOOP_ENABLE
    if (s_oc_tripped != 0U) {
        s_speed_ui_running = 0U;
    }
#endif
    s_speed_ui_target_eff = ramp;

    /* 目标值一变就置位一次, 让显示任务在界面上给个"已下发"的反馈。
     * 置位后显示任务读走并清零; 即便清零与置位撞在一起, 下一拍变化还会再置位。 */
    if (want != s_speed_ui_last_seen) {
        s_speed_ui_last_seen = want;
        s_speed_ui_pending   = 1U;
    }

#if APP_MOTION_ENABLE
    /* 位置环状态快照。位置环每拍都会改写速度给定, 所以它一旦在跑, 就要把
     * "目标值变了"这个标志再置一次 —— 因为对用户/界面来说, 真正在动的量是
     * 位置, 不是速度条; 而速度条的值此刻只是位置环的输出, 拿它当"用户设定"
     * 会把人误导 (实测就是屏上速度数字乱跳)。 */
    s_motion_running = (s_motion.enable && (s_speed_ui_running != 0U)) ? 1U : 0U;
    if (s_motion_takeover != 0U) {
        s_speed_ui_pending = 1U;
    }
#endif
}

uint8_t app_speed_target_pending(void)
{
    uint8_t p = s_speed_ui_pending;

    s_speed_ui_pending = 0U;      /* 显示任务读走即清 */
    return p;
}

uint8_t app_speed_is_running(void)
{
    /* 【不要在这里读 s_ctrl】那会变成"显示任务读控制任务正在写的结构体"。
     * app_speed_apply() 每拍会把状态拍成 s_speed_ui_running, 这里只读快照。 */
    return s_speed_ui_running;
}

uint8_t app_speed_is_calibrating(void)
{
#if !APP_FOC_CLOSED_LOOP_ENABLE && APP_ENC_LUT_AUTO_SWITCH && !APP_ENC_USE_BUILTIN_LUT
    /* 静态扫描造表阶段: 目标转速不参与控制 (磁场角度是被"钉"住的步进值)。
     * 条件与 app_run 的扫描段一致; 这几个变量只在 app_init/控制任务里写,
     * 且都不在 ISR 中改写, 所以显示任务读它们是安全的。 */
    if ((s_sweep_done == 0U) && (s_lut_full == 0U) && (s_lut_reject == 0U)) {
        return 1U;
    }
#endif
    return 0U;
}

/* ================= 位置环外部接口 =================
 *
 * 实现与分层在 app_motion.c (位置环 --P--> 速度给定 -> 速度环 --PI--> iq)。
 * 这里只做三件事: 转发查询、转发目标设定、以及把"位置环已接管"通知给显示层。
 * 数据来源都是快照或控制任务自己写的标量, 任意任务调用都安全。 */

uint8_t app_motion_is_enabled(void)
{
#if APP_MOTION_ENABLE
    return (s_motion.enable) ? 1U : 0U;
#else
    return 0U;
#endif
}

uint8_t app_motion_is_running(void)
{
#if APP_MOTION_ENABLE
    return s_motion_running;
#else
    return 0U;
#endif
}

uint8_t app_motion_took_over(void)
{
#if APP_MOTION_ENABLE
    uint8_t t = s_motion_takeover;

    s_motion_takeover = 0U;        /* 读走即清 */
    return t;
#else
    return 0U;
#endif
}

void app_motion_set_enable(uint8_t enable)
{
#if APP_MOTION_ENABLE
    app_motion_enable(&s_motion, (enable != 0U) ? true : false,
                      app_encoder_get_mech_rad(&s_encoder));
#else
    (void)enable;
#endif
}

float app_motion_get_speed_ref(void)
{
#if APP_MOTION_ENABLE
    return s_motion.speed_ref;
#else
    return 0.0f;
#endif
}

float app_motion_get_target_rad(void)
{
#if APP_MOTION_ENABLE
    return s_motion.pos_ref;
#else
    return 0.0f;
#endif
}

float app_motion_get_pos_rad(void)
{
#if APP_MOTION_ENABLE
    return s_motion.pos_fb;
#else
    return 0.0f;
#endif
}

float app_motion_get_err_rad(void)
{
#if APP_MOTION_ENABLE
    return app_motion_get_err(&s_motion);
#else
    return 0.0f;
#endif
}

void app_motion_set_target_rad(float pos_rad)
{
#if APP_MOTION_ENABLE
    app_motion_set_target(&s_motion, pos_rad);
#else
    (void)pos_rad;
#endif
}

/* ================= 主循环 ================= */
void app_run(void)
{
    static uint32_t tick = 0;
    int32_t iu_ma = 0, iv_ma = 0, iw_ma = 0;
    float ia, ib, ic, vbus;
    float du, dv, dw;

    diag_set_stage(APP_STAGE_LOOP_TOP);
    diag_tick();

    /* ---- 采集: 编码器 / 电流 / 母线电压 ---- */
    /* MT6701 走 SPI, 没有硬件周期触发, 由控制周期主动读一帧 (1kHz) */
    diag_set_stage(APP_STAGE_LOOP_ENCODER);   /* 给中断诊断标记: bit-bang 读编码器 */
    (void)bsp_mt6701_poll();
    /* 无论本帧成败都刷新诊断: 只靠成功回调的话, 编码器没接/没供电时
     * (poll 一直失败) CNT/ERR 会永远停在 0, 看不出任何信息。 */
    diag_set_enc(bsp_mt6701_get_raw_angle(),
                 bsp_mt6701_get_frame_count(),
                 bsp_mt6701_get_error_count(),
                 bsp_mt6701_get_last_raw(),
                 bsp_mt6701_get_transfer_fail());
    (void)app_encoder_update(&s_encoder);

#if APP_MOTION_ENABLE
    /* ---- 位置环 (外环): 位置误差 -> 速度给定 ----
     * 分层: 位置环(这里) -> 速度给定[RPM] -> 速度环(本函数后面的闭环段) -> iq
     *       -> 电流环(ADC 中断 20kHz) -> ud/uq
     * 必须放在闭环段【之前】—— 那里每拍读 s_speed_target_rpm。
     * (闭环段读的是未斜坡的目标值, 所以位置环每拍的更新会被直接采用, 不会被
     *  APP_SPEED_RAMP_RPM_S 拖慢。) */
    {
        float spd_ref = app_motion_run(&s_motion, app_encoder_get_mech_rad(&s_encoder));

#if APP_MOTION_DEMO_ENABLE
        /* 自动演示: 先原地保持, 之后每 APP_MOTION_DEMO_PERIOD_MS 在"基准角"与
         * "基准角+90°"之间来回, 便于直接观察位置环的跟随/超调。 */
        {
            static uint32_t demo_tick;
            static uint8_t  demo_hi;
            static uint8_t  demo_init;
            static float    demo_base;

            if ((demo_init == 0U) && (s_motion.enable)) {
                demo_init = 1U;
                demo_base = s_motion.pos_ref;      /* 使能时已设为"当前位置" */
            }
            if (demo_init != 0U) {
                demo_tick += APP_MAIN_LOOP_MS;
                if (demo_tick >= APP_MOTION_DEMO_PERIOD_MS) {
                    demo_tick = 0U;
                    demo_hi   = (uint8_t)(demo_hi ^ 1U);
                    app_motion_set_target(&s_motion,
                        demo_base + (demo_hi ? APP_MOTION_DEMO_STEP_RAD : 0.0f));
                }
            }
        }
#endif

        /* 交给速度环 (内环)。速度环由 APP_FOC_SPEED_LOOP_ENABLE 打开 —— 
         * 位置环必须靠它提供阻尼, 见 app.h 的说明。 */
        app_speed_target_set(spd_ref);

        /* 标记"本拍位置环接管了速度给定"。app_speed_apply() 会把状态拍成快照,
         * 显示层据此显示位置环状态而不是把速度条的值当成用户设定。 */
        s_motion_takeover = 1U;
    }

    /* ---- 按键位置点动 (板上 Key2/Key3 + J11 外接回起点键) ----
     * 映射表见 src/bsp/bsp_button.c 的 s_cfg[]:
     *   UP   = 板上 Key2 (SW5/KEYB/PC15) -> 位置 + (每步 5°)
     *   DOWN = 板上 Key3 (SW6/KEYC/PC19) -> 位置 -
     *   HOME = J11 PD8 (脚 42, 外接)     -> 回起点(重新定零), 可不接
     * 【为什么在控制任务里读按键】消抖和长按连发都需要稳定的采样周期, 控制任务
     * 正好是 1kHz; 而且按键最终要改位置目标, 这个目标只允许控制任务改。
     * 【每拍只走一步】按下/连发一次只把目标挪 APP_BTN_STEP_RAD (5°),
     * 并把目标限幅在"当前位置 ±APP_BTN_TARGET_WINDOW_RAD"之内 —— 防止连按堆出
     * 一个很大的位置误差, 一松手就满速冲过去撞限位。 */
    {
        uint32_t ev = bsp_button_poll_all();
        uint8_t  e_up   = (uint8_t)((ev >> ((uint32_t)BSP_BTN_UP   * 4U)) & 0xFU);
        uint8_t  e_down = (uint8_t)((ev >> ((uint32_t)BSP_BTN_DOWN * 4U)) & 0xFU);
        uint8_t  e_home = (uint8_t)((ev >> ((uint32_t)BSP_BTN_HOME * 4U)) & 0xFU);

        if (e_up != BSP_BTN_EV_NONE) {
            float tgt = app_motion_get_target_rad() + APP_BTN_STEP_RAD;
            float pos = app_motion_get_pos_rad();

            if (tgt > (pos + APP_BTN_TARGET_WINDOW_RAD)) {
                tgt = pos + APP_BTN_TARGET_WINDOW_RAD;
            }
            app_motion_set_target_rad(tgt);
        } else if (e_down != BSP_BTN_EV_NONE) {
            float tgt = app_motion_get_target_rad() - APP_BTN_STEP_RAD;
            float pos = app_motion_get_pos_rad();

            if (tgt < (pos - APP_BTN_TARGET_WINDOW_RAD)) {
                tgt = pos - APP_BTN_TARGET_WINDOW_RAD;
            }
            app_motion_set_target_rad(tgt);
        } else if (e_home != BSP_BTN_EV_NONE) {
            /* 回起点 = 把"当前位置"重新定为零点并停在这里。
             * 比分两步做: ①关掉位置环(速度给定归零) ②把展开基准清掉并重新使能 ——
             * 重新使能时 app_motion_enable() 会把当前角度设为目标, 于是电机原地
             * 停住而目标/反馈都归零, 不会"先冲回去再停"。 */
            app_motion_set_enable(0U);
            app_motion_set_target_rad(0.0f);
            s_motion.turns_rad    = 0.0f;
            s_motion.unwrap_valid = false;
            app_motion_set_enable(1U);
            LOG_I("MOT", "按键: 已把当前位置重新定为零点");
        }

        /* 接线排查: 上电后每 5 秒报一次三个按键的原始电平。
         * 松开时应全为 0 (低有效)。若恒为 1 -> 那个脚被拉低(按键坏/短路/脚选错);
         * 若按下时也不变 0 -> 按键没接上/焊盘不通。
         * 板上 Key2/Key3 是 PC15/PC19, 有 10K 外部上拉, 手动按住时对应位应变成 1。 */
        {
            static uint32_t btn_log_tick;

            btn_log_tick += APP_MAIN_LOOP_MS;
            if (btn_log_tick >= 5000U) {
                btn_log_tick = 0U;
                LOG_I("BTN", "按键电平: %s=%u %s=%u %s=%u (松开应全为 0, 按住该按键变 1)",
                      bsp_button_name(BSP_BTN_UP),
                      (unsigned)bsp_button_raw_level(BSP_BTN_UP),
                      bsp_button_name(BSP_BTN_DOWN),
                      (unsigned)bsp_button_raw_level(BSP_BTN_DOWN),
                      bsp_button_name(BSP_BTN_HOME),
                      (unsigned)bsp_button_raw_level(BSP_BTN_HOME));
            }
        }
    }
#endif

    /* ---- 转速给定下发 (LCD 速度条设定 -> 控制环) ----
     * 放在编码器更新之后: 速度环/软限速都要用本拍刚算出来的实测转速。 */
    app_speed_apply();

    /* ---- 【编码器/校准体检】每 APP_ENC_HEALTH_MS 一行, 串口直接看 ----
     * 编码器没接/没供电/线断/被烤坏时, 不接调试器也能一眼判定:
     *   frames 恒 0 + errs 每拍都涨 + miso=0  ->  编码器根本没有响应,
     *   此时固件看到的"转子不动"是真的读不到角度, 不是电机的问题。 */
    {
        static uint32_t enc_log_tick = 0U;
        static uint32_t enc_no_frame  = 0U;

        enc_log_tick += APP_MAIN_LOOP_MS;
        if (enc_log_tick >= APP_ENC_HEALTH_MS) {
            uint32_t fr = bsp_mt6701_get_frame_count();
            uint32_t er = bsp_mt6701_get_error_count();
#if !BSP_MT6701_USE_SEI
            uint32_t tf = bsp_mt6701_get_transfer_fail();
            uint8_t  mi = bsp_bb_miso_live();
#endif

            enc_log_tick = 0U;

#if BSP_MT6701_USE_SEI
            {
                bsp_sei_diag_t sd;
                bsp_sei_get_diag(&sd);
                LOG_I("ENC",
                      "health: reader=sei ctrl=%u frames=%u errs=%u angle=%u status=0x%x crc=%u track=%u raw=0x%06x peek=0x%06x irq_en=0x%08x irq_flag=0x%08x "
                      "sweep=%u/%u odo=%ld stall=%u lutfull=%u reject=%u",
                      (unsigned)sd.ctrl_index, (unsigned)fr, (unsigned)er,
                      (unsigned)bsp_mt6701_get_raw_angle(),
                      (unsigned)bsp_mt6701_get_status(),
                      (unsigned)bsp_mt6701_get_crc_ok(),
                      (unsigned)bsp_mt6701_get_track_loss(),
                      (unsigned)bsp_mt6701_get_raw(),
                      (unsigned)sd.peek_raw, (unsigned)sd.irq_enable,
                      (unsigned)sd.irq_flag,
                      (unsigned)s_sweep_step, (unsigned)s_sweep_steps,
                      (long)s_sweep_odo, (unsigned)s_sweep_stall,
                      (unsigned)s_lut_full, (unsigned)s_lut_reject);
            }
#else
            LOG_I("ENC",
                  "health: reader=%s bb=%d miso=%u frames=%u errs=%u tfail=%u raw=%u last=%06x "
                  "sweep=%u/%u odo=%ld stall=%u lut=%u/%u",
                  bsp_bb_is_active() ? "bitbang" : "spi2",
                  (int)bsp_bb_best_index(), (unsigned)mi,
                  (unsigned)fr, (unsigned)er, (unsigned)tf,
                  (unsigned)bsp_mt6701_get_raw_angle(),
                  (unsigned)bsp_mt6701_get_last_raw(),
                  (unsigned)s_sweep_step, (unsigned)s_sweep_steps,
                  (long)s_sweep_odo, (unsigned)s_sweep_stall,
                  (unsigned)s_lut_full, (unsigned)s_lut_reject);
#endif

            if (fr == 0U) {
                enc_no_frame++;
                if (enc_no_frame >= 5U) {
                    enc_no_frame = 0U;
#if BSP_MT6701_USE_SEI
                    LOG_I("ENC",
                          "!! SEI 一帧都没收到: 检查 SEI1 CK=PE19、RX=PE17、编码器 3.3V/GND、SSI 时序和 CSN/Z");
#else
                    LOG_I("ENC",
                          "!! 编码器一帧都没收到: MISO 空闲=%u (正常应为 1)。"
                          "查 J11: PC26=SCLK / PC28=MISO / PC24=CS, 以及编码器 3.3V/GND",
                          (unsigned)mi);
#endif
                }
            } else {
                enc_no_frame = 0U;
            }
        }
    }

    diag_set_stage(APP_STAGE_LOOP_VBUS);
    vbus = (float)bsp_adc_get_vbus_mv() * 0.001f;
    diag_set_vbus_mv((uint32_t)(vbus * 1000.0f));

    diag_set_stage(APP_STAGE_LOOP_CURRENT);
    {
        int32_t im[3] = {0, 0, 0};

        bsp_adc_get_currents_ma(&im[0], &im[1], &im[2]);
        /* 按开机自检实测的映射重排相序, 再统一到 FOC 的正方向 */
        iu_ma = im[s_phase_map[0]];
        iv_ma = im[s_phase_map[1]];
        iw_ma = im[s_phase_map[2]];
    }
    /* 统一到 FOC 的相电流正方向 (符号由开机校准/自检决定) */
    ia = (float)iu_ma * 0.001f * s_cur_sign;
    ib = (float)iv_ma * 0.001f * s_cur_sign;
    ic = (float)iw_ma * 0.001f * s_cur_sign;
    diag_set_currents_ma((int32_t)(ia * 1000.0f),
                         (int32_t)(ib * 1000.0f),
                         (int32_t)(ic * 1000.0f));

    /* ---- 实测控制周期 -> 交给控制器作为积分步长 ----
     * 上报是阻塞式的 (USB 写 + printf), 提高上报频率会拉长主循环。开环电角度
     * 是 elec_angle += elec_speed * dt 积分出来的, 若仍用名义 1ms, 磁场的
     * 实际频率就会随循环负载漂移 (负载重 -> 频率低), 导致转子失步。
     * 这里用 mcycle 实测, 保证磁场频率始终等于指令值。
     *
     * 【上 RTOS 之后这一段的地位变了】控制任务由 vTaskDelayUntil 锁 1ms, 但它
     * 并不是"绝对准时": 显示任务的长 SPI 传输、日志、以及内核临界区都会把它往后
     * 挤。实测周期一旦从 1ms 变成 5ms, 下面两个量就全错:
     *     编码器速度估算 (foc_speed_est_update) 是按【时间】差分的;
     *     电流环中断的电角度外推 (s_isr_elec_speed) 也是按【时间】差分的。
     * 所以每一拍都必须把【实测 dt】喂给它们, 而不是假设 1ms。 */
    {
        static uint64_t s_prev_mcycle;
        uint64_t now_cycle = hpm_csr_get_core_mcycle();

        if (s_prev_mcycle != 0U) {
            float dt_meas = (float)(now_cycle - s_prev_mcycle)
                            / (float)hpm_core_clock;

            /* 限幅: 排除首次采样、调试器暂停等异常大值 */
            if ((dt_meas > 0.00005f) && (dt_meas < 0.05f)) {
                s_dt_main = dt_meas;        /* 供角度外推估计电角速度用 */
                /* 【速度估算必须用实测 dt】foc_speed_est_update() 是
                 * speed = Δangle / ts。ts 写死 1ms 而实际跑了 5ms, 算出来的
                 * 转速就是真值的 5 倍 —— 于是软限速/遥测全被这个假转速带偏
                 * (实测现象: 屏一亮就报几百上千 RPM, 限速介入把 iq 砍掉, 电机
                 * 看起来就是堵转)。这里每拍把实测周期同步进去。 */
                foc_speed_est_set_ts(&s_encoder.speed_est, dt_meas);
#if !APP_CURRENT_LOOP_IN_ISR
                foc_controller_set_dt(&s_ctrl, dt_meas);
#else
                /* 电流环跑在 20kHz 中断里, 它的积分步长固定是 1/20000 ——
                 * 这里绝不能再按主循环周期改写, 否则积分增益差 20 倍。 */
#endif
            }
        }
        s_prev_mcycle = now_cycle;
    }

    /* ---- 控制任务节拍体检 (见 app.h 的 APP_LOG_CTRL_PERIOD) ----
     * 控制周期是这套 FOC 的隐含前提: 换向角是按 dt 外推的, 周期抖到几毫秒
     * 就等于电流矢量指错方向。这里每秒把实测周期统计打出来, "电机堵转/顿挫"
     * 到底是不是被显示任务挤出来的, 一眼就能看出来。 */
#if APP_LOG_CTRL_PERIOD
    {
        static uint32_t period_cnt;
        static uint32_t period_bad;
        static uint32_t period_sum100;   /* Σ 周期, 单位 1/100 ms (整数, 避免浮点格式) */
        static uint32_t period_max100;   /* 窗口内最大周期, 单位 1/100 ms */

        if ((s_dt_main > 0.0f) && (s_dt_main < 0.05f)) {
            uint32_t ms100 = (uint32_t)(s_dt_main * 100000.0f);   /* s -> 1/100 ms */

            period_cnt++;
            period_sum100 += ms100;
            if (ms100 > period_max100) {
                period_max100 = ms100;
            }
            if (ms100 > (APP_CTRL_PERIOD_WARN_MS * 100U)) {
                period_bad++;
            }
        }

        if (period_cnt >= (1000U / APP_MAIN_LOOP_MS)) {
            uint32_t avg100 = period_sum100 / period_cnt;

            LOG_I("APP", "ctrl period: avg=%u.%02u ms max=%u.%02u ms bad(>%ums)=%u/%u",
                  (unsigned)(avg100 / 100U), (unsigned)(avg100 % 100U),
                  (unsigned)(period_max100 / 100U), (unsigned)(period_max100 % 100U),
                  (unsigned)APP_CTRL_PERIOD_WARN_MS,
                  (unsigned)period_bad, (unsigned)period_cnt);
            period_cnt     = 0U;
            period_bad     = 0U;
            period_sum100  = 0U;
            period_max100  = 0U;
        }
    }
#endif

    /* ---- 控制: 统一控制器 (开环 or 电流环) ---- */
    diag_set_stage(APP_STAGE_LOOP_CONTROL);
    {
        float elec_rad = app_encoder_get_elec_rad(&s_encoder);

#if APP_ENC_USE_BUILTIN_LUT
        /* ---- 内置离线查表换向 (仅开环电压矢量模式) ----
         * elec_rad 就是"该 raw 处转子吸稳时的磁场电角度"(app_encoder 里查表得到),
         * 再加超前角把电压矢量放到 q 轴 -> 单位电流出最大转矩。
         *
         * 【电流环模式下不能叠加超前角】闭环时 foc_update() 直接把 elec_rad 拿去做
         * Park/反 Park, ud/uq 由 PI 算 —— 角度本身已经是"转子 d 轴"。再叠 90° 等于
         * 把整个 dq 坐标系转过去: 环路会去调错误的轴 (id 指令实际加在 q 轴上)。
         * 超前角是开环"没有电流反馈、只能盲推"的补偿, 闭环有反馈就不需要了。 */
        if (s_ctrl.mode == foc_ctrl_mode_openloop) {
            foc_controller_set_external_angle(&s_ctrl,
                elec_rad + APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f));
        }
#endif

#if APP_ENC_ANGLE_DRIVE
        /* ---- 编码器换向 (开环电压幅值) ----
         * 电压矢量的角度 = 编码器电角度 + 超前角。
         * 超前 90° 让矢量落在 q 轴 -> 单位电压出最大转矩;
         * 若直接填 enc_elec, 矢量落在 d 轴, 转矩为 0 (电机只会发热不转)。
         * 幅值仍走开环给定, 所以电压斜坡 + 过流折叠保护照常生效。 */
        {
            float lead = APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f);

            foc_controller_set_external_angle(&s_ctrl, elec_rad + lead);
        }
#endif

#if APP_ENC_LUT_AUTO_SWITCH || APP_ENC_LUT_LEARN || APP_ENC_LUT_USE
        /* ---- 编码器角度查表 (LUT) ----
         * 学习: 把 (编码器原始角, 当前软件电角度) 喂进对应箱做圆平均。
         *       LUT[bin] 逐渐收敛成"该机械位置对应的平均电角度"。
         * 使用: 电角度改从表里取 (相邻箱插值), 表项不足处自动退回线性公式。
         * 注意学习只在【开环旋转】时才有意义。 */
        if (!APP_ENC_USE_BUILTIN_LUT) {   /* 用内置离线表时整块旁路 */
            uint32_t raw = (uint32_t)app_encoder_get_raw_angle(&s_encoder);
            uint32_t sw_mrad =
                (uint32_t)(foc_normalize_angle(s_ctrl.openloop.elec_angle) * 1000.0f);

#if APP_ENC_LUT_AUTO_SWITCH
            if ((s_lut_reject != 0U) && (s_lut_full == 0U)) {
                /* 表被体检否决 (缺口太长 / 整圈不自洽): 保持开环 VF 旋转,
                 * 不换向。拿"编出来"的表换向 = 转子在补出来的那段里反复弹 + 堵转发热。 */
            } else if (s_lut_full == 0U) {
                /* 阶段 1: 学表。此时电压矢量角度仍然由内部积分的模拟电角度决定。 */
#if APP_CAL_SWEEP_ENABLE
                /* 表由【静态扫描】造 (见 app_diag.h / app.h 的说明):
                 * 动态学法会把负载角 δ 积分进表, 累积误差每次都能到 60~100°,
                 * 必然在某个位置卡死。这里把动态喂表彻底关掉。 */
                (void)sw_mrad;
#else
                if (s_ctrl.mode == foc_ctrl_mode_openloop) {
                    diag_lut_feed(raw, sw_mrad);
                }
#endif

                if (diag_lut_valid_bins() >= ENC_LUT_BINS) {
                    uint32_t fixed;
                    uint32_t di;
                    char     dline[256];

                    s_lut_full = 1U;

                    /* 一次性整表转储: 此刻转子还没换向(开环慢转), 阻塞打印无所谓,
                     * 而且转储只发生这一次, 不会像周期转储那样把电机"刹"住。
                     * 先打【学习原始结果】(A), 再打【修复后】(B), 两段对照即可定位
                     * "斜率坏"的箱 —— 也就是换向后会被焊住的那个机械位置。 */
                    LOG_I("ENC", "LUT dump A: learned (before repair)");
                    for (di = 0U; di < ENC_LUT_LINES; di++) {
                        (void)diag_lut_dump_line(dline, sizeof(dline), di);
                        app_log_write_raw(dline);
                    }

                    /* 切换向前先修表: 把"学习时转子被顶住"的坏箱用邻域插值顶掉。
                     * 这些箱斜率可能反向, 不修就是换向后的"磁弹簧"陷阱。 */
                    fixed = diag_lut_repair();
                    {
                        uint32_t h_back = 0U;
                        int32_t  h_neg  = 0;
                        int32_t  h_pos  = 0;
                        int32_t  h_sum  = diag_lut_health(&h_back, &h_neg, &h_pos);

                        LOG_I("ENC",
                              "LUT repair: %u/%u bins interpolated (span>%d mrad)",
                              (unsigned)fixed, (unsigned)ENC_LUT_BINS,
                              (int)ENC_LUT_SPAN_MAX);
                        LOG_I("ENC",
                              "LUT health: sum=%d (want 75396=12rev) back=%u worst=%d..%d",
                              (int)h_sum, (unsigned)h_back, (int)h_neg, (int)h_pos);
                    }

                    LOG_I("ENC", "LUT dump B: repaired");
                    for (di = 0U; di < ENC_LUT_LINES; di++) {
                        (void)diag_lut_dump_line(dline, sizeof(dline), di);
                        app_log_write_raw(dline);
                    }

                    /* 换向后把电压/电流限幅都降下来:
                     *   学表阶段用 0.7V / 4A 保证转子锁得住;
                     *   换向后矢量跟着转子走, 转速由 V/Ke 自限,
                     *   "手能不能停住"由转矩 ≈ Kt×I 决定, 所以电流限幅
                     *   (APP_ENC_LUT_RUN_CURRENT_LIMIT_A) 比电压更直接。
                     * 电压会按 FOC_OPENLOOP_VOLTAGE_SLEW 平滑爬过去。 */
                    foc_controller_set_voltage(&s_ctrl,
                                               APP_ENC_LUT_RUN_VOLTAGE_V);
                    foc_controller_set_current_limit(&s_ctrl,
                                                     APP_ENC_LUT_RUN_CURRENT_LIMIT_A);

                    /* 【存档】表已被接受, 立即写 Flash, 以后上电免校准 */
                    {
                        static int16_t nv_tbl[ENC_LUT_BINS];

                        diag_lut_snapshot(nv_tbl);
                        if (app_nv_save(nv_tbl, s_cal_offset_mrad)) {
                            LOG_I("ENC", "NV: LUT + offset saved to flash (offset=%ld mrad)",
                                  (long)s_cal_offset_mrad);
                        } else {
                            LOG_I("ENC", "NV: save FAILED");
                        }
                    }

                    LOG_I("ENC",
                          "LUT %u/%u full -> LUT commutation ON, lead=%d deg, V=%d mV Ilim=%d mA",
                          (unsigned)ENC_LUT_BINS, (unsigned)ENC_LUT_BINS,
                          (int)APP_ENC_LEAD_DEG,
                          (int)(APP_ENC_LUT_RUN_VOLTAGE_V * 1000.0f),
                          (int)(APP_ENC_LUT_RUN_CURRENT_LIMIT_A * 1000.0f));
                }
            } else {
                /* 阶段 2: 学满, 电角度改用查表值 + 超前角。
                 * 查表值就是"该机械位置对应的磁场电角度", 反过来说就是
                 * 转子电角度 + 平均负载角; 再加超前角即可换向。
                 *
                 * 【必须取回 int32_t】diag_lut_angle 返回的是 (uint32_t)(int32_t):
                 * 表里约一半箱是负角度, 直接 (float)uint32_t 会变成 4.29e9,
                 * 乘 0.001f 后落在 float32 间距 0.5rad 的台阶上 —— 电压矢量
                 * 角度变成每步 500mrad 的乱数, 换向自然抖/卡。
                 * 症状: CSV 第 1 列(sw)只出现间隔 500mrad 的几个值。 */
                uint32_t lut = diag_lut_angle(raw);

                if (lut != 0xFFFFFFFFU) {
                    float lead = APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f);
                    int   dwell = diag_lut_dwell(raw);

                    /* 【反磁弹簧踢一脚】表里只要有一小段误差落在 ~90°, 那里就是
                     * 零转矩假平衡点: 转矩 = Kt·I·cos(误差) ≈ 0, 转子顶在那儿
                     * 一动不动却一直通着电流(发热), 自己出不来。
                     * 静止超过 APP_ENC_STUCK_KICK_MS 就把超前角临时抬到
                     * APP_ENC_KICK_LEAD_DEG: 误差 90° 时转矩从 0 变成 cos(-60°)=0.5,
                     * 转子被拽出去后表就恢复正常工作了。 */
                    if (dwell != 0) {
                        if (s_stuck_ms < 0xFFFFU) { s_stuck_ms += APP_MAIN_LOOP_MS; }
                    } else {
                        s_stuck_ms = 0U;
                    }
                    if (s_stuck_ms >= APP_ENC_STUCK_KICK_MS) {
                        s_stuck_ms = 0U;
                        s_kick_ms  = APP_ENC_KICK_HOLD_MS;
                        LOG_I("ENC", "KICK raw=%u bin=%u lut=%d (顶住>%ums, 超前角临时 %d deg)",
                              (unsigned)raw, (unsigned)(raw / 16U), (int)(int32_t)lut,
                              (unsigned)APP_ENC_STUCK_KICK_MS,
                              (int)APP_ENC_KICK_LEAD_DEG);
                    }
                    if (s_kick_ms > 0U) {
                        s_kick_ms -= APP_MAIN_LOOP_MS;
                        lead = APP_ENC_KICK_LEAD_DEG * (FOC_TWO_PI / 360.0f);
                    }

                    foc_controller_set_external_angle(
                        &s_ctrl, ((float)(int32_t)lut * 0.001f) + lead);

                    /* 【卡顿定位】换向后转子被齿槽/转矩凹陷顶住时, 把位置打出来。
                     * raw 直接对上 LUT dump 的行: bin = raw/16, 例如 raw=11200
                     * -> LUT 行 LUT700 的第 0 个数。 */
                    if (dwell != 0) {
                        s_dwell_ticks++;
                        if (s_dwell_ticks >= APP_ENC_DWELL_LOG_TICKS) {
                            s_dwell_ticks = 0U;
                            LOG_I("ENC", "STALL raw=%u bin=%u lut=%d",
                                  (unsigned)raw, (unsigned)(raw / 16U),
                                  (int)(int32_t)lut);
                        }
                    } else {
                        s_dwell_ticks = 0U;
                    }

                    /* 【转速图】采满一次就把 64 个扇区的最小 4 拍位移打出来:
                     * 数值明显低的扇区就是"卡在哪个 raw"的答案。 */
                    if (diag_spdmap_feed(raw) != 0) {
                        uint32_t si;

                        for (si = 0U; si < ENC_SPDMAP_SECT; si += 16U) {
                            LOG_I("ENC",
                                  "SPDMAP%02u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                                  (unsigned)si,
                                  (unsigned)diag_spdmap_get(si + 0U),
                                  (unsigned)diag_spdmap_get(si + 1U),
                                  (unsigned)diag_spdmap_get(si + 2U),
                                  (unsigned)diag_spdmap_get(si + 3U),
                                  (unsigned)diag_spdmap_get(si + 4U),
                                  (unsigned)diag_spdmap_get(si + 5U),
                                  (unsigned)diag_spdmap_get(si + 6U),
                                  (unsigned)diag_spdmap_get(si + 7U),
                                  (unsigned)diag_spdmap_get(si + 8U),
                                  (unsigned)diag_spdmap_get(si + 9U),
                                  (unsigned)diag_spdmap_get(si + 10U),
                                  (unsigned)diag_spdmap_get(si + 11U),
                                  (unsigned)diag_spdmap_get(si + 12U),
                                  (unsigned)diag_spdmap_get(si + 13U),
                                  (unsigned)diag_spdmap_get(si + 14U),
                                  (unsigned)diag_spdmap_get(si + 15U));
                        }
                    }
                }
            }
#else
#  if APP_ENC_LUT_LEARN
            if (s_ctrl.mode == foc_ctrl_mode_openloop) {
                diag_lut_feed(raw, sw_mrad);
            }
#  endif

#  if APP_ENC_LUT_USE
            {
                uint32_t lut = diag_lut_angle(raw);

                if (lut != 0xFFFFFFFFU) {
                    /* 表里的角度 = 该位置的【磁场】电角度。
                     * 要用它换向, 仍需叠加超前角 (原理同编码器换向)。
                     * lut 是 (uint32_t)(int32_t), 负角度必须先取回 int32_t。 */
                    float lead = APP_ENC_LEAD_DEG * (FOC_TWO_PI / 360.0f);

                    foc_controller_set_external_angle(
                        &s_ctrl, ((float)(int32_t)lut * 0.001f) + lead);
                }
            }
#  endif
#endif
        }
#endif

#if APP_FOC_CLOSED_LOOP_ENABLE
        /* ================= 闭环安全保护 + 转矩给定 =================
         * 电流环这条路径【本身没有任何保护】(foc_controller_get_trip 在电流环
         * 模式下恒返回 0), 所以这里必须自己补两道:
         *   1) 过流硬保护: 任一相瞬时值超过阈值 -> 立即关断输出并锁死。
         *      没有它的话, PI 参数不合适时电流可能冲到 限幅电压/R ≈ 数十安。
         *   2) 软转速限幅: 闭环只给转矩, 超速就把 iq 减弱/清零。
         *
         * 【转矩给定量 iq 的三个来源, 按优先级】
         *   a) APP_FOC_SPEED_LOOP_ENABLE = 1 -> 速度环 PI: 按 (目标转速 - 实测)
         *      算 iq, "设多少转多少"。目标转速来自 LCD 速度条。
         *   b) 速度环关闭 -> iq = APP_FOC_IQ_REF_A (固定转矩), 目标转速只当
         *      "允许的最高转速"用 (软限速, 逻辑与原来完全一致)。
         *   c) 过流锁死 -> iq = 0。 */
        {
            static uint32_t s_oc_cnt;
            float imax = app_abs_f(ia);
            float rpm  = app_encoder_get_mech_rpm(&s_encoder);
            float iq   = APP_FOC_IQ_REF_A;

            if (app_abs_f(ib) > imax) { imax = app_abs_f(ib); }
            if (app_abs_f(ic) > imax) { imax = app_abs_f(ic); }

            /* 【去抖】必须连续 APP_FOC_OC_TRIP_CNT 拍都超限才锁死。
             * 实测出现过单拍 10A 的假值(开关瞬间把运放打飞), 而 12V+3.1Ω 下
             * 即使相间短路也只有 1.9A —— 单拍就锁死会让一次毛刺打断整个运行。 */
            if (imax > APP_FOC_OC_LIMIT_A) {
                if (s_oc_cnt < 0xFFFFU) { s_oc_cnt++; }
            } else {
                s_oc_cnt = 0U;
            }

            if ((s_oc_tripped == 0U) && (s_oc_cnt >= APP_FOC_OC_TRIP_CNT)) {
                s_oc_tripped = 1U;
#if APP_CURRENT_LOOP_IN_ISR
                s_isr_closed_loop = 0U;     /* 先停中断里的闭环, 再关输出, 避免抢 shadow */
#endif
                foc_controller_enable(&s_ctrl, false);
                bsp_pwm_enable_output(false);
                /* 把三相电流和当时的 id/iq 全打出来: 判断是"真过流"还是"电流换算
                 * (采样电阻/增益)偏大导致误触发"。注意这个保护是锁死的, 复位才恢复。 */
                LOG_I("FOC", "OVERCURRENT %d mA x%u (ia=%d ib=%d ic=%d) id=%d iq=%d rpm=%d sign=%d -> output OFF (锁死, 需复位)",
                      (int)(imax * 1000.0f), (unsigned)s_oc_cnt,
                      (int)(ia * 1000.0f), (int)(ib * 1000.0f), (int)(ic * 1000.0f),
                      (int)(s_ctrl.foc.id * 1000.0f), (int)(s_ctrl.foc.iq * 1000.0f),
                      (int)app_encoder_get_mech_rpm(&s_encoder), (int)s_cur_sign);
            }

            if (s_oc_tripped != 0U) {
                /* 锁死后速度环积分必须清零: 否则复位重跑时它会带着旧积分冲出去 */
#if APP_FOC_SPEED_LOOP_ENABLE
                s_spd_integ = 0.0f;
#endif
                iq = 0.0f;
            } else {
#if APP_FOC_SPEED_LOOP_ENABLE
                /* ---- a) 速度环 (目标 - 实测 -> iq) ----
                 * 【为什么目标要乘 s_speed_dir】速度环必须是负反馈: 给定"正转速"
                 * 时实测转速也必须朝正走。而"实测 rpm 的符号"取决于编码器计数
                 * 方向、相电流极性(自检结果为 ±1)、FOC 电角度约定这三件事的组合,
                 * 任何一处反了, 速度环就成了【正反馈】(iq 顶到限幅、转子被反拖)。
                 * s_speed_dir 就是这三件事合成后的符号, 由开机自检实测出来
                 * (见 app_init 的"速度环方向自检"); 不想自检就用它的静态默认值。
                 * 用 s_speed_dir 而不是改 Kp/Ki 的符号, 是因为 Kp/Ki 同时决定
                 * 环路增益的"量", 取负会让人误以为增益也变了。
                 *
                 * 【积分限幅 = 抗饱和】积分项本身就等于稳态所需的 iq, 所以把它
                 * 夹在 ±IQ_LIMIT 内就够了。不夹的话, 目标远大于实际 (例如堵转)
                 * 时积分会一路涨到天上, 松开负载后要好几秒才退出来。 */
                {
                    float target = s_speed_target_rpm * s_speed_dir;
                    float e      = target - rpm;
                    float kp     = APP_FOC_SPEED_KP;
                    float ki     = APP_FOC_SPEED_KI;

                    s_spd_integ += (ki * e * ((float)APP_MAIN_LOOP_MS * 0.001f));
                    if (s_spd_integ >  APP_FOC_SPEED_IQ_LIMIT_A) {
                        s_spd_integ =  APP_FOC_SPEED_IQ_LIMIT_A;
                    } else if (s_spd_integ < -APP_FOC_SPEED_IQ_LIMIT_A) {
                        s_spd_integ = -APP_FOC_SPEED_IQ_LIMIT_A;
                    }

                    iq = (kp * e) + s_spd_integ;
                    if (iq >  APP_FOC_SPEED_IQ_LIMIT_A) {
                        iq =  APP_FOC_SPEED_IQ_LIMIT_A;
                    } else if (iq < -APP_FOC_SPEED_IQ_LIMIT_A) {
                        iq = -APP_FOC_SPEED_IQ_LIMIT_A;
                    }
                }
#else
                /* ---- b) 软限速, 代替原来的开关式 ----
                 * 原逻辑: 超过 90% 限速就砍到 30%, 超过 100% 直接清零 —— 纯开关。
                 * 实测后果: 转速在 86~143RPM 之间来回冲(日志里 rpm 就是这么跳的),
                 * 电机转动时震动明显。原因不是齿槽: 0.3A 转矩 = 0.083N·m, 这个
                 * 4015 云台电机转动惯量约 3e-5 kg·m^2, 角加速度 2767 rad/s^2 ——
                 * 每毫秒变速 26RPM, 转矩一开一关就冲出几十 RPM。
                 * (齿槽转矩约额定的 1~3% = 0.002N·m, 换算到转速纹波只有 0.065RPM,
                 *  在 100RPM 下根本看不出来; 齿槽只在极低速时才表现为爬行。)
                 *
                 * 现在改成线性渐弱: 从 KNEE×限速 到 限速 之间把 iq 按比例降到 0,
                 * 环路增益更低、没有硬跳变, 转速纹波会小很多。
                 *
                 * 【限速取谁】用户把速度条拉离满量程时, 限速就是滑条值 (LCD 上
                 * 设多少就限到多少); 拉到满量程 (= 不打算额外限速) 时回到原来的
                 * APP_FOC_RPM_LIMIT, 保证 UI 的存在不会让电机比原来更"没劲"。 */
                {
                    float lim  = APP_FOC_RPM_LIMIT;
                    float arpm = app_abs_f(rpm);
                    float knee;
                    float trpm = s_speed_target_rpm;

                    if (trpm < 0.0f) {
                        trpm = -trpm;
                    }
                    /* 【滑条拉到 0 的特殊含义】"我不打算限速", 而不是"限到 0 转"。
                     * 界面上的 0 和"无限速"共用一格, 说不清楚; 与其让用户以为
                     * 拉到底会让电机停住, 不如把 0 当满量程, 并在 LCD 上写清楚。
                     *
                     * 【两个"不限速"入口】0 和 >= 满量程 (APP_SPEED_TARGET_MAX_RPM)
                     * 都表示不额外限速。而【上电默认值就是满量程】, 所以这个特性
                     * 默认是完全旁路的 —— 电机行为与没有速度条时一模一样, 只有
                     * 用户真的拖了滑条才会限速。这一点非常关键: 之前默认 300 RPM
                     * 等于一上电就给电机套了个低限速, 表现就是"屏一亮电机就堵转"。 */
                    if ((trpm <= 0.0f) || (trpm >= APP_SPEED_TARGET_MAX_RPM)) {
                        trpm = 0.0f;
                    } else if (trpm >= APP_FOC_RPM_LIMIT) {
                        trpm = 0.0f;
                    }
                    if (trpm > 0.0f) {
                        lim = trpm;
                    }

                    knee = lim * APP_FOC_RPM_KNEE;
                    if (arpm > lim) {
                        iq = 0.0f;
                    } else if ((knee > 0.0f) && (arpm > knee)) {
                        iq *= (lim - arpm) / (lim - knee);
                    }
                }
#endif /* APP_FOC_SPEED_LOOP_ENABLE */
            }

            foc_controller_set_current_ref(&s_ctrl, APP_FOC_ID_REF_A, iq);
        }
#endif

#if APP_CURRENT_LOOP_IN_ISR
        /* 电流环在 20kHz 中断里跑 (见 app_on_adc_conv): 这里只把"角度 + 电角速度 +
         * 母线电压"交给它, 并且【不要】再写占空比 —— 否则会和中断抢 shadow 寄存器。
         *
         * 【电角速度怎么来】用两次主循环之间的表角度差 / 实测 dt。
         *   - 不用编码器 rpm 换算: 那样要处理方向符号(LUT 方向与 FOC_ENCODER_DIRECTION
         *     相反), 容易搞错; 直接用角度差, 符号天然一致。
         *   - 过零要展开(角度是 0~2π 的锯齿), 否则每转一圈会窜出一个虚假的巨大速度。
         * 中断拿到这个速度后自己外推(见 app_on_adc_conv), 所以主循环被日志阻塞
         * 几十毫秒也不会让角度冻住 —— 之前"打印日志时电机顿一下"就是这个问题。 */
        {
            static float   prev_ang;
            static uint8_t have_prev;
            float d = elec_rad - prev_ang;

            if (d > FOC_PI) {
                d -= FOC_TWO_PI;
            } else if (d < -FOC_PI) {
                d += FOC_TWO_PI;
            }
            /* 首次没有上一拍, 直接跳过: 否则会算出一个虚假的巨大速度,
             * 中断按它外推会把角度甩出去(虽然下一个主循环就重同步了)。 */
            if ((have_prev != 0U) && (s_dt_main > 0.0002f) && (s_dt_main < 0.01f)) {
                float w = d / s_dt_main;            /* 电角速度 rad/s */

                if (w > 2000.0f) {                  /* 2000rad/s = 318Hz 电 = 1735RPM */
                    w = 2000.0f;
                } else if (w < -2000.0f) {
                    w = -2000.0f;
                }
                s_isr_elec_speed = w;
            }
            prev_ang  = elec_rad;
            have_prev = 1U;
        }

        s_isr_elec_rad = elec_rad;                  /* 重同步(中断随后自己外推) */
        s_isr_vbus     = vbus;
        du = dv = dw = 0.5f;
#else
        foc_controller_run(&s_ctrl, ia, ib, ic, elec_rad, vbus);
        foc_controller_get_duty(&s_ctrl, &du, &dv, &dw);
        bsp_pwm_set_duty_uvw(du, dv, dw);
#endif

        diag_set_control(foc_controller_get_trip(&s_ctrl),
                         (uint32_t)(s_ctrl.openloop.voltage * 1000.0f),
                         (uint32_t)(s_ctrl.openloop.elec_angle * 1000.0f),
                         (uint32_t)(du * 1000.0f),
                         (uint32_t)(dv * 1000.0f),
                         (uint32_t)(dw * 1000.0f));
    }
    diag_set_stage(APP_STAGE_LOOP_DONE);

    /* ---- 电流环体检: 每秒一行 id/iq/ud/uq ----
     * 做电流环没有这行等于闭着眼调: 看 iq 有没有跟到 iq_ref、
     * id 有没有被压住(应该 ~0)、ud/uq 有没有顶到限幅。
     * 若 iq 与 iq_ref 反号或者一路顶到限幅, 首先怀疑相电流极性 (看开机自检那行)。 */
#if APP_FOC_CLOSED_LOOP_ENABLE
    {
        static uint32_t cur_log_tick = 0U;
#if APP_CURRENT_LOOP_IN_ISR
        static uint32_t isr_cnt_prev = 0U;
#endif

        cur_log_tick += APP_MAIN_LOOP_MS;
        if (cur_log_tick >= 1000U) {
            cur_log_tick = 0U;
#if APP_CURRENT_LOOP_IN_ISR
            {
                /* 【采样看门狗】
                 * 电流环完全靠 ADC 中断驱动: 中断一旦停摆, 占空比就冻结在最后一拍,
                 * 可能给绕组一直通直流 —— 所以这个保护必须留。
                 *
                 * 【为什么不打印常规状态】原来每秒打一行 "ADC ISR rate=xxxx (正常约
                 * 24000), 最大间隔=xxxx us", 那个"正常值"其实取决于主循环实际周期
                 * (实测主循环约 0.96~1.2ms 之间变), 写死一个数只会误导; 而"最大间隔"
                 * 只报不处理, 看着像故障却什么也不做。都不打了, 只在真出问题时报警。
                 *
                 * 期望值这里改用【实测主循环周期累加】出来的本窗口真实时长来算,
                 * 不再假设窗口就是 1 秒。 */
                static float win_time;
                uint32_t     now  = s_isr_cnt;
                uint32_t     rate = now - isr_cnt_prev;
                uint32_t     expect;

                isr_cnt_prev = now;
                win_time += s_dt_main;
                expect = (uint32_t)((float)APP_ADC_ISR_FREQ_HZ * win_time);

#if APP_LOG_ISR_STAT
                /* 只在打开这个开关时打印(排障用, 见 app.h 的说明) */
                {
                    uint32_t dtmax_us = (uint32_t)(((uint64_t)s_isr_dt_max * 1000000U)
                                                   / (uint64_t)hpm_core_clock);

                    s_isr_dt_max = 0U;
                    LOG_I("FOC", "ADC ISR: 实测 %u 次 / 期望 %u 次, 最大间隔 %u us",
                          (unsigned)rate, (unsigned)expect, (unsigned)dtmax_us);
                }
#endif
                s_isr_dt_max = 0U;

                /* 【中断被拖长的现场】只在真发生时打印, 并指出当时主循环在哪:
                 * 15=编码器 16=LCD 17=日志 18=USB 12=电流 11=VBUS */
                if (s_isr_gap_us != 0U) {
                    LOG_I("FOC", "!! ADC 中断被拖长 %u us, 当时主循环 stage=%u "
                                 "(15=编码器 16=LCD 17=日志 18=USB)",
                          (unsigned)s_isr_gap_us, (unsigned)s_isr_gap_stage);
                    s_isr_gap_us = 0U;
                }
                win_time = 0.0f;

                if ((s_isr_closed_loop != 0U) && (rate < (expect / 4U))) {
                    s_isr_closed_loop = 0U;
                    foc_controller_enable(&s_ctrl, false);
                    bsp_pwm_enable_output(false);
                    LOG_I("FOC", "!! ADC 中断只跑了 %u 次 (该约 %u), 反馈已失效 -> 封锁输出 (需复位)",
                          (unsigned)rate, (unsigned)expect);
                }
            }
#endif
#if APP_MOTION_ENABLE
            /* 带上位置环数据: 目标是 posref, 反馈是 pos, 两者之差 err 就是位置环的
             * 输入; spdref 是位置环给出的速度给定(交给速度环)。调位置环看这行:
             *   err 长时间不归零 -> pos_kp 太小; err 反复过零/来回摆 -> pos_kp 太大。 */
            LOG_I("FOC", "id=%d iq=%d mA (ref %d/%d) ud=%d uq=%d mV vbus=%d mV rpm=%d sign=%d | "
                         "pos=%d mrad ref=%d mrad err=%d mrad spdref=%d rpm",
                  (int)(s_ctrl.foc.id * 1000.0f),
                  (int)(s_ctrl.foc.iq * 1000.0f),
                  (int)(s_ctrl.foc.id_ref * 1000.0f),
                  (int)(s_ctrl.foc.iq_ref * 1000.0f),
                  (int)(s_ctrl.foc.ud * 1000.0f),
                  (int)(s_ctrl.foc.uq * 1000.0f),
                  (int)bsp_adc_get_vbus_mv(),
                  (int)app_encoder_get_mech_rpm(&s_encoder),
                  (int)s_cur_sign,
                  (int)(app_motion_get_pos(&s_motion) * 1000.0f),
                  (int)(s_motion.pos_ref * 1000.0f),
                  (int)(app_motion_get_err(&s_motion) * 1000.0f),
                  (int)s_motion.speed_ref);
#else
            LOG_I("FOC", "id=%d iq=%d mA (ref %d/%d) ud=%d uq=%d mV vbus=%d mV rpm=%d sign=%d",
                  (int)(s_ctrl.foc.id * 1000.0f),
                  (int)(s_ctrl.foc.iq * 1000.0f),
                  (int)(s_ctrl.foc.id_ref * 1000.0f),
                  (int)(s_ctrl.foc.iq_ref * 1000.0f),
                  (int)(s_ctrl.foc.ud * 1000.0f),
                  (int)(s_ctrl.foc.uq * 1000.0f),
                  (int)bsp_adc_get_vbus_mv(),
                  (int)app_encoder_get_mech_rpm(&s_encoder),
                  (int)s_cur_sign);
#endif
        }
    }
#endif

    /* ---- 安装偏置校准量 / 校准验证 ----
     * foc.h 的换算关系:  elec = normalize(mech * POLE_PAIRS + offset)
     *
     * 【DIFF 的含义】
     *   用内置离线表时: enc_elec 就是表值, 而矢量角 = 表值 + 超前角, 所以
     *       DIFF ≈ 超前角 (90 度 = 1571 mrad) 为正常;
     *       DIFF 偏离超前角 = 转子没跟上 (负载角/失步), 偏小说明被负载拖住。
     *   用线性公式时: enc_elec = mech*POLE_PAIRS + offset, 于是
     *       DIFF = sw_elec - (mech * POLE_PAIRS + offset)
     *            = 磁场电角度 与 编码器电角度 之差 = 【残差 / 负载角】
     *   - 偏置标定正确 且 转子吸稳对齐 -> DIFF ≈ 0   (校准验证通过)
     *   - 旋转中 -> DIFF 就是负载角, 反映失步/喘振程度
     *   - 若 DIFF 稳定在某个非零常数 -> 那个常数就是还差的偏置, 补进配置即可 */
    {
        float sw_rad   = foc_normalize_angle(app_vec_angle_rad());
#if APP_ENC_USE_BUILTIN_LUT
        float enc_elec = app_encoder_get_elec_rad(&s_encoder);   /* = 表值 */
#else
        float mech_rad = app_encoder_get_mech_rad(&s_encoder);
        float enc_elec = foc_mech_to_elec(mech_rad, app_encoder_get_elec_offset_rad());
#endif
        float diff_rad = sw_rad - enc_elec;

        while (diff_rad > (FOC_TWO_PI * 0.5f)) {
            diff_rad -= FOC_TWO_PI;
        }
        while (diff_rad < -(FOC_TWO_PI * 0.5f)) {
            diff_rad += FOC_TWO_PI;
        }

        diag_set_cal((uint32_t)(sw_rad * 1000.0f),
                     (uint32_t)app_encoder_get_raw_angle(&s_encoder),
                     (uint32_t)(enc_elec * 1000.0f),
                     (int32_t)(diff_rad * 1000.0f));
    }

#if APP_CAL_SWEEP_ENABLE
    /* ---- 静态角度扫描标定 ----
     * 每步把磁场钉在第 k 步的电角度, 等转子吸稳 (APP_CAL_SWEEP_HOLD 个循环),
     * 取编码器角度平均后打印一行。扫完一遍自动关断输出, 避免持续堵转发热。
     * 静态下没有动态滞后, 所以得到的是真正的磁场电角度 <-> 编码器机械角关系:
     * 斜率均匀 = 编码器正常; 斜率不均 = 编码器有周期性误差。 */
    if ((s_sweep_done == 0U) && !APP_ENC_USE_BUILTIN_LUT) {
        if (s_sweep_pre < APP_CAL_SWEEP_PRESETTLE) {
            /* 预吸稳: 磁场先钉在 0°, 等转子从上电位置被拽过来的摆动彻底停住。
             * 这期间【不采样】—— 否则摆动过程会被当成"磁场在动、转子不动",
             * 把开头那几十个箱填成平表, 换向后正好停在里面出不来。 */
            s_sweep_pre++;
            foc_controller_set_align_angle(&s_ctrl, 0.0f);
            foc_controller_set_voltage(&s_ctrl, APP_CAL_SWEEP_HIGH_V);  /* 预吸稳里降到标定电压 */
        } else {
        {
            uint32_t raw_now = (uint32_t)app_encoder_get_raw_angle(&s_encoder);

            if (s_sweep_cnt == APP_CAL_SWEEP_DROP) {      /* 暂态结束 -> 取参考 */
                s_sweep_ref = raw_now;
                s_sweep_a1 = 0;
                s_sweep_a2 = 0;
            }
            if (s_sweep_cnt >= APP_CAL_SWEEP_DROP) {
                int32_t d = cal_wrap_raw((int32_t)raw_now - (int32_t)s_sweep_ref);

                if (s_sweep_cnt < APP_CAL_SWEEP_MID) {
                    s_sweep_a1 += d;                      /* 前半段: 正常电压 */
                } else {
                    s_sweep_a2 += d;                      /* 后半段: 弱场 */
                }
            }
            if (s_sweep_cnt == APP_CAL_SWEEP_MID) {
                foc_controller_set_voltage(&s_ctrl, APP_CAL_SWEEP_LOW_V);
            }
        }
        s_sweep_cnt++;

        if (s_sweep_cnt >= APP_CAL_SWEEP_HOLD) {
            char line[128];
            int32_t resid = g_cal.diff_mrad;      /* 已含当前 offset, 所以是残差 */

            /* 解卷绕平均: 丢掉前 1/3 暂态, 余下相对参考累加, 可跨 16384->0 */
            /* E1=正常电压段平均, E2=弱场段平均; dE 直接反映"转子是否真停在磁场角度上" */
            int32_t e1 = (int32_t)s_sweep_ref +
                         s_sweep_a1 / (int32_t)(APP_CAL_SWEEP_MID - APP_CAL_SWEEP_DROP);
            int32_t e2 = (int32_t)s_sweep_ref +
                         s_sweep_a2 / (int32_t)(APP_CAL_SWEEP_HOLD - APP_CAL_SWEEP_MID);
            uint32_t raw_avg =
                (uint32_t)(((int32_t)s_sweep_ref +
                            (s_sweep_a1 + s_sweep_a2) /
                            (int32_t)(APP_CAL_SWEEP_HOLD - APP_CAL_SWEEP_DROP)) & 0x3FFF);

            /* 【整圈里程计】净行程 = 各步平均位置的解卷绕累加。
             * 它是扫描的停止依据: 净行程到 16384 就说明编码器走过整整一圈,
             * 每个 bin 都被真实测量过 —— 表不再有"编出来"的段落。 */
            if (s_sweep_step == 0U) {
                s_sweep_odo_prev = raw_avg;
                s_sweep_odo      = 0;
                s_sweep_dmax     = 0;
                s_sweep_dmin     = 0;
            } else {
                int32_t dd = cal_wrap_raw((int32_t)raw_avg -
                                          (int32_t)s_sweep_odo_prev);

                s_sweep_odo += dd;
                s_sweep_odo_prev = raw_avg;
                if (dd > s_sweep_dmax) { s_sweep_dmax = dd; }
                if (dd < s_sweep_dmin) { s_sweep_dmin = dd; }
                /* 一步几乎没动 -> 转子被齿槽/失步顶住了。连着 APP_CAL_SWEEP_STALL_MAX
                 * 步不动就提前收尾, 不能拿 1.5V 一直顶着烧。 */
                if ((dd < 2) && (dd > -2)) { s_sweep_stall++; }
                else                       { s_sweep_stall = 0U; }
            }
            /* 每走完一个电周期, 打印这一周期转子实际推进了多少计数。
             * 理想值恒为 16384/极对数 = 1365。
             * 若在整圈里忽大忽小(0.5x~2.4x) -> 编码器刻度不均匀(磁环偏心/倾斜);
             * 换过磁环后应当落在 1365±130 之内。 */
            if ((s_sweep_step % APP_CAL_SWEEP_PHASES) == 0U) {
                if (s_sweep_step > 0U) {
                    (void)snprintf(line, sizeof(line),
                                   "CALCYC %2u ADV=%ld (ideal %ld)",
                                   (unsigned)(s_sweep_step / APP_CAL_SWEEP_PHASES),
                                   (long)cal_wrap_raw((int32_t)raw_avg - (int32_t)s_cyc_ref),
                                   (long)(16384U / (uint32_t)FOC_MOTOR_POLE_PAIRS));
                    app_log_write_raw(line);
                }
                s_cyc_ref = raw_avg;
            }

            /* 磁场每步走 6283/PHASES mrad; 与下面的 align 公式严格一致 */
            uint32_t sw_mrad = (s_sweep_step * 6283U / APP_CAL_SWEEP_PHASES) % 6283U;

            /* ---- 把这个静态点记进 LUT (这是本模式存在的意义) ----
             * 【按 1/I 外推掉齿槽偏移】静平衡时 Kt·I·sinδ = 齿槽/摩擦转矩,
             * 所以 δ ∝ 1/I: 弱场段(0.30V, 1/5 电流)的偏移是强场段的 5 倍。
             * 两段位移差 dE = 4·δ_强, 减掉 dE/4 就得到"无限电流"下真正的
             * 对齐点 —— 这样标出来的表连静态齿槽偏移都没有。
             * |dE| 过大 = 弱场下转子已经跟不住磁场(齿槽太强), 这时不修正,
             * 只统计出来给人看。 */
            {
                int32_t d_e = cal_wrap_raw(e2 - e1);
                int32_t raw_use;
                int32_t vh  = (int32_t)(APP_CAL_SWEEP_HIGH_V * 1000.0f);
                int32_t vl  = (int32_t)(APP_CAL_SWEEP_LOW_V * 1000.0f);

                /* 修正量 = dE × Vl/(Vh - Vl) —— 由电压比算出来, 不再写死 /4;
                 * 改了电压而忘了改系数, 外推就会多补/少补 (方向还是对的, 量错)。 */
                if ((d_e >= -12) && (d_e <= 12)) {
                    raw_use = e1 - (int32_t)(((int32_t)d_e * vl) / (vh - vl));
                } else {
                    raw_use = e1;
                    s_sweep_de_big++;
                }
                diag_lut_feed_static((uint32_t)(raw_use & 0x3FFF), sw_mrad);

                if (s_sweep_de_n == 0U) {
                    s_sweep_de_min = d_e;
                    s_sweep_de_max = d_e;
                }
                if (d_e < s_sweep_de_min) { s_sweep_de_min = d_e; }
                if (d_e > s_sweep_de_max) { s_sweep_de_max = d_e; }
                s_sweep_de_acc += d_e;
                s_sweep_de_n++;

                if ((s_sweep_step % 4U) == 0U) {
                    (void)snprintf(line, sizeof(line),
                                   "CALSTEP %4u SW=%u ENC=%u E1=%ld E2=%ld dE=%ld DIFF=%ld",
                                   (unsigned)s_sweep_step,
                                   (unsigned)sw_mrad,
                                   (unsigned)raw_avg,
                                   (long)(e1 & 0x3FFF), (long)(e2 & 0x3FFF),
                                   (long)d_e,
                                   (long)resid);
                    app_log_write_raw(line);
                }
            }

            /* 残差汇总: 相对第一个样本解卷绕后累加, 避免 ±pi 附近出错 */
            if (s_sweep_res_n == 0U) {
                s_sweep_res_ref = resid;
                s_sweep_res_min = resid;
                s_sweep_res_max = resid;
            }
            s_sweep_res_acc += cal_wrap_mrad(resid - s_sweep_res_ref);
            s_sweep_res_n++;
            if (resid < s_sweep_res_min) { s_sweep_res_min = resid; }
            if (resid > s_sweep_res_max) { s_sweep_res_max = resid; }

            s_sweep_a1   = 0;
            s_sweep_a2   = 0;
            s_sweep_cnt  = 0U;
            s_sweep_step++;
            foc_controller_set_voltage(&s_ctrl, APP_CAL_SWEEP_HIGH_V);  /* 恢复标定电压(低压) */

            if ((s_sweep_step >= s_sweep_steps) || (s_sweep_odo >= 16384) ||
                (s_sweep_stall >= APP_CAL_SWEEP_STALL_MAX)) {
                /* ---- 自动给出结论 ----
                 * 残差均值 = 当前 offset 还差多少。推荐偏置 = 当前 + 残差均值。
                 * 若推荐值与当前值几乎相同 -> 校准已被验证通过。 */
                int32_t mean_res = s_sweep_res_ref +
                                   s_sweep_res_acc / (int32_t)s_sweep_res_n;
                int32_t rec = (int32_t)(app_encoder_get_elec_offset_rad() * 1000.0f) + mean_res;

                while (rec < 0)     { rec += 6283; }
                while (rec >= 6283) { rec -= 6283; }

                s_cal_offset_mrad = rec;   /* 存档用 */
                s_sweep_done = 1U;

                /* 【扫完自动恢复旋转, 不再关断输出】
                 * 这里原来写的是 foc_controller_enable(&s_ctrl, false),
                 * 目的是防止"堵转+顶限幅"持续发热。但副作用是【每次标定完
                 * 电机都停着不动】, 极易被误判成"校准把电机搞坏了"。
                 * 扫描全程只有 STEPS × HOLD ≈ 3.6 秒, 堵转时间很短,
                 * 直接恢复 20 RPM 旋转既安全又不产生困惑。 */
                foc_controller_set_speed_rpm(&s_ctrl, APP_OPENLOOP_TARGET_RPM);

                (void)snprintf(line, sizeof(line),
                    "CAL RESULT n=%u residual_mean=%ld spread=%ld..%ld",
                    (unsigned)s_sweep_res_n, (long)mean_res,
                    (long)s_sweep_res_min, (long)s_sweep_res_max);
                app_log_write_raw(line);

                /* 【齿槽偏移体检】dE = 强场段与弱场段的编码器位置差 (counts)。
                 * 静平衡: Kt·I·sinδ = 齿槽/摩擦转矩 => δ ∝ 1/I, 所以
                 *   dE = 4·δ_强(1.5V/0.3V 电流比 5:1)。
                 * dE 只有 ±1~2 counts -> 齿槽很弱, 静态标定非常干净;
                 * dE 到 ±5 counts 以上 -> 齿槽已经明显把转子拉偏, 换向误差里
                 *   有这部分"负载角", 它随位置变化, 就是残余的停顿来源;
                 * big = 弱场下转子跟不住(|dE|>12)的步数, 这些步没有做外推修正。 */
                (void)snprintf(line, sizeof(line),
                    "CAL DEOFF n=%u mean=%ld min=%ld max=%ld big=%u",
                    (unsigned)s_sweep_de_n,
                    (long)((s_sweep_de_n != 0U) ? (s_sweep_de_acc / (int32_t)s_sweep_de_n) : 0),
                    (long)s_sweep_de_min, (long)s_sweep_de_max,
                    (unsigned)s_sweep_de_big);
                app_log_write_raw(line);

                /* 【整圈收尾汇报】走了多少步 = 磁场转了几个电周期。转子吸稳时
                 * "磁场转 P 个电周期 = 编码器走一整圈", 所以 fieldrev 就是这台
                 * 电机【极对数的实测值】(配 12 而实际只走了 11.x 圈 -> 配置偏小
                 * 或中途丢步)。odo 目标 16384 = 一整圈; dmax/dmin 是单步行程的
                 * 极值, 出现大负数说明转子倒退/失步过。 */
                {
                    uint32_t steps = (s_sweep_step != 0U) ? s_sweep_step : 1U;
                    uint32_t pairs = (steps + (APP_CAL_SWEEP_PHASES / 2U))
                                     / APP_CAL_SWEEP_PHASES;

                    s_sweep_pairs = pairs;
                    (void)snprintf(line, sizeof(line),
                        "CAL SWEEP end: steps=%u fieldrev=%u odo=%ld dmax=%ld dmin=%ld",
                        (unsigned)s_sweep_step, (unsigned)pairs,
                        (long)s_sweep_odo, (long)s_sweep_dmax, (long)s_sweep_dmin);
                    app_log_write_raw(line);
                }

                if (s_sweep_stall >= APP_CAL_SWEEP_STALL_MAX) {
                    /* 转子顶着不动: 这次扫出来的表右半边是空的, 直接否决换向 */
                    s_lut_reject = 1U;
                    (void)snprintf(line, sizeof(line),
                        "CAL SWEEP ABORT: 转子连续 %u 步不动 (raw~%u) -> 不换向",
                        (unsigned)s_sweep_stall,
                        (unsigned)app_encoder_get_raw_angle(&s_encoder));
                    app_log_write_raw(line);
                }

                /* 【补空缺 + 体检】走满整圈后, 缺口应只剩"刻度太疏跳过 1~2 箱",
                 * 用左右精确点弦插值补上 (邻居是静态精确点, 误差极小)。补完
                 * bins=1024, 主循环里"表学满 -> 换向"那段既有逻辑会自动接手。
                 * 【否决条件】缺口一旦长于 1/4 电周期(16 箱), 弦插值的 6283 分支
                 * 就判不出来 (端点的短支 + 名义斜率给不出真值: 实测 46 箱的缺口
                 * 被补成多整整一个电周期的斜坡, 换向后转子正好在那段里反复弹),
                 * 此时【不换向】, 保持开环 VF, 而不是拿假表把转子焊住。 */
                {
                    uint32_t missed  = ENC_LUT_BINS - diag_lut_valid_bins();
                    uint32_t maxhole = diag_lut_max_hole();
                    uint32_t fixed   = diag_lut_repair();
                    uint32_t hb = 0U;
                    int32_t  hn = 0;
                    int32_t  hp = 0;
                    int32_t  hs = diag_lut_health(&hb, &hn, &hp);
                    int32_t  want = (int32_t)s_sweep_pairs * 6283;

                    (void)snprintf(line, sizeof(line),
                        "CAL LUT: missed=%u repaired=%u valid=%u maxhole=%u",
                        (unsigned)missed, (unsigned)fixed,
                        (unsigned)diag_lut_valid_bins(), (unsigned)maxhole);
                    app_log_write_raw(line);

                    /* sum 应 = 极对数 × 6283 (整圈恰好那么多电周期), back 越小越好 */
                    (void)snprintf(line, sizeof(line),
                        "CAL LUT health: sum=%ld (want %ld = %u rev) back=%u worst=%ld..%ld",
                        (long)hs, (long)want, (unsigned)s_sweep_pairs,
                        (unsigned)hb, (long)hn, (long)hp);
                    app_log_write_raw(line);

                    if ((maxhole > 16U) ||
                        ((hs - want) > 6283) || ((hs - want) < -6283)) {
                        s_lut_reject = 1U;
                        (void)snprintf(line, sizeof(line),
                            "CAL LUT REJECTED: maxhole=%u gap_sum=%ld -> 不换向, 保持开环 VF",
                            (unsigned)maxhole, (long)(hs - want));
                        app_log_write_raw(line);
                    }
                }

                /* 不用 %f (工程刻意避开浮点打印), 拆成整数+小数 */
                (void)snprintf(line, sizeof(line),
                    "CAL -> 写进 foc_config.h:  #define FOC_ELEC_ANGLE_OFFSET (%ld.%03ldf)",
                    (long)(rec / 1000), (long)(rec % 1000));
                app_log_write_raw(line);

                app_log_write_raw("CALSWEEP done -> 已恢复旋转 (20 RPM)");
            } else {
                /* 磁场绝对累加: 每步 2π/PHASES 电角度, 整圈正好走完 极对数 个电周期 */
                foc_controller_set_align_angle(&s_ctrl,
                    (float)s_sweep_step * FOC_TWO_PI / (float)APP_CAL_SWEEP_PHASES);
            }
        }
        }   /* else: 预吸稳结束, 开始记表 */
    }
#endif

    /* ---- 周期性上报 ---- */
    if (++tick >= APP_REPORT_TICKS) {
        char msg[256];

        tick = 0;

#if APP_TELEMETRY_PAIR_ONLY
        /* ---- 极简 CSV: 三个整数 ----
         *   1) 软件(模拟)电角度      mrad, 0..6283
         *   2) 编码器【电角度】       mrad, 0..6283 (含已标定的偏置)
         *   3) 编码器【原始 14bit】   0..16383   <-- 判断编码器本身是否正常
         * 转子匀速转时, 第 3 列应当单调、均匀地推进, 一圈正好走完 0..16383。
         * 若它出现回退/跳变/长时间不动, 就是编码器或磁环的问题。 */
        {
            int32_t sw = (int32_t)(foc_normalize_angle(app_vec_angle_rad())
                                   * 1000.0f);
            /* 第 2 列: 编码器【电角度】。这里刻意用【线性公式】(mech×极对数+偏置),
             * 而不是 app_encoder_get_elec_rad() —— 后者在用内置查表时就是表值,
             * 会和第 4 列完全重复。保留线性公式后, 第 2 列与第 4 列之差就是
             * "编码器自身的周期误差", 也就是这张表正在修正的东西。 */
            int32_t enc_elec = (int32_t)(foc_mech_to_elec(
                app_encoder_get_mech_rad(&s_encoder),
                app_encoder_get_elec_offset_rad()) * 1000.0f);
            int32_t enc_raw =
                (int32_t)app_encoder_get_raw_angle(&s_encoder);

#if APP_TELEMETRY_LUT
            /* 第 4 列: 由编码器 raw 查表得到的电角度。
             * 用内置离线表时就是正在驱动矢量的那个值;
             * 用运行时 diag_lut 时, 表项样本不足会返回 0xFFFFFFFF, 折算成 -1,
             * 方便串口侧直接看出"这个位置还没学到". */
            {
                int32_t  lut_mrad;
#if APP_ENC_USE_BUILTIN_LUT
                lut_mrad = (int32_t)app_enc_lut_mrad((uint16_t)enc_raw);
#else
                uint32_t lut = diag_lut_angle((uint32_t)enc_raw);

                if (lut == 0xFFFFFFFFU) {
                    lut_mrad = -1;          /* 表项样本不足 */
                } else {
                    lut_mrad = (int32_t)lut;
                    if (lut_mrad < 0) {
                        lut_mrad += 6283;   /* 统一到 0..6283, 便于和 sw 对照 */
                    }
                }
#endif
                (void)diag_csv_telemetry_lut(msg, sizeof(msg), sw, enc_elec,
                                             enc_raw, lut_mrad);
            }
#else
            (void)diag_csv_telemetry(msg, sizeof(msg), sw, enc_elec, enc_raw);
#endif
        }
#elif APP_TELEMETRY_CAL_ONLY
        /* 校准模式: 只输出 SW / ENC / ENCEL / DIFF */
        (void)diag_cal_telemetry(msg, sizeof(msg));
#else
        (void)diag_telemetry(msg, sizeof(msg),
                             s_ctrl.openloop.elec_speed / FOC_TWO_PI,
                             app_encoder_get_mech_rpm(&s_encoder),
                             bsp_adc_get_vbus_mv());
#endif

        /* 遥测原样输出, 复用日志后端 (console + USB) */
        app_log_write_raw(msg);
    }

    /* ---- 校准映射表转储 ----
     * 串口上报 (5Hz) 与转速 (0.333圈/s) 恰好成 15:1, 每次上报都落在同一批
     * 机械位置 —— 只看上面那行日志是【看不到曲线形状】的。这张表由 1kHz 的
     * 控制循环累积, 一次旋转就能把整圈 64 箱填满, 每 5 秒打印一次。 */
    if ((APP_CAL_MAP_DUMP_TICKS > 0U) && (++s_map_tick >= APP_CAL_MAP_DUMP_TICKS)) {
        char line[192];
        uint32_t i;

        s_map_tick = 0;

        /* 只在所有箱都收到过样本后才打印, 避免输出半张表 */
        for (i = 0U; i < CAL_MAP_BINS; i++) {
            if (diag_cal_map_count(i) == 0U) {
                break;
            }
        }
        if (i >= CAL_MAP_BINS) {
            for (i = 0U; i < CAL_MAP_LINES; i++) {
                (void)diag_cal_map_line(line, sizeof(line), i);
                app_log_write_raw(line);
            }
        }
    }

    /* ---- 编码器角度查表 (LUT) 整表转储 ----
     * APP_ENC_LUT_DUMP_TICKS = 每 N 次上报打印一次整张表; 0 = 关闭。
     * 64 行会阻塞控制环, 和校准映射表一样, 看完就关。
     * 每行: "LUTxxx v0 v1 ... v15", 单位 mrad, -1 表示该箱样本不足。 */
#if (APP_ENC_LUT_DUMP_TICKS > 0U)
    {
        static uint32_t s_lut_tick;

        if (++s_lut_tick >= APP_ENC_LUT_DUMP_TICKS) {
            char line[256];
            uint32_t valid = diag_lut_valid_bins();
            uint32_t i;

            s_lut_tick = 0;

            (void)snprintf(line, sizeof(line),
                           "LUTBINS %u/%u err=%u tfail=%u lastraw=0x%06lx",
                           (unsigned)valid, (unsigned)ENC_LUT_BINS,
                           (unsigned)bsp_mt6701_get_error_count(),
                           (unsigned)bsp_mt6701_get_transfer_fail(),
                           (unsigned long)bsp_mt6701_get_last_raw());
            app_log_write_raw(line);

            if (valid >= ENC_LUT_BINS) {
                for (i = 0U; i < ENC_LUT_LINES; i++) {
                    (void)diag_lut_dump_line(line, sizeof(line), i);
                    app_log_write_raw(line);
                }
            }
        }
    }
#endif
}
