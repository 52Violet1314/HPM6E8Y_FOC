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

/* FreeRTOS: 只为了 app_delay_ms() 里判断"调度器是否已启动"——
 * 调度器起来之后必须改用 vTaskDelay 让出 CPU, 否则忙等会把日志/显示任务饿死
 * (详见 app_delay_ms 的注释)。FreeRTOS.h 必须在 task.h 之前。 */
#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "app_diag.h"
#include "app_log.h"
#include "app_encoder.h"
#include "app_scope.h"      /* 电流环录波: ISR 里采样, 触发后转储成 CSV */
#include "app_enc_lut.h"    /* 离线标定的 3200 组 raw -> 电角度表 */
#include "app_motion.h"     /* 位置环 (外环, 输出速度给定) */
#include "app_nv.h"
#include "app_ecat.h"       /* 内置 EtherCAT(ESC) 开箱自检 + 回环链路测试 */
#include "shm_core.h"       /* log_backend_shm: 反向日志环(core0 -> core1 的 UART14) */

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_usb.h"
#include "bsp_button.h"     /* J11 按键 (位置点动) */
#include "bsp_mt6701.h"
#include "bsp_srma35.h"     /* 锐鹰 SRMA35: 17bit 单圈绝对值, RS485/UART11 */
#if BSP_MT6701_USE_SEI
#include "bsp_sei.h"
#else
#include "bsp_spi.h"
#include "bsp_spi_bb.h"
#endif

#include "foc_controller.h"
#include "LADRC.h"          /* 速度环: 一阶 ADRC (与 pid.c 二选一, 见 app.h) */

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

/* ---- ADC 相电流通路失效标志 (只在上电自检里置位, 之后一直为 1) ----
 * 置位含义: 电流采样没有可信数据。此时【任何闭环都不允许运行】,
 * 但系统仍然继续跑(日志/屏幕/诊断都活着), 而不是死循环卡住 ——
 * 见 app_init 里"ADC 存活门槛"的说明。 */
static uint8_t s_adc_dead;

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

/* ---- 延迟格式化日志的格式化回调 (定义在 app_run 之前, 但 app_init 里就要注册) ---- */
static void app_log_format_rec(const app_log_rec_t *rec, char *out, uint32_t out_sz);
static float            s_speed_ui_last_seen;    /* 上一拍看到的目标值, 用来判"变了" */

#if APP_FOC_SPEED_LOOP_ENABLE
/* 速度环: PID 或一阶 ADRC, 由 app.h 的 APP_SPEED_LOOP_ALG 选择。
 *   - PID : 通用 pid.c, 与加 ADRC 之前的行为完全一致(回退用);
 *   - ADRC: LADRC.c, 参数与推导见 app.h 的"速度环: 一阶 LADRC 参数"。
 * 两条路的输入(给定/实测)与输出(iq)完全同量纲, 所以切换只改这一处。
 *
 * 【为什么把 init/reset 也包成函数】速度环的"清状态"分散在 4 个地方
 * (方向标定收尾、过流锁死、iq 阶跃测试、使能开关), 之前每个地方都写
 * PID_Reset(&s_pid_spd)。换成 ADRC 后如果漏改一处, 残留的 z1/z2 会在
 * 重新使能时直接变成一记电流冲击 —— 所以统一走 app_speed_loop_reset()。 */
#if (APP_SPEED_LOOP_ALG == APP_SPEED_LOOP_ALG_ADRC)
static LADRC_typedef s_ladrc_spd;

static void app_speed_loop_init(float ts)
{
    LADRC_Init(&s_ladrc_spd,
               APP_SPEED_ADRC_B0,
               APP_SPEED_ADRC_WC_RAD_S,
               APP_SPEED_ADRC_WO_RAD_S,
               ts,
               APP_SPEED_ADRC_U_MAX_A);
}

static void app_speed_loop_reset(void)
{
    LADRC_Reset(&s_ladrc_spd);
}

/* 每拍把【实测控制周期】喂给观测器: 主循环被日志/显示任务挤长时, 写死 1ms
 * 会让 wo 的实际值按比例缩水(观测器变慢)。只在合理范围内更新, 排除首拍和
 * 调试器暂停造成的异常值。 */
static void app_speed_loop_set_dt(float dt)
{
    if ((dt > 0.0002f) && (dt < 0.01f)) {
        LADRC_SetTs(&s_ladrc_spd, dt);
    }
}
#else
static PID_Controller_t s_pid_spd;

static void app_speed_loop_init(float ts)
{
    PID_InitSym(&s_pid_spd, APP_FOC_SPEED_KP, APP_FOC_SPEED_KI,
                 ts, APP_FOC_SPEED_IQ_LIMIT_A, APP_FOC_SPEED_IQ_LIMIT_A);
}

static void app_speed_loop_reset(void)
{
    PID_Reset(&s_pid_spd);
}

static void app_speed_loop_set_dt(float dt)
{
    (void)dt;       /* 通用 PID 的 dt 在 init 里定死, 不支持逐拍变化 */
}
#endif
#endif /* APP_FOC_SPEED_LOOP_ENABLE */

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

/* 把 raw 差值折算到 (-HALF, HALF] —— 抗 131072->0 回绕 (17bit 编码器) */
static int32_t cal_wrap_raw(int32_t v)
{
    while (v > (int32_t)APP_ENC_RAW_HALF)  { v -= (int32_t)APP_ENC_RAW_MAX; }
    while (v < -(int32_t)APP_ENC_RAW_HALF) { v += (int32_t)APP_ENC_RAW_MAX; }
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
/* 【电流环抖动量化】20kHz 域统计 id/iq 峰峰值。
 * 为什么需要: 100Hz 的遥测采样对 kHz 级振荡只能看到混叠(前面好几轮都被它骗了,
 * 明明在振却看不出幅度)。这里在中断里记 min/max, 任务侧每秒读一次再清零 ->
 * 得到的是"真实摆幅", 直接对应啸叫强度。 */
static volatile float s_isr_id_min = 1e9f, s_isr_id_max = -1e9f;
static volatile float s_isr_iq_min = 1e9f, s_isr_iq_max = -1e9f;
/* 【定位"谁堵了中断"】中断被拖长时, 记下当时主循环处于哪个阶段(见 app_diag.h 的
 * APP_STAGE_LOOP_*): 11=VBUS 12=电流 15=编码器 16=LCD 17=日志 18=USB。
 * 用途: 15ms 级的中断停摆会让高速下的电机顿挫, 但光看现象猜不出是谁;
 * 这个值能直接指到人。 */
static volatile uint32_t s_isr_gap_stage;
static volatile uint32_t s_isr_gap_us;
#endif

/* 主循环实测周期(s), 供角度外推估计电角速度用 */
static float s_dt_main = 0.001f;

/* 延时: 初始化阶段用 mcycle 忙等; 调度器跑起来之后【必须让出 CPU】
 *
 * 【为什么必须区分 —— 实测踩坑】
 *   延迟自检(app_run 里)会调用 app_init_run_cycles(), 它内部是 app_delay_ms(1)。
 *   如果这里永远是忙等, 那么控制任务(最高优先级)在整个探测期间 100% 占着 CPU,
 *   日志任务(优先级 -3)和显示任务(-4)【一次都轮不到】—— 现象是:
 *   LCD 之后串口一行都不再输出、屏幕也不刷新, 看起来像"卡死", 其实是饿死。
 *   所以调度器一旦启动就改用 vTaskDelay 主动让出; 初始化早期(还没调度器)
 *   仍然用忙等, 因为那时 vTaskDelay 不可用。 */
static void app_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }

    {
        uint64_t t0 = hpm_csr_get_core_mcycle();
        uint64_t n  = ((uint64_t)hpm_core_clock * ms) / 1000U;

        while ((hpm_csr_get_core_mcycle() - t0) < n) {
            /* busy wait */
        }
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

/* ================= 延迟自检专用: 相电流极性/相序探测 =================
 *
 * 【为什么需要它】延迟自检这条路绕过了 app_init 里那段"四角度极性自检",
 * 导致 s_cur_sign 停在默认 +1。实测后果:
 *     延迟自检完成后立刻
 *     OVERCURRENT 2563mA (ia=2563 ib=-662 ic=-1901) id=-1478 iq=2212 sign=1
 * 也就是电流环拿到的是【反号】反馈 -> PI 一路顶到限幅 -> 真实电流冲到 2.5A ->
 * 过流保护锁死。所以必须在使能电流环之前, 用【真实可用的 ADC】把极性测出来。
 *
 * 【判据】开环给一个固定电压矢量, 在电角度 0° 和 90° 各钉一次, 读三相电流:
 *   角度 0° : 施加的矢量在 +alpha 方向 -> 实测 alpha 分量为正 = 极性正
 *   角度 90°: 施加的矢量在 +beta  方向 -> 实测 beta 分量为正 = 相序不镜像
 * 两者合起来既能定极性(整体取反), 也能发现 V/W 互换(镜像)。
 *
 * 【安全性】电压只用 APP_ADC_PROBE_V(0.5V), 电流约 V/R ≈ 2A, 每次只保持
 * 60ms; 测完立刻关控制器、写零矢量。失败(电流太小无法判断)时返回 0,
 * 调用者必须保持电机封锁 —— 绝不在"没测出极性"的情况下使能电流环。
 *
 * @return bit0 = 成功, bit1 = 极性反(需要 s_cur_sign = -1), bit2 = 相序镜像
 *         0 = 失败/无法判断 */
uint8_t app_probe_current_polarity(float vbus)
{
    static const float ang[2] = {0.0f, 1.5707963f};
    float va[2];        /* 每个角度测到的 alpha 分量 (mA) */
    float vb[2];        /* 每个角度测到的 beta  分量 (mA) */
    uint8_t k;

    s_ctrl.openloop.voltage_slew = 100.0f;      /* 默认 1V/s 太慢, 这里快速到位 */
    foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_openloop);
    foc_controller_set_voltage(&s_ctrl, APP_ADC_PROBE_V);
    foc_controller_enable(&s_ctrl, true);

    for (k = 0U; k < 2U; k++) {
        uint32_t i;
        float    sa = 0.0f, sb = 0.0f, sc = 0.0f;

        foc_controller_set_align_angle(&s_ctrl, ang[k]);

        for (i = 0U; i < 80U; i++) {
            int32_t ia = 0, ib = 0, ic = 0;

            app_init_run_cycles(1U, vbus);

            /* 只统计后半段(前半段在爬电压/建立电流)。
             * 【40 拍 = 40ms】实测 0.5V 时信号只有 140mA 而噪声可达 80mA,
             * 平均窗口太短会判错; 加长到 40 拍把噪声压下去(见 app.h 的说明)。 */
            if (i >= 40U) {
                bsp_adc_get_currents_ma(&ia, &ib, &ic);
                sa += (float)ia;
                sb += (float)ib;
                sc += (float)ic;
            }
        }
        sa /= 40.0f; sb /= 40.0f; sc /= 40.0f;

        va[k] = sa;                                  /* Clarke: alpha = ia */
        vb[k] = (sb - sc) * 0.57735f;                /* beta = (ib - ic)/sqrt(3) */
    }

    foc_controller_enable(&s_ctrl, false);
    bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
    s_ctrl.openloop.voltage_slew = FOC_OPENLOOP_VOLTAGE_SLEW;
    foc_controller_set_mode(&s_ctrl, foc_ctrl_mode_current);

    LOG_I("ADC", "polarity probe: 0deg a=%d b=%d mA, 90deg a=%d b=%d mA",
          (int)va[0], (int)vb[0], (int)va[1], (int)vb[1]);

    /* 幅值门限: 0.5V / 约 0.25Ω ≈ 2A, 实测几百 mA 以上才可信 */
    if ((va[0] > -APP_ADC_PROBE_MIN_MA) && (va[0] < APP_ADC_PROBE_MIN_MA)) {
        LOG_I("ADC", "!! polarity probe 失败: 0deg 的 alpha 分量只有 %d mA (需 >%d),"
                     " 无法判断极性 -> 保持封锁", (int)va[0], (int)APP_ADC_PROBE_MIN_MA);
        return 0U;
    }

    {
        uint8_t res = 0x01U;                        /* bit0 = 成功 */

        if (va[0] < 0.0f) {
            res |= 0x02U;                           /* 极性反 */
        }
        /* 极性归一化后, 90° 处 beta 应为正; 为负说明相序镜像 */
        if ((vb[1] * ((va[0] < 0.0f) ? -1.0f : 1.0f)) < 0.0f) {
            res |= 0x04U;                           /* 相序镜像 */
        }
        return res;
    }
}

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

/* USB 虚拟串口后端 (仅主机已连接时输出; APP_LOG_USB_ENABLE=0 时整段不编译) */
#if APP_LOG_USB_ENABLE
static void log_backend_usb(const char *line)
{
    if (bsp_usb_connected()) {
        diag_set_stage(APP_STAGE_LOOP_USB);   /* 给中断诊断标记: 正在写 USB */
        (void)bsp_usb_write((const uint8_t *)line, (uint32_t)strlen(line));
        (void)bsp_usb_write((const uint8_t *)"\r\n", 2);
    }
}
#else
static void log_backend_usb(const char *line)
{
    (void)line;     /* 见 app.h 的 APP_LOG_USB_ENABLE: 调电流环时关掉,
                     * 免得 USB 提交把 ADC 中断推迟 250us */
}
#endif

/* ---- 反向日志环后端: core0 的日志交给 core1 从 UART14 输出 ----
 * 【为什么需要】core0 自己的两个口都指望不上: USB CDC 掉线就没了, UART0 是板上
 * 调试口(不一定接出来)。而 core1 有一条独立的 UART14 —— 于是把日志推进共享内存的
 * 反向环(log0), 由 core1 的显示任务取走直接写到那根线上。两个核的日志最终出现在
 * 同一个串口, core0 这边一根线都不用接。
 * 【开销】一次 strlen + memcpy 到 non-cacheable 共享区(约 200 字节), 在日志任务里跑。
 * 环满直接丢并计数(shm->log0_dropped), 绝不等待。 */
static void log_backend_shm(const char *line)
{
    (void)shm_log0_push(shm_get(), line);
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

        /* 20kHz 域记录 id/iq 的极值(见 s_isr_id_min 处的说明) */
        {
            float idv = s_ctrl.foc.id;
            float iqv = s_ctrl.foc.iq;

            if (idv < s_isr_id_min) { s_isr_id_min = idv; }
            if (idv > s_isr_id_max) { s_isr_id_max = idv; }
            if (iqv < s_isr_iq_min) { s_isr_iq_min = iqv; }
            if (iqv > s_isr_iq_max) { s_isr_iq_max = iqv; }
        }
    }
#endif

    /* 【电流环录波: 每拍记一组】放在闭环分支【外面】, 这样开环/调试模式下也能录。
     * 只做 6 个 int16 写入 + 一次指针自增(几十个周期), 对 20kHz 没有影响;
     * 转储由 app_scope_poll() 在控制任务里节流慢慢打(见 app_scope.h)。 */
    app_scope_sample((int32_t)(s_ctrl.foc.id_ref * 1000.0f),
                     (int32_t)(s_ctrl.foc.iq_ref * 1000.0f),
                     (int32_t)(s_ctrl.foc.id * 1000.0f),
                     (int32_t)(s_ctrl.foc.iq * 1000.0f),
                     (int32_t)(s_ctrl.foc.ud * 1000.0f),
                     (int32_t)(s_ctrl.foc.uq * 1000.0f));
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

/* ================= 方向标定 (物理量判据) =================
 *
 * 【判据】把速度环【整个旁路】, 直接给一个固定 iq, 看编码器【原始角】往哪边走:
 *     正 iq 让 raw 增大 -> 编码器正方向与 iq 正方向一致 -> s_speed_dir = +1
 *     正 iq 让 raw 减小 -> 反向                          -> s_speed_dir = -1
 * 这个判据里没有任何反馈环, 符号不会被绕回去; 位置环此时还没使能, 也不会把电机
 * 按住导致"测不出来"。这里是唯一能把"编码器计数方向 × 相电流极性 × 换向角约定"
 * 三者合成符号【实测】出来的地方 —— 速度环和位置环必须与它同号, 否则就是正反馈。
 *
 * 【为什么抽成函数】原来这段只写在 app_init 的"第四步"里, 而这块板子上电时
 * ADC 还没活(frames=0), app_init 在第二步就跑 return 了 —— 于是方向标定
 * 【从来没执行过】, s_speed_dir 一直是静态默认值 +1。实测后果: 速度环/位置环
 * 符号未验证, 一使能就是正反馈(电机原地 ±1500mA、±145RPM 剧烈震荡)。
 * 现在延迟自检路径也会调用它(app_run 里, 在使能位置环之前)。
 *
 * 【前提】电机必须【空载】—— 带载时转子被负载拖住, 判据不可信。 */
#if APP_FOC_CLOSED_LOOP_ENABLE && APP_FOC_SPEED_LOOP_ENABLE && APP_SPEED_DIR_AUTOTEST
static void app_calib_speed_dir(float vbus_f)
{
    uint16_t raw_before;
    int32_t  raw_delta;
    uint32_t i;

    s_speed_dir = APP_SPEED_DIR_SIGN;

    /* 判据用"原始编码器角"而不是滤波后的 rpm: 后者是 alpha=0.1 的低通,
     * 短窗口里远没跟上真实速度, 取符号等于在噪声里猜。 */
    raw_before = (int32_t)app_encoder_get_raw_angle(&s_encoder);

    /* 旁路速度环: 直接给固定 iq (d 轴 0) */
    foc_controller_set_current_ref(&s_ctrl, 0.0f, APP_SPEED_DIR_TEST_IQ_A);

    for (i = 0U; i < (APP_SPEED_DIR_TEST_MS / APP_MAIN_LOOP_MS); i++) {
        app_init_run_cycles(1U, vbus_f);
    }

    raw_delta = (int32_t)app_encoder_get_raw_angle(&s_encoder) - (int32_t)raw_before;
    /* 17bit 编码器, 按最短路径解卷绕(窗口内转角远小于半圈) */
    if (raw_delta > (int32_t)APP_ENC_RAW_HALF) {
        raw_delta -= (int32_t)APP_ENC_RAW_MAX;
    } else if (raw_delta < -(int32_t)APP_ENC_RAW_HALF) {
        raw_delta += (int32_t)APP_ENC_RAW_MAX;
    }

    /* 收尾: 电流归零, 交回外环。状态必须清 —— 测试期间转子会转到几十 RPM,
     * 速度环会攒下"相当于饱和出力"的积分/扰动估计。 */
    foc_controller_set_current_ref(&s_ctrl, APP_FOC_ID_REF_A, 0.0f);
    s_speed_target_rpm = 0.0f;
    app_speed_loop_reset();

    /* 幅值门限 1600 counts ≈ 4.4° (17bit: 131072/360*4.4): 远大于编码器噪声,
     * 又远小于本窗口的正常转角。动得太少(被齿槽/静摩擦顶住)时不要随手判方向。 */
    if ((raw_delta > 1600) || (raw_delta < -1600)) {
        s_speed_dir = (raw_delta < 0) ? -APP_SPEED_DIR_SIGN : APP_SPEED_DIR_SIGN;
        LOG_I("FOC", "方向标定: iq=+%d mA x %dms -> raw %u -> %u (d=%ld), s_speed_dir=%d",
              (int)(APP_SPEED_DIR_TEST_IQ_A * 1000.0f), (int)APP_SPEED_DIR_TEST_MS,
              (unsigned)raw_before,
              (unsigned)app_encoder_get_raw_angle(&s_encoder),
              (long)raw_delta, (int)s_speed_dir);
    } else {
        s_speed_dir = APP_SPEED_DIR_SIGN;
        LOG_I("FOC", "!! 方向标定失败: iq=+%d mA x %dms 只让 raw 动了 %ld counts "
                     "(需要 >1600) -> 转子可能被顶住/带载。保留静态 dir=%d, "
                     "请空载重试或加大 APP_SPEED_DIR_TEST_IQ_A",
              (int)(APP_SPEED_DIR_TEST_IQ_A * 1000.0f), (int)APP_SPEED_DIR_TEST_MS,
              (long)raw_delta, (int)s_speed_dir);
    }
}
#else
static void app_calib_speed_dir(float vbus_f)
{
    (void)vbus_f;      /* 未启用方向自检: 保持 APP_SPEED_DIR_SIGN 静态值 */
}
#endif

/* ================= 电流环"交棒"给 20kHz 中断 =================
 *
 * 【为什么必须收成一个函数】
 *   电流环的积分步长 (foc->ts) 必须等于它【实际被调用的周期】:
 *       主循环驱动 -> 1ms (foc_controller_init 给的 dt)
 *       中断驱动   -> 1/20000 = 50us
 *   Ki 是乘 dt 用的, 两者差 20 倍, 用错就等于把 Ki 放大/缩小 20 倍 ——
 *   这个环在 1kHz 上整定得很好, 换到 20kHz 上就是自激。
 *
 * 【实测事故 (2026-10-20, 就是本轮"还是啸叫"的根因)】
 *   这块板子上电时 ADC 出帧比 app_init 晚(实测 app_init 里等 300ms 仍然 frames=0,
 *   app_init 一返回就 19944 帧/秒), 于是 app_init 在"ADC 一帧都没采到"那一步
 *   提前 return —— 而把 ts 改成 1/20000 的那行代码在它【后面】, 从来没执行过。
 *   随后"延迟自检"路径补做了标定/极性/方向, 直接把 s_isr_closed_loop 置 1,
 *   而 foc->ts 还停在 foc_controller_init() 给的 1ms。后果:
 *       Ki_eff = 5844 × 20 = 116880  (等效把 300Hz 的环当 6kHz 调)
 *       开环穿越频率从 300Hz 抬到约 1.5kHz, 相位裕度 -60° -> 自激
 *   日志上的形状完全对得上: idpp≈1.5A / iqpp≈1.0A(给定只有 0.3A)、
 *   ud/uq 满摆 ±4~5V(=在限幅圆上打转)、电机啸叫、转速恒 0、
 *   而位置环/速度环接在这样一个内环后面, 无论怎么调都不可能跟踪。
 *
 *   现在"改 ts"和"置 s_isr_closed_loop"被绑在一起, 只有这一个入口,
 *   两条使能路径(正常 app_init / 延迟自检)都必须从这里过, 不可能再漏一个。 */
static void app_current_loop_handover_to_isr(void)
{
#if APP_CURRENT_LOOP_IN_ISR
    foc_controller_set_dt(&s_ctrl, 1.0f / (float)APP_ADC_ISR_FREQ_HZ);
    s_isr_closed_loop = 1U;
    LOG_I("FOC", "current loop -> ADC ISR @%u Hz (ts=%u us), ADC trig cmp=%u/%u",
          (unsigned)APP_ADC_ISR_FREQ_HZ, (unsigned)(1000000U / APP_ADC_ISR_FREQ_HZ),
          (unsigned)bsp_pwm_get_adc_trig_count(), (unsigned)bsp_pwm_get_reload());
#else
    /* 电流环留在主循环里: ts 就是 app_init 一开始给的那个 dt, 不要动 */
    foc_controller_set_dt(&s_ctrl, (float)APP_MAIN_LOOP_MS * 0.001f);
    LOG_I("FOC", "current loop runs in main loop @%u Hz (ts=%u us)",
          (unsigned)(1000U / APP_MAIN_LOOP_MS), (unsigned)(APP_MAIN_LOOP_MS * 1000U));
#endif
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
    /* 再挂一个"共享内存反向环"后端: core0 的日志由 core1 从 UART14 转发出去。
     * 这样即使 USB CDC 掉线、UART0 没接, core0 的日志依然看得到。 */
    app_log_add_backend(log_backend_shm);
    /* 注册"延迟格式化"的格式化回调: 热路径日志(app_log_post)只传整数,
     * 拼字符串由日志任务做 —— 见 app_log_format_rec。 */
    app_log_set_formatter(app_log_format_rec);

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

    /* 锐鹰 SRMA35: 17bit 单圈绝对值, RS485/UART11 (PD15=TX PD14=RX PD13=DE) */
    bsp_srma35_init();

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

#if APP_ECAT_SELFTEST
    /* 内置 EtherCAT(ESC) 开箱自检: 只点亮硬件 + 读链路状态, 不涉及协议栈。
     * 回环测试(网线把 J1<->J2 对接)期间保留; 测完把 app_ecat.h 里的
     * APP_ECAT_SELFTEST 改成 0, 这一整段就消失, 也不再有开机等待。 */
    app_ecat_selftest();
#endif

    /* 统一 FOC 控制器: 默认开环模式 */
    foc_controller_init(&s_ctrl, dt);
    foc_controller_set_voltage(&s_ctrl, APP_OPENLOOP_VOLTAGE_V);
    foc_controller_set_current_limit(&s_ctrl, APP_OPENLOOP_CURRENT_LIMIT);

#if APP_FOC_SPEED_LOOP_ENABLE
    /* 速度环初始化 (PID 或 ADRC, 由 app.h 的 APP_SPEED_LOOP_ALG 决定):
     *   - 输出限幅 = ±APP_FOC_SPEED_IQ_LIMIT_A (ADRC 复用同一个 iq 上限);
     *   - dt = 控制周期 APP_MAIN_LOOP_MS (速度环在 1kHz 主循环里跑);
     *   - ADRC 的 b0 推导 / KV54 / J / 带宽都写在 app.h 那一段里。
     * 【注意】这个 init 必须在 app_init 的早期(ADC 自检之前)完成: 那条 degraded
     * 路径会提前 return, 但速度环在 app_run 里照样每拍被调用。 */
    app_speed_loop_init((float)APP_MAIN_LOOP_MS * 0.001f);
#if (APP_SPEED_LOOP_ALG == APP_SPEED_LOOP_ALG_ADRC)
    LOG_I("ADRC", "速度环 ADRC: KV=%d RPM/V, J=%d (1e-7 kg.m2), Kt=%d mN.m/A, "
                  "b0=%d RPM/(s.A), wc=%d rad/s, wo=%d rad/s, iq_max=%d mA",
          (int)APP_SPEED_ADRC_KV_RPM_PER_V,
          (int)(APP_SPEED_ADRC_J_KG_M2 * 1.0e7f),
          (int)(APP_SPEED_ADRC_KT_NM_PER_A * 1000.0f),
          (int)APP_SPEED_ADRC_B0,
          (int)APP_SPEED_ADRC_WC_RAD_S,
          (int)APP_SPEED_ADRC_WO_RAD_S,
          (int)(APP_SPEED_ADRC_U_MAX_A * 1000.0f));
#else
    LOG_I("APP", "速度环 PID: kp=%d uA/RPM, ki=%d uA/(RPM.s), iq_max=%d mA",
          (int)(APP_FOC_SPEED_KP * 1.0e6f),
          (int)(APP_FOC_SPEED_KI * 1.0e6f),
          (int)(APP_FOC_SPEED_IQ_LIMIT_A * 1000.0f));
#endif
#endif

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
        bool           nv_ok  = false;

#if !APP_CAL_FORCE
        nv_ok = app_nv_load(nv_tbl, &nv_off);
#else
        LOG_I("ENC", "NV: APP_CAL_FORCE=1 -> ignore saved table, recalibrate");
#endif

        if (nv_ok) {
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
          (unsigned)APP_ENC_LUT_BINS, (float)APP_ENC_RAW_MAX / (float)APP_ENC_LUT_BINS,
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

    /* ================= ADC 存活门槛 (必须在标定/自检之前) =================
     *
     * 【为什么加这道门】实测踩过一次非常隐蔽的坑:
     *   PWM 计数器已启动、ADC 中断也使能了, 但【ADC 一帧都没采到】。
     *   此时 s_cur_u_raw 还是 0, 而零点 s_cur_u_offset 是 32767, 于是
     *   bsp_adc_get_currents_ma() 算出的是 **满量程负电流** (-16499mA)。
     *   后果:
     *     - 零点标定拿到的是 (0-32767), 记进 offset 后整个电流反馈永久偏掉;
     *     - 电流传感自检在"量程值"上打分 -> score=0 却仍然选出 sign=-1/map=0/1/2;
     *     - 电流环拿着假反馈积分 -> iq 顶限幅、uq 顶限幅、电机被盲推着转/抖。
     *   之前排查"为什么电机自己转/震荡"绕了很多圈, 根子就在这里:
     *   **电流环从头到尾都没有可信反馈, 却一直在闭环运行。**
     *
     * 【怎么判定"活着"】看 bsp_adc_get_frame_count(): 它只在 ADC 中断里自增。
     * 给 PWM/ADC 一点时间(实测 <20ms 就应该有几百帧), 等不到就判死。
     *
     * 【判死之后怎么办 —— 这里曾经写错过】
     *   第一版是"封锁输出 + while(1) 死循环打印"。这是个坏设计:
     *   板子会永久卡在 app_delay_ms() 的 mcycle 忙等里, 串口只剩一行重复,
     *   屏幕不再刷新、日志任务也不再跑 —— 看上去就是"程序卡死在
     *   hpm_csr_get_core_mcycle()", 既没法继续调试, 也没法判断是软件还是硬件。
     *   现在改成【降级运行】: 封锁 PWM 输出、跳过下面所有自检/标定、把电机
     *   永久保持在未使能状态, 但 app_init 正常返回、app_run 继续跑循环 ——
     *   串口能持续输出状态, 屏幕照常刷新, 你能看到"到底是什么坏了"。
     *   电流环没有可信反馈时, 电机唯一该处的状态就是"不出力"。 */
    {
        uint32_t wait_ms;
        uint32_t frames = 0U;

        for (wait_ms = 0U; wait_ms < APP_ADC_ALIVE_WAIT_MS; wait_ms++) {
            frames = bsp_adc_get_frame_count();

            if (frames >= APP_ADC_ALIVE_MIN_FRAMES) {
                break;
            }
            app_delay_ms(1U);
        }

        if (frames < APP_ADC_ALIVE_MIN_FRAMES) {
            /* 没有电流反馈: 永远不许使能电机 */
            foc_controller_enable(&s_ctrl, false);
            bsp_pwm_enable_output(false);
            s_adc_dead = 1U;

            LOG_I("ADC", "!! ADC 相电流一帧都没采到 (frames=%u, 等了 %ums) -> "
                         "封锁输出并跳过全部自检, 电机保持不出力。"
                         "请检查: ADC 时钟 / TRGM 触发(PWM0 TRGO) / DMA 缓冲 / 采样电阻供电。",
                  (unsigned)frames, (unsigned)APP_ADC_ALIVE_WAIT_MS);
        } else {
            LOG_I("ADC", "alive: %u frames in <=%ums (%.1f kHz)",
                  (unsigned)frames, (unsigned)APP_ADC_ALIVE_WAIT_MS,
                  (float)frames / ((float)APP_ADC_ALIVE_WAIT_MS * 0.001f) / 1000.0f);
        }
    }

    /* ADC 没活过来: 跳过零点标定/极性自检/使能, 直接返回。
     * app_run() 里还有一道 s_adc_dead 检查会把闭环彻底挡住。 */
    if (s_adc_dead != 0U) {
        LOG_I("APP", "degraded mode: 电流采样不可用, 本次上电只做诊断, 电机不会转动");
        LOG_I("PWM", "reload=%u, freq=%u Hz (output OFF)",
              (unsigned)bsp_pwm_get_reload(), (unsigned)BSP_PWM_FREQUENCY_HZ);
        return;
    }

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

        /* 【再判一次】零点必须离中点不远。若 offset 恰好等于 32767(默认值)且
         * 标定期间一帧新数据都没有, 说明 calibrate() 提前退出了——
         * 那时 offset 是"猜"的, 不能拿去闭环。同样降级, 不死循环。 */
        if (((int32_t)uo == mid) && ((int32_t)vo == mid) &&
            (bsp_adc_get_frame_count() < APP_ADC_ALIVE_MIN_FRAMES)) {
            foc_controller_enable(&s_ctrl, false);
            bsp_pwm_enable_output(false);
            s_adc_dead = 1U;
            LOG_I("ADC", "!! 零点标定没拿到有效样本, 电流反馈不可信 -> 封锁输出, 电机不出力");
            return;
        }
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
     * (自检阶段 s_isr_closed_loop=0, 那时由主循环直接驱动控制器)
     * 【两件事必须在同一个函数里做】见 app_current_loop_handover_to_isr 的说明:
     * 只置标志不改 ts, 就是 20 倍积分增益的自激。 */
    app_current_loop_handover_to_isr();
#endif
#if APP_MOTION_ENABLE
    /* 位置环: 只做初始化, 【先不要使能】。
     *
     * 【为什么必须等到方向自检之后】自检的做法是强行把速度给定设成 +6 RPM
     * 跑 6ms, 靠"实测 rpm 往哪边走"判方向。如果位置环已经使能, 它会把电机
     * 死死按在原地 —— 自检看到 rpm 几乎不动, 判据失效(实测会误判成"方向正常"),
     * 于是带着错误的方向符号进入位置环, 一使能就是正反馈:
     *     pos 从 4746 一路漂到 -8500000 mrad, err 顶满限幅, spdref=+60 饱和,
     *     而 rpm=-553 —— 位置环越使劲、电机往反方向跑得越快。
     * 所以顺序必须是: 电流环 -> 方向自检 -> 位置环使能(见下面)。 */
    app_motion_init(&s_motion);
#endif

    /* ================= 第四步: 方向标定 (物理量判据) =================
     *
     * 【为什么不能"给定转速看 rpm 符号"】上一版就是这么干的, 实测会判错 ——
     * 那个判据自己把符号绕回去了:
     *     速度环内部算的是 target * s_speed_dir - rpm
     *   拿 s_speed_target_rpm 当刺激, 刺激要经过 s_speed_dir 才变成真正的给定;
     *   只要 s_speed_dir 初值和实际相反, 环路会自己把"正刺激"翻成"负出力",
     *   而测得的速度符号又跟着翻 —— 读到的是"环路的行为", 不是"编码器和 iq
     *   的物理关系"。实测两次: 一次 rpm=-553/spdref=+60, 一次 rpm=+566/
     *   spdref=-60, 两次都是"给定与实际反号"的正反馈。
     *
     * 【现在的判据: 直接给固定 iq, 量速度怎么变】
     *   把速度环【整个旁路】(直接 foc_controller_set_current_ref), 给
     *   +APP_SPEED_DIR_TEST_IQ_A 这个很小的电流, 看实测 rpm 的变化方向:
     *       rpm 变大 -> 正 iq 让编码器 rpm 变大 -> s_speed_dir = +1
     *       rpm 变小 -> 正 iq 让编码器 rpm 变小 -> s_speed_dir = -1
     *   这个判据里没有任何反馈环, 符号不会被绕回去; 位置环此刻也还没使能,
     *   不会被位置环按住而"测不出来"。
     *
     * 【为什么必须做】位置环和速度环必须同号, 否则就是正反馈(位置环越使劲,
     * 电机往反方向跑得越快 —— 实测 pos 漂到 -8500000 mrad)。这里是唯一能把
     * "编码器计数方向 × 相电流极性 × FOC 角度约定"三者合成符号实测出来的地方。
     *
     * 【前提】电机必须【空载】—— 带载时转子被负载拖住/反转, 判据不可信。 */
    /* 方向标定: 抽成函数, 延迟自检路径也会调用(见 app_calib_speed_dir) */
    app_calib_speed_dir((float)bsp_adc_get_vbus_mv() * 0.001f);
#if APP_MOTION_ENABLE
    LOG_I("FOC", "speed loop: dir=%d, target=%d RPM (位置环每拍会重写它)",
          (int)s_speed_dir, (int)s_speed_target_rpm);
#endif

    /* ================= 第四步之二: 位置环使能 (必须在方向自检之后) =================
     *
     * 现在 s_speed_dir 已经由自检定下来了, 位置环才能安全使能:
     *   - app_motion_enable() 会把"当前位置"设为目标(原地保持), 不会一上电就冲;
     *   - 位置环内部取的方向系数就是上面自检定下的那一个,
     *     所以位置反馈和速度反馈必然同号, 不会再出现"位置环越使劲、
     *     电机反向飞转"的正反馈。
     * 【顺序不能反过来】反过来就是上一版实测到的现象(位置漂到 -8500000 mrad)。
     *
     * 【必须先读一次编码器!】这是一个独立的坑:
     *   在 app_init 走到这里之前, s_encoder.mech_rad 一直是 0
     *   (app_encoder_init() 只做 memset)。若直接 app_motion_enable(...,
     *   app_encoder_get_mech_rad()) 传进去的就是 0 -> 目标被设成 0,
     *   而转子实际停在某个非零角度(14bit 任意值) -> 上电瞬间就有一个
     *   最大 2π 的位置误差 -> 位置环立刻满力去"追 0 度"。
     *   所以这里先 poll 一次再 update, 让 mech_rad 是真实值。
     *   实测里 ref=4746 mrad 却 pos=344137 mrad 这种"一开始就差很远"的现象,
     *   有一部分就是这个原因造成的。 */
#if APP_MOTION_ENABLE
    (void)bsp_mt6701_poll();
    (void)app_encoder_update(&s_encoder);
    app_motion_enable(&s_motion, true, app_encoder_get_mech_rad(&s_encoder));
    LOG_I("MOT", "position loop ON: pos0=%d mrad, kp=%d mRPM/rad, 速度限幅=%d RPM, dir=%d",
          (int)(s_motion.pos_fb * 1000.0f),
          (int)(APP_MOTION_POS_KP * 1000.0f), (int)APP_MOTION_SPEED_LIMIT_RPM,
          (int)s_speed_dir);
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

float app_speed_get_dir(void)
{
    /* 位置环用它保证和速度环同号, 见 app.h 的说明 */
    return s_speed_dir;
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

/* ================= 延迟格式化 + 错峰投递 =================
 *
 * 【为什么】每秒一拍的结算窗口里有 4 条长日志(ctrl period / ADC ISR / 电流环峰
 * 峰值 / FOC 状态行), 合计约 500 字符、25 个转换。原来它们在同一拍里【连续
 * vsnprintf】—— 这一拍控制任务多花 100~200us(占 1ms 周期的 10~20%), 外环晚一拍,
 * 现象就是"打印一下电机顿一下"。
 *
 * 现在做两件事:
 *   1) 延迟格式化: 控制任务只把【已经算好的整数】塞进 app_log_rec_t 入队(约 0.3us),
 *      snprintf 交给日志任务(见 app_log.c 的 app_log_task_body);
 *   2) 错峰: 窗口结算时先把几条记录【快照】下来, 之后每拍只投递一条 ——
 *      5 条摊到 5ms, 不再挤在同一拍, 串口那边也不会一次涌出 500 字符。
 *
 * 判读提示: "ctrl period: max=" 那一行如果原来明显大于 avg, 改完应该基本贴住 avg。
 */
enum {
    APP_LR_CTRL_PERIOD = 1,     /* a0..a6: avg整,avg小, max整,max小, warn_ms, bad, cnt */
    APP_LR_ADC_ISR,             /* a0..a2: rate, expect, dtmax_us */
    APP_LR_IQ_PP,               /* a0..a1: idpp_mA, iqpp_mA */
    APP_LR_FOC_LINE,            /* a0..a13: id,iq,idref,iqref,ud,uq,vbus,rpm,dir,pos,ref,err,spdref,raw */
    APP_LR_ENC_HEALTH,          /* a0..a13: reader(0=spi2,1=bitbang),bb,miso,fr,er,tfail,raw,last,step,steps,odo,stall,lutfull,lutreject */
    APP_LR_SPEED_ADRC           /* a0..a5: ref,rpm,z1,err,z2(RPM/s),iq_mA */
};

/* 格式化回调: 【跑在日志任务里】, 不在控制任务里 —— 这就是"延迟格式化"的全部意义 */
static void app_log_format_rec(const app_log_rec_t *r, char *out, uint32_t out_sz)
{
    switch (r->id) {
#if APP_LOG_CTRL_PERIOD
    case APP_LR_CTRL_PERIOD:
        (void)snprintf(out, out_sz,
                       "[INFO] [APP] ctrl period: avg=%d.%02d ms max=%d.%02d ms bad(>%dms)=%d/%d",
                       (int)r->a[0], (int)r->a[1], (int)r->a[2], (int)r->a[3],
                       (int)r->a[4], (int)r->a[5], (int)r->a[6]);
        break;
#endif

#if APP_LOG_ISR_STAT
    case APP_LR_ADC_ISR:
        (void)snprintf(out, out_sz, "[INFO] [FOC] ADC ISR: 实测 %d 次 / 期望 %d 次, 最大间隔 %d us",
                       (int)r->a[0], (int)r->a[1], (int)r->a[2]);
        break;
#endif

#if APP_LOG_IQ_RIPPLE
    case APP_LR_IQ_PP:
        (void)snprintf(out, out_sz,
                       "[INFO] [FOC] 电流环 1s 峰峰值: idpp=%d mA iqpp=%d mA "
                       "(20kHz 域采样; 几十 mA 属正常开关纹波, 几百 mA 以上=自激)",
                       (int)r->a[0], (int)r->a[1]);
        break;
#endif

#if APP_LOG_FOC_LINE
    case APP_LR_FOC_LINE:
        (void)snprintf(out, out_sz,
                       "[INFO] [FOC] id=%d iq=%d mA (ref %d/%d) ud=%d uq=%d mV vbus=%d mV "
                       "rpm=%d dir=%d | pos=%d ref=%d err=%d mrad spdref=%d rpm raw=%d",
                       (int)r->a[0], (int)r->a[1], (int)r->a[2], (int)r->a[3],
                       (int)r->a[4], (int)r->a[5], (int)r->a[6], (int)r->a[7],
                       (int)r->a[8], (int)r->a[9], (int)r->a[10], (int)r->a[11],
                       (int)r->a[12], (int)r->a[13]);
        break;
#endif

#if APP_FOC_SPEED_LOOP_ENABLE
#  if (APP_SPEED_LOOP_ALG == APP_SPEED_LOOP_ALG_ADRC) && APP_LOG_ADRC_STATUS
    case APP_LR_SPEED_ADRC:
        /* 速度环 ADRC 每秒一行。判读:
         *   z1 与 rpm 差得多      -> 观测器跟不上, 提高 WO_HZ
         *   z2 大幅正负跳         -> 观测器在追噪声, 降低 WO_HZ
         *   err 长时间不收敛      -> WC_HZ 太小, 或 B0_SCALE 偏离实测
         *   iq 顶在限幅           -> 正常(大阶跃起步), 但持续顶住就是 b0 太小/负载重 */
        (void)snprintf(out, out_sz,
                       "[INFO] [ADRC] 速度环: ref=%d rpm=%d z1=%d err=%d RPM | "
                       "z2=%d RPM/s iq=%d mA (b0=%d RPM/(s.A), wc=%d, wo=%d rad/s)",
                       (int)r->a[0], (int)r->a[1], (int)r->a[2], (int)r->a[3],
                       (int)r->a[4], (int)r->a[5],
                       (int)APP_SPEED_ADRC_B0,
                       (int)APP_SPEED_ADRC_WC_RAD_S,
                       (int)APP_SPEED_ADRC_WO_RAD_S);
        break;
#  endif
#endif

#if APP_LOG_ENC_HEALTH
    case APP_LR_ENC_HEALTH:
        /* 全场最长的一行(约 185 字符 / 14 个参数), 每 APP_ENC_HEALTH_MS(5s) 一次。
         * 它是"控制任务里最贵的一次 vsnprintf", 所以同样延迟到日志任务里做。
         * 【注意】这里只覆盖非 SEI 的分支(本板走 bitbang); SEI 分支仍是内联 LOG_I。 */
        (void)snprintf(out, out_sz,
                       "[INFO] [ENC] health: reader=%s bb=%d miso=%d frames=%d errs=%d tfail=%d "
                       "raw=%d last=%06x sweep=%d/%d odo=%d stall=%d lut=%d/%d",
                       (r->a[0] != 0) ? "bitbang" : "spi2",
                       (int)r->a[1], (int)r->a[2], (int)r->a[3], (int)r->a[4],
                       (int)r->a[5], (int)r->a[6], (unsigned)r->a[7],
                       (int)r->a[8], (int)r->a[9], (int)r->a[10],
                       (int)r->a[11], (int)r->a[12], (int)r->a[13]);
        break;
#endif

    default:
        (void)snprintf(out, out_sz, "[WARN] [APP] 未知延迟日志 id=%u", (unsigned)r->id);
        break;
    }
}

#define APP_DEFER_MAX  (6U)    /* 1 秒窗口里有 5 条; 每 5 秒会和 ENC health 撞在一起 -> 6 条 */
static app_log_rec_t s_defer[APP_DEFER_MAX];
static uint8_t       s_defer_cnt;
static uint8_t       s_defer_pos;

/* 快照一条待发记录(在 1 秒结算点调用, 此时把要打印的数值定下来) */
static void app_defer_push(uint16_t id, const int32_t *a, uint8_t n)
{
    app_log_rec_t *r;

    if ((s_defer_cnt >= (uint8_t)APP_DEFER_MAX) || (n > APP_LOG_REC_MAX_ARGS)) {
        return;
    }
    r = &s_defer[s_defer_cnt++];
    r->id = id;
    r->n = n;
    for (uint8_t i = 0U; i < n; i++) {
        r->a[i] = a[i];
    }
}

/* 每拍最多投递一条 —— 放在 app_run() 的最前面, 任何 return 路径都不会漏掉它 */
static void app_defer_drain(void)
{
    if (s_defer_pos < s_defer_cnt) {
        (void)app_log_post(&s_defer[s_defer_pos]);
        s_defer_pos++;
        if (s_defer_pos >= s_defer_cnt) {
            s_defer_pos = 0U;
            s_defer_cnt = 0U;
        }
    }
}

/* ================= 主循环 ================= */
void app_run(void)
{
    static uint32_t tick = 0;
    int32_t iu_ma = 0, iv_ma = 0, iw_ma = 0;

    /* 【必须放在最前面】错峰投递那些"延迟格式化"的日志: 每拍只发一条。
     * 放在这里而不是结算点, 是为了不让 5 条挤在同一拍。 */
    app_defer_drain();
    float ia, ib, ic, vbus;
    float du, dv, dw;

    diag_set_stage(APP_STAGE_LOOP_TOP);
    diag_tick();

    /* ---- ADC 未就绪: 只做诊断, 但【持续观察】, 一旦活过来就补做自检 ----
     *
     * 【为什么不能在这里永久放弃】实测: app_init 里等 200ms/3000ms 都是 frames=0,
     * 而 app_init 一返回 frames 立刻开始涨(3 -> 19943/秒, 正好 20kHz)。
     * 也就是说 ADC 的出帧发生在 app_init 之后 —— 在 app_init 里死等永远等不到。
     * 所以改成"延迟自检": 这里每秒看一次帧计数, 涨到门槛就补做零点标定,
     * 然后才放开电机。在这之前电机一直是封锁的(不出力)。
     *
     * 【安全边界】没通过标定之前绝不使能; 标定通过后也只放开输出,
     * 电流环的参数/方向仍然沿用 app.h 里的配置。 */
    if (s_adc_dead != 0U) {
        uint32_t frames = bsp_adc_get_frame_count();

        /* 已经使能了就强制关掉(兜底) */
        if (s_ctrl.enabled) {
            foc_controller_enable(&s_ctrl, false);
            bsp_pwm_enable_output(false);
        }
        bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);

        if (frames < APP_ADC_ALIVE_MIN_FRAMES) {
            if ((tick % 1000U) == 0U) {
                LOG_I("ADC", "degraded: 等待电流采样 (frames=%u, 需要 >=%u), 电机保持封锁",
                      (unsigned)frames, (unsigned)APP_ADC_ALIVE_MIN_FRAMES);
            }
            tick++;
            return;
        }

        /* ---- 采样已活: 每秒只尝试一次 ----
         * 【为什么必须限频】标定(忙等) + 极性探测(120ms 忙等) 很重。
         * 若每个 1ms 控制周期都重试, 最高优先级的控制任务会 100% 占住 CPU,
         * 日志任务(优先级-3)/显示任务(-4) 永远轮不到 —— 实测现象就是
         * "串口在 LCD 之后一行都没有、屏幕也不刷新", 看着像卡死, 其实是饿死。 */
        tick++;
        if ((tick % 1000U) != 0U) {
            return;
        }

        {
            /* ---- 补做: 零点标定 -> 极性探测 -> 放开输出 ---- */
            uint16_t uo = 0U, vo = 0U;
            const int32_t mid = (int32_t)(65535U / 2U);
            int32_t iu = 0, iv = 0, iw = 0;

            bsp_adc_calibrate();
            bsp_adc_get_offsets(&uo, &vo);

            LOG_I("ADC", "!! 延迟自检: 采样已就绪(frames=%u), 补做零点标定 u=%u v=%u (mid=%d)",
                  (unsigned)frames, (unsigned)uo, (unsigned)vo, (int)mid);

            if ((uo == (uint16_t)mid) && (vo == (uint16_t)mid)) {
                LOG_I("ADC", "degraded: 标定仍无效(offset=mid), 电机保持封锁, 1s 后重试");
                return;
            }

            /* 标定有效: 打印一次实际电流读数(此时是零矢量, 应接近 0mA) */
            bsp_adc_get_currents_ma(&iu, &iv, &iw);
            LOG_I("ADC", "readback @0vector: ia=%d ib=%d ic=%d mA (应接近 0)",
                  (int)iu, (int)iv, (int)iw);

            /* ---- 极性自检 (必须在使能电流环之前) ----
             * 延迟自检绕过了 app_init 里的四角度极性自检; 不做这一步就使能,
             * 实测会因反馈反号把电流推到 2.5A 然后过流锁死。
             *
             * 【必须先打开 PWM 输出】探测是靠"给电压 -> 测电流"工作的。
             * 这一步漏了就是之前的 bug: 输出还关着, 绕组上没有电压,
             * 测到的电流恒为 0 -> 探测必然失败 -> 然后变成每秒重试的死循环。 */
            bsp_pwm_enable_output(true);
            do {
                uint8_t pr = app_probe_current_polarity((float)bsp_adc_get_vbus_mv() * 0.001f);

                if (pr == 0U) {
                    LOG_I("ADC", "degraded: 极性探测未通过(电流太小?), 电机保持封锁, 1s 后重试");
                    break;
                }
                if ((pr & 0x04U) != 0U) {
                    /* 相序镜像不是"取反"能修的: 需要改 s_phase_map。
                     * 带着镜像反馈跑电流环必然发散。 */
                    LOG_I("ADC", "!! 相序镜像, 保持封锁 (需要调整 s_phase_map 后重新验证)");
                    break;
                }

                s_cur_sign = ((pr & 0x02U) != 0U) ? -1.0f : 1.0f;
                /* 【强制覆盖: 两套判据会打架】
                 * 延迟路径用的是这里的两点判据(0°/90° 各钉一次, 只看 va[0] 的符号),
                 * 而 app_init 用的是四角度投影打分(0/90/180/270°, 6 种相序 x 2 种
                 * 极性里挑投影最大的)。两者对同一台电机可以给出【相反】的结论 ——
                 * 实测: 两点判据报 sign=-1 之后, 电流环 ud/uq 双双顶到母线限幅、
                 * id 冲到 2.1A, 而 iq 给定 +1500mA 实测 -498mA, 这正是"反馈反号
                 * -> 正反馈"的形状; 一旦正反馈, 外面套的速度环/位置环无论怎么调
                 * 都不可能守住(位置环那次 spdref 恒为 0, 根本没参与)。
                 *
                 * 判据谁对, 只能由"电流环跟不跟得上给定"来裁决:
                 *   - 强制 APP_CURRENT_SIGN=+1 后若 id≈0 且 iq 跟随给定 -> 两点判据的
                 *     符号反了(应改 app.c 里 `if (va[0] < 0.0f) res |= 0x02U;`);
                 *   - 若反而是 -1 才跟随 -> 两点判据正确, 去查角度/换向约定。
                 * 确认之后把这里改回 0(自动), 并把结论写进 app.h。 */
                if (APP_CURRENT_SIGN != 0.0f) {
                    if (s_cur_sign != APP_CURRENT_SIGN) {
                        LOG_I("ADC", "极性: 两点判据给出 %d, 被 APP_CURRENT_SIGN 覆盖为 %d",
                              (int)s_cur_sign, (int)APP_CURRENT_SIGN);
                    }
                    s_cur_sign = APP_CURRENT_SIGN;
                }
                LOG_I("ADC", "极性自检结果: sign=%d", (int)s_cur_sign);

                /* ---- 【采样时刻自检】把"ADC 触发点 vs 开关沿"的距离打出来 ----
                 * 三下桥全通(能采低边电流)的窗口以计数器回绕点为中心, 半宽:
                 *     reload/2 - duty_max×0.98×reload/2
                 * (0.98 见 bsp_pwm.c 的 s_duty_max; 闭环里 duty 被
                 *  vmax=vdc/√3×0.85 的矢量限幅压住, 最坏 0.925)
                 * 默认触发点 cmp=1 就落在窗口中心, 所以余量 = 上式。
                 * 【余量太小会怎样】实测到过 10A 的假值: 采样窗压到开关沿上,
                 * 运放被 12V 的 dv/dt 打飞。余量 < ~1us 就要怀疑读数了。 */
                {
                    uint32_t reload = bsp_pwm_get_reload();
                    uint32_t trig   = bsp_pwm_get_adc_trig_count();
                    uint32_t half   = reload / 2U;
                    uint32_t cmph   = (uint32_t)(0.925f * 0.98f * (float)reload) / 2U;
                    uint32_t margin = (half > cmph) ? (half - cmph) : 0U;

                    /* PWM 计数时钟: 20kHz × (reload+1) = 200MHz -> 1 count = 5ns */
                    LOG_I("ADC", "采样点: trig_cmp=%u/%u (窗口中心), 余量=%u counts ≈ %u ns, 采保=20clk",
                          (unsigned)trig, (unsigned)reload, (unsigned)margin,
                          (unsigned)(margin * 5U));
                }

                /* ---- 方向标定 (必须在这里做, 不能只靠 app_init) ----
                 * app_init 的"第四步: 方向标定"在这块板子上【从来不会执行】:
                 * 上电时 ADC 还没活(frames=0), app_init 在第二步就 return 了。
                 * 于是 s_speed_dir 一直停在静态默认值 +1, 速度环/位置环的符号
                 * 从没被实测过 —— 一使能就是正反馈: 实测 iq 顶 ±1500mA、
                 * 转速 ±145RPM 原地剧烈震荡, 位置环也守不住。
                 * 这里在【使能位置环之前】补做, 顺序和 app_init 的注释一致:
                 *     电流环 -> 方向自检 -> 位置环使能
                 * 测试要驱动电流, 所以先使能控制器; 测试内部结束时会归零并
                 * 清速度环积分。 */
                foc_controller_enable(&s_ctrl, true);
                app_calib_speed_dir((float)bsp_adc_get_vbus_mv() * 0.001f);

                /* ---- 全部通过: 使能电流环, 位置环用当前角度原地保持 ---- */
                s_adc_dead = 0U;

                foc_controller_enable(&s_ctrl, true);
#if APP_CURRENT_LOOP_IN_ISR
                /* 【本轮啸叫的根因就在这一行原来只有 s_isr_closed_loop = 1U】
                 * 延迟自检路径绕过了 app_init 里那次 set_dt(1/20000), 于是电流环
                 * 带着 ts=1ms 跑到 20kHz 中断里 —— Ki 等效放大 20 倍, 电流环自激、
                 * 满摆 ±5V、啸叫, 而外环(速度/位置)接在自激的内环后面永远跟不上。
                 * 现在统一从 app_current_loop_handover_to_isr() 走, 它同时改 ts。 */
                app_current_loop_handover_to_isr();
#endif
#if APP_MOTION_ENABLE
                (void)bsp_mt6701_poll();
                (void)app_encoder_update(&s_encoder);
                /* ---- 【按键必须在这里补初始化: app_init 里那次永远执行不到】 ----
                 * app_init 的 bsp_button_init() 在"ADC 一帧都没采到 -> degraded mode"
                 * 那个 return 的【后面】, 而这块板子上 ADC 出帧永远晚于 app_init
                 * (实测 app_init 等 300ms 仍 frames=0, 随后延迟自检里 frames=20521),
                 * 所以那几个按键脚从来没被配置过: 没复用成 GPIO、没开内部上拉、
                 * 方向也没设。浮空脚读出来是随机值 —— 轻则"按了没反应", 重则被当成
                 * 一直按住 -> 触发 HOME -> 位置目标归零 -> 位置环满力去追
                 * (bsp_button.c 的注释里记的就是这个事故)。
                 * 这里补一次。bsp_button_init() 是幂等的, 重复调用无害。 */
                {
                    static uint8_t s_btn_brought_up;

                    if (s_btn_brought_up == 0U) {
                        s_btn_brought_up = 1U;
                        bsp_button_init();
                        LOG_I("BTN", "按键就绪(延迟路径): %s=位置+  %s=位置-  %s=回起点",
                              bsp_button_name(BSP_BTN_UP),
                              bsp_button_name(BSP_BTN_DOWN),
                              bsp_button_name(BSP_BTN_HOME));
                    }
                }
                /* 【必须先 init 再 enable —— 这就是位置环"守不住"的根因】
                 * app_init() 在 ADC 没活过来时第 922 行就 return 了, 而
                 * app_motion_init(&s_motion) 在它后面的第 1256 行 —— 于是延迟
                 * 自检这条路【从来没有初始化过位置环】: pos_kp / speed_limit
                 * 停在 .bss 的 0, P 律 clamp(kp*err, 0) 恒等于 0。
                 * 现象就是日志里那几个数字: 位置环"已使能"、err 涨到 20~85 rad,
                 * 而 spdref 恒为 0 —— 环在, 但永远不出力, 电机只剩速度环自己在
                 * 守 0 转速, 一有扰动就漂走, 最后过流锁死。
                 * 注意 app_motion_init() 会把 pos_ref/pos_fb 清零, 所以必须在
                 * 下面的 app_motion_enable() 之前调用(enable 会用当前角度重新
                 * 建立"原地保持"的基准)。 */
                app_motion_init(&s_motion);
                app_motion_enable(&s_motion, true, app_encoder_get_mech_rad(&s_encoder));
                /* 【打结构体里的实际值, 不要打宏】
                 * 上一版这里打的是 APP_MOTION_POS_KP / APP_MOTION_SPEED_LIMIT_RPM,
                 * 结构体明明是 0 也照样显示 60 —— 正好把上面这个 bug 盖住了。 */
                LOG_I("MOT", "position loop ON: pos0=%d mrad, kp=%d mRPM/rad, 速度限幅=%d RPM, dir=%d",
                      (int)(s_motion.pos_fb * 1000.0f),
                      (int)(s_motion.pos_kp * 1000.0f), (int)s_motion.speed_limit,
                      (int)s_speed_dir);
#endif
                LOG_I("APP", "delayed bring-up done: 电机已使能(电流环闭环, sign=%d), 位置环原地保持",
                      (int)s_cur_sign);
            } while (0);

            if (s_adc_dead != 0U) {
                /* 任一步失败: 关控制器、写回零矢量、关输出, 保持封锁等下一次 */
                foc_controller_enable(&s_ctrl, false);
                bsp_pwm_set_duty_uvw(0.5f, 0.5f, 0.5f);
                bsp_pwm_enable_output(false);
            }
        }

        /* 【必须在所有路径上自增】下面的 1 秒门槛靠 tick 推进;
         * 漏掉任何一条 return 路径都会让 tick 停住 -> 每秒门槛失效 -> 又变成
         * 每个 1ms 周期都重试 -> 控制任务占满 CPU 把日志/显示任务饿死。 */
        tick++;
        return;
    }

    /* ---- 采集: 编码器 / 电流 / 母线电压 ---- */
    /* SRMA35 走 RS485(UART11), 由这里按 1kHz 主动发 CF 并收 11 字节 */
    diag_set_stage(APP_STAGE_LOOP_ENCODER);   /* 给中断诊断标记: 读编码器 */
    bsp_srma35_poll();
    bsp_srma35_poll_log();
    /* MT6701 已拆, 保留 poll 只为不改变 diag/LCD 的字段含义 (它会一直报错) */
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
         * 位置环必须靠它提供阻尼, 见 app.h 的说明。
         *
         * 【只有位置环真的在跑才接管】app_motion_run() 在未使能时返回 0; 如果照旧
         * 无条件 app_speed_target_set(spd_ref), 那就等于每拍把目标转速压成 0 ——
         * 屏幕上把位置环关掉之后滑条就彻底没用了(电机只会停住), 这正是"速度环和
         * 位置环打架"里最容易忽略的一半: 关掉位置环并不自动等于"回到速度模式",
         * 还必须停止用位置环的输出覆盖速度给定。
         * 切到速度模式时由界面重新下发一次滑条值(见 lcd_ui.c 的开关回调),
         * 否则目标会停在位置环最后一次写下的 spdref 上。 */
        if (s_motion.enable) {
            app_speed_target_set(spd_ref);

            /* 标记"本拍位置环接管了速度给定"。app_speed_apply() 会把状态拍成快照,
             * 显示层据此显示位置环状态而不是把速度条的值当成用户设定。 */
            s_motion_takeover = 1U;
        }
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
     * 一个很大的位置误差, 一松手就满速冲过去撞限位。
     *
     * 【上电屏蔽窗口】前 APP_BTN_BOOT_IGNORE_MS 内【丢弃所有按键事件】。
     * 这是被实测坑出来的: PC15/PC19 若因外部上拉未焊而浮空, 上电瞬间会被读成
     * "按下", 触发 HOME -> 把位置目标设成 0, 而转子实际停在非零角度 ->
     * 位置环立刻满力去追 0 => "上电电机就一直转"。
     * 消抖窗(20ms)挡不住这种"上电就一直低"的假按下(它会被当成一直按住),
     * 所以必须用一段开机屏蔽期。屏蔽期内按键状态照常采样(消抖正常进行),
     * 只是不产生事件。 */
    {
        static uint32_t s_boot_ms;
        uint32_t ev;
        uint8_t  e_up, e_down, e_home;

        ev = bsp_button_poll_all();

        if (s_boot_ms < APP_BTN_BOOT_IGNORE_MS) {
            s_boot_ms += APP_MAIN_LOOP_MS;
            e_up = e_down = e_home = BSP_BTN_EV_NONE;

            if (s_boot_ms == APP_BTN_BOOT_IGNORE_MS) {
                LOG_I("BTN", "按键屏蔽期结束, 此后按键才生效");
            }
        } else {
            e_up   = (uint8_t)((ev >> ((uint32_t)BSP_BTN_UP   * 4U)) & 0xFU);
            e_down = (uint8_t)((ev >> ((uint32_t)BSP_BTN_DOWN * 4U)) & 0xFU);
            e_home = (uint8_t)((ev >> ((uint32_t)BSP_BTN_HOME * 4U)) & 0xFU);
        }

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
            if (APP_LOG_BTN_LEVEL && (btn_log_tick >= 5000U)) {
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
#if APP_LOG_ENC_HEALTH
            {
                int32_t a[14];

                /* 【延迟格式化】这条 185 字符 / 14 参数, 原来每 5 秒在控制任务里内联
                 * vsnprintf 一次 —— 是"打印一下电机顿一下"里最贵的一条。 */
                a[0]  = bsp_bb_is_active() ? 1 : 0;
                a[1]  = (int32_t)bsp_bb_best_index();
                a[2]  = (int32_t)mi;
                a[3]  = (int32_t)fr;
                a[4]  = (int32_t)er;
                a[5]  = (int32_t)tf;
                a[6]  = (int32_t)bsp_mt6701_get_raw_angle();
                a[7]  = (int32_t)bsp_mt6701_get_last_raw();
                a[8]  = (int32_t)s_sweep_step;
                a[9]  = (int32_t)s_sweep_steps;
                a[10] = (int32_t)(long)s_sweep_odo;
                a[11] = (int32_t)s_sweep_stall;
                a[12] = (int32_t)s_lut_full;
                a[13] = (int32_t)s_lut_reject;
                app_defer_push(APP_LR_ENC_HEALTH, a, 14U);
            }
#else
            (void)er;       /* 关掉体检行时这两个量没人用 */
            (void)tf;
#endif  /* APP_LOG_ENC_HEALTH */
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
                 * 这里绝不能再按主循环周期改写, 否则积分增益差 20 倍。
                 * 【但必须"盯着"它】只要中断闭环是开着的, 每拍就把 ts 强制回
                 * 1/20000: 这样即使将来又多出某条使能路径忘了改 ts(2026-10-20
                 * 那次事故就是延迟自检路径漏了), 也最多错一拍, 不会一路自激。 */
                if (s_isr_closed_loop != 0U) {
                    foc_controller_set_dt(&s_ctrl, 1.0f / (float)APP_ADC_ISR_FREQ_HZ);
                }
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
            int32_t  a[7];

            /* 【延迟格式化 + 错峰】见 app_log_format_rec 上面的说明:
             * 这里只算数、快照, 不拼字符串。 */
            a[0] = (int32_t)(avg100 / 100U);
            a[1] = (int32_t)(avg100 % 100U);
            a[2] = (int32_t)(period_max100 / 100U);
            a[3] = (int32_t)(period_max100 % 100U);
            a[4] = (int32_t)APP_CTRL_PERIOD_WARN_MS;
            a[5] = (int32_t)period_bad;
            a[6] = (int32_t)period_cnt;
            app_defer_push(APP_LR_CTRL_PERIOD, a, 7U);

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
                /* 过流瞬间强制录一份: 录波是"环形一直采 + 触发后补一半"的,
                 * 所以这里触发拿到的是【过流前 25.6ms + 之后 25.6ms】,
                 * 正好覆盖失控的起点(转储在随后几秒里由 app_scope_poll 打出)。 */
                app_scope_trigger();
                /* 把三相电流和当时的 id/iq 全打出来: 判断是"真过流"还是"电流换算
                 * (采样电阻/增益)偏大导致误触发"。注意这个保护是锁死的, 复位才恢复。
                 * 【带上 duty】如果这次过流是"采样点踩到开关沿"造成的假值, duty 往往
                 * 正在极端值(接近 0 或 0.98): 三个占空比一起看就能分辨 —— 真过流时
                 * duty 通常还在中段, 假值时常常有一相已贴边。 */
                LOG_I("FOC", "OVERCURRENT %d mA x%u (ia=%d ib=%d ic=%d) id=%d iq=%d rpm=%d sign=%d "
                             "duty=%d/%d/%d permille -> output OFF (锁死, 需复位)",
                      (int)(imax * 1000.0f), (unsigned)s_oc_cnt,
                      (int)(ia * 1000.0f), (int)(ib * 1000.0f), (int)(ic * 1000.0f),
                      (int)(s_ctrl.foc.id * 1000.0f), (int)(s_ctrl.foc.iq * 1000.0f),
                      (int)app_encoder_get_mech_rpm(&s_encoder), (int)s_cur_sign,
                      (int)(s_ctrl.foc.duty_u * 1000.0f),
                      (int)(s_ctrl.foc.duty_v * 1000.0f),
                      (int)(s_ctrl.foc.duty_w * 1000.0f));
            }

            if (s_oc_tripped != 0U) {
                /* 锁死后速度环状态必须清零: 否则解锁/复位重跑时, 它会带着旧的
                 * 积分(PID)或旧的扰动估计 z2(ADRC)直接冲出去。 */
#if APP_FOC_SPEED_LOOP_ENABLE
                app_speed_loop_reset();
#endif
                iq = 0.0f;
            } else {
#if APP_FOC_SPEED_LOOP_ENABLE
                /* ---- a) 速度环 (给定 - 实测 -> iq) ----
                 * 算法由 app.h 的 APP_SPEED_LOOP_ALG 选择:
                 *    ADRC: LADRC_Run() —— 输出 = (wc·e - z2)/b0, z2 是 ESO 观测到的
                 *          集总扰动(负载/摩擦/反电动势/Kt、J 误差), 参数见 app.h;
                 *    PID : PID_Step()  —— 与加 ADRC 之前完全一致, 用于回退/对比。
                 *
                 * 【给定和实测都要乘 s_speed_dir】
                 * 速度环必须是负反馈: 给定"正转速"时实测转速也必须朝正走。而
                 * "实测 rpm 的符号"取决于编码器计数方向、相电流极性(自检 ±1)、
                 * FOC 电角度约定这三件事的组合, 任何一处反了, 环路就成了正反馈
                 * (现象: iq 顶到限幅、转子被反拖)。s_speed_dir 就是三者合成后的
                 * 符号, 由开机自检实测出来(见 app_init 的"速度环方向自检")。
                 * 两边都乘 dir, 等于把被控对象折算到"正 iq 使 y 增大"的坐标系,
                 * 这样 ADRC 的 b0 恒为正; 只给 target 乘(旧 PID 写法)在 dir=-1
                 * 时会把误差算错 —— dir=+1 时两种写法完全等价。 */
                {
                    float ref = s_speed_target_rpm * s_speed_dir;
                    float y   = rpm * s_speed_dir;

                    if (!s_ctrl.enabled) {
                        /* 停机期间转子可能被外力带着转, 观测器会去追一个"不受控"
                         * 的转速; 重新使能瞬间它会把这个当成扰动补一记电流冲击。
                         * 所以没使能就清状态, 并让 iq 保持 0。 */
                        app_speed_loop_reset();
                        iq = 0.0f;
                    } else {
                        /* 观测器步长用【实测周期】: 主循环被日志/显示任务挤长时,
                         * 写死 1ms 会让实际 wo 按比例缩水(观测器变慢)。PID 路径
                         * 不支持逐拍 dt, 这个调用在那边是空操作。 */
                        app_speed_loop_set_dt(s_dt_main);
#if (APP_SPEED_LOOP_ALG == APP_SPEED_LOOP_ALG_ADRC)
                        iq = LADRC_Run(&s_ladrc_spd, ref, y);
#else
                        iq = PID_Step(&s_pid_spd, ref - y);
#endif
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

#if APP_IQ_STEP_TEST_ENABLE
            /* 【电流环阶跃测试】完全旁路外环, 直接给 iq 方波 (见 app.h 的说明)。
             * 每次跳变都会命中录波的 |Δiq_ref| 触发, 于是串口上自动得到一串
             * "阶跃响应", 拿它调电流环 Kp/Ki。
             * 【幅值必须小】1.5A 在 3.1Ω 上要 4.65V, 而电压天花板只有
             * vdc/√3×0.85 ≈ 5.9V —— 大电流下环路一直贴限幅, 看到的不是增益
             * 特性而是饱和。默认 0.3A(约 1V)才有干净的阶跃。 */
            {
                static uint32_t step_ms;
                static uint8_t  step_hi;

                step_ms += APP_MAIN_LOOP_MS;
                if (step_ms >= APP_IQ_STEP_HALF_MS) {
                    step_ms = 0U;
                    step_hi = (uint8_t)(step_hi ^ 1U);
                }
                iq = (step_hi != 0U) ? APP_IQ_STEP_AMP_A : -APP_IQ_STEP_AMP_A;

                /* 位置环/速度环此刻算出来的东西全部作废, 顺便把速度环的状态
                 * 清掉 —— 否则测试完一切回闭环, 旧积分/旧 z2 会突然出力。
                 * (这个开关可以脱离速度环单独使用, 所以复位要跟着条件编译。) */
#if APP_FOC_SPEED_LOOP_ENABLE
                app_speed_loop_reset();
#endif
            }
#endif

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

    /* 录波转储: 触发了就把缓冲区按 APP_SCOPE_DUMP_MS 节流打成 CSV
     * (串口慢, 必须节流; 环形缓冲塞不下的行会被整行丢掉, 曲线就有洞) */
    app_scope_poll();

    /* ---- 电流环体检: 每秒一行 id/iq/ud/uq ----
     * 做电流环没有这行等于闭着眼调: 看 iq 有没有跟到 iq_ref、
     * id 有没有被压住(应该 ~0)、ud/uq 有没有顶到限幅。
     * 若 iq 与 iq_ref 反号或者一路顶到限幅, 首先怀疑相电流极性 (看开机自检那行)。 */
#if APP_FOC_CLOSED_LOOP_ENABLE
    {
        static uint32_t cur_log_tick = 0U;
#if APP_CURRENT_LOOP_IN_ISR
        static uint32_t isr_cnt_prev = 0U;
        static uint8_t  isr_first    = 1U;
        static float    win_time;           /* 本窗口(1s)累计的【真实】时长, 单位 s */
#endif

        /* 【每拍累加窗口时长, 不是"关闭窗口那一拍才加"】
         * 原来 win_time += s_dt_main 写在下面那个 if (cur_log_tick >= 1000) 里面,
         * 于是整个 1s 窗口只累加到【最后一拍】的 dt(约 1ms):
         *     expect = 20000 × 0.001 = 20 次
         * 日志里 "实测 20011 次 / 期望 27 次" 就是这么来的 —— 期望值本身是错的。
         * 更要紧的是下面那条看门狗 (rate < expect/4 就封锁输出) 因此永远不成立
         * (20000 < 5), 等于【没有任何保护】: 中断真停摆时占空比会冻结在最后一拍,
         * 绕组可能一直通直流而没人管。现在按每拍累加, expect 才是真的 20kHz 期望。 */
#if APP_CURRENT_LOOP_IN_ISR
        if (isr_first != 0U) {
            /* 第一窗口: 基准/时长都从现在开始记, 否则 rate 会是上电累计次数 */
            isr_first    = 0U;
            isr_cnt_prev = s_isr_cnt;
            win_time     = 0.0f;
        } else if ((s_dt_main > 0.0f) && (s_dt_main < 0.05f)) {
            win_time += s_dt_main;
        }
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
                 * 期望值这里用【实测主循环周期累加】出来的本窗口真实时长来算,
                 * 不再假设窗口就是 1 秒(那个累加现在在窗口外面, 见上)。 */
                uint32_t now  = s_isr_cnt;
                uint32_t rate = now - isr_cnt_prev;
                uint32_t expect;

                isr_cnt_prev = now;
                expect = (uint32_t)((float)APP_ADC_ISR_FREQ_HZ * win_time);

#if APP_LOG_ISR_STAT
                /* 只在打开这个开关时打印(排障用, 见 app.h 的说明) */
                {
                    uint32_t dtmax_us = (uint32_t)(((uint64_t)s_isr_dt_max * 1000000U)
                                                   / (uint64_t)hpm_core_clock);
                    int32_t  a[3];

                    s_isr_dt_max = 0U;
                    a[0] = (int32_t)rate;
                    a[1] = (int32_t)expect;
                    a[2] = (int32_t)dtmax_us;
                    app_defer_push(APP_LR_ADC_ISR, a, 3U);
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

                /* 【电流环峰峰值】取上个 1s 窗口在 20kHz 域记下的极值, 然后重新开始记。
                 * 判读: idpp/iqpp 只有几十 mA = 只有开关纹波(正常);
                 *       几百 mA~A 级  = 电流环在自激(这就是啸叫的来源)。 */
                {
#if APP_LOG_IQ_RIPPLE
                    float   idpp = (s_isr_id_max - s_isr_id_min) * 1000.0f;
                    float   iqpp = (s_isr_iq_max - s_isr_iq_min) * 1000.0f;
                    int32_t a[2];

                    a[0] = (int32_t)idpp;
                    a[1] = (int32_t)iqpp;
                    app_defer_push(APP_LR_IQ_PP, a, 2U);
#endif
                    s_isr_id_min = s_isr_id_max = s_ctrl.foc.id;
                    s_isr_iq_min = s_isr_iq_max = s_ctrl.foc.iq;
                }

                if ((s_isr_closed_loop != 0U) && (rate < (expect / 4U))) {
                    s_isr_closed_loop = 0U;
                    foc_controller_enable(&s_ctrl, false);
                    bsp_pwm_enable_output(false);
                    LOG_I("FOC", "!! ADC 中断只跑了 %u 次 (该约 %u), 反馈已失效 -> 封锁输出 (需复位)",
                          (unsigned)rate, (unsigned)expect);
                }
            }
#endif
            /* 【打点节流】这条 132 字符 = 1.4ms 串口时间(921600 baud), 和同一次
             * 突发里的 ctrl period / ADC ISR 加起来每秒 ~2.7ms。调电流环时把它
             * 降到 1/5 或 1/10(见 app.h 的 APP_LOG_FOC_EVERY_N), 串口就不会把
             * 后面的 100Hz CSV 挤掉, 日志任务对 ADC 中断的干扰也小得多。 */
            {
                static uint32_t foc_line_div;

                if (APP_LOG_FOC_LINE &&
                    (++foc_line_div >= (uint32_t)APP_LOG_FOC_EVERY_N)) {
                    foc_line_div = 0U;
#if APP_MOTION_ENABLE
            /* 带上位置环数据: 目标是 posref, 反馈是 pos, 两者之差 err 就是位置环的
             * 输入; spdref 是位置环给出的速度给定(交给速度环)。调位置环看这行:
             *   err 长时间不归零 -> pos_kp 太小; err 反复过零/来回摆 -> pos_kp 太大。
             *
             * 【这一行里的每个量都是独立可核对的, 排查方向时按这个顺序看】
             *   dir    = s_speed_dir (方向系数), 必须和 pos/raw 的走法自洽;
             *   raw    = 编码器原始角 (14bit), 它和 pos 必须【同向】增减;
             *   rpm    = 滤波后的机械转速, 它和 raw 的差分必须【同向】;
             *   spdref = 位置环给出的速度给定;
             *   iq     = 速度环 PI 的输出, 它和 (spdref - rpm) 必须【同向】。
             * 任何一对不同向, 那一对之间的环节就是反的 —— 不用再猜。
             *
             * 【延迟格式化】这里只算数(那些 *1000 的软件浮点乘法约 50ns 一次, 无所谓),
             * 字符串拼装在日志任务里做 —— 见 app_log_format_rec。 */
            {
                int32_t a[14];

                a[0]  = (int32_t)(s_ctrl.foc.id * 1000.0f);
                a[1]  = (int32_t)(s_ctrl.foc.iq * 1000.0f);
                a[2]  = (int32_t)(s_ctrl.foc.id_ref * 1000.0f);
                a[3]  = (int32_t)(s_ctrl.foc.iq_ref * 1000.0f);
                a[4]  = (int32_t)(s_ctrl.foc.ud * 1000.0f);
                a[5]  = (int32_t)(s_ctrl.foc.uq * 1000.0f);
                a[6]  = (int32_t)bsp_adc_get_vbus_mv();
                a[7]  = (int32_t)app_encoder_get_mech_rpm(&s_encoder);
                a[8]  = (int32_t)s_speed_dir;
                a[9]  = (int32_t)(app_motion_get_pos(&s_motion) * 1000.0f);
                a[10] = (int32_t)(s_motion.pos_ref * 1000.0f);
                a[11] = (int32_t)(app_motion_get_err(&s_motion) * 1000.0f);
                a[12] = (int32_t)s_motion.speed_ref;
                a[13] = (int32_t)bsp_mt6701_get_raw_angle();
                app_defer_push(APP_LR_FOC_LINE, a, 14U);
            }
#else
            {
                int32_t a[14];

                a[0] = (int32_t)(s_ctrl.foc.id * 1000.0f);
                a[1] = (int32_t)(s_ctrl.foc.iq * 1000.0f);
                a[2] = (int32_t)(s_ctrl.foc.id_ref * 1000.0f);
                a[3] = (int32_t)(s_ctrl.foc.iq_ref * 1000.0f);
                a[4] = (int32_t)(s_ctrl.foc.ud * 1000.0f);
                a[5] = (int32_t)(s_ctrl.foc.uq * 1000.0f);
                a[6] = (int32_t)bsp_adc_get_vbus_mv();
                a[7] = (int32_t)app_encoder_get_mech_rpm(&s_encoder);
                a[8] = (int32_t)s_cur_sign;
                app_defer_push(APP_LR_FOC_LINE, a, 9U);
            }
#endif
                }
            }
#if APP_FOC_SPEED_LOOP_ENABLE
#  if (APP_SPEED_LOOP_ALG == APP_SPEED_LOOP_ALG_ADRC) && APP_LOG_ADRC_STATUS
            /* 速度环 ADRC 状态行 (1s): 判读见 app.h 的 APP_LOG_ADRC_STATUS。
             * 和上面一样只算整数, 字符串留给日志任务拼。 */
            {
                int32_t a[6];
                float ref = s_speed_target_rpm * s_speed_dir;
                float rpm = app_encoder_get_mech_rpm(&s_encoder);

                a[0] = (int32_t)ref;
                a[1] = (int32_t)rpm;
                a[2] = (int32_t)s_ladrc_spd.z1;
                a[3] = (int32_t)(ref - rpm);
                a[4] = (int32_t)s_ladrc_spd.z2;
                a[5] = (int32_t)(s_ladrc_spd.u * 1000.0f);
                app_defer_push(APP_LR_SPEED_ADRC, a, 6U);
            }
#  endif
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
                            (int32_t)(APP_CAL_SWEEP_HOLD - APP_CAL_SWEEP_DROP)) & (int32_t)APP_ENC_RAW_MASK);

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
                 * 步不动就提前收尾, 不能拿 1.5V 一直顶着烧。
                 * 【门槛随分辨率走】17bit 下静止噪声实测 ±2 counts, 14bit 时代的
                 * "±2 就算没动"在这里等于把正常抖动当停滞, 所以放大到 ±16。 */
                if ((dd < 16) && (dd > -16)) { s_sweep_stall++; }
                else                         { s_sweep_stall = 0U; }
            }
            /* 每走完一个电周期, 打印这一周期转子实际推进了多少计数。
             * 理想值恒为 131072/极对数 = 10922 (17bit)。
             * 若在整圈里忽大忽小(0.5x~2.4x) -> 编码器刻度不均匀(磁环偏心/倾斜);
             * 换过磁环后应当落在 10922±1040 之内。 */
            if ((s_sweep_step % APP_CAL_SWEEP_PHASES) == 0U) {
                if (s_sweep_step > 0U) {
                    (void)snprintf(line, sizeof(line),
                                   "CALCYC %2u ADV=%ld (ideal %ld)",
                                   (unsigned)(s_sweep_step / APP_CAL_SWEEP_PHASES),
                                   (long)cal_wrap_raw((int32_t)raw_avg - (int32_t)s_cyc_ref),
                                   (long)(APP_ENC_RAW_MAX / (uint32_t)FOC_MOTOR_POLE_PAIRS));
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
                /* 17bit raw -> >>3 折成 14bit 等价量, 喂给 1024 箱的诊断 LUT
                 * (那张表是按 16384 一格建的, 整表只作诊断, 不参与换向) */
                diag_lut_feed_static((uint32_t)(raw_use >> 3), sw_mrad);

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
                                   (long)(e1 & (int32_t)APP_ENC_RAW_MASK), (long)(e2 & (int32_t)APP_ENC_RAW_MASK),
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

            if ((s_sweep_step >= s_sweep_steps) || (s_sweep_odo >= (int32_t)APP_ENC_RAW_MAX) ||
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

    /* ---- 周期性上报 (APP_TELEMETRY_ENABLE=0 时整块被编译掉: 这是最贵的一项) ---- */
    if (APP_TELEMETRY_ENABLE && (++tick >= APP_REPORT_TICKS)) {
        char msg[256];

        tick = 0;

#if APP_TELEMETRY_CURRENT_ONLY
        /* ---- 电流环 CSV: 调 PI 就看这一行 ----
         *   id_ref, id, iq_ref, iq, ud, uq     [mA, mA, mA, mA, mV, mV]
         * 每 APP_REPORT_TICKS 行一拍(默认 100Hz); ref 与实测相邻, 相减就是误差。
         *   误差长时间不为 0      -> Ki 太小(或积分被限幅)
         *   来回振荡/过冲        -> Kp 偏大, 先减 Kp 再谈 Ki
         *   |ud,uq| 贴住 Vbus/2  -> 电压饱和, 这时调 gains 没有意义,
         *                          先解决换向角/限幅(录波里能直接看到)
         * 想看 20kHz 的瞬态(阶跃响应/失控起点)用 app_scope 的 "SCOPE,..." 行。 */
        (void)diag_csv_current(msg, sizeof(msg),
                               (int32_t)(s_ctrl.foc.id_ref * 1000.0f),
                               (int32_t)(s_ctrl.foc.id * 1000.0f),
                               (int32_t)(s_ctrl.foc.iq_ref * 1000.0f),
                               (int32_t)(s_ctrl.foc.iq * 1000.0f),
                               (int32_t)(s_ctrl.foc.ud * 1000.0f),
                               (int32_t)(s_ctrl.foc.uq * 1000.0f));
#elif APP_TELEMETRY_PAIR_ONLY
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
