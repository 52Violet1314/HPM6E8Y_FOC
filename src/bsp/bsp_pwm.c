/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: 三相互补 PWM 实现 (PWMV2)
 *
 * 【与 STM32 的整体对照】
 *   STM32 (F1/F4/G4 高级定时器 TIM1/TIM8)          HPM6E8Y (PWMV2)
 *   ----------------------------------------------------------------
 *   RCC 使能 TIM1 时钟                              clock_add_to_group(BOARD_MOTOR_CLK_NAME)
 *   ARR (自动重装值) = 计数周期                     reload (计数器重载值)
 *   CCR1/CCR2/CCR3 (比较值 = 占空比)                SHADOW_VAL[n] (影子比较值)
 *   CCER 互补输出 + CCERxE                         pwmv2_setup_waveform_in_pair(互补对)
 *   BDTR.DTG (死区时间)                            pwmv2_pair_config_t.dead_zone_in_half_cycle
 *   TIM_OCMODE_PWM1 + 中心对齐(CMS)                pwmv2 中心对齐 + 比较值方式
 *   CR1.CEN (启动)                                 pwmv2_enable_multi_counter_sync + start_pwm_output_sync
 *   TIM1_TRGO -> ADC 外部触发                      PWM TRGO -> TRGM -> ADC 触发
 *   MOE (BDTR.MOE, 主输出使能)                     pwmv2_enable_software_force + force_output(0)
 *
 * 关键差异: HPM 的比较值/重载值都走"影子寄存器"(shadow), 修改前要 unlock,
 * 修改后 lock, 由硬件在下一个 reload 点统一生效 => 天然无毛刺, 不需要像
 * STM32 那样手动考虑 UG 位产生更新事件。这就是本文件里到处出现
 * pwmv2_shadow_register_unlock()/lock() 的原因。
 */
#include "bsp_pwm.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_pwmv2_drv.h"

#define BSP_PWM            (BOARD_BLDCPWM)          /* HPM_PWM0 */
#define BSP_PWM_TRGM       (BOARD_BLDCPWM_TRGM)     /* HPM_TRGM0 */

/* 三相 6 个通道: 0/1=U, 2/3=V, 4/5=W */
#define BSP_PWM_PHASE_UH   (BOARD_BLDC_UH_PWM_OUTPIN)
#define BSP_PWM_PHASE_UL   (BOARD_BLDC_UL_PWM_OUTPIN)
#define BSP_PWM_PHASE_VH   (BOARD_BLDC_VH_PWM_OUTPIN)
#define BSP_PWM_PHASE_VL   (BOARD_BLDC_VL_PWM_OUTPIN)
#define BSP_PWM_PHASE_WH   (BOARD_BLDC_WH_PWM_OUTPIN)
#define BSP_PWM_PHASE_WL   (BOARD_BLDC_WL_PWM_OUTPIN)

/* 每个"比较值"占用一个 shadow 槽位, 这里把它们固定分配成一张表。
 * 布局: [0]=reload  [1][2]=U 上/下  [3][4]=V 上/下  [5][6]=W 上/下
 *       [9]=ADC 触发比较值
 * 相当于 STM32 里 ARR / CCR1..CCR6 / 触发比较寄存器的地址分配表。 */
#define BSP_PWM_SHADOW_RELOAD   PWMV2_SHADOW_INDEX(0)
#define BSP_PWM_SHADOW_U        PWMV2_SHADOW_INDEX(1)
#define BSP_PWM_SHADOW_V        PWMV2_SHADOW_INDEX(3)
#define BSP_PWM_SHADOW_W        PWMV2_SHADOW_INDEX(5)
#define BSP_PWM_SHADOW_TRIG     PWMV2_SHADOW_INDEX(9)   /* ADC 触发比较值 */

static uint32_t s_reload;       /* 载波重载值 */
static uint32_t s_duty_max;     /* 乘 0.98 防饱和 */

/**
 * @brief  更新某一相的占空比(往一对 shadow 比较值里写值)
 * @param  shadow_top  该相"上桥"比较值占用的 shadow 索引; 下桥用 shadow_top+1
 * @param  duty        占空比, 0.0f ~ 1.0f (0.5f = 50%)
 *
 * 【原理 / 对照 STM32】
 *   HPM 的 PWMV2 采用"比较值"结构: 计数器从 0 数到 reload 再(中心对齐时)数回 0,
 *   比较值决定输出翻转点。中心对齐时, 一组对称的比较值
 *       cmp_lo = reload/2 - cmp_half
 *       cmp_hi = reload/2 + cmp_half
 *   会形成一段居中的高电平窗口, 窗口宽度 = 2*cmp_half = duty * reload,
 *   所以有效占空比 = duty。两个值分别写进 shadow_top(上桥 CHn) 与
 *   shadow_top+1(下桥互补 CHn+1)。
 *
 *   对应 STM32: 相当于往 TIMx->CCR1(和互补通道对应的 CCR2)写比较值,
 *   在中心对齐(PWM1 模式)下改 CCR 就是改占空比。
 *
 *   注意: HPM 写入前必须先 pwmv2_shadow_register_unlock(), 写完 lock。
 *   这类似 STM32 里"更新事件前写 CCR 进预装载寄存器, 更新事件时一起生效",
 *   区别是 HPM 的生效时机由硬件在 reload 点自动完成, 无需软件干预。
 *
 *   这里还乘了 s_duty_max(= reload*0.98) 做饱和保护, 避免 100% 占空比时
 *   上下桥比较值相等导致窄脉冲。
 */
static void pwm_phase_update(uint8_t shadow_top, float duty)
{
    uint32_t cmp_half;
    uint32_t reload_half = s_reload >> 1;   /* reload/2, 中心对齐的"中轴" */

    if (duty < 0.0f) {                      /* 限幅: STM32 里一般也做同样的饱和处理 */
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    /* 半窗口宽度 = duty * reload / 2 */
    cmp_half = (uint32_t)(duty * (float)s_duty_max) >> 1;

    /* 开锁 shadow -> 写入对称比较值 -> 上锁, 硬件在下一 reload 点生效 */
    pwmv2_shadow_register_unlock(BSP_PWM);
    pwmv2_set_shadow_val(BSP_PWM, shadow_top,     reload_half - cmp_half, 0, false);  /* 上桥翻转点 */
    pwmv2_set_shadow_val(BSP_PWM, shadow_top + 1, reload_half + cmp_half, 0, false);  /* 下桥翻转点 */
    pwmv2_shadow_register_lock(BSP_PWM);
}

/**
 * @brief  初始化 PWM0: 3 对互补通道 + 死区 + 中心对齐 + TRGO 触发 ADC
 *
 * 【整体流程 / 对照 STM32 初始化 TIM1 做三相互补驱动】
 *   1. 开时钟        -> clock_add_to_group(BOARD_MOTOR_CLK_NAME)   // 类似 RCC_APB2PeriphClockCmd(TIM1)
 *   2. 算周期        -> s_reload = f_clk / f_pwm - 1               // 类似 TIM_Period(ARR)
 *   3. 配比较单元    -> pwmv2_cmp_config_t (cmp_source=shadow)     // 类似 TIM_OCInitTypeDef
 *   4. 配互补输出+死区-> pwmv2_pair_config_t(dead_zone_in_half_cycle)// 类似 BDTR.DTG
 *   5. 计数器配置    -> shadow 作 reload, reload 时更新           // 类似 ARPE(自动预装载)
 *   6. TRGO->TRGM->ADC 触发                                        // 类似 TIM_SelectOutputTrigger(TIM_TRGO_UPDATE)
 *   7. 多计数器同步启动                                            // 类似 TIM_CtrlPWMOutputs + 启动
 *
 * 初始化结束后主动调用 bsp_pwm_enable_output(false) 把 6 路强制为低,
 * 相当于 STM32 里清掉 BDTR.MOE —— 上电默认电机自由停车, 安全。
 */
void bsp_pwm_init(void)
{
    pwmv2_cmp_config_t cmp_cfg[2] = {0};    /* 每相 2 个比较值(上/下桥) */
    pwmv2_pair_config_t pwm_cfg = {0};      /* 一对互补通道的公共配置 */
    uint32_t i;

    /* 0) 先把 PD00~PD05 复用到 PWM0 (UH/UL/VH/VL/WH/WL)。
     *    注意: board_init() 里并没有调用本板 pinmux 的 init_pwm0_pins(),
     *    如果这里不调用, PD0~PD5 就一直是默认功能, 栅极驱动的 6 路输入
     *    永远收不到 PWM -> 电机无论如何都不会转。 */
    init_pwm0_pins();

    /* 1) 打开电机 PWM 模块时钟(= STM32 的 RCC 使能 TIM1) */
    clock_add_to_group(BOARD_MOTOR_CLK_NAME, 0);

    /* 2) 计算计数周期.
     *    reload = 时钟频率 / 目标载波频率 - 1  (向上/向下计数共 reload+1 拍)
     *    相当于 STM32: TIM_Period = SystemCoreClock/TIM_Prescaler/20kHz - 1 */
    s_reload = clock_get_frequency(BOARD_MOTOR_CLK_NAME) / BSP_PWM_FREQUENCY_HZ - 1;
    s_duty_max = (uint32_t)((float)s_reload * 0.98f);   /* 留 2% 余量防饱和 */

    /* 3) 配置比较值来源. cmp_source = cmp_value_from_shadow_val 表示
     *    "比较值不是直接写死, 而是从 SHADOW_VAL[] 影子槽里取"。
     *    这相当于 STM32 的 CCRx 预装载(OCPreload), 写值不会立刻生效。 */
    cmp_cfg[0].cmp = s_reload;                              /* 初值: 最大 => 0% 占空 */
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;                        /* 不用高分辨率模式 */
    cmp_cfg[0].cmp_source = cmp_value_from_shadow_val;      /* 来自 shadow */
    cmp_cfg[0].cmp_source_index = BSP_PWM_SHADOW_U;         /* U 相上桥 -> shadow[1] */
    cmp_cfg[0].update_trigger = pwm_shadow_register_update_on_reload;  /* reload 时更新 */

    cmp_cfg[1].cmp = s_reload;
    cmp_cfg[1].enable_half_cmp = false;
    cmp_cfg[1].enable_hrcmp = false;
    cmp_cfg[1].cmp_source = cmp_value_from_shadow_val;
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(2);    /* U 相下桥 -> shadow[2] */
    cmp_cfg[1].update_trigger = pwm_shadow_register_update_on_reload;

    /* 4) 一对互补通道的公共配置: 开输出、关故障保护、不反相、设死区 */
    pwmv2_get_default_config(&pwm_cfg.pwm[0]);
    pwmv2_get_default_config(&pwm_cfg.pwm[1]);
    for (i = 0; i < 2; i++) {
        pwm_cfg.pwm[i].enable_output = true;                /* 通道使能(CCER.CCxE) */
        pwm_cfg.pwm[i].enable_async_fault = false;          /* 异步故障刹车, 此处关闭 */
        pwm_cfg.pwm[i].enable_sync_fault = false;           /* 同步故障刹车, 此处关闭 */
        pwm_cfg.pwm[i].invert_output = false;               /* 输出不反相 */
        pwm_cfg.pwm[i].enable_four_cmp = false;             /* 不用四比较值模式 */
        pwm_cfg.pwm[i].update_trigger = pwm_reload_update_on_reload;
        /* 死区时间(单位 tick): 对应 STM32 BDTR.DTG, 防止上下桥直通 */
        pwm_cfg.pwm[i].dead_zone_in_half_cycle = BSP_PWM_DEAD_TIME_TICK;
    }

    /* 5) 先关并复位三个半桥的计数器(= STM32 里 CEN=0 再 CNT=0) */
    pwmv2_disable_counter(BSP_PWM, pwm_counter_0);
    pwmv2_disable_counter(BSP_PWM, pwm_counter_1);
    pwmv2_disable_counter(BSP_PWM, pwm_counter_2);
    pwmv2_reset_counter(BSP_PWM, pwm_counter_0);
    pwmv2_reset_counter(BSP_PWM, pwm_counter_1);
    pwmv2_reset_counter(BSP_PWM, pwm_counter_2);

    /* 预置 shadow 值: reload + 六路比较值(初值全 reload => 0% 占空) + ADC 触发比较值.
     * 对应 STM32 上电先给 ARR/CCR1..CCR6 写初值, 保证启动瞬间不会输出怪波形。 */
    pwmv2_shadow_register_unlock(BSP_PWM);                                      /* 开锁 shadow */
    pwmv2_set_shadow_val(BSP_PWM, BSP_PWM_SHADOW_RELOAD, s_reload, 0, false);   /* shadow[0]=reload(=ARR) */
    pwmv2_set_shadow_val(BSP_PWM, PWMV2_SHADOW_INDEX(1), s_reload, 0, false);   /* U 上桥 */
    pwmv2_set_shadow_val(BSP_PWM, PWMV2_SHADOW_INDEX(2), s_reload, 0, false);   /* U 下桥 */
    pwmv2_set_shadow_val(BSP_PWM, PWMV2_SHADOW_INDEX(3), s_reload, 0, false);   /* V 上桥 */
    pwmv2_set_shadow_val(BSP_PWM, PWMV2_SHADOW_INDEX(4), s_reload, 0, false);   /* V 下桥 */
    pwmv2_set_shadow_val(BSP_PWM, PWMV2_SHADOW_INDEX(5), s_reload, 0, false);   /* W 上桥 */
    pwmv2_set_shadow_val(BSP_PWM, PWMV2_SHADOW_INDEX(6), s_reload, 0, false);   /* W 下桥 */
    pwmv2_set_shadow_val(BSP_PWM, BSP_PWM_SHADOW_TRIG, 1, 0, false);            /* ADC 触发点(周期起点) */
    pwmv2_shadow_register_lock(BSP_PWM);                                        /* 上锁 shadow */

    /* 三对互补波形: ch0/1(U), ch2/3(V), ch4/5(W)
     * pwmv2_setup_waveform_in_pair() 一次把"上桥+互补下桥+死区"整对配好,
     * 相当于 STM32 里 TIM_OC1Init()+TIM_OC1PreloadConfig()+TIM_BDTRConfig(),
     * 只是这里用一次调用完成一对通道。cmp_start_index 是这对通道用的比较器编号,
     * cmp_num=2 表示上下桥各占一个比较器。 */
    pwmv2_setup_waveform_in_pair(BSP_PWM, BSP_PWM_PHASE_UH, &pwm_cfg, PWMV2_CMP_INDEX(0), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = BSP_PWM_SHADOW_V;                             /* V 上桥 -> shadow[3] */
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(4);                        /* V 下桥 -> shadow[4] */
    pwmv2_setup_waveform_in_pair(BSP_PWM, BSP_PWM_PHASE_VH, &pwm_cfg, PWMV2_CMP_INDEX(4), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = BSP_PWM_SHADOW_W;                             /* W 上桥 -> shadow[5] */
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(6);                        /* W 下桥 -> shadow[6] */
    pwmv2_setup_waveform_in_pair(BSP_PWM, BSP_PWM_PHASE_WH, &pwm_cfg, PWMV2_CMP_INDEX(8), &cmp_cfg[0], 2);

    /* 三个计数器共用 reload shadow, 关闭 burst, reload 时更新
     * (相当于 STM32 的 ARR 预装载: 新周期值在下个周期才生效) */
    for (i = 0; i < 3; i++) {
        pwmv2_counter_select_data_offset_from_shadow_value(BSP_PWM,
            (pwm_counter_t)(pwm_counter_0 + i), BSP_PWM_SHADOW_RELOAD);         /* reload 取自 shadow[0] */
        pwmv2_counter_burst_disable(BSP_PWM, (pwm_counter_t)(pwm_counter_0 + i));
        pwmv2_set_reload_update_time(BSP_PWM, (pwm_counter_t)(pwm_counter_0 + i),
            pwm_reload_update_on_reload);
    }

    /* TRGO -> ADC 触发: 用比较器 16 + shadow[9] 的比较值 1 在周期起点触发采样。
     * 相当于 STM32: TIM_SelectOutputTrigger(TIM1, TIM_TRGO_OC4REF) 后
     * 由 TRGO 去触发 ADC 的注入/规则组。这里中间还过了 TRGM(交叉开关),
     * 见 bsp_adc.c 里 trgm_output_config()。
     * 在"下桥导通中点"采样母线电流是 FOC 的标准做法, 这里用周期起点,
     * 实际项目应根据电流环需求调整 shadow[9] 的值。 */
    pwmv2_select_cmp_source(BSP_PWM, BOARD_BLDCPWM_CMP_TRIG_CMP,
        cmp_value_from_shadow_val, BSP_PWM_SHADOW_TRIG);
    pwmv2_set_trigout_cmp_index(BSP_PWM, BOARD_BLDC_PWM_TRIG_OUT_CHN, BOARD_BLDCPWM_CMP_TRIG_CMP);
    pwmv2_cmp_select_counter(BSP_PWM, BOARD_BLDCPWM_CMP_TRIG_CMP, pwm_counter_0);
    pwmv2_issue_shadow_register_lock_event(BSP_PWM);    /* 立即把上面的 shadow 配置载入 */

    /* 三个计数器同步启动: 保证三相相位严格对齐(0x07 = counter0~2 全部) */
    pwmv2_enable_multi_counter_sync(BSP_PWM, 0x07);
    pwmv2_start_pwm_output_sync(BSP_PWM, 0x07);

    /* 安全起见: 初始化后关闭输出 (相当于 STM32 上电不清 BDTR.MOE) */
    bsp_pwm_enable_output(false);
}

/**
 * @brief  设置单相占空比
 * @param  phase  相选择: bsp_pwm_phase_u / _v / _w
 * @param  duty   占空比 0.0f ~ 1.0f
 *
 * 只是把相名翻译成对应的 shadow 起始索引, 真正干活的是 pwm_phase_update()。
 * 对照 STM32: 相当于封装了一个"往 TIMx->CCR? 写值"的小函数。
 */
void bsp_pwm_set_duty(bsp_pwm_phase_t phase, float duty)
{
    switch (phase) {
    case bsp_pwm_phase_u:
        pwm_phase_update(BSP_PWM_SHADOW_U, duty);
        break;
    case bsp_pwm_phase_v:
        pwm_phase_update(BSP_PWM_SHADOW_V, duty);
        break;
    case bsp_pwm_phase_w:
        pwm_phase_update(BSP_PWM_SHADOW_W, duty);
        break;
    default:
        break;
    }
}

/**
 * @brief  一次性设置三相占空比
 * @param  du/dv/dw  U/V/W 三相占空比, 各 0.0f ~ 1.0f
 *
 * 三相一起给值是 FOC/SVPWM 的常规用法: 电流环算完三个占空比后一次性更新,
 * 避免三相在不同载波周期更新造成电流畸变。三次调用都会在下一次 reload
 * 点一起生效(shadow 机制), 所以这里不需要额外加锁保护。
 */
void bsp_pwm_set_duty_uvw(float du, float dv, float dw)
{
    bsp_pwm_set_duty(bsp_pwm_phase_u, du);
    bsp_pwm_set_duty(bsp_pwm_phase_v, dv);
    bsp_pwm_set_duty(bsp_pwm_phase_w, dw);
}

/* 6 路 PWM 通道的引脚映射表(上/下桥成对), 供统一开关"强制输出"使用 */
static const pwm_channel_t s_all_channels[6] = {
    BOARD_BLDC_UH_PWM_OUTPIN, BOARD_BLDC_UL_PWM_OUTPIN,
    BOARD_BLDC_VH_PWM_OUTPIN, BOARD_BLDC_VL_PWM_OUTPIN,
    BOARD_BLDC_WH_PWM_OUTPIN, BOARD_BLDC_WL_PWM_OUTPIN,
};

/**
 * @brief  使能 / 关闭 6 路 PWM 输出
 * @param  enable  true = 开放输出(正常 PWM 波形); false = 强制全低(电机自由停车)
 *
 * 【原理 / 对照 STM32】
 *   enable = true  : 关掉"软件强制"位, 通道恢复由比较值产生的正常 PWM。
 *                    相当于 STM32 里置位 BDTR.MOE(主输出使能)。
 *
 *   enable = false : 走"软件强制输出"路径 —— 先打开 force 使能, 设置
 *                    立即生效, 再把 6 路强制成低电平。
 *                    相当于 STM32 里清除 BDTR.MOE: 所有 OCx 立即被拉到
 *                    无效电平, 电机因三相下桥全开(或全关)而自由停车。
 *
 *   注意这里 force_output_0 是"强制为 0", 对自举电容供电的下桥驱动而言
 *   三相下管全开可起到刹车/续流作用, 实际板子要结合栅极驱动确认。
 */
void bsp_pwm_enable_output(bool enable)
{
    uint32_t i;

    if (enable) {
        /* 打开输出 = 取消软件强制, 让硬件按比较值正常输出波形 */
        for (i = 0; i < 6; i++) {
            pwmv2_disable_software_force(BSP_PWM, (pwm_channel_t)s_all_channels[i]);
        }
        return;
    }

    /* 关闭输出 = 使能软件强制, 并立即生效(不等 reload) */
    for (i = 0; i < 6; i++) {
        pwmv2_set_force_update_time(BSP_PWM, s_all_channels[i], pwm_force_immediately);
        pwmv2_force_update_time_by_shadow(BSP_PWM, s_all_channels[i], pwm_force_update_shadow_immediately);
        pwmv2_enable_force_by_software(BSP_PWM, s_all_channels[i]);
        pwmv2_enable_software_force(BSP_PWM, s_all_channels[i]);
    }

    /* 把 6 路输出强制拉到低电平 */
    pwmv2_shadow_register_unlock(BSP_PWM);
    for (i = 0; i < 6; i++) {
        pwmv2_force_output(BSP_PWM, s_all_channels[i], pwm_force_output_0, false);
    }
    pwmv2_shadow_register_lock(BSP_PWM);
}

/**
 * @brief  读取 PWM 计数重载值
 * @return 载波周期对应的 tick 数 (reload)
 *
 * 用途: 换算 FOC 里的"每周期时间"、计算死区百分比, 或给上位机显示。
 * 对照 STM32: 相当于读 TIMx->ARR。
 */
uint32_t bsp_pwm_get_reload(void)
{
    return s_reload;
}
