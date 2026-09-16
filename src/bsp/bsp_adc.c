/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * BSP: ADC 采集实现
 *
 * 【与 STM32 ADC 的整体对照】
 *   STM32 (F4/G4 的 ADC + DMA + TIM 触发)        HPM6E8Y (ADC16)
 *   ----------------------------------------------------------------
 *   RCC 使能 ADC 时钟                            board_init_adc_clock(base, true)
 *   ADC_Init(分辨率/扫描/连续/外部触发)          adc16_config_t + adc16_init()
 *   ADC_RegularChannelConfig(通道/采样时间)      adc16_channel_config_t + adc16_init_channel()
 *   ADC_ExternalTrigConv (TIM1_TRGO 触发)        PWM TRGO -> TRGM -> ADC 触发(same_as_input)
 *   规则组 + DMA 循环搬运                       "抢占(Preemption)模式" + DMA + shadow 槽位
 *   ADC_IT_EOC / HAL_ADC_ConvCpltCallback       adc16 interrupt + bsp_adc_conv_complete_cb()
 *   HAL_ADC_GetValue / HAL_ADC_PollForConversion adc16_get_oneshot_result(阻塞读)
 *
 * 需要理解的两个 HPM 特色概念:
 *   1) 抢占模式(Preemption, 简称 PMT): HPM 的 ADC 把"规则组/注入组"的概念统一成
 *      "队列(queue) + 优先级"。"抢占触发"就是高优先级队列, 由硬件事件(这里是 PWM)
 *      触发, 结果通过 DMA 搬到内存数组里, 对应 STM32 的"TIM 触发 + DMA"组合。
 *   2) 结果槽位(slot): DMA 缓冲区里每个触发通道有一个固定槽位,
 *      索引 = trig_ch * 4 字节(见 BSP_ADC_SLOT), 所以取数时不能直接取 [0]。
 */
#include "bsp_adc.h"
#include "board.h"
#include "hpm_adc16_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_trgm_drv.h"

/* 相电流: U 用 ADC2, V 用 ADC3 (与板级 board.h 定义一致) */
#define BSP_ADC_CUR_U_BASE   (BOARD_BLDC_ADC_U_BASE)   /* HPM_ADC2 */
#define BSP_ADC_CUR_V_BASE   (BOARD_BLDC_ADC_V_BASE)   /* HPM_ADC3 */
#define BSP_ADC_CUR_U_CH     (BOARD_BLDC_ADC_CH_U)     /* 5 */
#define BSP_ADC_CUR_V_CH     (BOARD_BLDC_ADC_CH_V)     /* 4 */
#define BSP_ADC_TRG          (BOARD_BLDC_ADC_TRG)      /* ADC16_CONFIG_TRG0A */
#define BSP_ADC_IRQn         BOARD_BLDC_ADC_IRQn       /* IRQn_ADC2 */
#define BSP_ADC_TRIG_FLAG    (BOARD_BLDC_ADC_TRIG_FLAG)/* adc16_event_trig_complete */

/* 母线电压 / 温度 */
#define BSP_ADC_VBUS_BASE    (HPM_ADC3)
#define BSP_ADC_VBUS_CH      (7U)
#define BSP_ADC_TEMP_BASE    (HPM_ADC1)
#define BSP_ADC_TEMP_CH      (10U)

#define BSP_ADC_FULL_SCALE   (65535U)
#define BSP_ADC_REF_MV       (3300U)

#define BSP_ADC_OFFSET_SAMPLES   (256U)
#define BSP_ADC_MCP9700_OFFSET_MV    (500)
#define BSP_ADC_MCP9700_SLOPE_MV_C   (10)

/* 抢占模式 DMA 缓冲区(必须放在 noncacheable 段) */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
static uint32_t s_adc_u_buff[BOARD_BLDC_ADC_PMT_DMA_SIZE_IN_4BYTES];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
static uint32_t s_adc_v_buff[BOARD_BLDC_ADC_PMT_DMA_SIZE_IN_4BYTES];

static volatile uint16_t s_cur_u_raw;
static volatile uint16_t s_cur_v_raw;
static volatile bool s_cur_ready;

static uint16_t s_cur_u_offset = BSP_ADC_FULL_SCALE / 2U;
static uint16_t s_cur_v_offset = BSP_ADC_FULL_SCALE / 2U;

/* 每次结果写在对应 trig 通道的槽位
 * DMA 缓冲区按 4 字节一字排开, 一个触发通道占 4 字节, 所以槽位偏移 = trig_ch*4。
 * 这一点和 STM32 习惯的 "DMA 数组下标 = 规则通道序号" 不同, 是 HPM 特有的。 */
#define BSP_ADC_SLOT ((uint32_t)BSP_ADC_TRG * 4U)

/**
 * @brief  相电流采样完成中断服务函数
 *
 * 【触发路径】
 *   PWM0 周期起点产生 TRGO
 *     -> TRGM 交叉开关(trgm_output_same_as_input)
 *     -> ADC2 / ADC3 的抢占触发 PTRGI0A
 *     -> ADC 采样完成, DMA 把结果写进 s_adc_u_buff / s_adc_v_buff
 *     -> 置"触发完成"中断标志 -> 进入本函数
 *
 * 【做了什么】
 *   1. 判断并清掉 ADC2 的 trig_complete 标志(= STM32 里清 ADC_FLAG_EOC);
 *   2. 从两个 DMA 缓冲区的固定槽位取出原始码值, 缓存到全局变量;
 *   3. 置 s_cur_ready 标志, 并回调 bsp_adc_conv_complete_cb(),
 *      用户可在回调里触发 FOC 电流环计算。
 *
 * 对照 STM32: 等价于 ADC 转换完成中断里读 DR / 读 DMA 数组 + 调用
 *             HAL_ADC_ConvCpltCallback()。这里把 U/V 两路 ADC 放在同一
 *             中断里一起取, 保证同一载波周期的电流是"对齐"的。
 */
SDK_DECLARE_EXT_ISR_M(BSP_ADC_IRQn, bsp_adc_isr)
void bsp_adc_isr(void)
{
    /* 检查是否为"抢占触发完成"事件; 不是就直接退出(共享中断向量) */
    if ((adc16_get_status_flags(BSP_ADC_CUR_U_BASE) & BSP_ADC_TRIG_FLAG) != 0U) {
        adc16_clear_status_flags(BSP_ADC_CUR_U_BASE, BSP_ADC_TRIG_FLAG);   /* 清标志, 防重复进中断 */

        /* 取原始码值(16bit): DMA 缓冲是 uint32_t, 低 16 位有效 */
        s_cur_u_raw = (uint16_t)(s_adc_u_buff[BSP_ADC_SLOT] & 0xFFFFU);
        s_cur_v_raw = (uint16_t)(s_adc_v_buff[BSP_ADC_SLOT] & 0xFFFFU);
        s_cur_ready = true;                                               /* 告诉主循环"有新数据" */

        bsp_adc_conv_complete_cb();                                       /* 用户钩子: 可用来跑 FOC */
    }
}

/**
 * @brief  电流采样完成回调 (弱定义)
 * @note   默认空实现。用户在自己的 .c 里定义同名函数即可覆盖(链接器优先选强符号),
 *         这是嵌入式里常见的"C 语言回调/虚函数"写法, 相当于 STM32 的
 *         __weak void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)。
 *
 *         典型用法: 在这里做 Clarke/Park 变换 + 电流 PI, 实现"电流环与 PWM 同步"。
 */
__attribute__((weak)) void bsp_adc_conv_complete_cb(void)
{
}

/**
 * @brief  初始化相电流采集: PWM 硬件触发 + 抢占模式 + DMA
 * @return status_success 成功; status_fail 失败
 *
 * 【为什么要用"PWM 触发 + DMA"而不是 while 轮询?】
 *   FOC 电流环要求"在确定的时刻采样"——必须在 PWM 下桥导通的中点采样,
 *   否则采到的电流既有开关噪声又没有物理意义。所以不能让 CPU 想采就采,
 *   必须由 PWM 硬件产生触发信号。这也是 STM32 里用 TIM1_TRGO 触发 ADC
 *   (ADC_ExternalTrigConv_T1_CC4 / TRGO) 的原因, 原理完全一致。
 *
 * 【逐个配置的含义, 对照 STM32】
 *   board_init_adc_clock()               -> RCC 使能 ADC 时钟
 *   cfg.res = 16bit                      -> ADC_Resolution_12b 之类
 *   cfg.conv_mode = preemption           -> 高优先级队列(硬件触发一次转一次)
 *   cfg.adc_clk_div = /4                 -> ADC 时钟分频, 保证 < 最高频率
 *   cfg.wait_dis / sel_sync_ahb / ahb_en -> 总线等待与同步方式, 默认值即可
 *   adc16_init_channel(): sample_cycle   -> ADC_SampleTime(采样保持时间, 需匹配源阻抗)
 *   adc16_set_pmt_config(): trig_len=1   -> 每次触发只转换 1 个通道
 *                          inten[0]=true -> 该通道转换完产生中断
 *   adc16_init_pmt_dma(): 绑 DMA 缓冲     -> 类似 HAL_ADC_Start_DMA 的目标数组
 *   trgm_output_config(): 把 PWM TRGO 接到 ADC 触发输入
 *                                        -> 类似配置 TIM1 的 TRGO 输出源
 *   adc16_enable_pmt_queue(): 使能队列    -> 类似 ADC 外部触发使能
 *   adc16_enable_interrupts + intc_*     -> NVIC 使能中断
 *
 * 【DMA 缓冲区必须放 noncacheable 段】
 *   CPU 有 D-Cache, DMA 写内存不经过 Cache。若缓冲区在可缓存区, CPU 读到的
 *   可能是旧数据(或 Cache 回写覆盖 DMA 结果)。所以此处用
 *   ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT 把数组放到非缓存且对齐的段。
 *   对照 STM32H7 也要处理 Cache 一致性(MPU 配置 noncacheable 或手动 Clean/Invalidate)。
 */
static hpm_stat_t bsp_adc_init_current(void)
{
    adc16_config_t cfg;
    adc16_channel_config_t ch_cfg;
    adc16_pmt_config_t pmt_cfg = {0};

    /* 使能 U/V 两路 ADC 的时钟(= RCC 使能) */
    board_init_adc_clock(BSP_ADC_CUR_U_BASE, true);
    board_init_adc_clock(BSP_ADC_CUR_V_BASE, true);

    /* ---- 通用 ADC 配置 ---- */
    adc16_get_default_config(&cfg);         /* 先取默认值, 再改需要的字段 */
    cfg.res = adc16_res_16_bits;            /* 16 位分辨率(对应 ADC_Resolution_x) */
    cfg.conv_mode = adc16_conv_mode_preemption;  /* 抢占模式: 硬件触发的高优先级队列 */
    cfg.adc_clk_div = adc16_clock_divider_4;     /* ADC 内核时钟 = 输入时钟/4 */
    cfg.wait_dis = false;
    cfg.sel_sync_ahb = false;
    cfg.adc_ahb_en = true;

    if (adc16_init(BSP_ADC_CUR_U_BASE, &cfg) != status_success) {
        return status_fail;
    }
    if (adc16_init(BSP_ADC_CUR_V_BASE, &cfg) != status_success) {
        return status_fail;
    }

    /* ---- 通道配置: 采样时间 + 通道号 ---- */
    adc16_get_channel_default_config(&ch_cfg);
    ch_cfg.sample_cycle = 20;               /* 采样保持周期数(= ADC_SampleTime) */
    ch_cfg.ch = BSP_ADC_CUR_U_CH;           /* U 相电流输入通道 */
    if (adc16_init_channel(BSP_ADC_CUR_U_BASE, &ch_cfg) != status_success) {
        return status_fail;
    }
    ch_cfg.ch = BSP_ADC_CUR_V_CH;           /* V 相电流输入通道 */
    if (adc16_init_channel(BSP_ADC_CUR_V_BASE, &ch_cfg) != status_success) {
        return status_fail;
    }

    /* ---- 抢占配置: 同一个触发通道, 每次触发转换 1 个通道 ----
     * trig_ch    : 使用哪个触发通道(TRG0A), 决定结果落在 DMA 缓冲的哪个槽位
     * trig_len   : 本次触发连续转换几个通道, 这里 1 个
     * inten[0]   : 转换完成后是否上报中断
     * adc_ch[0]  : 第 0 个被转换的通道号 */
    pmt_cfg.trig_ch = BSP_ADC_TRG;
    pmt_cfg.trig_len = 1;
    pmt_cfg.inten[0] = true;
    pmt_cfg.adc_ch[0] = BSP_ADC_CUR_U_CH;
    adc16_set_pmt_config(BSP_ADC_CUR_U_BASE, &pmt_cfg);

    pmt_cfg.adc_ch[0] = BSP_ADC_CUR_V_CH;   /* V 相复用同一套触发/中断配置 */
    adc16_set_pmt_config(BSP_ADC_CUR_V_BASE, &pmt_cfg);

    /* ---- 绑定 DMA 目标缓冲区 ----
     * HPM 的 DMA 只认"系统地址", 而缓冲区可能在本核的 local RAM,
     * 所以要用 core_local_mem_to_sys_address() 做地址转换。
     * 对照 STM32: 相当于给 DMA 外设配置 DMA 目的地址。 */
    adc16_init_pmt_dma(BSP_ADC_CUR_U_BASE,
        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&s_adc_u_buff[0]));
    adc16_init_pmt_dma(BSP_ADC_CUR_V_BASE,
        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&s_adc_v_buff[0]));

    /* ---- PWM0 TRGO -> TRGM 交叉开关 -> ADC 抢占触发 ----
     * HPM 的 TRGM 是片上"信号交叉开关", 可以把任意片上信号路由到任意目的。
     * 这里把 PWM0 的 TRGO 输入直连到 ADC 的 PTRGI0A 触发输入。
     * 对照 STM32: STM32 的外部触发源是写死在 ADC_ExternalTrigConv 里的
     *             (如 TIM1_TRGO), 没有 TRGM 这种可编程路由矩阵;
     *             HPM 更灵活, 但也要多配一步 TRGM。 */
    {
        trgm_output_t trgm_cfg;
        trgm_cfg.invert = false;                        /* 触发信号不反相 */
        trgm_cfg.type = trgm_output_same_as_input;      /* 输出 = 输入(直通) */
        trgm_cfg.input = BOARD_BLDC_PWM_TRG_ADC;        /* 输入源: PWM0_TRGO_0 */
        trgm_output_config(BOARD_BLDCPWM_TRGM, BOARD_BLDC_TRG_ADC, &trgm_cfg);  /* 输出到 ADC 触发 */
    }

    /* ---- 使能抢占队列(相当于打开 ADC 的外部触发) ---- */
    adc16_enable_pmt_queue(BSP_ADC_CUR_U_BASE, BSP_ADC_TRG);
    adc16_enable_pmt_queue(BSP_ADC_CUR_V_BASE, BSP_ADC_TRG);

    /* ---- 开中断并使能 NVIC ---- */
    adc16_enable_interrupts(BSP_ADC_CUR_U_BASE, BSP_ADC_TRIG_FLAG);
    intc_m_enable_irq_with_priority(BSP_ADC_IRQn, 1);   /* 优先级 1, 需早于普通任务响应 */

    return status_success;
}

/**
 * @brief  初始化一个"单次转换/阻塞读取"的 ADC 通道
 * @param  base  ADC 外设基址 (如 HPM_ADC1 / HPM_ADC3)
 * @param  ch    通道号
 * @return status_success / status_fail
 *
 * 用于母线电压、温度这类"慢变量": 不需要和 PWM 严格同步, 也不需要 DMA,
 * 直接需要时读一次即可(阻塞方式, 类似 HAL_ADC_PollForConversion + HAL_ADC_GetValue)。
 *
 * 对照 STM32:
 *   adc16_set_blocking_read(base)  = 打开"读结果寄存器时自动启动转换"的便捷模式,
 *                                    相当于 HAL_ADC_Start() 后轮询等待 EOC;
 *   adc16_enable_oneshot_mode()    = 单次转换模式(转完就停), 相当于
 *                                    ADC_ContinuousConvMode = DISABLE。
 */
static hpm_stat_t bsp_adc_init_oneshot(ADC16_Type *base, uint8_t ch)
{
    adc16_config_t cfg;
    adc16_channel_config_t ch_cfg;

    board_init_adc_clock(base, true);       /* RCC 使能该 ADC 时钟 */

    adc16_get_default_config(&cfg);         /* 通用配置, 与电流用的 ADC 相同 */
    cfg.res = adc16_res_16_bits;
    cfg.adc_clk_div = adc16_clock_divider_4;
    cfg.wait_dis = false;
    cfg.sel_sync_ahb = false;
    cfg.adc_ahb_en = true;
    if (adc16_init(base, &cfg) != status_success) {
        return status_fail;
    }

    adc16_get_channel_default_config(&ch_cfg);
    ch_cfg.sample_cycle = 20;               /* 采样保持周期, 慢变量可以放宽 */
    ch_cfg.ch = ch;
    if (adc16_init_channel(base, &ch_cfg) != status_success) {
        return status_fail;
    }

    adc16_set_blocking_read(base);          /* 阻塞读模式: 读一次 = 转一次 */
#if defined(ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT) && ADC_SOC_BUSMODE_ENABLE_CTRL_SUPPORT
    adc16_enable_oneshot_mode(base);        /* 单次转换(转完停), 需芯片支持 */
#endif
    return status_success;
}

/**
 * @brief  初始化整个 ADC BSP
 * @note   依次初始化:
 *           1) 相电流      -> PWM 触发 + 抢占 + DMA + 中断(实时, 用于 FOC)
 *           2) 母线电压    -> ADC3 单次阻塞读
 *           3) 板载温度    -> ADC1 单次阻塞读
 *         对照 STM32: 相当于把 MX_ADC1_Init()/MX_ADC2_Init() 集中到一个入口里。
 */
void bsp_adc_init(void)
{
    s_cur_ready = false;    /* 还没采到数据 */

    (void)bsp_adc_init_current();
    (void)bsp_adc_init_oneshot(BSP_ADC_VBUS_BASE, BSP_ADC_VBUS_CH);
    (void)bsp_adc_init_oneshot(BSP_ADC_TEMP_BASE, BSP_ADC_TEMP_CH);
}

/**
 * @brief  查询是否有新的相电流采样结果
 * @return true = 自上次查询以来至少完成过一次电流采样
 * @note   s_cur_ready 由中断置位, 主循环可据此决定是否更新控制量。
 *         更严谨的用法是在 bsp_adc_conv_complete_cb() 里直接做电流环,
 *         让"采样->计算->更新 PWM"严格发生在一个载波周期内。
 */
bool bsp_adc_current_ready(void)
{
    return s_cur_ready;
}

/**
 * @brief  读取三相电流原始码值
 * @param  iu,iv,iw  输出指针, 可传 NULL 表示不需要该相
 *
 * 【为什么 W 相不用采?】
 *   三相绕组星形连接, 任意时刻 iu + iv + iw = 0, 所以知道两相就能算出第三相。
 *   本板子只做了两路电流采样(两电阻方案), iw = -(iu+iv) 反推即可。
 *   代码里以"零点码值"为中心做减法, 结果再限幅到 [0, FULL_SCALE]。
 *   对照 STM32: 就是 INA/运放 -> ADC 通道 -> 读 DR 的过程, 这里是读 DMA 缓存。
 *
 * @note   返回的是"未扣零点"的原始码值(相对零点)。
 */
void bsp_adc_get_currents_raw(uint16_t *iu, uint16_t *iv, uint16_t *iw)
{
    uint16_t u = s_cur_u_raw;
    uint16_t v = s_cur_v_raw;
    /* 三相电流之和为零: iw = -(iu + iv), 以零点为中心 */
    int32_t w = (int32_t)(2 * (int32_t)s_cur_u_offset) - (int32_t)u - (int32_t)v;

    if (iu) { *iu = u; }
    if (iv) { *iv = v; }
    /* 限幅, 防止反推值溢出 uint16_t */
    if (iw) { *iw = (uint16_t)((w < 0) ? 0 : ((w > (int32_t)BSP_ADC_FULL_SCALE) ? BSP_ADC_FULL_SCALE : w)); }
}

/**
 * @brief  读取三相电流 (单位 mA, 已扣除零点并换算)
 * @param  iu,iv,iw  输出指针(mA), 可传 NULL
 *
 * 【换算公式推导】
 *   硬件: 采样电阻 R_shunt(10 mΩ) -> 运放放大 G(10 倍) -> ADC 输入。
 *   电流 I 在 ADC 引脚上的电压: V = I * R_shunt * G
 *   ADC 码值      : code = V / Vref * FULL_SCALE
 *   所以          : I = (code - offset) * Vref / (FULL_SCALE * R_shunt * G)
 *
 *   代码里为避免浮点, 把 R_shunt 用 mΩ 表示(10)、G 用"×10"表示(100),
 *   于是分母 = FULL_SCALE * R_shunt_mohm * GAIN_X10, 分子再乘 10 抵消单位,
 *   最终单位正好是 mA。对照 STM32 里常见的
 *     current = (adc - offset) * Vref / 4096 / R_shunt / gain
 *   是完全一样的公式, 只是这里做了整数化处理。
 *
 * iw 同样由 iu + iv = 0 得到。
 */
void bsp_adc_get_currents_ma(int32_t *iu, int32_t *iv, int32_t *iw)
{
    /* 每 LSB 对应电流(µA): Vref_mV / FS / (Rshunt * gain) * 1000 */
    /* counts -> mA: counts * Vref_mV / FS / (Rshunt_ohm * gain) */
    const int32_t iu_cnt = (int32_t)s_cur_u_raw - (int32_t)s_cur_u_offset;   /* 减去零点偏移 */
    const int32_t iv_cnt = (int32_t)s_cur_v_raw - (int32_t)s_cur_v_offset;
    /* shunt_ohm = mohm/1000, gain = GAIN_X10/10  => 分母 *10 */
    const int32_t denom = (int32_t)BSP_ADC_CURRENT_SHUNT_MOHM * (int32_t)BSP_ADC_CURRENT_GAIN_X10;
    /* 用 int64 中间量防溢出: 码值 * 3300 * 10 可能超过 32bit */
    int32_t u_ma = (denom != 0) ? (int32_t)(((int64_t)iu_cnt * BSP_ADC_REF_MV * 10) / ((int64_t)BSP_ADC_FULL_SCALE * denom)) : 0;
    int32_t v_ma = (denom != 0) ? (int32_t)(((int64_t)iv_cnt * BSP_ADC_REF_MV * 10) / ((int64_t)BSP_ADC_FULL_SCALE * denom)) : 0;

    if (iu) { *iu = u_ma; }
    if (iv) { *iv = v_ma; }
    if (iw) { *iw = -(u_ma + v_ma); }
}

/**
 * @brief  读取母线电压 (单位 mV)
 * @return 母线电压 mV; 转换失败返回 0
 *
 * 【原理】
 *   母线电压很高(可能几十伏), 不能直接进 ADC, 先用分压电阻
 *   R_top(200k) / R_bottom(10k) 降压:
 *       Vpin = Vbus * R_bottom / (R_top + R_bottom)
 *   反解:  Vbus = code / FULL_SCALE * Vref * (R_top + R_bottom) / R_bottom
 *   代码里为了保留精度先做乘法再除, 并加 den/2 实现四舍五入。
 *
 * 对照 STM32: 就是 HAL_ADC_GetValue() 读规则通道 + 软件分压换算。
 */
uint32_t bsp_adc_get_vbus_mv(void)
{
    uint16_t raw = 0;
    uint64_t num, den;

    /* 阻塞式读一次转换结果(内部会自动启动一次单次转换) */
    if (adc16_get_oneshot_result(BSP_ADC_VBUS_BASE, BSP_ADC_VBUS_CH, &raw) != status_success) {
        return 0;
    }

    /* Vbus = raw/FS * Vref * (Rtop+Rbot)/Rbot, 先乘后除避免精度损失 */
    num = (uint64_t)raw * BSP_ADC_REF_MV * (BSP_ADC_VBUS_TOP_OHM + BSP_ADC_VBUS_BOTTOM_OHM);
    den = (uint64_t)BSP_ADC_FULL_SCALE * BSP_ADC_VBUS_BOTTOM_OHM;
    return (uint32_t)((num + (den / 2U)) / den);        /* +den/2 四舍五入 */
}

/**
 * @brief  读取板载温度传感器 (MCP9700) 温度
 * @return 温度, 单位 0.001 ℃ (毫摄氏度); 转换失败返回 0
 *
 * 【原理】
 *   MCP9700 是模拟输出温度传感器, 特性为:
 *       25℃ 时输出 500 mV, 之后每升高 1℃ 输出增加 10 mV:
 *       Vout(mV) = 500 + 10 * T(℃)
 *   先由 ADC 码值算出电压 mV, 再反解:
 *       T(℃) = (Vout_mV - 500) / 10
 *   函数返回毫摄氏度(×1000), 避免浮点。
 *
 * 对照 STM32: 与"读 NTC 分压 + 查表/公式换算"性质相同, 只是传感器是线性的。
 */
int32_t bsp_adc_get_temperature_mc(void)
{
    uint16_t raw = 0;
    int32_t mv;

    if (adc16_get_oneshot_result(BSP_ADC_TEMP_BASE, BSP_ADC_TEMP_CH, &raw) != status_success) {
        return 0;
    }

    /* 码值 -> 引脚电压 (mV), 四舍五入 */
    mv = (int32_t)(((uint64_t)raw * BSP_ADC_REF_MV + (BSP_ADC_FULL_SCALE / 2U)) / BSP_ADC_FULL_SCALE);
    /* 线性反解 -> 毫摄氏度 */
    return (mv - BSP_ADC_MCP9700_OFFSET_MV) * 1000 / BSP_ADC_MCP9700_SLOPE_MV_C;
}

/**
 * @brief  相电流零点校准
 * @note   必须在"电机静止、PWM 输出关闭"时调用。
 *
 * 【为什么需要校准?】
 *   运放/采样链路的零点通常不在码值中点(65535/2), 会有几到几十 LSB 的偏差。
 *   若直接按理论中点算, 会引入一个恒定的电流偏置, FOC 里表现为转矩脉动。
 *   做法是连续采 N 次(此时电流为 0), 取平均值作为新的零点 offset,
 *   之后所有电流换算都减去这个 offset。
 *
 * 对照 STM32: 等价于 FOC 初始化时的 "offset calibration" —— 关 PWM,
 *             多次读 ADC 求平均, 存入 Ia_offset/Ib_offset。
 *
 * @note   实现上是连续读 N 次全局变量(由中断持续刷新), 因此调用期间应保证
 *         PWM 已关闭且 ADC 触发仍在运行, 才能采到真实零点。
 */
void bsp_adc_calibrate(void)
{
    uint32_t i;
    uint32_t u_sum = 0;
    uint32_t v_sum = 0;

    for (i = 0; i < BSP_ADC_OFFSET_SAMPLES; i++) {
        u_sum += s_cur_u_raw;
        v_sum += s_cur_v_raw;
    }
    /* 平均值作为零点, 后续换算全部减去它 */
    s_cur_u_offset = (uint16_t)(u_sum / BSP_ADC_OFFSET_SAMPLES);
    s_cur_v_offset = (uint16_t)(v_sum / BSP_ADC_OFFSET_SAMPLES);
}
