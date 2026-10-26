#include "type.h"
#include "hpm_spi_drv.h"
#include "hpm_gpio_drv.h"
#include "board.h"

/* ---- SPI1 TX DMA 需要的头 ---- */
#include "hpm_dmav2_drv.h"      /* DMAV2(HDMA): dma_setup_handshake / 状态查询 */
#include "hpm_dmamux_drv.h"     /* DMAMUX: 把外设请求线接到某个 DMA 通道      */
#include "hpm_l1c_drv.h"        /* D-Cache 维护: 启动 DMA 前兜底 writeback    */
#include "hpm_interrupt.h"      /* SDK_DECLARE_EXT_ISR_M / intc_m_enable_irq_* */
/* RTOS: DMA 完成用信号量通知, 传输期间让出 CPU(见文件下半部分说明) */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "app_rtos.h"           /* APP_IRQ_PRIO_LCD_DMA (core1 的优先级表) */
#include "app_log.h"            /* LOG_*: core1 没有控制台, 日志走共享内存 -> core0 */

#include "LCDHw.h"
#include "stdlib.h"






#define LCD_GPIO_CTRL  HPM_GPIO0

/* 引脚定义集中在 LCDHw.h(LCD_PAD_*), 这样 LCDTouch.c 能对"触摸软件 SPI 有没有
 * 撞 LCD 引脚"做编译期断言(见 LCDTouch.c 顶部)。这里保留原来的名字做别名。 */
#define LCDGPIO_CS   LCD_PAD_CS         /* 片选引脚        PC11 */
#define LCDGPIO_LED  LCD_PAD_LED        /* 背光控制引脚    PC08 */
#define LCDGPIO_RS   LCD_PAD_RS         /* 寄存器/数据选择 PC04 */
#define LCDGPIO_RST  LCD_PAD_RST        /* 复位引脚        PC03 */

/* SPI1 TX DMA 通路初始化: 实现放在下面的 DMA 段里, 而 LCD_SPIInit() 要调用它,
 * 所以这里前置声明一下。 */
static void lcd_dma_init(void);

 
//GPIO置位（拉高）
#define	LCD_CS_SET  LCD_GPIO_SET(LCDGPIO_CS,1)    //片选端口  	PB11
#define	LCD_RS_SET	LCD_GPIO_SET(LCDGPIO_RS,1)    //数据/命令  PB10	  
#define	LCD_RST_SET	LCD_GPIO_SET(LCDGPIO_RST,1)   //复位			  PB12

//GPIO复位（拉低）							    
#define	LCD_CS_CLR  LCD_GPIO_SET(LCDGPIO_CS,0)     //片选端口  	PB11
#define	LCD_RS_CLR	LCD_GPIO_SET(LCDGPIO_RS,0)     //数据/命令  PB10	 
#define	LCD_RST_CLR	LCD_GPIO_SET(LCDGPIO_RST,0)    //复位			  PB12


void LCD_GPIOInit(void)
{
	//spi gpio config
	/* set max frequency slew rate(200M) */
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_GPIO_C_11;
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PC10_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_SPI1_MISO;
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PC13_FUNC_CTL_SPI1_MOSI;
   
	/* set max frequency slew rate(200M) */
	HPM_IOC->PAD[IOC_PAD_PC11].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);
	HPM_IOC->PAD[IOC_PAD_PC10].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);
	HPM_IOC->PAD[IOC_PAD_PC12].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);
	HPM_IOC->PAD[IOC_PAD_PC13].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);


	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_CS),GPIO_GET_PIN_INDEX(LCDGPIO_CS), 1);

	//gpio config for LCD control pin
	HPM_IOC->PAD[LCDGPIO_LED].FUNC_CTL = IOC_PC08_FUNC_CTL_GPIO_C_08;
    HPM_IOC->PAD[LCDGPIO_LED].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                        IOC_PAD_PAD_CTL_SPD_SET(3) |
                                        IOC_PAD_PAD_CTL_DS_SET(7) |
                                        IOC_PAD_PAD_CTL_OD_SET(0) |
                                        IOC_PAD_PAD_CTL_PE_SET(1) |
                                        IOC_PAD_PAD_CTL_PS_SET(1) |
                                        IOC_PAD_PAD_CTL_PRS_SET(1);

	HPM_IOC->PAD[LCDGPIO_RST].FUNC_CTL = IOC_PC03_FUNC_CTL_GPIO_C_03;
    HPM_IOC->PAD[LCDGPIO_RST].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);

	HPM_IOC->PAD[LCDGPIO_RS].FUNC_CTL = IOC_PC04_FUNC_CTL_GPIO_C_04;
    HPM_IOC->PAD[LCDGPIO_RS].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);

	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_LED),GPIO_GET_PIN_INDEX(LCDGPIO_LED), 1);
	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_RST),GPIO_GET_PIN_INDEX(LCDGPIO_RST), 1);
	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_RS),GPIO_GET_PIN_INDEX(LCDGPIO_RS), 1);
}


void LCD_GPIO_SET(u32 pin,u8 val)
{
	gpio_write_pin(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin), val);
}

#define LCD_SPI                    HPM_SPI1
#define LCD_SPI_SCLK_FREQ          (32000000UL)
#define LCD_SPI_DATA_LEN_BITS      (8U)
#define LCD_SPI_DATA_LEN_BYTES     ((LCD_SPI_DATA_LEN_BITS + 7) / 8)
#if (SPI_SOC_TRANSFER_COUNT_MAX == 512)
#if (LCD_SPI_DATA_LEN_BYTES > 2)
#define TRANSFER_COUNT_MAX          (SPI_SOC_TRANSFER_COUNT_MAX * 4)
#else
#define TRANSFER_COUNT_MAX          (SPI_SOC_TRANSFER_COUNT_MAX * LCD_SPI_DATA_LEN_BYTES)
#endif
#else
#define TRANSFER_COUNT_MAX          SPI_SOC_TRANSFER_COUNT_MAX
#endif



int LCD_SPIInit(void)
{
    spi_timing_config_t timing_config;
    spi_format_config_t format_config;
    uint32_t src_clk;
    uint32_t best_sclk = 0U;
    uint32_t best_error = 0xFFFFFFFFU;
    uint32_t div;

    /* 时钟组 1 = 挂到 CPU1(core1): 双核之后 LVGL/SPI1 整个跑在 core1 上。
     * 组的含义是"只要这个 CPU 不进低功耗, 组内外设的时钟就不停", 所以外设归哪个核
     * 用, 就加到哪个组。board_init_clock() 里已经 clock_connect_group_to_cpu(1, 1)。 */
    clock_add_to_group(clock_spi1, 1);
    /* pins init*/
	//board_init_spi_pins_with_gpio_as_cs(LCD_SPI);
	LCD_GPIOInit();//LCD GPIO初始化

    src_clk = clock_get_frequency(clock_spi1);
    for (div = 2U; div <= 510U; div += 2U) {
        uint32_t sclk;
        uint32_t error;

        if ((src_clk % div) != 0U) {
            continue;
        }
        sclk = src_clk / div;
        error = (sclk > LCD_SPI_SCLK_FREQ) ?
                (sclk - LCD_SPI_SCLK_FREQ) : (LCD_SPI_SCLK_FREQ - sclk);
        if (error < best_error) {
            best_error = error;
            best_sclk = sclk;
        }
    }
    if (best_sclk == 0U) {
        return -1;
    }

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = src_clk;
    timing_config.master_config.sclk_freq_in_hz = best_sclk;
    if (spi_master_timing_init(LCD_SPI, &timing_config) != status_success) {
        LOG_E("LCD", "spi_master_timing_init 失败 (src=%u, want=%u)", src_clk, best_sclk);
        return -1;
    }
    /* 日志里留一行实参: ILI9341 串行写周期最小 100ns(=10MHz), 超太多初始化就会失败 */
    LOG_I("LCD", "SPI1 src_clk=%u Hz -> sclk=%u Hz (target %u)",
          src_clk, best_sclk, (unsigned)LCD_SPI_SCLK_FREQ);

    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = LCD_SPI_DATA_LEN_BITS;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    format_config.common_config.lsb = false;
    spi_format_init(LCD_SPI, &format_config);

	/* SPI1 TX DMA 通路(全局唯一使用 HDMA 的地方)。放在这里是因为它必须在
	 * 任何一次像素写之前就绪 —— LCD_Init() 末尾的 LCD_Clear() 就会用到。 */
	lcd_dma_init();

	return 0;
}


/* ======================================================================
 *  像素通路失败计数 —— 定位"不规则长条 / 花屏"的证据
 *
 * 【为什么需要它】hpm_spi_drv.c 的 spi_write_data() 只会在 TX FIFO 满时空转,
 * 【不会丢字节】: 被中断/任务抢占只是让它变慢。所以"轮询写被抢占 -> 少发几个
 * 字节"这条因果链是站不住的, 不能当结论写。
 *
 * 真正能丢数据的只有 spi_transfer() 的两个返回值(hpm_spi_drv.c:434~503):
 *   · status_spi_master_busy : 进函数时发现 SPIACTIVE 还置着 -> 立刻返回,
 *                              【一个字节都没发】。对 LCD 来说 = 这条命令/参数/
 *                              整块像素被吞掉。
 *   · status_timeout         : spi_wait_for_idle_status() 的 5000 次重试耗尽后
 *                              返回(超时的是"等待空闲", 数据其实还在往外移)。
 * 注意这两个是【量级相当】的: 一次 512B 传输在 25MHz 下要 164us, 而 5000 次
 * 空转在几百 MHz 的核上只有几十~几百 us —— 到底会不会踩到, 必须实测。
 *
 * 原来的代码: 前者 printf 一行了事(显示任务里同步写 921600bps 串口!), 后者
 * 完全被忽略。这里改成分类计数, 由显示任务每 5 秒走日志队列汇总一行,
 * 上板跑一次就能判断到底有没有丢、丢的是哪一类。
 * ====================================================================== */
static lcd_tx_stats_t s_lcd_stats;

static void lcd_stat_fail(hpm_stat_t stat)
{
	s_lcd_stats.last_status = (u32)stat;
	if (stat == status_spi_master_busy) {
		s_lcd_stats.busy_count++;
	} else if (stat == status_timeout) {
		s_lcd_stats.timeout_count++;
	} else {
		s_lcd_stats.fail_count++;
	}
}

void LCD_GetTxStats(lcd_tx_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	*out = s_lcd_stats;
}

/* ======================================================================
 *  SPI1 像素通路: TX DMA(本分支的候选根治方案, 需上板验证)
 *
 * 【解决的是哪一件事】原来的 bulk 写(512B/块)是纯 CPU 轮询喂 TX FIFO:
 *   while (transferred < count) { if (!(TXFULL)) ptr->DATA = 下一字节; }
 *   TX FIFO 只有 8 字节深(SPI_SOC_FIFO_DEPTH), 33MHz 下只够 ~1.9us。
 *   于是: (a) 显示任务(最低优先级)在整个刷屏期间占着 CPU 空转, 控制任务每 1ms
 *   只能等它让出; (b) 一旦 CPU 被中断/任务抽走, FIFO 见底。
 *   把"喂 FIFO"这件事交给 DMA 之后, 上面两条同时消失: 数据流由硬件保证,
 *   显示任务在传输期间阻塞在信号量上, 控制任务的 1ms 节拍完全不受影响。
 *
 * 【注意】本改动并不能替代"实验 1"的结论: 如果实测 spi_transfer 的 busy/timeout
 *   计数恒为 0(即一个字节都没丢), 那"长条"的根因就在 LVGL/窗口命令那一侧,
 *   这个 DMA 分支就不是解药, 不要合。判定见 docs/reports/LCD_DISPLAY_FIX_REPORT.md 第 3 节。
 *
 * 【顺序严格照 hpm_spi_drv.h 的要求】先关 SPI 的 TX DMA 请求 -> 配 SPI 本次传输
 *   -> 配好并启动 DMA 通道 -> 再打开请求(数据此时才开始流动) -> 等 TC -> 等 SPI 空闲。
 *
 * 【缓冲区】源缓冲必须放 noncacheable 段(与 ADC 抢占 DMA 同理, 见 bsp_adc.c):
 *   DMA 不过 D-Cache, 若在可缓存段, CPU 刚写的字节可能还在 Cache 里。
 *   这里用 ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT 放进 AXI_SRAM_NONCACHEABLE
 *   (0x01280000 起, board_init_pmp() 已配成 non-cacheable), 启动前再 writeback 兜底。
 *
 * 【出错兜底】DMA 初始化失败或单块传输失败都会自动退回 CPU 轮询写; 连续失败
 *   LCD_TX_DMA_FAIL_LIMIT 次则永久退回 —— 绝不会因为 DMA 没配好而黑屏。
 * ====================================================================== */
#define LCD_TX_DMA_ENABLE        (1U)   /* 置 0 = 直接使用 CPU 轮询写(代码仍编译) */
#define LCD_TX_DMA               HPM_HDMA
#define LCD_TX_DMAMUX            HPM_DMAMUX
#define LCD_TX_DMA_CHANNEL       (0U)   /* 全工程没有别的模块使用 HDMA */
#define LCD_TX_DMA_REQ_SRC       (HPM_DMA_SRC_SPI1_TX)
/* 【不能带括号】SDK_DECLARE_EXT_ISR_M 里是 `irq_handler_wrapper_##irq_num`,
 *   带括号的话会把 "_" 和 "(" 粘在一起 -> "pasting ... does not give a valid
 *   preprocessing token"。必须是一个光秃秃的记号(和 bsp_adc.c 的写法一致)。 */
#define LCD_TX_DMA_IRQ           IRQn_HDMA
#define LCD_TX_DMA_MAX_BYTES     (512U) /* 保守上限: SPI_TRANSCTRL.WRTRANCNT 只有 9bit */
#define LCD_TX_DMA_WAIT_MS       (50U)  /* 单块死等保护: 512B@25MHz 只要 ~165us */
#define LCD_TX_DMA_FAIL_LIMIT    (8U)

#define LCD_PIXEL_TRANSFER_BUFFER_SIZE (512U)
#if (LCD_PIXEL_TRANSFER_BUFFER_SIZE > LCD_TX_DMA_MAX_BYTES)
#error "pixel transfer buffer must not exceed LCD_TX_DMA_MAX_BYTES"
#endif

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(32)
static u8 s_lcd_tx_buffer[LCD_PIXEL_TRANSFER_BUFFER_SIZE];

static SemaphoreHandle_t s_lcd_dma_sem;
static volatile u32 s_lcd_dma_status = DMA_CHANNEL_STATUS_ONGOING;
static u8 s_lcd_dma_fail_streak;

/* DMA 完成中断: 只"记状态 + 放信号量"(短 ISR)。
 * dma_check_transfer_status() 是 W1C(读即清标志), 所以状态必须在这里存下来 ——
 * 任务里再读一次只会读到 ONGOING。 */
SDK_DECLARE_EXT_ISR_M(LCD_TX_DMA_IRQ, lcd_hw_dma_isr)
void lcd_hw_dma_isr(void)
{
    BaseType_t woken = pdFALSE;
    u32 st = dma_check_transfer_status(LCD_TX_DMA, LCD_TX_DMA_CHANNEL);

    if (st != DMA_CHANNEL_STATUS_ONGOING) {
        s_lcd_dma_status = st;
        if (s_lcd_dma_sem != NULL) {
            (void)xSemaphoreGiveFromISR(s_lcd_dma_sem, &woken);
        }
    }
    portYIELD_FROM_ISR(woken);
}

/* DMA 通路初始化(调度器启动前由 LCD_SPIInit() 调用) */
static void lcd_dma_init(void)
{
    if (s_lcd_stats.dma_disabled != 0U) {
        return;
    }

    clock_add_to_group(clock_hdma, 1);      /* 时钟组 1 = CPU1(core1); board_init_clock() 已把 hdma 加进组 0, 这里再挂到组 1, 幂等 */
    s_lcd_dma_sem = xSemaphoreCreateBinary();
    if (s_lcd_dma_sem == NULL) {
        s_lcd_stats.dma_disabled = 1U;      /* 堆不够: 老老实实轮询写 */
        return;
    }

    /* DMAMUX: HDMA 通道 n 的请求源 = DMAMUX 通道 n(DMAMUX_MUXCFG_HDMA_MUX0 == 0) */
    dmamux_config(LCD_TX_DMAMUX, LCD_TX_DMA_CHANNEL, LCD_TX_DMA_REQ_SRC, true);

    /* 优先级必须 >= configMAX_SYSCALL_INTERRUPT_PRIORITY(=4), 因为 ISR 里要放
     * FreeRTOS 信号量; 取 6 比 USB(5) 低, 见 app_rtos.h 的中断优先级表。 */
    intc_m_enable_irq_with_priority(LCD_TX_DMA_IRQ, APP_IRQ_PRIO_LCD_DMA);

    s_lcd_stats.dma_ready = 1U;
}

/* 等一次 DMA 传输结束。status_success = 本次搬完。 */
static hpm_stat_t lcd_dma_wait(void)
{
    u32 st;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* 关键: 阻塞让出 CPU。刷屏期间控制任务照常拿 1ms 节拍。 */
        if (xSemaphoreTake(s_lcd_dma_sem, pdMS_TO_TICKS(LCD_TX_DMA_WAIT_MS)) != pdTRUE) {
            (void)dma_check_transfer_status(LCD_TX_DMA, LCD_TX_DMA_CHANNEL);
            dma_disable_channel(LCD_TX_DMA, LCD_TX_DMA_CHANNEL);
            s_lcd_stats.dma_error_count++;
            return status_timeout;
        }
        st = s_lcd_dma_status;
    } else {
        /* 上电初始化阶段(还没起调度器): 不能调 xSemaphoreTake, 自旋等中断 */
        u32 guard = 0U;

        do {
            st = s_lcd_dma_status;
            if (st != DMA_CHANNEL_STATUS_ONGOING) {
                break;
            }
            guard++;
        } while (guard < 2000000U);

        if (st == DMA_CHANNEL_STATUS_ONGOING) {
            st = dma_check_transfer_status(LCD_TX_DMA, LCD_TX_DMA_CHANNEL);
            if (st == DMA_CHANNEL_STATUS_ONGOING) {
                dma_disable_channel(LCD_TX_DMA, LCD_TX_DMA_CHANNEL);
                s_lcd_stats.dma_error_count++;
                return status_timeout;
            }
        }
    }

    s_lcd_stats.last_dma_status = st;
    if ((st & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) != 0U) {
        s_lcd_stats.dma_error_count++;
        return status_fail;
    }
    if ((st & DMA_CHANNEL_STATUS_TC) == 0U) {
        s_lcd_stats.dma_error_count++;
        return status_timeout;
    }

    s_lcd_stats.dma_chunk_count++;
    return status_success;
}

/* 用 DMA 把 len 字节写到 SPI1(CS/RS 由调用者管) */
static hpm_stat_t lcd_dma_write_block(const u8 *buff, u32 len)
{
    spi_control_config_t cfg;
    dma_handshake_config_t hs;
    hpm_stat_t stat;

    /* 1) 上一块必须彻底移出, 否则 spi_setup_dma_transfer 会直接回 busy */
    if (spi_wait_for_idle_status(LCD_SPI) != status_success) {
        s_lcd_stats.dma_error_count++;
        return status_timeout;
    }
    spi_disable_tx_dma(LCD_SPI);

    /* 2) 配 SPI(软复位 + 方向/长度 + CMD/ADDR 相位), 但先不开 DMA 请求 */
    spi_master_get_default_control_config(&cfg);
    cfg.common_config.trans_mode = spi_trans_write_only;
    cfg.common_config.data_phase_fmt = spi_single_io_mode;
    cfg.common_config.tx_dma_enable = false;
    cfg.common_config.rx_dma_enable = false;
    if (spi_setup_dma_transfer(LCD_SPI, &cfg, NULL, NULL, len, 0U) != status_success) {
        s_lcd_stats.dma_error_count++;
        return status_fail;
    }

    /* 3) 缓冲区在 AXI_SRAM_NONCACHEABLE 段, CPU 写它【不经过 D-Cache】,
     *    所以【不能】在这里调 l1c_dc_writeback():
     *      - 语义上没有必要(那段本来就 non-cacheable);
     *      - 更致命的是 l1c_dc_writeback() 内部有
     *            ASSERT_ADDR_SIZE: addr 和 size 都必须是 HPM_L1C_CACHELINE_SIZE 的整数倍
     *        而这里 len 是 512 或更零碎的字节数(比如一次 flush 只有几十字节),
     *        于是【每次刷屏都会触发 assert 并使系统停在那里】——
     *        实测日志里那条
     *          assertion "size % HPM_L1C_CACHELINE_SIZE == 0" failed
     *            file hpm_l1c_drv.c line 111 function l1c_dc_writeback
     *        就是它。
     *    将来若把缓冲改回可缓存段, 必须自己按 cacheline 向上对齐后再 writeback/
     *    invalidate, 不能直接把任意 len 丢给驱动。 */

    /* 4) 配好并启动 DMA 通道(此刻 SPI 还没发请求, 不会丢数据)。
     *    dst_fixed = true 同时意味着 dst_mode = HANDSHAKE: 由 SPI 的 TX 请求
     *    节拍推动, 每个请求搬 1 字节, FIFO 满就自动停。 */
    s_lcd_dma_status = DMA_CHANNEL_STATUS_ONGOING;
    dma_default_handshake_config(LCD_TX_DMA, &hs);
    hs.src = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (u32)buff);
    hs.dst = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (u32)&LCD_SPI->DATA);
    hs.size_in_byte = len;
    hs.data_width = DMA_TRANSFER_WIDTH_BYTE;
    hs.ch_index = (u8)LCD_TX_DMA_CHANNEL;
    hs.src_fixed = false;
    hs.dst_fixed = true;
    hs.en_infiniteloop = false;
    hs.interrupt_mask = DMA_INTERRUPT_MASK_HALF_TC;  /* 只屏蔽半传输中断; TC/ERR/ABORT 使能 */
    stat = dma_setup_handshake(LCD_TX_DMA, &hs, true);
    if (stat != status_success) {
        s_lcd_stats.dma_error_count++;
        return stat;
    }

    /* 5) 清掉可能残留的通知, 再打开 SPI 的 TX DMA 请求: 数据从这一刻开始流动 */
    (void)xSemaphoreTake(s_lcd_dma_sem, 0);
    spi_enable_tx_dma(LCD_SPI);

    stat = lcd_dma_wait();

    /* 6) 关请求并等最后一个字节移出, 保证调用者可以安全抬 CS */
    spi_disable_tx_dma(LCD_SPI);
    if (spi_wait_for_idle_status(LCD_SPI) != status_success) {
        s_lcd_stats.dma_error_count++;
        return status_timeout;
    }

    return stat;
}

u8 SPI_WriteByte(u8 data)
{
	u8 recivedata = 0U;
	spi_control_config_t control_config;
	hpm_stat_t stat;
	spi_master_get_default_control_config(&control_config);
	control_config.common_config.trans_mode = spi_trans_write_read_together;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.tx_dma_enable = false;
	control_config.common_config.rx_dma_enable = false;
	stat = spi_transfer(LCD_SPI, &control_config, NULL, NULL,
	                    &data, 1U, &recivedata, 1U);
	if (stat != status_success) {
		/* 【显示任务里绝不能 printf】原来这行在 921600bps 下要 ~0.2ms, 而且
		 * 是最低优先级任务同步写串口。只计数, 汇总交给 lcd_app.c。 */
		s_lcd_stats.byte_cmd_fail++;
		lcd_stat_fail(stat);
	}

	return recivedata;
}

/* 原来的纯 CPU 轮询写法(出错时的退路, 语义与原实现一致) */
static u8 lcd_spi_write_block_poll(u8 *data,u32 len)
{
	u32 transfer_size;

	while (len > 0U) {
		transfer_size = len;
		if (transfer_size > TRANSFER_COUNT_MAX) {
			transfer_size = TRANSFER_COUNT_MAX;
		}

		spi_control_config_t control_config;
		hpm_stat_t stat;
		spi_master_get_default_control_config(&control_config);
		control_config.common_config.trans_mode = spi_trans_write_only;
		control_config.common_config.data_phase_fmt = spi_single_io_mode;
		control_config.common_config.tx_dma_enable = false;
		control_config.common_config.rx_dma_enable = false;
		stat = spi_transfer(LCD_SPI, &control_config, NULL, NULL,
		                    data, transfer_size, NULL, 0U);
		if (stat != status_success) {
			lcd_stat_fail(stat);
			return 1U;
		}
		s_lcd_stats.chunk_count++;
		data += transfer_size;
		len -= transfer_size;
	}

	return 0U;
}

/* 块写入口: DMA 可用走 DMA, 否则/出错退回轮询。
 * 两条路径写出的字节序列完全相同(都是按顺序写 SPI1->DATA), 所以退回轮询
 * 不会引入额外的画面差异。 */
u8 SPI_WriteBlock(u8 *data,u32 len)
{
	if ((s_lcd_stats.dma_ready != 0U) && (s_lcd_stats.dma_disabled == 0U) && (len > 0U)) {
		if (lcd_dma_write_block(data, len) == status_success) {
			s_lcd_dma_fail_streak = 0U;
			return 0U;
		}
		/* 单次失败: 记一次, 连续失败到上限就永久退回轮询, 不让屏幕卡死 */
		s_lcd_stats.dma_fallback_count++;
		if (s_lcd_dma_fail_streak < 0xFFU) {
			s_lcd_dma_fail_streak++;
		}
		if (s_lcd_dma_fail_streak >= LCD_TX_DMA_FAIL_LIMIT) {
			s_lcd_stats.dma_disabled = 1U;
		}
	}

	return lcd_spi_write_block_poll(data, len);
}
	   
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u8 data)
{ 
   LCD_CS_CLR;     
	 LCD_RS_CLR;	  
   SPI_WriteByte(data);
   LCD_CS_SET;	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u8 data)
{
   LCD_CS_CLR;
	 LCD_RS_SET;
   SPI_WriteByte(data);
   LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(u16 Data)
{	
	Data=~Data;
	LCD_CS_CLR;
	LCD_RS_SET;  
	SPI_WriteByte(Data>>8);
	SPI_WriteByte(Data);
	LCD_CS_SET;
}

void Lcd_WriteData_Block(u8 *data,u32 len)
{	
	LCD_CS_CLR;
	LCD_RS_SET;  
	SPI_WriteBlock(data,len);
	LCD_CS_SET;
}

#define LCD_WRITE_RETRY_MAX            (2U)

/* 内部: 把一段【紧凑排列】的像素(bpp=16)写出去。调用者负责 CS/RS。
 * 不拉 CS —— 这样逐行写的情况(见下面的 LCD_WritePixelsEx)可以共用一次片选,
 * 面板那边看起来就是一整条连续的 GRAM 写入。 */
static void lcd_write_pixels_tight(const u8 *data, u32 pixel_count)
{
	u32 chunk_pixels;
	u32 i;
	u32 retry;

	while (pixel_count > 0U) {
		chunk_pixels = pixel_count;
		if (chunk_pixels > (LCD_PIXEL_TRANSFER_BUFFER_SIZE / 2U)) {
			chunk_pixels = LCD_PIXEL_TRANSFER_BUFFER_SIZE / 2U;
		}

		/* Match the master_lcd RGB565 wire format: byte-swapped and inverted.
		 * 【注意】这里用的是 s_lcd_tx_buffer(放在 AXI_SRAM_NONCACHEABLE 段),
		 * 不是普通的局部 static —— 它会被 DMA 直接读, 不能留在可缓存段。 */
		for (i = 0U; i < chunk_pixels; i++) {
			s_lcd_tx_buffer[i * 2U] = (u8)~data[i * 2U + 1U];
			s_lcd_tx_buffer[i * 2U + 1U] = (u8)~data[i * 2U];
		}

		/* 【失败为什么改成"重发"而不是 break】
		 * 原来失败就 break: 这条 band 剩下的像素一个都不发, 后半截保持上一帧的
		 * 内容 —— 屏幕上正好就是"不规则长条"。而 spi_transfer() 失败里最常见的
		 * status_spi_master_busy 是"上一笔还在移", 隔一小会儿重发基本就成。
		 * 所以这里重发 LCD_WRITE_RETRY_MAX 次, 仍失败才放弃并计数
		 * (band_abort_count); 无论成功失败都继续往后写 —— 因为 busy 时一个字节
		 * 都没发出, GRAM 写指针没动, 继续写下一块仍然是接着的。
		 * 【注意】这一步只是"不静默丢数据", 不是对根因下结论: 根因看
		 * LCD_GetTxStats() 的 busy/timeout 计数。 */
		retry = 0U;
		while (SPI_WriteBlock(s_lcd_tx_buffer, chunk_pixels * 2U) != 0U) {
			if (retry >= LCD_WRITE_RETRY_MAX) {
				s_lcd_stats.band_abort_count++;
				break;
			}
			retry++;
			s_lcd_stats.retry_count++;
		}

		data += chunk_pixels * 2U;
		pixel_count -= chunk_pixels;
	}
}

void LCD_WritePixels(const u8 *data, u32 pixel_count)
{
	LCD_CS_CLR;
	LCD_RS_SET;
	lcd_write_pixels_tight(data, pixel_count);
	LCD_CS_SET;
}

/* ============================================================================
 * 带行间距(stride)的像素写 —— 【"数字乱码"的真正解药】
 *
 * 【为什么必须有这个函数】LVGL 在 PARTIAL 模式下渲染【窄区域】时, 每一行会按
 * LV_DRAW_BUF_STRIDE_ALIGN(=64, 见 lv_conf.h) 向上对齐, 不是紧凑排列:
 *
 *     28px 宽的区域: 一行 28×2 = 56 字节 -> 对齐到 64 字节(每行多 8 字节填充)
 *     48px("RPM"):  96 -> 128
 *     95px(状态行): 190 -> 192
 *     320px(全屏带): 640 -> 640  ← 正好对齐, 所以【全屏刷新看不出问题】
 *
 * 证据: lv_draw_buf.c 的 width_to_stride() 最后一步是
 *   return LV_ROUND_UP(width_byte, LV_DRAW_BUF_STRIDE_ALIGN);
 * 而 lv_refr.c 的 refr_area() 在 PARTIAL 模式下用 LV_STRIDE_AUTO 重建缓冲区,
 * 最终 header->stride 就是这个值。
 *
 * 【不知道会怎样】原来这里按"紧凑"直接把这 1232 个像素甩给面板: 第 1 行对,
 * 第 2 行整体右移 4 个像素, 第 3 行右移 8 个……到第 29 行已经偏了 112 个像素
 * —— 字形被彻底撕歪成乱码。而且它是【静态】的: 写完之后没有任何东西会去修它,
 * 所以只要数值不变, 屏上就一直是一团乱码(这也是"静止时还是乱"的原因, 它跟
 * 面板扫描撕裂完全是两回事)。
 *
 * 面板的窗口是按【紧凑行】解释数据的, 所以这里逐行把 w 个像素挑出来发,
 * 每行之间在源数据里跳过 (stride_bytes - w*2) 个填充字节。
 * ========================================================================== */
void LCD_WritePixelsEx(const u8 *data, u32 w, u32 h, u32 stride_bytes)
{
	u32 row;

	if ((data == NULL) || (w == 0U) || (h == 0U) || (stride_bytes < (w * 2U))) {
		return;
	}

	LCD_CS_CLR;
	LCD_RS_SET;
	for (row = 0U; row < h; row++) {
		lcd_write_pixels_tight(data + ((size_t)row * stride_bytes), w);
	}
	LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(u16 Color)
{
  	unsigned int i,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	LCD_CS_CLR;
	LCD_RS_SET;
	for(i=0;i<lcddev.height;i++)
	{
    for(m=0;m<lcddev.width;m++)
    {	
			Lcd_WriteData_16Bit(Color);
		}
	}
	 LCD_CS_SET;
} 



/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	//SPI2_Init(); //硬件SPI2初始化
	LCD_SPIInit();										 
 	LCD_RESET(); //LCD 复位
//*************2.4inch ILI9341初始化**********//	
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xD9); //0xC1 
	LCD_WR_DATA(0X30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x12);   //SAP[2:0];BT[3:0] 0x01
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x08); 	 //30
	LCD_WR_DATA(0x26); 	 //30
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0XB7); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x08); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x1D); 
	LCD_WR_DATA(0x1A); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0D); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x49); 
	LCD_WR_DATA(0X66); 
	LCD_WR_DATA(0x3B); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x01); 
	LCD_WR_DATA(0x09); 
	LCD_WR_DATA(0x05); 
	LCD_WR_DATA(0x04); 		 
	LCD_WR_REG(0XE1);    //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x18); 
	LCD_WR_DATA(0x1D); 
	LCD_WR_DATA(0x02); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x04); 
	LCD_WR_DATA(0x36); 
	LCD_WR_DATA(0x13); 
	LCD_WR_DATA(0x4C); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x13); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x2E); 
	LCD_WR_DATA(0x2F); 
	LCD_WR_DATA(0x05); 
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	delay_ms(120);
	LCD_WR_REG(0x29); //display on

  LCD_direction(USE_HORIZONTAL);//设置LCD显示方向
	LCD_GPIO_SET(LCDGPIO_LED,1);//点亮背光	 
	LCD_Clear(WHITE);//清全屏白色
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//开始写入GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(u8 direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;	
		default:break;
	}		
}	 




