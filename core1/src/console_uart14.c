/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核) 独立控制台 —— UART14 @ 921600, HDMA 发送
 *
 * ============================ 为什么需要它 ============================
 * 在这之前 core1 的日志全部写进共享内存的槽位环, 由 core0 的日志任务搬到控制台
 * (见 app_log_ring.c / console_ring.c)。那条路有一个致命的耦合: core0 的日志任务
 * 是低优先级任务, 一旦 core0 的控制环跑满 CPU(实测 979Hz, 每拍 1.021ms > 1ms 周期,
 * vTaskDelayUntil 就永远不再阻塞), 低优先级任务一次都轮不到 —— 于是两个核的日志
 * 会【一起】消失, 屏幕却照常刷。排错时这非常难受: 显示明明活着, 却看不到一行日志。
 *
 * 这里给 core1 一条【自己的】物理串口, 不经过 core0, 也不经过共享内存。
 *
 * ============================ 接线 ============================
 *   J11-40 (PF24, TXD) -> USB-TTL 模块的 RXD
 *   J11-37 (PF25, RXD) -> USB-TTL 模块的 TXD   (本文件只发不收, 可以不接)
 *   J11-19 或 20 (GND) -> USB-TTL 的 GND       (必须共地!)
 *   电脑端: 921600 8-N-1
 *
 * ============================ 发送通路 ============================
 *   任务: 行 -> 环形缓冲(4KB) -> [HDMA] -> UART14 THR -> 线
 *   DMA 用 HDMA 通道 1(LCD 像素 DMA 用通道 0, 不冲突)。
 *
 * 【为什么 DMA 的中断是"全屏蔽"的 —— 这一点很关键】
 *   IRQn_HDMA(=56) 是【所有 HDMA 通道共用】的一条中断线, 而 core1 上已经有一个
 *   lcd_hw_dma_isr 在处理 LCD 的通道 0; 它只清通道 0 的标志。
 *   如果本模块打开自己的 TC 中断, 通道 1 的标志没人清 -> 中断线一直挂着 ->
 *   那个 ISR 会被无限重入, 显示直接完蛋。
 *   所以这里 interrupt_mask = DMA_INTERRUPT_MASK_ALL: 本通道【不产生任何中断】,
 *   完成状态由 core1_console_uart_poll() 读走(读即清)。
 *   这个选择还有一个好处: 完全不用改 LCDHw.c 的中断处理, 刷屏那条路一个字节没动。
 *
 * 【poll 由谁调】core1 只有显示任务一个任务(5ms/拍), 由它在每拍调用
 *   core1_console_uart_poll() 续传。代价是行尾最多 5ms 延迟 —— 日志无所谓。
 *
 * 【失败怎么办】DMA 任何一步失败 -> 自动退回"TX FIFO 中断"那条路(和 core0 控制台
 *   同一个实现), 控制台不会因此哑掉。想强制走 FIFO 就把 CORE1_CONSOLE_USE_DMA 改 0。
 */
#include <string.h>

#include "board.h"              /* HPM_UART14 / IRQn_UART14 / pinmux.h / BOARD_RUNNING_CORE */
#include "hpm_uart_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_dmav2_drv.h"      /* HDMA: dma_setup_handshake / dma_check_transfer_status */
#include "hpm_dmamux_drv.h"

#include "shm_core.h"           /* 反向日志环(log0): core0 的日志经这里转发出去 */
#include "console_uart14.h"

#define CON_UART_TX_BASE        HPM_UART14
#define CON_UART_IRQ            IRQn_UART14

#define CON_UART_RING_SZ        (4096U)                 /* 必须是 2 的幂 */
#define CON_UART_RING_MASK      (CON_UART_RING_SZ - 1U)

#define CON_UART_IRQ_PRIO       (6U)                    /* 低于 LCD 像素 DMA(4) */

#if CORE1_CONSOLE_USE_DMA
#define CON_UART_DMA            HPM_HDMA
#define CON_UART_DMA_CHANNEL    (1U)                    /* 通道 0 被 LCD 像素 DMA 占了 */
#define CON_UART_DMA_MUX        HPM_DMAMUX
#define CON_UART_DMA_REQ        HPM_DMA_SRC_UART14_TX   /* = 0x31 */
#endif

/* 【必须放非缓存区】DMA 不过 D-Cache: 放普通 .bss 里, CPU 刚写的字节可能还在
 * cache 里没落到 SRAM, DMA 读到的就是旧数据(日志会错乱/重复)。
 * 与 LCDHw.c 的 s_lcd_tx_buffer 同样的处理。 */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(32)
static volatile uint8_t  s_tx_buf[CON_UART_RING_SZ];

static volatile uint32_t s_tx_head;         /* 生产: 任务写 */
static volatile uint32_t s_tx_tail;         /* 消费: DMA / 中断读 */
static volatile uint8_t  s_ready;
static volatile uint8_t  s_use_dma;
static volatile uint8_t  s_dma_busy;
static volatile uint32_t s_dma_len;
static volatile uint32_t s_drop_bytes;
static volatile uint32_t s_dma_err;

/* ------------------------------------------------------------------ */
/* 通路 1: TX FIFO 中断 (退路, 与 core0 的控制台同一实现)              */
/* ------------------------------------------------------------------ */

/* 中断: 尽量把环里的字节喂进 TX FIFO, 喂空了就关掉这个中断。
 * 【注意】只有 s_use_dma == 0 时才会注册/使能它。 */
SDK_DECLARE_EXT_ISR_M(CON_UART_IRQ, core1_console_uart14_isr)
void core1_console_uart14_isr(void)
{
    while (s_tx_tail != s_tx_head) {
        if (uart_send_byte(CON_UART_TX_BASE, s_tx_buf[s_tx_tail]) != status_success) {
            return;                             /* FIFO 满: 等下次中断, 不等待 */
        }
        s_tx_tail = (s_tx_tail + 1U) & CON_UART_RING_MASK;
    }
    uart_disable_irq(CON_UART_TX_BASE, uart_intr_tx_slot_avail);
}

/* ------------------------------------------------------------------ */
/* 通路 2: HDMA 发送                                                   */
/* ------------------------------------------------------------------ */

#if CORE1_CONSOLE_USE_DMA
/* 启动一次传输: 只发【从 tail 到缓冲区末尾】这一段连续内存。
 * 环回绕的那一段等这次完成后再发一次(在 poll 里), 这样 DMA 的源地址永远连续。 */
static void con_dma_kick(void)
{
    dma_handshake_config_t hs;
    uint32_t tail = s_tx_tail;
    uint32_t head = s_tx_head;
    uint32_t len;

    if ((s_use_dma == 0U) || (s_dma_busy != 0U) || (head == tail)) {
        return;
    }

    len = (head > tail) ? (head - tail) : (CON_UART_RING_SZ - tail);

    dma_default_handshake_config(CON_UART_DMA, &hs);
    hs.src = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&s_tx_buf[tail]);
    hs.dst = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&CON_UART_TX_BASE->THR);
    hs.size_in_byte = len;
    hs.data_width = DMA_TRANSFER_WIDTH_BYTE;
    hs.ch_index = (uint8_t)CON_UART_DMA_CHANNEL;
    hs.src_fixed = false;                       /* 源: 逐字节前进 */
    hs.dst_fixed = true;                        /* 目的: 固定为 UART 的 THR */
    hs.en_infiniteloop = false;
    /* 【全屏蔽】见文件头: 共用 IRQn_HDMA, 本通道绝不能产生中断 */
    hs.interrupt_mask = DMA_INTERRUPT_MASK_ALL;

    if (dma_setup_handshake(CON_UART_DMA, &hs, true) != status_success) {
        s_dma_err++;
        s_use_dma = 0U;                         /* 退路: 下面 poll 会切到 FIFO 中断 */
        return;
    }

    s_dma_len  = len;
    s_dma_busy = 1U;
}
#endif /* CORE1_CONSOLE_USE_DMA */

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

/* 按"要不要 DMA"配一次 UART。dma_enable 会改 FIFO 的工作方式, 两条路不能共用。 */
static int con_uart_apply_config(int use_dma)
{
    uart_config_t cfg;

    uart_default_config(CON_UART_TX_BASE, &cfg);    /* 8-N-1 / FIFO 打开 */
    cfg.baudrate = CORE1_CONSOLE_BAUD;
    /* 【必须自己填】uart_init() 用它算分频, 而 uart_default_config() 不填这个字段。
     * 漏了就拿栈上的随机值去算 -> 波特率乱, 收到的是乱码。 */
    cfg.src_freq_in_hz = clock_get_frequency(clock_uart14);
    cfg.fifo_enable = true;
    cfg.dma_enable = (use_dma != 0);

    return (uart_init(CON_UART_TX_BASE, &cfg) == status_success) ? 0 : -1;
}

void core1_console_uart_init(void)
{
    int use_dma = 0;

    /* 时钟挂到组 1(= CPU1/core1): 只要 core1 不睡, 这路 UART 的时钟就不停。
     * 必须先加组再读频率, 否则读到的是没开时钟的 0。 */
    clock_add_to_group(clock_uart14, 1);
    clock_add_to_group(clock_hdma, 1);          /* 幂等: LCDHw.c 已经加过 */
    init_uart14_pins();                         /* PF24 = TXD, PF25 = RXD */

#if CORE1_CONSOLE_USE_DMA
    /* DMAMUX: HDMA 通道 n 的请求源 = DMAMUX 通道 n */
    dmamux_config(CON_UART_DMA_MUX, CON_UART_DMA_CHANNEL, CON_UART_DMA_REQ, true);
    use_dma = 1;
#endif

    if (con_uart_apply_config(use_dma) != 0) {
        /* DMA 模式的 UART 配不起来: 退回不带 DMA 的配置 */
        use_dma = 0;
        if (con_uart_apply_config(0) != 0) {
            s_ready = 0U;                       /* 两条路都不行: 调用者退回共享环 */
            return;
        }
    }

    s_tx_head  = 0U;
    s_tx_tail  = 0U;
    s_dma_busy = 0U;
    s_dma_err  = 0U;
    s_use_dma  = (use_dma != 0) ? 1U : 0U;

    uart_disable_irq(CON_UART_TX_BASE, uart_intr_tx_slot_avail);
    intc_m_enable_irq_with_priority(CON_UART_IRQ, CON_UART_IRQ_PRIO);

    s_ready = 1U;
}

int core1_console_uart_ready(void)
{
    return (s_ready != 0U) ? 1 : 0;
}

uint32_t core1_console_uart_dropped(void)
{
    return s_drop_bytes;
}

uint32_t core1_console_uart_dma_errors(void)
{
    return s_dma_err;
}

/* ------------------------------------------------------------------ */
/* 发送                                                                */
/* ------------------------------------------------------------------ */

int core1_console_uart_write(const char *data, int size)
{
    uint32_t head;
    uint32_t used;
    uint32_t room;
    int      n = 0;

    if ((s_ready == 0U) || (data == NULL) || (size <= 0)) {
        return 0;
    }

    head = s_tx_head;
    used = (head - s_tx_tail) & CON_UART_RING_MASK;
    room = CON_UART_RING_SZ - used - 1U;

    while (n < size) {
        char     c    = data[n];
        uint32_t need = (c == '\n') ? 2U : 1U;  /* '\n' 自动补 '\r' */

        if (room < need) {
            s_drop_bytes += (uint32_t)(size - n);
            break;
        }
        if (c == '\n') {
            s_tx_buf[head] = (uint8_t)'\r';
            head = (head + 1U) & CON_UART_RING_MASK;
        }
        s_tx_buf[head] = (uint8_t)c;
        head = (head + 1U) & CON_UART_RING_MASK;
        room -= need;
        n++;
    }
    s_tx_head = head;

    if (n > 0) {
#if CORE1_CONSOLE_USE_DMA
        if (s_use_dma != 0U) {
            con_dma_kick();
            return n;
        }
#endif
        uart_enable_irq(CON_UART_TX_BASE, uart_intr_tx_slot_avail);
    }
    return n;
}

void core1_console_uart_write_line(const char *line)
{
    if (line == NULL) {
        return;
    }
    (void)core1_console_uart_write(line, (int)strlen(line));
    (void)core1_console_uart_write("\n", 1);
}

/* ---- 转发 core0 的日志: 共享内存反向环(log0) -> 本串口 ----
 * core0 的 USB CDC 会掉线、UART0 又未必接出来, 所以它的日志经这个环交给 core1 输出。
 * 【为什么不取完】每次最多 8 行: 显示任务还要刷屏, 不能让日志把它占住。
 * 【没就绪就不取】s_ready==0 时直接返回, 数据留在环里 —— 一旦取了却没地方写,
 * 就等于把 core0 的日志悄悄扔了。 */
void core1_console_forward_core0(void)
{
    char     line[SHM_LOG_LINE_LEN];
    uint32_t budget = 8U;
    shm_t   *s = shm_get();

    if (s_ready == 0U) {
        return;
    }
    while (budget > 0U) {
        budget--;
        if (shm_log0_pop(s, line, sizeof(line)) == 0) {
            break;
        }
        core1_console_uart_write_line(line);
    }
}

/* 由显示任务每拍(5ms)调用: 收掉已完成的 DMA, 并续传环里剩下的数据。
 * FIFO 模式下它是空操作(那边由中断自己推进)。 */
void core1_console_uart_poll(void)
{
#if CORE1_CONSOLE_USE_DMA
    if (s_ready == 0U) {
        return;
    }
    if (s_use_dma == 0U) {
        return;
    }

    if (s_dma_busy != 0U) {
        /* dma_check_transfer_status() 是"读即清", 必须在这里读走, 否则标志一直挂着 */
        uint32_t st = dma_check_transfer_status(CON_UART_DMA, CON_UART_DMA_CHANNEL);

        if (st == DMA_CHANNEL_STATUS_ONGOING) {
            return;                             /* 还在搬, 下一拍再看 */
        }
        s_dma_busy = 0U;

        if ((st & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) != 0U) {
            /* DMA 出错了: 永久退回 FIFO 中断那条路, 控制台不能因此哑掉 */
            s_dma_err++;
            s_use_dma = 0U;
            if (con_uart_apply_config(0) != 0) {
                s_ready = 0U;
                return;
            }
            uart_enable_irq(CON_UART_TX_BASE, uart_intr_tx_slot_avail);
            return;
        }
        s_tx_tail = (s_tx_tail + s_dma_len) & CON_UART_RING_MASK;
    }

    con_dma_kick();                             /* 还有数据就接着发(含环回绕那一段) */
#endif
}
