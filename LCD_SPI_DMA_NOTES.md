# 分支 `feature/lcd-touch-spi-dma`: SPI1 像素通路改用 TX DMA(候选方案)

> 这个分支**不并入** `feature/lcd-touch`, 它是"实验 1 证明传输侧确实在丢数据"时
> 才合的解药。因果分析与上板实验步骤见 `LCD_DISPLAY_FIX_REPORT.md`。

分支关系: `main(4fd2b2a)` → `feature/lcd-touch(1f8e558, 调查+计数+断言)` →
本分支(仅多一个提交: SPI1 TX DMA)。

## 改动内容

| 文件 | 改了什么 |
|------|----------|
| `src/lcd/LCDHw.c` | 新增 SPI1 TX DMA 通路: DMAMUX→HDMA 通道 0, 中断+信号量, 失败自动退回轮询 |
| `src/lcd/LCDHw.h` | `lcd_tx_stats_t` 追加 DMA 统计字段(`dma_chunk_count`/`dma_error_count`/`dma_fallback_count`/`last_dma_status`/`dma_ready`/`dma_disabled`) |
| `src/rtos/app_rtos.h` | 中断优先级表新增 `APP_IRQ_PRIO_LCD_DMA = 6`(必须 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`=4, 因为 ISR 里要 give 信号量) |

## 为什么这么做(和"撕裂/花屏"的关系)

原来的 bulk 写(512B/块)是**纯 CPU 轮询**喂 TX FIFO:

```c
while (transferred < count) { if (!(TXFULL)) ptr->DATA = 下一字节; }
```

TX FIFO 只有 8 字节深(`SPI_SOC_FIFO_DEPTH`), 33MHz 下只够 ~1.9us。改成 DMA 之后:

1. **数据流不再依赖 CPU**: FIFO 由 DMA 按 SPI 的 TX 请求节拍补, 中断/任务抢占
   抢不走 DMA 的服务能力 —— 传输过程中断开的风险从根上消失;
2. **显示任务不再空转占 CPU**: 传输期间它阻塞在信号量上(见 `lcd_dma_wait()`),
   控制任务的 1ms 硬周期不再被刷屏拖累(原来是"显示任务占着 CPU 空转 1.3~3.2ms,
   控制任务只能等它让出")。

## 关键实现点(踩过的坑)

1. **`SDK_DECLARE_EXT_ISR_M` 的 IRQ 参数不能带括号**: 宏内部是
   `irq_handler_wrapper_##irq_num`, 写成 `(IRQn_HDMA)` 会得到
   `pasting "irq_handler_wrapper_" and "(" does not give a valid preprocessing token`。
   必须写成光秃秃的 `IRQn_HDMA`(和 `bsp_adc.c` 的 `BSP_ADC_IRQn` 写法一致)。
2. **顺序必须照 `hpm_spi_drv.h` 的注释**: 先关 SPI 的 TX DMA 请求 → 配 SPI 本次传输
   (`spi_setup_dma_transfer(tx_dma_enable=false)`) → 配好并启动 DMA 通道
   (`dma_setup_handshake(..., start=true)`) → 再打开 TX DMA 请求(数据此时才开始流动)。
   反过来做会让 SPI 在 FIFO 空的情况下先开始跑, 开头几个字节不可控。
3. **DMA 源缓冲必须 noncacheable**: 用
   `ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(32)` 放进 `.noncacheable`
   (= AXI_SRAM_NONCACHEABLE, `board_init_pmp()` 已把 0x01280000~0x012BFFFF 配成
   non-cacheable), 启动前再 `l1c_dc_writeback()` 兜底。
4. **`dma_check_transfer_status()` 是 W1C(读即清标志)**: 状态必须在 ISR 里存下来
   (`s_lcd_dma_status`), 任务里再读一次只会读到 `ONGOING`。
5. **单字节命令不走 DMA**: `SPI_WriteByte` 仍是原来的轮询路径 —— 1 个字节不可能把
   8 深的 FIFO 写空, 走 DMA 只是白白多付一次配置开销。所以本分支只改了 bulk 通路。
6. **失败兜底**: DMA 初始化失败(信号量建不出来)或单块传输失败 → 退回 CPU 轮询写;
   连续 `LCD_TX_DMA_FAIL_LIMIT`(8)次失败 → 永久退回。一行开关
   `LCD_TX_DMA_ENABLE = 0` 也能整体退回。**不会因为 DMA 配错而黑屏。**

## 已完成的验证(本机)

```
$ D:\sdk_env\tools\python3\python3.exe tools\_cc.py
要编译 25 个工程源文件
全部通过                      (无新增告警)

$ cd build && cmd /c _relink.bat && cmd /c _postrun.bat
           FLASH:      796824 B        16 MB      4.75%
        AXI_SRAM:      325440 B       512 KB     62.07%
-> output\demo.elf / output\demo.bin

$ riscv32-unknown-elf-nm output/demo.elf | findstr s_lcd_tx_buffer
01280180 b s_lcd_tx_buffer          <- 落在 .noncacheable(0x01280000~0x012BFFFF)
$ riscv32-unknown-elf-objdump -h output/demo.elf
 14 .noncacheable 00004008  01280000  ...
```

## 上板必须确认的点(按重要性)

1. **开机自检彩条**: 把 `lcd_app.h` 的 `LCD_APP_RAWTEST` 改成 1 烧一版 —— 四条竖条
   必须是 红/绿/蓝/白。彩条走的就是新的 DMA 通路, 彩条对了说明 DMA 配置(通道/
   DMAMUX/字节序/长度)全对。这一步不做, 后面所有现象都没法解释。
2. 彩条不对(黑屏/花屏/颜色反) → 立刻把 `LCD_TX_DMA_ENABLE` 改 0 回到轮询,
   并把 `LCD 5s: ... dma_ready/dma_error_count/last_dma_status` 那几行贴回来。
3. 彩条正常后看日志里 `dma_chunk_count` 是否在涨、`dma_error_count` 是否为 0;
   同时确认控制任务的 1ms 节拍没有因为刷屏而漂(这是本分支的主要收益之一)。
4. 撕裂/长条是否消失 —— 这是判断"传输侧丢数据"假说的最终依据。
5. 若撕裂仍在(而实验 1 的 busy/tmo 计数也恒为 0) → 说明根因在 LVGL/窗口命令侧,
   本分支应放弃, 转去查 report 第 3 节列的那几项。
