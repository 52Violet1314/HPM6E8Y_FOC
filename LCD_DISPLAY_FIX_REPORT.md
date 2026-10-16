# LCD 撕裂 / 触摸花屏: 调查报告与改动记录

分支: `feature/lcd-touch`(worktree: `C:\Users\14420\Desktop\HPMicro\HPM6E8Y-display`)
分叉点: `4fd2b2a`(main 上的"引入 FreeRTOS"提交)。**没有改动 main 工作区任何文件。**

---

## 0. 一句话结论

| 现象 | 结论状态 | 说明 |
|------|----------|------|
| 触摸花屏 | **已排除**"引脚/CS 冲突" | 代码 + 原理图双重证据, 并加了编译期断言防止以后退化 |
| 撕裂(不规则长条) | **待定, 已布好测量手段** | 已否证"轮询被抢占→少发字节"; 真正能丢数据的只有 `spi_transfer()` 的 `busy/timeout` 两个返回值, 上板看计数器即可定案 |
| 触摸与撕裂的关系 | **同一个通路** | 触摸只是让重绘面积/刷屏次数成倍上升, 把偶发错误放大成"花屏", 触摸本身不抢 SPI1 |

---

## 1. 触摸花屏: 引脚/CS 冲突 —— 已排除(不是猜, 有两条实证)

### 1.1 代码里的引脚定义: 两组完全不相交

| 用途 | 引脚 | 出处 |
|------|------|------|
| LCD SCLK | PC10 | `LCDHw.c`(现集中在 `LCDHw.h` 的 `LCD_PAD_SCLK`) |
| LCD CS | PC11 | 同上 |
| LCD MISO | PC12 | 同上 |
| LCD MOSI | PC13 | 同上 |
| LCD RS / RST / LED | PC04 / PC03 / PC08 | 同上 |
| 触摸 T_SCLK | PC20 | `LCDTouch.c: TC_GPIO_SPICLK` |
| 触摸 T_CS | PC21 | `TC_GPIO_SPICS` |
| 触摸 T_MISO | PC22 | `TC_GPIO_SPIMISO` |
| 触摸 T_MOSI | PC23 | `TC_GPIO_SPIMOSI` |
| 触摸 T_IRQ | PC09 | `TC_GPIO_IRQ` |

`LCD_PAD_*` 与 `TC_GPIO_*` 没有任何一个相等 → **不存在"触摸位操作打到 LCD 引脚"**。
另外 `TC_COMM_MODE_SPI = 0`, 触摸走的是**纯 GPIO 位操作**(`TC_GPIOInit()`), 连 SPI3 外设都没使能
(`TC_SPIInit()` 不会被调用), 所以也不可能通过外设寄存器影响 SPI1。

### 1.2 原理图: J10 是"两组独立走线", 不是"共总线 + 双片选"

来源 `HPM6EMotorV20.pdf` 的网络表(已提取到 `build/schematic_text.txt`, J10 那一段):

```
J10.3  = CS     -> PC11(B11/C6 网络)     J10.10 = T_SCLK -> PC20(A11)
J10.4  = RESET  -> PC03(A7)              J10.11 = T_CS   -> PC21(B11)   <- 注意编号是"引脚编号"不是同一条线
J10.5  = DC/RS  -> PC04(B7)              J10.12 = T_MOSI -> PC23(A13)
J10.6  = MOSI   -> PC13(C7)              J10.13 = T_MISO -> PC22(A12)
J10.7  = SCLK   -> PC10(D5)              J10.14 = T_IRQ  -> PC09(C5)
J10.8  = LED    -> PC08(B9)
J10.9  = MISO   -> PC12(D6)
```

即 ILI9341 与 XPT2046 在排针上就是**两套独立的 SCK/MOSI/MISO + 各自的 CS**。
(常见的"共用 SCK/MOSI、只靠片选区分"的模块接法**不是**本板的接法。)

### 1.3 新增: 编译期断言 + 负向测试

`LCDTouch.c` 顶部新增 `TC_PIN_CONFLICTS`(7×5=35 组比较), 任何重叠直接 `#error`。
用一份改写过的副本做了正负对照测试(`build/_wip/pin_guard_test.py`, 走 `compile_commands.json` 的原命令):

```
--- LCDTouch_bad(把触摸 CS 改成 PC11 = LCD CS): 退出码 1
    error: #error "LCD/Touch pin overlap: XPT2046 bit-banged SPI must NOT share SCK/MOSI/MISO/CS with the ILI9341 LCD!"
--- LCDTouch_ok (把触摸 CS 改成 PC25, 不冲突):   退出码 0
冲突用例 编译失败 = True (期望 True)
对照用例 编译通过 = True (期望 True)
```

### 1.4 那触摸花屏到底是什么

见下面第 2 节: 触摸与撕裂共用同一条像素通路, 触摸让"错误的机会"变多。

---

## 2. 撕裂(不规则长条): 因果链分析与待定项

### 2.1 已否证的说法: "CPU 轮询写被抢占 → 少发几个字节"

`hpm_spi_drv.c` 的 `spi_write_data()`:

```c
while (transferred < count) {
    status = ptr->STATUS;
    if (!(status & SPI_STATUS_TXFULL_MASK)) { ptr->DATA = 下一字节; transferred++; retry = 0; }
    else { if (retry > 5000) break; retry++; }
}
```

TX FIFO 满只让循环空转, **不会丢字节**; 被中断/任务抢占也只是让它变慢。
所以"FIFO 断流 → 像素错位"这条链**缺少一个能丢数据的环节**, 不能当结论。
(我一开始的初判就是这条, 已被否证, 记在这里以免后面又绕回来。)

### 2.2 真正能丢数据的环节: `spi_transfer()` 的两个失败返回值

`hpm_spi_drv.c: spi_transfer()` 里只有两处会让"数据/命令根本没发出去":

1. `status_spi_master_busy` —— 进函数第一件事:
   ```c
   if ((mode == spi_master_mode) && (spi_is_active(ptr) == true)) { return status_spi_master_busy; }
   ```
   **一个字节都不写就返回**。此时:
   - 若发生在 `SPI_WriteBlock` → `LCD_WritePixels()` 里原来的写法直接 `break`
     → **这条 band 剩下的像素一个都不发**(后半截保持上一帧内容 = 不规则长条);
   - 若发生在 `SPI_WriteByte`(`LCD_WR_REG`/`LCD_WR_DATA`) → 窗口命令 0x2A/0x2B/0x2C
     或参数被整条吞掉, 而**返回值原来被完全忽略**(只 printf 一行)
     → 后面按"新窗口"算出来的像素其实写进了旧窗口/错位的位置 = 花屏。
2. `status_timeout` —— 函数末尾 `spi_wait_for_idle_status()`:
   ```c
   do { status = ptr->STATUS; if (retry > 5000) break; retry++; } while (status & SPIACTIVE);
   ```
   重试上限**只有 5000 次空转**。超时返回时数据其实还在往外移, 但调用者把它当"失败"
   处理就会提前收尾(抬 CS)、接着发下一条命令 → 于是下一条命令极可能撞上
   `busy`(第 1 条), 形成连锁。

**量级是否够得着(必须实测, 不能拍脑袋)**: 一次 512B 传输的 SPI 时间是
`512*8 / SCLK`(SCLK=25MHz → 164us; 33.3MHz → 123us), 而 5000 次空转在几百 MHz
的核上是几十~几百 us —— **两者同量级**, 所以这条链成立与否完全取决于实测数字。
实际 SCLK 在上电时已经打出来了: `[LCD] SPI1 src_clk=... -> sclk=...`(`LCDHw.c: LCD_SPIInit`)。

### 2.3 已经确证的次要问题(与根因无关, 但确实在拖后腿)

- `SPI_WriteByte()` 失败时 `printf("spi_transfer fail\n")` 跑在**最低优先级显示任务**里,
  921600bps 下一行 ~0.2ms 同步写串口 —— 已改成计数, 日志改由日志任务汇总。
- `LCD_WritePixels()` 失败即 `break`, 静默丢掉半条 band —— 已改成"重发同一块
  `LCD_WRITE_RETRY_MAX` 次, 仍失败才计数并继续往后写"(`band_abort_count`)。
  busy 时 GRAM 写指针没动, 所以继续写下一块仍然是接着的, 比直接放弃更接近正确画面。

---

## 3. 上板要做的事(实验 1 / 2 / 3)与判定分支

### 实验 1(最可能定案): 看计数器
显示任务每 5 秒走日志队列打一行(不是每帧打, 不拖慢刷屏):

```
LCD 5s: flush=.. pix=.. areamax=.. chunk=.. busy=.. tmo=.. fail=.. retry=.. abort=.. cmd=.. last=..
```

- `busy`/`tmo`/`abort` **出现非 0** → 传输侧确实在丢数据/丢命令, "长条"有了确定来源;
  此时把 `chunk` 的增长速度和 `pix`(每 5 秒写出的像素数)一起看, 还能算出 SPI 带宽占用。
- 三者**恒为 0** → 一个字节都没丢, 那就与传输无关, 转去查 LVGL 侧(见下)。

### 实验 2a: 任务级抢占(`lcd_app.c: LCD_APP_TEST_CRITICAL_FLUSH = 1`)
把整条 band 包在 `taskENTER_CRITICAL()/taskEXIT_CRITICAL()` 里跑一次。
**注意局限**: `configMAX_SYSCALL_INTERRUPT_PRIORITY = 4`, 优先级 1 的电流环 ADC 中断
不受临界区影响, 照样会插进来 —— 这个实验只能排除"任务切换", 不能排除"20kHz 电流环中断"。
**测完必须改回 0**(1.3ms 的临界区不能留在正式代码里)。

### 实验 2b: 中断级抢占(手工, 只在电机静止时做)
临时 `intc_m_disable_irq(BOARD_BLDC_ADC_IRQn)`(或把它的 PLIC 优先级降到 7)再刷屏。
会停掉电流环, 所以**只能在电机不转时看画面**。消失 → 与中断抢占有关。

### 实验 3: 确认 area 与窗口一致
`lcd_app.c: LCD_APP_TRACE_FLUSH` 默认开, 前 4 次 flush 会打:
```
LCD flush#1 area=(x1,y1)-(x2,y2) 320x20 pix=6400
```
和 `LCD_SetWindows(area->x1, area->y1, area->x2, area->y2)` 是同一组数, 一看就知道
LVGL 给的是不是我们以为的那条 band(顺便看 area 有没有超过 draw buffer 的 20 行)。

### 如果实验 1 的计数器恒为 0, 下一步查这些
彩条自检(`LCD_APP_RAWTEST = 1`, 见 `lcd_app.h`)已经把下面这些**与抢占无关**的因素
排除掉了: 面板本身、SPI 时钟/时序、字节序与取反约定、`LCD_SetWindows` 的坐标、
以及"每个字节都翻一次 CS"(`LCD_WR_REG`/`LCD_WR_DATA` 每字节一次 CS 脉冲)——
因为彩条走的是**同一对函数**、且在调度器启动前跑, 画面是完美的。剩下的只可能是
"只有 LVGL 场景才会出现"的东西:
1. LVGL flush 的 area 与本函数实际写的区域/顺序(`flush_ready` 的时机);
2. partial 模式下 area 是否超出 320×20 的 draw buffer(LVGL 会自己拆, 但要确认);
3. 多次小 area 连续 flush 时, 窗口命令流与像素流之间的时序(尤其 `busy` 计数为 0
   但画面仍然错位的话, 说明问题在"命令/参数被吞"之外);
4. 最后才怀疑 LVGL 版本/配置(9.4 + `LV_USE_OS LV_OS_NONE` + 单 buffer partial)。

---

## 4. 本次改动清单(提交 A, 全部为"测量 + 不丢数据 + 防退化", 未触碰传输方式)

| 文件 | 改了什么 | 为什么 |
|------|----------|--------|
| `src/lcd/LCDHw.h` | ① 集中定义 `LCD_PAD_*` 引脚宏(附原理图依据) ② 新增 `lcd_tx_stats_t` + `LCD_GetTxStats()` | ① 让触摸侧能做编译期冲突断言 ② 让"到底有没有丢数据/丢哪一类"可见 |
| `src/lcd/LCDHw.c` | ① 引脚改用 `LCD_PAD_*` 别名 ② `printf` 改成分类计数 ③ `LCD_WritePixels` 失败改为"重发同一块"而不是 `break` | ① 单一出处 ② 显示任务不许同步写串口 ③ 不再静默丢掉半条 band |
| `src/lcd/LCDTouch.c` | 顶部加 `TC_PIN_CONFLICTS` 编译期断言 + 完整的引脚核查证据注释 | 把"不是引脚冲突"这个结论固化下来, 以后改引脚会编译报错 |
| `src/lcd/lcd_app.c` | ① flush 统计(次数/像素/最大 area) ② 每 5 秒一行汇总走日志队列 ③ 前 4 次 flush 打 area ④ 实验用临界区开关(默认关) | 定案 + 诊断; 日志走队列, 不拖慢刷屏 |
| `LCD_DISPLAY_FIX_REPORT.md` | 本文件 | 因果链、证据、上板清单 |
| `tools/_setup_worktree_build.py` | worktree 构建环境准备脚本(复制 main 的 `build/` 并改写绝对路径) | `build/` 在 .gitignore 里, worktree 建出来没有它, 而 cmake/ninja 在本环境起不来 |

## 5. 构建证据(本机可复现)

```
$ cd C:\Users\14420\Desktop\HPMicro\HPM6E8Y-display
$ python tools/_setup_worktree_build.py          # 一次性: 复制+改写 build/
$ D:\sdk_env\tools\python3\python3.exe tools\_cc.py
要编译 25 个工程源文件
全部通过
$ cd build && cmd /c _relink.bat && cmd /c _postrun.bat
Memory region         Used Size  Region Size  %age Used
           FLASH:      793488 B        16 MB      4.73%
        AXI_SRAM:      325952 B       512 KB     62.17%
AXI_SRAM_NONCACHEABLE:   16392 B       256 KB      6.25%
-> output\demo.elf / output\demo.bin 已生成
```

> 工具小坑: `_cc.py` 用 `subprocess(..., text=True)` 捕获编译器输出, 中文警告在
> Windows 默认 GBK 下会抛 `UnicodeDecodeError`(在读取线程里, 不影响判定结果,
> "全部通过"/"有失败" 仍然可靠)。要干净输出可先 `chcp 65001` 或给 python 设
> `PYTHONUTF8=1`。

## 6. 需要上板确认的点

1. 烧 `demo.bin` 后, 控制台每 5 秒应出现 `LCD 5s: ...` 一行 —— 请把**前 3~5 行**贴回来。
   重点看 `busy` / `tmo` / `abort` 是否非 0。
2. 顺便贴一下开机那行 `[LCD] SPI1 src_clk=... -> sclk=...`(用来算 512B 传输时间 vs 5000 次重试预算)。
3. 撕裂如果还在, 请**拍一张照片**(手机拍屏也行), 并说明是"静止界面就有"还是"拖动滑条时才有"。
4. 触摸花屏如果还在, 也拍一张, 并说明是"按下瞬间"还是"拖动过程中"。
5. 如果方便, 再做一次实验 2a(`LCD_APP_TEST_CRITICAL_FLUSH=1` 烧一版)看照片是否变化,
   这一条能直接判定"任务级抢占"是否相关。

## 7. SPI1 TX DMA(备选方案, 不在本提交里)

代码已写好并**独立编译+链接通过**, 放在另一个分支 `feature/lcd-touch-spi-dma`
(从本提交分出来), 等实验 1/2 的结论再决定是否启用:

- 做法: DMAMUX 把 `HPM_DMA_SRC_SPI1_TX` 接到 HDMA 通道 0, 内存(自增/字节)→`SPI1->DATA`
  (固定/字节)、由 SPI 请求节拍推动; 显示任务在传输期间阻塞在信号量上让出 CPU;
  源缓冲放 noncacheable 段(`ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT`);
  失败自动退回原来的 CPU 轮询写。
- 它解决的是"FIFO 由 CPU 喂"这件事本身(与 busy/timeout 是否发生无关):
  传输期间 CPU 被抽走多久都不影响数据流, 而且显示任务不再占着 CPU 轮询。
- **只在实验证明"传输侧确实在丢数据"时才值得上。** 若实验 1 的计数恒为 0,
  说明问题在 LVGL 侧, 那这个分支就当作备选留着。
