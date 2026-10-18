# 双核架构: core0 = 电机控制, core1 = LVGL 显示

> 分支: `feature/dual-core-lvgl-core1`
> 适用: HPM6E8Y (HPM6E80, 双 RISC-V 核) + 本地化 SDK `hpm_sdk_localized_for_hpm6e00YZ` (v1.12.1)
> 相关代码: `src/dualcore/`(core0 侧 + 共用协议)、`core1/`(显示核工程)、`src/rtos/`、`src/lcd/`

---

## 1. 为什么要拆

拆之前, 显示(LVGL + SPI 刷屏 + 触摸)和控制(FOC)挤在同一个核的两个任务里:

* SPI 刷一条 band 就要 1.3ms 量级, 触摸扫描、LVGL 重绘也都在这个核上;
* 电流环在 ADC 中断(20kHz)里, 虽然优先级最高不会被任务级抢占, 但它要和
  刷屏抢 **数据缓存/总线/flash 取指**, 实测控制周期在 0.96~1.2ms 之间漂;
* 之前那份"长条/花屏"排查报告里, 20kHz 电流环中断始终是一个无法排除的嫌疑
  (它优先级高于 RTOS 临界区, 任何 `taskENTER_CRITICAL()` 都挡不住它)。

拆开之后, 刷屏跑的指令、占的总线、产生的缓存污染全都在 **另一个核** 上, 控制核
的 1ms 节拍和 20kHz 电流环从此只受自己的代码影响 —— 这是物理隔离, 不是调度优先级
能比的。

```
┌──────────────────────────── core0 (电机控制) ────────────────────────────┐
│  ADC 中断 20kHz: 电流环 (PLIC 1, 不受 RTOS 临界区影响)                    │
│  控制任务 1ms:   采集/保护/换向/遥测  →  core1_sync()                    │
│  日志任务:       串口(921600)+USB CDC, 并转发 core1 的日志               │
│  外设: PWM/ADC/SEI(编码器)/UART0/USB/按键                                │
└───────────────▲──────────────────────────────────────────┬──────────────┘
                │ SHARE_RAM (0x012FC000, 16KB, 非缓存)      │
     遥测快照 ↓ │                                          ↑ 命令 + 日志 + 心跳
┌───────────────┴──────────────── core1 (显示) ─────────────▼──────────────┐
│  独立 FreeRTOS (V10.5.1, 非 SMP): 只有 1 个显示任务, 5ms/拍              │
│  LVGL 9 + ILI9341(SPI1 + HDMA) + XPT2046 触摸(GPIO 软件 SPI)             │
│  代码: flash 0x80100000 起就地执行(XIP); 数据: AXI_SRAM; 栈/堆: DLM      │
└─────────────────────────────────────────────────────────────────────────┘
```

## 2. 资源归属(改代码前先看这张表)

| 资源 | 归属 | 说明 |
| --- | --- | --- |
| 电流环 ADC 中断、PWM、SEI/QEI、编码器 | core0 | 实时性命脉, 不许别的核碰 |
| 控制台 UART0、USB CDC、J11 按键 | core0 | 唯一写者, 日志统一从这里出 |
| SPI1 + HDMA(ch0) + LCD 的 GPIO | core1 | `src/lcd/LCDHw.c`, 时钟组 1 |
| 触摸(GPIO 软件 SPI, SPI3 备用) | core1 | `src/lcd/LCDTouch.c`, 时钟组 1 |
| PLIC | 每核一份 | 一个中断必须由**要处理它的那个核**使能; 没有"给另一个核使能"的 API |
| MCHTMR | 每核一份 | 两个核的 FreeRTOS tick 各用各的(`port.c` 会用 `mhartid` 选 mtimecmp) |
| PMP/PMA | 每核一份 | 所以两个核都要 `board_init_pmp()`, SHARE_RAM 才是非缓存 |
| SHARE_RAM 0x012FC000 (16KB) | 双方 | 唯一的跨核数据通路, 协议见 `src/dualcore/shm_core.h` |
| AXI_SRAM 0x01200000~0x012C0000 | core0 | 数据/bss/堆 |
| AXI_SRAM 0x012C0000~0x01300000 | core1 | 非缓存 64KB + 数据/bss 176KB |
| DLM 0x00200000 / 0x00240000 | core0 / core1 | 各自的栈堆 |
| flash 0x80000000~0x80100000 | core0 | 上限 1MB(现在只用了 ~140KB) |
| flash 0x80100000~0x80200000 | core1 | 1MB, 镜像头 + 代码 + 只读数据 |

## 3. core1 是怎么起来的(硬件流程)

HPM6E80 上电**永远从 core0 开始**, core1 自己不会启动(SDK 文档原话: "Core1 cannot
self start")。所以:

1. **链接期**: core1 的镜像被 core0 的构建放进自己的 elf, 用
   `-Wl,--section-start=.core1_img=0x80100000` 钉在 flash 的 0x80100000
   (见根 `CMakeLists.txt`)。所以**烧 core0 一个 elf 就等于烧了两个核**。
2. **上电**: core0 的 `board_init()` / `app_init()` 先把时钟树、电源、引脚复用配好
   (core1 依赖它们, 自己绝不再配一遍 PLL)。
3. `core1_start()` (`src/dualcore/core1_boot.c`):
   * 读 0x80100000 处的**镜像头** (`core1_boot_info_t`) 校验 magic/version/entry ——
     没烧 core1、烧了旧版本、链接选项被改过, 都在这里被挡住并报错, 而不是把 core1
     扔进空白 flash 里跑飞;
   * `sysctl_set_cpu_entry(HPM_SYSCTL, HPM_CORE1, 0x80100010)`;
   * `sysctl_release_cpu()` —— 清掉 `SYSCTL->CPU[1].LP.HALT`(寄存器在 0xF4002C00),
     core1 从 0x80100010 开始取指。
4. **core1 的 main** (`core1/src/main.c`) 只补"每个核独立"的两件事:
   `clock_update_core_clock()`(mhartid=1 → `hpm_core_clock` 取 CPU1 频率)和
   `board_init_pmp()`(SHARE_RAM 在本核也是非缓存); 然后握手 → 面板 → LVGL →
   自己的 FreeRTOS 调度器。
5. core0 侧 `core1_health_check()` 每秒看一次 core1 的心跳和"启动阶段码",
   把 core1 的进展/异常打到串口上 —— **core1 没有控制台, 共享内存就是它的串口**。

### 为什么代码不放在 core1 的 ILM(和 SDK 例程不一样)

SDK 的 `samples/multicore/*` 是把 core1 镜像**拷进 ILM(256KB)** 再跑。本工程显示核
带 LVGL 9, 实测链接出来 `.text + .rodata ≈ 700KB`, 256KB 的 ILM 装不下; core1 能用的
RAM(DLM 256KB + AXI_SRAM 240KB)同样装不下。所以改成**留在 flash 里就地执行(XIP)**:

* 好处: LVGL 全功能可用、不用裁控件/字体; core0 启动时不需要搬 700KB, 也没有跨核
  cache 一致性问题;
* 代价: core1 取指走 XPI(有缓存), 比 ILM 慢一些 —— 对 320x240 的界面完全够用;
* core1 的数据/栈仍在自己的 AXI_SRAM/DLM 里, 和 core0 完全不重叠。

## 4. 共享内存协议(`src/dualcore/shm_core.h`)

全部是**单生产者-单消费者、无锁**结构(谁写哪一段是固定的, 所以不需要锁):

| 方向 | 内容 | 机制 |
| --- | --- | --- |
| core0 → core1 | 实测转速 / 目标 / 斜坡后目标 / 方向 / 运行状态 | 先写数据, 最后 `telemetry_seq++`(release); core1 用 seqlock 取"整帧一致"的快照 |
| core1 → core0 | 界面滑条设定的目标转速 | 先写值, 再 `cmd_seq++`(release); core0 在 1kHz 控制任务里读走并调 `app_speed_target_set()` |
| core0 → core1 | "目标已下发到控制环"的序号 | 计数器(不是一次性脉冲), 5ms 才看一次的显示核也不会漏 |
| core1 → core0 | 日志行 | 32 槽定长环形缓冲, 槽位状态字握手, 满了丢弃并计数(绝不阻塞刷屏) |
| core1 → core0 | 心跳 / 启动阶段 / 致命错误码 | 每秒被 core0 检查一次 |

**非缓存**: `board_init_pmp()` 把 SHARE_RAM 配成 `MEM_TYPE_MEM_NON_CACHE_BUF`, 两个核
都调过, 所以跨核读写不需要任何 cache 维护指令。

**界面代码一行都没改**: 显示层原来调用的 `app_speed_get_rpm()` 等函数, 在 core1 上由
`core1/src/app_telem_client.c` 用**同名同签名**实现(内部读共享内存快照), 写方向的
`app_speed_target_set()` 变成"发命令给 core0", 仍然保持"控制环只有 core0 写"的规矩。

## 5. 构建

```powershell
# 一次构建两个核: core1 由 core0 的构建自动驱动(见根 CMakeLists.txt 的双核段落)
cmake -S . -B build -G Ninja
cmake --build build
```

产物:

| 文件 | 说明 |
| --- | --- |
| `build/output/demo.elf` / `.bin` | **烧写用**(里面含 core1 的全部代码) |
| `build/core1/output/core1.elf` | 显示核单独调试用(gdb/OpenOCD 可加载它的符号) |
| `build/core1/output/core1.bin` | core1 镜像本体(被转成 C 数组编进 core0) |
| `build/generated/sec_core_img.c` | 自动生成, 不要手改 |

core1 也可以单独构建(排错时很方便):

```powershell
cmake -S core1 -B build/core1 -G Ninja -DBOARD=hpm6e00YZ
cmake --build build/core1
```

改动 `core1/` 或 `src/lcd/` 后重新 `cmake --build build`, 会自动重建 core1 镜像并
重新生成 `sec_core_img.c`。

## 6. 烧写与验证

烧写流程和以前**完全一样**(仍然只烧 `demo.elf`):

```powershell
# VS Code: 任务 "flash (OpenOCD + FT2232)"
D:/sdk_env/tools/openocd/openocd.exe -f openocd.cfg `
    -c "program build/output/demo.elf verify reset exit"
```

复位后串口(921600)上应该按顺序看到:

```
[INFO][DUAL] core1 镜像 711304 字节 @ flash 0x80100000, 入口 0x80100010, 已释放 core1
[INFO][CORE1] 显示核启动: CPU1 600MHz, 代码 @flash 0x80100000 (入口 0x80100010), 共享区 0x012FC000 (6384 字节)
[INFO][DUAL] core1 阶段: 2 (共享内存握手完成)
[INFO][LCD] SPI1 src_clk=... -> sclk=... (target ...)
[INFO][DUAL] core1 阶段: 3 (面板/触摸初始化完成)   ← 屏这时应该亮起来
[INFO][DUAL] core1 阶段: 4 (LVGL 界面建立完成)
[INFO][DUAL] core1 阶段: 5 (FreeRTOS 调度器已启动)
[INFO][DUAL] core1 阶段: 6 (显示任务正在刷屏)
[INFO][LCD] 5s: flush=... pix=... ...              ← 每 5 秒一行的像素通路统计(从 core1 转过来)
```

界面行为与拆核前一致: 大字号显示实测转速, 速度条拖动能改目标转速(RPM),
"斜坡中 x/y RPM" 与"已下发"反馈仍然有效 —— 只是这些数据现在跨了一次核。

## 7. 排障对照表

| 现象 | 含义 / 处理 |
| --- | --- |
| `core1 镜像头无效(...)` 且屏不亮 | 0x80100000 处不是 core1 的镜像: 用**完整**的 `demo.elf` 重烧(别只烧 core0 的旧文件) |
| `core1 镜像被链接到 0x...`, 不是约定的 0x80100000 | 有人动了根 CMakeLists 的 `--section-start`; 恢复它并重新配置 |
| 阶段停在 1 (已进 main) | core1 卡在最早的一段: 时钟/`board_init_pmp()`; 重点看 XIP 是否可用(见第 8 节) |
| 阶段停在 2 (握手完成) | 卡在 `LCD_Init()` / `TP_Init()`: 面板接线、SPI1 时序、背光 |
| 阶段停在 3 | 卡在 `lv_init()` / 建界面(例如 draw buffer 的对齐断言) |
| 阶段停在 4/5 | 卡在 `vTaskStartScheduler()`: FreeRTOS 堆不够, 或者 tick 中断没来 |
| `core1 1s 无心跳` | 显示核死了或没起来: 看它最后报的阶段码; `core1_fault` 非 0 时有具体错误码 |
| 界面数字不动, 但 flush 统计在涨 | 共享内存没在更新: 检查 core0 那边 `core1_sync()` 是否被调(控制任务里) |
| 拖滑条电机不动 | 命令方向断了: 看 core0 是否收到 `cmd_seq`(可在 `core1_sync()` 里临时加日志) |

## 8. 已知风险 / 未在硬件上验证的点

1. **core1 从 flash 取指(XIP)**: 这是本设计唯一一个"没在实物上验证过"的假设。
   SDK 的例程把 core1 放 ILM, 并没有说不能 XIP; HPM6E00 的 flash 是共享外设,
   两个核都是总线主设备, 理论上没有障碍。验证方法很简单: 烧写后看串口是否出现
   "阶段: 6"。如果 core1 一开始就起不来(阶段停在 1 且无心跳), 就说明这条路不通,
   届时的退路是**裁剪 LVGL**(关掉不用的控件/字体、`LV_MEM_SIZE` 调小)把它压进
   256KB ILM, 回到 SDK 例程的 `ram_core1.ld` 流程 —— 但按当前链接结果的构成
   (LVGL 控件 + 字体 ≈ 630KB)看, 要砍到 256KB 以下会牺牲不少功能。
2. **FreeRTOS 的两份实例**: FreeRTOS 10.5.1 非 SMP, 两个核各跑一套调度器, 各自的
   内核数据结构在自己的内存里 —— 这是 SDK 的 erpc 双核 RTOS 例程同样的用法。
3. **`sysctl_release_cpu()` + `GPR[0]` 入口**: 与 SDK 的 `multicore_release_cpu()`
   完全一致(只是省掉了 ILM 拷贝那一步)。
4. 触摸/LCD 中断优先级: LCD 像素 DMA 完成中断用 6(>= `configMAX_SYSCALL_INTERRUPT_PRIORITY`
   = 4), 它在 core1 的 PLIC 里使能, 不会打扰 core0。
