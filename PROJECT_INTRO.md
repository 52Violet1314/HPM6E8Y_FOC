# 工程简介 —— YeeZi HPM6EMotor V2.0

> 目标芯片 HPM6E8Y（HPM6E80，BGA289） · 板卡 `hpm6e00YZ` · SDK `hpm_sdk_localized_for_hpm6e00YZ`（v1.12.1 本地化）
> 分支 `feature/dual-core-lvgl-core1` · 构建 `cmake -S . -B build -G Ninja && cmake --build build`

---

## 一、简介（正文）

**YeeZi HPM6EMotor V2.0 是一套基于国产先楫半导体 HPM6E8Y（双 RISC-V 核 @600 MHz）的永磁无刷电机磁场定向控制（FOC）固件工程，面向 4015 中空杯型电机的伺服驱动应用。** 工程以本地化裁剪的 HPMicro SDK v1.12.1 为底座，采用 core0 / core1 双核异构分工：**core0 承担硬实时控制**——在 20 kHz 的 ADC 完成中断里完成"相电流采样 → Clarke/Park → PI → 反 Park/SVPWM"的完整电流环，在 1 ms 硬周期控制任务里完成位置环、速度环、过流/欠压/温度保护与状态上报；**core1 独立运行一套 FreeRTOS + LVGL 9**，驱动 ILI9341 320×240 液晶与 XPT2046 触摸，构成与实时控制物理隔离的人机界面。两核通过 16 KB 非缓存共享 RAM 以无锁单生产者-单消费者协议交换遥测、命令、日志与心跳，core1 镜像就地执行（XIP）于 flash `0x80100000`，随 core0 的**单个 elf 一起烧写**。

硬件层面，功率级由三相 20 kHz 中心对齐互补 PWM（带可配死区）驱动，采用 10 mΩ 分流电阻 + 10 倍差分运放做两电阻法相电流采样，母线电压经 200 K/10 K 分压监测、MCP9700 采集功率级温度；转子位置来自 MT6701 14 bit 磁编码器（SPI2/J11 扩展排针，或板载 SEI 通道，两套后端可切换）。软件分层为 `main → app → FOC → BSP → Hardware`，中断留在 BSP 层经注册回调上抛，外设与算法解耦；典型的工程化细节包括：FreeRTOS 控制/日志双任务划分、`USE_SYSCALL_INTERRUPT_PRIORITY` 让 RTOS 临界区只屏蔽低优先级中断而绝不遮挡电流环、上电自检链（ADC 存活探测 → 零点标定 → 四角度电流极性/相序辨识 → 闭环方向验证）、以及 3200 组离线标定的"编码器原始角 → 电角度"查表换向。芯片内置的 EtherCAT 从站控制器（ESC）与 2 路 100 M PHY 已完成引脚与时钟级的可行性验证，为后续接入总线型伺服（SSC/SOES + CiA402）预留了硬件通道。

---

## 二、核心指标

| 项目 | 指标 |
| --- | --- |
| 主控 | HPM6E8Y / HPM6E80，双 RISC-V 核，每核 600 MHz，BGA289 |
| 电机 | 4015 中空杯型无刷电机，11 极对；相电阻 2.7 Ω、相电感 1.3 mH |
| 功率级 | 三相全桥，20 kHz 中心对齐互补 PWM，死区 100 tick |
| 采样 | 10 mΩ 分流 + ×10 差分运放（1 LSB ≈ 0.504 mA），两电阻法，PWM/TRGM 触发 + DMA |
| 控制环 | 电流环 20 kHz（跑在 ADC 中断内）；速度环 1 kHz；位置环 1 kHz 外环 |
| 环路带宽 | 电流环 60 Hz（整定历程 300 → 100 → 60 Hz，Kp/Ki 按 Ls·ωc / Rs·ωc 同比例缩放） |
| 位置精度 | 位置死区 1.5 mrad（≈0.086°，约 4 个编码器 count） |
| 保护 | 过流 2 A（3 拍去抖锁死）、母线欠压、转速软限幅、ADC 帧存活看门狗 |
| 人机界面 | ILI9341 320×240（SPI1 + HDMA）、XPT2046 触摸、LVGL 9，5 ms/拍 |
| 通信 | 控制台 UART14 @921600（core1 独立串口；两个核的日志都从这里出）、USB CDC-ACM（CherryUSB）、双核共享内存 |
| 存储占用 | core0 应用 flash ≈140 KB；core1 镜像 652 KB；整包 `demo.bin` 1.70 MB |
| 调试输出 | 出厂固件**全部关闭**（编译期开关，见 `src/app/app.h` 的"调试输出总开关"），`-DAPP_DEBUG_ALL=1` 一键恢复 |

---

## 三、双核资源划分

```
┌────────────────────── core0（电机控制核，600MHz） ──────────────────────┐
│  ADC 中断 20kHz : 电流环（PLIC 优先级 7，RTOS 临界区屏蔽不到）           │
│  控制任务  1ms  : 采集 / 保护 / 换向 / 位置环 / 速度环 → core1_sync()    │
│  日志任务       : 串口 921600 + USB CDC，并转发 core1 的日志             │
│  外设: PWM0 / ADC2·ADC3 / SEI / UART0 / USB / 按键 / 编码器 SPI2         │
└──────────────▲────────────────────────────────────┬────────────────────┘
               │  SHARE_RAM 0x012FC000（16KB，非缓存）│
    遥测快照 ↓ │                                     ↑ 命令 + 日志 + 心跳
┌──────────────┴──────────────── core1（显示核，600MHz） ─────────────────┐
│  独立 FreeRTOS V10.5.1（非 SMP），单一显示任务 5ms/拍                    │
│  LVGL 9 + ILI9341（SPI1 + HDMA）+ XPT2046 触摸（GPIO 软件 SPI）          │
│  代码：flash 0x80100000 起就地执行（XIP）；数据：AXI_SRAM；栈堆：DLM     │
└─────────────────────────────────────────────────────────────────────────┘
```

| 资源 | 归属 | 说明 |
| --- | --- | --- |
| 电流环 ADC 中断、PWM、SEI/QEI、编码器 | core0 | 实时性命脉 |
| 控制台 UART0、USB CDC、J11 按键 | core0 | 唯一写者，日志统一出口 |
| SPI1 + HDMA + LCD GPIO、触摸 | core1 | 物理隔离刷屏总线与 cache 污染 |
| PLIC / MCHTMR / PMP | 每核一份 | 中断须由处理它的核自己使能 |
| SHARE_RAM 16 KB | 双方 | 唯一跨核通路（seqlock + 环形缓冲） |
| flash 0x80000000~0x80100000 / 0x80100000~0x80200000 | core0 / core1 | 各 1 MB，单 elf 烧写 |

---

## 四、软件模块清单

| 层 | 路径 | 职责 |
| --- | --- | --- |
| 入口 | [src/main.c](src/main.c) | `board_init` → `app_init` → `core1_start` → 启动 RTOS |
| 应用编排 | [src/app/](src/app/) | 诊断、分级日志、编码器封装、位置环、电流环录波、NV 存档、离线段表 |
| 控制算法 | [src/FOC/](src/FOC/) | Clarke/Park、SVPWM、电流环 PI、速度/位置 PID、开环驱动、控制器状态机 |
| 板级驱动 | [src/bsp/](src/bsp/) | PWM / ADC / SPI / MT6701 / SEI / USB-CDC / 按键 |
| 协议层 | [src/Hardware/mt6701/](src/Hardware/mt6701/) | MT6701 帧解析与 CRC（纯协议，无硬件依赖） |
| 双核 | [src/dualcore/](src/dualcore/) | 镜像头校验、CPU1 释放、共享内存遥测/命令/日志协议 |
| 显示核 | [core1/](core1/) | 独立 main、FreeRTOS、共享内存客户端、日志环形缓冲、串口 14 转发 |
| 显示层 | [src/lcd/](src/lcd/) | LCDHw / LCDTouch / LCDGui / lcd_ui（core1 编译） |

关键设计文档：[DUAL_CORE.md](DUAL_CORE.md)（双核架构与排障表）、[ETHERCAT.md](ETHERCAT.md)（内置 ESC 使用方法）、[LCD_DISPLAY_FIX_REPORT.md](LCD_DISPLAY_FIX_REPORT.md) 与 [LCD_RPM_TEARING_FIX.md](LCD_RPM_TEARING_FIX.md)（显示通路排查）、[J11_pinout.md](J11_pinout.md)（扩展排针引脚表）。

---

## 五、可扩展方向

1. **EtherCAT 总线伺服**：片内 ESC（`0xF1700000`）+ 2 路 100 M PHY 已就绪，SYNC0 可直接驱动 20 kHz 电流环实现多轴同拍；待补 SII 镜像、ESI XML 与从站协议栈（官方 SSC / 开源 SOES），应用层按 CiA402 对象字典实现。
2. **闭环带宽提升**：当前受 20 kHz 采样抖动与任务抢占限制，补上反电动势前馈/解耦后可将电流环带宽从 60 Hz 提回数百 Hz。
3. **离线段表固化**：3200 组电角度查表现由开机静态扫描生成（约 3.2 分钟），可持久化到 flash 免除每次标定。
4. **多轴/多电机**：分层架构与 BSP 回调机制便于复制到第二套功率级；core1 尚有余量承载更复杂的界面与数据记录。
