# 内置 EtherCAT（ESC）使用方法

> 适用：YeeZi HPM6EMotor V2.0 / HPM6E8Y（HPM6E80，BGA289）
> 结论先行：**芯片里内置的是 EtherCAT 从站控制器（ESC）硬件 + 2 个 100M PHY；SDK 只给了寄存器级驱动，没有协议栈。**
> 想真正跑起 EtherCAT，还差一个从站协议栈（SSC / SOES）和一份 SII（EEPROM 镜像）。

---

## 1. 片内 ESC 是什么

| 项目 | 说明 |
|---|---|
| 基地址 | `0xF1700000`（`HPM_ESC_BASE`，见 `soc/HPM6E00/HPM6E80/hpm_soc_ip.h:611`） |
| CPU 侧接口（PDI） | `0x80 = On-chip bus`：ESC 寄存器 / 过程数据 RAM **直接内存映射**，CPU 用普通 load/store 访问，不需要 SPI/并口时序 |
| 端口 | ESC 内核有多端口描述（port0~port3），这颗只引出 **P0、P1** 两个（`board.h`: `BOARD_ECAT_SUPPORT_PORT1=1`、`BOARD_ECAT_SUPPORT_PORT2=0`） |
| PHY | **片内集成 2 个 100M PHY**，MDI 直接出到 RJ45（板上没有外置 PHY 芯片） |
| 中断 | `IRQn_ESC`=133（AL 事件，**常开**）、`IRQn_ESC_SYNC0`=134、`IRQn_ESC_SYNC1`=135、`IRQn_ESC_RESET`=136 |
| 时钟 | 外设时钟 `clock_esc0`；ESC 内核 100M 由 `GPR_CFG0.CLK100_EN` 使能；PHY 的 25M 参考时钟由 `PHY_CFG1.REFCK_25M_OE` 使能 |

### 寄存器空间（`soc/HPM6E00/ip/hpm_esc_regs.h`）

| 偏移 | 内容 |
|---|---|
| `0x000 - 0xFFF` | ESC 标准寄存器：Type/Revision、FMMU、SyncManager、DC、AL 状态机、EEPROM(SII) 控制、MII 管理、端口/链路、错误计数 |
| `0x1000 - 0xFFFF` | **过程数据 RAM（PDRAM）**，实际大小读 `RAM_SIZE` 寄存器（单位 KB） |
| `0x10000 - 0x1EFFF` | PDRAM 别名窗口（`PDRAM_ALS`） |
| `0x1F000 - 0x1F0A0` | 厂商私有：`GPR_CFG0/1/2`、`PHY_CFG0/1`、GPIO、`IO_CFG[9]`（CTR 信号）、`GPR_STATUS` |

上电第一件事建议先把能力寄存器打出来（正常应该看到 4/8 之类的 FMMU、SM 数量）：

```c
printf("ESC type=%02X rev=%02X FMMU=%d SM=%d RAM=%dKB PORT_DESC=%02X PDI=%02X\n",
       HPM_ESC->TYPE, HPM_ESC->REVISION,
       ESC_FMMU_NUM_NUM_GET(HPM_ESC->FMMU_NUM),
       ESC_SYNCM_NUM_NUM_GET(HPM_ESC->SYNCM_NUM),
       ESC_RAM_SIZE_SIZE_GET(HPM_ESC->RAM_SIZE),
       HPM_ESC->PORT_DESC,
       ESC_PDI_CTRL_PDI_GET(HPM_ESC->PDI_CTRL));
```

---

## 2. 这块板子上的 EtherCAT 是怎么接的

从 `HPM6EMotorV20.pdf` 的 `ethercat.SchDoc`（两张 HR911105A 带变压器 RJ45）+ `boards/hpm6e00YZ/pinmux.c` 的 `init_esc_pins()` 对出来：

| 功能 | 引脚 | 备注 |
|---|---|---|
| **Port0 MDI** | PA20/PA21/PA22/PA23 = PHY0 的 RD_N/RD_P/TD_N/TD_P | 直连 RJ45 变压器（两张 RJ45 谁是 P0 以丝印为准） |
| **Port1 MDI** | PA16/PA17/PA18/PA19 = PHY1 的 RD_N/RD_P/TD_N/TD_P | 同上，PHY1 |
| Port0 MII | PV00 ~ PV11（TXCK/TXEN/TXD0-3/RXCK/RXDV/RXER/RXD0-3） | 内部 PHY↔ESC |
| Port1 MII | PW00 ~ PW11 | 内部 PHY↔ESC |
| PHY 参考时钟 | PW20（P1）、PW21（P0）= `ESC0_REFCK` | |
| MDIO / MDC | PA30 / PA31 | **和 TSW（普通以太网）共用**，两套驱动别同时抢 |
| PHY0/PHY1 复位 | PV12 / PW12（GPIO，低有效 `resetn`） | `board_init_ethercat()` 里默认**停在复位态** |
| RBIAS | PA24/PA27，外接 2.49k 到地 | 模拟偏置，勿动 |
| LINK 反馈 | PA25 = `ESC0_CTR_0` ← PHY1 LED1；PA28 = `ESC0_CTR_1` ← PHY0 LED1 | 所以 `BOARD_ECAT_PORT0/1_LINK_INVERT = true`（LED 低有效） |
| ESC EEPROM | **板上没有** | 只能用 EEPROM 模拟；PA12/PA13 是 I2C1，将来要外挂 EEPROM 时用 |

> 注意：`board.h` 里还有 `BOARD_ECAT_LED_RUN_CTRL_INDEX(1)` / `LED_ERROR_CTRL_INDEX(6)`，但本板 pinmux 只引了 `CTR_0/CTR_1` 且都给了 LINK。
> 也就是说 **RUN/ERR 灯大概率没接**（RJ45 自带的那两个灯是 PHY 驱动的）。要指示状态就写 `RUN_LED_OVRD`/`ERR_LED_OVRD` 或用串口打 `AL_STAT`。

---

## 3. SDK 里已经有什么（`hpm_sdk_localized_for_hpm6e00YZ`）

| 文件 | 内容 |
|---|---|
| `drivers/inc/hpm_esc_drv.h` | 全部 ESC 驱动 API（400+ 行，大部分是 static inline 寄存器操作） |
| `drivers/src/hpm_esc_drv.c` | `esc_mdio_read/write`、`esc_check_eeprom_loading` |
| `soc/HPM6E00/ip/hpm_esc_regs.h` | 完整寄存器结构体 + 全部位域宏（4500 行） |
| `boards/hpm6e00YZ/board.c` | `board_init_ethercat()`、`board_init_switch_led()` |
| `boards/hpm6e00YZ/pinmux.c` | `init_esc_pins()`、`init_esc_eeprom_pin()`、`init_esc_eeprom_as_i2c_pin()`、`init_esc_in_out_pin()` |
| `boards/hpm6e00YZ/board.h` | `BOARD_ECAT_*` 全套板级宏（端口、PHY 地址、复位脚、CTR 索引） |
| `components/eeprom_hal` | 通用 EEPROM 抽象（AT24C128 等），外挂 EEPROM 时才用得上 |

**已经编进工程了**：构建里带 `-DHPMSOC_HAS_HPMSDK_ESC=y`（`drivers/CMakeLists.txt:76` 条件编译），`build/` 下能看到 `hpm_esc_drv.c.obj`。不需要改 CMake。

主要 API（`hpm_esc_drv.h`）：

```c
/* 时钟 / PHY */
esc_core_enable_clock(ptr, true);          /* ESC 内核 100M */
esc_phy_enable_clock(ptr, true);           /* PHY 25M refclk */
esc_config_eeprom_and_clock(ptr, &cfg);    /* EEPROM 模拟/大小 + 上面两个时钟，一把配完 */

/* EEPROM（SII）模拟 */
esc_get_eeprom_cmd(ptr);                   /* 读 ESC 要什么命令 */
esc_get_eeprom_word_address(ptr);          /* 字地址（×2 = 字节地址） */
esc_write_eeprom_data(ptr, data64);        /* 回 8 字节 */
esc_eeprom_emulation_ack(ptr, cmd, ack_err, crc_err);
esc_check_eeprom_loading(ptr);             /* 上电加载结果 */

/* PHY 管理 */
esc_mdio_read(ptr, phy_addr, reg, &val);   /* BMSR=1, BMCR=0 */
esc_mdio_write(ptr, phy_addr, reg, val);

/* 端口/信号 */
esc_config_ctrl_signal_function(ptr, ctr_idx, esc_ctrl_signal_func_alt_nmii_link0, true);
esc_config_nmii_link_source(ptr, true, true, false);   /* LINK 来自 IO 而不是寄存器 */

/* 中断 */
esc_enable_irq(ptr, esc_sync0_irq_mask | esc_sync1_irq_mask);
```

**没有的东西**（我翻过 hpm_sdk 1.12.1 的整棵 git 树，2 万个文件）：
从站协议栈、SSC 移植、ESI/SII 生成、任何 EtherCAT 例程。`samples/` 里没有 `esc`/`ethercat` 目录。

---

## 4. 上手步骤（裸机 bring-up 骨架）

### 4.1 初始化

```c
#include "hpm_esc_drv.h"
#include "board.h"

void ecat_hw_init(void)
{
    /* 1) 时钟 + 54 根引脚 + PHY 复位脚（此时 PHY 仍在复位） */
    board_init_ethercat(HPM_ESC);

    /* 2) EEPROM 模拟（板上没 EEPROM）+ 内核/PHY 时钟 */
    esc_eeprom_clock_config_t cfg = {
        .eeprom_emulation      = true,   /* 必须！否则 ESC 会去 I2C 找 EEPROM */
        .eeprom_size_over_16kbit = false,
        .core_clock_en         = true,
        .phy_refclk_en         = true,
    };
    esc_config_eeprom_and_clock(HPM_ESC, &cfg);

    /* 3) 释放 PHY 复位（低有效 → 置 1），等 PHY 起来 */
    gpio_set_pin_output_with_initial(BOARD_ECAT_PHY0_RESET_GPIO,
        BOARD_ECAT_PHY0_RESET_GPIO_PORT_INDEX, BOARD_ECAT_PHY0_RESET_PIN_INDEX,
        !BOARD_ECAT_PHY_RESET_LEVEL);
    gpio_set_pin_output_with_initial(BOARD_ECAT_PHY1_RESET_GPIO,
        BOARD_ECAT_PHY1_RESET_GPIO_PORT_INDEX, BOARD_ECAT_PHY1_RESET_PIN_INDEX,
        !BOARD_ECAT_PHY_RESET_LEVEL);
    board_delay_ms(50);

    /* 4) LINK 信号取 IO（PHY LED），并按板上极性取反 */
    esc_config_nmii_link_source(HPM_ESC, true, true, false);
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK0_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link0,
                                    BOARD_ECAT_PORT0_LINK_INVERT);
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK1_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link1,
                                    BOARD_ECAT_PORT1_LINK_INVERT);

    /* 5) 读 PHY 确认链路 */
    uint16_t bmsr = 0;
    if (esc_mdio_read(HPM_ESC, BOARD_ECAT_PORT0_PHY_ADDR, 1, &bmsr) == status_success) {
        printf("PHY0 BMSR=0x%04X link=%d 100M=%d\n", bmsr, (bmsr >> 2) & 1, (bmsr >> 13) & 1);
    }
}
```

### 4.2 EEPROM 模拟服务（协议栈轮询里调）

ESC 会通过 `EEPROM_CTRL_STAT` 发命令，PDI 必须回数据，否则主站读不到 SII（识别不了从站）。

```c
static const uint8_t sii_image[] = { /* SSC 生成的 EEPROM 镜像 */ };

void ecat_eeprom_poll(void)
{
    uint8_t cmd = esc_get_eeprom_cmd(HPM_ESC);
    if (cmd == esc_eeprom_idle_cmd) {
        return;
    }
    if (cmd == esc_eeprom_read_cmd) {
        uint32_t byte_addr = esc_get_eeprom_word_address(HPM_ESC) * 2U;
        uint64_t d;
        memcpy(&d, &sii_image[byte_addr], sizeof(d));
        esc_write_eeprom_data(HPM_ESC, d);
    }
    /* 写命令：掉电保存到 flash；reload：重新加载 */
    esc_eeprom_emulation_ack(HPM_ESC, (esc_eeprom_cmd_t)cmd, false, false);
}
```

### 4.3 过程数据 + 周期中断

```c
#define ECAT_PDRAM ((volatile uint8_t *)0xF1701000UL)   /* + PDO 偏移 */

/* SYNC0 → 电流环（比 RTOS 任务靠谱） */
esc_enable_irq(HPM_ESC, esc_sync0_irq_mask);
intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, 7);   /* 必须 > configMAX_SYSCALL_INTERRUPT_PRIORITY(4) */
intc_m_enable_irq_with_priority(IRQn_ESC, 7);         /* AL 事件（Mailbox/状态机） */

void isr_esc_sync0(void)
{
    /* 取 PDO 输入 → 跑 FOC → 写 PDO 输出。SYNC0 是事件中断，没有要清的标志位 */
    foc_step();
}

/* AL 事件（mailbox/状态机/PDO 同步），协议栈里处理 */
void isr_esc(void)
{
    uint32_t al_evt = HPM_ESC->AL_EVT_REQ;   /* 0x220 */
    (void)al_evt;
}
```

> 更硬核的玩法：`HPM_TRGM0_INPUT_SRC_ESC_SYNC0`（trgmux 输入 4）、DMAMUX 请求 `0x76/0x77` 也能用，
> SYNC0 可以硬件直连 ADC/PWM 触发，连中断都不用进。

---

## 5. 协议栈从哪来（三选一）

| 方案 | 说明 | 代价 |
|---|---|---|
| **HPMicro 官方从站方案** | 找 FAE/代理要 HPM6E00 的 EtherCAT 从站例程（SSC 5.12 移植 + CiA402），和这颗 ESC 对得上 | 要申请；不一定公开 |
| **Beckhoff SSC** | 官方 Slave Stack Code Tool 生成协议栈 + ESI XML；你只写 `esc_hw.c`（寄存器读写/EEPROM/中断）+ 应用层对象字典 | 需要 SSC 授权；工作量中等 |
| **SOES**（`OpenEtherCATsociety/SOES`） | 开源从站栈，自带 CoE/FoE/EoE、对象字典工具；但 HAL 里只有 LAN9252/XMC4/TI ESC，**没有 HPM 端口**，要自己写一份 `esc_hw.c` | GPLv2 / 商业双授权；HPM 是内存映射，移植比 SPI 的 LAN9252 简单很多 |

不管哪条路，都需要：
1. **SII 镜像**（EEPROM 内容，含 Vendor ID/Product Code/PDO 配置/SM 配置）→ 烧进 flash，用 4.2 的模拟服务喂给 ESC；
2. **ESI XML** → 给主站（TwinCAT）导入；
3. 应用层：电机驱动一般做 **CiA402**（0x6040 控制字 / 0x6041 状态字 / 0x6060 模式 / 0x607A 目标位置 / 0x6064 实际位置 …）。

---

## 6. 主站怎么测

- **PC**：TwinCAT 3（Windows，最省事；网卡要支持 EtherCAT 直通）或 IGH EtherCAT Master（Linux）。
- **HPM 当主站**：`hpmicro/ethercat_soem`（SOEM 的 HPM 移植）+ `hpmicro/hpm_apps` 里的 `apps/ecat_master_motor_control`（SOEM + CiA402 例程，可直接参考）。

---

## 7. 和本工程结合要注意的

1. **放 core0**：ESC 中断和 FOC 电流环都在 core0，别跨核访问 PDRAM。
2. **中断优先级**：工程开了 `USE_SYSCALL_INTERRUPT_PRIORITY`，临界区会屏蔽 ≤4 的中断。ESC 的 SYNC0/AL 事件必须设 **7**（比 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 高），否则会被日志/USB 的临界区挡掉，表现为"偶发丢帧/状态机乱跳"。
3. **DC 同步**：20kHz 电流环建议由 SYNC0 驱动（而不是 PWM 自由跑），这样多轴才能同拍。
4. **MDIO 冲突**：PA30/PA31 与 TSW 共用，`board_init_tsw_smi()` 也会切这组脚。用 EtherCAT 时别同时初始化 TSW 的 SMI。
5. **缓存**：内核有 32KB L1 D-Cache。PDRAM 在 `0xF1700000` 外设区，HPM 默认不走 cache；如果发现 PDO 数据"晚一拍"，先确认这段地址没被映射成 cacheable。
6. **PHY 复位**：`board_init_ethercat()` 之后 PHY 是**复位态**，不拉高就永远没 link —— 这是最容易踩的坑。

---

## 8. 相关文件速查

- 驱动：`hpm_sdk_localized_for_hpm6e00YZ/drivers/inc/hpm_esc_drv.h`、`drivers/src/hpm_esc_drv.c`
- 寄存器：`hpm_sdk_localized_for_hpm6e00YZ/soc/HPM6E00/ip/hpm_esc_regs.h`
- 板级：`hpm_sdk_localized_for_hpm6e00YZ/boards/hpm6e00YZ/{board.c,board.h,pinmux.c}`
- 基地址/中断：`soc/HPM6E00/HPM6E80/hpm_soc_ip.h`、`hpm_soc_irq.h`
- 原理图：`HPM6EMotorV20.pdf` 里的 `ethercat` 页（两张 HR911105A + 变压器）
