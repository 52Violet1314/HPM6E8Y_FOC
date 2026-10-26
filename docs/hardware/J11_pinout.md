# J11 扩展排针引脚表（已核对）

来源：`docs/hardware/HPM6EMotorV20.pdf` PAGE 2，用 pymupdf 按坐标提取（脚号 x≈594/607，网络名 x≈545/643），
不是靠纯文本顺序猜的。提取脚本：[tools/schematic/j11_table.py](../../tools/schematic/j11_table.py)
（`python tools/schematic/j11_table.py`，依赖 pymupdf；探针脚本 `j11_probe.py`）。

| 脚 | 网络 | 脚 | 网络 |
|----|------|----|------|
| 1  | **+5V**  | 2  | **+5V**  |
| 3  | PD17 | 4  | PD18 |
| 5  | PD20 | 6  | PD21 |
| 7  | PD9  | 8  | PD10 |
| 9  | PD11 | 10 | **PC28** |
| 11 | **PC24** | 12 | PD16 |
| 13 | PD24 | 14 | PD19 |
| 15 | PD26 | 16 | PD22 |
| 17 | PD28 | 18 | PD30 |
| 19 | **GND** | 20 | **GND** |
| 21 | PD31 | 22 | PD23 |
| 23 | PD27 | 24 | PD25 |
| 25 | PF13 | 26 | PD29 |
| 27 | PC25 | 28 | PF12 |
| 29 | PF14 | 30 | PF15 |
| 31 | **PC26** | 32 | PF11 |
| 33 | PF16 | 34 | PF17 |
| 35 | PF26 | 36 | PF27 |
| 37 | PF25 | 38 | PF20 |
| 39 | PF21 | 40 | PF24 |
| 41 | PF19 | 42 | PD8 |
| 43 | **+3V3** | 44 | **+3V3** |

## 本次编码器接线用到的脚

| 信号 | MCU 引脚 | J11 脚 | 固件复用值 |
|------|----------|--------|-----------|
| SCLK | PC26 | 31 | `IOC_PC26_FUNC_CTL_SPI2_SCLK` = ALT5 |
| MISO | PC28 | 10 | `IOC_PC28_FUNC_CTL_SPI2_MISO` = ALT5 |
| CS   | PC24 | 11 | `IOC_PC24_FUNC_CTL_GPIO_C_24` = ALT0 |
| 电源 | —    | 43 或 44 (+3V3) | — |
| 地   | —    | 19 或 20 (GND)  | — |

> 注意：PC29（SPI2_MOSI）**不在 J11 上**，MT6701 是只读器件，不需要它。
> SPI2.CS_0（PC27）也不在 J11 上，所以片选用 GPIO 软件控制。
