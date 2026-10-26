# 文档索引

仓库根目录只留 `README.md`（GitHub 落地页）和构建/调试入口文件，
其余文字资料、板卡资料与演示媒体全部收在 `docs/` 下：

```
docs/
  design/     设计与原理: 架构、算法推导、可行性验证
  hardware/   板卡硬件资料: 原理图、引脚表、外设/传感器协议
  reports/    一次性排查记录（结论保留, 过程留档）
  media/      演示媒体: images/ 截图与照片, videos/ 实拍视频
```

| 文件 | 内容 |
|---|---|
| [design/PROJECT_INTRO.md](design/PROJECT_INTRO.md) | 工程简介与核心指标（**先读这个**） |
| [design/DUAL_CORE.md](design/DUAL_CORE.md) | 双核架构、共享内存协议、启动流程与排障表 |
| [design/SPEED_LOOP_LADRC.md](design/SPEED_LOOP_LADRC.md) | 速度环一阶 LADRC：手稿推导 ↔ `src/FOC/LADRC.c` 逐行对照 |
| [design/ETHERCAT.md](design/ETHERCAT.md) | 片内 EtherCAT 从站（ESC）的可行性验证与引脚/时钟 |
| [hardware/J11_pinout.md](hardware/J11_pinout.md) | J11 扩展排针引脚定义 |
| [hardware/HPM6EMotorV20.pdf](hardware/HPM6EMotorV20.pdf) | 板卡原理图（YeeZi HPM6EMotor V2.0） |
| [hardware/encoder_SRMA35/README.md](hardware/encoder_SRMA35/README.md) | 锐鹰 SRMA35/46 RS485 多圈绝对值编码器通信协议（摘录 + 解码要点） |
| [reports/LCD_DISPLAY_FIX_REPORT.md](reports/LCD_DISPLAY_FIX_REPORT.md) | 显示问题排查记录（SPI/DMA/时序） |
| [reports/LCD_RPM_TEARING_FIX.md](reports/LCD_RPM_TEARING_FIX.md) | 转速大字撕裂/乱码的定位过程 |
| [reports/LCD_SPI_DMA_NOTES.md](reports/LCD_SPI_DMA_NOTES.md) | LCD SPI + HDMA 的实现笔记 |

## 媒体

| 文件 | 说明 |
|---|---|
| [media/videos/lcd_ui_foc_running.mp4](media/videos/lcd_ui_foc_running.mp4) | 实拍：LVGL 界面实时刷新 + FOC 闭环运行（20 s，竖屏） |
| [media/videos/motor_running_520rpm.mp4](media/videos/motor_running_520rpm.mp4) | 实拍：整机运行 520 RPM，板载双 PHY + 4015 云台电机（4 s，横屏） |
| [media/images/poster_lcd_ui_foc_running.jpg](media/images/poster_lcd_ui_foc_running.jpg) | 第一条视频的封面帧（README 里当缩略图用） |
| [media/images/poster_motor_running_520rpm.jpg](media/images/poster_motor_running_520rpm.jpg) | 第二条视频的封面帧 |
| [media/images/speed_loop_ladrc_notes.jpg](media/images/speed_loop_ladrc_notes.jpg) | 速度环一阶 LADRC 的手写推导原稿（转录见 [design/SPEED_LOOP_LADRC.md](design/SPEED_LOOP_LADRC.md)） |

## 放文件的约定

- **能在克隆后复现的**才进仓库：设计文档、原理图、协议摘录、演示媒体。
- **一次性的**（构建日志、OpenOCD 输出、flash dump、临时分析脚本）留在
  `build/`、`tmp/`，两者都已在 `.gitignore` 里；根目录散落的
  `flash_*.txt` / `ocd_*.txt` / `*.log` 同样忽略。
- 新排查记录请放 `docs/reports/`，文件名用 `现象_结论.md` 的形式；
  新设计文档放 `docs/design/`；新硬件资料放 `docs/hardware/`。
- 大文件（视频/PDF）请控制在几十 MB 以内 —— 本仓库没有启用 Git LFS。
