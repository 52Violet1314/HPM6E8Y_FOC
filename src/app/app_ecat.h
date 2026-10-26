/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 内置 EtherCAT(ESC) 开箱自检 + 回环(链路)测试
 *
 * 用途: 在引入任何协议栈之前, 先证明"硬件这条路"是通的。
 *
 * 测试接线(回环): 一根网线把两个 EtherCAT 口对接(J1 <-> J2, 也就是 P0 <-> P1)。
 *   预期结果(实测已确认):
 *     - ESC DL_STAT: PLP0/PLP1 = 1, LP0/LP1 = 0 (两个口都"开着", 不内部环回),
 *       CP0/CP1 也会变 1 —— 这是本板唯一可用的链路判据;
 *     - MDIO(PDI 侧)读不到任何 PHY, PHY_STAT 也恒 0 —— 【正常】: 片内 PHY 没有
 *       对外暴露寄存器通道。LINK 是 PHY 的 LED 输出经 ESC0_CTR_x 回灌进 ESC 的,
 *       所以速率/双工这类寄存器值读不到, 但"有没有 link"是准确的;
 *     - EEPROM_loaded=0 —— 正常, 还没跑协议栈、没喂 SII。
 *
 * 拔掉网线再采样: PLP 变 0、LP 变 1(该口转入内部环回)、lost_link 计数 +1,
 * 用来确认链路检测真的在工作。
 *
 * 【注意】这一步不能验证报文通路: ESC 只转发/处理别人发来的帧, 自己不会发帧,
 * 所以 PDO 通路要等接了主站(PC 裸帧 / SOEM / TwinCAT)才算验证。
 */
#ifndef APP_ECAT_H
#define APP_ECAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* 开机自检开关: 1 = app_init() 里跑一次 ESC/PHY 自检(默认开, 测完可以关掉) */
#define APP_ECAT_SELFTEST              (1)

/* 自检之后继续"监视链路"多久(ms), 期间每 APP_ECAT_LINK_WATCH_STEP_MS 采样一次,
 * 只有状态变化才打印一行。0 = 不监视(开机不额外等待)。
 * 做回环测试时保留默认值: 开机后有 3 秒时间插/拔网线看状态跳变。 */
#define APP_ECAT_LINK_WATCH_MS         (3000U)
#define APP_ECAT_LINK_WATCH_STEP_MS    (250U)

/**
 * @brief ESC/PHY 自检: 初始化 -> 打印能力寄存器与两个口的状态 -> (可选)监视链路
 *
 * 放在 app_init() 里、其它 BSP 初始化之后调用; 阻塞 (LINK_WATCH_MS + 自协商时间)。
 */
void app_ecat_selftest(void);

/**
 * @brief 只打印一次当前状态(不初始化、不等待), 供运行中排查使用
 */
void app_ecat_dump_status(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ECAT_H */
