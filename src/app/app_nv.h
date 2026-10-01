/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * APP: 校准数据的 Flash 持久化 (LUT 表 + 电角度偏移)
 *
 *   校准一次之后把表存进 XPI NOR 尾部, 以后上电直接读表换向,
 *   不用再等 3 分钟静态扫描。
 */
#ifndef APP_NV_H
#define APP_NV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 上电先探测一次 XPI NOR (只读, 不擦写) */
void app_nv_init(void);

/**
 * @brief 从 Flash 读回已保存的 LUT + 偏移角, CRC 校验通过才返回 true。
 * @param table_out          至少 ENC_LUT_BINS 个 int16 的输出缓冲
 * @param offset_mrad_out    标定偏置输出 (mrad)
 */
bool app_nv_load(int16_t *table_out, int32_t *offset_mrad_out);

/**
 * @brief 把 LUT + 偏移角写入 Flash (擦 1 个扇区 + 编程)。
 *        只在固件接受一张表之后调用; 内部会关全局中断, 阻塞约数百 ms。
 */
bool app_nv_save(const int16_t *table, int32_t offset_mrad);

#ifdef __cplusplus
}
#endif

#endif /* APP_NV_H */
