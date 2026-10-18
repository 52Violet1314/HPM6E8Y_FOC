/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核): 镜像头
 *
 * 这个 16 字节的结构体被链接到 core1 镜像的【最前面】(0x80100000), core0 在释放
 * core1 之前先读它, 用来判断"core1 的镜像到底烧没烧、版本对不对"(见
 * src/dualcore/core1_boot.c)。定义与约定见 src/dualcore/core1_boot_info.h。
 *
 * 之所以要有它: core1 的入口是写死的 flash 地址, core0 没法"链不到就报错" ——
 * 没有这个头, 忘烧 core1 镜像时 core0 会把 core1 扔进 0xFF 空白 flash 里执行,
 * 现象是随机跑飞。有了头, 就是串口上一行明确的错误信息。
 */
#include "core1_boot_info.h"

__attribute__((section(".core1_boot_info"), used))
const core1_boot_info_t g_core1_boot_info = {
    .magic    = CORE1_IMG_MAGIC,
    .version  = CORE1_IMG_VERSION,
    .entry    = CORE1_IMG_ENTRY,
    .reserved = 0U,
};
