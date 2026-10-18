/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 双核: core1 镜像头(两个核共用同一份定义)
 *
 * 【它解决什么问题】core1 的代码在 flash 里就地执行(XIP), 入口地址是写死的
 * (0x80100010), core0 上电时"直接跳过去"。万一 core1 的镜像没烧、烧了个旧版本、
 * 或者 flash 里那段根本是空的, core0 就会把 core1 放进一堆 0xFF 里执行 —— 现象是
 * 随机跑飞, 极其难查。
 *
 * 所以 core1 的镜像最前面 16 字节放这个头(链接脚本 .core1_boot_info 段), core0
 * 在释放 core1 之前先读一遍: magic/version 不对就不放, 并在串口明确报出来。
 * 这 16 字节也决定了入口地址: 0x80100000 + 16 = 0x80100010。
 *
 * 【改这个结构体要注意】长度必须保持 16 字节, 并且同步 core1/linker/core1_flash.ld
 * 里的 CORE1_IMG_HDR(那里有 ASSERT 兜底: 对不上直接链接失败)。
 */
#ifndef CORE1_BOOT_INFO_H
#define CORE1_BOOT_INFO_H

#include <stdint.h>

/* 放在 flash 里的位置: 0x80000000..0x80100000 留给 core0(见 core1_flash.ld 说明) */
#define CORE1_IMG_BASE          (0x80100000UL)
#define CORE1_IMG_HDR_SIZE      (16U)
#define CORE1_IMG_ENTRY         (CORE1_IMG_BASE + CORE1_IMG_HDR_SIZE)
/* 可用空间 = 链接脚本里 FLASH 区的长度(1MB); 超出会链接失败, 这里只是给运行期检查兜底 */
#define CORE1_IMG_MAX_SIZE      (0x00100000UL)

#define CORE1_IMG_MAGIC         (0x31435248UL)  /* 'H','R','C','1' = HPM RISC-V Core1 */
#define CORE1_IMG_VERSION       (1U)

typedef struct {
    uint32_t magic;         /* CORE1_IMG_MAGIC */
    uint32_t version;       /* CORE1_IMG_VERSION: 两边镜像必须同一次构建出来的 */
    uint32_t entry;         /* 入口绝对地址 (= CORE1_IMG_ENTRY), 冗余存一份便于自检 */
    uint32_t reserved;      /* 对齐到 16 字节 */
} core1_boot_info_t;

/* 编译期护栏: 头必须是 16 字节(链接脚本的 ASSERT 按 16 算) */
typedef char core1_boot_info_size_must_be_16[(sizeof(core1_boot_info_t) == CORE1_IMG_HDR_SIZE) ? 1 : -1];

#endif /* CORE1_BOOT_INFO_H */
