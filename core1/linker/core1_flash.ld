/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * core1(显示核) 的链接脚本 —— 代码在 flash 里就地执行(XIP)
 *
 * 【为什么不学 SDK 的 multicore 例程放 ILM】
 *   SDK 的 sec_core_img 流程是"把 core1 镜像拷进它的 ILM(256KB)再跑"。本工程的
 *   显示核带 LVGL(9.x), 实测链接出来:
 *       .text + .rodata ≈ 700KB (LVGL 全部控件/字体/绘制 + FreeRTOS + 驱动)
 *   —— 256KB 的 ILM 装不下, 而 core1 能用的 RAM(DLM 256KB + AXI_SRAM 240KB)
 *   同样装不下。所以 core1 的代码不放 ILM, 而是【留一段 flash 给它, 就地执行】:
 *       core1 代码/常量: flash 0x80100000 起 (1MB, 见下面 FLASH 区)
 *       core1 数据/bss  : AXI_SRAM 0x012D0000 起(自己的 176KB 那一半)
 *       core1 栈/堆     : DLM 0x00240000 起(自己的 256KB)
 *   代价是取指要走 XPI(有缓存), 收益是 LVGL 可以按原样全功能使用, 而且
 *   两个核的镜像都只烧一次(见根 CMakeLists 里的 .core1_img 段)。
 *
 * 【为什么 flash 偏移取 1MB】
 *   0x80000000..0x80100000 留给 core0(LVGL 搬走之后它只有 ~100KB, 1MB 很宽裕)。
 *   真长到超过 1MB, 链接时会直接报 .core1_img 与 core0 的段重叠(见下面注), 不会
 *   悄悄踩掉。
 *
 * 【镜像头】文件最前面 16 字节是 core1_boot_info_t(magic/version/entry, 见
 *   src/dualcore/core1_boot_info.h): core0 释放 core1 之前先来读它 —— 忘了烧
 *   core1 镜像(或版本不匹配)时, core0 会在串口明确报出来, 而不是让 core1 跑去
 *   执行 0xFF 0xFF ... 的空白 flash。所以真正的入口是 0x80100010, 不是 0x80100000。
 */

ENTRY(_start)

STACK_SIZE = _stack_size;
HEAP_SIZE = _heap_size;

/* 镜像头长度: 改 core1_boot_info_t 时必须同步改这里(下面有 ASSERT 兜底) */
CORE1_IMG_BASE  = 0x80100000;
CORE1_IMG_HDR   = 16;

MEMORY
{
    FLASH (rx) : ORIGIN = 0x80100000, LENGTH = 1M
    DLM (w) : ORIGIN = 0x00240000, LENGTH = 256K
    AXI_SRAM_NONCACHEABLE (wx) : ORIGIN = 0x012C0000, LENGTH = _noncacheable_size
    AXI_SRAM (wx) : ORIGIN = 0x012C0000 + _noncacheable_size, LENGTH = 240K - _noncacheable_size
    SHARE_RAM (w) : ORIGIN = 0x012FC000, LENGTH = 16K
}

SECTIONS
{
    /* 镜像头: core0 读它做"core1 镜像到底烧没烧/版本对不对"的检查 */
    .core1_boot_info : {
        . = ALIGN(CORE1_IMG_HDR);
        KEEP(*(.core1_boot_info))
        . = ALIGN(CORE1_IMG_HDR);
    } > FLASH

    .start : {
        . = ALIGN(8);
        KEEP(*(.start))
        . = ALIGN(8);
    } > FLASH

    /* 头长度变了 -> 入口地址就变了 -> 这里直接链接失败, 免得 core0 跳到半个字上 */
    ASSERT(ADDR(.start) == (CORE1_IMG_BASE + CORE1_IMG_HDR),
           "****** FAILED! .core1_boot_info 长度变了, 同步 core1_boot_info.h 的 CORE1_IMG_ENTRY ******")

    .vectors : {
        . = ALIGN(8);
        KEEP(*(.isr_vector))
        KEEP(*(.isr_vector.*))
        KEEP(*(.vector_table))
        KEEP(*(.isr_s_vector))
        KEEP(*(.isr_s_vector.*))
        KEEP(*(.vector_s_table))
        . = ALIGN(8);
    } > FLASH

    .rel : {
        KEEP(*(.rel*))
    } > FLASH

    .text : {
        . = ALIGN(8);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        *(.srodata)
        *(.srodata*)

        *(.hash)
        *(.dyn*)
        *(.gnu*)
        *(.pl*)

        KEEP (*(.init))
        KEEP (*(.fini))

        /* section information for usbh class */
        . = ALIGN(8);
        __usbh_class_info_start__ = .;
        KEEP(*(.usbh_class_info))
        __usbh_class_info_end__ = .;

        /* RT-Thread related sections - Start */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;
        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;
        . = ALIGN(4);
        __rtmsymtab_start = .;
        KEEP(*(RTMSymTab))
        __rtmsymtab_end = .;
        /* RT-Thread related sections - end */

        . = ALIGN(8);
    } > FLASH

    .eh_frame :
    {
        __eh_frame_start = .;
        KEEP(*(.eh_frame))
        __eh_frame_end = .;
    }  > FLASH

    .eh_frame_hdr :
    {
        KEEP(*(.eh_frame_hdr))
    }  > FLASH
    __eh_frame_hdr_start = SIZEOF(.eh_frame_hdr) > 0 ? ADDR(.eh_frame_hdr) : 0;
    __eh_frame_hdr_end = SIZEOF(.eh_frame_hdr) > 0 ? . : 0;

    . = ALIGN(8);
    PROVIDE (__etext = .);
    PROVIDE (_etext = .);
    PROVIDE (etext = .);

    /* ---- 以下是"要拷到 RAM 里"的段: LMA 在 flash 里紧跟着 .text ---- */
    __data_load_addr__ = etext;
    .data : AT(__data_load_addr__) ALIGN(8) {
        __data_start__ = .;
        __global_pointer$ = . + 0x800;
        *(.data)
        *(.data*)
        *(.sdata)
        *(.sdata*)

        KEEP(*(.jcr))
        KEEP(*(.dynamic))
        KEEP(*(.got*))
        KEEP(*(.got))
        KEEP(*(.gcc_except_table))
        KEEP(*(.gcc_except_table.*))

        . = ALIGN(8);
        PROVIDE(__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE(__preinit_array_end = .);

        . = ALIGN(8);
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);

        . = ALIGN(8);
        PROVIDE(__fini_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE(__fini_array_end = .);

        . = ALIGN(8);
        KEEP(*crtbegin*.o(.ctors))
        KEEP(*(EXCLUDE_FILE (*crtend*.o) .ctors))
        KEEP(*(SORT(.ctors.*)))
        KEEP(*(.ctors))

        . = ALIGN(8);
        KEEP(*crtbegin*.o(.dtors))
        KEEP(*(EXCLUDE_FILE (*crtend*.o) .dtors))
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))

        . = ALIGN(8);
        __data_end__ = .;
        PROVIDE (__edata = .);
        PROVIDE (_edata = .);
        PROVIDE (edata = .);
    } > AXI_SRAM

    /* .fast = 必须在 RAM 里执行的函数(本工程 core1 没有这种函数, 留段是为了
     * 与 SDK 的 reset.c/start.S 保持同一套符号约定) */
    __fast_load_addr__ = etext + SIZEOF(.data);
    .fast : AT(__fast_load_addr__) ALIGN(8) {
        PROVIDE(__ramfunc_start__ = .);
        *(.fast)
        *(.fast.*)
        . = ALIGN(8);
        PROVIDE(__ramfunc_end__ = .);
    } > AXI_SRAM

    __tdata_load_addr__ = etext + SIZEOF(.data) + SIZEOF(.fast);
    .tdata : AT(__tdata_load_addr__) ALIGN(8) {
        PROVIDE(__tdata_start__ = .);
        *(.tdata)
        *(.tdata.*)
        *(.gnu.linkonce.td.*)
        . = ALIGN(8);
        PROVIDE(__tdata_end__ = .);
    } > AXI_SRAM

    .tbss (NOLOAD) : {
        . = ALIGN(8);
        PROVIDE(__tbss_start__ = .);
        __thread_pointer$ = .;
        *(.tbss)
        *(.tbss.*)
        *(.gnu.linkonce.tb.*)
        *(.tcommon)
        . = ALIGN(8);
        PROVIDE(__tbss_end__ = .);
    } > AXI_SRAM

    __noncacheable_init_load_addr__ = etext + SIZEOF(.data) + SIZEOF(.fast) + SIZEOF(.tdata);
    .noncacheable.init : AT(__noncacheable_init_load_addr__) ALIGN(8) {
        __noncacheable_init_start__ = .;
        KEEP(*(.noncacheable.init))
        KEEP(*(.noncacheable.init.*))
        . = ALIGN(8);
        __noncacheable_init_end__ = .;
    } > AXI_SRAM_NONCACHEABLE

    __fast_ram_init_load_addr__ = etext + SIZEOF(.data) + SIZEOF(.fast) + SIZEOF(.tdata) + SIZEOF(.noncacheable.init);
    .fast_ram.init : AT(__fast_ram_init_load_addr__) ALIGN(8) {
        __fast_ram_init_start__ = .;
        KEEP(*(.fast_ram.init))
        KEEP(*(.fast_ram.init.*))
        . = ALIGN(8);
        __fast_ram_init_end__ = .;
    } > DLM

    .bss (NOLOAD) : {
        . = ALIGN(8);
        __bss_start__ = .;
        *(.bss)
        *(.bss*)
        *(.sbss*)
        *(.scommon)
        *(.scommon*)
        *(.dynsbss*)
        *(COMMON)
        . = ALIGN(8);
        _end = .;
        __bss_end__ = .;
    } > AXI_SRAM

    .framebuffer (NOLOAD) : {
        . = ALIGN(8);
        KEEP(*(.framebuffer))
        KEEP(*(.framebuffer.*))
        . = ALIGN(8);
    } > AXI_SRAM

    .noncacheable (NOLOAD) : {
        . = ALIGN(8);
        KEEP(*(.noncacheable))  /* Legacy sections, kept for backwards compatibility */
        KEEP(*(.noncacheable.non_init))
        KEEP(*(.noncacheable.non_init.*))
        . = ALIGN(8);
        __noncacheable_bss_start__ = .;
        KEEP(*(.noncacheable.bss))
        KEEP(*(.noncacheable.bss.*))
        . = ALIGN(8);
        __noncacheable_bss_end__ = .;
    } > AXI_SRAM_NONCACHEABLE

    .sh_mem (NOLOAD) : {
        KEEP(*(.sh_mem))
        KEEP(*(.sh_mem.*))
    } > SHARE_RAM

    .fast_ram (NOLOAD) : {
        . = ALIGN(8);
        KEEP(*(.fast_ram))  /* Legacy sections, kept for backwards compatibility */
        KEEP(*(.fast_ram.non_init))
        KEEP(*(.fast_ram.non_init.*))
        . = ALIGN(8);
        __fast_ram_bss_start__ = .;
        KEEP(*(.fast_ram.bss))
        KEEP(*(.fast_ram.bss.*))
        . = ALIGN(8);
        __fast_ram_bss_end__ = .;
    } > DLM

    .heap (NOLOAD) : {
        . = ALIGN(8);
        __heap_start__ = .;
        . += HEAP_SIZE;
        . = ALIGN(8);
        __heap_end__ = .;
    } > DLM

    .stack (NOLOAD) : {
        . = ALIGN(16);
        __stack_base__ = .;
        . += STACK_SIZE;
        . = ALIGN(16);
        PROVIDE (_stack = .);
        PROVIDE (_stack_safe = .);
    } > DLM

    __noncacheable_start__ = ORIGIN(AXI_SRAM_NONCACHEABLE);
    __noncacheable_end__ = ORIGIN(AXI_SRAM_NONCACHEABLE) + LENGTH(AXI_SRAM_NONCACHEABLE);
    __share_mem_start__ = ORIGIN(SHARE_RAM);
    __share_mem_end__ = ORIGIN(SHARE_RAM) + LENGTH(SHARE_RAM);

    /* ---- 上电时 core0 要搬进 core1 的东西 = 这些"有初值"的段 ----
     * 注意 .core1_boot_info/.start/.vectors/.text/.rodata 都在 flash 里【就地执行】,
     * 所以 __fw_size__ 只算"要拷贝的部分", 不再代表整个镜像大小。 */
    __fw_size__ = SIZEOF(.data) + SIZEOF(.fast) + SIZEOF(.tdata) + SIZEOF(.noncacheable.init) + SIZEOF(.fast_ram.init);
    __last_addr__ = __fast_ram_init_load_addr__ + SIZEOF(.fast_ram.init);

    /* 数据段的 LMA 必须落在 flash 区内(否则 core0 一开机从 flash 读初值会取到空) */
    ASSERT(__last_addr__ <= (ORIGIN(FLASH) + LENGTH(FLASH)),
           "****** FAILED! core1 的 flash 区装不下(镜像 > 1MB), 见 core1/linker/core1_flash.ld ******")
}
