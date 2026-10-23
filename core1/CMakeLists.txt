# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
#
# core1(显示核) 工程: LVGL + ILI9341 + XPT2046
#
# 【这个工程和根目录那个工程的关系】
#   根 CMakeLists.txt  = core0(电机控制核), 链接到 flash, 是真正被烧写的那个 elf。
#   core1/CMakeLists.txt = 本文件, 显示核。它也被链接到 flash, 但地址固定在
#     0x80100000 起 1MB(见 linker/core1_flash.ld), 由 core0 的构建把这段镜像
#     编进 core0 的 elf(.core1_img 段) —— core1 就在那里就地执行(XIP)。
#     为什么不像 SDK 的 multicore 例程那样放 ILM: LVGL 太大, 装不下。
#
# 【单独构建本工程】也能用(排查问题时很方便):
#   cmake -S core1 -B build/core1 -G Ninja -DBOARD=hpm6e00YZ
#   cmake --build build/core1
# 但注意: 根工程的构建会覆盖 build/core1 的内容, 两边用的是同一个目录。

cmake_minimum_required(VERSION 3.13)

# ---- 构建类型: 次级核镜像 ----
# sec_core_img 会让 SDK 做三件事(都在 SDK 的 CMakeLists 里判断):
#   1) BUILD_FOR_SECONDARY_CORE=ON -> 板级 CMakeLists 定义 BOARD_RUNNING_CORE=HPM_CORE1;
#   2) 构建结束后自动调 bin2c.py 生成 sec_core_img.c (这里指到 build 目录里,
#      不往源码树里丢生成文件; 真正给 core0 用的是 cmake/core1_img2c.py 生成的那份,
#      因为它还要把数组放进 .core1_img 段);
#   3) 链接脚本默认取 ram_core1.ld —— 但下面用 CUSTOM_GCC_LINKER_FILE 换成我们自己的
#      flash 版本(代码就地执行, 不拷进 ILM), 理由见 core1/linker/core1_flash.ld。
set(HPM_BUILD_TYPE "sec_core_img")
set(APP_NAME "core1")
set(SEC_CORE_IMG_C_ARRAY_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/sec_core_img_auto.c")
# 【必须是绝对路径】SDK 会检查
set(CUSTOM_GCC_LINKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/linker/core1_flash.ld")
# 非缓存区大小: 用自定义链接脚本时 SDK 不会自动设成"次级核"的 64K, 会退回默认的
# 256K —— 那样 AXI_SRAM 区长度会算成 240K-256K(负数), 段被放到区外还不报错(实测)。
# 必须在 find_package 之前设置: SDK 是在配置过程中读这个变量来生成链接符号的。
set(NONCACHEABLE_RAM_SIZE 64K)

# ---- 组件开关 (core1 只要 GUI + RTOS) ----
set(CONFIG_LVGL 1)
set(CONFIG_LVGL_CUSTOM_PORTABLE 1)
set(CONFIG_FREERTOS 1)

# ---- SDK (用工程里那份本地化 SDK, 与 core0 完全同一个) ----
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../hpm_sdk_localized_for_hpm6e00YZ")
set(LOCALIZED_BOARD "hpm6e00YZ")
if(BOARD)
  if(NOT ${BOARD} MATCHES ${LOCALIZED_BOARD})
    message(FATAL_ERROR "ABORT:\n hpm sdk has been localized for ${LOCALIZED_BOARD} already.")
  endif()
endif()
set(BOARD ${LOCALIZED_BOARD})

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(yeezi_core1)

# ---- 和 core0 一致: 让临界区按 PLIC 优先级屏蔽而不是关全局 MIE ----
# core1 的实时中断是 LCD 像素 DMA(优先级 4, <=阈值) -> 临界区里可屏蔽;
# 它会在中断里调 xSemaphoreGiveFromISR, 按规范【必须】可屏蔽, 所以优先级
# 从 6 降到 4(见 core1/rtos/app_rtos.h)。两核用同一个 port, 开关必须一致,
# 否则同一个 elf 里两套 port.c 实现会不一致。
sdk_compile_definitions(-DUSE_SYSCALL_INTERRUPT_PRIORITY=1)

# ---- 头文件路径 ----
sdk_inc(rtos)                 # core1 自己的 FreeRTOSConfig.h / app_rtos.h
sdk_inc(src)
sdk_inc(../src/dualcore)      # shm_core.h: 两个核共用同一份共享内存协议
sdk_inc(../src/app)           # app.h: 遥测客户端必须与 core0 声明一致
sdk_inc(../src/lcd)           # 面板/触摸/LVGL 桥(与 core0 共用同一批源文件)

# ---- core1 自己的代码 ----
sdk_app_src(src/main.c)
sdk_app_src(src/boot_info.c)        # 镜像头: core0 靠它判断"烧的是不是我"
sdk_app_src(src/shm_client.c)
sdk_app_src(src/app_telem_client.c)
sdk_app_src(src/app_log_ring.c)
sdk_app_src(src/console_ring.c)     # 覆盖 SDK 的弱 bsp_console_write: printf 也走日志环
# core1 自己的控制台: UART14 = PF24(TXD)/PF25(RXD) = J11 40/37
# 【为什么需要它】原来 core1 的日志全靠 core0 的日志任务转发, 而 core0 的控制环
# 一旦跑满 CPU(实测 979Hz > 1kHz 周期)就不再让出, 低优先级的日志任务永远轮不到
# —— 两个核的日志会一起消失。这条 UART 让 core1 彻底独立。
sdk_app_src(src/console_uart14.c)
sdk_app_src(rtos/app_rtos.c)

# ---- 显示/触摸/LVGL 桥 ----
# 【注意】这几个文件原来在 core0 的 CMakeLists 里。双核之后它们只在 core1 里编译
# (core0 那份已经删掉): 同一份源码在哪个核上编译, 决定了它访问的是哪个核的外设。
# 它们对 app 层的依赖(app_speed_*)由 src/app_telem_client.c 在 core1 侧实现。
sdk_app_src(../src/lcd/LCDHw.c)
sdk_app_src(../src/lcd/LCDTouch.c)
sdk_app_src(../src/lcd/LCDGui.c)
sdk_app_src(../src/lcd/lcd_app.c)
sdk_app_src(../src/lcd/lcd_ui.c)
# 大字转速用的等宽数字字体(由 tools/_mk_digitfont.py 生成, 位图借自 montserrat_40)
sdk_app_src(../src/lcd/lv_font_digits_40.c)

# LVGL 的绘制/字体代码用到 libm
sdk_link_libraries(m)
