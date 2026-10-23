# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
#
# YeeZi HPM6EMotor V2.0 应用工程
#   MCU   : HPM6E8Y (HPM6E80, BGA289)
#   Board : hpm6e00YZ
#   SDK   : ./hpm_sdk_localized_for_hpm6e00YZ (本地化)
#
# 板级引脚配置见: hpm_sdk_localized_for_hpm6e00YZ/boards/hpm6e00YZ/pinmux.c

cmake_minimum_required(VERSION 3.13)

# ---- 组件开关 ----
set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_CDC_ACM 1)
# HPM 数学库 (软件子集): FOC 使用其查表快速 sin/cos
set(CONFIG_HPM_MATH 1)
# LVGL 9.x GUI middleware.  The board-specific display/input port remains
# application-owned until the panel controller and pinout are selected.
set(CONFIG_LVGL 1)
set(CONFIG_LVGL_CUSTOM_PORTABLE 1)
# FreeRTOS: 显示与打印各派一个任务, 控制任务锁 1ms 硬周期。
#   中间件已拷入本地化 SDK: hpm_sdk_localized_for_hpm6e00YZ/middleware/FreeRTOS
#   配置见 src/rtos/FreeRTOSConfig.h, 任务划分见 src/rtos/app_rtos.c
#   CherryUSB 会自动切到 usb_osal_freertos.c (它的 CMakeLists 里判的就是这个开关)
set(CONFIG_FREERTOS 1)

# ---- 构建类型 (HPM_BUILD_TYPE) ----
#   ram       : 链接到 ILM/RAM (入口 0x0)。SDK 默认值。
#               注意: 芯片复位后是从 flash(0x80000000) 启动的, 直接复位运行
#               RAM 镜像会跑到旧/空 flash 内容里, 表现为 J-Link 一直卡住、
#               永远到不了 main。除非调试器额外把 PC 指到 RAM 入口, 否则别用。
#   flash_xip : 链接到 flash(0x80000000)。J-Link 下载时编程 flash, 复位后
#               ROM 正常跳转到应用 -> main。用 J-Link 下载/调试请选这个。
# 需要临时用 RAM 方式时: cmake -DHPM_BUILD_TYPE=ram ...
if(NOT DEFINED HPM_BUILD_TYPE)
  set(HPM_BUILD_TYPE "flash_xip")
endif()

# 以下 "### START ### ... ### END ###" 段落由工程本地化(Localization)自动添加
### START ###
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/hpm_sdk_localized_for_hpm6e00YZ")
set(LOCALIZED_BOARD "hpm6e00YZ")
if(BOARD)
  if(NOT ${BOARD} MATCHES ${LOCALIZED_BOARD})
    message(FATAL_ERROR "ABORT:\n hpm sdk has been localized for ${LOCALIZED_BOARD} already.")
  endif()
endif()
set(BOARD ${LOCALIZED_BOARD})
### END ###


find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(yeezi_hpm6emotor)

# ---- 【关键】让 RTOS 临界区按 PLIC 优先级屏蔽, 而不是关全局 MIE ----
# HPM 的 PLIC: 数值越大优先级越高, 只有 "优先级 > THRESHOLD" 才被投递。
# 不定义这个开关时, portDISABLE_INTERRUPTS() 就是 clear_csr(mstatus, MIE) —— 
# 临界区里【所有】中断全关, 优先级形同虚设: 日志/队列/USB 一进临界区,
# 20kHz 电流环就被挡掉(实测 1.5ms 级的间隔 -> 电机"跳一下" + 电流环自激)。
# 定义后临界区改成 "PLIC 阈值 = configMAX_SYSCALL_INTERRUPT_PRIORITY(=4)":
#   电流环 ADC = 7 (>4) -> 永不被临界区屏蔽;
#   串口TX=3 / USB=4 (<=4) -> 可被屏蔽(USB 会在中断里调 RTOS API, 必须可屏蔽)。
# 出处: HPMicro 知识库《在HPM系列芯片上实现FreeRTOS对中断优先级的管理》(sdk v1.8+)。
sdk_compile_definitions(-DUSE_SYSCALL_INTERRUPT_PRIORITY=1)

# ---- BSP ----
sdk_inc(src/bsp)
sdk_app_src(src/bsp/bsp_pwm.c)
sdk_app_src(src/bsp/bsp_adc.c)
sdk_app_src(src/bsp/bsp_usb.c)
sdk_app_src(src/bsp/usb_cdc_acm.c)
# 编码器 = MT6701, 经 SPI2 走 J11 扩展排针。
sdk_app_src(src/bsp/bsp_spi.c)
sdk_app_src(src/bsp/bsp_mt6701.c)
# GPIO 软件模拟 SPI 作为接线排查/兜底后端
sdk_app_src(src/bsp/bsp_spi_bb.c)
# J11 排针按键 (位置点动; 低有效 + 内部上拉 + 消抖/长按连发)
sdk_app_src(src/bsp/bsp_button.c)

# ---- LCD/LVGL: 已整体搬到 core1 (见 core1/CMakeLists.txt) ----
# 这里【故意】不再编译 src/lcd/*: 双核之后显示层归 core1, 同一份源码在哪个核上
# 编译就决定了它操作的是哪个核的外设(SPI1/HDMA/GPIO)。core0 里再留一份会白白
# 占 flash, 更要命的是留下"到底哪个核在刷屏"的歧义。

# ---- Hardware: 传感器/驱动器驱动 (纯协议层, 无硬件) ----
# MT6701 磁编码器帧解析/CRC
sdk_inc(src/Hardware/mt6701)
sdk_app_src(src/Hardware/mt6701/mt6701.c)

# ---- FOC: 磁场定向控制算法 ----
sdk_inc(src/FOC)
sdk_app_src(src/FOC/foc.c)
sdk_app_src(src/FOC/foc_math.c)
# 通用 PID (电流环/速度环/位置环共用, 语义同 F405_M3508_FOC 的 PID.c)
sdk_app_src(src/FOC/pid.c)
sdk_app_src(src/FOC/foc_openloop.c)
sdk_app_src(src/FOC/foc_controller.c)
# FOC 使用 sinf/cosf/sqrtf/fmodf, 需要链接 libm
sdk_link_libraries(m)

# ---- APP: 应用编排层 ----
sdk_inc(src/app)
sdk_app_src(src/app/app.c)
sdk_app_src(src/app/app_diag.c)
sdk_app_src(src/app/app_log.c)
sdk_app_src(src/app/app_encoder.c)
# 位置环 (外环): 位置误差 -> 速度给定, 内环是 app.c 里的速度环
sdk_app_src(src/app/app_motion.c)
# 电流环录波 (20kHz 采样 -> 触发 -> 串口 CSV 转储), 调 PI 用
sdk_app_src(src/app/app_scope.c)
# 离线标定的 3200 组"编码器 raw -> 电角度"查表 (tools/enc_lut_build.py 生成)
sdk_app_src(src/app/app_enc_lut.c)
sdk_app_src(src/app/app_nv.c)

# ---- RTOS: 任务划分 (控制 / 日志) ----
sdk_inc(src/rtos)
sdk_app_src(src/rtos/app_rtos.c)

# ---- 双核: core1(显示核) 的镜像与协作 ----
# core1 是一个独立的 CMake 工程(core1/), 走 SDK 的 sec_core_img 构建类型, 但用
# core1/linker/core1_flash.ld 把它链接到 flash 的 0x80100000 起 1MB —— 也就是
# 【就地执行(XIP), 不拷进 ILM】: 显示核带 LVGL, 实测 .text+.rodata ≈ 700KB, 装不进
# 256KB 的 ILM(详见 core1/linker/core1_flash.ld 顶部说明)。构建顺序:
#   1) 配置并编译 core1  (产物 core1/output/core1.bin, 已按 flash 布局排好)
#   2) cmake/core1_img2c.py 把它转成 C 数组 sec_core_img[](段 .core1_img)
#   3) core0 链接时用 --section-start 把这个段钉在 0x80100000, 于是这份镜像就
#      躺在 core0 的 elf/烧写文件里; 上电后 core0 检查镜像头并释放 core1
#      (见 src/dualcore/core1_boot.c)。
# 结果: 仍然是"只烧一个 elf"; core1 的代码不占 core0 的 RAM, 只占 flash。
set(CORE1_SRC_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/core1)
set(CORE1_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/core1)
set(CORE1_IMG_BIN   ${CORE1_BUILD_DIR}/output/core1.bin)
set(CORE1_IMG_SRC   ${CMAKE_CURRENT_BINARY_DIR}/generated/sec_core_img.c)
set(CORE1_IMG_BASE  0x80100000)     # 必须与 src/dualcore/core1_boot_info.h 的 CORE1_IMG_BASE 一致

# 依赖: core1 自己的源文件 + 与它共用的显示层 + 共享内存/镜像头 + 链接脚本
file(GLOB_RECURSE CORE1_DEPS CONFIGURE_DEPENDS
    ${CORE1_SRC_DIR}/src/*.c  ${CORE1_SRC_DIR}/src/*.h
    ${CORE1_SRC_DIR}/rtos/*.c ${CORE1_SRC_DIR}/rtos/*.h
    ${CORE1_SRC_DIR}/linker/*.ld
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lcd/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lcd/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dualcore/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app/app.h
)
list(APPEND CORE1_DEPS ${CORE1_SRC_DIR}/CMakeLists.txt)

set_source_files_properties(${CORE1_IMG_SRC} PROPERTIES GENERATED TRUE)
add_custom_command(
    OUTPUT ${CORE1_IMG_SRC}
    # core1 独立配置(仅首次)+ 编译(独立 build 目录, 不动 core0 的 build.ninja)。
    # HPM_SDK_BASE 必须显式传: 构建阶段(ninja)继承的是启动 ninja 时的环境,
    # 不是本文件里 set(ENV{HPM_SDK_BASE} ...) 改的那个进程内环境。
    COMMAND ${CMAKE_COMMAND} -E env HPM_SDK_BASE=${HPM_SDK_BASE}
            ${CMAKE_COMMAND}
            -DSRC=${CORE1_SRC_DIR} -DBUILD=${CORE1_BUILD_DIR}
            -DGEN=${CMAKE_GENERATOR} -DBOARD=${BOARD}
            -DCTYPE=${CMAKE_BUILD_TYPE}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/core1_build.cmake
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/core1_img2c.py
            ${CORE1_IMG_BIN} ${CORE1_IMG_SRC} sec_core_img
    DEPENDS ${CORE1_DEPS}
    COMMENT "构建 core1(显示核/LVGL) 镜像并生成 sec_core_img.c"
    VERBATIM
)
add_custom_target(core1_image DEPENDS ${CORE1_IMG_SRC})
add_dependencies(app core1_image)

# 把 core1 的镜像钉在它自己的 flash 地址上(和 core1 的链接地址必须一致!)
sdk_ld_options(-Wl,--section-start=.core1_img=${CORE1_IMG_BASE})
sdk_app_src(${CORE1_IMG_SRC})

# ---- 双核: core0 侧的启动与协作 (检查镜像头/释放 core1/遥测/日志转发) ----
sdk_inc(src/dualcore)
sdk_app_src(src/dualcore/core1_boot.c)

sdk_app_src(src/main.c)
generate_ide_projects()
