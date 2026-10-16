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

# ---- LCD/LVGL: migrated from the fullfuncgui board example ----
sdk_inc(src/lcd)
sdk_app_src(src/lcd/LCDHw.c)
sdk_app_src(src/lcd/LCDTouch.c)
sdk_app_src(src/lcd/LCDGui.c)
sdk_app_src(src/lcd/lcd_app.c)
sdk_app_src(src/lcd/lcd_ui.c)

# ---- Hardware: 传感器/驱动器驱动 (纯协议层, 无硬件) ----
# MT6701 磁编码器帧解析/CRC
sdk_inc(src/Hardware/mt6701)
sdk_app_src(src/Hardware/mt6701/mt6701.c)

# ---- FOC: 磁场定向控制算法 ----
sdk_inc(src/FOC)
sdk_app_src(src/FOC/foc.c)
sdk_app_src(src/FOC/foc_math.c)
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
# 离线标定的 3200 组"编码器 raw -> 电角度"查表 (tools/enc_lut_build.py 生成)
sdk_app_src(src/app/app_enc_lut.c)
sdk_app_src(src/app/app_nv.c)

# ---- RTOS: 任务划分 (控制 / 日志 / 显示) ----
sdk_inc(src/rtos)
sdk_app_src(src/rtos/app_rtos.c)

sdk_app_src(src/main.c)
generate_ide_projects()
