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

sdk_app_src(src/main.c)
generate_ide_projects()
