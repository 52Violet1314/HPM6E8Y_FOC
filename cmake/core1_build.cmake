# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
#
# 脚本模式 (cmake -P): 配置(仅在需要时)并编译 core1 工程
#
# 【为什么不在根 CMakeLists 的 add_custom_command 里直接写两次 cmake】
#   每次构建都重跑一次 core1 的 configure 会重写 build/core1/build.ninja, 让里面
#   那个 ninja 每次都去重算/压实自己的日志 —— 在 Windows 上实测偶发
#   "ninja: error: failed recompaction: Permission denied"(文件被占用)。
#   第一次之后就没必要再显式 configure 了: cmake --build 发现 core1 的
#   CMakeLists 有改动时自己会重跑(cmake 的自带规则), 行为完全一致但少写一遍文件。
#
# 参数:
#   -DSRC    : core1 源码目录
#   -DBUILD  : core1 构建目录
#   -DGEN    : 生成器(与父工程一致, 一般是 Ninja)
#   -DBOARD  : 板名
#   -DCTYPE  : CMAKE_BUILD_TYPE
#   (HPM_SDK_BASE 走环境变量: 由调用方的 cmake -E env 传进来)

foreach(v SRC BUILD GEN BOARD CTYPE)
    if(NOT DEFINED ${v})
        message(FATAL_ERROR "core1_build.cmake: 缺少参数 ${v}")
    endif()
endforeach()

if(NOT EXISTS "${BUILD}/CMakeCache.txt")
    message(STATUS "core1: first-time configure -> ${BUILD}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -S "${SRC}" -B "${BUILD}"
                -G "${GEN}" -DBOARD=${BOARD} -DCMAKE_BUILD_TYPE=${CTYPE}
        RESULT_VARIABLE rc
    )
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "core1: configure failed (rc=${rc})")
    endif()
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${BUILD}"
    RESULT_VARIABLE rc
)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "core1: build failed (rc=${rc})")
endif()
