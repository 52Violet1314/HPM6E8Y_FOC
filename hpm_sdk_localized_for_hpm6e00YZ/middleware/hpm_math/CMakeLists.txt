# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
#
# HPM Math middleware (software subset, GNU RISC-V toolchain)
#
# 说明:
#   上游完整版 hpm_math 还包含 Andes(nds) DSP 库 (nds_dsp/libdsp*.a),
#   只能在 Andes/nds-gcc 工具链下使用。本工程使用 GNU riscv32-unknown-elf
#   工具链, 因此这里只编译与工具链无关的软件数学库 (sw_dsp)：
#     - hpm_math_fast_trig_* : 查表 + 插值的快速 sin/cos (FOC 使用)
#     - hpm_software_cfft_float / 位反转 等
#   如需 DSP 加速版本, 请改用 Andes 工具链并参考 SDK 上游 CMakeLists。

sdk_inc(./)
sdk_src(sw_dsp/hpm_math_sw.c)
sdk_ld_options("-lm")
