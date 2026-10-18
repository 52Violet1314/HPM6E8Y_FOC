#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 core1 的 bin 转成 core0 的 C 数组 (段名 .core1_img)

用法: core1_img2c.py <core1.bin> <输出.c> [变量名]

【为什么不用 SDK 的 bin2c.py】
  两个额外要求:
    1) 数组必须放在 .core1_img 段 —— 根 CMakeLists 用
       -Wl,--section-start=.core1_img=0x80100000 把它钉在 core1 的 flash 地址上,
       core1 就在那里就地执行(XIP)。bin2c.py 只会生成默认段(.rodata)。
    2) sec_core_img 必须是段的【第一个字节】: core0 会用
       "(uint32_t)sec_core_img == 0x80100000" 做链接期自检, 所以尺寸变量
       sec_core_img_size 放到普通 .rodata 里, 不能排在数组前面。
"""
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

SECTION = ".core1_img"
MAX_BYTES = 1024 * 1024          # 与 core1/linker/core1_flash.ld 的 FLASH LENGTH 一致
PER_LINE = 16                    # 每行 16 字节, 可读性/编译速度折中


def fail(msg):
    sys.stderr.write("core1_img2c: %s\n" % msg)
    sys.exit(1)


def main():
    if len(sys.argv) < 3:
        fail("用法: core1_img2c.py <core1.bin> <输出.c> [变量名]")

    bin_path = sys.argv[1]
    out_path = sys.argv[2]
    var = sys.argv[3] if len(sys.argv) > 3 else "sec_core_img"

    try:
        with open(bin_path, "rb") as fh:
            data = fh.read()
    except OSError as exc:
        fail("读不到 %s: %s" % (bin_path, exc))

    if not data:
        fail("core1 的 bin 是空的: %s" % bin_path)
    if len(data) > MAX_BYTES:
        fail("core1 镜像 %d 字节, 超过 flash 预留的 %d 字节(1MB)" % (len(data), MAX_BYTES))
    if len(data) < 1024:
        fail("core1 镜像只有 %d 字节, 明显没链接成功" % len(data))

    out = []
    out.append("/*")
    out.append(" * 自动生成, 不要手改: %s" % var)
    out.append(" * 来源: core1/output/core1.bin (工具: cmake/core1_img2c.py)")
    out.append(" *")
    out.append(" * 这一段被链接到 flash 的 0x80100000 (见根 CMakeLists 的 --section-start),")
    out.append(" * 也就是 core1 就地执行(XIP)的代码; core0 只读它的镜像头并释放 core1。")
    out.append(" * 镜像大小: %d 字节" % len(data))
    out.append(" */")
    out.append("#include <stdint.h>")
    out.append("")
    # 数组必须排第一: core0 用它的地址做"是否落在 0x80100000"的链接期自检
    out.append('__attribute__((section("%s"), used, aligned(4)))' % SECTION)
    out.append("const uint8_t %s[%d] = {" % (var, len(data)))
    for i in range(0, len(data), PER_LINE):
        out.append("    " + "".join("0x%02x, " % b for b in data[i:i + PER_LINE]).rstrip())
    out.append("};")
    out.append("")
    # 尺寸变量放默认段(.rodata), 不要污染 .core1_img 的开头
    out.append('__attribute__((used))')
    out.append("const uint32_t %s_size = %dU;" % (var, len(data)))
    out.append("")

    with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(out))

    print("core1 镜像 %d 字节 -> %s (段 %s)" % (len(data), out_path, SECTION))


if __name__ == "__main__":
    main()
