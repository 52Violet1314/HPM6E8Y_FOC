#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解码从共享内存 dump 出来的 core1 日志环。

为什么要这样看日志: USB CDC 控制台挂了之后, core1 的日志(包括每 5 秒一行的
"LCD 5s: flush=... pix=... busy=...")会堆在共享内存的环里没人取。直接用调试器
把环 dump 出来解码, 就绕开了串口。

环的布局见 src/dualcore/shm_core.h:
    log[] 起点 = 共享区基址 + 0x4C
    shm_log_slot_t = { uint32_t state; char text[192]; }  -> 每个槽 196 字节
"""
import os
import struct
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
BIN = os.path.join(ROOT, 'tmp', 'shm_log.bin')
SLOTS, LINE = 32, 192
SLOT_SZ = 4 + LINE

data = open(BIN, 'rb').read()
assert len(data) >= SLOTS * SLOT_SZ, 'dump 太短: %d' % len(data)

lines = []
for i in range(SLOTS):
    off = i * SLOT_SZ
    state = struct.unpack_from('<I', data, off)[0]
    raw = data[off + 4:off + 4 + LINE]
    txt = raw.split(b'\x00', 1)[0].decode('utf-8', 'replace')
    lines.append((i, state, txt))

used = [x for x in lines if x[1] == 1 and x[2].strip()]
print('槽位: %d 个, 其中有效 %d 行' % (SLOTS, len(used)))
print()

# 按"环顺序"打印: 全满时以第一个有效槽为最旧
first = used[0][0] if used else 0
order = sorted(used, key=lambda x: (x[0] - first) % SLOTS)
for n, (i, st, txt) in enumerate(order, 1):
    print('%2d [slot %2d] %s' % (n, i, txt))

print()
print('=== 只挑 LCD / 统计行 ===')
for i, st, txt in order:
    if ('LCD' in txt) or ('flush' in txt) or ('5s' in txt):
        print('  %s' % txt)
