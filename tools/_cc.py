#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 compile_commands.json 的原始参数编译【全部工程源码】(src/ 下),
然后可以手工链接 (tools/_link.ps1)。沙箱里 cmake/ninja 起不来, 用它代替增量构建。

用法: python tools/_cc.py [只编译文件名里含该子串的文件...]
      python tools/_cc.py --sdk <子串>   连本地化 SDK 里的文件一起编译
      (改过 hpm_sdk_localized_for_hpm6e00YZ 下的文件时必须用 --sdk, 否则
       只会链接到旧的 SDK 目标文件, 补丁静默失效 —— 踩过这个坑)
"""
import json
import os
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ROOT = os.path.abspath('.')
db = json.load(open(os.path.join(ROOT, 'build', 'compile_commands.json'), encoding='utf-8'))

args = sys.argv[1:]
with_sdk = False
if args and args[0] == '--sdk':
    with_sdk = True
    args = args[1:]

# 工程源码 = 仓库内、且不在本地化 SDK 里的文件。
# 【为什么不再只认 src/】双核之后多了一个"生成文件"也要编:
#   build/generated/sec_core_img.c (core1 的镜像数组, 由 CMake 的自定义命令生成)。
# 它在 compile_commands.json 里是 core0 的正常源文件, 但路径里没有 /src/, 只认
# src/ 的话它永远编不到 -> 链接时报 undefined reference to `sec_core_img`。
SDK = os.path.normcase(os.path.join(ROOT, 'hpm_sdk_localized_for_hpm6e00YZ'))
targets = []
for e in db:
    f = os.path.normcase(os.path.normpath(e['file']))
    if not f.startswith(os.path.normcase(ROOT)):
        continue
    if with_sdk:
        # --sdk: 连 SDK 里的文件一起编 (靠后面的过滤器收窄)
        targets.append(e)
        continue
    if f.startswith(SDK):
        continue
    targets.append(e)

filt = args
if filt:
    targets = [e for e in targets if any(k.lower() in e['file'].lower() for k in filt)]

print('要编译 %d 个工程源文件' % len(targets))
ok = True
for e in sorted(targets, key=lambda x: x['file']):
    rel = os.path.relpath(e['file'], ROOT)
    p = subprocess.run(e['command'], shell=True, cwd=e['directory'],
                       capture_output=True, text=True)
    out = ((p.stdout or '') + (p.stderr or '')).strip()
    if p.returncode != 0:
        ok = False
        print('FAILED  %s' % rel)
        print('\n'.join('    ' + l for l in out.splitlines()[:30]))
    elif out:
        print('warn    %s' % rel)
        print('\n'.join('    ' + l for l in out.splitlines()[:10]))

print('\n%s' % ('全部通过' if ok else '有失败'))
sys.exit(0 if ok else 1)
