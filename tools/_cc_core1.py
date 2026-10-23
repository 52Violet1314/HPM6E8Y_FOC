#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 build/core1/compile_commands.json 编译 **core1(显示核)** 的工程源码。

为什么需要它: 双核之后 src/lcd 下的文件(LVGL/ILI9341/界面)只编进 core1 的镜像,
它们在 core0 的 compile_commands.json 里根本不存在 —— 用 tools/_cc.py 验证 lcd_ui.c
的改动会得到"要编译 0 个文件/全部通过"这种假结果。

用法: python tools/_cc_core1.py [文件名子串...]
"""
import json
import os
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ROOT = os.path.abspath('.')
DB = os.path.join(ROOT, 'build', 'core1', 'compile_commands.json')
if not os.path.exists(DB):
    print('找不到 %s' % DB)
    sys.exit(2)

db = json.load(open(DB, encoding='utf-8'))
args = [a for a in sys.argv[1:]]

# 只编仓库自己的源码(不含本地化 SDK 里那一大堆)
SDK = os.path.normcase(os.path.join(ROOT, 'hpm_sdk_localized_for_hpm6e00YZ'))
targets = []
for e in db:
    f = os.path.normcase(os.path.normpath(e['file']))
    if not f.startswith(os.path.normcase(ROOT)) or f.startswith(SDK):
        continue
    targets.append(e)

if args:
    targets = [e for e in targets if any(k.lower() in e['file'].lower() for k in args)]

print('core1: 要编译 %d 个工程源文件' % len(targets))
ok = True
for e in sorted(targets, key=lambda x: x['file']):
    rel = os.path.relpath(e['file'], ROOT)
    p = subprocess.run(e['command'], shell=True, cwd=e['directory'],
                       capture_output=True, text=True, errors='replace')
    out = ((p.stdout or '') + (p.stderr or '')).strip()
    if p.returncode != 0:
        ok = False
        print('FAILED  %s' % rel)
        print('\n'.join('    ' + l for l in out.splitlines()[:40]))
    elif out:
        print('warn    %s' % rel)
        print('\n'.join('    ' + l for l in out.splitlines()[:10]))

print('\n%s' % ('core1 全部通过' if ok else 'core1 有失败'))
sys.exit(0 if ok else 1)
