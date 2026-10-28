#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 main 工作区里现成的 build/ 复制到本 worktree, 并把其中的【绝对路径】
从 ...\\HPM6E8Y\\ 改写成 ...\\HPM6E8Y-display\\。

【为什么需要它】
    build/ 在 .gitignore 里, 所以 git worktree add 出来的
    C:\\...\\HPM6E8Y-display 里没有 build/ 目录; 而 tools/_cc.py 依赖
    build/compile_commands.json, build/_relink.bat / _postrun.bat 用来链接。
    这个环境里 cmake/ninja 起不来(会挂死), 没法重新 configure,
    所以只能"复制 + 改写路径"。

【为什么必须改写路径, 不能直接引用 main 的 build/】
    main 的 compile_commands.json 里每条 e['file'] / e['command'] 都是
    C:\\...\\HPM6E8Y\\src\\... 的绝对路径。若在 worktree 里直接用它,
    编译的会是 **main 工作区** 的源码(改动完全不生效), 而且 .obj / demo.elf
    也会覆盖写进 main 的 build/ —— 既拿不到证据, 又弄脏 main。
    改写之后: 源码、头文件、输出全部落在本 worktree 内。

用法(在 worktree 根目录):
    python tools/_setup_worktree_build.py [--src <main 工作区路径>]
"""
import argparse
import os
import re
import shutil
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ROOT = os.path.abspath('.')
DEFAULT_SRC = r'C:\Users\14420\Desktop\HPMicro\HPM6E8Y'

# 需要改写路径的文本文件(build.ninja 用不上, 但改了一致性更好;
# .ninja_log / .ninja_deps / *.obj / *.a 是二进制, 跳过)
TEXT_FILES = [
    'compile_commands.json',
    'build.ninja',
    '_link.bat',
    '_relink.bat',
    '_postrun.bat',
    'CMakeCache.txt',
]


def rewrite(path, old_root, new_root):
    with open(path, 'r', encoding='utf-8', errors='replace', newline='') as f:
        text = f.read()

    # 只改写"后面跟着路径分隔符或引号/行尾"的那个仓库根, 避免误伤
    # 形如 HPM6E8Y-display 的已有字符串(用 (?!-display) 挡住)。
    pat = re.compile(re.escape(old_root) + r'(?!-display)(?=[\\/"\'\s]|$)')
    text, n = pat.subn(lambda m: m.group(0) + '-display', text)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=DEFAULT_SRC,
                    help='main 工作区(含现成 build/)的路径')
    ap.add_argument('--no-copy', action='store_true',
                    help='跳过复制, 只重写路径(脚本可重复执行)')
    args = ap.parse_args()

    src_build = os.path.join(os.path.abspath(args.src), 'build')
    dst_build = os.path.join(ROOT, 'build')

    if not os.path.isdir(src_build):
        print('找不到 %s' % src_build)
        return 1
    if not os.path.normcase(ROOT).endswith('hpm6e8y-display'):
        print('【拒绝执行】当前目录 %s 看起来不是 worktree, '
              '本脚本只允许在 HPM6E8Y-display 里跑' % ROOT)
        return 1

    print('复制 %s -> %s' % (src_build, dst_build))
    if os.path.isdir(dst_build):
        if not args.no_copy:
            shutil.rmtree(dst_build, ignore_errors=True)
    if not args.no_copy:
        shutil.copytree(src_build, dst_build, symlinks=False, ignore_dangling_symlinks=True)

    old_root = os.path.abspath(args.src).replace('\\', '/')
    # JSON 里 Windows 路径被转义成双反斜杠(compile_commands.json 的 "file" 字段),
    # 所以正斜杠 / 单反斜杠 / 双反斜杠三种写法都要改。
    variants = [old_root, old_root.replace('/', '\\'), old_root.replace('/', '\\\\')]
    total = 0
    for rel in TEXT_FILES:
        p = os.path.join(dst_build, rel)
        if not os.path.isfile(p):
            continue
        n = 0
        for v in variants:
            if v == old_root:
                n += rewrite(p, v, ROOT.replace('\\', '/'))
            else:
                n += rewrite(p, v, v[:-len('HPM6E8Y')] + 'HPM6E8Y-display')
        total += n
        print('  改写 %-24s %d 处' % (rel, n))

    os.chdir(dst_build)
    print('\n共改写 %d 处绝对路径' % total)
    print('自检: build/ 里还有没有指向 main 的绝对路径')
    left = 0
    for rel in TEXT_FILES:
        p = os.path.join(dst_build, rel)
        if not os.path.isfile(p):
            continue
        with open(p, 'r', encoding='utf-8', errors='replace') as f:
            for i, line in enumerate(f, 1):
                for m in re.finditer(r'[A-Za-z]:[\\/][^\s"\']*HPM6E8Y[\\/]', line):
                    if '-display' not in m.group(0):
                        left += 1
                        if left <= 5:
                            print('  残留 %s:%d %s' % (rel, i, m.group(0)))
    print('残留条数: %d' % left)
    return 0


if __name__ == '__main__':
    sys.exit(main())
