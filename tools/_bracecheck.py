#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""粗略统计 C 文件的花括号平衡, 找出第一个不平衡的位置。
只用于定位 "expected declaration at end of input" 这类括号漏写的编译错误。
注意: 会忽略 // 注释和字符串字面量, 不处理 /* */ 与字符常量 —— 够用就行。"""
import re
import sys

path = sys.argv[1]
lines = open(path, encoding='utf-8').read().splitlines()

depth = 0
in_block = False
for i, line in enumerate(lines, 1):
    code = line
    if in_block:
        if '*/' in code:
            code = code.split('*/', 1)[1]
            in_block = False
        else:
            continue
    if '/*' in code:
        head, rest = code.split('/*', 1)
        if '*/' in rest:
            code = head + rest.split('*/', 1)[1]
        else:
            code = head
            in_block = True
    code = re.sub(r'"(?:[^"\\]|\\.)*"', '""', code)
    code = re.sub(r"'(?:[^'\\]|\\.)*'", "''", code)
    code = re.sub(r'//.*', '', code)

    d = code.count('{') - code.count('}')
    if d != 0:
        print('%5d  depth=%d  %s' % (i, depth + d, line.strip()[:100]))
    depth += d

print('final depth =', depth)
