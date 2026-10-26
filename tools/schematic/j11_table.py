#!/usr/bin/env python3
"""从原理图 PDF 里按坐标提取 J11 扩展排针的引脚号 / 网络名。

依赖:  pip install pymupdf
用法:  python tools/schematic/j11_table.py [原理图.pdf]
       默认读 docs/hardware/HPM6EMotorV20.pdf 的第 2 页（PAGE 2）。

输出即 docs/hardware/J11_pinout.md 里那张表 —— 手册里的表是"按 x 坐标分列"
提取的，不是按纯文本顺序猜的，所以脚号和网络名不会串行。

两个已知的 PDF 排版坑，脚本里已经处理：
  1) 右侧网络名后面粘着一个上标 "2"（原理图里的备用功能标记），PyMuPDF 把
     它们并成一个 word（如 "PD18"+"2" -> "PD182"，x1 由 658.4 伸到 661~662），
     所以按"右边界越过 658.8 且形如 P[A-Z]数字"就把最后一位去掉；
  2) 同一行会被切成两个 y 桶，其中一桶只剩孤立的数字，跳过空行即可。
"""
import os
import re
import sys
from collections import defaultdict

import fitz

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PDF = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    REPO, "docs", "hardware", "HPM6EMotorV20.pdf")

NET_RE = re.compile(r"^P[A-Z]\d{1,2}$")
BLEED_X1 = 658.8        # 右侧网络名(不含上标标记)的右边界约 658.4

page = fitz.open(PDF)[1]                 # PAGE 2
words = page.get_text("words")

# J11 symbol bbox: pin numbers x≈594(left/odd) & x≈607(right/even),
# net labels x~545-560 (left) and x~640-665 (right); rows y 268..435
sel = [w for w in words if 500 <= (w[0] + w[2]) / 2 <= 700 and 266 <= (w[1] + w[3]) / 2 <= 436]

rows = defaultdict(list)
for w in sel:
    rows[round(w[1] / 3.0)].append(w)    # cluster by y (3pt buckets)

table = []
for key in sorted(rows):
    row = sorted(rows[key], key=lambda w: w[0])
    odd_num = odd_net = even_num = even_net = None
    for w in row:
        t = w[4].strip()
        cx = (w[0] + w[2]) / 2
        if t.isdigit():
            if 585 <= cx < 606 and odd_num is None:     # 奇数脚号列
                odd_num = t
            elif cx >= 606 and even_num is None:        # 偶数脚号列
                even_num = t
            continue                                    # 数字不属于网络名
        if t.startswith("PI"):                          # 引脚类型标注, 如 PIN?
            continue
        if w[2] >= BLEED_X1 and re.match(r"^P[A-Z]\d+$", t):
            t = t[:-1]                                  # 去掉粘上来的上标 "2"
        if cx < 585 and odd_net is None:
            odd_net = t
        elif cx >= 606 and even_net is None:
            even_net = t
    table.append((odd_num, odd_net, even_num, even_net))

print("  odd#  net(odd)        even#  net(even)")
print("  " + "-" * 46)
for a, b, c, d in table:
    if a or c:
        print(f"  {str(a):>4}  {str(b):<14} {str(c):>5}  {d}")
