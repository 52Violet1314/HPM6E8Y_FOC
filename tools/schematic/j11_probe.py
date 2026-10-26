#!/usr/bin/env python3
"""排查用探针: 把原理图 PDF 第 2 页上 J11 附近的文字连坐标一起打出来。

依赖:  pip install pymupdf
用法:  python tools/schematic/j11_probe.py [原理图.pdf]

用来核对 docs/hardware/J11_pinout.md 的表 —— 当网表里出现没见过的网络名时,
先跑这个看它到底在 J11 符号的哪一侧。
"""
import os
import sys

import fitz

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PDF = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    REPO, "docs", "hardware", "HPM6EMotorV20.pdf")

doc = fitz.open(PDF)
page = doc[1]                      # PAGE 2
words = page.get_text("words")

print("=== PC2x / PFxx net-label positions on PAGE 2 ===")
for w in words:
    t = w[4].strip()
    if t in ("PC24", "PC25", "PC26", "PC27", "PC28", "PC29",
             "PF14", "PF16", "PF19", "PF21", "PF25", "PF26"):
        print(f"   {t:5s} x={w[0]:7.1f} y={w[1]:7.1f}")

print()
print("=== words around the J11 symbol (x 380..830, y 240..450) ===")
sel = [w for w in words
       if 380 <= (w[0] + w[2]) / 2 <= 830 and 240 <= (w[1] + w[3]) / 2 <= 450]
sel.sort(key=lambda w: (round(w[1], 1), w[0]))
for w in sel:
    print(f"   x={w[0]:7.1f} y={w[1]:7.1f}  {w[4]!r}")
