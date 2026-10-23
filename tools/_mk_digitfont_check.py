#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证等宽数字字体: 把"改前(montserrat 比例字体 + 右对齐)"和"改后(等宽数字库)"
的真实排版都渲染出来, 逐位量出墨迹的左边沿, 并输出对比 PNG。

判据: 改前每个数字的墨迹左边沿在格子里到处跑(1 和 0 差 ~11px), 改后全部对齐到
同一列 -> 改一位数字时, 其它三位纹丝不动。

跑法: python tools/_mk_digitfont_check.py
输出: tools/lcd_sim/out/digits_before_after.png  (+ 终端里的逐位表)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lcd_sim'))
from lvfont import LvFont                                    # noqa: E402
from PIL import Image, ImageDraw                             # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
MONT = os.path.join(ROOT, 'hpm_sdk_localized_for_hpm6e00YZ', 'middleware', 'lvgl',
                    'src', 'font', 'lv_font_montserrat_40.c')
DIGI = os.path.join(ROOT, 'src', 'lcd', 'lv_font_digits_40.c')
OUT = os.path.join(ROOT, 'tools', 'lcd_sim', 'out', 'digits_before_after.png')

CELL_W = 28
CELLS = 4
X0 = 12          # lcd_ui.c: lv_obj_align(..., 12 + i*28, 30)
Y0 = 30
BG = (0x0E, 0x16, 0x21)
FG = (0x35, 0xC8, 0xF0)


def ink_box(font, ch, cell_x, mode):
    """返回该字符墨迹相对屏幕的 (左边沿, 中心), 与 LVGL 的画字公式一致。

    mode='before': 28px 定宽标签 + LV_TEXT_ALIGN_RIGHT -> pen = cell_x + (28 - 文本宽)
    mode='after' : 等宽字体, 文本宽就是 28             -> pen = cell_x
    """
    gid = font.gid(ch)
    if gid is None:
        return None, None
    g = font.glyph[gid]
    w = font.text_width(ch)
    off = CELL_W - w
    if off < 0:
        off = 0
    left = cell_x + off + g.ox
    return left, left + g.w / 2.0


def render(font, text, mode, w=320, h=240):
    img = Image.new('RGB', (w, h), BG)
    px = img.load()
    for i, ch in enumerate(text):
        if i >= CELLS:
            break
        cell_x = X0 + i * CELL_W
        pen = cell_x + (CELL_W - font.text_width(ch) if mode == 'before' else 0)
        if mode == 'before' and pen < cell_x:
            pen = cell_x
        gid = font.gid(ch)
        if gid is None:
            continue
        g = font.glyph[gid]
        a = font.alpha(gid)
        bx, by, _, _ = font.glyph_box(ch, pen << 4, Y0)
        for r in range(g.h):
            for c in range(g.w):
                al = a[r][c]
                if not al:
                    continue
                x, y = bx + c, by + r
                if 0 <= x < w and 0 <= y < h:
                    o = px[x, y]
                    px[x, y] = tuple((o[k] * (255 - al) + FG[k] * al) // 255 for k in range(3))
    return img


def main():
    before = LvFont(MONT, 'montserrat_40')
    after = LvFont(DIGI, 'digits_40')
    print('montserrat_40: line_height=%d base_line=%d' % (before.line_height, before.base_line))
    print('digits_40    : line_height=%d base_line=%d glyphs=%d'
          % (after.line_height, after.base_line, len(after.glyph)))
    assert (before.line_height, before.base_line) == (after.line_height, after.base_line), \
        '垂直度量必须一致, 否则布局会动'

    # ---- 1. 逐位量墨迹(相对各自格子的起点) ----
    print()
    print('每个数字在自己的 28px 格子里的墨迹位置:')
    print('  字符 |  改前: 左边沿/中心  |  改后: 左边沿/中心')
    jit_lb, jit_cb, jit_la, jit_ca = [], [], [], []
    for ch in '0123456789':
        lb, cb = ink_box(before, ch, 0, 'before')
        la, ca = ink_box(after, ch, 0, 'after')
        jit_lb.append(lb), jit_cb.append(cb), jit_la.append(la), jit_ca.append(ca)
        print('   %s   |   %3d px / %5.1f px   |   %3d px / %5.1f px'
              % (ch, lb, cb, la, ca))
    print()
    print('  改前 左边沿极差 %2d px, 中心极差 %.1f px   <- 墨迹在格子里乱跑'
          % (max(jit_lb) - min(jit_lb), max(jit_cb) - min(jit_cb)))
    print('  改后 左边沿极差 %2d px, 中心极差 %.1f px   <- 中心对齐(窄字形天生两边留白多)'
          % (max(jit_la) - min(jit_la), max(jit_ca) - min(jit_ca)))
    print()
    print('  【关键】改一位数字时, 这一位的墨迹中心会不会跳:')
    for a, b in (('0', '1'), ('8', '1')):
        _, cb0 = ink_box(before, a, 0, 'before')
        _, cb1 = ink_box(before, b, 0, 'before')
        _, ca0 = ink_box(after, a, 0, 'after')
        _, ca1 = ink_box(after, b, 0, 'after')
        print('    %s -> %s : 改前中心跳 %.1f px, 改后跳 %.1f px'
              % (a, b, abs(cb1 - cb0), abs(ca1 - ca0)))
    assert max(jit_ca) - min(jit_ca) <= 1.0, '等宽字体下墨迹中心应当对齐(<=1px)'

    # ---- 2. 渲染对比图 ----
    samples = ['320', '111', '100', '888']
    scale = 4
    cw, chh = 116, 50          # 只截数字那一条带
    imgs = []
    for s in samples:
        for mode, font in (('before', before), ('after', after)):
            im = render(font, s, mode).crop((X0, Y0, X0 + cw, Y0 + chh))
            imgs.append(im.resize((cw * scale, chh * scale), Image.NEAREST))

    pad = 8
    W = imgs[0].width * 2 + pad * 3
    H = (imgs[0].height + pad) * len(samples) + pad
    canvas = Image.new('RGB', (W, H), (0x20, 0x20, 0x28))
    d = ImageDraw.Draw(canvas)
    y = pad
    for i, s in enumerate(samples):
        a = imgs[i * 2]
        b = imgs[i * 2 + 1]
        canvas.paste(a, (pad, y))
        canvas.paste(b, (pad * 2 + a.width, y))
        d.text((pad + 4, y + 4), 'BEFORE %s' % s, fill=(255, 120, 120))
        d.text((pad * 2 + a.width + 4, y + 4), 'AFTER %s' % s, fill=(120, 255, 140))
        y += a.height + pad

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    canvas.save(OUT)
    print()
    print('对比图: %s  (%dx%d)' % (os.path.relpath(OUT, ROOT), canvas.width, canvas.height))

    # ---- 3. ASCII 自检: 改后 "100" 的每一位应该落在同一列 ----
    print()
    print('改后 "100" 的墨迹列(每个 # 是 1px, | 是格子边界):')
    img = render(after, '100', 'after')
    px = img.load()
    for yy in range(Y0, Y0 + 44):
        row = ''
        for xx in range(X0, X0 + CELLS * CELL_W):
            if (xx - X0) % CELL_W == 0:
                row += '|'
            row += '#' if px[xx, yy] != BG else ' '
        print('   ' + row)


if __name__ == '__main__':
    main()
