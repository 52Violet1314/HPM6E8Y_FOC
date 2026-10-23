#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查大字转速后面的 "RPM" 单位标签: 量几何关系 + 渲染改前/改后对比图。

为什么单独查它: 数字被拆成 4 个定宽格子之后, 数字的"盒子"变了, 而单位标签是另一
个独立对象、位置写死 —— 两者是不是重叠、基线是不是对齐、位数变化时距离会不会变,
都必须实测, 不能靠看代码。

判据(全部来自字体文件的真实度量 + lcd_ui.c 的真实坐标):
  1) 盒子不重叠           -> 刷数字不会碰到单位(单位不可能被撕/被抹);
  2) 基线一致             -> 单位不会比数字高/低几像素(看着"没对齐");
  3) 数字右端到单位的距离恒定 -> 单位不会随位数"飘"走。

跑法: python tools/_unit_check.py
输出: tools/lcd_sim/out/unit_before_after.png
"""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'lcd_sim'))
from lvfont import LvFont                                    # noqa: E402
from PIL import Image, ImageDraw                             # noqa: E402

FONTDIR = os.path.join(ROOT, 'hpm_sdk_localized_for_hpm6e00YZ', 'middleware', 'lvgl',
                       'src', 'font')
DIGI = os.path.join(ROOT, 'src', 'lcd', 'lv_font_digits_40.c')
OUT = os.path.join(ROOT, 'tools', 'lcd_sim', 'out', 'unit_before_after.png')

CELLS, CELL_W = 4, 28
DIGIT_X0, DIGIT_Y0 = 12, 30
BG = (0x0E, 0x16, 0x21)
FG_DIGIT = (0x35, 0xC8, 0xF0)
FG_UNIT = (0x8F, 0xA3, 0xB4)

# 改前 / 改后: (数字组是否右对齐, 单位 x, 单位 y)
BEFORE = dict(right=False, unit_x=150, unit_y=50)
AFTER = dict(right=True, unit_x=132, unit_y=48)


def cell_box(i, right_align, n):
    """第 i 个格子的盒子。right_align=True 时数字占最后 n 个格子。"""
    x = DIGIT_X0 + i * CELL_W
    return x, DIGIT_Y0, x + CELL_W - 1, DIGIT_Y0 + 43


def draw_text(font, img, px, text, box_x, box_y, color, cell_align=None):
    line_w = font.text_width(text)
    pen = box_x
    if cell_align == 'right':
        pen = box_x + (CELL_W - line_w)
    def put(x, y, c, a):
        if 0 <= x < img.width and 0 <= y < img.height:
            o = px[x, y]
            px[x, y] = tuple((o[k] * (255 - a) + color[k] * a) // 255 for k in range(3))
    font.draw(put, box_y, pen, text, color)


def render(digfont, unitfont, value, style, w=210, h=56, ox=0, oy=24):
    """渲染 x 0..210 / y 24..80 这一条(大字 + 单位)。"""
    img = Image.new('RGB', (w, h), BG)
    px = img.load()
    s = str(value)
    n = len(s)
    for i in range(CELLS):
        if style['right']:
            src = i - (CELLS - n)
            ch = s[src] if src >= 0 else None
        else:
            ch = s[i] if i < n else None
        if ch is None:
            continue
        cx = DIGIT_X0 + i * CELL_W - ox
        draw_text(digfont, img, px, ch, cx, DIGIT_Y0 - oy, FG_DIGIT)
    draw_text(unitfont, img, px, 'RPM', style['unit_x'] - ox,
              style['unit_y'] - oy, FG_UNIT)
    return img


def main():
    m20 = LvFont(os.path.join(FONTDIR, 'lv_font_montserrat_20.c'))
    dig = LvFont(DIGI, 'digits_40')

    print('=== 字体度量 ===')
    print('  digits_40(数字)      line_height=%d base_line=%d' % (dig.line_height, dig.base_line))
    print('  montserrat_20(单位)  line_height=%d base_line=%d' % (m20.line_height, m20.base_line))

    uw = m20.text_width('RPM')
    print()
    print('=== "RPM" 单位 ===')
    print('  文本宽 %dpx' % uw)

    print()
    print('=== 逐项判据 ===')
    ok = True

    # 1) 重叠
    dgroup = (DIGIT_X0, DIGIT_Y0, DIGIT_X0 + CELLS * CELL_W - 1, DIGIT_Y0 + dig.line_height - 1)
    for nm, st in (('改前', BEFORE), ('改后', AFTER)):
        ub = (st['unit_x'], st['unit_y'], st['unit_x'] + uw - 1, st['unit_y'] + m20.line_height - 1)
        ov = not (ub[2] < dgroup[0] or ub[0] > dgroup[2] or
                  ub[3] < dgroup[1] or ub[1] > dgroup[3])
        print('  %s: 单位盒子 (%d,%d)-(%d,%d), 数字组 (%d,%d)-(%d,%d) -> %s'
              % (nm, ub[0], ub[1], ub[2], ub[3], dgroup[0], dgroup[1], dgroup[2], dgroup[3],
                 '重叠!' if ov else '不重叠 (净空 %dpx)' % (ub[0] - dgroup[2] - 1)))
        if ov:
            ok = False

    # 2) 基线
    d_base = DIGIT_Y0 + dig.line_height - dig.base_line
    print('  数字基线 y = %d' % d_base)
    for nm, st in (('改前', BEFORE), ('改后', AFTER)):
        u_base = st['unit_y'] + m20.line_height - m20.base_line
        flag = '' if u_base == d_base else '   <-- 没对齐'
        print('  %s: 单位基线 y=%d -> 差 %+d px%s' % (nm, u_base, u_base - d_base, flag))
        if nm == '改后':
            assert u_base == d_base, '改后基线必须对齐'

    # 3) 数字右端到单位的距离
    #    判据看【最后一个格子】的右边界(它决定视觉上的"数字宽度"), 不是看墨迹:
    #    '-' 的墨迹比数字窄一点, 墨迹会差 1px, 那是字形本身的宽度差, 不是布局在飘。
    print('  数字右端到单位的距离(单位起点 %d):' % AFTER['unit_x'])
    for nm, st in (('改前', BEFORE), ('改后', AFTER)):
        cell_d, ink_d = [], []
        for v in (5, 42, 320, -999):
            s = str(v)
            n = len(s)
            last = CELLS - 1 if st['right'] else n - 1
            cell_right = DIGIT_X0 + last * CELL_W + CELL_W - 1
            g = dig.glyph[dig.gid(s[-1])]
            ink_right = DIGIT_X0 + last * CELL_W + g.ox + g.w - 1
            cell_d.append(st['unit_x'] - cell_right - 1)
            ink_d.append(st['unit_x'] - ink_right - 1)
        print('    %s: 按格子边界 %s %s; 按墨迹 %s (差 1px 是 "-" 比数字窄)'
              % (nm, cell_d, '恒定' if len(set(cell_d)) == 1 else '随位数变化', ink_d))
        if nm == '改后':
            assert len(set(cell_d)) == 1, '改后数字右边界必须恒定'
            assert max(ink_d) - min(ink_d) <= 1

    print()
    print('判据: %s' % ('全部满足' if ok else '仍有不满足项'))

    # ---- 对比图 ----
    vals = (0, 42, 320, -999)
    imgs = []
    for v in vals:
        for st in (BEFORE, AFTER):
            im = render(dig, m20, v, st)
            imgs.append(im.resize((im.width * 3, im.height * 3), Image.NEAREST))
    pad = 8
    W = imgs[0].width * 2 + pad * 3
    H = (imgs[0].height + pad) * len(vals) + pad
    canvas = Image.new('RGB', (W, H), (0x20, 0x20, 0x28))
    d = ImageDraw.Draw(canvas)
    y = pad
    for i, v in enumerate(vals):
        a, b = imgs[i * 2], imgs[i * 2 + 1]
        canvas.paste(a, (pad, y))
        canvas.paste(b, (pad * 2 + a.width, y))
        d.text((pad + 6, y + 6), 'BEFORE %d' % v, fill=(255, 120, 120))
        d.text((pad * 2 + a.width + 6, y + 6), 'AFTER %d' % v, fill=(120, 255, 140))
        y += a.height + pad
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    canvas.save(OUT)
    print('对比图: %s  (%dx%d)' % (os.path.relpath(OUT, ROOT), canvas.width, canvas.height))

    # ---- ASCII 自检: 改后 320 的那条 ----
    print()
    print('改后 "320 RPM" 的实际像素(每字符 1px, . = 背景, # = 数字, + = 单位):')
    img = render(dig, m20, 320, AFTER)
    col = {(FG_DIGIT): '#', (FG_UNIT): '+'}
    for yy in range(img.height):
        row = ''
        for xx in range(img.width):
            p = img.load()[xx, yy]
            if p == BG:
                row += '.'
            elif abs(p[0] - FG_UNIT[0]) + abs(p[1] - FG_UNIT[1]) + abs(p[2] - FG_UNIT[2]) < \
                    abs(p[0] - FG_DIGIT[0]) + abs(p[1] - FG_DIGIT[1]) + abs(p[2] - FG_DIGIT[2]):
                row += '+'
            else:
                row += '#'
        print('   ' + row)


if __name__ == '__main__':
    main()
