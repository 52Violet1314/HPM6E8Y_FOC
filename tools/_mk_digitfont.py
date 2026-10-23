#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 LVGL 自带的 montserrat_40 里抽出数字字形, 生成一个【等宽数字字体】。

为什么要这个文件:
  src/lcd/lcd_ui.c 把大字转速拆成 4 个 28px 定宽的格子。但 montserrat 是**比例字体**,
  每个数字的字形步进都不一样(实测):
      '-' 15.31px   '1' 14.81px   '2' 22.94px   '3' 22.88px   '4' 26.75px
      '5' 22.94px   '6' 24.69px   '7' 23.94px   '8' 25.75px   '9' 24.69px
  最宽 26.75 / 最窄 14.81, 差 11.94px。把这样一个字形右对齐塞进 28px 的格子里, 墨迹
  的左边沿会随字形在 ~12px 范围内乱跳 —— 每变一位, 这一位的数字就横向"跳"一下,
  看起来就是抖/花。这不是撕裂, 但和撕裂一样难看, 而且每次改屏都会发生。

本脚本做的事(不改字形, 只改"步进 + 水平定位"):
  1. 按 cmap 把 '-' '.' '/' '0'..'9' 的 glyph_id 找出来(montserrat_40 的 ASCII cmap 是
     FORMAT0_TINY: range_start=32, glyph_id_start=1 -> gid = 1 + (ord(ch) - 32));
  2. 原样复制这些字形的位图字节(bpp=4, 不重新编码 -> 形状与原来逐位相同, 零风险);
  3. 把每个数字的 adv_w 统一成 28px(=448, 单位 1/16px), 并把 ofs_x 改成
     (28 - box_w)/2, 让墨迹在每个 28px 格子里**居中**;
  4. '-' '.' '/' 保持窄步进(它们不是等宽数字的一部分)。

结果: 4 个 28px 的格子里, 每个数字的墨迹都落在同一个位置, 改一位只影响这一位。
line_height / base_line / ofs_y 全部保持原值 -> 垂直位置和原来一模一样, 布局不动。
"""
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC_FONT = os.path.join(ROOT, 'hpm_sdk_localized_for_hpm6e00YZ', 'middleware', 'lvgl',
                        'src', 'font', 'lv_font_montserrat_40.c')
OUT = os.path.join(ROOT, 'src', 'lcd', 'lv_font_digits_40.c')

CELL_PX = 28            # 与 lcd_ui.c 的 UI_RPM_CELL_W 一致
DIGIT_ADV = CELL_PX * 16  # adv_w 单位是 1/16 px

CHARS = [chr(c) for c in range(0x2D, 0x3A)]   # '-' '.' '/' '0'..'9' -> 13 个, 连续
assert len(CHARS) == 13


def parse(src):
    m = re.search(r'glyph_bitmap\[\] = \{(.*?)\n\};', src, re.S)
    if not m:
        sys.exit('glyph_bitmap not found')
    bitmap = bytes(int(x, 16) for x in re.findall(r'0x([0-9a-fA-F]{1,2})', m.group(1)))

    m = re.search(r'static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc\[\] = \{(.*?)\n\};',
                  src, re.S)
    if not m:
        sys.exit('glyph_dsc not found')
    dsc = [dict(zip(('bitmap_index', 'adv_w', 'box_w', 'box_h', 'ofs_x', 'ofs_y'),
                    (int(a), int(b), int(c), int(d), int(e), int(f))))
           for a, b, c, d, e, f in re.findall(
               r'\{\s*\.bitmap_index\s*=\s*(\d+)\s*,\s*\.adv_w\s*=\s*(\d+)\s*,\s*'
               r'\.box_w\s*=\s*(\d+)\s*,\s*\.box_h\s*=\s*(\d+)\s*,\s*'
               r'\.ofs_x\s*=\s*(-?\d+)\s*,\s*\.ofs_y\s*=\s*(-?\d+)\s*\}', m.group(1))]

    line_height = int(re.search(r'\.line_height = (\d+)', src).group(1))
    base_line = int(re.search(r'\.base_line = (\d+)', src).group(1))
    return bitmap, dsc, line_height, base_line


def main():
    src = open(SRC_FONT, encoding='utf-8', errors='replace').read()
    bitmap, dsc, line_height, base_line = parse(src)

    # ---- 1. 取出需要的字形(按 cmap FORMAT0_TINY: gid = 1 + ord - 32) ----
    picked = []
    for ch in CHARS:
        gid = 1 + (ord(ch) - 32)
        g = dict(dsc[gid])
        g['ch'] = ch
        g['gid'] = gid
        # 该字形在原文件里的字节范围(下一个字形的起点就是它的终点)
        nxt = dsc[gid + 1]['bitmap_index']
        if nxt < g['bitmap_index']:
            sys.exit('bitmap_index not monotonic at gid %d' % gid)
        g['raw'] = bitmap[g['bitmap_index']:nxt]
        if not g['raw'] and (g['box_w'] * g['box_h']) > 0:
            sys.exit('empty bitmap for %r' % ch)
        picked.append(g)

    # ---- 2. 等宽化: 数字统一 28px 步进, 墨迹居中; 符号保持原步进 ----
    out_bmp = bytearray()
    out_dsc = []
    report = []
    for g in picked:
        is_digit = g['ch'].isdigit()
        if is_digit:
            adv = DIGIT_ADV
            ofs_x = (CELL_PX - g['box_w']) // 2
        else:
            adv = g['adv_w']
            ofs_x = g['ofs_x']

        out_dsc.append({'bitmap_index': len(out_bmp), 'adv_w': adv,
                        'box_w': g['box_w'], 'box_h': g['box_h'],
                        'ofs_x': ofs_x, 'ofs_y': g['ofs_y'], 'ch': g['ch']})
        out_bmp += g['raw']
        report.append((g['ch'], g['adv_w'] / 16.0, adv / 16.0,
                       g['ofs_x'], ofs_x, g['box_w']))

    # ---- 3. 生成 C 文件 ----
    L = []
    L.append('/*')
    L.append(' * Copyright (c) 2026')
    L.append(' * SPDX-License-Identifier: BSD-3-Clause')
    L.append(' *')
    L.append(' * 【自动生成, 不要手改】由 tools/_mk_digitfont.py 生成。')
    L.append(' *')
    L.append(' * 等宽数字字体(位图取自 LVGL 自带的 lv_font_montserrat_40, 形状逐位相同):')
    L.append(' *   - 所有数字 0~9 步进统一 %dpx, 墨迹在格子里居中;' % CELL_PX)
    L.append(' *   - line_height / base_line / ofs_y 与原字体一致 -> 垂直位置不变;')
    L.append(' *   - 只含 "-" "." "/" "0"~"9" 共 %d 个字形, 约 %d 字节位图。'
             % (len(picked), len(out_bmp)))
    L.append(' *')
    L.append(' * Size: 40 px           (字形高度取自 montserrat_40)')
    L.append(' * Advance: %d px         (所有数字统一等宽)' % CELL_PX)
    L.append(' *')
    L.append(' * 为什么需要它: 比例字体(montserrat)的数字步进 14.81~26.75px 不等, 塞进')
    L.append(' * 定宽格子后墨迹左右乱跳, 每改一位数字就横向抖一下。等宽之后改一位只动这一位。')
    L.append(' */')
    L.append('')
    L.append('#include "lvgl.h"')
    L.append('')
    L.append('/* 【本文件不依赖 LV_FONT_MONTSERRAT_40】位图是生成时拷进来的, 不是运行时引用。')
    L.append(' * 所以现在界面已经不用 montserrat_40 了, 可以把 lv_conf.h 里的')
    L.append(' * LV_FONT_MONTSERRAT_40 改成 0 省下那份字体的 flash —— 本文件照样编译。 */')
    L.append('')
    L.append('/*---------------------')
    L.append(' *    BITMAPS')
    L.append(' *---------------------*/')
    L.append('')
    L.append('static const uint8_t glyph_bitmap[] = {')
    line = '    '
    for i, byte in enumerate(out_bmp):
        frag = '0x%02x,' % byte
        if len(line) + len(frag) + 1 > 100:
            L.append(line.rstrip())
            line = '    '
        line += frag + ' '
        if (i % 16) == 15 and len(line) > 60:
            L.append(line.rstrip())
            line = '    '
    if line.strip():
        L.append(line.rstrip())
    L.append('};')
    L.append('')
    L.append('/*---------------------')
    L.append(' *    GLYPH DESCRIPTORS')
    L.append(' *---------------------*/')
    L.append('')
    L.append('static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {')
    L.append('    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, '
             '.ofs_x = 0, .ofs_y = 0}, /* id = 0 reserved */')
    for g in out_dsc:
        L.append('    {.bitmap_index = %d, .adv_w = %d, .box_w = %d, .box_h = %d, '
                 '.ofs_x = %d, .ofs_y = %d},/* U+%04X "%s" */'
                 % (g['bitmap_index'], g['adv_w'], g['box_w'], g['box_h'],
                    g['ofs_x'], g['ofs_y'], ord(g['ch']),
                    g['ch'].replace('\\', '\\\\')))
    L.append('};')
    L.append('')
    L.append('/*---------------------')
    L.append(' *    CHARACTER MAPPING')
    L.append(' *---------------------*/')
    L.append('')
    L.append('/* 0x2D..0x39 连续 13 个码点, FORMAT0_TINY: gid = 1 + (code - 0x2D) */')
    L.append('static const lv_font_fmt_txt_cmap_t cmaps[] = {')
    L.append('    {')
    L.append('        .range_start = %d, .range_length = %d, .glyph_id_start = 1,'
             % (ord(CHARS[0]), len(CHARS)))
    L.append('        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0,')
    L.append('        .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY')
    L.append('    }')
    L.append('};')
    L.append('')
    L.append('/*--------------------')
    L.append(' *  ALL CUSTOM DATA')
    L.append(' *--------------------*/')
    L.append('')
    L.append('static const lv_font_fmt_txt_dsc_t font_dsc = {')
    L.append('    .glyph_bitmap = glyph_bitmap,')
    L.append('    .glyph_dsc = glyph_dsc,')
    L.append('    .cmaps = cmaps,')
    L.append('    .kern_dsc = NULL,      /* 等宽数字不需要字距调整 */')
    L.append('    .kern_scale = 0,')
    L.append('    .cmap_num = 1,')
    L.append('    .bpp = 4,              /* 与原字体一致 */')
    L.append('    .kern_classes = 0,')
    L.append('    .bitmap_format = 0,    /* LV_FONT_FMT_TXT_PLAIN */')
    L.append('};')
    L.append('')
    L.append('/*-----------------')
    L.append(' *  PUBLIC FONT')
    L.append(' *----------------*/')
    L.append('')
    L.append('const lv_font_t lv_font_digits_40 = {')
    L.append('    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,')
    L.append('    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,')
    L.append('    .line_height = %d,' % line_height)
    L.append('    .base_line = %d,' % base_line)
    L.append('    .subpx = LV_FONT_SUBPX_NONE,')
    L.append('    .underline_position = -3,')
    L.append('    .underline_thickness = 2,')
    L.append('    .dsc = &font_dsc')
    L.append('};')
    L.append('')
    L.append('')

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(L))

    print('写入 %s' % os.path.relpath(OUT, ROOT))
    print('  字形 %d 个, 位图 %d 字节, line_height=%d base_line=%d'
          % (len(out_dsc), len(out_bmp), line_height, base_line))
    print()
    print('  字符   原步进     新步进   原ofs_x -> 新ofs_x   box_w')
    for ch, w_old, w_new, ox_old, ox_new, bw in report:
        print('   %s    %6.2f   %6.2f      %2d  ->  %2d        %d'
              % (ch, w_old, w_new, ox_old, ox_new, bw))
    print()
    mx = max(r[5] for r in report if r[0].isdigit())
    print('  最宽数字墨迹 %dpx <= 格子 %dpx -> 不会被裁' % (mx, CELL_PX))
    if mx > CELL_PX:
        sys.exit('!!! 数字比格子宽, 会被裁')


if __name__ == '__main__':
    main()
