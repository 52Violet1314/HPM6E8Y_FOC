#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
校验生成的 3200 组查表数据, 并画成 PNG 便于肉眼确认。

    python tools/enc_lut_check.py [--lut tools/out_lut/enc_elec_lut_3200.h]
                                  [--out tools/out_lut/enc_elec_lut_3200.png]
"""
import argparse
import math
import os
import re
import sys

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

from PIL import Image, ImageDraw

ENC_FULL = 16384
TWO_PI_MRAD = 6283
PP = 11


def wrap_pm(x):
    while x > TWO_PI_MRAD / 2:
        x -= TWO_PI_MRAD
    while x < -TWO_PI_MRAD / 2:
        x += TWO_PI_MRAD
    return x


def load(path):
    txt = open(path, encoding='utf-8').read()
    body = txt.split('{', 1)[1].split('};', 1)[0]
    return [int(x) for x in re.findall(r'-?\d+', body)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lut', default='tools/out_lut/enc_elec_lut_3200.h')
    ap.add_argument('--csv', default='tools/out_lut/enc_elec_lut_3200.csv')
    ap.add_argument('--out', default='tools/out_lut/enc_elec_lut_3200.png')
    args = ap.parse_args()

    v = load(args.lut)
    n = len(v)
    print('表长 %d, 值域 %d..%d' % (n, min(v), max(v)))

    # ---- 解卷绕成连续相位 (随 raw 增大而下降) ----
    nom = -PP * TWO_PI_MRAD / n
    phi = [float(v[0])]
    for i in range(1, n):
        d = v[i] - (phi[-1] % TWO_PI_MRAD)
        d = wrap_pm(d)
        # 期望增量约 nom, 把 d 折到最接近 nom 的分支
        k = round((nom - d) / TWO_PI_MRAD)
        phi.append(phi[-1] + d + k * TWO_PI_MRAD)

    total = phi[0] - phi[-1]
    print('解卷绕后总下降 %.0f mrad = %.2f 个电周期 (= 极对数, 应为 %d)' %
          (total, total / TWO_PI_MRAD, PP))

    # ---- 与理想线性锯齿比 ----
    xs = list(range(n))
    mx = sum(xs) / n
    my = sum(phi) / n
    b = sum((xs[i] - mx) * (phi[i] - my) for i in range(n)) / sum((x - mx) ** 2 for x in xs)
    a = my - b * mx
    res = [phi[i] - (a + b * xs[i]) for i in range(n)]
    rms = math.sqrt(sum(r * r for r in res) / n)
    print('拟合斜率 %.4f mrad/箱 = %.4f mrad/count (名义 %.4f)'
          % (b, b / (ENC_FULL / n), nom / (ENC_FULL / n)))
    print('相对直线的偏差: RMS %.0f mrad, 峰峰 %.0f mrad (%.1f 度电), 极值 %.0f..%.0f'
          % (rms, max(res) - min(res), (max(res) - min(res)) * 180 / math.pi / 1000,
             min(res), max(res)))

    # ---- 接缝 ----
    print('接缝: v[0]=%d, v[-1]=%d, 增量 %.1f mrad (名义 %.1f)'
          % (v[0], v[-1], wrap_pm(v[0] - v[-1]), nom))

    # ---- 单调性 ----
    steps = [wrap_pm(v[(i + 1) % n] - v[i]) for i in range(n)]
    rev = [i for i in range(n) if steps[i] >= 0]
    print('相邻增量: 均值 %.1f, 极值 %.1f..%.1f, 反转 %d 处'
          % (sum(steps) / n, min(steps), max(steps), len(rev)))

    # ---- 折叠补的箱 ----
    fold = []
    if os.path.exists(args.csv):
        for line in open(args.csv, encoding='utf-8').read().splitlines()[1:]:
            p = line.split(',')
            if len(p) > 6 and p[6] == 'fold':
                fold.append(int(p[0]))
    print('折叠补的箱 %d 个 (%.1f%%)' % (len(fold), 100.0 * len(fold) / n))

    # ---- 出图 ----
    W, H = 1400, 760
    img = Image.new('RGB', (W, H), 'white')
    d = ImageDraw.Draw(img)
    m = 60
    pw, ph = W - 2 * m, (H - 3 * m) // 2

    def panel(y0, title, ymin, ymax, series, colors):
        d.rectangle([m, y0, m + pw, y0 + ph], outline=(180, 180, 180))
        d.text((m, y0 - 16), title, fill=(0, 0, 0))
        for k in range(5):
            yy = y0 + ph - ph * k // 4
            val = ymin + (ymax - ymin) * k / 4
            d.line([m, yy, m + pw, yy], fill=(238, 238, 238))
            d.text((4, yy - 6), '%.0f' % val, fill=(90, 90, 90))
        for yy, val in ((y0 + ph, ymin), (y0, ymax)):
            pass
        for ser, col in zip(series, colors):
            pts = []
            for i in range(n):
                x = m + pw * i // (n - 1)
                t = (ser[i] - ymin) / (ymax - ymin) if ymax > ymin else 0
                yy = y0 + ph - int(ph * max(0.0, min(1.0, t)))
                pts.append((x, yy))
            d.line(pts, fill=col)
        d.text((m + pw - 150, y0 - 16), 'x: raw 0..16383', fill=(120, 120, 120))

    # 面板 1: 表值本身 (锯齿, 每圈 11 个电周期)
    panel(m, 'enc_elec_lut[raw]  (mrad, 0..6283)  = sawtooth, %d cycles/rev' % PP,
          -200, TWO_PI_MRAD + 200, [[float(v[i]) for i in range(n)]], [(200, 30, 30)])
    # 面板 2: 相对直线的偏差 + 折叠区
    y0b = 2 * m + ph
    ymin, ymax = min(res) - 20, max(res) + 20
    d.rectangle([m, y0b, m + pw, y0b + ph], outline=(180, 180, 180))
    for f in fold:
        x = m + pw * f // (n - 1)
        d.line([x, y0b, x, y0b + ph], fill=(255, 220, 220))
    d.line([m, y0b + ph // 2, m + pw, y0b + ph // 2], fill=(200, 200, 200))
    pts = []
    for i in range(n):
        x = m + pw * i // (n - 1)
        t = (res[i] - ymin) / (ymax - ymin)
        pts.append((x, y0b + ph - int(ph * t)))
    d.line(pts, fill=(20, 80, 200))
    d.text((m, y0b - 16),
           'deviation from ideal sawtooth (mrad)   pink = bins filled by electrical-period folding',
           fill=(0, 0, 0))
    for k in range(5):
        yy = y0b + ph - ph * k // 4
        d.text((4, yy - 6), '%.0f' % (ymin + (ymax - ymin) * k / 4), fill=(90, 90, 90))

    img.save(args.out)
    print('图已保存: %s' % args.out)


if __name__ == '__main__':
    main()
