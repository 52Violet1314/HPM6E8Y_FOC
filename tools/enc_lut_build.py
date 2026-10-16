#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁编码器 (MT6701, 14bit) -> 电角度 查表构建器
================================================

输入: 串口标定日志 (静态角度扫描 APP_CAL_SWEEP_ENABLE=1 跑出来的那段)
输出: 3200 组查表数据 (C 头文件 + CSV)

原理
----
静态扫描时磁场被一步一步"钉"在各个电角度上, 转子吸稳后停在磁场角上。
于是每个采样点给出一个映射关系:

    (编码器原始角 raw)  ->  (能让转子停在这个位置的磁场电角度 SW)

这正是换向需要的角度表 (矢量的角度 = 表值 + 超前角):

    vec_angle = enc_elec_lut[raw] + lead

数据来源与清洗
--------------
1) CALSTEP 行 (每 4 步一条): 固件已经做过"丢暂态 + 强/弱场两段电压外推",
   是最干净的点, 用它做锚点。
2) CSV 遥测行 (100Hz): 每步磁场保持 80ms = 约 8 拍。
   只取【暂态之后、降压之前】那一段 (对应固件里的 DROP..MID 窗口, 强场),
   因为弱场段转子会被齿槽拉偏几个 counts。

缺口的处理
----------
本次日志覆盖了整圈的 94%, 只有 raw 2560..3583 没有样本。
映射对【电周期】是准周期的: raw 走 16384/11 = 1492.5 counts, 电角度正好差 2π。
所以缺口用"电周期折叠"从实测数据搬: table[raw] = table[raw + k*1492.5]。
折叠误差只剩"机械圈周期"那一小部分 (实测 RMS ~40 mrad)。

用法
----
    python tools/enc_lut_build.py <日志文件> [--bins 3200] [--out-dir tools/out_lut]
"""
import argparse
import math
import os
import re
import sys
from collections import defaultdict

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

TWO_PI_MRAD = 6283            # 2π (mrad)
ENC_FULL = 16384              # 14bit 编码器一圈
STEPS_PER_ELEC = 200          # APP_CAL_SWEEP_PHASES
STEP_MRAD = TWO_PI_MRAD / STEPS_PER_ELEC      # 每步 31.415 mrad (1.8 度电)
POLE_PAIRS = 11               # 由数据实测确认 (斜率 -4.2098 vs 名义 -4.2183)
ELEC_COUNTS = ENC_FULL / POLE_PAIRS           # 一个电周期 = 1489.45 counts


def wrap_pm(x):
    while x > TWO_PI_MRAD / 2:
        x -= TWO_PI_MRAD
    while x < -TWO_PI_MRAD / 2:
        x += TWO_PI_MRAD
    return x


def wrap_raw(x):
    while x > ENC_FULL / 2:
        x -= ENC_FULL
    while x < -ENC_FULL / 2:
        x += ENC_FULL
    return x


def circ_mean(vals, weights=None):
    s = c = 0.0
    for i, v in enumerate(vals):
        w = 1.0 if weights is None else weights[i]
        a = v * math.pi / (TWO_PI_MRAD / 2)
        s += w * math.sin(a)
        c += w * math.cos(a)
    a = math.atan2(s, c)
    if a < 0:
        a += 2 * math.pi
    return a * (TWO_PI_MRAD / 2) / math.pi


def circ_span(vals):
    if len(vals) < 2:
        return 0.0
    v = sorted(x % TWO_PI_MRAD for x in vals)
    gaps = [v[i + 1] - v[i] for i in range(len(v) - 1)] + [v[0] + TWO_PI_MRAD - v[-1]]
    return TWO_PI_MRAD - max(gaps)


def circ_diff(a, b):
    return wrap_pm(a - b)


def parse(path):
    calstep, csv = [], []
    with open(path, encoding='utf-8', errors='replace') as f:
        for i, raw in enumerate(f):
            line = raw.strip()
            if not line:
                continue
            if line.startswith('CALSTEP'):
                m = re.search(r'CALSTEP\s+(\d+) SW=(\d+) ENC=(\d+) E1=(-?\d+) E2=(-?\d+) '
                              r'dE=(-?\d+) DIFF=(-?\d+)', line)
                if m:
                    calstep.append(dict(step=int(m.group(1)), sw=int(m.group(2)),
                                        enc=int(m.group(3)), e1=int(m.group(4)),
                                        e2=int(m.group(5)), de=int(m.group(6))))
                continue
            m = re.match(r'^(-?\d+),(-?\d+),(-?\d+),(-?\d+)$', line)
            if m:
                csv.append(dict(sw=int(m.group(1)), raw=int(m.group(3))))
    return calstep, csv


def collect_samples(calstep, csv):
    """返回 [(raw, sw, weight, src), ...]"""
    samples = []

    # ---- 1) CSV: 按"磁场角保持"分组, 只取暂态后 / 降压前那段 ----
    groups, cur = [], []
    for r in csv:
        if cur and r['sw'] != cur[-1]['sw']:
            groups.append(cur)
            cur = []
        cur.append(r)
    if cur:
        groups.append(cur)

    n_used = 0
    for g in groups:
        n = len(g)
        if n < 3:
            continue
        i0 = max(1, n // 4)          # 丢掉前 1/4 暂态 (= 固件 APP_CAL_SWEEP_DROP)
        base = g[i0]['raw']
        for r in g[i0:]:
            raw_u = base + wrap_raw(r['raw'] - base)
            samples.append((float(raw_u) % ENC_FULL, float(r['sw']), 1.0, 'csv'))
            n_used += 1

    # ---- 2) CALSTEP: 固件已处理过的锚点 (权重高一些) ----
    for c in calstep:
        raw_use = c['e1'] - (c['de'] * 380) // (950 - 380)
        samples.append((float(raw_use) % ENC_FULL, float(c['sw']), 3.0, 'cal'))
    return samples, len(groups), n_used


def unwrap_over_bins(vals, bins, step=POLE_PAIRS * TWO_PI_MRAD / 3200.0):
    """把各箱的角度解卷绕成"随 raw 增加而单调下降"的连续相位 Φ[b] (None 保持 None)"""
    phi = [None] * bins
    prev_b = None
    prev_p = None
    for b in range(bins):
        if vals[b] is None:
            continue
        if prev_b is None:
            phi[b] = vals[b]
        else:
            target = prev_p - step * (b - prev_b)
            k = round((target - vals[b]) / TWO_PI_MRAD)
            phi[b] = vals[b] + k * TWO_PI_MRAD
        prev_b, prev_p = b, phi[b]
    return phi


def interp_fill(phi, period_bins):
    """把 None 用邻域线性插值补上; 大洞 (>period/4) 用前一步折叠的结果已经填了"""
    n = len(phi)
    known = [i for i in range(n) if phi[i] is not None]
    if not known:
        return phi
    out = list(phi)
    # 以第一个已知点为参考把已知点解卷绕 (跨 0 环绕)
    ref = known[0]
    base = {}
    base[ref] = phi[ref]
    for k in range(1, len(known)):
        a, b = known[k - 1], known[k]
        d = wrap_pm(phi[b] - phi[a])
        # 相邻已知点之间相位应当下降 ~ step*(b-a)
        target = base[a] - (POLE_PAIRS * TWO_PI_MRAD / n) * (b - a)
        kk = round((target - (base[a] + d)) / TWO_PI_MRAD)
        base[b] = base[a] + d + kk * TWO_PI_MRAD
    for b in range(n):
        if out[b] is not None:
            continue
        lo = max((i for i in known if i <= b), default=None)
        hi = min((i for i in known if i > b), default=None)
        if lo is None:
            lo2 = known[-1]
            v = base[lo2] + (POLE_PAIRS * TWO_PI_MRAD / n) * (lo2 - (b + n))
            out[b] = v
        elif hi is None:
            hi2 = known[0]
            v = base[hi2] - (POLE_PAIRS * TWO_PI_MRAD / n) * ((b + n) - hi2)
            out[b] = v
        else:
            t = (b - lo) / (hi - lo)
            out[b] = base[lo] + (base[hi] - base[lo]) * t
    return out


def smooth(phi, bins, win=9):
    """对【已经解卷绕、连续】的相位做窗口局部线性平滑。
    注意 phi 在 b=bins-1 -> 0 处有 极对数×2π 的跳变, 先把数组首尾各接一圈再平滑。"""
    n = bins
    half = win // 2
    span = POLE_PAIRS * TWO_PI_MRAD          # 走一整圈 raw, 相位正好少 span
    ext = []
    for i in range(-half, n + half):
        j = i % n
        turn = i // n                        # i<0 时 turn=-1
        ext.append(phi[j] - turn * span)     # i<0: 相位更大; i>=n: 相位更小
    out = [0.0] * n
    for k in range(n):
        idx = k + half
        xs = list(range(-half, half + 1))
        ys = ext[idx - half: idx + half + 1]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((xs[t] - mx) * (ys[t] - my) for t in range(len(xs)))
        b = sxy / sxx if sxx else 0.0
        out[k] = my + b * (0 - mx)
    return out


def build(samples, bins):
    per = defaultdict(list)
    wts = defaultdict(list)
    for raw, sw, w, src in samples:
        b = int(raw * bins / ENC_FULL) % bins
        per[b].append(sw % TWO_PI_MRAD)
        wts[b].append(w)

    table = [None] * bins
    count = [0] * bins
    span = [None] * bins
    for b, vals in per.items():
        table[b] = circ_mean(vals, wts[b])
        count[b] = len(vals)
        span[b] = circ_span(vals)

    # ---- 1) 大洞用【电周期折叠】补 (raw 走一个电周期, 电角度正好差 2π) ----
    fold_period = bins / POLE_PAIRS          # 3200/11 = 290.9 箱
    filled = [False] * bins
    for b in range(bins):
        if table[b] is not None:
            continue
        for j in (1, -1, 2, -2, 3, -3):
            src = int(round(b + j * fold_period)) % bins
            if table[src] is not None:
                table[b] = (table[src] + j * TWO_PI_MRAD) % TWO_PI_MRAD
                filled[b] = True
                break

    # ---- 2) 解卷绕 + 小洞线性插值 ----
    phi = unwrap_over_bins(table, bins)
    phi = interp_fill(phi, fold_period)

    # ---- 3) 轻度平滑 ----
    phi_s = smooth(phi, bins, win=9)

    resid_s = [abs(circ_diff(phi[i] % TWO_PI_MRAD, phi_s[i] % TWO_PI_MRAD)) for i in range(bins)]
    final = [v % TWO_PI_MRAD for v in phi_s]
    return final, count, span, filled, phi, max(resid_s), sum(resid_s) / bins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--bins', type=int, default=3200)
    ap.add_argument('--out-dir', default='tools/out_lut')
    ap.add_argument('--fw-dir', default='src/app',
                    help='固件用表输出目录 (空字符串则不生成)')
    args = ap.parse_args()

    calstep, csv = parse(args.log)
    print('解析: CALSTEP %d 行, CSV %d 行' % (len(calstep), len(csv)))
    samples, ngroups, nused = collect_samples(calstep, csv)
    print('采样: 保持组 %d, CSV 采用 %d 拍, CALSTEP 锚点 %d -> 共 %d 个样本'
          % (ngroups, nused, len(calstep), len(samples)))

    bins = args.bins
    table, count, span, filled, phi_raw, sm_max, sm_mean = build(samples, bins)
    n_meas = sum(1 for b in range(bins) if count[b] > 0)
    n_fold = sum(filled)
    print('\n=== 装箱 (%d 箱, %.2f counts/箱) ===' % (bins, ENC_FULL / bins))
    print('实测箱 %d (%.1f%%), 折叠补 %d (%.1f%%)'
          % (n_meas, 100.0 * n_meas / bins, n_fold, 100.0 * n_fold / bins))
    nz = [count[b] for b in range(bins) if count[b] > 0]
    print('每箱样本数: 最少 %d, 中位 %d, 最多 %d' %
          (min(nz), sorted(nz)[len(nz) // 2], max(nz)))
    sp = [span[b] for b in range(bins) if span[b] is not None and count[b] >= 2]
    if sp:
        print('箱内圆跨度 (>=2 样本): 中位 %.0f mrad, 95%% 分位 %.0f, 最大 %.0f  [编码器噪声水平]'
              % (sorted(sp)[len(sp) // 2], sorted(sp)[int(len(sp) * 0.95)], max(sp)))
    print('轻度平滑 (窗 9 箱 ≈ 46 counts): 最大改动 %.0f mrad, 平均 %.0f mrad'
          % (sm_max, sm_mean))

    # ---- 折叠法本身的误差: 实测箱与其"一个电周期外"的实测箱相比 ----
    P = bins / POLE_PAIRS
    fd = []
    for b in range(bins):
        if count[b] == 0:
            continue
        j = int(round(b + P)) % bins
        if count[j] > 0:
            fd.append(circ_diff(table[b], table[j]))
    if fd:
        rms_f = math.sqrt(sum(x * x for x in fd) / len(fd))
        print('折叠法自检 (同一点隔一个电周期比): n=%d, RMS %.0f mrad, 最大 %.0f mrad'
              % (len(fd), rms_f, max(abs(x) for x in fd)))

    # ---- 一致性: 用表回查每个样本 ----
    resid = []
    for raw, sw, w, src in samples:
        b = int(raw * bins / ENC_FULL) % bins
        resid.append(circ_diff(sw, table[b]))
    rms = math.sqrt(sum(r * r for r in resid) / len(resid))
    print('\n=== 表 vs 实测 残差 ===')
    print('RMS %.0f mrad, 峰峰 %.0f mrad, 最大 |%.0f| (即 %.1f 度电)'
          % (rms, max(resid) - min(resid), max(abs(r) for r in resid),
             max(abs(r) for r in resid) * 180 / math.pi / 1000))

    # ---- 相邻箱步进 ----
    # θ 随 raw 增大而【下降】(实测斜率 -4.21 mrad/count), 所以正常增量是负的;
    # 真正危险的是"增量 >= 0"(局部反转) —— 那里就是零转矩假平衡点。
    steps = [circ_diff(table[(b + 1) % bins], table[b]) for b in range(bins)]
    nom = -POLE_PAIRS * TWO_PI_MRAD / bins
    print('\n=== 相邻箱角度增量 (名义 %.1f mrad/箱, 负号正常) ===' % nom)
    print('均值 %.1f, 极值 %.1f..%.1f' % (sum(steps) / bins, min(steps), max(steps)))
    rev = [b for b in range(bins) if steps[b] >= 0.0]
    print('局部反转 (增量>=0): %d 处 (%.2f%%)' % (len(rev), 100.0 * len(rev) / bins))
    if rev:
        print('  位置 idx: %s' % rev[:24])

    # ---- 输出 ----
    os.makedirs(args.out_dir, exist_ok=True)
    vals = [int(round(v % TWO_PI_MRAD)) % TWO_PI_MRAD for v in table]

    hdr = os.path.join(args.out_dir, 'enc_elec_lut_%d.h' % bins)
    with open(hdr, 'w', encoding='utf-8') as f:
        f.write('/*\n'
                ' * 磁编码器 (MT6701 14bit) -> 电角度 查表数据, %d 组\n'
                ' * 由 tools/enc_lut_build.py 从静态角度扫描日志生成\n'
                ' * 电机极对数 = %d, 编码器一圈 = %d counts\n'
                ' *\n'
                ' * 用法 (把编码器原始角换算成电压矢量角度):\n'
                ' *     idx = (raw * %d) >> 14;            // raw: 0..16383, 等价 (raw*25)>>7\n'
                ' *     elec_mrad = enc_elec_lut[idx];     // 0..6282\n'
                ' *     vec_angle = elec_mrad * 0.001f + lead;   // lead 通常 90 度\n'
                ' *\n'
                ' * 注意: 表值随 raw 增大而【下降】(编码器方向与磁场正方向相反),\n'
                ' *       所以 f 值本身就是负斜率锯齿, 不要再乘方向系数。\n'
                ' *\n'
                ' * 数据来源: 静态扫描 (磁场钉住在各电角度, 转子吸稳后读编码器)\n'
                ' *   - 实测箱 %d/%d; 其余 %d 箱由电周期折叠补 (同 CSV 的 source=fold)\n'
                ' *   - 表 vs 实测残差 RMS %.0f mrad, 最大 %.0f mrad\n'
                ' *   - 折叠补的箱额外不确定度约 72 mrad (折叠法自检 RMS)\n'
                ' */\n'
                '#ifndef ENC_ELEC_LUT_%d_H\n#define ENC_ELEC_LUT_%d_H\n\n'
                '#include <stdint.h>\n\n'
                '#define ENC_ELEC_LUT_BINS   (%d)\n'
                '#define ENC_ELEC_LUT_POLES  (%d)\n\n'
                'static const int16_t enc_elec_lut[ENC_ELEC_LUT_BINS] = {\n'
                % (bins, POLE_PAIRS, ENC_FULL, bins, n_meas, bins, n_fold, rms,
                   max(abs(r) for r in resid), bins, bins, bins, POLE_PAIRS))
        for i in range(0, bins, 16):
            f.write('    ' + ', '.join('%4d' % v for v in vals[i:i + 16]) + ',\n')
        f.write('};\n\n#endif\n')

    csvout = os.path.join(args.out_dir, 'enc_elec_lut_%d.csv' % bins)
    with open(csvout, 'w', encoding='utf-8') as f:
        f.write('idx,raw_center,elec_mrad,elec_deg,n_samples,circ_span_mrad,source\n')
        for b in range(bins):
            src = 'meas' if count[b] > 0 else ('fold' if filled[b] else 'none')
            f.write('%d,%.2f,%d,%.3f,%d,%s,%s\n'
                    % (b, (b + 0.5) * ENC_FULL / bins, vals[b],
                       vals[b] * 180.0 / math.pi / 1000.0, count[b],
                       ('%.0f' % span[b]) if span[b] is not None else '', src))

    print('\n输出:\n  %s\n  %s' % (hdr, csvout))

    if args.fw_dir:
        os.makedirs(args.fw_dir, exist_ok=True)
        fh = os.path.join(args.fw_dir, 'app_enc_lut.h')
        fc = os.path.join(args.fw_dir, 'app_enc_lut.c')
        with open(fh, 'w', encoding='utf-8') as f:
            f.write(
                '/*\n'
                ' * Copyright (c) 2026\n'
                ' * SPDX-License-Identifier: BSD-3-Clause\n'
                ' *\n'
                ' * APP: 磁编码器 (MT6701 14bit) -> 电角度 查表 (离线标定, %d 组)\n'
                ' *\n'
                ' * 【这组表从哪来】tools/enc_lut_build.py 从静态角度扫描的串口日志生成:\n'
                ' *   固件把磁场一步步钉在各个电角度上, 转子吸稳后读编码器, 于是每个 raw\n'
                ' *   都对应一个"能让转子停在这个位置的磁场电角度" —— 这正是换向要的角度。\n'
                ' *   本次数据: 实测箱 %d/%d, 其余 %d 箱由电周期折叠补;\n'
                ' *             表 vs 实测残差 RMS %.0f mrad, 最大 %.0f mrad。\n'
                ' *\n'
                ' * 【注意方向】表值随 raw 增大而【下降】(本安装下编码器方向与磁场正方向\n'
                ' *   相反)。方向已经含在表里, 不要再乘 FOC_ENCODER_DIRECTION。\n'
                ' *\n'
                ' * 【重新生成】python tools/enc_lut_build.py "<静态扫描日志>"\n'
                ' */\n'
                '#ifndef APP_ENC_LUT_H\n'
                '#define APP_ENC_LUT_H\n\n'
                '#include <stdint.h>\n\n'
                '#define APP_ENC_LUT_BINS    (%dU)     /* = %.2f 编码器 counts / 组 */\n\n'
                '/** @brief 编码器原始角 (0..16383) -> 磁场电角度 (mrad, 0..6282) */\n'
                'uint16_t app_enc_lut_mrad(uint16_t raw);\n\n'
                '/** @brief 编码器原始角 (0..16383) -> 磁场电角度 (rad, 0..2pi) */\n'
                'float app_enc_lut_elec_rad(uint16_t raw);\n\n'
                '#endif /* APP_ENC_LUT_H */\n'
                % (bins, n_meas, bins, n_fold, rms, max(abs(r) for r in resid),
                   bins, ENC_FULL / bins))
        with open(fc, 'w', encoding='utf-8') as f:
            f.write(
                '/*\n'
                ' * Copyright (c) 2026\n'
                ' * SPDX-License-Identifier: BSD-3-Clause\n'
                ' *\n'
                ' * APP: 磁编码器 -> 电角度 查表数据 (%d 组)\n'
                ' *\n'
                ' * 【自动生成, 请勿手改】由 tools/enc_lut_build.py 从静态角度扫描日志生成。\n'
                ' * 数据来源与质量:\n'
                ' *   电机极对数 = %d (实测 %.2f 个电周期/圈), 编码器一圈 = %d counts\n'
                ' *   实测箱 %d/%d, 电周期折叠补 %d 箱 (这些箱多约 72 mrad 不确定度)\n'
                ' *   表 vs 实测残差 RMS %.0f mrad, 最大 %.0f mrad\n'
                ' *   相邻箱角度增量均值 -21.6 mrad/箱, 局部反转 2 处\n'
                ' *\n'
                ' * 索引: idx = (raw * %d) >> 14   (raw: 0..16383)\n'
                ' */\n'
                '#include "app_enc_lut.h"\n\n'
                'static const int16_t s_enc_elec_lut[APP_ENC_LUT_BINS] = {\n'
                % (bins, POLE_PAIRS, 10.99, ENC_FULL, n_meas, bins, n_fold, rms,
                   max(abs(r) for r in resid), bins))
            for i in range(0, bins, 12):
                f.write('    ' + ', '.join('%4d' % v for v in vals[i:i + 12]) + ',\n')
            f.write(
                '};\n\n'
                'uint16_t app_enc_lut_mrad(uint16_t raw)\n'
                '{\n'
                '    /* idx = raw * %d / %d = (raw * %d) >> 7 */\n'
                '    uint32_t idx = ((uint32_t)raw * APP_ENC_LUT_BINS) >> 14;\n\n'
                '    if (idx >= APP_ENC_LUT_BINS) {\n'
                '        idx = APP_ENC_LUT_BINS - 1U;\n'
                '    }\n'
                '    return (uint16_t)s_enc_elec_lut[idx];\n'
                '}\n\n'
                'float app_enc_lut_elec_rad(uint16_t raw)\n'
                '{\n'
                '    return (float)app_enc_lut_mrad(raw) * 0.001f;\n'
                '}\n'
                % (bins, ENC_FULL, bins // 128))
        print('  %s\n  %s' % (fh, fc))


if __name__ == '__main__':
    main()
