#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LUT 标定日志分析器 (只用标准库)

用法:  python tools/lut_log_analyze.py <日志文件> [--top N]

做四件事:
  1) 重算表体检: 整圈累积电角度 sum / 倒退箱数 back / 最大最小步进
     (期望 sum = 极对数 x 6283; 12 对极 = 75396)
  2) 【一一对应检验】学习阶段的每一格 (raw/16) 里, "磁场电角度"的圆跨度。
     转子若老老实实跟着磁场走, 每格跨度应该只有几十 mrad;
     跨度几百~几千 mrad 的格子 = 那段里转子被顶住/打滑, 磁场却在继续走,
     记录下来的一格就是"多值"的, 表在那里必然不准。
  3) 换向后【顿/卡在哪】: 64 扇区 dwell 排行 + 每扇区 raw 速率
  4) STALL / KICK 行的 raw 直方图
"""
import re, sys, math
from collections import defaultdict

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

BINS = 1024
CNT_PER_BIN = 16384 // BINS          # 16
NOMINAL = 73.6                       # mrad / bin (12 极对名义值)

def wrap(d):
    while d > 3141: d -= 6283
    while d < -3141: d += 6283
    return d

def parse(path):
    tbls, cur, csv, marks = [], None, [], []
    for i, raw in enumerate(open(path, encoding='utf-8', errors='replace')):
        line = raw.rstrip('\r\n')
        m = re.match(r'^LUT(\d{3}) (.*)$', line)
        if m:
            if cur is None:
                cur = [None] * BINS
                tbls.append(cur)
            base = int(m.group(1))
            for k, v in enumerate(m.group(2).split()):
                if base + k < BINS and cur[base + k] is None:
                    cur[base + k] = int(v)
            continue
        cur = None
        m = re.match(r'^(\d+),(\d+),(\d+),(-?\d+)$', line)
        if m:
            csv.append((i, int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))))
            continue
        if re.search(r'LUT dump|LUT health|repair:|commutation ON|STALL|KICK|CAL SWEEP|CAL LUT', line):
            marks.append((i, line.strip()))
    return tbls, csv, marks

def health(t):
    s = b = 0; wn = wp = 0; n = 0
    for i in range(BINS):
        a, c = t[i], t[(i + 1) % BINS]
        if a is None or c is None: continue
        d = wrap(c - a)
        num = NOMINAL - d
        k = int(round(num / 6283.0))
        d += k * 6283
        s += d; n += 1
        if d < 0:
            b += 1
            wn = min(wn, d)
        wp = max(wp, d)
    return s, b, wn, wp, n

def circ_span(vals):
    """圆跨度: 6283 - 最大空隙"""
    if len(vals) < 2: return 0.0
    v = sorted(v % 6283 for v in vals)
    gaps = [v[i+1] - v[i] for i in range(len(v)-1)] + [v[0] + 6283 - v[-1]]
    return 6283.0 - max(gaps)

def main():
    path = sys.argv[1]
    top = 12
    if '--top' in sys.argv:
        top = int(sys.argv[sys.argv.index('--top') + 1])
    tbls, csv, marks = parse(path)

    print('=== 关键行 (%d) ===' % len(marks))
    for i, l in marks[:12]:
        print('  %s' % l[:110])
    if len(marks) > 12:
        print('  ...')
        for i, l in marks[-6:]:
            print('  %s' % l[:110])

    for bi, t in enumerate(tbls):
        if t[0] is None: continue
        s, b, wn, wp, n = health(t)
        print('\n=== 表 #%d (%d 格有效) 体检: sum=%d (12rev=%d, 13rev=%d) back=%d worst=%d..%d ==='
              % (bi, n, s, 12*6283, 13*6283, b, wn, wp))

    if not csv:
        print('\n(日志里没有 CSV 遥测行, 跳过 2/3/4)')
        return

    on = next((i for i, l in marks if 'commutation ON' in l), None)
    learn = [r for r in csv if on is None or r[0] < on]
    run   = [r for r in csv if on is not None and r[0] > on]
    print('\n学习阶段采样 %d 行, 换向后 %d 行 (= %.1f s @10ms)' % (len(learn), len(run), len(run)*0.01))

    # ---- 2) 一一对应检验 ----
    if learn:
        byb = defaultdict(list)
        for _, sw, encE, raw, lut in learn:
            byb[raw // CNT_PER_BIN].append(sw)
        rows = sorted(((circ_span(v), k, len(v)) for k, v in byb.items()), reverse=True)
        print('\n=== 一一对应检验: 每格(16 counts)里磁场角的圆跨度 (只看样本>=2 的格) ===')
        print('  全表 %d 格有样本, 跨度 >200mrad 的 %d 格, >500mrad 的 %d 格'
              % (len(rows), sum(1 for s,k,n in rows if s > 200), sum(1 for s,k,n in rows if s > 500)))
        print('  最差的 %d 格:' % top)
        for s, k, n in rows[:top]:
            print('    bin %4d (raw %5d..%5d)  n=%3d  跨度=%5.0f mrad (%.0f deg)' % (k, k*16, k*16+15, n, s, s*180/math.pi))
        # 64 扇区汇总
        sec = defaultdict(list)
        for s, k, n in rows:
            if n >= 2: sec[k // 16].append(s)
        print('  64 扇区 (每扇区 256 counts) 的跨度中位数/最大:')
        line = ''
        for i in range(64):
            v = sec.get(i, [])
            med = sorted(v)[len(v)//2] if v else 0
            line += '%3.0f ' % med
            if i % 16 == 15:
                print('    raw %5d: %s' % ((i-15)*256, line)); line = ''

    # ---- 3) 换向后 顿在哪里 ----
    if run:
        S = 64; W = 256
        dwell = [0]*S; dsum = [0.0]*S; dn = [0]*S
        for k in range(1, len(run)):
            d = run[k][3] - run[k-1][3]
            if d > 8192: d -= 16384
            if d < -8192: d += 16384
            s = (run[k][3] // W) % S
            dwell[s] += 1
            if abs(d) < 4000: dsum[s] += d; dn[s] += 1
        tot = sum(dwell)
        print('\n=== 换向后 dwell 排行 (顿/卡就在 dwell 高的扇区) ===')
        for s in sorted(range(S), key=lambda x: -dwell[x])[:top]:
            r = dsum[s]/dn[s] if dn[s] else 0
            print('  raw %5d..%5d  dwell %4d 行 (%5.0f ms, %.1f%% of run)  raw速率 %7.1f counts/拍'
                  % (s*W, s*W+W-1, dwell[s], dwell[s]*10, 100.0*dwell[s]/tot, r))
        net = 0
        for k in range(1, len(run)):
            d = run[k][3] - run[k-1][3]
            if d > 8192: d -= 16384
            if d < -8192: d += 16384
            net += d
        print('  换向后净行程 %d counts (%.1f 圈), 平均 %.0f counts/拍 (%s RPM 若刻度均匀)'
              % (net, net/16384.0, net/max(1,len(run)), '%.0f' % (net/max(1,len(run))*100/16384*60)))

    # ---- 4) STALL / KICK ----
    st = [(i, l) for i, l in marks if 'STALL' in l or 'KICK' in l]
    if st:
        bins = defaultdict(int)
        for _, l in st:
            m = re.search(r'raw=(\d+)', l)
            if m: bins[int(m.group(1)) // 256 * 256] += 1
        print('\n=== STALL/KICK %d 条, 按 256 counts 汇总 ===' % len(st))
        for s in sorted(bins, key=lambda x: -bins[x])[:top]:
            print('  raw %5d..%5d : %d' % (s, s+255, bins[s]))

if __name__ == '__main__':
    main()
