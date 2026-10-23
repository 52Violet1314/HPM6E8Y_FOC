# -*- coding: utf-8 -*-
"""转速显示"撕裂/乱码"模拟器 —— 先复现, 再验证改法。

模拟两套**显示策略**(固件 lcd_ui.c 的两版):
  current : 现状 —— 单个大字标签; 节流计数器按 APP_MAIN_LOOP_MS(=1ms) 累加而显示任务
            其实是 5ms 一拍 -> 实际 100ms 才比一次; 只要整数变了就改文本(实测转速
            的末位一直在跳); 另外 lv_obj_set_style_text_color(lbl_fb) 每拍无条件调用
            -> 状态行每拍都被标脏, 每 10ms 就被重刷一遍(纯浪费带宽)。
  fixed   : 改法 —— 4 个定宽数字位(只有变的那一位被标脏/刷屏); 真时间节流 +
            迟滞(>=2 RPM 才改, 另有 500ms 心跳); 状态行颜色只在变化时写。

时钟: 只有**一个**时钟(面板时间)。显示任务每 5ms 一拍、LVGL 每 10ms 刷一次;
刷屏是阻塞的, 所以它自然会把后面的节拍往后推 —— 和板上 vTaskDelayUntil 的行为一致。

输出: tools/lcd_sim/out/*.png(整屏)、report.txt、以及终端 ASCII 图。
"""
import argparse
import math
import os
import random
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PIL import Image

from panel import ILI9341, Timing
from ui import UI, rgb, to565, UI_BG

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

FPS = 70.0
TICK_S = 0.005        # core1 显示任务周期 (CORE1_DISP_PERIOD_MS)
REFR_S = 0.010        # LV_DEF_REFR_PERIOD


class RpmSource:
    """实测转速 = 真值 + 相关噪声(OU 过程)。

    转速估计器不可能"一位都不跳": 编码器量化 + 电流纹波 + 1ms 采样, 末位一直在动。
    sigma 就是"数字跳得有多厉害"的旋钮。
    """

    def __init__(self, true_rpm=320.0, sigma=3.0, tau=0.060, seed=7):
        self.true, self.sigma, self.tau = true_rpm, sigma, tau
        self.rnd = random.Random(seed)
        self.n, self.est = 0.0, true_rpm

    def step(self, dt):
        a = math.exp(-dt / self.tau)
        self.n = a * self.n + math.sqrt(1 - a * a) * self.rnd.gauss(0, self.sigma)
        self.est = self.true + self.n
        return self.est


class Policy:
    """固件侧的显示策略。"""

    def __init__(self, ui, mode, log, deadband=3, beat_ms=1000.0, min_gap_ms=100.0,
                 tau_s=0.250):
        self.ui, self.mode, self.log = ui, mode, log
        self.shown = 320
        self.acc_ms = 0
        self.since_change_ms = 0.0
        self.updates = 0
        self.cand, self.cand_ms = 320, 0.0
        self.commit_ms = 0.0
        self.tau_s = tau_s
        self.disp = 320.0
        self.deadband = deadband
        self.beat_ms, self.min_gap_ms = beat_ms, min_gap_ms

    def tick(self, now_ms, rpm):
        ui = self.ui
        self.acc_ms += 1                      # 显示任务每 5ms 一拍
        if self.mode == "current":
            ui.touch(ui.fb)                   # 现状: 每拍无条件写颜色 -> 每拍标脏
            if self.acc_ms < 20:              # 20 x "1ms" = 其实 100ms
                return
            self.acc_ms = 0
            v = int(rpm + 0.5) if rpm >= 0 else int(rpm - 0.5)
            v = max(-999, min(999, v))
            if v != self.shown:
                self.shown = v
                ui.set_text(ui.digits[0], "%d" % v)
                self.updates += 1
                self.log.append((now_ms, v))
        else:
            if self.acc_ms < 4:               # 真时间: 4 x 5ms = 20ms
                return
            self.acc_ms = 0
            now = now_ms
            # 显示滤波(EMA, tau=250ms) + 取整: 表头该显示"读数", 不是瞬时采样。
            # 这一条才是让数字不再乱跳的关键: 噪声被平均掉, 真变化 0.3s 内跟上。
            a = 1.0 - math.exp(-0.020 / self.tau_s)
            self.disp += (rpm - self.disp) * a
            v = int(self.disp + 0.5) if self.disp >= 0 else int(self.disp - 0.5)
            v = max(-999, min(999, v))
            # 采纳条件: 读数与显示值差 >= 死区(3 RPM) —— 真变化 0.3s 内跟上;
            # 另加 1s 心跳, 免得长期差 1~2 RPM 一直挂着。
            # 再限一个最小改写间隔(100ms), 加速时数字不会 30 次/秒地刷。
            if ((abs(v - self.shown) >= self.deadband
                 or ((now - self.commit_ms) >= self.beat_ms and v != self.shown))
                    and (now - self.commit_ms) >= self.min_gap_ms):
                self.commit(v, now)

    def commit(self, v, now):
        ui = self.ui
        self.shown = v
        self.commit_ms = now
        self.since_change_ms = 0.0
        self.updates += 1
        self.log.append((now, v))
        self.set_cells(v)

    def set_cells(self, v):
        """左对齐的整组数字(与固件 ui_rpm_render 一致): 不补前导空格。"""
        ui = self.ui
        s = "%d" % v
        for i in range(len(ui.digits)):
            t = s[i] if i < len(s) else ""
            if ui.digits[i].text != t:
                ui.set_text(ui.digits[i], t)


class Refs:
    """某个数值下, 数字区"稳定后"应该长什么样(直接用界面自己渲染)。"""

    def __init__(self, ui):
        self.ui, self.cache = ui, {}

    def band(self, value):
        key = int(value)
        if key in self.cache:
            return self.cache[key]
        ui = self.ui
        area = ui.digit_band()
        if ui.mode == "current":
            saved = ui.digits[0].text
            ui.digits[0].text = "%d" % key
            box = ui.digits[0].box()
            area = (12, 30, max(box[2], area[2]), 73)
        else:
            saved = [d.text for d in ui.digits]
            s = "%d" % key                     # 左对齐整组, 与固件一致
            for i in range(len(ui.digits)):
                ui.digits[i].text = s[i] if i < len(s) else ""
        # 【必须在"换成目标文本"的状态下渲染, 否则两版参考图一模一样】
        tmp = Image.new("RGB", (320, 240), rgb(UI_BG))
        for ob in ui.objs:
            x0, y0, x1, y1 = ob.box()
            if x0 <= area[2] and area[0] <= x1 and y0 <= area[3] and area[1] <= y1:
                ob.draw(tmp)
        if ui.mode == "current":
            ui.digits[0].text = saved
        else:
            for i, t in enumerate(saved):
                ui.digits[i].text = t
        px = tmp.load()
        rows = {}
        for y in range(area[1], area[3] + 1):
            rows[y] = [to565(px[x, y]) for x in range(area[0], area[2] + 1)]
        self.cache[key] = (area, rows)
        return self.cache[key]


def classify(vis_dict, refs, old_v, new_v, x0, x1):
    """判定此刻显示的是旧值 / 新值 / **两者拼在一起**(=肉眼看到数字被撕开)。

    【只看"新旧本来就不一样的那些行"】数字框有 44 行, 但一个数字的墨迹只占 29 行,
    上下各有一圈背景。只有墨迹那几行被拼开才看得出来; 背景行"新旧相同", 拼开也
    看不出来。所以判据只统计 refs 不同的行 —— 这样数出来的就是**视觉上**的撕裂。
    """
    v_old = refs.band(old_v)
    v_new = refs.band(new_v)
    diff_rows = [r for r in v_old[1]
                 if r in v_new[1] and v_old[1][r] != v_new[1][r]]
    if not diff_rows:
        return "same"                       # 新旧一模一样(理论上不会走到)
    n_old = n_new = 0
    for r in diff_rows:
        area, _ = v_old
        a0, a1 = max(x0, area[0]), min(x1, area[2])
        if a1 < a0:
            continue
        seg = vis_dict[r][a0 - x0:a1 - x0 + 1] if r in vis_dict else None
        if seg is None:
            continue
        if seg == v_new[1][r][a0 - area[0]:a1 - area[0] + 1]:
            n_new += 1
        elif seg == v_old[1][r][a0 - area[0]:a1 - area[0] + 1]:
            n_old += 1
        else:
            return "spliced"                # 行内就是两个字形混着 -> 一定是撕开
    if n_new == len(diff_rows):
        return "new"
    if n_old == len(diff_rows):
        return "old"
    if n_new and n_old:
        return "spliced"
    return "new"


def run(mode, duration_ms=4000, sclk=25e6, seed=7, true_rpm=320.0, sigma=3.0,
        tau=0.060, axis="y", ramp_at=None):
    tm = Timing(sclk_hz=sclk)
    panel = ILI9341(tm, frame_hz=FPS, axis=axis)
    ui = UI(panel, mode="current" if mode == "current" else "fixed")

    ui.redraw((0, 0, 319, 239))
    panel.prime([[to565(ui.img.load()[x, y]) for x in range(320)] for y in range(240)])
    panel.t = 0.0

    src = RpmSource(true_rpm, sigma, tau, seed)
    log = []
    pol = Policy(ui, mode, log)
    refs = Refs(ui)
    refs.band(320)

    area = ui.digit_band()
    x0, x1 = area[0], area[2]
    rows = list(range(area[1], area[3] + 1))

    samples = []
    stats = dict(digit_flushes=0, digit_bytes=0, fb_flushes=0, fb_bytes=0,
                 total_flushes=0, total_bytes=0)
    next_refresh = panel.t
    tick_acc = 0.0
    end = duration_ms / 1000.0
    prev_vis = None
    transitions = 0
    SAMPLE_S = 0.002                     # 每 2ms 判一次"屏上显示的是什么"
    last_sample = -1.0

    while panel.t < end:
        panel.t += 0.001                      # 唯一时钟: 面板时间
        tick_acc += 0.001
        while tick_acc >= TICK_S:
            tick_acc -= TICK_S
            rpm = src.step(TICK_S)
            if ramp_at is not None and panel.t >= ramp_at[0] / 1000.0:
                src.true = ramp_at[1]
            pol.tick(panel.t * 1000.0, rpm)

        if panel.t >= next_refresh:
            next_refresh = panel.t + REFR_S
            n = ui.flush_pending()
            stats["total_flushes"] += n
            for (tf, sub) in ui.flush_log[-n:] if n else []:
                npix = (sub[2] - sub[0] + 1) * (sub[3] - sub[1] + 1)
                nb = npix * 2
                stats["total_bytes"] += nb
                if sub[1] <= 73 and sub[3] >= 30 and sub[0] <= 116:
                    stats["digit_flushes"] += 1
                    stats["digit_bytes"] += nb
                elif sub[1] <= 181 and sub[3] >= 164:
                    stats["fb_flushes"] += 1
                    stats["fb_bytes"] += nb

        # 采样: 面板此刻显示的数字带(每 2ms 一次)
        if panel.t - last_sample >= SAMPLE_S:
            last_sample = panel.t
            new_v = log[-1][1] if log else 320
            old_v = log[-2][1] if len(log) >= 2 else 320
            vis = panel.visible_rows(rows, panel.t, x0, x1)
            cls = classify(vis, refs, old_v, new_v, x0, x1)
            flat = [v for r in rows for v in vis[r]]
            if prev_vis is not None and flat != prev_vis:
                transitions += 1
            prev_vis = flat
            samples.append((panel.t * 1000.0, cls, pol.shown, src.est))

    spliced = [s for s in samples if s[1] == "spliced"]
    dur_s = duration_ms / 1000.0
    return ui, panel, dict(
        mode=mode, axis=axis, sclk=sclk,
        updates=pol.updates, update_rate=pol.updates / dur_s,
        samples=len(samples), spliced=len(spliced) * SAMPLE_S * 1000.0,
        spliced_pct=100.0 * len(spliced) / max(1, len(samples)),
        spliced_episodes=count_episodes(samples),
        visible_changes_per_s=transitions / dur_s,
        digit_flushes=stats["digit_flushes"], digit_bytes=stats["digit_bytes"],
        fb_flushes=stats["fb_flushes"], fb_bytes=stats["fb_bytes"],
        total_flushes=stats["total_flushes"], total_bytes=stats["total_bytes"],
        digit_spi_ms=stats["digit_bytes"] * 8 / sclk * 1000.0,
        fb_spi_ms=stats["fb_bytes"] * 8 / sclk * 1000.0,
        total_spi_ms=stats["total_bytes"] * 8 / sclk * 1000.0,
        shown_final=pol.shown, sample_list=samples, log=log,
    )


def count_episodes(samples):
    ep, prev = 0, None
    for (_, c, _, _) in samples:
        if c == "spliced" and prev != "spliced":
            ep += 1
        prev = c
    return ep


def render_png(panel, t, path):
    rows = panel.visible_rows(range(240), t, 0, 319)
    img = Image.new("RGB", (320, 240))
    px = img.load()
    for y in range(240):
        r = rows[y]
        for x in range(320):
            v = r[x]
            px[x, y] = (((v >> 11) & 0x1F) << 3, ((v >> 5) & 0x3F) << 2, (v & 0x1F) << 3)
    img.save(path)


def ascii_band(panel, t, x0=12, x1=116, y0=35, y1=72, step=2, thresh=45):
    rows = panel.visible_rows(range(y0, y1 + 1), t, x0, x1)
    out = []
    for y in range(y0, y1 + 1, step):
        line = []
        for x in range(0, x1 - x0 + 1, step):
            v = rows[y][x]
            r = ((v >> 11) & 0x1F) << 3
            g = ((v >> 5) & 0x3F) << 2
            b = (v & 0x1F) << 3
            lum = (r * 299 + g * 587 + b * 114) // 1000
            line.append("#" if lum > thresh else ".")
        out.append("".join(line))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=int, default=4000)
    ap.add_argument("--sclk", type=float, default=25e6)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--axis", default="both", choices=["x", "y", "both"])
    ap.add_argument("--sigma", type=float, default=3.0)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    axes = ["y", "x"] if args.axis == "both" else [args.axis]
    lines = []
    for axis in axes:
        for mode in ("current", "fixed"):
            ui, panel, r = run(mode, args.duration, args.sclk, args.seed,
                               sigma=args.sigma, axis=axis)
            msg = ("[axis=%s %-7s] 数字改写 %4.1f 次/秒 | 屏上数字每秒变 %4.1f 次 | "
                   "被撕开 %5.1f ms (%.2f%%, %d 段) | 刷屏占用: 数字 %5.1fms, 状态行 %5.1fms, "
                   "合计 %5.1fms (%.0f%% 的 SPI 时间)" %
                   (axis, mode, r["update_rate"], r["visible_changes_per_s"], r["spliced"],
                    r["spliced_pct"], r["spliced_episodes"],
                    r["digit_spi_ms"], r["fb_spi_ms"], r["total_spi_ms"],
                    100.0 * r["total_spi_ms"] / (args.duration)))
            print(msg)
            lines.append(msg)
            if mode == "current" and r["spliced_episodes"] > 0:
                t = next(s[0] for s in r["sample_list"] if s[1] == "spliced") / 1000.0
                render_png(panel, t, os.path.join(OUT, "before_torn_axis%s.png" % axis))
                print("    ↓ 被撕开那一瞬的数字带 (t=%.3fs)" % t)
                print(ascii_band(panel, t))
                lines.append(ascii_band(panel, t))
            else:
                render_png(panel, panel.t, os.path.join(OUT, "after_clean_axis%s.png" % axis))
                print("    ↓ 结束时刻的数字带")
                print(ascii_band(panel, panel.t))
                lines.append(ascii_band(panel, panel.t))

    with open(os.path.join(OUT, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n(sclk=%.1fMHz, sigma=%.1fRPM) 报告: %s" %
          (args.sclk / 1e6, args.sigma, os.path.join(OUT, "report.txt")))


if __name__ == "__main__":
    main()
