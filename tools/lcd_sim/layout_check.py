# -*- coding: utf-8 -*-
"""界面布局自检: 把"改前(单标签)"和"改后(4 个定宽数字位)"两种界面对同一组数值
各渲染一张图, 逐像素比较 —— 差异必须**只**出现在大字转速所占的那一块
(x 12..116, y 30..73)。用来证明"拆数字位"没有把别的文字/滑条挤歪。

另外顺手把 "320" 的数字带按 1:1 打成 ASCII, 便于在没有看图工具时核对字形。

输出: tools/lcd_sim/out/layout_before.png / layout_after.png
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PIL import Image

from ui import UI, UI_BG, rgb

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


class DummyPanel:
    """布局渲染不需要真面板, 只要 UI 建得起来。"""

    def __init__(self):
        self.t = 0.0

    def set_window(self, *a):
        pass

    def write_pixels_lvgl(self, *a):
        pass


def render(mode, value):
    ui = UI(DummyPanel(), mode=mode)
    if mode == "current":
        ui.digits[0].text = "%d" % value
    else:
        s = "%d" % value                       # 左对齐整组(与固件一致)
        for i in range(len(ui.digits)):
            ui.digits[i].text = s[i] if i < len(s) else ""
    ui.img = Image.new("RGB", (320, 240), rgb(UI_BG))
    ui.redraw((0, 0, 319, 239))
    return ui


def band_ascii(ui, x0=12, x1=130, y0=30, y1=73):
    px = ui.img.load()
    out = []
    for y in range(y0, y1 + 1):
        line = []
        for x in range(x0, x1 + 1):
            r, g, b = px[x, y]
            line.append("#" if (r * 299 + g * 587 + b * 114) // 1000 > 45 else ".")
        out.append("".join(line))
    return "\n".join(out)


def main():
    os.makedirs(OUT, exist_ok=True)
    a = render("current", 320)
    b = render("fixed", 320)
    a.img.save(os.path.join(OUT, "layout_before.png"))
    b.img.save(os.path.join(OUT, "layout_after.png"))

    pa, pb = a.img.load(), b.img.load()
    bad = []
    for y in range(240):
        for x in range(320):
            if pa[x, y] != pb[x, y]:
                if not (12 <= x <= 130 and 30 <= y <= 73):
                    bad.append((x, y))
    print("两版界面不同的像素: %d 个" % sum(1 for y in range(240) for x in range(320)
                                       if pa[x, y] != pb[x, y]))
    print("落在大字转速区(x12..116,y30..73)之外的: %d 个 %s" %
          (len(bad), "-> 布局没被影响" if not bad else "-> 有别的元素被挪动了!"))

    print("\n--- 改后: 大字区 1:1 ASCII ('320') ---")
    print(band_ascii(b))
    print("\n--- 改前: 大字区 1:1 ASCII ('320') ---")
    print(band_ascii(a))


if __name__ == "__main__":
    main()

