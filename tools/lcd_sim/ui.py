# -*- coding: utf-8 -*-
"""LVGL-lite: 只实现本工程界面用到的那一小部分语义, 但**语义要与 LVGL 一致**。

对齐的是这三条(它们决定了"什么时候往屏上写了多少像素"):
  1) lv_label_set_text() -> 先把【旧】位置标脏, 改完尺寸再把【新】位置标脏
     (lv_label_refr_text 里两处 lv_obj_invalidate) -> 脏区 = 旧框 ∪ 新框。
  2) lv_obj_set_style_text_color() 在 LVGL 9 里**不做"值没变就跳过"**
     (lv_obj_style.c:378 lv_obj_set_local_style_prop 无条件 lv_obj_refresh_style)
     -> 只要每拍都调, 这个对象每拍都被标脏。这正是 lcd_ui.c 现在在做的事。
  3) 部分刷新(PARTIAL)模式下, 脏区按 draw buffer 切成横条
     (lv_refr.c:784 get_max_row = data_size / (area_w*2), 再按 area_h 截断),
     每条一次 LCD_SetWindows + LCD_WritePixels。draw buffer = 320*20*2 = 12800B。

界面布局/字体/颜色全部照抄 src/lcd/lcd_ui.c(坐标一字不改), 这样模拟出来的像素
就是屏上真正会出现的像素。
"""
from PIL import Image, ImageDraw

from lvfont import LvFont

FONT_DIR = "hpm_sdk_localized_for_hpm6e00YZ/middleware/lvgl/src/font/"
BUF_BYTES = 320 * 20 * 2          # lcd_app.c: draw_buffer

UI_BG, UI_PANEL, UI_TRACK = 0x0E1621, 0x16212E, 0x22303F
UI_ACCENT, UI_TEXT, UI_DIM = 0x35C8F0, 0xE8F1F8, 0x8FA3B4
UI_WARN, UI_BAD, UI_OK = 0xFFB300, 0xFF5252, 0x5BE37D


def rgb(h):
    return ((h >> 16) & 0xFF, (h >> 8) & 0xFF, h & 0xFF)


def to565(px):
    r, g, b = px
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


class Obj:
    z = 0

    def box(self):
        raise NotImplementedError

    def draw(self, img):
        raise NotImplementedError


class Label(Obj):
    def __init__(self, text, font, color, x=0, y=0, align="tl",
                 fixed_w=None, text_align="left"):
        self.text, self.font, self.color = text, font, color
        self.x, self.y, self.align = x, y, align
        self.fixed_w, self.text_align = fixed_w, text_align

    def box(self):
        w = self.fixed_w or self.font.text_width(self.text)
        h = self.font.line_height * (self.text.count("\n") + 1)
        if self.align == "tl":
            x0 = self.x
        elif self.align == "tr":
            x0 = 320 - self.x - w
        elif self.align == "bl":
            x0 = self.x
        else:
            raise ValueError(self.align)
        y0 = self.y if self.align != "bl" else 240 + self.y - h
        return (x0, y0, x0 + max(w, 1) - 1, y0 + h - 1)

    def draw(self, img):
        x0, y0, x1, y1 = self.box()
        px = img.load()
        c = rgb(self.color)

        def put(x, y, _c, a):
            if not (0 <= x < img.width and 0 <= y < img.height):
                return
            bg = px[x, y]
            px[x, y] = tuple((cc * a + bb * (255 - a)) // 255 for cc, bb in zip(c, bg))

        for i, line in enumerate(self.text.split("\n")):
            lt = y0 + i * self.font.line_height
            if self.text_align == "right" and self.fixed_w:
                w = self.font.text_width(line)
                self.font.draw(put, lt, x0 + self.fixed_w - w, line, self.color)
            else:
                self.font.draw(put, lt, x0, line, self.color)


class Rounded(Obj):
    def __init__(self, x0, y0, x1, y1, radius, color):
        self.r = (x0, y0, x1, y1)
        self.radius, self.color = radius, color

    def box(self):
        return self.r

    def draw(self, img):
        d = ImageDraw.Draw(img)
        d.rounded_rectangle(list(self.r), radius=self.radius, fill=rgb(self.color))


class UI:
    """界面 + 脏区 + 部分刷新(flush)模型。"""

    def __init__(self, panel, mode="current", base_line_fonts=(12, 14, 16, 20, 40)):
        self.panel = panel
        self.mode = mode
        self.f = {n: LvFont("%slv_font_montserrat_%d.c" % (FONT_DIR, n)) for n in base_line_fonts}
        self.img = Image.new("RGB", (320, 240), rgb(UI_BG))
        self.objs = []
        self.invalid = []
        self.flush_log = []
        self._build()

    # ---------------- 建界面 (照抄 lcd_ui.c) ----------------
    def _build(self):
        f = self.f
        o = self.objs
        o.append(Label("SPEED", f[20], UI_TEXT, 10, 8))
        o.append(Label("MOTOR", f[12], UI_DIM, 10, 13, "tr"))
        self.unit = Label("RPM", f[20], UI_DIM, 150, 50)
        o.append(self.unit)
        o.append(Label("TARGET", f[14], UI_TEXT, 10, 86))
        self.target_v = Label("600 RPM", f[16], UI_ACCENT, 10, 84, "tr")
        o.append(self.target_v)
        o.append(Label("0", f[12], UI_DIM, 10, 146))
        o.append(Label("600", f[12], UI_DIM, 10, 146, "tr"))
        # 滑条: 268x26 @TOP_MID(0,106); knob 20x34 居中 -> y 102..135
        o.append(Rounded(26, 106, 293, 131, 8, UI_TRACK))
        o.append(Rounded(26, 106, 293, 131, 8, UI_ACCENT))
        o.append(Rounded(283, 102, 303, 135, 17, 0xFFFFFF))
        self.fb = Label("FOC RUNNING", f[14], UI_OK, 10, 164)
        o.append(self.fb)
        self.readback = Label("SPEED LOOP OWNS IQ", f[12], UI_WARN, 10, 182)
        o.append(self.readback)
        o.append(Label("SLIDE = SET TARGET\nAPPLIED AT 1kHz", f[12], UI_DIM, 10, -6, "bl"))

        if self.mode == "current":
            # lcd_ui.c 现状: 一个大字标签
            self.digits = [Label("320", f[40], UI_ACCENT, 12, 30)]
        else:
            # 改法: 4 个定宽数字位(每个 28px 宽, 右对齐) —— 只有变的那一位会被标脏/刷屏
            pitch = 28   # 与固件 UI_RPM_CELL_W 一致(实测最宽数字步进 27px, 取 28)
            self.pitch = pitch
            init = "320"
            self.digits = [Label("", f[40], UI_ACCENT, 12 + i * pitch, 30,
                                 fixed_w=pitch, text_align="right")
                           for i in range(4)]
            # 左对齐的整组数字(与固件 ui_rpm_render 一致): 不补前导空格
            for i, ch in enumerate(init):
                self.digits[i].text = ch
        self.objs.extend(self.digits)
        self.redraw((0, 0, 319, 239))

    # ---------------- 脏区 ----------------
    def invalidate(self, area):
        """标脏。LVGL 是**按对象**记脏区的(lv_refr.c 只把相交的区域合并), 所以这里
        也是一个列表: 数字带和状态行不重叠 -> 两条独立的 flush, 而不是一大块。
        (合并成一大块会严重高估一次刷屏的数据量, 撕裂的位置也会算错。)"""
        a = (max(0, area[0]), max(0, area[1]), min(319, area[2]), min(239, area[3]))
        if a[2] < a[0] or a[3] < a[1]:
            return
        self.invalid.append(list(a))
        # 只合并相交的(与 lv_refr.c 的 lv_area_is_on/join 语义一致)
        merged = True
        while merged:
            merged = False
            for i in range(len(self.invalid)):
                for j in range(i + 1, len(self.invalid)):
                    a, b = self.invalid[i], self.invalid[j]
                    if a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]:
                        self.invalid[i] = [min(a[0], b[0]), min(a[1], b[1]),
                                           max(a[2], b[2]), max(a[3], b[3])]
                        del self.invalid[j]
                        merged = True
                        break
                if merged:
                    break

    def set_text(self, lbl, text):
        old = lbl.box()
        if lbl.text == text:
            return
        lbl.text = text
        new = lbl.box()
        self.invalidate((min(old[0], new[0]), min(old[1], new[1]),
                         max(old[2], new[2]), max(old[3], new[3])))

    def touch(self, lbl):
        """模拟 LVGL 里"无条件 set_style_text_color" -> 对象被标脏。"""
        self.invalidate(lbl.box())

    # ---------------- 画 ----------------
    def redraw(self, area):
        d = ImageDraw.Draw(self.img)
        d.rectangle(list(area), fill=rgb(UI_BG))
        for ob in self.objs:
            x0, y0, x1, y1 = ob.box()
            if x0 <= area[2] and area[0] <= x1 and y0 <= area[3] and area[1] <= y1:
                ob.draw(self.img)

    def flush_pending(self):
        """lv_refr_now(): 逐块把脏区按 draw buffer 切成横条, 每条一次窗格写。"""
        if not self.invalid:
            return 0
        areas = self.invalid
        self.invalid = []
        n = 0
        for area in areas:
            x0, y0, x1, y1 = area
            w = x1 - x0 + 1
            max_row = max(1, min(BUF_BYTES // (w * 2), y1 - y0 + 1))
            row = y0
            while row <= y1:
                sy1 = min(row + max_row - 1, y1)
                sub = (x0, row, x1, sy1)
                self.redraw(sub)
                buf = self._rgb565_le(sub)
                self.panel.set_window(*sub)
                self.panel.write_pixels_lvgl(buf, (sub[2] - sub[0] + 1) * (sub[3] - sub[1] + 1))
                self.flush_log.append((self.panel.t, sub))
                n += 1
                row = sy1 + 1
        return n

    def _rgb565_le(self, area):
        px = self.img.load()
        x0, y0, x1, y1 = area
        out = []
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                v = to565(px[x, y])
                out.append(v & 0xFF)
                out.append(v >> 8)
        return out

    # ---------------- 采样: 面板此刻"显示"的数字区是什么 ----------------
    def visible_digit_rows(self, t=None):
        rows = range(self.digit_band()[1], self.digit_band()[3] + 1)
        snap = self.panel.snapshot(t, rows)
        return snap

    def digit_band(self):
        return (12, 30, 12 + (4 * self.pitch if self.mode != "current" else 104) - 1, 73)

