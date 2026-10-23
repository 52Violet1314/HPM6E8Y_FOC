# -*- coding: utf-8 -*-
"""LVGL 9.4 内置字体的**忠实**解码器 (只处理了本工程用到的 Montserrat, bpp=4)。

为什么不是"随便解析一下": 之前这个仓库里有人试过解析 lv_font_montserrat_*.c,
得出了"字体数据可能坏了"的错误方向(见 build/sch/font_search.py 的穷举解码)。
真正的事实是:

  1) lv_font_fmt_txt_glyph_dsc_t 的 bitmap_index 是 `uint32_t : 20` + `adv_w : 12`
     两个位域, 但 C 初始化器是按【字段】写的
     ({.bitmap_index = 0, .adv_w = 0, ...}), 所以源码里读到的就是净索引,
     **不需要 >>12**。
  2) 位图是**连续位流**: lv_font_fmt_txt.c:106 的 `stride_in = g_dsc->stride`,
     而 font_dsc 里没有 .stride 字段(LVGL 自带 Montserrat 就是没有) ->
     lv_font_fmt_txt.c:259 `if(fdsc->stride == 0) dsc_out->stride = 0;`
     -> stride_in == 0 -> 行之间【不补齐】, 一个字形是一个连续的 nibble 流。
  3) bpp=4 的 nibble 是**高半个字节先**, 值经过 opa4_table 映射到 0..255
     (lv_font_fmt_txt.c:60 那张表: 0,17,34,...,255; 也就是 v*17)。

本模块按上述三条实现, 并可以直接把渲染结果打成 ASCII 艺术图, 让人用眼睛核对
(不需要看图工具就能验证"字形对不对")。
"""
import re

# lv_font_fmt_txt.c 的 opa4_table
OPA4 = [0, 17, 34, 51, 68, 85, 102, 119, 136, 153, 170, 187, 204, 221, 238, 255]


class Glyph:
    __slots__ = ("idx", "adv", "w", "h", "ox", "oy")

    def __init__(self, idx, adv, w, h, ox, oy):
        self.idx, self.adv, self.w, self.h, self.ox, self.oy = idx, adv, w, h, ox, oy


class LvFont:
    """从 LVGL 生成的 .c 字体文件里读出字形, 并提供与 LVGL 一致的排版。"""

    def __init__(self, path, name=None):
        src = open(path, encoding="utf-8", errors="replace").read()
        self.name = name or path
        self.size = int(re.search(r"\*\s*Size:\s*(\d+)\s*px", src).group(1))
        self.line_height = int(re.search(r"\.line_height\s*=\s*(\d+)", src).group(1))
        self.base_line = int(re.search(r"\.base_line\s*=\s*(\d+)", src).group(1))
        assert re.search(r"\.bpp\s*=\s*4\b", src), "本解码器只实现 bpp=4"
        # font_dsc 里【必须没有】.stride 才是"连续位流"
        dsc = src.split("static const lv_font_fmt_txt_dsc_t font_dsc", 1)
        dsc_body = dsc[1].split("};", 1)[0] if len(dsc) > 1 else src
        self.plain_bitstream = (".stride" not in dsc_body)
        assert self.plain_bitstream, "本解码器假定 stride==0(连续位流)"

        body = src.split("glyph_dsc[] = {", 1)[1].split("\n};", 1)[0]
        self.glyph = []
        for m in re.finditer(
                r"\.bitmap_index\s*=\s*(\d+),\s*\.adv_w\s*=\s*(\d+),\s*"
                r"\.box_w\s*=\s*(\d+),\s*\.box_h\s*=\s*(\d+),\s*"
                r"\.ofs_x\s*=\s*(-?\d+),\s*\.ofs_y\s*=\s*(-?\d+)", body):
            self.glyph.append(Glyph(*(int(g) for g in m.groups())))

        bm = src.split("glyph_bitmap[] = {", 1)[1].split("\n};", 1)[0]
        bm = re.sub(r"/\*.*?\*/", " ", bm, flags=re.S)      # 剥注释(U+XXXX 里有十六进制)
        self.bitmap = [int(v, 16) for v in re.findall(r"0x[0-9a-fA-F]+", bm)]
        assert self.bitmap, "没读到 glyph_bitmap"

        cm = src.split("cmaps[] = {", 1)[1].split("}", 1)[0]
        self.range_start = int(re.search(r"\.range_start\s*=\s*(\d+)", cm).group(1))
        self.glyph_id_start = int(re.search(r"\.glyph_id_start\s*=\s*(\d+)", cm).group(1))
        self.range_length = int(re.search(r"\.range_length\s*=\s*(\d+)", cm).group(1))

    # ---------------- 字形查找 ----------------
    def gid(self, ch):
        cp = ord(ch)
        if not (self.range_start <= cp < self.range_start + self.range_length):
            return None
        gid = self.glyph_id_start + (cp - self.range_start)
        return gid if 0 <= gid < len(self.glyph) else None

    def alpha(self, gid):
        """返回 box_h 行 x box_w 列的 alpha(0..255) —— 与 lv_font_fmt_txt.c 的 bpp=4 分支逐句对应。"""
        g = self.glyph[gid]
        out = [[0] * g.w for _ in range(g.h)]
        if g.w == 0 or g.h == 0:
            return out
        i = 0            # 半个字节计数(lv_font_fmt_txt.c 里的 i)
        pos = g.idx      # bitmap_in
        for y in range(g.h):
            for x in range(g.w):
                i &= 0x1
                if i == 0:
                    out[y][x] = OPA4[(self.bitmap[pos] >> 4) & 0xF]
                else:
                    out[y][x] = OPA4[self.bitmap[pos] & 0xF]
                    pos += 1
                i += 1
            # stride_in == 0 -> 不做行对齐补偿(lv_font_fmt_txt.c:180 的 if(stride_in) 不成立)
        return out

    # ---------------- 排版 ----------------
    def text_width_fx(self, s):
        """LVGL 用 1/16 像素定点累加 adv_w。"""
        total = 0
        for ch in s:
            gid = self.gid(ch)
            if gid is not None:
                total += self.glyph[gid].adv
        return total

    def text_width(self, s):
        return (self.text_width_fx(s) + 8) >> 4

    def glyph_box(self, ch, pen_fx, line_top):
        """返回字形墨迹在屏幕上的位置 (x1,y1,w,h), 与 LVGL 的画字公式一致。

        LVGL: 行顶 = y, 基线 = y + line_height - base_line,
              字形位图【顶边】 = 基线 - ofs_y - box_h, 左边 = (pen_fx>>4) + ofs_x。
        """
        gid = self.gid(ch)
        if gid is None:
            return None
        g = self.glyph[gid]
        baseline = line_top + self.line_height - self.base_line
        x1 = (pen_fx >> 4) + g.ox
        y1 = baseline - g.oy - g.h
        return (x1, y1, g.w, g.h)

    def draw(self, put, line_top, pen_x_px, s, color):
        """put(x, y, rgb, alpha) 由调用者提供(写进面板 GRAM / 预览图)。

        返回结束时的笔位(像素)。pen 用 1/16 定点, 与 LVGL 一致。
        """
        pen_fx = pen_x_px << 4
        for ch in s:
            gid = self.gid(ch)
            if gid is None:
                continue
            g = self.glyph[gid]
            if g.w and g.h:
                a = self.alpha(gid)
                bx, by, _, _ = self.glyph_box(ch, pen_fx, line_top)
                for row in range(g.h):
                    for col in range(g.w):
                        al = a[row][col]
                        if al:
                            put(bx + col, by + row, color, al)
            pen_fx += g.adv
        return (pen_fx + 8) >> 4


def ascii_art(font, s, threshold=128):
    """把一行字打成 ASCII, 便于在没有看图工具时核对字形与排版。"""
    w = font.text_width(s)
    h = font.line_height
    grid = [["." for _ in range(max(w, 1))] for _ in range(h)]
    put = lambda x, y, c, a: grid[y].__setitem__(x, "#" if a >= threshold else "+") \
        if 0 <= y < h and 0 <= x < w else None
    font.draw(put, 0, 0, s, 0xFFFFFF)
    return "\n".join("".join(r) for r in grid)


if __name__ == "__main__":
    import sys
    d = "hpm_sdk_localized_for_hpm6e00YZ/middleware/lvgl/src/font/"
    for size in (12, 20, 40):
        f = LvFont("%slv_font_montserrat_%d.c" % (d, size))
        print("=== montserrat_%d  line_height=%d base_line=%d glyphs=%d plain=%s ===" %
              (size, f.line_height, f.base_line, len(f.glyph), f.plain_bitstream))
        txt = "0123456789" if size == 40 else "SPEED 320 RPM"
        if size == 40:
            for ch in "0138":
                print("--- '%s' box=%s ---" % (ch, f.glyph[f.gid(ch)].__dict__ if hasattr(f.glyph[f.gid(ch)], '__dict__') else
                      (f.glyph[f.gid(ch)].w, f.glyph[f.gid(ch)].h, f.glyph[f.gid(ch)].ox, f.glyph[f.gid(ch)].oy)))
                g = f.glyph[f.gid(ch)]
                a = f.alpha(f.gid(ch))
                for row in a:
                    print("   " + "".join("#" if v >= 128 else ("+" if v else ".") for v in row))
        else:
            print(ascii_art(f, txt))
        print("  width('%s')=%d px" % (txt, f.text_width(txt)))
