# -*- coding: utf-8 -*-
"""ILI9341 类控制器的 GRAM + 扫描 + SPI1 写通路时序模型(用来在 PC 上重现"撕裂")。

核心事实: 面板**一直在自己扫**, 与我们写不写 GRAM 无关。某一"显示单元"在时刻 t
显示的内容 = t 之前最后一次扫到它时 GRAM 里的值。于是"我们正在改数字的那 1~3ms"
里被扫过的单元, 这一帧还是旧内容, 下一帧才是新内容 —— 肉眼看就是数字被撕开。

扫描轴是可选的, 因为从代码/原理图里**看不出**这块屏到底沿哪个轴扫:
  · axis='y': 扫描沿 y(每行一次), 一次窗格写会让"上下半截分别是新旧"-> 横向裂口
  · axis='x': 扫描沿 x(每列一次, ILI9341 横屏 MV=1 时就是这样), 我们按行写像素,
              于是左列是旧、右列是新 -> 数字被"从左往右擦过去"
两种几何都会把同一时刻的两个数字拼在一起, 所以结论(更新要少、要短)与轴向无关;
报告里两个都会给。

时序参数全部取自本仓库:
  · SCLK        目标 32MHz(LCD_SPI_SCLK_FREQ), 实际由 LCD_SPIInit() 选最接近的偶数分频
  · 命令字节    LCD_WR_REG/LCD_WR_DATA 每字节一次 spi_transfer(), 内含 SPI 软复位
                (TRANSCTRL + TXFIFORST/RXFIFORST/SPIRST)+等复位+等 SPIACTIVE 落
  · 像素块      LCD_WritePixels 每 256 像素(512B)一块: 逐像素取反+换字节拷进
                s_lcd_tx_buffer, 再 DMA(SPI1_TX->HDMA0)发; 每块另有固定开销
"""
from bisect import bisect_right


class Timing:
    def __init__(self, sclk_hz=25_000_000, cmd_byte_s=1.0e-6, chunk_overhead_s=4.0e-6,
                 chunk_pixels=256):
        self.sclk_hz = sclk_hz
        self.byte_s = 8.0 / sclk_hz
        self.cmd_byte_s = cmd_byte_s + self.byte_s
        self.chunk_overhead_s = chunk_overhead_s
        self.chunk_pixels = chunk_pixels

    def window_cmd_s(self):
        """LCD_SetWindows = 3 个命令字节(2A/2B/2C) + 8 个参数字节 = 11 字节。"""
        return 11 * self.cmd_byte_s


class ILI9341:
    def __init__(self, timing, w=320, h=240, frame_hz=70.0, axis="y"):
        self.tm = timing
        self.w, self.h = w, h
        self.frame_s = 1.0 / frame_hz
        self.axis = axis
        self.unit = h if axis == "y" else w          # 一帧里被扫过的"行/列"数
        self.unit_s = self.frame_s / self.unit

        self.t = 0.0
        self.gram_base = [[0] * w for _ in range(h)]
        self.gram = [[0] * w for _ in range(h)]
        # 每行: 时间升序的 [(时刻, x0, [像素...]), ...]
        self.events = [[] for _ in range(h)]
        self.window = (0, 0, w - 1, h - 1)
        self.stats = {"windows": 0, "pixels": 0, "chunks": 0}

    # ---------------- 扫描 ----------------
    def last_pass_before(self, unit, t):
        first = unit * self.unit_s
        if t < first:
            return None
        return first + int((t - first) / self.frame_s) * self.frame_s

    def _row_state(self, r, t):
        """第 r 行在时刻 t 的 GRAM 内容(按事件表重放)。"""
        row = list(self.gram_base[r])
        for (te, x0, px) in self.events[r]:
            if te > t:
                break
            row[x0:x0 + len(px)] = px
        return row

    def _pix_at(self, r, x, limit):
        """第 r 行第 x 列在 t<=limit 时的 GRAM 值(从最新事件往回找, 通常 1~2 步)。"""
        for (te, ex0, px) in reversed(self.events[r]):
            if te <= limit:
                if ex0 <= x < ex0 + len(px):
                    return px[x - ex0]
        return self.gram_base[r][x]

    def visible_rows(self, rows, t, x0=0, x1=None):
        """返回 {row: [该行 x0..x1 的可见像素]} —— 面板此刻真正显示的内容。"""
        x1 = self.w - 1 if x1 is None else x1
        out = {}
        if self.axis == "y":
            for r in rows:
                pt = self.last_pass_before(r, t)
                limit = pt if pt is not None else -1.0
                out[r] = [self._pix_at(r, x, limit) for x in range(x0, x1 + 1)]
            return out

        # 沿 x 扫描: 每一列 x 在各自时刻被"拍", 拍的是 GRAM 的第 x 列(所有行)
        pass_t = [self.last_pass_before(x, t) for x in range(x0, x1 + 1)]
        for r in rows:
            out[r] = [0] * (x1 - x0 + 1)
        for j, x in enumerate(range(x0, x1 + 1)):
            pt = pass_t[j]
            limit = pt if pt is not None else -1.0
            for r in rows:
                out[r][j] = self._pix_at(r, x, limit)
        return out

    # ---------------- 驱动侧动作 ----------------
    def set_window(self, x0, y0, x1, y1):
        self.t += self.tm.window_cmd_s()
        self.window = (x0, y0, x1, y1)
        self.stats["windows"] += 1

    def write_pixels_lvgl(self, buf, npx):
        """buf = LVGL 原生 RGB565 小端字节流; 与 LCD_WritePixels() 逐句对应。"""
        x0, y0, x1, y1 = self.window
        ww = x1 - x0 + 1
        tm = self.tm
        ch = tm.chunk_pixels
        t0 = self.t
        self.t = t0 + tm.chunk_overhead_s * ((npx + ch - 1) // ch) + 2 * tm.byte_s * npx
        self.stats["chunks"] += (npx + ch - 1) // ch
        self.stats["pixels"] += npx

        nh = (npx + ww - 1) // ww
        for row in range(nh):
            a = row * ww
            b = min(a + ww, npx)
            seg = [buf[i * 2] | (buf[i * 2 + 1] << 8) for i in range(a, b)]
            idx_last = b - 1
            t_row = t0 + tm.chunk_overhead_s * (idx_last // ch + 1) \
                + 2 * tm.byte_s * (idx_last + 1)
            gy = y0 + row
            self.gram[gy][x0:x0 + len(seg)] = seg
            self.events[gy].append((t_row, x0, seg))

    # ---------------- 基线 ----------------
    def prime(self, image_rows):
        self.gram_base = [list(r) for r in image_rows]
        self.gram = [list(r) for r in image_rows]
        self.events = [[] for _ in range(self.h)]

    # ---------------- 统计辅助 ----------------
    def write_span(self, row=None):
        ev = self.events[row] if row is not None else [e for r in self.events for e in r]
        return (ev[0][0], ev[-1][0]) if ev else (0.0, 0.0)
