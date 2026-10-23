/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * LCD: LVGL 电机界面实现 (转速显示 + 速度条设定目标转速)
 *
 * 布局与数据流见 lcd_ui.h。
 *
 * 【RTOS 下这是谁在跑】上 RTOS 之后本模块只由【显示任务】调用
 * (app_rtos.c 的 app_disp_task -> lcd_app_task_handler -> lcd_ui_refresh),
 * 约 200Hz。滑条回调也在这个任务里, 所以这里碰 LVGL 不需要自己加锁 ——
 * 锁由调用者 (lcd_app_task_handler) 持有, 见 lcd_app.h 的说明。
 * 从本文件【只能】调 app 的跨任务安全接口 (app_speed_*), 绝不能直接读
 * s_ctrl / s_encoder 这类控制任务的内部状态。
 *
 * 【为什么刷新要节流】lv_label_set_text_fmt() 会重建文本并触发重绘。显示任务
 * 5ms 来一次, 全量刷新会白白吃掉 SPI 刷屏带宽 (刷屏很慢, 是显示任务里最大头
 * 的开销)。所以:
 *   - 外层 20ms 节流 (转速本来就是低通估算值, 50Hz 显示足够);
 *   - 文本内容没变就不调 set_text (LVGL 内部对相同字符串其实有短路, 但显式跳过
 *     更省事, 也避免每拍走一遍 snprintf)。
 */
#include <string.h>
#include <stdio.h>
#include <math.h>       /* expf: 大字显示滤波的系数 */

#include "lvgl.h"

#include "lcd_ui.h"
#include "app.h"            /* app_speed_get_* / APP_FOC_CLOSED_LOOP_ENABLE */

/* 颜色 */
#define UI_BG            0x0E1621
#define UI_PANEL         0x16212E
#define UI_TRACK         0x22303F
#define UI_ACCENT        0x35C8F0
#define UI_TEXT          0xE8F1F8
#define UI_DIM           0x8FA3B4
#define UI_WARN          0xFFB300
#define UI_BAD           0xFF5252
#define UI_OK            0x5BE37D

/* 刷新节流 (ms) —— 【真时间】。
 * 原来这里是 `s_ui.tick_ms += APP_MAIN_LOOP_MS`(APP_MAIN_LOOP_MS = 1) 来数拍,
 * 而显示任务其实是 5ms 一拍(CORE1_DISP_PERIOD_MS = 5) -> "20ms 节流"实际是 100ms,
 * 注释和代码对不上。改成用 lv_tick_get() 的真时间, 这样"多少 ms"就是多少 ms。 */
#define UI_REFRESH_MS        20U

/* ==================== 大字转速: 为什么要做"显示滤波 + 去抖" ====================
 * 现象: 屏上转速数字看着是乱码/被撕开。
 *
 * 原因有两条, 都是"刷屏"这件事本身引起的, 与数值算得对不对无关:
 *  1) 实测转速(编码器 + 速度估计)的**末位一直在跳**。界面原来只要整数变一位就
 *     重画大字 -> 每秒改 7~10 次。而 ILI9341 没有 TE 同步线(J10 上就没有),
 *     面板是自由扫描的: 我们改 GRAM 的那 ~3ms 里被扫过的行, 这一帧还是旧数字,
 *     下一帧才是新的 —— 于是屏上是"两个数字拼在一起"。改得越勤, 越像乱码。
 *  2) 大字原来是一个整标签, 改一位就要重刷 110x44 的整块(9.7KB, 25MHz 下 3.1ms),
 *     比面板扫过这条带的时间(2.6ms)还长 -> 几乎每次改写都会被撕开。
 *
 * 对策(三条, 缺一条效果都差很多):
 *  · 大字拆成 4 个定宽数字位(UI_RPM_CELLS x UI_RPM_CELL_W): 只有变的那一位被
 *    标脏/刷屏 -> 一次只写 28x44(2.5KB, 0.78ms), 远小于面板扫这条带的时间;
 *    数字位用等宽数字库(UI_FONT_DIGITS), 保证每位墨迹落在同一个位置;
 *  · 显示值做 EMA 滤波(UI_RPM_FILTER_MS): 屏上该显示"读数", 不是瞬时采样。
 *    噪声被平均掉, 数字就稳了; 真变化 0.3s 内照样跟上;
 *  · 去抖 + 限速: 与显示值差 >= UI_RPM_DEADBAND 才改, 两次改写至少隔
 *    UI_RPM_MIN_GAP_MS, 另有 UI_RPM_FORCE_MS 心跳兜底。
 * 【注意】这一整套只影响"显示", 控制环一个字节都没动。
 * ========================================================================== */
/* 大字数字用的【等宽数字字体】: 见 lv_font_digits_40.c, 由 tools/_mk_digitfont.py 生成。
 *
 * 【为什么不能用 LVGL 自带的 montserrat_40】它是比例字体, 数字的字形步进实测是
 *   '-' 15.31  '1' 14.81  '2' 22.94  '3' 22.88  '4' 26.75
 *   '5' 22.94  '6' 24.69  '7' 23.94  '8' 25.75  '9' 24.69   (px)
 * 最宽 26.75 / 最窄 14.81, 差 11.94px。把这种字形塞进 28px 的定宽格子再右对齐,
 * 墨迹的左边沿会在 2..13px 之间乱跳、墨迹中心在 14..18.5px 之间跳 —— 每改一位数字,
 * 这一位就横向弹一下, 看起来和撕裂一样花。等宽数字库把 10 个数字的步进统一成 28px、
 * 墨迹居中(中心跳动 <= 0.5px), 于是"改一位只影响这一位", 其它三位一个像素都不动。
 * 验证: python tools/_mk_digitfont_check.py (算改前/改后跳动量 + 出对比图) */
extern const lv_font_t lv_font_digits_40;
#define UI_FONT_DIGITS      (&lv_font_digits_40)
/* 想退回比例字体做对照: 改成 (&lv_font_montserrat_40), 并把 lv_conf.h 的
 * LV_FONT_MONTSERRAT_40 打开(现在界面已经不用它了, 关掉可以省 flash)。 */

#define UI_RPM_CELLS         (4)        /* ±999 -> "-999".."999" 四位 */
/* 数字位宽(px) = 数字库的步进(advance), 两者必须一致, 否则会被 LVGL 换行/裁掉。
 * lv_font_digits_40 里所有数字 adv_w = 28px、最宽墨迹 26px -> 取 28, 留 2px 余量。
 * 数字组【右对齐】, 右边界恒为 12 + 4*28 = 124。 */
#define UI_RPM_CELL_W        (28)
#define UI_RPM_RIGHT_X       (12 + (UI_RPM_CELLS * UI_RPM_CELL_W))   /* = 124 */
/* "RPM" 单位: 紧贴数字组右边界后面 8px。
 * 【为什么不再钉在 x=150】数字组右对齐之后右边界恒定, 单位必须贴着它才不会"飘";
 * 原来 x=150 是给左对齐的最长数字(-999, 右端 123)留的, 于是 1~3 位的读数
 * (0~600 的正常工况)与单位之间空出 56~110px, 看着是两块不相干的东西。 */
#define UI_UNIT_X            (UI_RPM_RIGHT_X + 8)                    /* = 132 */
/* 【y 为什么是 48 而不是 50: 基线对齐】数字位字体 line_height=44 / base_line=8,
 * 盒顶 30 -> 基线 y = 30 + 44 - 8 = 66;单位是 montserrat_20(line_height=22 /
 * base_line=4), 盒顶 y -> 基线 = y + 22 - 4 = y + 18。令 y + 18 = 66 -> y = 48。
 * 原来的 50 让单位基线落在 68, 比数字低 2px —— 既不居中也不贴底, 就是"没对齐"。 */
#define UI_UNIT_Y            (48)
#define UI_RPM_FILTER_MS     (250.0f)   /* 显示滤波时间常数 (ms) */
#define UI_RPM_DEADBAND      (3)        /* 差这么多(含)以上才算真变化 (RPM) */
#define UI_RPM_MIN_GAP_MS    (100U)     /* 两次改写大字的最小间隔 (ms) */
#define UI_RPM_FORCE_MS      (1000U)    /* 心跳: 至少这么久跟一次真实值 (ms) */

typedef struct {
    lv_obj_t *lbl_digit[UI_RPM_CELLS];  /* 大字: 实测转速(每位一个定宽标签) */
    lv_obj_t *lbl_unit;         /* "RPM" 单位 */
    lv_obj_t *lbl_target;       /* "目标 300 RPM" */
    lv_obj_t *slider;           /* 速度条: 目标转速 */
    lv_obj_t *lbl_fb;           /* 反馈行: 电机状态 / 使用说明 */
    lv_obj_t *lbl_readback;     /* 实际下发值 (只在"未接管"时提示) */
    lv_obj_t *mode_sw;          /* 【模式开关】位置模式(位置环) <-> 速度模式(滑条) */
    lv_obj_t *lbl_mode;         /* 开关左边的 "POS" 提示 */
    /* 【位置滑条】和速度条占同一块地方, 按模式二选一显示:
     *   位置模式(POS ON) -> 显示 pos_slider, 标题改 "POSITION mrad";
     *   速度模式(POS OFF)-> 显示 slider,     标题改 "TARGET"。
     * 【为什么共用一块地方】320x240 已经被大字/刻度/开关/反馈行占满了, 再塞一根
     * 26px 高的滑条必然压到别的东西上。而两根滑条本来就互斥(哪个环在跑就用哪个),
     * 所以轮流用同一块区域最自然。 */
    lv_obj_t *pos_slider;       /* 位置条: 目标位置 (mrad, -3141..+3141) */
    lv_obj_t *lbl_cap;          /* 滑条标题 (随模式改文字) */
    lv_obj_t *lbl_min;          /* 左端刻度 (随模式改文字) */
    lv_obj_t *lbl_max;          /* 右端刻度 (随模式改文字) */

    int32_t   target_rpm;       /* 滑条设定值 (RPM, 无符号) */
    int32_t   target_mrad;      /* 位置条设定值 (mrad) */
    uint32_t  last_refr_ms;     /* 上次界面逻辑刷新(lv_tick_get, 真时间) */
    uint8_t   mode_on;          /* 上次同步过的位置环状态(0xFF = 还没同步过) */
    uint8_t   pos_shown;        /* 位置条当前是否显示 (0xFF = 还没同步过) */

    /* ---- 大字转速的显示状态 ---- */
    float     rpm_filt;         /* EMA 滤波后的显示用转速 */
    int32_t   rpm_shown;        /* 当前屏上显示的整数 */
    uint32_t  rpm_ms;           /* 上次改大字的时刻 */
    uint32_t  rpm_color;        /* 大字当前颜色(去重: 颜色没变就不写) */
    char      digit[UI_RPM_CELLS];  /* 每一位当前字符('\0' = 空位) */

    /* 【文本缓存: 必须的, 不是优化】
     * lv_label_set_text_fmt() 进函数第一件事就是 lv_obj_invalidate(obj), 且它会
     * 释放旧字符串再重新格式化 —— 也就是说【内容没变也照样标脏 + 走一遍
     * vsnprintf + 触发一轮重绘】。上 RTOS 之后重绘意味着显示任务吃掉几毫秒 CPU
     * (SPI 是轮询/DMA 阻塞写的), 那是会直接影响刷屏节奏的。所以每次写文本前先跟自己
     * 缓存的上一次内容比一下, 一样就整个跳过。
     * 【同样重要】lv_obj_set_style_text_color() 在 LVGL 9 里【不做】"值没变就跳过"
     * (lv_obj_style.c 的 lv_obj_set_local_style_prop 无条件 lv_obj_refresh_style),
     * 所以颜色也必须自己缓存 —— 状态行原来每 5ms 无条件写一次颜色, 等于让这一行
     * 每 10ms 被重刷一遍(实测约 110ms/秒 的 SPI 时间花在一条没变的文字上)。*/
    uint32_t  fb_color;         /* 状态行当前颜色 */
    char      fb[40];
    char      readback[40];
    char      target[20];
} ui_t;

static ui_t s_ui;

/* ---- 文本写入: 内容没变就什么都不做 ---- */
static void ui_text(lv_obj_t *lbl, char *cache, uint32_t cache_sz, const char *txt)
{
    if (strcmp(cache, txt) == 0) {
        return;
    }
    strncpy(cache, txt, cache_sz - 1U);
    cache[cache_sz - 1U] = '\0';
    lv_label_set_text(lbl, txt);
}

static void ui_text_fmt(lv_obj_t *lbl, char *cache, uint32_t cache_sz,
                        const char *fmt, int32_t v1, int32_t v2)
{
    char tmp[40];

    if (v2 >= 0) {
        (void)snprintf(tmp, sizeof(tmp), fmt, (int)v1, (int)v2);
    } else {
        (void)snprintf(tmp, sizeof(tmp), fmt, (int)v1);
    }
    ui_text(lbl, cache, cache_sz, tmp);
}

/* ---- 文本颜色: 颜色没变就什么都不做 ----
 * LVGL 9 的 lv_obj_set_style_text_color() 内部是**无条件** lv_obj_refresh_style()
 * (lv_obj_style.c 的 lv_obj_set_local_style_prop), 也就是"值没变也标脏"。
 * 状态行原来每拍都写一次颜色 -> 那一行每 10ms 被重刷一遍。 */
static void ui_text_color(lv_obj_t *lbl, uint32_t color, uint32_t *cache)
{
    if (*cache == color) {
        return;
    }
    *cache = color;
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
}

/* ==================== 大字转速: 按"位"写 ====================
 * 【为什么要按位写】一个整标签改一位就要重刷 110x44(9.7KB, 25MHz 下 3.1ms), 比
 * 面板扫过这条带的时间(2.6ms)还长 -> 几乎每次都会被撕开; 拆成 4 个定宽数字位后,
 * 改一位只刷 26x44(2.3KB, 0.73ms), 明显短于面板扫一条带的时间, 撕裂概率大降。
 * 代价: 数字宽度是等宽的(定宽位), 不是比例字体那种"1"更窄的排版 —— 表头这样更整齐。 */
static void ui_rpm_render(int32_t v)
{
    char buf[8];
    int  n;
    int  i;

    /* 【右对齐的整组数字: 个位永远落在最后一个格子】为什么不是左对齐:
     * "RPM" 单位是一个独立的标签, 位置钉死在 UI_UNIT_X —— 如果数字组左对齐(从 x=12
     * 开始画), 那么位数一变, 数字右端到单位的距离就会从 113px(1 位)摆到 29px(4 位),
     * 也就是单位会"飘": 0~600 这种 1~3 位的正常读数下, 单位和数字之间空着 56~113px,
     * 看着是两块不相干的东西。
     * 改成右对齐后, 数字组的右边界恒定在 12+4*28 = 124, 单位紧贴其后, 间距永远 8px,
     * 像仪表的读数那样。位数变化(99 -> 100)两种排法的改动格子数一样多(都是 3 个),
     * 所以这个改动不会让刷屏变多。 */
    (void)snprintf(buf, sizeof(buf), "%d", (int)v);
    n = (int)strlen(buf);
    if (n > UI_RPM_CELLS) {             /* 理论上到不了(上游已限幅 ±999), 兜底 */
        int skip = n - UI_RPM_CELLS;    /* 超长时保留最低位, 而不是高位 */
        (void)memmove(buf, &buf[skip], (size_t)UI_RPM_CELLS);
        n = UI_RPM_CELLS;
        buf[n] = '\0';
    }
    for (i = 0; i < UI_RPM_CELLS; i++) {
        /* 让 buf 的最后一个字符落在最后一个格子里 */
        int  src = i - (UI_RPM_CELLS - n);
        char one = (src >= 0) ? buf[src] : '\0';

        if (s_ui.digit[i] != one) {         /* 只有变的那一位才 lv_label_set_text */
            s_ui.digit[i] = one;
            if (one == '\0') {
                lv_label_set_text(s_ui.lbl_digit[i], "");
            } else {
                char t[2];

                t[0] = one;
                t[1] = '\0';
                lv_label_set_text(s_ui.lbl_digit[i], t);
            }
        }
    }
}

/* 认下一次显示值: 改数字 + 记录时刻 */
static void ui_rpm_commit(int32_t v, uint32_t now)
{
    s_ui.rpm_shown = v;
    s_ui.rpm_ms    = now;
    ui_rpm_render(v);
}

/* 大字刷新: EMA 滤波 + 死区 + 限速 + 心跳(判据见文件上方的说明) */
static void ui_rpm_update(float rpm, uint32_t now)
{
    float   alpha;
    int32_t v;
    int32_t diff;

    /* 显示滤波: alpha = 1 - exp(-Ts/tau)。Ts 用真实节流周期 UI_REFRESH_MS。
     * 用浮点只是为了写清楚; 只在 20ms 一拍里算一次, 对控制环零影响。
     * 【这一步是"数字不跳"的关键】屏上该显示"读数", 而不是瞬时采样: 转速估计的
     * 末位噪声(几个 RPM)被平均掉, 真变化 0.3s 内照样跟上。 */
    alpha = 1.0f - expf(-(float)UI_REFRESH_MS / UI_RPM_FILTER_MS);
    s_ui.rpm_filt += (rpm - s_ui.rpm_filt) * alpha;

    v = (int32_t)((s_ui.rpm_filt >= 0.0f) ? (s_ui.rpm_filt + 0.5f)
                                          : (s_ui.rpm_filt - 0.5f));
    if (v > 999) {
        v = 999;
    } else if (v < -999) {
        v = -999;
    }

    diff = v - s_ui.rpm_shown;
    if (diff < 0) {
        diff = -diff;
    }

    /* 改屏的条件: 差到死区以上(真变化), 或者心跳时间到了(长期小偏差也跟上);
     * 并且距上次改屏至少 UI_RPM_MIN_GAP_MS —— 加速时数字一秒能变几十次,
     * 那样既看不清、又一直在撕屏。 */
    if (((diff >= UI_RPM_DEADBAND) ||
         (((uint32_t)(now - s_ui.rpm_ms) >= UI_RPM_FORCE_MS) && (diff > 0))) &&
        ((uint32_t)(now - s_ui.rpm_ms) >= UI_RPM_MIN_GAP_MS)) {
        ui_rpm_commit(v, now);
    }
}

/* 滑条回调 (跑在显示任务里): 把新值交给 app。
 *
 * 【为什么只调 app_speed_target_set() 就够了】那个函数只写一个 32 位对齐的
 * volatile float, 是"跨任务安全"的 (见 app.h 的跨任务约定)。真正的下发
 * (写控制器/斜坡/软限速) 由控制任务在 app_speed_apply() 里做。
 *
 * 【千万不要在这里写硬件或控制器】LVGL 回调跑在显示任务上下文, 直接改 s_ctrl
 * 会和控制任务的 1ms 计算打架; 而且拖手指时回调一次能来好几遍。 */
static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t   v      = lv_slider_get_value(slider);

    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        s_ui.target_rpm = v;
        app_speed_target_set((float)v);
    }
}

int32_t lcd_ui_get_target_rpm(void)
{
    return s_ui.target_rpm;
}

/* 位置条回调: 语义和速度条完全一样 —— 只把目标值发给 core0(经共享内存命令区),
 * 由控制任务在它的 1ms 节拍里锁存。显示任务绝不直接改位置环的数据。 */
static void pos_slider_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        int32_t v = lv_slider_get_value(lv_event_get_target(e));

        s_ui.target_mrad = v;
        app_motion_set_target_rad((float)v * 0.001f);   /* mrad -> rad */
    }
}

int32_t lcd_ui_get_target_mrad(void)
{
    return s_ui.target_mrad;
}

/* ==================== 模式开关: 位置模式 <-> 速度模式 ====================
 * 开关只做两件事: 把使能命令发给 core0(经过共享内存, 见 shm_client_set_motion),
 * 以及切回速度模式时把滑条的值重新下发一次。
 *
 * 【为什么切回速度模式必须重新下发一次】位置环在跑的时候每拍都在改写速度给定,
 * 所以 app 里那份"目标转速"早就不是滑条的值了(是位置环最后一次的输出, 通常非 0)。
 * 关掉位置环之后如果不管它, 电机会保持那个残留转速, 而界面显示的却是滑条值 ——
 * 两边对不上, 看起来就像"关不掉"。 */
static void motion_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    uint8_t   on = (lv_obj_has_state(sw, LV_STATE_CHECKED) != 0) ? 1U : 0U;

    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    app_motion_set_enable(on);
    if (on == 0U) {
        app_speed_target_set((float)s_ui.target_rpm);
    }
}

/* ---- 小工具: 建一个标签 ---- */
static lv_obj_t *ui_label(lv_obj_t *parent, const char *text, const lv_font_t *font,
                          uint32_t color)
{
    lv_obj_t *lbl = lv_label_create(parent);

    lv_label_set_text(lbl, text);
    if (font != NULL) {
        lv_obj_set_style_text_font(lbl, font, 0);
    }
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
    return lbl;
}

void lcd_ui_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *title;
    int32_t   init_rpm;
    int32_t   i;

    s_ui.last_refr_ms = lv_tick_get();
    s_ui.rpm_ms       = s_ui.last_refr_ms;
    s_ui.rpm_color    = 0xFFFFFFFFU;    /* 还没设过颜色 */
    s_ui.fb_color     = 0xFFFFFFFFU;
    s_ui.mode_on      = 0xFFU;          /* 还没同步过开关 -> 第一拍一定同步一次 */
    s_ui.rpm_filt     = app_speed_get_rpm();
    s_ui.rpm_shown    = 0;
    for (i = 0; i < UI_RPM_CELLS; i++) {
        s_ui.digit[i] = '\0';
    }

    /* 上电时滑条位置 = 应用层当前的目标转速, 两边一上来就是一致的 */
    init_rpm = (int32_t)app_speed_get_target();
    if (init_rpm < LCD_UI_RPM_MIN) {
        init_rpm = LCD_UI_RPM_MIN;
    }
    if (init_rpm > LCD_UI_RPM_MAX) {
        init_rpm = LCD_UI_RPM_MAX;
    }
    s_ui.target_rpm = init_rpm;

    lv_obj_set_style_bg_color(screen, lv_color_hex(UI_BG), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(screen, 0, 0);

    /* ---- 标题 ----
     * 【全部界面文本必须是 ASCII】LVGL 只编译了 Montserrat, 它没有中文字形
     * (lv_conf.h 里 LV_FONT_SOURCE_HAN_SANS_SC_*_CJK 都是 0)。写中文进去在屏上
     * 就是空框/缺字 —— 这正是之前"转速那一块画面不规则"的一部分原因, 不是排版
     * 问题。所以要中文界面必须先把 CJK 字体编进去(阈值 LV_MEM_SIZE 也得跟着涨),
     * 在那之前这里只用英文+数字。 */
    title = ui_label(screen, "SPEED", &lv_font_montserrat_20, UI_TEXT);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 8);
    {
        lv_obj_t *en = ui_label(screen, "MOTOR", &lv_font_montserrat_12, UI_DIM);

        lv_obj_align(en, LV_ALIGN_TOP_RIGHT, -10, 13);
    }

    /* ---- 大字转速 ----
     * 【为什么是 4 个定宽标签, 而不是一个标签】见文件上方"按位写"的说明:
     * 拆开后改一位只重刷 28x44(2.5KB), 而不是 112x44(9.9KB) —— 刷屏时间从 3.1ms
     * 降到 0.78ms, 明显短于面板扫过这条带的时间(2.6ms), 撕裂机会小得多。
     * 【字体必须是等宽的】UI_FONT_DIGITS 的每一位步进都等于 UI_RPM_CELL_W,
     * 所以格子边界就是字形边界: 位数变化时不会整体左右移动, 也不会互相挤。
     * (右对齐在新字体下已经是恒等操作, 留着是为了退回比例字体时行为不变。) */
    for (i = 0; i < UI_RPM_CELLS; i++) {
        s_ui.lbl_digit[i] = ui_label(screen, "", UI_FONT_DIGITS, UI_ACCENT);
        lv_obj_set_width(s_ui.lbl_digit[i], UI_RPM_CELL_W);
        lv_obj_set_style_text_align(s_ui.lbl_digit[i], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(s_ui.lbl_digit[i], LV_ALIGN_TOP_LEFT,
                     12 + (i * UI_RPM_CELL_W), 30);
    }

    /* 上电就把当前转速显示出来。
     * 【为什么必须显式写一次】ui_rpm_update() 只在"差值 >= 死区"或心跳到点时才改屏,
     * 转速本来就是 0 的时候差值是 0 -> 永远不写, 大字会一直空着。 */
    {
        int32_t v0 = (int32_t)((s_ui.rpm_filt >= 0.0f) ? (s_ui.rpm_filt + 0.5f)
                                                       : (s_ui.rpm_filt - 0.5f));

        if (v0 > 999) {
            v0 = 999;
        } else if (v0 < -999) {
            v0 = -999;
        }
        ui_rpm_commit(v0, s_ui.rpm_ms);
    }

    /* "RPM" 单位: 位置由 UI_UNIT_X / UI_UNIT_Y 定(见文件上方那两个宏的说明:
     * 贴着数字组右边界 + 与数字基线对齐)。它是静态标签, 建好之后
     * lcd_ui_refresh() 永远不碰它 -> 它不会参与任何刷屏, 本身不可能被撕。 */
    s_ui.lbl_unit = ui_label(screen, "RPM", &lv_font_montserrat_20, UI_DIM);
    lv_obj_align(s_ui.lbl_unit, LV_ALIGN_TOP_LEFT, UI_UNIT_X, UI_UNIT_Y);

    /* ---- 速度条 (目标转速) ----
     * 【纵向排布的依据】滑条本体占 y 106..132, 但 knob 设成 34px 高,
     * 会从 y=102 突出到 y=140 —— 所以刻度 0/600 必须放到 144 以下,
     * 否则旋钮压到数字上(之前就是这个问题)。 */
    s_ui.lbl_cap = ui_label(screen, "TARGET", &lv_font_montserrat_14, UI_TEXT);
    lv_obj_align(s_ui.lbl_cap, LV_ALIGN_TOP_LEFT, 10, 86);

    s_ui.lbl_target = ui_label(screen, "--- RPM", &lv_font_montserrat_16, UI_ACCENT);
    lv_obj_align(s_ui.lbl_target, LV_ALIGN_TOP_RIGHT, -10, 84);

    s_ui.lbl_min = ui_label(screen, "0", &lv_font_montserrat_12, UI_DIM);
    lv_obj_align(s_ui.lbl_min, LV_ALIGN_TOP_LEFT, 10, 146);

    s_ui.lbl_max = ui_label(screen, "600", &lv_font_montserrat_12, UI_DIM);
    lv_obj_align(s_ui.lbl_max, LV_ALIGN_TOP_RIGHT, -10, 146);

    s_ui.slider = lv_slider_create(screen);
    lv_obj_set_size(s_ui.slider, 268, 26);
    lv_obj_align(s_ui.slider, LV_ALIGN_TOP_MID, 0, 106);
    lv_slider_set_range(s_ui.slider, LCD_UI_RPM_MIN, LCD_UI_RPM_MAX);
    lv_slider_set_value(s_ui.slider, init_rpm, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui.slider, lv_color_hex(UI_TRACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ui.slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ui.slider, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui.slider, lv_color_hex(UI_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui.slider, 8, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_ui.slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_ui.slider, 0, LV_PART_KNOB);
    /* 旋钮做得比滑轨高一点, 手指好按 (电阻屏 + 无校准也能拖得住) */
    lv_obj_set_style_width(s_ui.slider, 20, LV_PART_KNOB);
    lv_obj_set_style_height(s_ui.slider, 34, LV_PART_KNOB);
    lv_obj_add_event_cb(s_ui.slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---- 位置条 (目标位置, mrad) ----
     * 和速度条【完全同一块几何位置】, 创建后先隐藏; 由 lcd_ui_refresh() 按模式
     * 二选一显示。范围 0..6283 mrad = 0..360° 机械角, 一像素约 23 mrad (1.3°),
     * 电阻屏上够用了。 */
    s_ui.target_mrad = 0;
    s_ui.pos_shown   = 0xFFU;           /* 强制第一次刷新时同步一次可见性 */
    s_ui.pos_slider  = lv_slider_create(screen);
    lv_obj_set_size(s_ui.pos_slider, 268, 26);
    lv_obj_align(s_ui.pos_slider, LV_ALIGN_TOP_MID, 0, 106);
    lv_slider_set_range(s_ui.pos_slider, -3141, 3141);
    lv_slider_set_value(s_ui.pos_slider, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_ui.pos_slider, lv_color_hex(UI_TRACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ui.pos_slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_ui.pos_slider, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_ui.pos_slider, lv_color_hex(UI_WARN), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_ui.pos_slider, 8, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_ui.pos_slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_ui.pos_slider, 0, LV_PART_KNOB);
    lv_obj_set_style_width(s_ui.pos_slider, 20, LV_PART_KNOB);
    lv_obj_set_style_height(s_ui.pos_slider, 34, LV_PART_KNOB);
    lv_obj_add_event_cb(s_ui.pos_slider, pos_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_flag(s_ui.pos_slider, LV_OBJ_FLAG_HIDDEN);

    /* ---- 【模式开关】位置环(位置模式) <-> 滑条(速度模式) ----
     * 【为什么必须有个开关】两个环抢的是同一个变量: 位置环每拍把它的输出 spdref
     * 写进 app_speed_target_set(), 而滑条也写同一个目标值 —— 谁最后写谁说了算,
     * 表现就是"速度环和位置环打架"。所以这里明确选一个:
     *   开关 ON  = 位置模式: 位置环使能(原地保持 + 按键点动), 滑条变灰不参与;
     *   开关 OFF = 速度模式: 位置环关闭, 滑条直接给目标转速。
     * 【放在哪】滑条刻度那一行(y 146..161)中间: 旋钮最高到 y≈140, 两边"0"/"600"
     * 只占 x<30 和 x>280, 中间是空的, 不挤任何东西。
     * 【状态从哪来】开关画的是 core0 回报的真实状态(SHM_STATUS_MOTION),
     * 不是"我按了什么就显示什么" —— 使能可能因为未运行/过流而没生效。 */
    s_ui.mode_sw = lv_switch_create(screen);
    lv_obj_set_size(s_ui.mode_sw, 46, 24);
    lv_obj_align(s_ui.mode_sw, LV_ALIGN_TOP_LEFT, 140, 143);
    lv_obj_add_event_cb(s_ui.mode_sw, motion_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_ui.lbl_mode = ui_label(screen, "POS", &lv_font_montserrat_12, UI_DIM);
    lv_obj_align(s_ui.lbl_mode, LV_ALIGN_TOP_LEFT, 108, 149);

    /* ---- 反馈行 ---- */
    s_ui.lbl_fb = ui_label(screen, "INIT...", &lv_font_montserrat_14, UI_DIM);
    lv_obj_align(s_ui.lbl_fb, LV_ALIGN_TOP_LEFT, 10, 164);

    s_ui.lbl_readback = ui_label(screen, "", &lv_font_montserrat_12, UI_WARN);
    lv_obj_align(s_ui.lbl_readback, LV_ALIGN_TOP_LEFT, 10, 182);

    {
        lv_obj_t *hint = ui_label(screen,
            "POS ON = HOLD POSITION\n"
            "POS OFF = SLIDE SETS RPM",
            &lv_font_montserrat_12, UI_DIM);

        lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 10, -6);
    }
}

void lcd_ui_refresh(float rpm, float target, float target_eff,
                    uint8_t running, uint8_t busy, uint8_t pending)
{
    uint32_t now;
    int32_t  raw_i;
    int32_t  tgt_i;
    uint32_t color;
    uint8_t  dragging;
    int32_t  i;

    if (s_ui.lbl_digit[0] == NULL) {
        return;                     /* 界面还没建好 */
    }

    /* 【真时间节流】见 UI_REFRESH_MS 的说明: 以前按"拍了多少下"数, 而每拍其实是
     * 5ms, 于是 20ms 变成了 100ms。lv_tick_get() 是 LVGL 的毫秒时基(由 mchtmr 提供),
     * 用它判断"离上次过了多久"。 */
    now = lv_tick_get();
    if ((uint32_t)(now - s_ui.last_refr_ms) < UI_REFRESH_MS) {
        return;
    }
    s_ui.last_refr_ms = now;

    /* ---- 大字颜色 ----------------------------------------------------------
     * 【超速/反转用颜色提示】正转青, 反转或标定中黄, 接近限幅红。
     * 颜色判据用"原始 rpm"的四舍五入值, 与显示的滤波器无关(颜色要跟得快)。
     * 【必须自己缓存颜色】lv_obj_set_style_text_color() 在 LVGL 9 里不做去重
     * (lv_obj_style.c 无条件 lv_obj_refresh_style), 每写一次就标脏一次 —— 4 个
     * 数字位每 20ms 白标脏一次, 就是每 20ms 多刷 4 块屏。 */
    raw_i = (int32_t)((rpm >= 0.0f) ? (rpm + 0.5f) : (rpm - 0.5f));

    if ((busy != 0U) || (raw_i < 0)) {
        color = UI_WARN;
    } else if ((float)raw_i > APP_FOC_RPM_LIMIT * 0.9f) {
        color = UI_BAD;
    } else {
        color = UI_ACCENT;
    }

    if (color != s_ui.rpm_color) {
        s_ui.rpm_color = color;
        for (i = 0; i < UI_RPM_CELLS; i++) {
            lv_obj_set_style_text_color(s_ui.lbl_digit[i], lv_color_hex(color), 0);
        }
    }

    /* ---- 实测转速: 滤波 + 去抖 + 限速 ----
     * 【为什么必须限幅再显示】大字是 montserrat_40, x 从 12 开始, 屏宽只有 320。
     * 40px 字体一个数字约 22~24px 宽, 4 个定宽数字位共 104px, 加上负号就是 "-999"
     * (正好 4 位)。而实测转速在符号错/堵转时会冲到几百甚至上千 RPM —— "-1042" 或
     * 编码器异常时的 "-2147483" 会直接画到屏幕外面, 看起来就是"转速那块什么都没
     * 显示"。所以显示值夹在 ±999。【注意】这只影响显示, 不影响控制。
     * 限幅、滤波、去抖这三件事的顺序: 先滤波 -> 取整 -> 限幅 -> 比较是否要改屏。 */
    ui_rpm_update(rpm, now);

    /* ---- 目标值文本: 位置模式显示 rad(带符号), 速度模式显示 RPM ----
     * 【为什么这里要分模式】lbl_target 是同一个标签, 位置模式下再显示 "300 RPM"
     * 就是单位不对 —— 滑条此刻给的是位置(rad), 不是转速。 */
    tgt_i = s_ui.target_rpm;
    if (s_ui.mode_on != 0U) {
        int32_t mr = s_ui.target_mrad;
        int32_t am = (mr < 0) ? -mr : mr;
        char    tmp[24];

        (void)snprintf(tmp, sizeof(tmp), "%s%d.%02d rad", (mr < 0) ? "-" : "+",
                       (int)(am / 1000), (int)((am % 1000) / 10));
        ui_text(s_ui.lbl_target, s_ui.target, sizeof(s_ui.target), tmp);
    } else {
        ui_text_fmt(s_ui.lbl_target, s_ui.target, sizeof(s_ui.target), "%d RPM", tgt_i, -1);
    }

    /* ---- 滑条位置跟随 ---- */
    /* 只有"用户没在拖"且"滑条值与外部目标不一致"时才回写 —— 覆盖正在拖动的
     * 滑条会和手指打架 (手指往右、代码把它拉回来)。 */
    dragging = lv_obj_has_state(s_ui.slider, LV_STATE_PRESSED) ? 1U : 0U;
    if (dragging == 0U) {
        int32_t want = (int32_t)((target >= 0.0f) ? (target + 0.5f) : (target - 0.5f));

        if (want < LCD_UI_RPM_MIN) { want = LCD_UI_RPM_MIN; }
        if (want > LCD_UI_RPM_MAX) { want = LCD_UI_RPM_MAX; }
        if ((want != tgt_i) && (lv_slider_get_value(s_ui.slider) != want)) {
            lv_slider_set_value(s_ui.slider, want, LV_ANIM_OFF);
            s_ui.target_rpm = want;
            ui_text_fmt(s_ui.lbl_target, s_ui.target, sizeof(s_ui.target),
                        "%d RPM", want, -1);
        }
    }

    /* ---- 模式开关: 跟随 core0 回报的【真实】状态 ----
     * 画的是实际使能位, 不是"我按了什么": 使能可能因为电机未运行/过流封锁而没生效。
     * 【位置模式下必须把滑条变灰】位置环每拍都在改写速度给定, 这时滑条拖了也白拖
     * (而且会误导用户以为能调转速) —— 让它在视觉上就"不参与"。
     * 用 lv_obj_add_state() 改状态【不会】触发 LV_EVENT_VALUE_CHANGED(那个只有用户
     * 点击才发), 所以不会和上面的回调递归打架。 */
    {
        uint8_t on = (app_motion_is_enabled() != 0U) ? 1U : 0U;

        if (on != s_ui.mode_on) {
            s_ui.mode_on = on;
            if (on != 0U) {
                lv_obj_add_state(s_ui.mode_sw, LV_STATE_CHECKED);
                lv_obj_add_state(s_ui.slider, LV_STATE_DISABLED);
            } else {
                lv_obj_remove_state(s_ui.mode_sw, LV_STATE_CHECKED);
                lv_obj_remove_state(s_ui.slider, LV_STATE_DISABLED);
            }
            /* 【两根滑条共用同一块地方】位置模式显示位置条(目标位置), 速度模式
             * 显示速度条(目标转速) —— 只在这个状态真的翻转时才动, 免得每拍都去
             * 改可见性和文字(那会白白标脏 -> 多刷两遍屏)。 */
            if (on != 0U) {
                lv_obj_add_flag(s_ui.slider, LV_OBJ_FLAG_HIDDEN);
                lv_obj_remove_flag(s_ui.pos_slider, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text(s_ui.lbl_cap, "POSITION rad");
                lv_label_set_text(s_ui.lbl_min, "-3.14");
                lv_label_set_text(s_ui.lbl_max, "+3.14");
            } else {
                lv_obj_remove_flag(s_ui.slider, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(s_ui.pos_slider, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text(s_ui.lbl_cap, "TARGET");
                lv_label_set_text(s_ui.lbl_min, "0");
                lv_label_set_text(s_ui.lbl_max, "600");
            }
        }
    }

    /* ---- 状态行 ----
     * 【颜色只在变化时写】这一行原来每个分支都无条件 set_style_text_color() 一次,
     * 而 LVGL 不做去重 -> 这一行每 20ms 被标脏一次, 每 10ms 的刷新里就被重刷一遍
     * (105x17 像素, 约 1.1ms 的 SPI 时间), 实测占到整个刷屏时间的 2/3。内容没变
     * 的重刷虽然看不出撕裂, 但它把显示任务和 SPI 都占住了, 是"刷屏很慢"的主因。 */
    if (busy != 0U) {
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "CALIBRATING");
        ui_text_color(s_ui.lbl_fb, UI_WARN, &s_ui.fb_color);
        ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                "TARGET IGNORED");
    } else if (running == 0U) {
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "MOTOR OFF");
        ui_text_color(s_ui.lbl_fb, UI_BAD, &s_ui.fb_color);
        ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback), "");
    } else {
#if APP_FOC_CLOSED_LOOP_ENABLE
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "FOC RUNNING");
#else
        ui_text(s_ui.lbl_fb, s_ui.fb, sizeof(s_ui.fb), "OPENLOOP RUN");
#endif
        ui_text_color(s_ui.lbl_fb, UI_OK, &s_ui.fb_color);

#if APP_FOC_SPEED_LOOP_ENABLE
        /* 速度环是位置环的内环, 所以真正决定"谁在指挥"的是位置环的使能位。 */
        if (s_ui.mode_on != 0U) {
            ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                    "POSITION HOLD (SLIDE OFF)");
        } else {
            ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                    "SPEED MODE: SLIDE = RPM");
        }
#else
        /* 没有速度环时, 目标转速只作为软限速/开环频率给定 —— 老实告诉用户,
         * 免得看到"实际转速不等于目标"以为是 bug。 */
        int32_t eff_i = (int32_t)((target_eff >= 0.0f) ? (target_eff + 0.5f)
                                                       : (target_eff - 0.5f));
        if ((pending != 0U) || (eff_i != tgt_i)) {
            /* 目标刚被拖过 / 斜坡还没爬到: 用"当前值 -> 目标值"显示出来 */
            ui_text_fmt(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                        "RAMP %d > %d RPM", eff_i, tgt_i);
        } else if (tgt_i >= LCD_UI_RPM_MAX) {
            /* 拉到底 = 不额外限速 (软限速上限仍是 app 里的 APP_FOC_RPM_LIMIT)。
             * 这是刻意的: 600 RPM 低于默认限速 1000 RPM, 若照字面限到 600,
             * 用户一拖到底反而把转速上限压低了。 */
            ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                    "FULL = NO EXTRA LIMIT");
        } else if (tgt_i == 0) {
            ui_text(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                    "0 = NO LIMIT");
        } else {
            ui_text_fmt(s_ui.lbl_readback, s_ui.readback, sizeof(s_ui.readback),
                        "LIMIT %d RPM", tgt_i, -1);
        }
#endif
    }
}
