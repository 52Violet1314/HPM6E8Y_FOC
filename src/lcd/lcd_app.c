/*
 * LVGL bridge migrated from the fullfuncgui board example.
 *
 * LCD: ILI9341, 320x240, RGB565 over SPI1.
 * Touch: XPT2046-compatible resistive controller over GPIO bit-banged SPI.
 */
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"

/* RTOS: LVGL 互斥锁。FreeRTOSConfig.h 里 configUSE_MUTEXES = 1。 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "type.h"
#include "LCDHw.h"
#include "LCDTouch.h"
#include "app.h"            /* app_speed_get_rpm / target / running / calibrating */
#include "lcd_app.h"
#include "lcd_ui.h"
#include "touch_calibration.h"
#include "lvgl.h"
#include "app_log.h"        /* 汇总统计走日志队列(不阻塞刷屏) */

#define TFT_HOR_RES                 (320U)
#define TFT_VER_RES                 (240U)
#define LVGL_DRAW_BUFFER_LINES      (20U)

/* ==================== 诊断开关(排查"长条/花屏"用) ====================
 * LCD_APP_TRACE_FLUSH: 前 4 次 flush 打一行 area 坐标(和 LCD_SetWindows 的入参
 *   是同一组数), 用来确认 LVGL 给我们的就是"我们以为的那条 band"。
 *   排查完改成 0。
 * LCD_APP_TEST_CRITICAL_FLUSH: 实验用 —— 把整条 band 的传输包在
 *   taskENTER_CRITICAL()/taskEXIT_CRITICAL() 里(禁止任务切换)。
 *   用来回答"长条是不是任务级抢占造成的"。
 *   【必须知道它的局限】configMAX_SYSCALL_INTERRUPT_PRIORITY = 4, 优先级 1 的
 *   电流环 ADC 中断不受内核临界区影响, 照样会插进来 —— 所以这个实验只能排除
 *   "任务切换", 不能排除"20kHz 电流环中断"。要排除中断, 得临时
 *   intc_m_disable_irq(BOARD_BLDC_ADC_IRQn)(见报告里的实验 2b), 那种改法会让
 *   电流环停跑, 只能在电机静止时做。
 *   【测完必须改回 0】1.3ms 的临界区不能留在正式代码里。 */
#define LCD_APP_TRACE_FLUSH           (1)
#define LCD_APP_TRACE_FLUSH_COUNT     (4U)
#define LCD_APP_TEST_CRITICAL_FLUSH   (0)

#define HPM_LVGL_MCHTMR            HPM_MCHTMR
#define HPM_LVGL_MCHTMR_CLK        clock_mchtmr0

extern u8 CMD_RDX;
extern u8 CMD_RDY;

/* ---- 刷屏统计(每 5 秒汇总一行, 见 lcd_app_task_handler) ---- */
static u32 s_flush_count;        /* flush 调用次数 */
static u32 s_flush_pixels;       /* 累计写出的像素数 */
static u32 s_area_max_pixels;    /* 单次 area 最大像素数 */
static u32 s_stat_tick;          /* 5s 节流(本任务 5ms 一拍) */

/* LVGL 互斥锁 (在 lcd_app_ui_init 里建, 那时还没起调度器 -> 单线程, 无需加锁) */
static SemaphoreHandle_t s_lvgl_mtx;

int lcd_app_lvgl_lock(void)
{
    if (s_lvgl_mtx == NULL) {
        return 1;       /* 还没建锁: 说明是调度器启动前的单线程阶段 */
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return 1;       /* 同上: 上电初始化阶段没有并发 */
    }
    /* 拿不到就放弃本帧, 绝不阻塞等待 —— 见 lcd_app.h 的说明 */
    return (xSemaphoreTake(s_lvgl_mtx, 0) == pdTRUE) ? 1 : 0;
}

void lcd_app_lvgl_unlock(void)
{
    if ((s_lvgl_mtx != NULL) &&
        (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
        (void)xSemaphoreGive(s_lvgl_mtx);
    }
}

static uint32_t lcd_tick_cb(void)
{
    static uint32_t mchtmr_freq_in_khz;

    if (mchtmr_freq_in_khz == 0U) {
        clock_add_to_group(HPM_LVGL_MCHTMR_CLK, 0);
        mchtmr_freq_in_khz = clock_get_frequency(HPM_LVGL_MCHTMR_CLK) / 1000U;
    }

    return (uint32_t)(mchtmr_get_count(HPM_LVGL_MCHTMR) / mchtmr_freq_in_khz);
}

static lv_coord_t touch_calibrate_axis(uint16_t raw, float factor,
                                       int16_t offset, uint16_t screen_size)
{
    float coordinate = factor * raw + offset;

    if (coordinate <= 0.0f) {
        return 0;
    }
    if (coordinate >= (float)(screen_size - 1U)) {
        return (lv_coord_t)(screen_size - 1U);
    }
    return (lv_coord_t)(coordinate + 0.5f);
}

static void lcd_flush_cb(lv_display_t *display, const lv_area_t *area,
                         uint8_t *pixel_map)
{
    uint32_t width = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t height = (uint32_t)(area->y2 - area->y1 + 1);
    uint32_t pixels = width * height;

    s_flush_count++;
    s_flush_pixels += pixels;
    if (pixels > s_area_max_pixels) {
        s_area_max_pixels = pixels;
    }

#if LCD_APP_TRACE_FLUSH
    /* 只打前几次: 确认 LVGL 给的 area 与 LCD_SetWindows 的入参一致(实验 3)。
     * 走日志队列 -> 由日志任务写串口, 显示任务不会被串口拖住。 */
    if (s_flush_count <= LCD_APP_TRACE_FLUSH_COUNT) {
        LOG_I("LCD", "flush#%u area=(%d,%d)-(%d,%d) %ux%u pix=%u",
              (unsigned)s_flush_count, (int)area->x1, (int)area->y1,
              (int)area->x2, (int)area->y2,
              (unsigned)width, (unsigned)height, (unsigned)pixels);
    }
#endif

#if LCD_APP_TEST_CRITICAL_FLUSH
    taskENTER_CRITICAL();       /* 实验 2a: 禁止任务切换(不影响优先级 1 的电流环中断) */
#endif

    LCD_SetWindows((u16)area->x1, (u16)area->y1,
                   (u16)area->x2, (u16)area->y2);
    LCD_WritePixels(pixel_map, pixels);

#if LCD_APP_TEST_CRITICAL_FLUSH
    taskEXIT_CRITICAL();
#endif

    lv_display_flush_ready(display);
}

static void lcd_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    static lv_point_t last_point;

    LV_UNUSED(indev);

    if (TP_Scan(1U) != 0U) {
        last_point.x = touch_calibrate_axis(tp_dev.x, TOUCH_CAL_XFAC,
                                            TOUCH_CAL_XOFF, TFT_HOR_RES);
        last_point.y = touch_calibrate_axis(tp_dev.y, TOUCH_CAL_YFAC,
                                            TOUCH_CAL_YOFF, TFT_VER_RES);
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    data->point = last_point;
}

void lcd_app_hardware_init(void)
{
    LCD_Init();

    tp_dev.xfac = TOUCH_CAL_XFAC;
    tp_dev.yfac = TOUCH_CAL_YFAC;
    tp_dev.xoff = TOUCH_CAL_XOFF;
    tp_dev.yoff = TOUCH_CAL_YOFF;
    tp_dev.touchtype = TOUCH_CAL_TOUCHTYPE;
    CMD_RDX = TOUCH_CAL_CMD_RDX;
    CMD_RDY = TOUCH_CAL_CMD_RDY;
    TP_Init();

#if LCD_APP_RAWTEST
    lcd_app_rawtest_bars();
    delay_ms(2000);     /* 让彩条停 2 秒: 肉眼确认后再交给 LVGL (否则会被立刻覆盖) */
#endif
}

/* 裸 LCD 自检彩条: 四条竖条 = 红/绿/蓝/白。
 * 只用 LCD_SetWindows + LCD_WritePixels (与 LVGL flush 相同的一对函数),
 * 不经过任何 LVGL 逻辑, 所以"有彩条"就等于"面板 + SPI + 色彩通路都是好的"。 */
void lcd_app_rawtest_bars(void)
{
    static u16 line[TFT_HOR_RES];
    static const u16 colors[4] = { RED, GREEN, BLUE, WHITE };
    u16 x;
    u16 y;

    printf("[LCD] rawtest: 4 bars (R/G/B/W) -> panel + SPI + color path\r\n");

    LCD_SetWindows(0, 0, (u16)(TFT_HOR_RES - 1U), (u16)(TFT_VER_RES - 1U));
    for (y = 0U; y < TFT_VER_RES; y++) {
        u16 c = colors[((uint32_t)y * 4U) / TFT_VER_RES];

        for (x = 0U; x < TFT_HOR_RES; x++) {
            line[x] = c;
        }
        LCD_WritePixels((const u8 *)line, (u32)TFT_HOR_RES);
    }
}

void lcd_app_ui_init(void)
{
    /* 【必须对齐到 LV_DRAW_BUF_ALIGN】lv_conf.h 里 LV_DRAW_BUF_ALIGN = 64,
     * 而 lv_display_set_buffers() 第一件事就是断言
     *     buf1 == lv_draw_buf_align(buf1, cf)
     * (lv_display.c:475, buf_align() 内部按 64 向上取整)。
     * 普通 static uint8_t 数组只保证 8/16 字节对齐 -> 断言 "buf1 is not aligned"
     * 立刻命中; 而 lcd_app_ui_init() 在 main.c 里排在 app_init() 之后、主循环
     * 之前, 一旦卡死就永远进不了 app_run() -> PWM 停在 50%/50%/50% 零矢量,
     * 看起来就是"电机驱动不起来"。LV_ATTRIBUTE_MEM_ALIGN 即 lv_conf.h 里的
     * __attribute__((aligned(64))), 与 LV_DRAW_BUF_ALIGN 保持一致。 */
    static uint8_t draw_buffer[TFT_HOR_RES * LVGL_DRAW_BUFFER_LINES * 2U] LV_ATTRIBUTE_MEM_ALIGN;
    lv_display_t *display;
    lv_indev_t *indev;

    lv_init();
    lv_tick_set_cb(lcd_tick_cb);

    /* LVGL 互斥锁: 必须在创建任务之前建好 (现在还是单线程初始化阶段, 不会竞争)。
     * 建失败(nil)也能跑 —— 那时 lcd_app_lvgl_lock() 退化成"直接放行", 与上 RTOS
     * 之前的行为一致。 */
    s_lvgl_mtx = xSemaphoreCreateMutex();

    display = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(display, draw_buffer, NULL, sizeof(draw_buffer),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, lcd_flush_cb);

    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lcd_touch_read_cb);

    lcd_ui_create();
}

void lcd_app_task_handler(void)
{
    /* ---- LVGL 串行化 ----
     * LVGL 不是线程安全的: 谁碰 lv_* 谁就得持有这把锁。目前 [显示任务] 是唯一
     * 的调用者 (lcd_app_task_handler 在 app_rtos.c 里由显示任务周期调用),
     * 触摸读取也在 lv_timer_handler() 内部由同一个任务完成 —— 所以现在这把锁
     * 是"空转"的。留着它是为了将来从别的任务/中断往界面推数据时不会踩内存,
     * 也是把"LVGL 只能有一个线程碰"这条规矩写进代码里。
     *
     * 【重要】拿不到锁就直接返回, 不要阻塞等待: 本函数跑在最低优先级任务里,
     * 阻塞也不会抢占控制任务, 但会让刷屏节奏乱掉; 而丢一帧无所谓。
     * 【重要】调度器还没起来时 (上电初始化阶段) 不能调 RTOS API。 */
    if (lcd_app_lvgl_lock() != 0) {
        /* ---- 电机界面数据刷新 ----
         * 每拍先刷新 (内部 20ms 节流 + 文本去重), 再跑 LVGL 的定时器/重绘。
         * 顺序反过来的话, 本拍写进去的文本要等下一拍才画出来。
         *
         * 【为什么放在 LVGL 里而不是 app 里】app 层不依赖 LVGL (分层要求),
         * 所以由这里主动去 app 拉数据。拉的都是 app 给的只读快照
         * (见 app.h 的"跨任务约定"), 不直接读控制任务的内部结构体。 */
        lcd_ui_refresh(app_speed_get_rpm(),
                       app_speed_get_target(),
                       app_speed_get_target_eff(),
                       app_speed_is_running(),
                       app_speed_is_calibrating(),
                       app_speed_target_pending());

        /* Do not delay here: the FOC loop owns the 1 ms real-time cadence. */
        (void)lv_timer_handler();

        lcd_app_lvgl_unlock();
    }

    /* ---- 每 5 秒汇总一行"像素通路"统计(判断到底有没有丢数据) ----
     * 本任务周期 5ms(见 app_rtos.c) -> 1000 拍 = 5s。
     * 【这一行是定案用的证据, 不是普通日志】
     *   chunk = 成功写出的 512B 块数;
     *   busy  = spi_transfer 返回 status_spi_master_busy 的次数
     *           (进函数时 SPIACTIVE 还置着 -> 这条命令/整块像素一个字节都没发);
     *   tmo   = 返回 status_timeout 的次数(等 SPI 空闲的 5000 次重试耗尽);
     *   retry/abort = 单块重发次数 / 重发仍失败后放弃该 band 剩余像素的次数
     *           (abort > 0 且 busy/tmo > 0 => "不规则长条"有了确定来源);
     *   若 busy/tmo/abort 恒为 0, 说明一个字节都没丢, "长条"就必须去查
     *   LVGL 侧(area/render/flush_ready 时序), 见报告的实验 1 结论分支。
     * 【为什么不放在 LCDHw.c】驱动层不打日志; 这里只做一次 vsnprintf + 入队,
     *   真正的串口写出由日志任务做, 不会拖慢刷屏。 */
    s_stat_tick++;
    if (s_stat_tick >= 1000U) {
        lcd_tx_stats_t st;

        s_stat_tick = 0U;
        LCD_GetTxStats(&st);
        LOG_I("LCD",
              "5s: flush=%u pix=%u areamax=%u chunk=%u busy=%u tmo=%u fail=%u retry=%u abort=%u cmd=%u last=%u",
              (unsigned)s_flush_count, (unsigned)s_flush_pixels,
              (unsigned)s_area_max_pixels,
              (unsigned)st.chunk_count, (unsigned)st.busy_count,
              (unsigned)st.timeout_count, (unsigned)st.fail_count,
              (unsigned)st.retry_count, (unsigned)st.band_abort_count,
              (unsigned)st.byte_cmd_fail, (unsigned)st.last_status);
    }
}

void delay_ms(u32 ms)
{
    board_delay_ms(ms);
}

void delay_us(u32 us)
{
    board_delay_us(us);
}
