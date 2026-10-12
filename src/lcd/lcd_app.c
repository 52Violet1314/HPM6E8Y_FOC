/*
 * LVGL bridge migrated from the fullfuncgui board example.
 *
 * LCD: ILI9341, 320x240, RGB565 over SPI1.
 * Touch: XPT2046-compatible resistive controller over GPIO bit-banged SPI.
 */
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"

#include "type.h"
#include "LCDHw.h"
#include "LCDTouch.h"
#include "lcd_app.h"
#include "lcd_ui.h"
#include "touch_calibration.h"
#include "lvgl.h"

#define TFT_HOR_RES                 (320U)
#define TFT_VER_RES                 (240U)
#define LVGL_DRAW_BUFFER_LINES      (20U)

#define HPM_LVGL_MCHTMR            HPM_MCHTMR
#define HPM_LVGL_MCHTMR_CLK        clock_mchtmr0

extern u8 CMD_RDX;
extern u8 CMD_RDY;

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

    LCD_SetWindows((u16)area->x1, (u16)area->y1,
                   (u16)area->x2, (u16)area->y2);
    LCD_WritePixels(pixel_map, width * height);
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
}

void lcd_app_ui_init(void)
{
    static uint8_t draw_buffer[TFT_HOR_RES * LVGL_DRAW_BUFFER_LINES * 2U];
    lv_display_t *display;
    lv_indev_t *indev;

    lv_init();
    lv_tick_set_cb(lcd_tick_cb);

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
    /* Do not delay here: the FOC loop owns the 1 ms real-time cadence. */
    lv_timer_handler();
}

void delay_ms(u32 ms)
{
    board_delay_ms(ms);
}

void delay_us(u32 us)
{
    board_delay_us(us);
}
