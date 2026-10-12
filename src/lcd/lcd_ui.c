/* Minimal LVGL screen used to verify the migrated LCD/GUI path. */
#include "lvgl.h"

#include "lcd_ui.h"

void lcd_ui_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *title;
    lv_obj_t *subtitle;

    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    title = lv_label_create(screen);
    lv_label_set_text(title, "HPM6E8Y  LVGL");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 28);

    subtitle = lv_label_create(screen);
    lv_label_set_text(subtitle, "ILI9341 / XPT2046");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x60D8FF), 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 58);
}
