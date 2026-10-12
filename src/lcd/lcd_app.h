/*
 * LVGL application bridge for the full-function GUI LCD example.
 */
#ifndef LCD_APP_H
#define LCD_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void lcd_app_hardware_init(void);
void lcd_app_ui_init(void);
void lcd_app_task_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_APP_H */
