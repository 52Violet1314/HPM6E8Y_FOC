/*
 * LVGL application bridge for the full-function GUI LCD example.
 */
#ifndef LCD_APP_H
#define LCD_APP_H

/* ================= 裸 LCD 自检 (排除 LVGL 的干扰) =================
 *
 *   1 = lcd_app_hardware_init() 里, LCD_Init() 之后直接画四条竖色条
 *       (红/绿/蓝/白), 停 2 秒再交给 LVGL。整条路径只用
 *       LCD_SetWindows() + LCD_WritePixels() —— 和 LVGL flush 用的是同一对函数,
 *       但不经过 lv_init / lv_timer_handler / 任何 LVGL 逻辑。
 *
 *       这样一次烧录就能把故障范围切开:
 *         彩条正常(红绿蓝白)  -> 面板/SPI/色彩通路都好, 问题在 LVGL 层
 *         彩条反色(青紫黄黑)  -> 通路好, 但"写前取反"这个约定错了
 *         只有黑/什么都没有    -> 面板没被初始化: SPI 时钟、时序、面板型号、接线
 *
 *   0 = 正常启动, 不画彩条。
 *
 *   诊断完请改回 0。 */
#define LCD_APP_RAWTEST             (0)

#ifdef __cplusplus
extern "C" {
#endif

void lcd_app_hardware_init(void);
void lcd_app_ui_init(void);
void lcd_app_task_handler(void);

/**
 * @brief  取 LVGL 互斥锁 (LVGL 不是线程安全的, 谁碰 lv_* 谁先拿锁)
 * @return 1 = 拿到锁 (调用者负责 unlock), 0 = 没拿到 / 当前不需要锁
 * @note   调度器启动前调用会直接返回 1 且不加锁 (那时还没并发);
 *         拿不到锁时【不要重试等待】—— 这里跑在最低优先级任务里, 丢一帧没事。
 */
int  lcd_app_lvgl_lock(void);

/** @brief 释放 LVGL 互斥锁 (与 lcd_app_lvgl_lock 配对) */
void lcd_app_lvgl_unlock(void);

/** @brief 裸 LCD 自检彩条 (不经过 LVGL), 见 LCD_APP_RAWTEST 说明 */
void lcd_app_rawtest_bars(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_APP_H */
