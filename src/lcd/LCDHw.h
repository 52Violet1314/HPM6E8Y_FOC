
#ifndef __LCD_H
#define __LCD_H		

/* IOC_PAD_xxx / u8 / u16 来自 board.h(-> hpm_common.h -> soc 的 ioc 寄存器头)。
 * 这里显式包含一次: 本头文件对外暴露 LCD_PAD_* 引脚宏, 别的 .c 只 include
 * 本文件时也必须能解析这些宏。 */
#include "board.h"

//LCD重要参数集
typedef struct  
{										    
	u16 width;			//LCD 宽度
	u16 height;			//LCD 高度
	u16 id;				  //LCD ID
	u8  dir;			  //横屏还是竖屏控制：0，竖屏；1，横屏。	
	u16	 wramcmd;		//开始写gram指令
	u16  setxcmd;		//设置x坐标指令
	u16  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
/////////////////////////////////////用户配置区///////////////////////////////////	 
#define USE_HORIZONTAL  	 0//定义液晶屏顺时针旋转方向 	0-0度旋转，1-90度旋转，2-180度旋转，3-270度旋转

//////////////////////////////////////////////////////////////////////////////////	  
//定义LCD的尺寸
#define LCD_W 320
#define LCD_H 240

/* ======================= LCD 引脚(PAD)定义 =======================
 * 【为什么要在这里集中定义】原来这些引脚宏散在 LCDHw.c 的 #define 里, 结果
 * 无法对"触摸的软件 SPI 引脚有没有和 LCD 撞车"做编译期检查 —— 那正是
 * "一碰屏幕就花屏"最容易被怀疑的硬伤(见 LCDTouch.c 顶部的编译期断言)。
 *
 * 【本板实测接线(不是猜的)】来源: HPM6EMotorV20.pdf 原理图 J10(16 脚显示口)
 * 的网络表(build/schematic_text.txt 里 J10 那一段):
 *   J10.3  = CS     -> PC11       J10.10 = T_SCLK -> PC20
 *   J10.4  = RESET  -> PC03       J10.11 = T_CS   -> PC21
 *   J10.5  = DC/RS  -> PC04       J10.12 = T_MOSI -> PC23
 *   J10.6  = MOSI   -> PC13       J10.13 = T_MISO -> PC22
 *   J10.7  = SCLK   -> PC10       J10.14 = T_IRQ  -> PC09
 *   J10.8  = LED    -> PC08
 *   J10.9  = MISO   -> PC12
 * 即 LCD(SPI1) 与触摸(XPT2046, 软件 SPI)在【排针上就是两组独立的线】,
 * 不是"共总线 + 两个片选"的接法。 */
#define LCD_PAD_CS    IOC_PAD_PC11   /* 片选(软件 GPIO)   */
#define LCD_PAD_SCLK  IOC_PAD_PC10   /* SPI1.SCLK         */
#define LCD_PAD_MISO  IOC_PAD_PC12   /* SPI1.MISO         */
#define LCD_PAD_MOSI  IOC_PAD_PC13   /* SPI1.MOSI         */
#define LCD_PAD_RS    IOC_PAD_PC04   /* 数据/命令选择     */
#define LCD_PAD_RST   IOC_PAD_PC03   /* 复位              */
#define LCD_PAD_LED   IOC_PAD_PC08   /* 背光              */

/* ---------------- 像素通路自检数据(用于定位"长条/花屏"到底是哪一步丢数据) ----------------
 * 由 LCDHw.c 统计, 显示任务每 5 秒走日志队列打一行(见 lcd_app.c)。
 *
 * 【为什么要区分 busy 和 timeout】这两个返回值是**唯一**能真正丢数据的路径
 * (hpm_spi_drv.c):
 *   - status_spi_master_busy : spi_transfer() 开头发现 SPIACTIVE 还置着, 直接返回,
 *                              【一个字节都没发】。对 LCD 来说 = 这条命令/参数被吞了。
 *   - status_timeout         : spi_wait_for_idle_status() 的 5000 次重试耗尽而返回
 *                              (注意: 此时数据其实还在往外移, 超时的是"等待"),
 *                              调用者若把它当失败处理就会提前收尾/抬 CS。
 * 反过来, spi_write_data() 只会在 TXFULL 时空转, 【不会丢字节】——所以
 * "轮询被抢占 -> 少发几个字节"这个链条是不成立的, 必须靠这两个计数来定案。 */
typedef struct {
    u32 chunk_count;       /* 成功写出的数据块数(512B/块) */
    u32 busy_count;        /* spi_transfer 返回 status_spi_master_busy 的次数 */
    u32 timeout_count;     /* spi_transfer 返回 status_timeout 的次数 */
    u32 fail_count;        /* 其它失败(status_fail 等) */
    u32 byte_cmd_fail;     /* 单字节命令/参数(SPI_WriteByte)失败次数 */
    u32 retry_count;       /* 因失败重发同一块的次数 */
    u32 band_abort_count;  /* 重发仍失败、放弃本条 band 剩余像素的次数 */
    u32 last_status;       /* 最近一次 spi_transfer 的返回值(hpm_stat_t) */

    /* ---- SPI1 TX DMA 通路(见 LCDHw.c 顶部说明) ---- */
    u32 dma_chunk_count;   /* 由 DMA 搬完的块数 */
    u32 dma_error_count;   /* DMA 失败/超时的次数 */
    u32 dma_fallback_count;/* 因 DMA 失败而回退到 CPU 轮询写的次数 */
    u32 last_dma_status;   /* 最近一次 DMA 通道状态(hpm_dmav2_drv.h) */
    u8  dma_ready;         /* 1 = DMA 通路初始化成功 */
    u8  dma_disabled;      /* 1 = 已永久退回 CPU 轮询(连续失败达上限/初始化失败) */
} lcd_tx_stats_t;

void LCD_GetTxStats(lcd_tx_stats_t *out);


//TFTLCD部分外要调用的函数		   
extern u16  POINT_COLOR;//默认红色    
extern u16  BACK_COLOR; //背景颜色.默认为白色
#if 0
////////////////////////////////////////////////////////////////////
//-----------------LCD端口定义---------------- 
#define GPIO_TYPE  GPIOB  //GPIO组类型
#define LED      9        //背光控制引脚        PB9
#define LCD_CS   11       //片选引脚            PB11
#define LCD_RS   10       //寄存器/数据选择引脚 PB10 
#define LCD_RST  12       //复位引脚            PB12


//QDtech全系列模块采用了三极管控制背光亮灭，用户也可以接PWM调节背光亮度
#define	LCD_LED PBout(LED) //LCD背光    		 PB9
//如果使用官方库函数定义下列底层，速度将会下降到14帧每秒，建议采用我司推荐方法
//以下IO定义直接操作寄存器，快速IO操作，刷屏速率可以达到28帧每秒！ 

//GPIO置位（拉高）
#define	LCD_CS_SET  GPIO_TYPE->BSRR=1<<LCD_CS    //片选端口  	PB11
#define	LCD_RS_SET	GPIO_TYPE->BSRR=1<<LCD_RS    //数据/命令  PB10	  
#define	LCD_RST_SET	GPIO_TYPE->BSRR=1<<LCD_RST   //复位			  PB12

//GPIO复位（拉低）							    
#define	LCD_CS_CLR  GPIO_TYPE->BRR=1<<LCD_CS     //片选端口  	PB11
#define	LCD_RS_CLR	GPIO_TYPE->BRR=1<<LCD_RS     //数据/命令  PB10	 
#define	LCD_RST_CLR	GPIO_TYPE->BRR=1<<LCD_RST    //复位			  PB12
#endif

//画笔颜色
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	0X841F //浅绿色
#define LIGHTGRAY     0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      	0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)
	    															  
void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(u16 Color);	 
void LCD_SetCursor(u16 Xpos, u16 Ypos);
void LCD_DrawPoint(u16 x,u16 y);//画点
u16  LCD_ReadPoint(u16 x,u16 y); //读点
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);		   
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);

u16 LCD_RD_DATA(void);//读取LCD数据									    
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
void LCD_WR_DATA(u8 data);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
u16 LCD_ReadRAM(void);		   
u16 LCD_BGR2RGB(u16 c);
void LCD_SetParam(void);
void Lcd_WriteData_16Bit(u16 Data);
void Lcd_WriteData_Block(u8 *data,u32 len);
void LCD_WritePixels(const u8 *data, u32 pixel_count);
void LCD_direction(u8 direction );

//如果仍然觉得速度不够快，可以使用下面的宏定义,提高速度.
//注意要去掉lcd.c中void LCD_WR_DATA(u16 data)函数定义哦
/*
#if LCD_USE8BIT_MODEL==1//使用8位并行数据总线模式
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	DATAOUT(data<<8);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	}
	#else//使用16位并行数据总线模式
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	} 	
#endif
*/
				  		 
#endif  
	 
	 



