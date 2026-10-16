#include "type.h"
#include "hpm_spi_drv.h"
#include "hpm_gpio_drv.h"
#include "board.h"

#include "LCDHw.h"
#include "stdlib.h"






#define LCD_GPIO_CTRL  HPM_GPIO0

/* 引脚定义集中在 LCDHw.h(LCD_PAD_*), 这样 LCDTouch.c 能对"触摸软件 SPI 有没有
 * 撞 LCD 引脚"做编译期断言(见 LCDTouch.c 顶部)。这里保留原来的名字做别名。 */
#define LCDGPIO_CS   LCD_PAD_CS         /* 片选引脚        PC11 */
#define LCDGPIO_LED  LCD_PAD_LED        /* 背光控制引脚    PC08 */
#define LCDGPIO_RS   LCD_PAD_RS         /* 寄存器/数据选择 PC04 */
#define LCDGPIO_RST  LCD_PAD_RST        /* 复位引脚        PC03 */

 
//GPIO置位（拉高）
#define	LCD_CS_SET  LCD_GPIO_SET(LCDGPIO_CS,1)    //片选端口  	PB11
#define	LCD_RS_SET	LCD_GPIO_SET(LCDGPIO_RS,1)    //数据/命令  PB10	  
#define	LCD_RST_SET	LCD_GPIO_SET(LCDGPIO_RST,1)   //复位			  PB12

//GPIO复位（拉低）							    
#define	LCD_CS_CLR  LCD_GPIO_SET(LCDGPIO_CS,0)     //片选端口  	PB11
#define	LCD_RS_CLR	LCD_GPIO_SET(LCDGPIO_RS,0)     //数据/命令  PB10	 
#define	LCD_RST_CLR	LCD_GPIO_SET(LCDGPIO_RST,0)    //复位			  PB12


void LCD_GPIOInit(void)
{
	//spi gpio config
	/* set max frequency slew rate(200M) */
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_GPIO_C_11;
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PC10_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_SPI1_MISO;
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PC13_FUNC_CTL_SPI1_MOSI;
   
	/* set max frequency slew rate(200M) */
	HPM_IOC->PAD[IOC_PAD_PC11].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);
	HPM_IOC->PAD[IOC_PAD_PC10].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);
	HPM_IOC->PAD[IOC_PAD_PC12].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);
	HPM_IOC->PAD[IOC_PAD_PC13].PAD_CTL = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);


	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_CS),GPIO_GET_PIN_INDEX(LCDGPIO_CS), 1);

	//gpio config for LCD control pin
	HPM_IOC->PAD[LCDGPIO_LED].FUNC_CTL = IOC_PC08_FUNC_CTL_GPIO_C_08;
    HPM_IOC->PAD[LCDGPIO_LED].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) |
                                        IOC_PAD_PAD_CTL_SPD_SET(3) |
                                        IOC_PAD_PAD_CTL_DS_SET(7) |
                                        IOC_PAD_PAD_CTL_OD_SET(0) |
                                        IOC_PAD_PAD_CTL_PE_SET(1) |
                                        IOC_PAD_PAD_CTL_PS_SET(1) |
                                        IOC_PAD_PAD_CTL_PRS_SET(1);

	HPM_IOC->PAD[LCDGPIO_RST].FUNC_CTL = IOC_PC03_FUNC_CTL_GPIO_C_03;
    HPM_IOC->PAD[LCDGPIO_RST].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);

	HPM_IOC->PAD[LCDGPIO_RS].FUNC_CTL = IOC_PC04_FUNC_CTL_GPIO_C_04;
    HPM_IOC->PAD[LCDGPIO_RS].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);

	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_LED),GPIO_GET_PIN_INDEX(LCDGPIO_LED), 1);
	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_RST),GPIO_GET_PIN_INDEX(LCDGPIO_RST), 1);
	gpio_set_pin_output_with_initial(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(LCDGPIO_RS),GPIO_GET_PIN_INDEX(LCDGPIO_RS), 1);
}


void LCD_GPIO_SET(u32 pin,u8 val)
{
	gpio_write_pin(LCD_GPIO_CTRL, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin), val);
}

#define LCD_SPI                    HPM_SPI1
#define LCD_SPI_SCLK_FREQ          (32000000UL)
#define LCD_SPI_DATA_LEN_BITS      (8U)
#define LCD_SPI_DATA_LEN_BYTES     ((LCD_SPI_DATA_LEN_BITS + 7) / 8)
#if (SPI_SOC_TRANSFER_COUNT_MAX == 512)
#if (LCD_SPI_DATA_LEN_BYTES > 2)
#define TRANSFER_COUNT_MAX          (SPI_SOC_TRANSFER_COUNT_MAX * 4)
#else
#define TRANSFER_COUNT_MAX          (SPI_SOC_TRANSFER_COUNT_MAX * LCD_SPI_DATA_LEN_BYTES)
#endif
#else
#define TRANSFER_COUNT_MAX          SPI_SOC_TRANSFER_COUNT_MAX
#endif



int LCD_SPIInit(void)
{
    spi_timing_config_t timing_config;
    spi_format_config_t format_config;
    uint32_t src_clk;
    uint32_t best_sclk = 0U;
    uint32_t best_error = 0xFFFFFFFFU;
    uint32_t div;

    clock_add_to_group(clock_spi1, 0);
    /* pins init*/
	//board_init_spi_pins_with_gpio_as_cs(LCD_SPI);
	LCD_GPIOInit();//LCD GPIO初始化

    src_clk = clock_get_frequency(clock_spi1);
    for (div = 2U; div <= 510U; div += 2U) {
        uint32_t sclk;
        uint32_t error;

        if ((src_clk % div) != 0U) {
            continue;
        }
        sclk = src_clk / div;
        error = (sclk > LCD_SPI_SCLK_FREQ) ?
                (sclk - LCD_SPI_SCLK_FREQ) : (LCD_SPI_SCLK_FREQ - sclk);
        if (error < best_error) {
            best_error = error;
            best_sclk = sclk;
        }
    }
    if (best_sclk == 0U) {
        return -1;
    }

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = src_clk;
    timing_config.master_config.sclk_freq_in_hz = best_sclk;
    if (spi_master_timing_init(LCD_SPI, &timing_config) != status_success) {
        printf("[LCD] spi_master_timing_init FAIL (src=%u, want=%u)\n", src_clk, best_sclk);
        return -1;
    }
    /* 日志里留一行实参: ILI9341 串行写周期最小 100ns(=10MHz), 超太多初始化就会失败 */
    printf("[LCD] SPI1 src_clk=%u Hz -> sclk=%u Hz (target %u)\n",
           src_clk, best_sclk, (unsigned)LCD_SPI_SCLK_FREQ);

    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = LCD_SPI_DATA_LEN_BITS;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    format_config.common_config.lsb = false;
    spi_format_init(LCD_SPI, &format_config);

	return 0;
}


/* ======================================================================
 *  像素通路失败计数 —— 定位"不规则长条 / 花屏"的证据
 *
 * 【为什么需要它】hpm_spi_drv.c 的 spi_write_data() 只会在 TX FIFO 满时空转,
 * 【不会丢字节】: 被中断/任务抢占只是让它变慢。所以"轮询写被抢占 -> 少发几个
 * 字节"这条因果链是站不住的, 不能当结论写。
 *
 * 真正能丢数据的只有 spi_transfer() 的两个返回值(hpm_spi_drv.c:434~503):
 *   · status_spi_master_busy : 进函数时发现 SPIACTIVE 还置着 -> 立刻返回,
 *                              【一个字节都没发】。对 LCD 来说 = 这条命令/参数/
 *                              整块像素被吞掉。
 *   · status_timeout         : spi_wait_for_idle_status() 的 5000 次重试耗尽后
 *                              返回(超时的是"等待空闲", 数据其实还在往外移)。
 * 注意这两个是【量级相当】的: 一次 512B 传输在 25MHz 下要 164us, 而 5000 次
 * 空转在几百 MHz 的核上只有几十~几百 us —— 到底会不会踩到, 必须实测。
 *
 * 原来的代码: 前者 printf 一行了事(显示任务里同步写 921600bps 串口!), 后者
 * 完全被忽略。这里改成分类计数, 由显示任务每 5 秒走日志队列汇总一行,
 * 上板跑一次就能判断到底有没有丢、丢的是哪一类。
 * ====================================================================== */
static lcd_tx_stats_t s_lcd_stats;

static void lcd_stat_fail(hpm_stat_t stat)
{
	s_lcd_stats.last_status = (u32)stat;
	if (stat == status_spi_master_busy) {
		s_lcd_stats.busy_count++;
	} else if (stat == status_timeout) {
		s_lcd_stats.timeout_count++;
	} else {
		s_lcd_stats.fail_count++;
	}
}

void LCD_GetTxStats(lcd_tx_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	*out = s_lcd_stats;
}

u8 SPI_WriteByte(u8 data)
{
	u8 recivedata = 0U;
	spi_control_config_t control_config;
	hpm_stat_t stat;

	spi_master_get_default_control_config(&control_config);
	control_config.common_config.trans_mode = spi_trans_write_read_together;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.tx_dma_enable = false;
	control_config.common_config.rx_dma_enable = false;
	stat = spi_transfer(LCD_SPI, &control_config, NULL, NULL,
	                    &data, 1U, &recivedata, 1U);
	if (stat != status_success) {
		/* 【显示任务里绝不能 printf】原来这行在 921600bps 下要 ~0.2ms, 而且
		 * 是最低优先级任务同步写串口。只计数, 汇总交给 lcd_app.c。 */
		s_lcd_stats.byte_cmd_fail++;
		lcd_stat_fail(stat);
	}

	return recivedata;
}

u8 SPI_WriteBlock(u8 *data,u32 len)
{
	u32 transfer_size;

	while (len > 0U) {
		transfer_size = len;
		if (transfer_size > TRANSFER_COUNT_MAX) {
			transfer_size = TRANSFER_COUNT_MAX;
		}

		spi_control_config_t control_config;
		hpm_stat_t stat;
		spi_master_get_default_control_config(&control_config);
		control_config.common_config.trans_mode = spi_trans_write_only;
		control_config.common_config.data_phase_fmt = spi_single_io_mode;
		control_config.common_config.tx_dma_enable = false;
		control_config.common_config.rx_dma_enable = false;
		stat = spi_transfer(LCD_SPI, &control_config, NULL, NULL,
		                    data, transfer_size, NULL, 0U);
		if (stat != status_success) {
			lcd_stat_fail(stat);
			return 1U;
		}
		s_lcd_stats.chunk_count++;
		data += transfer_size;
		len -= transfer_size;
	}

	return 0U;
}
	   
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u8 data)
{ 
   LCD_CS_CLR;     
	 LCD_RS_CLR;	  
   SPI_WriteByte(data);
   LCD_CS_SET;	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u8 data)
{
   LCD_CS_CLR;
	 LCD_RS_SET;
   SPI_WriteByte(data);
   LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(u16 Data)
{	
	Data=~Data;
	LCD_CS_CLR;
	LCD_RS_SET;  
	SPI_WriteByte(Data>>8);
	SPI_WriteByte(Data);
	LCD_CS_SET;
}

void Lcd_WriteData_Block(u8 *data,u32 len)
{	
	LCD_CS_CLR;
	LCD_RS_SET;  
	SPI_WriteBlock(data,len);
	LCD_CS_SET;
}

#define LCD_PIXEL_TRANSFER_BUFFER_SIZE (512U)
#define LCD_WRITE_RETRY_MAX            (2U)

void LCD_WritePixels(const u8 *data, u32 pixel_count)
{
	static u8 transfer_buffer[LCD_PIXEL_TRANSFER_BUFFER_SIZE];
	u32 chunk_pixels;
	u32 i;
	u32 retry;

	LCD_CS_CLR;
	LCD_RS_SET;

	while (pixel_count > 0U) {
		chunk_pixels = pixel_count;
		if (chunk_pixels > (LCD_PIXEL_TRANSFER_BUFFER_SIZE / 2U)) {
			chunk_pixels = LCD_PIXEL_TRANSFER_BUFFER_SIZE / 2U;
		}

		/* Match the master_lcd RGB565 wire format: byte-swapped and inverted. */
		for (i = 0U; i < chunk_pixels; i++) {
			transfer_buffer[i * 2U] = (u8)~data[i * 2U + 1U];
			transfer_buffer[i * 2U + 1U] = (u8)~data[i * 2U];
		}

		/* 【失败为什么改成"重发"而不是 break】
		 * 原来失败就 break: 这条 band 剩下的像素一个都不发, 后半截保持上一帧的
		 * 内容 —— 屏幕上正好就是"不规则长条"。而 spi_transfer() 失败里最常见的
		 * status_spi_master_busy 是"上一笔还在移", 隔一小会儿重发基本就成。
		 * 所以这里重发 LCD_WRITE_RETRY_MAX 次, 仍失败才放弃并计数
		 * (band_abort_count); 无论成功失败都继续往后写 —— 因为 busy 时一个字节
		 * 都没发出, GRAM 写指针没动, 继续写下一块仍然是接着的。
		 * 【注意】这一步只是"不静默丢数据", 不是对根因下结论: 根因看
		 * LCD_GetTxStats() 的 busy/timeout 计数。 */
		retry = 0U;
		while (SPI_WriteBlock(transfer_buffer, chunk_pixels * 2U) != 0U) {
			if (retry >= LCD_WRITE_RETRY_MAX) {
				s_lcd_stats.band_abort_count++;
				break;
			}
			retry++;
			s_lcd_stats.retry_count++;
		}

		data += chunk_pixels * 2U;
		pixel_count -= chunk_pixels;
	}

	LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(u16 Color)
{
  	unsigned int i,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	LCD_CS_CLR;
	LCD_RS_SET;
	for(i=0;i<lcddev.height;i++)
	{
    for(m=0;m<lcddev.width;m++)
    {	
			Lcd_WriteData_16Bit(Color);
		}
	}
	 LCD_CS_SET;
} 



/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	//SPI2_Init(); //硬件SPI2初始化
	LCD_SPIInit();										 
 	LCD_RESET(); //LCD 复位
//*************2.4inch ILI9341初始化**********//	
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xD9); //0xC1 
	LCD_WR_DATA(0X30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x12);   //SAP[2:0];BT[3:0] 0x01
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x08); 	 //30
	LCD_WR_DATA(0x26); 	 //30
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0XB7); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x08); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x1D); 
	LCD_WR_DATA(0x1A); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0D); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x49); 
	LCD_WR_DATA(0X66); 
	LCD_WR_DATA(0x3B); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x01); 
	LCD_WR_DATA(0x09); 
	LCD_WR_DATA(0x05); 
	LCD_WR_DATA(0x04); 		 
	LCD_WR_REG(0XE1);    //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x18); 
	LCD_WR_DATA(0x1D); 
	LCD_WR_DATA(0x02); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x04); 
	LCD_WR_DATA(0x36); 
	LCD_WR_DATA(0x13); 
	LCD_WR_DATA(0x4C); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x13); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x2E); 
	LCD_WR_DATA(0x2F); 
	LCD_WR_DATA(0x05); 
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	delay_ms(120);
	LCD_WR_REG(0x29); //display on

  LCD_direction(USE_HORIZONTAL);//设置LCD显示方向
	LCD_GPIO_SET(LCDGPIO_LED,1);//点亮背光	 
	LCD_Clear(WHITE);//清全屏白色
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//开始写入GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(u8 direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;	
		default:break;
	}		
}	 




