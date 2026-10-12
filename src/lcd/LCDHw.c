#include "type.h"
#include "hpm_spi_drv.h"
#include "hpm_gpio_drv.h"
#include "board.h"

#include "LCDHw.h"
#include "stdlib.h"






#define LCD_GPIO_CTRL  HPM_GPIO0

#define LCDGPIO_CS   IOC_PAD_PC11       //片选引脚            PB11
#define LCDGPIO_LED  IOC_PAD_PC08       //背光控制引脚        PB9
#define LCDGPIO_RS   IOC_PAD_PC04       //寄存器/数据选择引脚 PB10 
#define LCDGPIO_RST  IOC_PAD_PC03       //复位引脚            PB12

 
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
        printf("spi_master_timing_init fail\n");
        return -1;
    }

    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = LCD_SPI_DATA_LEN_BITS;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    format_config.common_config.lsb = false;
    spi_format_init(LCD_SPI, &format_config);

	return 0;
}


u8 SPI_WriteByte(u8 data)
{
	u8 recivedata = 0U;
	spi_control_config_t control_config;

	spi_master_get_default_control_config(&control_config);
	control_config.common_config.trans_mode = spi_trans_write_read_together;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.tx_dma_enable = false;
	control_config.common_config.rx_dma_enable = false;
	if (spi_transfer(LCD_SPI, &control_config, NULL, NULL,
	                 &data, 1U, &recivedata, 1U) != status_success) {
		printf("spi_transfer fail\n");
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
		spi_master_get_default_control_config(&control_config);
		control_config.common_config.trans_mode = spi_trans_write_only;
		control_config.common_config.data_phase_fmt = spi_single_io_mode;
		control_config.common_config.tx_dma_enable = false;
		control_config.common_config.rx_dma_enable = false;
		if (spi_transfer(LCD_SPI, &control_config, NULL, NULL,
		                 data, transfer_size, NULL, 0U) != status_success) {
			printf("spi_transfer fail\n");
			return 1U;
		}
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

void LCD_WritePixels(const u8 *data, u32 pixel_count)
{
	static u8 transfer_buffer[LCD_PIXEL_TRANSFER_BUFFER_SIZE];
	u32 chunk_pixels;
	u32 i;

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

		if (SPI_WriteBlock(transfer_buffer, chunk_pixels * 2U) != 0U) {
			break;
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




