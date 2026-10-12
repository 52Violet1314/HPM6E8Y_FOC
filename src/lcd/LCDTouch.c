#include "type.h"

#include "LCDTouch.h" 
#include "LCDHw.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "LCDGui.h"	

#include "hpm_spi_drv.h"
#include "hpm_gpio_drv.h"
#include "board.h"


//#define sqrt(x) x


#define TC_COMM_MODE_SPI	0

#define TC_GPIO_CTRL  	HPM_GPIO0
#define TC_GPIO_IRQ		IOC_PAD_PC09

#define TC_GPIO_SPICS	IOC_PAD_PC21
#define TC_GPIO_SPICLK	IOC_PAD_PC20
#define TC_GPIO_SPIMISO	IOC_PAD_PC22
#define TC_GPIO_SPIMOSI	IOC_PAD_PC23

void TC_GPIOInit(void)
{
	u32 padctl;

	padctl=IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);
	/* set max frequency slew rate(200M) */
    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PC21_FUNC_CTL_GPIO_C_21;
    HPM_IOC->PAD[IOC_PAD_PC21].PAD_CTL = padctl;

    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PC20_FUNC_CTL_GPIO_C_20;
    HPM_IOC->PAD[IOC_PAD_PC20].PAD_CTL = padctl;

    HPM_IOC->PAD[IOC_PAD_PC22].FUNC_CTL = IOC_PC22_FUNC_CTL_GPIO_C_22;
    HPM_IOC->PAD[IOC_PAD_PC22].PAD_CTL = padctl;

    HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PC23_FUNC_CTL_GPIO_C_23;
    HPM_IOC->PAD[IOC_PAD_PC23].PAD_CTL = padctl;

	gpio_set_pin_output_with_initial(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_SPICS),GPIO_GET_PIN_INDEX(TC_GPIO_SPICS), 1);
	gpio_set_pin_output_with_initial(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_SPICLK),GPIO_GET_PIN_INDEX(TC_GPIO_SPICLK), 1);
	gpio_set_pin_output_with_initial(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_SPIMOSI),GPIO_GET_PIN_INDEX(TC_GPIO_SPIMOSI), 1);
	gpio_set_pin_input(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_SPIMISO),GPIO_GET_PIN_INDEX(TC_GPIO_SPIMISO));

	HPM_IOC->PAD[TC_GPIO_IRQ].FUNC_CTL = IOC_PC09_FUNC_CTL_GPIO_C_09;
    HPM_IOC->PAD[TC_GPIO_IRQ].PAD_CTL = padctl;

	gpio_set_pin_input(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_IRQ),GPIO_GET_PIN_INDEX(TC_GPIO_IRQ));
}

void TC_GPIOInit_SPI(void)
{
	//spi gpio config
	/* set max frequency slew rate(200M) */
    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PC21_FUNC_CTL_GPIO_C_21;
    HPM_IOC->PAD[IOC_PAD_PC21].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);

    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PC20_FUNC_CTL_SPI3_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC20].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);

    HPM_IOC->PAD[IOC_PAD_PC22].FUNC_CTL = IOC_PC22_FUNC_CTL_SPI3_MISO;
    HPM_IOC->PAD[IOC_PAD_PC22].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);

    HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PC23_FUNC_CTL_SPI3_MOSI;
    HPM_IOC->PAD[IOC_PAD_PC23].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);

	gpio_set_pin_output_with_initial(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_SPICS),GPIO_GET_PIN_INDEX(TC_GPIO_SPICS), 1);


	HPM_IOC->PAD[TC_GPIO_IRQ].FUNC_CTL = IOC_PC09_FUNC_CTL_GPIO_C_09;
    HPM_IOC->PAD[TC_GPIO_IRQ].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_HYS_SET(1);

	gpio_set_pin_input(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(TC_GPIO_IRQ),GPIO_GET_PIN_INDEX(TC_GPIO_IRQ));
}


u8 TC_GPIOGet(u32 pin)
{
	delay_us(1);
	return gpio_read_pin(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin));
}

void TC_GPIOSet(u32 pin, u8 val)
{
	gpio_write_pin(TC_GPIO_CTRL, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin), val);
}


#define TC_SPI                    HPM_SPI3
#define TC_SPI_SCLK_FREQ          (2000000UL)
#define TC_SPI_DATA_LEN_BITS      (8U)




int TC_SPIInit(void)
{
    spi_timing_config_t timing_config;
    spi_format_config_t format_config;

    clock_add_to_group(clock_spi3, 0);
    /* pins init*/
	TC_GPIOInit_SPI();
    //board_init_spi_pins_with_gpio_as_cs(LCD_SPI);

    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(clock_spi3);
    timing_config.master_config.sclk_freq_in_hz = TC_SPI_SCLK_FREQ;
    if (spi_master_timing_init(TC_SPI, &timing_config) != status_success) {
        return -1;
    }

    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = TC_SPI_DATA_LEN_BITS;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
    format_config.common_config.lsb = false;
    spi_format_init(TC_SPI, &format_config);

	return 0;
}


u8 TC_SPIRW(u8 data)
{
	u8 read_data;
	spi_control_config_t control_config;
	spi_master_get_default_control_config(&control_config);
	control_config.common_config.trans_mode = spi_trans_write_read_together;
	control_config.common_config.data_phase_fmt = spi_single_io_mode;
	control_config.common_config.tx_dma_enable = false;
	control_config.common_config.rx_dma_enable = false;
	spi_transfer(TC_SPI, &control_config, NULL, NULL, &data, 1U, &read_data, 1U);

	return read_data;	
}

_m_tp_dev tp_dev=
{
	TP_Init,
	TP_Scan,
	TP_Adjust,
	0,
	0,
 	0,
	0,
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 		
};					
//Ĭ��Ϊtouchtype=0������.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;

/*****************************************************************************
 * @name       :void TP_Write_Byte(u8 num)   
 * @date       :2018-08-09 
 * @function   :Write a byte data to the touch screen IC with SPI bus
 * @parameters :num:Data to be written
 * @retvalue   :None
******************************************************************************/  	 			    					   
void TP_Write_Byte(u8 num)    
{  
#if !TC_COMM_MODE_SPI	
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)
			TC_GPIOSet(TC_GPIO_SPIMOSI,1);  
		else 
			TC_GPIOSet(TC_GPIO_SPIMOSI,0);   
		num<<=1;    
		TC_GPIOSet(TC_GPIO_SPICLK,0); 	 
		TC_GPIOSet(TC_GPIO_SPICLK,1);		//��������Ч	        
	}	
#endif	 			    
}

/*****************************************************************************
 * @name       :u16 TP_Read_AD(u8 CMD)	  
 * @date       :2018-08-09 
 * @function   :Reading adc values from touch screen IC with SPI bus
 * @parameters :CMD:Read command,0xD0 for x,0x90 for y
 * @retvalue   :Read data
******************************************************************************/    
u16 TP_Read_AD(u8 CMD)	  
{ 	
#if !TC_COMM_MODE_SPI	
	
	u8 count=0; 	  
	u16 Num=0; 
	TC_GPIOSet(TC_GPIO_SPICLK,0);		//������ʱ�� 	 
	TC_GPIOSet(TC_GPIO_SPIMOSI,0); 	//����������
	TC_GPIOSet(TC_GPIO_SPICS,0); 		//ѡ�д�����IC
	TP_Write_Byte(CMD);//����������
	delay_us(6);//ADS7846��ת��ʱ���Ϊ6us
	TC_GPIOSet(TC_GPIO_SPICLK,0); 	     	    
	delay_us(1);    	   
	TC_GPIOSet(TC_GPIO_SPICLK,1);		//��1��ʱ�ӣ����BUSY	    	    
	TC_GPIOSet(TC_GPIO_SPICLK,0); 	     	    
	for(count=0;count<16;count++)//����16λ����,ֻ�и�12λ��Ч 
	{ 				  
		Num<<=1; 	 
		TC_GPIOSet(TC_GPIO_SPICLK,0);	//�½�����Ч  	    	   
		TC_GPIOSet(TC_GPIO_SPICLK,1);
		if(TC_GPIOGet(TC_GPIO_SPIMISO))Num++; 		 
	}  	
	Num>>=4;   	//ֻ�и�12λ��Ч.
	TC_GPIOSet(TC_GPIO_SPICS,1);		//�ͷ�Ƭѡ	 
	return(Num); 
#else
	u8 data[2];
	u16 adcval;

	TC_GPIOSet(TC_GPIO_SPICS,0);
	TC_SPIRW(CMD);
	delay_us(6); 
	data[0]=TC_SPIRW(0);
	data[1]=TC_SPIRW(0);

	TC_GPIOSet(TC_GPIO_SPICS,1);

	adcval= data[0]<<8 | data[1];

	return adcval>>4;
#endif
}

#define READ_TIMES 5 	//��ȡ����
#define LOST_VAL 1	  	//����ֵ
/*****************************************************************************
 * @name       :u16 TP_Read_XOY(u8 xy)  
 * @date       :2018-08-09 
 * @function   :Read the touch screen coordinates (x or y),
								Read the READ_TIMES secondary data in succession 
								and sort the data in ascending order,
								Then remove the lowest and highest number of LOST_VAL 
								and take the average
 * @parameters :xy:Read command(CMD_RDX/CMD_RDY)
 * @retvalue   :Read data
******************************************************************************/  
u16 TP_Read_XOY(u8 xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;

	for(i=0;i<READ_TIMES;i++)
		buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
		sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);

	return temp;   
} 

/*****************************************************************************
 * @name       :u8 TP_Read_XY(u16 *x,u16 *y)
 * @date       :2018-08-09 
 * @function   :Read touch screen x and y coordinates,
								The minimum value can not be less than 100
 * @parameters :x:Read x coordinate of the touch screen
								y:Read y coordinate of the touch screen
 * @retvalue   :0-fail,1-success
******************************************************************************/ 
u8 TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  												   
	//if(xtemp<100||ytemp<100)return 0;//����ʧ��
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}

#define ERR_RANGE 50 //��Χ 
/*****************************************************************************
 * @name       :u8 TP_Read_XY2(u16 *x,u16 *y) 
 * @date       :2018-08-09 
 * @function   :Read the touch screen coordinates twice in a row, 
								and the deviation of these two times can not exceed ERR_RANGE, 
								satisfy the condition, then think the reading is correct, 
								otherwise the reading is wrong.
								This function can greatly improve the accuracy.
 * @parameters :x:Read x coordinate of the touch screen
								y:Read y coordinate of the touch screen
 * @retvalue   :0-fail,1-success
******************************************************************************/ 
u8 TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    

    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)
		return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)
		return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else 
		return 0;	  
} 

/*****************************************************************************
 * @name       :void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
 * @date       :2018-08-09 
 * @function   :Draw a touch point,Used to calibrate							
 * @parameters :x:Read x coordinate of the touch screen
								y:Read y coordinate of the touch screen
								color:the color value of the touch point
 * @retvalue   :None
******************************************************************************/  
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y);//����
	LCD_DrawLine(x,y-12,x,y+13);//����
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	gui_circle(x,y,POINT_COLOR,6,0);//������Ȧ
}	

/*****************************************************************************
 * @name       :void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
 * @date       :2018-08-09 
 * @function   :Draw a big point(2*2)					
 * @parameters :x:Read x coordinate of the point
								y:Read y coordinate of the point
								color:the color value of the point
 * @retvalue   :None
******************************************************************************/   
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y);//���ĵ� 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}	

/*****************************************************************************
 * @name       :u8 TP_Scan(u8 tp)
 * @date       :2018-08-09 
 * @function   :Scanning touch event				
 * @parameters :tp:0-screen coordinate 
									 1-Physical coordinates(For special occasions such as calibration)
 * @retvalue   :Current touch screen status,
								0-no touch
								1-touch
******************************************************************************/  					  
u8 TP_Scan(u8 tp)
{			   
	if(TC_GPIOGet(TC_GPIO_IRQ)==0)//�а�������
	{
		if(tp)TP_Read_XY2(&tp_dev.x,&tp_dev.y);//��ȡ��������
		else if(TP_Read_XY2(&tp_dev.x,&tp_dev.y))//��ȡ��Ļ����
		{
	 		tp_dev.x=tp_dev.xfac*tp_dev.x+tp_dev.xoff;//�����ת��Ϊ��Ļ����
			tp_dev.y=tp_dev.yfac*tp_dev.y+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//֮ǰû�б�����
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//��������  
			tp_dev.x0=tp_dev.x;//��¼��һ�ΰ���ʱ������
			tp_dev.y0=tp_dev.y;  	   			 
		}			   
	}else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~(1<<7);//��ǰ����ɿ�	
		}else//֮ǰ��û�б�����
		{
			tp_dev.x0=0;
			tp_dev.y0=0;
			tp_dev.x=0xffff;
			tp_dev.y=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//���ص�ǰ�Ĵ���״̬
}
	  
//////////////////////////////////////////////////////////////////////////	 
//������EEPROM����ĵ�ַ�����ַ,ռ��13���ֽ�(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE 40
/*****************************************************************************
 * @name       :void TP_Save_Adjdata(void)
 * @date       :2018-08-09 
 * @function   :Save calibration parameters		
 * @parameters :None
 * @retvalue   :None
******************************************************************************/ 										    
void TP_Save_Adjdata(void)
{
#if 0	
	s32 temp;			 
	//����У�����!		   							  
	temp=tp_dev.xfac*100000000;//����xУ������      
    AT24CXX_WriteLenByte(SAVE_ADDR_BASE,temp,4);   
	temp=tp_dev.yfac*100000000;//����yУ������    
    AT24CXX_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);
	//����xƫ����
    AT24CXX_WriteLenByte(SAVE_ADDR_BASE+8,tp_dev.xoff,2);		    
	//����yƫ����
	AT24CXX_WriteLenByte(SAVE_ADDR_BASE+10,tp_dev.yoff,2);	
	//���津������
	AT24CXX_WriteOneByte(SAVE_ADDR_BASE+12,tp_dev.touchtype);	
	temp=0X0A;//���У׼����
	AT24CXX_WriteOneByte(SAVE_ADDR_BASE+13,temp); 
#endif
}

/*****************************************************************************
 * @name       :u8 TP_Get_Adjdata(void)
 * @date       :2018-08-09 
 * @function   :Gets the calibration values stored in the EEPROM		
 * @parameters :None
 * @retvalue   :1-get the calibration values successfully
								0-get the calibration values unsuccessfully and Need to recalibrate
******************************************************************************/ 	
u8 TP_Get_Adjdata(void)
{	
#if 0				  
	s32 tempfac;
	tempfac=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+13);//��ȡ�����,���Ƿ�У׼���� 		 
	if(tempfac==0X0A)//�������Ѿ�У׼����			   
	{    												 
		tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE,4);		   
		tp_dev.xfac=(float)tempfac/100000000;//�õ�xУ׼����
		tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+4,4);			          
		tp_dev.yfac=(float)tempfac/100000000;//�õ�yУ׼����
	    //�õ�xƫ����
		tp_dev.xoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+8,2);			   	  
 	    //�õ�yƫ����
		tp_dev.yoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+10,2);				 	  
 		tp_dev.touchtype=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+12);//��ȡ�������ͱ��
		if(tp_dev.touchtype)//X,Y��������Ļ�෴
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	 
		}else				   //X,Y��������Ļ��ͬ
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
#endif
	return 1;
}	
 
//��ʾ�ַ���
const u8* TP_REMIND_MSG_TBL="Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.";

/*****************************************************************************
 * @name       :void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac)
 * @date       :2018-08-09 
 * @function   :Display calibration results	
 * @parameters :x0:the x coordinates of first calibration point
								y0:the y coordinates of first calibration point
								x1:the x coordinates of second calibration point
								y1:the y coordinates of second calibration point
								x2:the x coordinates of third calibration point
								y2:the y coordinates of third calibration point
								x3:the x coordinates of fourth calibration point
								y3:the y coordinates of fourth calibration point
								fac:calibration factor 
 * @retvalue   :None
******************************************************************************/ 	 					  
void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac)
{	  
	POINT_COLOR=RED;
	LCD_ShowString(40,140,16,"x1:",1);
 	LCD_ShowString(40+80,140,16,"y1:",1);
 	LCD_ShowString(40,160,16,"x2:",1);
 	LCD_ShowString(40+80,160, 16,"y2:",1);
	LCD_ShowString(40,180, 16,"x3:",1);
 	LCD_ShowString(40+80,180, 16,"y3:",1);
	LCD_ShowString(40,200, 16,"x4:",1);
 	LCD_ShowString(40+80,200, 16,"y4:",1);  
 	LCD_ShowString(40,220, 16,"fac is:",1);     
	LCD_ShowNum(40+24,140,x0,4,16);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,140,y0,4,16);	//��ʾ��ֵ
	LCD_ShowNum(40+24,160,x1,4,16);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,160,y1,4,16);	//��ʾ��ֵ
	LCD_ShowNum(40+24,180,x2,4,16);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,180,y2,4,16);	//��ʾ��ֵ
	LCD_ShowNum(40+24,200,x3,4,16);		//��ʾ��ֵ
	LCD_ShowNum(40+24+80,200,y3,4,16);	//��ʾ��ֵ
 	LCD_ShowNum(40+56,220,fac,3,16); 	//��ʾ��ֵ,����ֵ������95~105��Χ֮��.
}

/*****************************************************************************
 * @name       :u8 TP_Get_Adjdata(void)
 * @date       :2018-08-09 
 * @function   :Calibration touch screen and Get 4 calibration parameters
 * @parameters :None
 * @retvalue   :None
******************************************************************************/ 		 
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//���껺��ֵ
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	float fac; 	
	u16 outtime=0;
 	cnt=0;				
	POINT_COLOR=BLUE;
	BACK_COLOR =WHITE;
	LCD_Clear(WHITE);//����   
	POINT_COLOR=RED;//��ɫ 
	LCD_Clear(WHITE);//���� 	   
	POINT_COLOR=BLACK;
	LCD_ShowString(10,40,16,"Please use the stylus click",1);//��ʾ��ʾ��Ϣ
	LCD_ShowString(10,56,16,"the cross on the screen.",1);//��ʾ��ʾ��Ϣ
	LCD_ShowString(10,72,16,"The cross will always move",1);//��ʾ��ʾ��Ϣ
	LCD_ShowString(10,88,16,"until the screen adjustment",1);//��ʾ��ʾ��Ϣ
  	LCD_ShowString(10,104,16,"is completed.",1);//��ʾ��ʾ��Ϣ
	 
	TP_Drow_Touch_Point(20,20,RED);//����1 
	tp_dev.sta=0;//���������ź� 
	tp_dev.xfac=0;//xfac��������Ƿ�У׼��,����У׼֮ǰ�������!�������	 
	while(1)//�������10����û�а���,���Զ��˳�
	{
		tp_dev.scan(1);//ɨ����������
		if((tp_dev.sta&0xc0)==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{	
			outtime=0;		
			tp_dev.sta&=~(1<<6);//��ǰ����Ѿ�����������.
						   			   
			pos_temp[cnt][0]=tp_dev.x;
			pos_temp[cnt][1]=tp_dev.y;
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//�����1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//����2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//�����2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//����3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//�����3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//����4
					break;
				case 4:	 //ȫ���ĸ����Ѿ��õ�
	    		    //�Ա����
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,2�ľ���
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�3,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,3�ľ���
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
						continue;
					}//��ȷ��
								   
					//�Խ������
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,4�ľ���
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,3�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
						continue;
					}//��ȷ��
					//������
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//�õ�xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//�õ�xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//�õ�yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//�õ�yoff  
					if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//������Ԥ����෴��.
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
						LCD_ShowString(40,26, 16,"TP Need readjust!",1);
						tp_dev.touchtype=!tp_dev.touchtype;//�޸Ĵ�������.
						if(tp_dev.touchtype)//X,Y��������Ļ�෴
						{
							CMD_RDX=0X90;
							CMD_RDY=0XD0;	 
						}else				   //X,Y��������Ļ��ͬ
						{
							CMD_RDX=0XD0;
							CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					POINT_COLOR=BLUE;
					LCD_Clear(WHITE);//����
					LCD_ShowString(35,110, 16,"Touch Screen Adjust OK!",1);//У�����
					delay_ms(1000);
					TP_Save_Adjdata();  
 					LCD_Clear(WHITE);//����   
					return;//У�����				 
			}
		}
		delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			TP_Get_Adjdata();
			break;
	 	} 
 	}
}		



/*****************************************************************************
 * @name       :u8 TP_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization touch screen
 * @parameters :None
 * @retvalue   :0-no calibration
								1-Has been calibrated
******************************************************************************/  
u8 TP_Init(void)
{			    		   
	//ע��,ʱ��ʹ��֮��,��GPIO�Ĳ�������Ч

	#if TC_COMM_MODE_SPI
 	TC_SPIInit();   
	#else
	TC_GPIOInit();
	#endif

  	TP_Read_XY(&tp_dev.x,&tp_dev.y);//��һ�ζ�ȡ��ʼ��	 
 	//AT24CXX_Init();//��ʼ��24CXX
	if(TP_Get_Adjdata())
		return 0;//�Ѿ�У׼
	else			   //δУ׼?
	{ 										    
		LCD_Clear(WHITE);//����
	    TP_Adjust();  //��ĻУ׼ 
		TP_Save_Adjdata();	 
	}			
	TP_Get_Adjdata();	
	return 1; 									 
}




