/**
 * Original code from www.lcdwiki.com, see copyright and info below.
 * 
 * Modified for use in ADSB Alert
*/

//--Start lcdwiki----------------------------------------------------------------
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32H743IIT6,����ԭ��Apollo STM32F4/F7������,��Ƶ400MHZ������25MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/22
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
//--End lcdwiki------------------------------------------------------------------



//=================================================================================
//     LCD                   STM32
//      VCC                  DC5V/3.3V	(5V preferred)
//      GND                  GND
//=================================================================================

// Pins changed for ADSB project, see the 'NEW' column below.

//     LCD              	   OLD    		NEW
//    SDI(MOSI)                PB15			C12
//    SDO(MISO)                PB14			C11
//==================================================================================
//     LCD 					 	OLD			NEW
//     LED                   	PD6       	Not defined (yet), would be useful for backlight dimming
//     SCK                   	PB13      	C10
//     LCD_RS                  	PD5       	D1 - this is normally called DC for data/command, no idea where they got 'rs' from
//     LCD_RST                 	PD12      	D0
//     LCD_CS                  	PD11      	D2
//==================================================================================

// TODO Touch Panel pins

//	   LCD                   OLD		NEW
//     CTP_INT               PH11
//     CTP_SDA               PI3
//     CTP_RST               PI8
//     CTP_SCL               PH6


#ifndef __LCD_H
#define __LCD_H		
// #include "sys.h"	 
#include "stdlib.h"
#include "main.h"

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;



typedef struct  
{										    
	u16 width;
	u16 height;
	u16 id;
	u8  dir;
	u16	 wramcmd;
	u16  rramcmd;
	u16  setxcmd;
	u16  setycmd;
}_lcd_dev; 	


extern SPI_HandleTypeDef hspi3;

extern _lcd_dev lcddev;

#define USE_HORIZONTAL  	 0	// Might be rotation, values 0 through 3

#define LCD_W 320
#define LCD_H 480

extern u16  POINT_COLOR;
extern u16  BACK_COLOR;

////////////////////////////////////////////////////////////////////

#ifdef LCD2_BACKLIGHT_PIN
#define	LCD_LED(n) (n ? HAL_GPIO_WritePin(LCD2_BACKLIGHT_PORT,LCD2_BACKLIGHT_PIN, GPIO_PIN_SET):HAL_GPIO_WritePin(LCD2_BACKLIGHT_PORT,LCD2_BACKLIGHT_PIN,GPIO_PIN_RESET))
#else
#define LCD_LED(n)
#endif

// 
#define	LCD_CS_SET  HAL_GPIO_WritePin(LCD2_CS_GPIO_Port, LCD2_CS_Pin, GPIO_PIN_SET);
#define	LCD_RS_SET	HAL_GPIO_WritePin(LCD2_DC_GPIO_Port, LCD2_DC_Pin, GPIO_PIN_SET);
#define	LCD_RST_SET	HAL_GPIO_WritePin(LCD2_RESET_GPIO_Port, LCD2_RESET_Pin, GPIO_PIN_SET);

#define	LCD_CS_CLR  HAL_GPIO_WritePin(LCD2_CS_GPIO_Port, LCD2_CS_Pin, GPIO_PIN_RESET);
#define	LCD_RS_CLR	HAL_GPIO_WritePin(LCD2_DC_GPIO_Port, LCD2_DC_Pin, GPIO_PIN_RESET);
#define	LCD_RST_CLR	HAL_GPIO_WritePin(LCD2_RESET_GPIO_Port, LCD2_RESET_Pin, GPIO_PIN_RESET);

// XXX get rid of these colour defs and use the UG ones instead?

#define D_WHITE       0xFFFF
// #define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 		0XFFE0
#define GBLUE		0X07FF
// #define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 		0XBC40
#define BRRED 		0XFC07
#define GRAY  		0X8430
#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458
#define LIGHTGREEN	0X841F
#define LIGHTGRAY   0XEF5B
#define LGRAY 		0XC618
#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12
	    															  
void LCD_Init(void);
// void LCD_DisplayOn(void);
// void LCD_DisplayOff(void);
void LCD_Clear(u16 Color);	 
void LCD_SetCursor(u16 Xpos, u16 Ypos);
void LCD_DrawPoint(u16 x,u16 y);
void LCD_DrawPixel(int16_t x, int16_t y, uint16_t color);
u16  LCD_ReadPoint(u16 x,u16 y);
// void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
// void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);		   
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);
void LCD_WriteWindow(u16 x, u16 y, u16 width, u16 height, u16 *buf);

u8   LCD_RD_DATA(void);
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
void LCD_WR_DATA(u8 data);
u8   LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
u16  LCD_ReadRAM(void);		   
u16  LCD_BGR2RGB(u16 c);
void LCD_SetParam(void);
void Lcd_WriteData_16Bit(u16 Data);
void LCD_direction(u8 direction );
u16  LCD_Read_ID(void);			  		 

#endif  // __LCD_H
