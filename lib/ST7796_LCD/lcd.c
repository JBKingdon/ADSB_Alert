

//--Start lcdwiki----------------------------------------------------------------
// ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
// ����Ӳ������Ƭ��STM32H743IIT6,����ԭ��Apollo STM32F4/F7������,��Ƶ400MHZ������25MHZ
// QDtech-TFTҺ������ for STM32 IOģ��
// xiao��@ShenZhen QDtech co.,LTD
// ��˾��վ:www.qdtft.com
// �Ա���վ��http://qdtech.taobao.com
// wiki������վ��http://www.lcdwiki.com
// ��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
// �̻�(����) :+86 0755-23594567
// �ֻ�:15989313508���빤��
// ����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com
// ����֧��QQ:3002773612  3002778157
// ��������QQȺ:324828016
// ��������:2018/08/22
// �汾��V1.0
// ��Ȩ���У�����ؾ���
// Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
// All rights reserved
//--End lcdwiki------------------------------------------------------------------

// Modified for use in ADSB Alert

#include "lcdST7796.h"
#include "stdlib.h"
#include "stdint.h"
#include "main.h"
// #include "delay.h"
// #include "spi.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdio.h>

extern bool spiTransmitInProgress;

_lcd_dev lcddev;

u16 POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
u16 DeviceCode;


/**
 * Change spi data rate without doing a full re-init
*/
void SPI_SetSpeed(u32 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    __HAL_SPI_DISABLE(&hspi3);
    hspi3.Instance->CFG1&=~(0X7<<28);
    hspi3.Instance->CFG1|=SPI_BaudRatePrescaler;
    __HAL_SPI_ENABLE(&hspi3);
}


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
	HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
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
	HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
	LCD_CS_SET;
}

// TODO check how this is used and see if we can use HAL_SPI_Receive instead
u8 LCD_RD_DATA(void)
{
	u8 txData, rxData;
	LCD_CS_CLR;
	LCD_RS_SET;
	SPI_SetSpeed(SPI_BAUDRATEPRESCALER_32);
	
	txData = 0xFF;
	HAL_SPI_TransmitReceive(&hspi3, &txData, &rxData, 1, HAL_MAX_DELAY);       

	SPI_SetSpeed(SPI_BAUDRATEPRESCALER_8);
	LCD_CS_SET;
	return rxData;
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

u8 LCD_ReadReg(u8 LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);
	return LCD_RD_DATA();
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

void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
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
	LCD_CS_CLR;
	LCD_RS_SET;
	// SPI2_ReadWriteByte(Data >> 8);
	// SPI2_ReadWriteByte(Data);
	uint8_t data[] = {Data >> 8, Data & 0xFF};
	HAL_SPI_Transmit(&hspi3, data, 2, HAL_MAX_DELAY);
	LCD_CS_SET;
}

u16 Lcd_ReadData_16Bit(void)
{
	uint8_t txData;
	uint8_t r, g;
	LCD_CS_CLR;
	LCD_RS_CLR;
	// SPI2_ReadWriteByte(lcddev.rramcmd);
	txData = lcddev.rramcmd;
	HAL_SPI_Transmit(&hspi3, &txData, 1, HAL_MAX_DELAY);
	SPI_SetSpeed(SPI_BAUDRATEPRESCALER_32);
	LCD_RS_SET;
	// SPI2_ReadWriteByte(0xFF);
	txData = 0xFF;
	HAL_SPI_TransmitReceive(&hspi3, &txData, &r, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, &g, 1, HAL_MAX_DELAY);

	// r = SPI2_ReadWriteByte(0xFF);
	// g = SPI2_ReadWriteByte(0xFF);
	SPI_SetSpeed(SPI_BAUDRATEPRESCALER_8);
	LCD_CS_SET;

	uint16_t result = r << 8 | g;
	return result;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
				y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/
void LCD_DrawPoint(u16 x, u16 y)
{
	LCD_SetCursor(x, y);
	Lcd_WriteData_16Bit(POINT_COLOR);
}

/**
 * Set a pixel to the given colour
 * 
 * Added for UG integration
*/
void LCD_DrawPixel(int16_t x, int16_t y, uint16_t colour)
{
	LCD_SetCursor(x, y);
	Lcd_WriteData_16Bit(colour);
}


u16 LCD_ReadPoint(u16 x, u16 y)
{
	u16 color;
	LCD_SetCursor(x, y);
	color = Lcd_ReadData_16Bit();
	return color;
}

/**
 * Enable or disable the memory increment on the dma stream.
 * 
 * Allows filling with a solid colour without needing a large
 * source buffer.
*/
void setDMAmemInc(const bool inc)
{
	DMA_HandleTypeDef *hdma = hspi3.hdmatx;

	uint32_t registerValue = ((DMA_Stream_TypeDef *)hdma->Instance)->CR;

    if (inc) {
		registerValue |= DMA_SxCR_MINC;
	} else {
    	registerValue &= ~DMA_SxCR_MINC;
	}

	/* Write to DMA Stream CR register */
	((DMA_Stream_TypeDef *)hdma->Instance)->CR = registerValue;
}

/*
 * @brief Sets SPI interface transfer size (0=8bit, 1=16 bit)
 * @param size 0=8bit, 1=16 bit
 * @return none
 */
static void SPI_SetSize(uint8_t size)
{
	static uint8_t spiSize = 0; // Persistent across calls

	if (spiSize != size)
	{
		__HAL_SPI_DISABLE(&hspi3);
		spiSize = size;
		if (size == 1)
		{
			hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
			// The HAL doesn't allow dynamic changes to datasize, so the code just writes the register directly

			// H723 uses lowest 5 bits of CFG1 to hold datasize-1
			hspi3.Instance->CFG1 = (hspi3.Instance->CFG1 & 0xFFFFFFE0) | SPI_DATASIZE_16BIT;
		}
		else
		{
			hspi3.Init.DataSize = SPI_DATASIZE_8BIT;

			// H723 uses lowest 5 bits of CFG1 to hold datasize-1
			hspi3.Instance->CFG1 = (hspi3.Instance->CFG1 & 0xFFFFFFE0) | SPI_DATASIZE_8BIT;
		}
	}
}

// Buffer used for LCD_Clear when using DMA
// Needs to be aligned for cache management. Use a full cacheline (32 bytes) to prevent side effects
ALIGN_32BYTES (uint16_t fillBuffer[16]);

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
 ******************************************************************************/
void LCD_Clear(u16 Color)
{
	const uint32_t CLEAR_CHUNKSIZE = 62 * 1024 / 2;	// divide by 2 since we're transferring 16bit words

	// enable writing to the entire screen area
	LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);

	// Switch SPI to 16 bit mode
	SPI_SetSize(1);

	// turn off memory increment on the dma stream
	setDMAmemInc(false);

	LCD_CS_CLR;
	LCD_RS_SET;

	// only using the first word of the buffer since mem increment is disabled
	fillBuffer[0] = Color;

	// flush the dcache for the buffer so that the data is in ram for the dma to use
	SCB_CleanDCache_by_Addr((uint32_t *) fillBuffer, sizeof(uint16_t));

	uint32_t pixelsToClear = lcddev.width * lcddev.height;

	while(pixelsToClear > 0) {
		uint32_t thisChunk = pixelsToClear > CLEAR_CHUNKSIZE ? CLEAR_CHUNKSIZE : pixelsToClear;
		HAL_StatusTypeDef status = HAL_BUSY;
		while (status == HAL_BUSY) {
			// DMA is transferring 16 bit words, and the last param to the call is in words not bytes
			status = HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)fillBuffer, thisChunk);
			if (status == HAL_BUSY) osThreadYield();
		}
		spiTransmitInProgress = true;
		pixelsToClear -= thisChunk;
	}

	// Need to wait for the transfer to complete before releasing CS
	while (spiTransmitInProgress) {
		osThreadYield();
	}

	LCD_CS_SET;

	// Reset the SPI to 8 bit mode
	SPI_SetSize(0);

	// re-enable memory increment on the dma stream
	setDMAmemInc(true);
}

/**
 * LCD_WriteWindow(u16 x1, u16 y1, u16 x2, u16 y2, u16 *buf)
 * 
 * Copy the buffer to the specified window
*/
void LCD_WriteWindow(u16 x1, u16 y1, u16 x2, u16 y2, u16 *buf)
{
	HAL_StatusTypeDef status;

	const uint32_t CHUNKSIZE = 48*1024;
	LCD_SetWindows(x1, y1, x2, y2);

	uint32_t bytesToSend = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;
	uint8_t * pData = (uint8_t *)buf;

	// Make sure that the buffer is in RAM for the dma to read
	SCB_CleanDCache_by_Addr((uint32_t *) buf, bytesToSend);

	// Switch to 16 bit transfers
	SPI_SetSize(1);

	LCD_CS_CLR;
	LCD_RS_SET;

	while(bytesToSend > 0) {
		const uint32_t bytesToWrite = bytesToSend > CHUNKSIZE ? CHUNKSIZE : bytesToSend;

		status = HAL_BUSY;
		while (status == HAL_BUSY) {
			status = HAL_SPI_Transmit_DMA(&hspi3, pData, bytesToWrite/2);
			if (status == HAL_BUSY) osThreadYield();
		}
		spiTransmitInProgress = true;

		pData += bytesToWrite;
		bytesToSend -= bytesToWrite;
	}

	// Need to wait for the transfer to complete before releasing CS
	while (spiTransmitInProgress) {
		osThreadYield();
	}

	LCD_CS_SET;

	// restore 8 bit transfers
	SPI_SetSize(0);
}

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
 ******************************************************************************/
void LCD_GPIOInit(void)
{
	// gpios are setup by lcd2_gpio_init in main.c

	// Initial values for pins
	HAL_GPIO_WritePin(LCD2_RESET_GPIO_Port, LCD2_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD2_CS_GPIO_Port, LCD2_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD2_DC_GPIO_Port, LCD2_DC_Pin, GPIO_PIN_SET);

	LCD_LED(1);
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
	HAL_Delay(100);
	LCD_RST_SET;
	HAL_Delay(50);

}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
 ******************************************************************************/
void LCD_Init(void)
{
	// SPI2_Init();
	LCD_GPIOInit();
	LCD_RESET();

	// 3.5 ST7796S IPS
	// TODO - document what all this does, replace the magic numbers with constants
	LCD_WR_REG(0x11);

	HAL_Delay(120); // Delay 120ms

	LCD_WR_REG(0x36); // Memory Data Access Control MY,MX~~
	LCD_WR_DATA(0x48);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x55); // LCD_WR_DATA(0x66);

	LCD_WR_REG(0xF0); // Command Set Control
	LCD_WR_DATA(0xC3);

	LCD_WR_REG(0xF0);
	LCD_WR_DATA(0x96);

	LCD_WR_REG(0xB4);
	LCD_WR_DATA(0x01);

	LCD_WR_REG(0xB7);
	LCD_WR_DATA(0xC6);

	// LCD_WR_REG(0xB9);
	// LCD_WR_DATA(0x02);
	// LCD_WR_DATA(0xE0);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA(0x80);
	LCD_WR_DATA(0x45);

	LCD_WR_REG(0xC1);
	LCD_WR_DATA(0x13); // 18  //00

	LCD_WR_REG(0xC2);
	LCD_WR_DATA(0xA7);

	LCD_WR_REG(0xC5);
	LCD_WR_DATA(0x0A);

	LCD_WR_REG(0xE8);
	LCD_WR_DATA(0x40);
	LCD_WR_DATA(0x8A);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x29);
	LCD_WR_DATA(0x19);
	LCD_WR_DATA(0xA5);
	LCD_WR_DATA(0x33);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0xD0);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x33);
	LCD_WR_DATA(0x30);
	LCD_WR_DATA(0x33);
	LCD_WR_DATA(0x47);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x2B);
	LCD_WR_DATA(0x31);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0xD0);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x0B);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x2F);
	LCD_WR_DATA(0x33);
	LCD_WR_DATA(0x47);
	LCD_WR_DATA(0x38);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x16);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x32);

	LCD_WR_REG(0xF0);
	LCD_WR_DATA(0x3C);

	LCD_WR_REG(0xF0);
	LCD_WR_DATA(0x69);

	HAL_Delay(120);

	LCD_WR_REG(0x21);

	LCD_WR_REG(0x29);

	LCD_direction(USE_HORIZONTAL);
	LCD_Clear(D_WHITE);
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
void LCD_SetWindows(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd)
{
	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA(xStar >> 8);
	LCD_WR_DATA(0x00FF & xStar);
	LCD_WR_DATA(xEnd >> 8);
	LCD_WR_DATA(0x00FF & xEnd);

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA(yStar >> 8);
	LCD_WR_DATA(0x00FF & yStar);
	LCD_WR_DATA(yEnd >> 8);
	LCD_WR_DATA(0x00FF & yEnd);

	LCD_WriteRAM_Prepare();
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
	LCD_SetWindows(Xpos, Ypos, Xpos, Ypos);
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
	lcddev.setxcmd = 0x2A;
	lcddev.setycmd = 0x2B;
	lcddev.wramcmd = 0x2C;
	lcddev.rramcmd = 0x2E;
	lcddev.dir = direction % 4;
	switch (lcddev.dir)
	{
	case 0:
		lcddev.width = LCD_W;
		lcddev.height = LCD_H;
		LCD_WriteReg(0x36, (1 << 3) | (1 << 6));
		break;
	case 1:
		lcddev.width = LCD_H;
		lcddev.height = LCD_W;
		LCD_WriteReg(0x36, (1 << 3) | (1 << 5));
		break;
	case 2:
		lcddev.width = LCD_W;
		lcddev.height = LCD_H;
		LCD_WriteReg(0x36, (1 << 3) | (1 << 7));
		break;
	case 3:
		lcddev.width = LCD_H;
		lcddev.height = LCD_W;
		LCD_WriteReg(0x36, (1 << 3) | (1 << 7) | (1 << 6) | (1 << 5));
		break;
	default:
		break;
	}
}

u16 LCD_Read_ID(void)
{
	uint8_t tData;
	u8 i, val[3] = {0};
	LCD_WR_REG(0xF0); // Command Set Control
	LCD_WR_DATA(0xC3);

	LCD_WR_REG(0xF0);
	LCD_WR_DATA(0x96);
	LCD_CS_CLR;
	for (i = 1; i < 4; i++)
	{
		LCD_RS_CLR;
		// SPI2_ReadWriteByte(0xFB);
		tData = 0xFB;
		HAL_SPI_Transmit(&hspi3, &tData, 1, HAL_MAX_DELAY);
		LCD_RS_SET;
		// SPI2_ReadWriteByte(0x10 + i);
		tData = 0x10 + i;
		HAL_SPI_Transmit(&hspi3, &tData, 1, HAL_MAX_DELAY);
		LCD_RS_CLR;
		// SPI2_ReadWriteByte(0xD3);
		tData = 0xD3;
		HAL_SPI_Transmit(&hspi3, &tData, 1, HAL_MAX_DELAY);
		SPI_SetSpeed(SPI_BAUDRATEPRESCALER_32);
		LCD_RS_SET;
		// val[i - 1] = SPI2_ReadWriteByte(0xFF);
		tData = 0xFF;
		HAL_SPI_TransmitReceive(&hspi3, &tData, &(val[i-1]), 1, HAL_MAX_DELAY);
		LCD_RS_CLR;
		// SPI2_ReadWriteByte(0xFB);
		tData = 0xFB;
		HAL_SPI_Transmit(&hspi3, &tData, 1, HAL_MAX_DELAY);
		LCD_RS_SET;
		// SPI2_ReadWriteByte(0x00);
		tData = 0x00;
		HAL_SPI_Transmit(&hspi3, &tData, 1, HAL_MAX_DELAY);
	}
	SPI_SetSpeed(SPI_BAUDRATEPRESCALER_8);
	LCD_CS_SET;
	LCD_WR_REG(0xF0); // Command Set Control
	LCD_WR_DATA(0x3C);
	LCD_WR_REG(0xF0);
	LCD_WR_DATA(0x69);
	lcddev.id = val[1];	// Why did we read 4 bytes and only use 2?
	lcddev.id <<= 8;
	lcddev.id |= val[2];
	return lcddev.id;
}
