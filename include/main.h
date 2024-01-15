/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define LED_PIN                                GPIO_PIN_3
#define LED_GPIO_PORT                          GPIOE
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOE_CLK_ENABLE()

#define EPD_MOSI_Pin GPIO_PIN_1
#define EPD_MOSI_GPIO_Port GPIOC
#define EPD_CS_Pin GPIO_PIN_10
#define EPD_CS_GPIO_Port GPIOE
#define EPD_DC_Pin GPIO_PIN_12
#define EPD_DC_GPIO_Port GPIOE
#define EPD_BUSY_Pin GPIO_PIN_14
#define EPD_BUSY_GPIO_Port GPIOE
#define EPD_SCK_Pin GPIO_PIN_10
#define EPD_SCK_GPIO_Port GPIOB
#define EPD_RESET_Pin GPIO_PIN_11
#define EPD_RESET_GPIO_Port GPIOB

#define LCD_DC_Pin GPIO_PIN_5
#define LCD_DC_GPIO_Port GPIOD
#define LCD_RESET_Pin GPIO_PIN_6
#define LCD_RESET_GPIO_Port GPIOD

// 3.5" touch display on spi3
#define LCD2_SCK_Pin GPIO_PIN_10
#define LCD2_SCK_GPIO_Port GPIOC
#define LCD2_MISO_Pin GPIO_PIN_11
#define LCD2_MISO_GPIO_Port GPIOC
#define LCD2_MOSI_Pin GPIO_PIN_12
#define LCD2_MOSI_GPIO_Port GPIOC
#define LCD2_RESET_Pin GPIO_PIN_0
#define LCD2_RESET_GPIO_Port GPIOD
#define LCD2_DC_Pin GPIO_PIN_1
#define LCD2_DC_GPIO_Port GPIOD
#define LCD2_CS_Pin GPIO_PIN_2
#define LCD2_CS_GPIO_Port GPIOD

// XXX Add the backlight control pin

// XXX Add the touch screen pins



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
