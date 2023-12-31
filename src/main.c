/**
 * Initial beginnings of an ADSB receiver for STM32
 * 
 * NB The WeAct H273 board seems to have issues with startup - why?
 *    Looks like the 3v3 from the stlink doesn't have quite enough oomph, use usb power as well
*/

#define WAVESHARE

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
// #include <stdlib.h> // malloc() free()
#include "usb_device.h"
#include "usbd_cdc.h"
#include <stdbool.h>
#include "decoder.h"
#include "contactManager.h"
#include "localConfig.h"
#include "r820t2.h"
#include "main.h"
#include "lcd.h"      // For small LCD panel
// #include "ugui.h"
// #include "epaper.h"
#include "EPD_2in9_V2.h"  // Waveshare library for ePaper display
#include "GUI_Paint.h"    // UI library for the ePaper display
// #include "ImageData.h" // for the ePaper test code
// #include "Debug.h"        // for the ePaper library

#include "perfUtil.h"


// only valid if width is a multiple of 8
#if EPD_2IN9_V2_WIDTH % 8 != 0
#error "width not multiple of 8"
#endif
__attribute__((section(".fast_data")))
uint8_t image_bw[EPD_2IN9_V2_WIDTH / 8 * EPD_2IN9_V2_HEIGHT];


// #define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4096)   /* number of entries of array aADCxConvertedData[] */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  16384)   /* number of entries of array aADCxConvertedData[] */

// TODO - is it possible to hold samples as packed 8 bit values?
/* Variable containing ADC conversions data */
ALIGN_32BYTES (static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);


// The bitstream array holds one message worth of 1Mhz bits converted from the raw samples.
// A message contains 112 bits, adding a little to avoid risk of overrun
#define BITSTREAM_BUFFER_ENTRIES   (120)  
uint8_t bitstream[BITSTREAM_BUFFER_ENTRIES];


extern USBD_HandleTypeDef hUsbDeviceHS;

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;


TIM_HandleTypeDef htim7;  // not used yet

osThreadId_t demodTaskHandle;
osThreadId_t epaperTID;
osThreadId_t statusTID;

const osThreadAttr_t demodTask_attributes = {
  .name = "demodTask",
  .stack_size = 8192, // XXX check regularly
  .priority = (osPriority_t) osPriorityRealtime1,
};

const osThreadAttr_t statusThread_attributes = {
  .name = "status",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t epaperThread_attributes = {
  .name = "epaper",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

void SystemClock_Config(void);
// static void MPU_Initialize(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

void demodTask(void *argument);
void statusTask(void *argument);
void epaperTask(void *argument);
void Error_Handler(void);

void LED_Init();

uint16_t globalAvg; // For debugging the DC average

// Stats
uint32_t conversionsCompleted = 0;
uint32_t missedBuffers = 0;
uint32_t usSpentByDemod = 0;
uint32_t demodPercent = 0;

// Flags used by the ISR handlers to indicate which half of the buffer is ready
bool lowBuf = false;
bool hiBuf = false;

int main(void)
{
  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_InvalidateDCache(); // Check if this is still needed after increasing flash wait states
  SCB_EnableDCache();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Configure the system clock */
  SystemClock_Config();

  // Enable the debug counter
  initPerfUtil();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  // HAL_Delay(2000); // allow the usb serial to connect for debug

  /* Init scheduler */
  osKernelInitialize();

  // Create the thread(s)
  // If there isn't enough heap osThreadNew fails and returns NULL,
  // More heap can be added in FreeRTOSConfig.h.
  demodTaskHandle = osThreadNew(demodTask, NULL, &demodTask_attributes);

  // epaper thread
  epaperTID = osThreadNew(epaperTask, NULL, &epaperThread_attributes);

  // status thread
  statusTID = osThreadNew(statusTask, NULL, &statusThread_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */

  while (1) {}
}

void epd_io_init(void)
{

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin default states */
  HAL_GPIO_WritePin(EPD_RESET_GPIO_Port, EPD_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EPD_DC_GPIO_Port, EPD_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_SET);
  

  /*Configure ePaper GPIO output pins (not spi peripheral pins): DC_Pin RESET_Pin CS pin */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  GPIO_InitStruct.Pin = EPD_RESET_Pin;
  HAL_GPIO_Init(EPD_RESET_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = EPD_DC_Pin;
  HAL_GPIO_Init(EPD_DC_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = EPD_CS_Pin;
  HAL_GPIO_Init(EPD_CS_GPIO_Port, &GPIO_InitStruct);

  /* configure the epaper module busy pin */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = EPD_BUSY_Pin;
  HAL_GPIO_Init(EPD_BUSY_GPIO_Port, &GPIO_InitStruct);
  
}

/**
  * @brief System Clock Configuration
  * 
  * Clock config for H723 at 550MHz
  * 
  * NB Clock for ADC is in stm32h7xx_hal_msp.c
  * 
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  // __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  // __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 110;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) // _3 seems to make startup unreliable
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /**
   * 8 bits with 1.5cycle sampling time requires 6 clocks
   * We want 4 MHz rate (initially), so we need 24MHz ADC clock (after divisor)
  */


  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;  // any chance of getting some oversampling? Maybe with interleaved mode?
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }


  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; // XXX is this enough, can we get more (with higher adc clock?)
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    Error_Handler();
  }


}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4096;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  LED_Init();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_DC_Pin|LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


}

/**
 * Calculate the average value of the input array
*/
uint16_t calculateAverage(uint16_t *input, int len)
{
  uint32_t sum = 0;

  for(int i=0; i<len; i++) {
    sum += input[i];
  }

  return (uint16_t)(sum / len);
}

/**
 * Get the highest value in the samples
*/
uint16_t findMaxValue(uint16_t *input, int len)
{
  uint16_t max = 0;

  for(int i=0; i<len; i++) {
    if(input[i] > max) {
      max = input[i];
    }
  }

  return max;
}


// OLD code, remove once confident in the new code

/**
 * Convert from 2MHz amplitude array to bitstream
 * 
 * Pairs of input samples are compared, a falling pair codes 1, rising codes 0
 * 
 * The output array must be large enough to hold len/2 values
 * 
 * @param input array of 2MHz samples
 * @param len number of entries in the input array
 * @param output array to write the bitstream to
*/
// void ampToBitStream(uint16_t *input, int len, uint8_t *output)
// {
//   const int nOutBits = len/2;
//   for(int i=0; i<nOutBits; i++) {
//     if(input[2*i] > input[2*i+1]) {
//       output[i] = 1;
//     } else {
//       output[i] = 0;
//     }
//   }
// }

// OLD code, remove once confident in the new code

/**
 * Search a bitstream for the ADSB preamble pattern
 * The preamble is 11??00??. The ? bits are indeterminate
 * as they are not valid pulse encodings and will be randomly
 * converted by ampToBitStream
*/
// int findPreamble(uint8_t *input, int len, int start)
// {
//   for(int i=start; i<len; i++) {
//     // 11??00??
//     if (input[i] == 1 && input[i+1] == 1 && input[i+4] == 0 && input[i+5] == 0) {
//       return i;
//     }
//   }
//   return -1;
// }


void printArray(uint16_t *input, int len)
{
  // buffer to hold 16 values, max 3 chars per value plus a space between each is 64 chars, leave some room for cowardice
  char buffer[80];

  int outputLen = 0;
  for(int i=0; i<len; i++) {
    const int ret = sprintf(&(buffer[outputLen]), "%d ", input[i]);
    if (ret > 0) {
      outputLen += ret;
    } else {
      // something went wrong, abort
      printf("ERROR sprintf returned %d\n", ret);
      return;
    }
    if ((i+1) % 16 == 0) {
      buffer[outputLen]=0;
      printf("%s", buffer);
      outputLen = 0;
      osDelay(1);
    }
  }

  buffer[outputLen]=0;
  printf("%s\n", buffer);

}

/**
 * Apply a simple windowing filter to the data.
 * 
 * TODO optimise
*/
void filterSamples(uint16_t *input, int len)
{
  for(int i=0; i<len-3; i++) {
    input[i] = input[i] + input[i+1] + input[i+2] + input[i+3];
  }
}


/**
 * Search for preamble patterns in the filtered data
 * 
 * The input data is pre-decimation, so at 8Msps. Slide a window along
 * the data comparing value[n] with value[n+4] to get bit values for the 4 fixed bits of the preamble.
 * 
 * The preamble contains 8 slots coding 11?00??? where the ? should have 1us of low signal. So the offsets 
 * in ticks are
 * 0  hi
 * 4  low
 * 8  hi
 * 12 low
 * 16 low
 * 20 low 
 * 24 low
 * 28 hi
 * 32 low
 * 36 high
 * 40 low
 * 44 low
 * 48 low
 * 52 low
 * 56 low
 * 60 low
 * 
 * @param input the array of filtered samples
 * @param len the length of the input array
 * @param start the index in the input array to start searching from
 * @return the index of the preamble in the input array, or -1 if not found
 * 
 * TODO generalise for different sample rates?
 * TODO set a threshold for strength to accept the match, or rely on CRC checking later?
*/
int findPreambleInFilteredData(uint16_t *input, int len, int start)
{
  uint32_t strength = 0;
  int bestIndex = -1;
  bool found = false;
  for(int i=start; i<len-60; i++) {
    bool thisIndexGood = false;
    if ((input[i] > input[i+4]) && (input[i+8] > input[i+12]) &&
      (input[i+24] < input[i+28]) && (input[i+32] < input[i+36])) {

      // compare the 'unset' periods are at a low level
      const uint32_t highLevel = (input[i] + input[i+8] + input[i+28] + input[i+36])/4;
      const uint32_t lowLevel = (input[i+4] + input[i+12] + input[i+24] + input[i+32])/4;
      const uint32_t threshold = (highLevel + lowLevel)/2;

      // Check that all the 'unset' periods are at a low level
      if ((input[i+16] < threshold) && (input[i+20] < threshold) && (input[i+40] < threshold) && (input[i+44] < threshold) && 
        (input[i+48] < threshold) && (input[i+52] < threshold) && (input[i+56] < threshold) && (input[i+60] < threshold))  {
        thisIndexGood = true;
      }

      // Test an average of the unset periods
      // const uint32_t sumOfZeroPeriods = input[i+16] + input[i+20] + input[i+40] + input[i+44] +
      //                                   input[i+48] + input[i+52] + input[i+56] + input[i+60];
      // const uint32_t zeroPeriodsAvg = sumOfZeroPeriods/8;
      // thisIndexGood = (zeroPeriodsAvg < threshold);


      if (thisIndexGood) {
        // Estimate the quality of the match
        uint32_t s = highLevel - lowLevel;  // Basic, but it's a start
        if (s > strength) {
          // printf("Strength %ld better than %ld\n", s, strength);
          bestIndex = i;
          strength = s;
        } else {
          // strength on this index is worse than the previous, so we're done for this search
          found = true;
          break;
        }
      } else if (bestIndex != -1) {
        // If we've previously found a match, we're done for this search
        found = true;
        break;
      }

    }
  }

  if (found) {
    // printf("Found preamble at index %d with strength %ld\n", bestIndex, strength);
    return bestIndex;
  } else {
    return -1;
  }
}   

/**
 * Convert filtered amplitude data to bitstream
 * 
 * The start index is obtained from preamble detection. It points to the start of
 * the data and defines the phase offset.
 * 
 * To demodulate the samples we will need to step through the data comparing sample n with
 * sample n+4. A falling value decodes as 1 while a rising value decodes as 0.
 * 
 * TODO There may be a benefit to combining sample pairs, either n+1 or n-1 to correct for sub-sample phase offset.
 * 
 * @param input the array of filtered samples
 * @param len the length of the input array
 * @param start the index in the input array to start converting from
 * @return the number of bits converted or -1 if there is an error
*/
int filteredAmplitudeToBitStream(uint16_t *input, int len, int start)
{
  // There are at most 112 bits needed
  for(int i=0; i<112; i++) {
    const int inputIndex = i*8;
    if (start+inputIndex+4 >= len) return i;
    if (input[start+inputIndex] > input[start+inputIndex+4]) {
      bitstream[i] = 1;
    } else {
      bitstream[i] = 0;
    }
  }

  return 112;
}

const uint32_t PlotRadius = 115;
const UG_U16 BackgroundColour = C_DIM_GRAY;

void drawBackground()
{
  // LCD_Fill(0, 0, 239, 239, BackgroundColour);
  UG_FillScreen(BackgroundColour);

  UG_DrawCircle(120, 120, PlotRadius, C_LIGHT_GRAY);

  // Center marker
  const uint32_t CenterMarkerSize = 2;
  UG_DrawTriangle(120-CenterMarkerSize, 120+CenterMarkerSize, 120, 120-CenterMarkerSize, 120+CenterMarkerSize, 120+CenterMarkerSize, C_WHITE);

}

int EPD_test(void)
{
  const uint16_t ROTATION = 270;

  printf("EPD_2IN9_V2_test Demo\r\n");
  // if (DEV_Module_Init() != 0)
  // {
  //   return -1;
  // }

  printf("e-Paper Init and Clear...\r\n");
  EPD_2IN9_V2_Init();
  EPD_2IN9_V2_Clear();
  DEV_Delay_ms(1000);

  // Create a new image cache
  UBYTE *BlackImage;
  UWORD Imagesize = ((EPD_2IN9_V2_WIDTH % 8 == 0) ? (EPD_2IN9_V2_WIDTH / 8) : (EPD_2IN9_V2_WIDTH / 8 + 1)) * EPD_2IN9_V2_HEIGHT;
  if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    printf("Failed to apply for black memory...\r\n");
    return -1;
  }
  printf("Paint_NewImage\r\n");
  Paint_NewImage(BlackImage, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, ROTATION, WHITE);
  Paint_Clear(WHITE);

#if 0 // show image for array
  Paint_NewImage(BlackImage, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, ROTATION, WHITE);
  printf("show image for array\r\n");
  Paint_SelectImage(BlackImage);
  Paint_Clear(WHITE);
  Paint_DrawBitMap(gImage_2in9);

  EPD_2IN9_V2_Display(BlackImage);
  DEV_Delay_ms(3000);
#endif

#if 1 // Drawing on the image
  EPD_2IN9_V2_Init_Fast();
  Paint_NewImage(BlackImage, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, ROTATION, WHITE);
  printf("Drawing\r\n");
  // 1.Select Image
  Paint_SelectImage(BlackImage);
  Paint_Clear(WHITE);

  // 2.Drawing on the image
  printf("Drawing:BlackImage\r\n");
  Paint_DrawPoint(10, 80, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);

  Paint_DrawLine(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(70, 70, 20, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  Paint_DrawRectangle(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawRectangle(80, 70, 130, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  Paint_DrawCircle(45, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawCircle(105, 95, 20, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  Paint_DrawLine(85, 95, 125, 95, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawLine(105, 75, 105, 115, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

  Paint_DrawString_EN(10, 0, "waveshare", &Font16, BLACK, WHITE);
  Paint_DrawString_EN(10, 20, "hello world", &Font12, WHITE, BLACK);

  Paint_DrawNum(10, 33, 123456789, &Font12, BLACK, WHITE);
  Paint_DrawNum(10, 50, 987654321, &Font16, WHITE, BLACK);

  // Paint_DrawString_CN(130, 0, "���abc", &Font12CN, BLACK, WHITE);
  // Paint_DrawString_CN(130, 20, "΢ѩ����", &Font24CN, WHITE, BLACK);

  EPD_2IN9_V2_Display_Base(BlackImage);
  DEV_Delay_ms(3000);
#endif

#if 1 // Partial refresh, example shows time
  Paint_NewImage(BlackImage, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, ROTATION, WHITE);
  printf("Partial refresh\r\n");
  Paint_SelectImage(BlackImage);

  PAINT_TIME sPaint_time;
  sPaint_time.Hour = 12;
  sPaint_time.Min = 34;
  sPaint_time.Sec = 56;
  UBYTE num = 10;
  for (;;)
  {
    sPaint_time.Sec = sPaint_time.Sec + 1;
    if (sPaint_time.Sec == 60)
    {
      sPaint_time.Min = sPaint_time.Min + 1;
      sPaint_time.Sec = 0;
      if (sPaint_time.Min == 60)
      {
        sPaint_time.Hour = sPaint_time.Hour + 1;
        sPaint_time.Min = 0;
        if (sPaint_time.Hour == 24)
        {
          sPaint_time.Hour = 0;
          sPaint_time.Min = 0;
          sPaint_time.Sec = 0;
        }
      }
    }
    Paint_ClearWindows(150, 80, 150 + Font20.Width * 7, 80 + Font20.Height, WHITE);
    Paint_DrawTime(150, 80, &sPaint_time, &Font20, WHITE, BLACK);

    num = num - 1;
    if (num == 0)
    {
      break;
    }
    EPD_2IN9_V2_Display_Partial(BlackImage);

    // Does this still work if you suspend the display between updates? Yes
    EPD_2IN9_V2_Sleep();

    DEV_Delay_ms(500); // Analog clock 1s
  }
#endif

#if 1 // show image for array
  free(BlackImage);
  printf("show Gray------------------------\r\n");
  Imagesize = ((EPD_2IN9_V2_WIDTH % 4 == 0) ? (EPD_2IN9_V2_WIDTH / 4) : (EPD_2IN9_V2_WIDTH / 4 + 1)) * EPD_2IN9_V2_HEIGHT;
  if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    printf("Failed to apply for black memory...\r\n");
    return -1;
  }
  EPD_2IN9_V2_Gray4_Init();
  printf("4 grayscale display\r\n");
  Paint_NewImage(BlackImage, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, ROTATION, WHITE);
  Paint_SetScale(4);
  Paint_Clear(0xff);

  Paint_DrawPoint(10, 80, GRAY4, DOT_PIXEL_1X1, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 90, GRAY4, DOT_PIXEL_2X2, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 100, GRAY4, DOT_PIXEL_3X3, DOT_STYLE_DFT);
  Paint_DrawLine(20, 70, 70, 120, GRAY4, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(70, 70, 20, 120, GRAY4, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawRectangle(20, 70, 70, 120, GRAY4, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawRectangle(80, 70, 130, 120, GRAY4, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(45, 95, 20, GRAY4, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawCircle(105, 95, 20, GRAY2, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawLine(85, 95, 125, 95, GRAY4, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawLine(105, 75, 105, 115, GRAY4, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawString_EN(10, 0, "waveshare", &Font16, GRAY4, GRAY1);
  Paint_DrawString_EN(10, 20, "hello world", &Font12, GRAY3, GRAY1);
  Paint_DrawNum(10, 33, 123456789, &Font12, GRAY4, GRAY2);
  Paint_DrawNum(10, 50, 987654321, &Font16, GRAY1, GRAY4);
  // Paint_DrawString_CN(150, 0, "���abc", &Font12CN, GRAY4, GRAY1);
  // Paint_DrawString_CN(150, 20, "���abc", &Font12CN, GRAY3, GRAY2);
  // Paint_DrawString_CN(150, 40, "���abc", &Font12CN, GRAY2, GRAY3);
  // Paint_DrawString_CN(150, 60, "���abc", &Font12CN, GRAY1, GRAY4);
  // Paint_DrawString_CN(150, 80, "΢ѩ����", &Font24CN, GRAY1, GRAY4);
  EPD_2IN9_V2_4GrayDisplay(BlackImage);
  DEV_Delay_ms(3000);

  // Paint_NewImage(BlackImage, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, 0, WHITE);
  // Paint_SetScale(4);
  // Paint_Clear(WHITE);
  // Paint_DrawBitMap(gImage_2in9_4Gray);
  // EPD_2IN9_V2_4GrayDisplay(BlackImage);
  // DEV_Delay_ms(3000);

#endif

  printf("Clear...\r\n");
  EPD_2IN9_V2_Init();
  EPD_2IN9_V2_Clear();

  printf("Goto Sleep...\r\n");
  EPD_2IN9_V2_Sleep();
  free(BlackImage);
  BlackImage = NULL;
  DEV_Delay_ms(2000); // important, at least 2s
  // close 5V
  // printf("close 5V, Module enters 0 power consumption ...\r\n");
  // DEV_Module_Exit();
  return 0;
}

/**
 * Task dedicated to maintaining the epaper display with it's very slow update rate
*/
void epaperTask(void *argument)
{
  const uint16_t EPD_ROTATION = 270;

  // uint32_t cs, ce;

  // static uint32_t tLast = 0;
  char buf[40]; // for generating strings for the displays
  sFONT * contactListFont = &Font12;

  epd_io_init();

  // Splash screen
  Paint_NewImage(image_bw, EPD_2IN9_V2_WIDTH, EPD_2IN9_V2_HEIGHT, EPD_ROTATION, WHITE);
  Paint_Clear(WHITE);

  {
    char * str = "ADSB ALERT";

    int strLenP = strlen(str) * Font24.Width;

    int x = (EPD_2IN9_V2_HEIGHT - strLenP) / 2;  // X = Height, Y = Width
    int y = (EPD_2IN9_V2_WIDTH / 2) - 24;

    Paint_DrawString_EN(x, y, str, &Font24, BLACK, WHITE);
  }
  {
    char * str = "v0.0.0";

    int strLenP = strlen(str) * Font12.Width;

    int x = (EPD_2IN9_V2_HEIGHT - strLenP) / 2;
    int y = (EPD_2IN9_V2_WIDTH / 2) - 6 + 24;

    Paint_DrawString_EN(x, y, str, &Font12, BLACK, WHITE);
  }

  EPD_2IN9_V2_Init();
  EPD_2IN9_V2_Display_Base(image_bw);
  EPD_2IN9_V2_Sleep();

  vTaskDelay(2000);

  // Clear the splash screen to minimise ghosting
  EPD_2IN9_V2_Init();
  EPD_2IN9_V2_Clear();
  EPD_2IN9_V2_Sleep();

  TickType_t lastWakeTime = xTaskGetTickCount();

  // Round to nearest 5 seconds
  lastWakeTime = lastWakeTime - (lastWakeTime % 5000);

  while(true)
  {
    vTaskDelayUntil(&lastWakeTime, 5000);

    Paint_Clear(WHITE);

    // Display uptime
    const uint32_t tSeconds = lastWakeTime / 1000;

    sprintf(buf, "uptime: %lu:%02lu:%02lu", tSeconds / 3600, (tSeconds / 60) % 60, tSeconds % 60);
    Paint_DrawString_EN(EPD_2IN9_V2_HEIGHT - (Font12.Width * strlen(buf)), 128-Font12.Height, buf, &Font12, BLACK, WHITE);

    // Display the time used for decoding the RF
    sprintf(buf, "demod: %2lu%%", demodPercent);
    Paint_DrawString_EN(0, 128-Font12.Height, buf, &Font12, BLACK, WHITE);

    const int nContacts = getNumContacts();

    if (nContacts > 0) {
      // float minRange = 100;
      // int32_t closestIndex = -1;

      Paint_DrawString_EN(0, 0, " Range Brng  Spd Trk Alt   Msgs Age", contactListFont, BLACK, WHITE);

      int epdLine = 1;
      for(int i=0; i<nContacts; i++) {
        aircraft_t *aircraft = getContact(i);
        if (aircraft) { // in case the number of entries in the list changes since we read it

          // By comparing the track and the bearing we can tell which aircraft are closing
          // We need to have lat/lon and track/speed info to do this
          bool closing = false;
          if (aircraft->speed != 0 && aircraft->lat != 0) {
            const int bearing = aircraft->bearing;
            const int track = aircraft->track;
            int trackDiff = abs(track - bearing);
            // Allow for the discontinuity 359 to 0
            if (trackDiff > 180) {
              trackDiff = 360 - trackDiff;
            }
            if (trackDiff < 90) {
              closing = true;
            }
          }

          const uint32_t tLocal = aircraft->timestamp; // privatise to avoid race condition
          const uint32_t tNow = HAL_GetTick();
          uint32_t tSince = (tNow - tLocal)/1000;
          float rangeNM = aircraft->range / 1.852;

          // Generate a line for the EPD display
          if (epdLine < 10 && rangeNM < 100) {
            sprintf(buf, "%6.1f%6.1f%4d%4d%6u%4lu%4lu", rangeNM, 
                    aircraft->bearing+180, aircraft->speed, aircraft->track, aircraft->modeC, 
                    aircraft->messages, tSince);
            uint16_t foreground = BLACK, background = WHITE;
            if (closing) {
              foreground = WHITE;
              background = BLACK;
            }
            Paint_DrawString_EN(0, epdLine * contactListFont->Height, buf, contactListFont, foreground, background);
            epdLine++;
            if (epdLine >= 10) break; // no need to process more aircraft if we're out of display lines
          }

          // if (closing && rangeNM < minRange) {
          //   minRange = rangeNM;
          //   closestIndex = i;
          // }

          // Radar plot - old code for the LCD, might be useful later
          // if (rangeNM < 50) {
          //   // scale range to the size of the circle for a max 30NM display
          //   uint32_t radius = rangeNM * PlotRadius / 30;
          //   // and limit so that distant contacts are drawn at the edge
          //   if (radius > PlotRadius) radius = PlotRadius;
          //   uint16_t cx = 120 + (int)(radius * sin((aircraft->bearing+180)*M_PI/180));
          //   uint16_t cy = 120 - (int)(radius * cos((aircraft->bearing+180)*M_PI/180)); // minus since 0 is top of display
          //   // printf("drawing at %u %u\n", cx, cy);
          //   UG_COLOR contactColour = C_GREEN;
          //   if (closing) contactColour = C_RED;
          //   // UG_DrawPixel(cx, cy, contactColour);
          //   UG_FillFrame(cx-1, cy-1, cx+1, cy+1, contactColour);
          // } // if (rangeNM < 50)

        } // if (aircraft)
      } // for (each contact)

    }

    // Update the epaper display
    // cs = getCycles();
    // Takes about 1149933 us
    EPD_2IN9_V2_Display_Partial(image_bw);
    // ce = getCycles();
    // printf("display partial %lu\n", getDeltaUs(cs, ce));

    EPD_2IN9_V2_Sleep();

    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if (stackHighWaterMark == 0) {
      printf("ERROR: stack overflow on epaper task!!\n");
    }

  }
}

/**
 * Runs periodically to write debug to stdout and update the LCD
 * 
 *
*/
void statusTask(void *argument)
{
  // uint32_t cs, ce;   // For perf measurements
  static uint32_t tLast = 0;
  char buf[40]; // for generating strings for the displays

  LCD_init();

  LCD_Fill(0, 0, 239, 239, C_DARK_BLUE);
  LCD_PutStr(10, 80, "ADSB ALERT", FONT_24X40, C_WHITE, C_DARK_BLUE);
  LCD_PutStr(20, 120, "V0.0 (alpha)", FONT_16X26, C_WHITE, C_DARK_BLUE);
  UG_Update();

  vTaskDelay(1000);

  // ST7789_Test();

  drawBackground();
  UG_Update();

  while(true)
  {
    vTaskDelay(2000);

    #ifndef BEAST_OUTPUT

    const uint32_t now = HAL_GetTick();
    uint32_t diff = now - tLast;
    uint64_t sps = (uint64_t)ADC_CONVERTED_DATA_BUFFER_SIZE*1000 * conversionsCompleted / diff;
    printf("Time taken: %lu, buffers:%lu sps=%lu", diff, conversionsCompleted, (uint32_t)sps);
    tLast = now;
    conversionsCompleted = 0;

    if (missedBuffers) {
      printf(" %ld missed buffers\n", missedBuffers);
      missedBuffers = 0;
    } else {
      printf("\n");
    }

    demodPercent = usSpentByDemod / (diff * 10);

    printf("us demod %lu (%lu%%)\n", usSpentByDemod, demodPercent);
    usSpentByDemod = 0;

    // printf("average %u\n", globalAvg);

    drawBackground();

    const int nContacts = getNumContacts();

    UG_FontSelect(FONT_7X12);
    UG_SetForecolor(C_WHITE);
    UG_SetBackcolor(BackgroundColour);

    sprintf(buf, "%d ", nContacts);
    UG_PutString(0, 0, buf);

    if (nContacts > 0) {
      int nApproaching = 0;
      float minRange = 100;
      int32_t closestIndex = -1;
      printf("\n%d Contacts:\n", nContacts);
      printf("Index\t");
      #ifdef SHOW_ADDR
      printf("Addr\t");
      #endif
      printf("Range\tBearing\tSpeed\ttrack\tBaroAlt\tGpsAlt\tMsgs\tAge\n");
      // Paint_DrawString_EN(0, 0, " Range Brng  Spd Trk Alt   Msgs Age", contactListFont, BLACK, WHITE);

      int oldContactIndex = -1;
      for(int i=0; i<nContacts; i++) {
        aircraft_t *aircraft = getContact(i);
        if (aircraft) { // in case the number of entries in the list changes since we read it
          if (aircraft->timestamp + 120000 < now) {
            oldContactIndex = i;
            continue; // SKIP processing old contact
          }
          // By comparing the track and the bearing we can tell which aircraft are closing
          // We need to have lat/lon and track/speed info to do this
          bool closing = false;
          if (aircraft->speed != 0 && aircraft->lat != 0) {
            const int bearing = aircraft->bearing;
            // if (bearing == 0) {
            //   printf("lat %f lon %f time %lu\n", aircraft->lat, aircraft->lon, aircraft->timestampLatLon);
            // }
            const int track = aircraft->track;
            int trackDiff = abs(track - bearing);
            // Allow for the discontinuity 359 to 0
            if (trackDiff > 180) {
              trackDiff = 360 - trackDiff;
            }
            // printf("bearing %d, track %d, diff %d\n", bearing, track, trackDiff);
            if (trackDiff < 90) {
              closing = true;
              nApproaching++;
            }
          }

          const uint32_t tLocal = aircraft->timestamp; // privatise to avoid race condition
          const uint32_t tNow = HAL_GetTick();
          uint32_t tSince = (tNow - tLocal)/1000;
          printf("%2d: %c ", i, closing ? 'C' : ' ');
          #ifdef SHOW_ADDR
          printf("%6lx\t", aircraft->addr);
          #endif
          float rangeNM = aircraft->range / 1.852;
          printf("%2.2f\t%3.1f\t%4d\t%4d\t%6u\t%6u\t%3lu\t%2lu\n", rangeNM, 
                  aircraft->bearing+180, aircraft->speed, aircraft->track, aircraft->modeC, aircraft->altitude, aircraft->messages, tSince);


          if (closing && rangeNM < minRange) {
            minRange = rangeNM;
            closestIndex = i;
          }

          if (rangeNM < 50) {
            // scale range to the size of the circle for a max 30NM display
            uint32_t radius = rangeNM * PlotRadius / 30;
            // and limit so that distant contacts are drawn at the edge
            if (radius > PlotRadius) radius = PlotRadius;
            uint16_t cx = 120 + (int)(radius * sin((aircraft->bearing+180)*M_PI/180));
            uint16_t cy = 120 - (int)(radius * cos((aircraft->bearing+180)*M_PI/180)); // minus since 0 is top of display
            // printf("drawing at %u %u\n", cx, cy);
            UG_COLOR contactColour = C_GREEN;
            if (closing) contactColour = C_RED;
            // UG_DrawPixel(cx, cy, contactColour);
            UG_FillFrame(cx-1, cy-1, cx+1, cy+1, contactColour);
          } // if (rangeNM < 50)
        } // if (aircraft)
      } // for (each contact)
      // printf("furthest: %d, oldest: %d\n", findMostDistantContact(), findOldestContact());

      if (oldContactIndex != -1) {
        removeAircraftByIndex(oldContactIndex);
      }

      // Show the number of approaching aircraft
      sprintf(buf, "%d ", nApproaching);
      UG_PutString(0, 20, buf);

      // Show the closest aircraft info in the top right corner
      if (closestIndex != -1) {
        aircraft_t *aircraft = getContact(closestIndex);
        if (aircraft) {
          sprintf(buf, "%d", aircraft->modeC);
          size_t len = strlen(buf);
          UG_PutString(239-(len * 7), 0, buf);

          sprintf(buf, "%2.1f", minRange);
          len = strlen(buf);
          UG_PutString(239-(len * 7), 20, buf);

          sprintf(buf, "%d", aircraft->speed);
          len = strlen(buf);
          UG_PutString(239-(len * 7), 40, buf);
        }
      } else {
        // not needed if doing a redraw each time
        // TODO replace with fill calls
        // UG_PutString(239-(5*7),  0, "     ");
        // UG_PutString(239-(4*7), 20, "    ");
        // UG_PutString(239-(3*7), 40, "   ");
      }
    } // if (nContacts > 0)

    // Update the LCD
    UG_Update();

    #endif // BEAST_OUTPUT

    // Check for stack overflow on this task...
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if (stackHighWaterMark == 0) {
      printf("ERROR: stack overflow on status task!!\n");
    }

    // ...and the demod task
    stackHighWaterMark = uxTaskGetStackHighWaterMark(demodTaskHandle);
    if (stackHighWaterMark == 0) {
      printf("ERROR: stack overflow on main task!!\n");
    }

  } // while(true)
}

void processBuffer(uint16_t *input, int len)
{
  #define BUFFERS_FOR_AVG 10
  // static uint16_t nAvgCounter = 0;
  // static uint16_t averages[BUFFERS_FOR_AVG];

  // TODO - we don't really want to be calculating an average on every buffer

  // Just use a fixed value? How much part to part variation is there?
  globalAvg = 120;

  // This is vulnerable to getting a bad estimate
  // if (nAvgCounter < BUFFERS_FOR_AVG) {
  //   // Takes about 53us
  //   uint16_t avg = calculateAverage(input, len);
  //   averages[nAvgCounter++] = avg;
  //   if (nAvgCounter == BUFFERS_FOR_AVG) {
  //     globalAvg = 0;
  //     for(int i=0; i<BUFFERS_FOR_AVG; i++) {
  //       globalAvg += averages[i];
  //     }
  //     globalAvg /= BUFFERS_FOR_AVG;
  //   }
  // }

  uint16_t max;
  // debugging - dump data when there's a strong signal
  // uint16_t max = findMaxValue(input, len);
  // bool dumpIt = (max > 180);
  // if (dumpIt) {
  //   // dump input[] to stdout
  //   printf("RAW:\n");
  //   printArray(input, len);
  // }

  max = 0;  // Get a new max after removing dc offset
  for(int i=0; i < len; i++) {
    int t = input[i];
    // DC offset and abs value
    const uint16_t amp = (t > globalAvg) ? (t - globalAvg) : globalAvg - t;
    input[i] = amp;
    if (amp > max) {
      max = amp;
    }
  }

  // if (dumpIt) {
  //   printf("ABS:\n");
  //   printArray(input, len);
  // }

  if (max > 10) {
    filterSamples(input, len);
    // printf("FILTERED:\n");
    // printArray(input, len);

    // Find the preamble in the filtered data
    int preambleIndex = 0;
    while(preambleIndex < len) {
      preambleIndex = findPreambleInFilteredData(input, len, preambleIndex);
      if (preambleIndex == -1) {
        break;
      } else {
        // printf("Found preamble at %d\n", preambleIndex);

        int nBitsConverted = filteredAmplitudeToBitStream(input, len, preambleIndex+64);

        bool success = false;
        if (nBitsConverted == 112) {
          success = decodeModeS(bitstream, 112);
        }
        if (success) {
          preambleIndex += (120 * 8); // move the pointer past the message
        } else {
          preambleIndex++;
        }
      }
    }

  // } else {

  }

}

/**
  * @brief  Task that receives raw ADC samples and processes them to extract ADSB messages
  * @param  argument: Not used
  * @retval None
  * 
  * Each time the ADC has filled the (half) buffer this task will get woken by a notify event from
  * the irq handler.
  */
void demodTask(void *argument)
{
  vTaskDelay(1000);

  // Configure the tuner
  R820T2_init();

  for(;;)
  {
    uint32_t msg; // not used yet
    xTaskNotifyWait(0, 0, &msg, portMAX_DELAY);

    uint32_t cyclesStart = getCycles();

    if (lowBuf)
    {
      processBuffer(aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE/2);
      lowBuf = false;
    }

    if (hiBuf)
    {
      processBuffer(&(aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE/2]), ADC_CONVERTED_DATA_BUFFER_SIZE/2);
      hiBuf = false;
    }

    uint32_t cyclesEnd = getCycles();
    usSpentByDemod += getDeltaUs(cyclesStart, cyclesEnd);
  }

}

/* MPU Configuration 
*/
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * send printf to the usb cdc
 * 
 * TODO Implement a better buffering system to make this more efficient
*/
int _write(int fd, char *ptr, int len)
{
  uint32_t attempts = 0;

  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, (uint8_t *) ptr, len);

  // if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK)
  uint8_t ret = USBD_BUSY;
  while (ret == USBD_BUSY) {
    ret = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
    if (ret == USBD_OK) {
      break;
    }
    attempts++;
    if (attempts > 100) {
      return 0;
    }
    osDelay(1);
  }

  return len; 
}

/**
 * Read from the usb cdc
 * NOT IMPLEMENTED
*/
int _read(int fd, char *ptr, int len){
  //read from USB CDC
  return 0;
}

// int _close(int fd){
//   return -1;
// }

// int _isatty(int fd){
//   return 1;
// }


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void LED_Init()
{
  LED_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Conversion complete callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes
  // NB ADC_CONVERTED_DATA_BUFFER_SIZE is the array size where the values are 16 bit, so the total array size is *2, but we're
  // only invalidating 1/2 of it, so ADC_CONVERTED_DATA_BUFFER_SIZE * 2 / 2
  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[0], ADC_CONVERTED_DATA_BUFFER_SIZE);
  if (lowBuf) missedBuffers++;
  lowBuf = true;

  if (demodTaskHandle) {  // need to be careful as the task may not have been started when we first get going
    BaseType_t taskWoken;
    xTaskNotifyFromISR(demodTaskHandle , 0, eNoAction, &taskWoken);
    if (taskWoken == pdTRUE) {
      portYIELD_FROM_ISR(taskWoken);
    }
  }
  // printf("message sent, status %ld\n", status);
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE/2], ADC_CONVERTED_DATA_BUFFER_SIZE);
  conversionsCompleted++;
  if (hiBuf) missedBuffers++;
  hiBuf = true;

  if (demodTaskHandle) {  // need to be careful as the task may not have been started when we first get going
    BaseType_t taskWoken;
    xTaskNotifyFromISR(demodTaskHandle , 0, eNoAction, &taskWoken);
    if (taskWoken == pdTRUE) {
      portYIELD_FROM_ISR(taskWoken);
    }
  }
}
