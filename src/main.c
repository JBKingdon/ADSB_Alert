/**
 * Initial beginnings of an ADSB receiver for STM32
 * 
 * NB The WeAct H273 board seems to have issues with startup - why?
 *    Looks like the 3v3 from the stlink doesn't have quite enough oomph, use usb power as well
*/

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc.h"
#include <stdbool.h>
#include "decoder.h"
#include "contactManager.h"
#include "localConfig.h"
#include "r820t2.h"
#include "main.h"
#include "ugui.h"

#include "EPD_2in9_V2.h"  // Waveshare library for ePaper display
#include "GUI_Paint.h"    // UI library for the ePaper display
// #include "ImageData.h" // for the ePaper test code
// #include "Debug.h"        // for the ePaper library

#include "lcdST7796.h"    // For the 3.5" LCD panel

#include "perfUtil.h"

// Max range in NM to show on the 'radar'
// TODO make the range dynamic depending on the closest approaching contact
#define MAX_PLOT_RANGE 20


// Frame buffer for the epaper display
// only valid if width is a multiple of 8
#if EPD_2IN9_V2_WIDTH % 8 != 0
#error "width not multiple of 8"
#endif
__attribute__((section(".fast_data")))
uint8_t image_bw[EPD_2IN9_V2_WIDTH / 8 * EPD_2IN9_V2_HEIGHT];


// Storage for the radar display window on the large LCD
#define RADAR_WIN_SIZE 240
ALIGN_32BYTES (uint16_t radarWindow[RADAR_WIN_SIZE * RADAR_WIN_SIZE]);

// TODO implement processing packets that cross buffer boundaries and then reduce buffer size again
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
// SPI_HandleTypeDef hspi1;    // 1" lcd
SPI_HandleTypeDef hspi2;    // epaper
SPI_HandleTypeDef hspi3;    // 3.5" lcd
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim7;  // for task profiling

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
// static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);

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

// more stats to do:
// packets that crossed the buffer boundary
// crc fails

volatile uint32_t ulHighFrequencyTimerTicks = 0;


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
  // MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();

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

/**
 * WARNING - assumes all three pins share the same port
*/
void lcd2_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin default states */
  HAL_GPIO_WritePin(LCD2_RESET_GPIO_Port, LCD2_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD2_DC_GPIO_Port, LCD2_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD2_CS_GPIO_Port, LCD2_CS_Pin, GPIO_PIN_SET);

  // Configure GPIO pins : LCD2_RESET_Pin LCD2_DC_Pin LCD2_CS_Pin 
  GPIO_InitStruct.Pin = LCD2_RESET_Pin|LCD2_DC_Pin|LCD2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD2_CS_GPIO_Port, &GPIO_InitStruct);

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
  * 
  * Use TIM7 for FreeRTOS task profiling?
  * 
  * Input clock is 275MHz, we want a tick in the 10 to 100 kHz range (exact value doesn't matter)
  * 
  * Prescale of 25 gives 11MHz input, period of 550 gives 20kHz output
  * 
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 25; // 11MHz
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 550;  // 20kHz  TODO check if this is n+1 (not critical for this use case)
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

  HAL_TIM_Base_Start_IT(&htim7);
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
// static void MX_SPI1_Init(void)
// {
//   /* SPI1 parameter configuration*/
//   hspi1.Instance = SPI1;
//   hspi1.Init.Mode = SPI_MODE_MASTER;
//   hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
//   hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//   hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
//   hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//   hspi1.Init.NSS = SPI_NSS_SOFT;
//   hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
//   // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
//   // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//   // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
//   // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
//   hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//   hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//   hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//   hspi1.Init.CRCPolynomial = 0x0;
//   hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//   hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
//   hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
//   hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//   hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//   hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
//   hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
//   hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//   hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
//   hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
//   if (HAL_SPI_Init(&hspi1) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

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
  * @brief SPI3 init for 3.5" LCD
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
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

  /* DMA1_Stream0_IRQn interrupt configuration, for ADC buffer */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  /* DMA1_Stream1_IRQn interrupt configuration, for SPI3 transfers */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

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
FAST_CODE void filterSamples(uint16_t *input, int len)
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
FAST_CODE int findPreambleInFilteredData(uint16_t *input, int len, int start)
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
FAST_CODE int filteredAmplitudeToBitStream(uint16_t *input, int len, int start)
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

const UG_U16 BackgroundColour = C_BLACK;

/**
 * Clear the window and draw the distance rings and center marker
 * @param size diameter of the plot in pixels
*/
void drawBackground(const uint16_t size)
{
  UG_FillScreen(BackgroundColour);

  const UG_S16 half_width = UG_GetXDim() / 2;
  const UG_S16 half_height = UG_GetYDim() / 2;

  UG_DrawCircle(half_width, half_height, size/2, C_LIGHT_GRAY);

  // TODO intermediate circles should depend on the scale
  if (MAX_PLOT_RANGE == 20) {
    UG_DrawCircle(half_width, half_height, size/4, C_DARK_GRAY); // 10NM marker
  }

  // Center marker
  const uint32_t CenterMarkerSize = 2;
  UG_DrawTriangle(half_width-CenterMarkerSize, half_height+CenterMarkerSize, half_width, half_height-CenterMarkerSize, 
                  half_width+CenterMarkerSize, half_height+CenterMarkerSize, C_WHITE);
}

aircraft_t localContactArray[MAX_CONTACTS];

/**
 * comparison func for passing to qsort
 * Operates on localContactArray which must have been setup prior to calling qsort
*/
static int compareAircraft(const void *p1, const void *p2)
{
  uint8_t index1 = *(uint8_t *)p1;
  uint8_t index2 = *(uint8_t *)p2;

  // aircraft_t *aircraft1 = getContact(index1);
  // aircraft_t *aircraft2 = getContact(index2);
  aircraft_t *aircraft1 = &localContactArray[index1];
  aircraft_t *aircraft2 = &localContactArray[index2];

  if (aircraft1 == NULL || aircraft2 == NULL) {
    // The list must have changed while we were sorting it, return 0 as we have nothing good to do
    return 0;
  }

  // do we need to defend against nan? Would be more efficient to make sure nan was never written to the range field
  if (isnan(aircraft1->range) && isnan(aircraft2->range)) {
    // pedantic, should never happen
    return 0;
  } else if (isnan(aircraft1->range)) {
    return 1;
  } else if (isnan(aircraft2->range)) {
    return -1;
  }

  if (aircraft1->range > aircraft2->range) {
    return 1;
  } else if (aircraft1->range < aircraft2->range) {
    return -1;
  }

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
  char buf[64]; // for generating strings for the displays
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
    // printf("e waiting...\n");
    vTaskDelayUntil(&lastWakeTime, 5000);
    // printf("e wait complete\n");

    Paint_Clear(WHITE);

    // Display uptime
    const uint32_t tSeconds = lastWakeTime / 1000;

    sprintf(buf, "uptime: %lu:%02lu:%02lu", tSeconds / 3600, (tSeconds / 60) % 60, tSeconds % 60);
    Paint_DrawString_EN(EPD_2IN9_V2_HEIGHT - (Font12.Width * strlen(buf)), 128-Font12.Height, buf, &Font12, BLACK, WHITE);

    // Display the time used for decoding the RF
    sprintf(buf, "demod: %2lu%%", demodPercent);
    Paint_DrawString_EN(0, 128-Font12.Height, buf, &Font12, BLACK, WHITE);

    // Init the lookup array that we can sort by distance
    uint8_t aircraftIndexByDistance[MAX_CONTACTS];
    for (int i=0; i<MAX_CONTACTS; i++) {
      aircraftIndexByDistance[i] = i;
    }

    int nContacts = getNumContacts();
    // printf("e nContacts %u\n", nContacts);

    // Copy all the aircraft to a local array so that it can't change while we're using it
    while(true)
    {
      bool failed = false;
      for (int i=0; i<nContacts; i++) {
        aircraft_t *c = getContact(i);
        if (c) {
          localContactArray[i] = *c;
        } else {
          printf("ERROR: contact %d is NULL\n", i);
          // how to handle this case? Should probably start over
          failed = true;
          break;
        }
      }

      if (nContacts != getNumContacts()) {
        failed = true;
        nContacts = getNumContacts();
      }

      if (!failed) break;
    }

    if (nContacts > 0) {

      Paint_DrawString_EN(0, 0, "Cllsgn Range Brng  Spd Trk  Alt  Msgs Age", contactListFont, BLACK, WHITE);

      // printf("e sorting\n");
      qsort(aircraftIndexByDistance, nContacts, sizeof(uint8_t), compareAircraft);
      // printf("e sorted\n");

      int epdLine = 1;
      for(int i=0; i<nContacts; i++) {
        // aircraft_t *aircraft = getContact(aircraftIndexByDistance[i]);
        // aircraft_t *aircraft = getContact(i);
        aircraft_t *aircraft = &localContactArray[aircraftIndexByDistance[i]];
        // This condition can't fail when using the local copy array, so if we keep it we can remove the test
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

          // Check the length of the callsign
          char *ptr = aircraft->callsign;
          uint16_t len = 0;
          while (*ptr != '\0') {
            len++;
            if (len > 8) {
              printf("ERROR: callsign too long\n");
              // prevent the bad string from being written to the display
              aircraft->callsign[0] = '\0';
              break;
            }
            ptr++;
          }
          // printf("e len %d\n", len);

          // Generate a line for the EPD display
          if (epdLine < 10 && rangeNM < 100) {
            char * cs;
            if (aircraft->callsign[0] == '\0') {
              cs = "        ";
            } else {
              cs = aircraft->callsign;
            }
            sprintf(buf, "%s%4.1f%6.1f%4d%4d%6u%4u%4lu", cs, rangeNM, 
            // sprintf(buf, "%4.1f%6.1f%4d%4d%6u%4u%4lu", rangeNM, 
                    aircraft->bearing+180, aircraft->speed, aircraft->track, aircraft->modeC, 
                    aircraft->messages, tSince);
            uint16_t foreground = BLACK, background = WHITE;
            if (closing) {
              foreground = WHITE;
              background = BLACK;
            }
            // printf("e sprint done\n");
            Paint_DrawString_EN(0, epdLine * contactListFont->Height, buf, contactListFont, foreground, background);
            // printf("e draw done\n");
            epdLine++;
            if (epdLine >= 10) break; // no need to process more aircraft if we're out of display lines
          }


        } // if (aircraft)
      } // for (each contact)

    } // if (nContacts > 0)

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
 * Draw the contact position on the 'radar' display
*/
void drawContact(const uint16_t plotDiameter, const aircraft_t *aircraft, bool closing)
{
  const uint16_t PlotRadius = plotDiameter/2;
  const float rangeNM = aircraft->range / 1.852;
  const uint32_t halfWidth = UG_GetXDim()/2;
  const uint32_t halfHeight = UG_GetYDim()/2;

  if (rangeNM < 50) {
    // scale range to the size of the circle for the display (MAX_PLOT_RANGE in NM)
    uint32_t radius = rangeNM * PlotRadius / MAX_PLOT_RANGE;
    // and limit so that distant contacts are drawn at the edge
    if (radius > PlotRadius) radius = PlotRadius;
    uint16_t cx = halfWidth + (int)(radius * sin((aircraft->bearing+180)*M_PI/180));
    uint16_t cy = halfHeight - (int)(radius * cos((aircraft->bearing+180)*M_PI/180)); // minus since 0 is top of display

    // TODO Once we have threat assesment, make the highest threat RED
    UG_COLOR contactColour = C_GREEN;
    if (closing) contactColour = C_YELLOW;

    // Draw a dot to show the position of the aircraft
    // printf("DC aircraft...\n");
    const uint32_t MarkSize = 2;
    UG_FillFrame(cx-MarkSize, cy-MarkSize, cx+MarkSize, cy+MarkSize, contactColour);
    // printf("DC aircraft done\n");

    // Draw a line representing the velocity vector of the aircraft
    // TODO figure out what to do when the line goes outside the drawing area
    const uint32_t speed = aircraft->speed * PlotRadius / (MAX_PLOT_RANGE * 60);  // 60s distance in pixels
    const float bearing = aircraft->track;
    const float bearingRad = bearing * M_PI / 180;
    const int32_t x = cx + (int)(speed * sinf(bearingRad));
    const int32_t y = cy - (int)(speed * cosf(bearingRad));

    // printf("DC vector...\n");
    UG_DrawLine(cx, cy, x, y, contactColour);
    // printf("DC vector done\n");

    // Don't show the position estimate indicator if the contact is clipped to the edge of the display
    if (radius != PlotRadius) {
      // Draw a dot to show the estimated position along the track if the age is older than 10s
      const uint32_t tPosition = aircraft->timestampLatLon;
      const uint32_t tNow = HAL_GetTick();
      const uint32_t tSincePosition = (tNow - tPosition)/1000;
      if (tSincePosition > 10) {
        const uint32_t estDistance = speed * tSincePosition / 60; // scale the original 60s velocity vector by tSincePosition

        const int32_t xEst = cx + (int)(estDistance * sinf(bearingRad));
        const int32_t yEst = cy - (int)(estDistance * cosf(bearingRad));
        // printf("DC est...\n");
        UG_FillFrame(xEst-1, yEst-1, xEst+1, yEst+1, C_WHITE);
        // printf("DC est done\n");

      }
    }

  } // if (rangeNM < 50)

}

/**
 * Runs periodically to write debug to stdout and update the LCD
 * 
*/
void statusTask(void *argument)
{
  // uint32_t cs, ce;  // For perf measurements
  static uint32_t tLast = 0;
  char buf[40]; // for generating strings for the displays

  lcd2_gpio_init();
  LCD_Init();
  LCD_direction(1);

  UG_DEVICE deviceLCD2 = {
    .x_dim = LCD_H,   // using in landscape mode, so x = _h, y - _w
    .y_dim = LCD_W,
    .pset = LCD_DrawPixel,
    .flush = NULL,     // draws straight onto the display, so no flush needed
  };

  UG_GUI guiLCD2;

  memset(&guiLCD2, 0, sizeof(guiLCD2));

  UG_Init(&guiLCD2, &deviceLCD2);

  UG_FontSetHSpace(0);
  UG_FontSetVSpace(0);

  UG_FontSelect(FONT_24X40);
  UG_FontSetTransparency(true);

  LCD_Clear(C_DARK_BLUE);
  UG_PutString(LCD_H/2-(5*24), 130, "ADSB ALERT");

  UG_FontSelect(FONT_7X12);
  UG_PutString(LCD_H/2-(3*7), 200, "V0.0.0");

  UG_Update();

  vTaskDelay(1000);

  LCD_Clear(BackgroundColour);

  // Set up a frame for rendering the radar display (so that it doesn't flicker during updates)

  UG_DEVICE deviceRadarWindow = {
    .x_dim = RADAR_WIN_SIZE,
    .y_dim = RADAR_WIN_SIZE,
    .pset = UG_drawPixelFB,
    .flush = NULL,
    .fb = radarWindow   // memory for the frame buffer
  };

  UG_GUI guiRadarWindow;
  memset(&guiRadarWindow, 0, sizeof(guiRadarWindow));

  UG_Init(&guiRadarWindow, &deviceRadarWindow);

  UG_FontSetHSpace(0);
  UG_FontSetVSpace(0);

  UG_FontSelect(FONT_7X12);
  UG_SetForecolor(C_WHITE);
  UG_SetBackcolor(BackgroundColour);

  while(true)
  {
    vTaskDelay(2000);

    #ifndef BEAST_OUTPUT

    const uint32_t now = HAL_GetTick();
    uint32_t diff = now - tLast;
    // uint64_t sps = (uint64_t)ADC_CONVERTED_DATA_BUFFER_SIZE*1000 * conversionsCompleted / diff;
    // printf("Time taken: %lu, buffers:%lu sps=%lu", diff, conversionsCompleted, (uint32_t)sps);
    tLast = now;
    conversionsCompleted = 0;

    if (missedBuffers) {
      printf(" %ld missed buffers\n", missedBuffers);
      missedBuffers = 0;
    // } else {
    //   printf("\n");
    }

    // char statsBuffer[512];
    // vTaskGetRunTimeStats(statsBuffer);
    // printf("%s\n", statsBuffer);

    uint32_t idlePercent = 0;
    const uint32_t idleCount = ulTaskGetIdleRunTimeCounter();
    const uint32_t totalCountDiv100 = ulHighFrequencyTimerTicks / 100;
    if (totalCountDiv100 > 0) {
      idlePercent = idleCount / (ulHighFrequencyTimerTicks / 100);
      // printf("Idle %lu%%\n", idlePercent);
    }


    // demodPercent is global and used by epaper task
    demodPercent = usSpentByDemod / (diff * 10);

    // printf("us demod %lu (%lu%%)\n", usSpentByDemod, demodPercent);
    usSpentByDemod = 0;

    // printf("average %u\n", globalAvg);

    // Display uptime
    const uint32_t tSeconds = now / 1000;

    sprintf(buf, "uptime: %lu:%02lu:%02lu", tSeconds / 3600, (tSeconds / 60) % 60, tSeconds % 60);
    UG_SelectGUI(&guiLCD2);
    UG_FontSelect(FONT_10X16);
    UG_FontSetTransparency(false);
    // 360 is the midpoint of the right hand half of the display. strlen/2 * fontwidth centres the text
    const uint32_t x = 360 - (strlen(buf) * 5);
    UG_PutString(x, 319-16, buf);

    // Show some stats, e.g. total aircraft, total messages
    UG_FontSelect(FONT_8X12);
    sprintf(buf, "aircraft %lu", totalContactsSeen);
    UG_PutString(250, 0, buf);

    sprintf(buf, "messages %lu", totalMessages);
    UG_PutString(250, 14, buf);

    sprintf(buf, "corrected %lu", totalCorrected);
    UG_PutString(250, 28, buf);

    sprintf(buf, "maxRange %.1f", maxRange / 1.852);  // convert to nautical miles
    UG_PutString(250, 42, buf);

    sprintf(buf, "idle %lu%%", idlePercent);
    UG_PutString(250, 56, buf);


    UG_SelectGUI(&guiRadarWindow);
    drawBackground(RADAR_WIN_SIZE-10);

    const int nContacts = getNumContacts();

    sprintf(buf, "%d ", nContacts);
    UG_PutString(0, 0, buf);

    if (nContacts > 0) {
      int nApproaching = 0;
      float minRange = 100;
      int32_t closestIndex = -1;
      printf("\n%d Contacts:\n", nContacts);
      printf("Index\t");
      #ifdef SHOW_ADDR
      printf("Addr Callsign ");
      #endif
      printf("Wake  Range Brng Speed track  bAlt   gAlt  vSpd Msgs Age\n");

      int oldContactIndex = -1;
      for(int i=0; i<nContacts; i++) {
        aircraft_t *aircraft = getContact(i);
        if (aircraft) { // in case the number of entries in the list changes since we read it
          if (aircraft->timestamp + 120000 < now) {
            oldContactIndex = i;
            continue; // SKIP processing old contact
          }
          // printf("calcs...\n");
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
          uint32_t tNow = HAL_GetTick();
          uint32_t tSince = (tNow - tLocal)/1000;
          printf("%2d: %c ", i, closing ? 'C' : ' ');
          #ifdef SHOW_ADDR
          char * cs;
          if (aircraft->callsign[0] == '\0') {
            cs = "        ";
          } else {
            cs = aircraft->callsign;
          }

          printf("%6lx %s ", aircraft->addr, cs);
          // printf("%6lx ", aircraft->addr);
          #endif
          float rangeNM = aircraft->range / 1.852;  // range is held in km, so convert to NM
          printf("%4u %6.2f %5.1f %4d %4d %6u %6u %5d %4u %3lu\n", aircraft->wake_class, rangeNM, 
                  aircraft->bearing+180, aircraft->speed, aircraft->track, aircraft->modeC, aircraft->altitude, aircraft->vert_rate,
                  aircraft->messages, tSince);

          if (closing && rangeNM < minRange) {
            minRange = rangeNM;
            closestIndex = i;
          }

          // printf("drawing...\n");
          drawContact(RADAR_WIN_SIZE-10, aircraft, closing);

        } // if (aircraft)
        // printf("next\n");
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
          UG_FONT * activeFont = UG_GetGUI()->font;
          const uint32_t fWidth = UG_GetFontWidth(activeFont);
          UG_PutString((RADAR_WIN_SIZE-1)-(len * fWidth), 0, buf);

          sprintf(buf, "%2.1f", minRange);
          len = strlen(buf);
          UG_PutString((RADAR_WIN_SIZE-1)-(len * fWidth), 20, buf);

          sprintf(buf, "%d", aircraft->speed);
          len = strlen(buf);
          UG_PutString((RADAR_WIN_SIZE-1)-(len * fWidth), 40, buf);
        }
      }
    } // if (nContacts > 0)

    LCD_WriteWindow(0, 0, RADAR_WIN_SIZE, RADAR_WIN_SIZE, radarWindow);

    #endif // not BEAST_OUTPUT

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

FAST_CODE void processBuffer(uint16_t *input, int len)
{
  #define BUFFERS_FOR_AVG 10
  // static uint16_t nAvgCounter = 0;
  // static uint16_t averages[BUFFERS_FOR_AVG];

  // Just use a fixed value? How much part to part variation is there?
  // In theory this might be expected to be 127 but it seems to mostly average lower
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
  vTaskDelay(1000); // is this needed?

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
    if (attempts > 10) {
      return 0;
    }
    osDelay(2);
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
FAST_CODE void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    ulHighFrequencyTimerTicks++;
  } else if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
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
FAST_CODE void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
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
FAST_CODE void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE/2], ADC_CONVERTED_DATA_BUFFER_SIZE);
  conversionsCompleted++;

  if (conversionsCompleted % 500 == 0) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
  }

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

volatile bool spiTransmitInProgress = false;

/**
 * When using DMA to transfer pixel data, we need to know when the dma transfer is complete.
*/
FAST_CODE void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi3) {
    // Clear the global bool to track the SPI transmit state
    spiTransmitInProgress = false;

    if (statusTID) {
      BaseType_t taskWoken = 0;
      xTaskNotifyFromISR(statusTID , 0, eNoAction, &taskWoken);
      if (taskWoken == pdTRUE) {
        portYIELD_FROM_ISR(taskWoken);
      }
    }
  } else {
    // Not expecting anything else to be using DMA yet
    printf("SPI callback called with hspi != &hspi3\n");
  }
}

