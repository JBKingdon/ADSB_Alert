/**
 * Initial beginnings of an ADSB receiver for STM32
 * 
 * NB The WeAct H273 board seems to have issues with startup - why?
 *    Looks like the 3v3 from the stlink doesn't have quite enough oomph
*/

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc.h"
#include <stdbool.h>

#define LED_PIN                                GPIO_PIN_3
#define LED_GPIO_PORT                          GPIOE
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOE_CLK_ENABLE()

extern USBD_HandleTypeDef hUsbDeviceHS;

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4096)   /* number of entries of array aADCxConvertedData[] */

/* Variable containing ADC conversions data */
ALIGN_32BYTES (static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);

// The bitstream array holds the 1Mhz bits converted from the raw samples.
// With 8Msps raw input and only processing each half of the buffer at a time we have
#define BITSTREAM_BUFFER_ENTRIES   ((uint32_t)  (ADC_CONVERTED_DATA_BUFFER_SIZE/16))  
uint8_t bitstream[BITSTREAM_BUFFER_ENTRIES];


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim7;

osThreadId_t blink01Handle;
const osThreadAttr_t blink01_attributes = {
  .name = "blink01",
  .stack_size = 8192, // XXX check regularly
  .priority = (osPriority_t) osPriorityNormal,
};

void SystemClock_Config(void);
// static void MPU_Initialize(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
void StartBlink01(void *argument);
void Error_Handler(void);

void LED_Init();

float globalSum = 0;

uint32_t conversionsCompleted = 0;

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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();


  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink01 */
  blink01Handle = osThreadNew(StartBlink01, NULL, &blink01_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  LED_Init();

}


// int doFloatTest() 
// {

//   float a = 3.14159f;
//   float b = 2.71828f;

//   float sum = 0.0f;
//   for(int i=0; i<5000000; i++) {
//     sum += a * b; 
//   }

//   // printf("Sum is: %f\n", sum);
//   globalSum += sum;

//   return 0;
// }

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
void ampToBitStream(uint16_t *input, int len, uint8_t *output)
{
  const int nOutBits = len/2;
  for(int i=0; i<nOutBits; i++) {
    if(input[2*i] > input[2*i+1]) {
      output[i] = 1;
    } else {
      output[i] = 0;
    }
  }
}

/**
 * Search a bitstream for the ADSB preamble pattern
 * The preamble is 11??00??. The ? bits are indeterminate
 * as they are not valid pulse encodings and will be randomly
 * converted by ampToBitStream
*/
int findPreamble(uint8_t *input, int len, int start)
{
  for(int i=start; i<len; i++) {
    // 11??00??
    if (input[i] == 1 && input[i+1] == 1 && input[i+4] == 0 && input[i+5] == 0) {
      return i;
    }
  }
  return -1;
}

/**
 * read 8 bits from the bitstream starting at the specified index
 * 
 * NB The caller is responsible for making sure there are enough values in the stream
 * 
 * @param input bitstream to read from as an array of uint8_t, values must be 0 or 1
 * @param start index in the bitstream to start reading from
*/
uint8_t read8Bits(uint8_t *input, int start)
{
  uint8_t df = 0;

  for(int i=0; i<8; i++) {
    df = df << 1;
    uint8_t t = input[start+i];
    if (t > 1) {
      printf("ERROR bad bitsream value %d at index %d\n", t, start+i);
      return 0;
    }
    df |= t;
  }

  return df;
}

/**
 * read 24 bits from the bitstream starting at the specified index
 * 
 * NB The caller is responsible for making sure there are enough values in the stream
 * 
 * @param input bitstream to read from as an array of uint8_t, values must be 0 or 1
 * @param start index in the bitstream to start reading from
 * 
 * TODO generalise to N and replace the width specific versions with wrappers
*/
uint32_t read24Bits(uint8_t *input, int start)
{
  uint32_t res = 0;

  for(int i=0; i<24; i++) {
    res = res << 1;
    uint8_t t = input[start+i];
    if (t > 1) {
      printf("ERROR bad bitsream value %d at index %d\n", t, start+i);
      return 0;
    }
    res |= t;
  }

  return res;
}


/**
 * Attempt to decode a modeS message at the specified start index
 * 
 * TODO pass the start index of the data instead of the preamble
 * TODO what do we want the output of this to be, and what return values?
*/
void decodeModeS(uint32_t preambleStart)
{
  // Do we have enough bits for a complete message?
  if ((preambleStart + 120) < BITSTREAM_BUFFER_ENTRIES) 
  {
    // Read bits 8 to 12 for the DF
    const int dfStart = preambleStart + 8;
    uint8_t df = 0;

    for(int i=0; i<5; i++) {
      df = df << 1;
      if (bitstream[dfStart+i] > 1) {
        printf("ERROR bad bitsream value %d at index %d\n", bitstream[dfStart+i], dfStart+i);
        return;
      }
      df |= bitstream[dfStart+i];
    }

    // printf("DF: %02x (%d)\n", df, df);

    switch (df) {
      case 0:
      case 4:
      case 5:
      case 11:
        // printf("df0,4,5,11\n");
        // if (preambleStart + 56 < (ADC_CONVERTED_DATA_BUFFER_SIZE/16)) {
        //   uint8_t b1 = read8Bits(bitstream, preambleStart + 32);
        //   uint8_t b2 = read8Bits(bitstream, preambleStart + 40);
        //   uint8_t b3 = read8Bits(bitstream, preambleStart + 48);
        //   uint32_t address = (b1 << 16) | (b2 << 8) | b3;
        //   printf("Address: %08lx\n", address);
        // } else {
        //   printf("packet too close to the end to read address\n");
        // }
        break;
      case 17:  // These are what we are expecting for ADS-B squitter messages
          // DF starts at +8
          // CA at +13
          // Address at +16
          // ME (payload) at +40
          // Parity at +96
        {
          // printf("ADS-B squitter message\n");
          uint8_t b1 = read8Bits(bitstream, preambleStart + 16);
          uint8_t b2 = read8Bits(bitstream, preambleStart + 24);
          uint8_t b3 = read8Bits(bitstream, preambleStart + 32);
          uint32_t address = (b1 << 16) | (b2 << 8) | b3;
          printf("Address: %06lx\n", address);
        }
        break;
      default:
        // if ((df & 0b11000) == 0b11000) {
        //   printf("extended length message\n");
        // }
        break;
    }
  } // if (enough bits)

}

// From dump1090, crc calculation
//
// ===================== Mode S detection and decoding  ===================
//
// Parity table for MODE S Messages.
// The table contains 112 elements, every element corresponds to a bit set
// in the message, starting from the first bit of actual data after the
// preamble.
//
// For messages of 112 bit, the whole table is used.
// For messages of 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the
// end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have
// the CRC xored with the sender address as they are reply to interrogations,
// but a casual listener can't split the address from the checksum.
//
const uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

/**
 * modified to operate on the bitsream array which is one byte per bit
*/
uint32_t modeSChecksum(int bits) 
{
  uint32_t   crc = 0;

  int        offset = (bits == 112) ? 0 : (112-56);

  const uint32_t * pCRCTable = &modes_checksum_table[offset];
  int j;

  // We don't really need to include the checksum itself
  bits -= 24;
  for(j = 0; j < bits; j++) {
    // If bit is set, xor with corresponding table entry.
    if (bitstream[j]) {crc ^= *pCRCTable;} 
    pCRCTable++;
  }

  return (crc & 0x00FFFFFF); // JBK - it shouldn't be possible for any of the high bits to be set, so this is unnecessary?
}


/**
 * decodeModeS using bitstream with no preamble
*/
bool decodeModeS_NEW()
{
  bool crcOk = false;

  // Read 5 bits for the DF
  const int dfStart = 0;
  uint8_t df = 0;

  for(int i=0; i<5; i++) {
    df = df << 1;
    if (bitstream[dfStart+i] > 1) {
      printf("ERROR bad bitsream value %d at index %d\n", bitstream[dfStart+i], dfStart+i);
      return false;
    }
    df |= bitstream[dfStart+i];
  }

  // printf("DF: %02x (%d)\n", df, df);

  switch (df) {
    case 0:
    case 4:
    case 5:
    case 11:
      // printf("df0,4,5,11\n");
      // if (preambleStart + 56 < (ADC_CONVERTED_DATA_BUFFER_SIZE/16)) {
      //   uint8_t b1 = read8Bits(bitstream, preambleStart + 32);
      //   uint8_t b2 = read8Bits(bitstream, preambleStart + 40);
      //   uint8_t b3 = read8Bits(bitstream, preambleStart + 48);
      //   uint32_t address = (b1 << 16) | (b2 << 8) | b3;
      //   printf("Address: %08lx\n", address);
      // } else {
      //   printf("packet too close to the end to read address\n");
      // }
      break;
    case 17:  // These are what we are expecting for ADS-B squitter messages
        // DF starts at +8
        // CA at +13
        // Address at +16
        // ME (payload) at +40
        // Parity at +96
      {
        // printf("ADS-B squitter message\n");
        const uint32_t expectedCrc = modeSChecksum(112);
        const uint32_t actualCrc = read24Bits(bitstream, 88);
        crcOk = (actualCrc == expectedCrc);
        // printf("Expected CRC: %08lx, actual CRC: %08lx, %s\n", expectedCrc, actualCrc, crcOk ? "MATCHED" : "failed");

        if (crcOk) {
          uint8_t b1 = read8Bits(bitstream, 8);
          uint8_t b2 = read8Bits(bitstream, 16);
          uint8_t b3 = read8Bits(bitstream, 24);
          uint32_t address = (b1 << 16) | (b2 << 8) | b3;
          printf("Address: %06lx\n", address);
        }
      }
      break;
    default:
      // if ((df & 0b11000) == 0b11000) {
      //   printf("extended length message\n");
      // }
      break;
  }

  return crcOk;
}

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

/**
  * @brief  Initial testing only
  * @param  argument: Not used
  * @retval None
  */
void StartBlink01(void *argument)
{
  // static uint32_t tLast = 0;
	uint32_t count = 0;

  for(;;)
  {
    // TODO real thread will use queue based notification

    // HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    // osDelay(1000);
    // printf("Hello from printf! %ld\n", count++);
    
    // printf("Time taken: %ld\n", t1-t0);

    // uint32_t sysClockFreq;
    // sysClockFreq = HAL_RCC_GetSysClockFreq();

    // printf("Clock freq %lu\n", sysClockFreq);

    if (lowBuf) {
      bool oldHiBuf = hiBuf;

      uint16_t localSamples[ADC_CONVERTED_DATA_BUFFER_SIZE/2];

      // Grab the data into the local array as quickly as possible
      memcpy(localSamples, aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE); // half the buffer, so /2, but 2 bytes per sample so *2

      // if hiBuf was set while we were doing the copy then we likely have corrupted data
      if (hiBuf) {
        printf("ERROR: HI BUF SET, old value was %d\n", oldHiBuf);
      }

      uint16_t avg = calculateAverage(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);
      // printf("Average: %d\n", avg);

      uint16_t max;
      // debugging - dump data when there's a strong signal
      // uint16_t max = findMaxValue(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);
      // bool dumpIt = (max > 180);
      // if (dumpIt) {
      //   // dump localSamples[] to stdout
      //   printf("RAW:\n");
      //   printArray(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);
      // }

      max = 0;  // Get a new max after removing dc offset
      for(int i=0; i<(ADC_CONVERTED_DATA_BUFFER_SIZE/2); i++) {
        int t = localSamples[i];
        // DC offset and abs value
        const uint16_t amp = (t > avg) ? (t - avg) : avg - t;
        localSamples[i] = amp;
        if (amp > max) {
          max = amp;
        }
      }

      // if (dumpIt) {
      //   printf("ABS:\n");
      //   printArray(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);
      // }

      if (max > 40) {
        filterSamples(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);
        // printf("FILTERED:\n");
        // printArray(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);

        // Find the preamble in the filtered data
        int preambleIndex = 0;
        while(preambleIndex < (ADC_CONVERTED_DATA_BUFFER_SIZE/2)) {
          preambleIndex = findPreambleInFilteredData(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2, preambleIndex);
          if (preambleIndex == -1) {
            break;
          } else {
            // printf("Found preamble at %d\n", preambleIndex);

            filteredAmplitudeToBitStream(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2, preambleIndex+64);

            bool success = decodeModeS_NEW();
            if (success) {
              preambleIndex += (120 * 8); // move the pointer past the message
            } else {
              preambleIndex++;
            }
          }
        }

      }


      const uint16_t SIGNAL_THRESHOLD = 1000; // XXX high value for testing, see how low we can reduce it to later

      if (max > SIGNAL_THRESHOLD)
      {
        uint16_t decimated[ADC_CONVERTED_DATA_BUFFER_SIZE/8]; // half the input buffer, with a 4:1 decimation
        uint16_t localSamplesDecimated[ADC_CONVERTED_DATA_BUFFER_SIZE/8];

        // uint16_t globalRangeMax = 0;

        for(int offset=0; offset<4; offset++) 
        {
          // printf("offset: %d\n", offset);
          uint16_t min = 9999, max = 0;
          for(int i=0; i<(ADC_CONVERTED_DATA_BUFFER_SIZE/8)-1; i++) {
            const int j = (i * 4) + offset;
            uint16_t a = localSamples[j];
            uint16_t b = localSamples[j+1];
            uint16_t c = localSamples[j+2];
            uint16_t d = localSamples[j+3];
            localSamplesDecimated[i] = a + b + c + d;
            // printf("i, j, a, b, c, d: %d %d %d %d %d %d, res %d\n", i, j, a, b, c, d, decimated[i]);
            // printf("%d ", localSamplesDecimated[i]);
            if (localSamplesDecimated[i] < min) {
              min = localSamplesDecimated[i];
            } else if (localSamplesDecimated[i] > max) {
              max = localSamplesDecimated[i];
            }
          }
          // printf("\n");
          // printf("Min: %d, Max: %d, range %d\n", min, max, max-min);

          // if (max-min > globalRangeMax) {
          //   globalRangeMax = max-min;
            memcpy(decimated, localSamplesDecimated, (ADC_CONVERTED_DATA_BUFFER_SIZE/8)*2);
            // printf("copying data for offset %d\n",offset);
          // }

          // Loop to check both 0 and 1 starting points for ampToBitStream as we don't know the initial phase.
          // TODO check for both phases with a sliding window that doesn't convert the whole buffer up front
          // Maybe use probability of each bit being 1 and generate an overal confidence of matching the preamble.

          for(int phase=0; phase<2; phase++) {

            // printf("phase %d\n", phase);

            ampToBitStream(&decimated[phase], ADC_CONVERTED_DATA_BUFFER_SIZE/8, bitstream);
            // for(int k=0; k<(ADC_CONVERTED_DATA_BUFFER_SIZE/16); k++) {
            //   printf("%d ", bitstream[k]);
            // }
            // printf("\n");

            int preambleStart = 0; 
            
            while(preambleStart < (BITSTREAM_BUFFER_ENTRIES-120))
            {
              preambleStart = findPreamble(bitstream, ADC_CONVERTED_DATA_BUFFER_SIZE/16, preambleStart);
              if (preambleStart != -1) {
                // printf("Preamble found at %d\n", preambleStart);
                printf(".");
                // for(int i=preambleStart; i<preambleStart+16; i++) {
                //   if (i >= (ADC_CONVERTED_DATA_BUFFER_SIZE/16)) break;
                //   printf("%d ", bitstream[i]);
                // }
                // printf("\n");

                // TODO if we got a success/fail return value we would know how much to increment the start pointer by
                decodeModeS(preambleStart);
              } else {
                break;  // stop searching for preambles
              }
              preambleStart++; // step past the current hit and search again
            } // while (still room for a packet)

          } // for(int phase=0; phase<2; phase++)


        } // for(int offset=0; offset<4; offset++)

        // printf("Signal %u\n", globalRangeMax);


        // // find the first sample
        // const uint32_t HIGH_THRESHOLD = 15;

        // int32_t signal_start = -1;
        // for(int i=0; i<(ADC_CONVERTED_DATA_BUFFER_SIZE/2); i++) {
        //   if (localSamples[i] > HIGH_THRESHOLD) {
        //     signal_start = i;
        //     break;
        //   }
        // }

        // if (signal_start == -1) {
        //   // shouldn't happen
        //   printf("No signal found\n");
        // } else {

        //   #define HUMAN_FORMAT 1
        //   #ifdef HUMAN_FORMAT
        //   const uint32_t SamplesPerLine = 32;
        //   for(int i=signal_start; i<(ADC_CONVERTED_DATA_BUFFER_SIZE/2); i++) {
        //     printf("%d ", localSamples[i]);
        //     if (i % SamplesPerLine == (SamplesPerLine - 1)) {
        //       printf("\n");
        //     }
        //   }
        // }
        // #else
        // for(int i=signal_start; i<(ADC_CONVERTED_DATA_BUFFER_SIZE/2); i++) {
        //   printf("%d\n", localSamples[i]);
        // }
        // #endif
      }

      lowBuf = false;
    }


    if (hiBuf) {
      // printf("hibuf set\n");

      // get the stack high water mark
      if((count++ % 10000) == 0) {
        // const uint32_t now = HAL_GetTick();
        // uint32_t diff = now - tLast;
        // uint64_t sps = (uint64_t)ADC_CONVERTED_DATA_BUFFER_SIZE*1000 * conversionsCompleted / diff;
        // printf("Time taken: %lu, buffers:%lu sps=%lu\n", diff, conversionsCompleted, (uint32_t)sps);
        // tLast = now;
        // conversionsCompleted = 0;

        // debug will have thrown timing out, so clear lowBuf too
        // lowBuf = false;

        UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("Stack high water mark %lu\n", stackHighWaterMark);
        if (stackHighWaterMark == 0) {
          printf("ERROR: stack overflow!!\n");
        }

      }

      hiBuf = false;

    }

  }

}

/* MPU Configuration */

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
 * TODO Implement a better buffering system to make this more efficient`
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
 * TODO add some debug commands to set thresholds
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
  lowBuf = true;
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
  hiBuf = true;
}
