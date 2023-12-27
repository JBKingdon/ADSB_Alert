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
#include "decoder.h"
#include "contactManager.h"

#define LED_PIN                                GPIO_PIN_3
#define LED_GPIO_PORT                          GPIOE
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOE_CLK_ENABLE()

extern USBD_HandleTypeDef hUsbDeviceHS;

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4096)   /* number of entries of array aADCxConvertedData[] */

// TODO - is it possible to hold samples as packed 8 bit values?
/* Variable containing ADC conversions data */
ALIGN_32BYTES (static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);

// The bitstream array holds the 1Mhz bits converted from the raw samples.
// With 8Msps raw input and only processing each half of the buffer at a time we have
// TODO current usage should be that the bitstream is max 112 bits long to hold a single message. Confirm and resize.
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

const osThreadAttr_t statusThread_attributes = {
  .name = "status",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,  // TODO when the queue is implemented for notifying the work thread this can be lowered
};


void SystemClock_Config(void);
// static void MPU_Initialize(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
void StartBlink01(void *argument);
void statusTask(void *argument);
void Error_Handler(void);

void LED_Init();

float globalSum = 0;

uint32_t conversionsCompleted = 0;

bool lowBuf = false;
bool hiBuf = false;

uint32_t missedBuffers = 0;

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

  // status thread
  osThreadNew(statusTask, NULL, &statusThread_attributes);

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

void statusTask(void *argument)
{
  static uint32_t tLast = 0;

  while(true)
  {
    osDelay(2000);

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

    const int nContacts = getNumContacts();
    if (nContacts > 0) {
      printf("\nContacts:\n");
      printf("Index\tAddr\tRange\tBearing\tMsgs\tAge\n");
      uint32_t tNow = HAL_GetTick();
      for(int i=0; i<nContacts; i++) {
        aircraft_t *aircraft = getContact(i);
        uint32_t tSince = (tNow - aircraft->timestamp)/1000;
        printf("%2d: %6lx\t%2.2f\t%3.2f\t%3lu\t%2lu\n", i, aircraft->addr, aircraft->range / 1.852, 
                aircraft->bearing, aircraft->messages, tSince);
      }
      printf("furthest: %d, oldest: %d\n", findMostDistantContact(), findOldestContact());
    }

    #endif

    // Need the taskHandle of the main task
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if (stackHighWaterMark == 0) {
      printf("ERROR: stack overflow on status task!!\n");
    }

    stackHighWaterMark = uxTaskGetStackHighWaterMark(blink01Handle);
    if (stackHighWaterMark == 0) {
      printf("ERROR: stack overflow on main task!!\n");
    }

  }
}

void processBuffer(uint16_t *input, int len)
{
  uint16_t avg = calculateAverage(input, len);
  // printf("Average: %d\n", avg);

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
    const uint16_t amp = (t > avg) ? (t - avg) : avg - t;
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

  }

}

/**
  * @brief  Initial testing only
  * @param  argument: Not used
  * @retval None
  */
void StartBlink01(void *argument)
{
  // static uint32_t tLast = 0;
	// uint32_t count = 0;
  uint16_t localSamples[ADC_CONVERTED_DATA_BUFFER_SIZE/2];

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

      // Grab the data into the local array as quickly as possible
      memcpy(localSamples, aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE); // half the buffer, so /2, but 2 bytes per sample so *2

      // if hiBuf was set while we were doing the copy then we likely have corrupted data
      if (hiBuf) {
        printf("ERROR: HI BUF SET, old value was %d\n", oldHiBuf);
      }

      processBuffer(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);

      lowBuf = false;
    }


    if (hiBuf) {
      // printf("hibuf set\n");

      bool oldLoBuf = lowBuf;


      // Grab the data into the local array as quickly as possible
      memcpy(localSamples, &(aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE/2]), ADC_CONVERTED_DATA_BUFFER_SIZE); // half the buffer, so /2, but 2 bytes per sample so *2

      // if loBuf was set while we were doing the copy then we likely have corrupted data
      if (lowBuf) {
        printf("ERROR: Low BUF SET, old value was %d\n", oldLoBuf);
      }

      processBuffer(localSamples, ADC_CONVERTED_DATA_BUFFER_SIZE/2);


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
  if (lowBuf) missedBuffers++;
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
  if (hiBuf) missedBuffers++;
  hiBuf = true;
}
