/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include <SHTC3.h>
#include <st7735.h>
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSIZE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Light */
osThreadId_t LightHandle;
const osThreadAttr_t Light_attributes = {
  .name = "Light",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART */
osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2C_SHTC3 */
osThreadId_t I2C_SHTC3Handle;
const osThreadAttr_t I2C_SHTC3_attributes = {
  .name = "I2C_SHTC3",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue_ADC */
osMessageQueueId_t myQueue_ADCHandle;
const osMessageQueueAttr_t myQueue_ADC_attributes = {
  .name = "myQueue_ADC"
};
/* Definitions for myQueue_SHCT3 */
osMessageQueueId_t myQueue_SHCT3Handle;
const osMessageQueueAttr_t myQueue_SHCT3_attributes = {
  .name = "myQueue_SHCT3"
};
/* Definitions for myEvent01 */
osEventFlagsId_t myEvent01Handle;
const osEventFlagsAttr_t myEvent01_attributes = {
  .name = "myEvent01"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
volatile uint32_t value_ADC=0;
volatile uint8_t flag_uart=0;
volatile uint8_t Data_rx[BUFSIZE];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc2);
  HAL_TIM_Base_Start(&htim2);
  HAL_UART_Receive_IT(&huart1, Data_rx,2);
  ST7735_Init(&hspi2);
  ST7735_FillScreen(ST7735_BLACK);
  /* USER CODE END 2 */

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

  /* Create the queue(s) */
  /* creation of myQueue_ADC */
  myQueue_ADCHandle = osMessageQueueNew (5, sizeof(uint32_t), &myQueue_ADC_attributes);

  /* creation of myQueue_SHCT3 */
  myQueue_SHCT3Handle = osMessageQueueNew (16, sizeof(uint8_t), &myQueue_SHCT3_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Light */
  LightHandle = osThreadNew(StartTask03, NULL, &Light_attributes);

  /* creation of UART */
  UARTHandle = osThreadNew(StartTask04, NULL, &UART_attributes);

  /* creation of I2C_SHTC3 */
  I2C_SHTC3Handle = osThreadNew(StartTask05, NULL, &I2C_SHTC3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of myEvent01 */
  myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);

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
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_CS_Pin|LCD_RST_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_OUT_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LED_OUT_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	value_ADC = HAL_ADC_GetValue(&hadc2);																	// every millisecond
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		flag_uart=1;
	}
}
void send_float(uint8_t len, float *ptrData,UART_HandleTypeDef *huart )
{
	uint8_t *data;
	data=(uint8_t*)ptrData;
	for(int i=0;i<len; i++)
		HAL_UART_Transmit(&huart1, (data+i), 1, 100);
}
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    uint32_t run = in_max - in_min;
    uint32_t rise = out_max - out_min;
    uint32_t delta = x - in_min;

    return (delta * rise) / run + out_min;
}

/*
  Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 байт(127 бит)

*/
uint8_t crc_8(uint8_t *pcBlock, int len)
{
    uint8_t crc_8=0xFF;
    int i;


    while (len--)
    {
        crc_8 ^= *pcBlock++;

    		for(int i=0; i<8; i++){
    			if(crc_8 & 0x80)
    				crc_8=(crc_8<<1u)^polin;
    			else
    				crc_8=crc_8<<1u;
    		}
    }

    return crc_8;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Light thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint8_t ADC_1=0, crc_rx=0x00;
	char str[10];

	uint8_t byteWait=0x01;
	uint8_t byte_receive=0x00;
	uint8_t service_byte = 0xAF;
	uint8_t tx_data[3]={0,0,0};
  /* Infinite loop */
	for(;;)
	{
		byte_receive=xEventGroupWaitBits(myEvent01Handle, byteWait, byteWait, pdFALSE, 100);
		ADC_1=map(value_ADC, 0, 4095,0, 100);
		if(ADC_1<50)
			HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);


		if(byte_receive==byteWait)
		{
			tx_data[0]=service_byte; tx_data[1]=ADC_1;
			crc_rx=crc_8(tx_data,2);
			tx_data[2]=crc_rx;
			HAL_UART_Transmit(&huart1, tx_data,sizeof(tx_data),100);

		}
		sprintf(str,"%d",ADC_1);
		ST7735_DrawString(2, 2, str, ST7735_ORANGE, 1);
		osDelay(100);
	}
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	uint8_t str_dead[]={0xAD,0xDE};
	char brig[]				="brigth_esp32";
	char temper[]			="temper_esp32";
	char mqtt_data[]	="mqtt_getdata";
	char mqtt_pub[]		="mqtt_publish";
	uint8_t repeat[3]={0xFF,0xDD,0x00};
	uint8_t data_ADC=0;
	uint8_t data_rx=0xAF, crc_rx = 0;
  /* Infinite loop */
	repeat[2]=crc_8(repeat,2);

	for(;;)
	{

		//i2c_trans_temp(&read_t, &read_f, &crc_ok);

		//xQueueReceive(myQueue_ADCHandle, &data_ADC, ( TickType_t ) 10);
		//xQueueReceive(myQueue_SHCT3Handle, data_SHCT3, ( TickType_t ) 10);
		//crc_rx = crc_8(&data_rx,1);
		if(flag_uart==1)
		{
			crc_rx = crc_8(&Data_rx,1);
			if(crc_rx==Data_rx[1])
			{
				switch(Data_rx[0])
				{
					case(0xAF):
					{
						//byte_tx=0xF1;
						ST7735_DrawString(2, 7, temper, ST7735_YELLOW, 1);
						xEventGroupSetBits(myEvent01Handle, 0x01);
						//data_ADC=map(value_ADC, 0, 4095,0, 100);
						//HAL_UART_Transmit(&huart1, &data_ADC,sizeof(data_ADC),100);
						break;
					}
					case(0x1F):																					// Запрос температуры
					{
						ST7735_DrawString(2, 7, brig, ST7735_YELLOW, 1);
						xEventGroupSetBits(myEvent01Handle, 0x02);
						break;
					}
					case(0x06):																					// Данные приняты
					{
						ST7735_DrawString(2, 9, mqtt_data, ST7735_YELLOW, 1);

						break;
					}
					case(0x08):																					// данные успешно опубликованы
					{
						ST7735_DrawString(2, 9, mqtt_pub, ST7735_YELLOW, 1);
						break;
					}
					default:
					{;
						HAL_UART_Transmit(&huart1, str_dead, sizeof(str_dead), 100);
						break;
					}
				}
			}
			else
			{

				HAL_UART_Transmit(&huart1, repeat, sizeof(repeat), 100);
			}

			/*if(Data_rx==0xAF)
			{
				HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
			}
			else
				HAL_UART_Transmit(&huart1, &data_ADC, sizeof(data_ADC) , 100);
			}*/

				//HAL_UART_Transmit(&huart1, Buf1, sizeof(Buf1), 100);

			flag_uart=0;

		}
		if (flag_uart==0)
			HAL_UART_Receive_IT(&huart1, Data_rx, 2);
		//sprintf(str,"%d.%3\t",(uint32_t)read_t, (uint16_t)((read_t - (uint32_t)read_t)*1000));
		//HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
		osDelay(100);
	}
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the I2C_SHCT3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	float read_t=18.2;
	float read_h=20.6;
	char str[25];
	char str1[25];
	uint8_t byteWait=0x02;
	uint8_t byteReceive=0;
	uint8_t service_byte = 0x1F;
	uint8_t crc_ok=0;
  /* Infinite loop */
	for(;;)
	{
		//xEventGroupWaitBits(myEvent01Handle, byteWait, byteWait, pdFALSE, portMAX_DELAY);
		i2c_trans_temp(&read_t, &read_h, &crc_ok);

		byteReceive=xEventGroupWaitBits(myEvent01Handle, byteWait, byteWait, pdFALSE, 1);

		//sprintf(str,"%2.1f",read_t);
		//sprintf(str1,"%3.2f", read_h);
	//	ST7735_DrawString(2, 4, str, ST7735_GREEN, 1);					//Отображаем принитые температуру на экране st7735
	//	ST7735_DrawString(2, 5, str1, ST7735_RED, 1);


		if((crc_ok==1)&&(byteReceive==byteWait))
		{
			HAL_UART_Transmit(&huart1, &service_byte, 1,10);
			send_float(sizeof(read_t),&read_t,&huart1);
			send_float(sizeof(read_h),&read_h,&huart1);

		}
		sprintf(str,"%2.1f",read_t);
		sprintf(str1,"%3.2f", read_h);
		ST7735_DrawString(2, 4, str, ST7735_GREEN, 1);					//Отображаем принитые температуру на экране st7735
		ST7735_DrawString(2, 5, str1, ST7735_RED, 1);
		osDelay(100);				// опрашиваем раз в 100 мс
	}
  /* USER CODE END StartTask05 */
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
