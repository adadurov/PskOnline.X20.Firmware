/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "max30102.h"
#include "debug.h"
#include "uuid.h"
#include "ring_buffer.h"
#include "i2c_erratum.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define TR_BUF_SAMPLES              64
#define TR_BUF_SAMPLE_T             uint32_t

#define RING_BUFFER_SAMPLES         1024

typedef struct {
    // номер пакета после получения команды старт (32-бит)			4
    uint32_t package_number;

	// количество переполнений кругового буфера (32 бит)			4
    uint32_t ring_buffer_overflows;

	// количество измерений в круговом буфере (16 бит)				2
    uint16_t ring_buffer_data_count;

	// количество следующих дальше 32-битных сэмплов (16 бит)		2
	uint8_t num_samples;

	// flexible array of samples
	TR_BUF_SAMPLE_T samples[];
} usb_package;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void ExecutePendingCommands(ring_buffer *buffer);

void Physio_Start();

void Physio_Stop();

void Physio_UseRamp();

void Physio_UsePpg();


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char serialNumber[20];

WAVEFORM_SENSOR_STATE sensorState;

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
  uint16_t usb_package_size = sizeof(usb_package) + TR_BUF_SAMPLES * sizeof(TR_BUF_SAMPLE_T) + 16;

  sensorState.bitsPerSample = 24;
  sensorState.samplingRate = 800;
  sensorState.physioTransferSize = usb_package_size;

  sensorState.started = 0;
  sensorState.usingPpg = 1;
  sensorState.startFlipped = 0;
  sensorState.stopFlipped = 0;
  sensorState.stopTicks = 0;
  sensorState.startTicks = 0;

  sensorState.Start = &Physio_Start;
  sensorState.Stop = &Physio_Stop;
  sensorState.UsePpg = &Physio_UsePpg;
  sensorState.UseRamp = &Physio_UseRamp;

  CDC_SetSensorInterface(&sensorState);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  debug_write_init(&huart2);
  debug_write_newline(); debug_write_newline(); debug_write_newline();
  debug_write_string("\r\nPSK-X20 Initializing.................");
  debug_write_newline();

  debug_write_string("  Built on:  ");
  debug_write_string(__DATE__ " " __TIME__);
  debug_write_newline();

  ring_buffer *pRingBuf = ring_buffer_alloc(RING_BUFFER_SAMPLES);
  if (0 == pRingBuf)
  {
    debug_write_string(".................failed to allocate ring buffer for ");
    debug_write_int(RING_BUFFER_SAMPLES);
    debug_write_newline();
  }
  usb_package *transmit_buffer = malloc(usb_package_size);
  if (0 == transmit_buffer)
  {
    debug_write_string(".................failed to allocate transmit_buffer of ");
    debug_write_int(usb_package_size);
    debug_write_newline();
  }

  // align to 4 bytes
  transmit_buffer = (usb_package *)(((long long int)transmit_buffer) & 0xFFFFFFFFC);
  transmit_buffer->package_number = 0;

  HAL_StatusTypeDef max30102_status = MAX30102_Init(&hi2c2);

  if (HAL_OK == max30102_status)
  {
	  uint8_t partId = MAX30102_GetPartId(&hi2c2);
	  debug_write_string("  MAX30102 part_id: ");
	  debug_write_int(partId);
	  debug_write_newline();
  }
  else
  {
	  debug_write_string("    Failed to initialize MAX30102 chip.");
  }

  // needs a buffer of at least 15 bytes
  get_uid_str(serialNumber);
  debug_write_string("  STM32 UUID: ");
  debug_write_string(serialNumber);
  debug_write_newline();

  debug_write_string("\r\nPSK-X20 Initialized, ready to rock!\r\n");

  HAL_TIM_Base_Start_IT(&htim4);

  uint8_t sample[6];
  int16_t availableSamples;
  uint32_t ramp = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	ExecutePendingCommands(pRingBuf);

	  	if (!sensorState.started)
  	  	{
	  		continue;
  	  	}

	    /* USER CODE BEGIN 3 */
	      availableSamples = MAX30102_GetNumSamplesInFifo(&hi2c2) - 1;

	      for (int16_t i = 0; i < availableSamples; ++i)
	      {
	          MAX30102_ReadFifo(&hi2c2, sample, 6);
	          uint32_t IR = (sample[3] << 16) + (sample[4] << 8) + sample[5];

	          if (sensorState.usingPpg)
	          {
//                  debug_write_string(" IR: ");
//                  debug_write_int(IR);
//                  debug_write_newline();
	        	  ring_buffer_add_sample(pRingBuf, IR);
	          }
	          else
	          {
//                  debug_write_string(" RAMP: ");
//                  debug_write_int(ramp);
//                  debug_write_newline();
	        	  ring_buffer_add_sample(pRingBuf, ++ramp);
	          }

	          // put the value to our circular buffer for transmitting via USB
	      }

	      if( ! CDC_FreeToTransmit() ) continue;

		  uint16_t num_samples = TR_BUF_SAMPLES;
		  uint16_t ring_buffer_samples = ring_buffer_get_count(pRingBuf);
	      if (ring_buffer_samples >= num_samples)
	      {
	    	  // copy samples from the ring buffer to the transmit buffer
	    	  for( uint16_t i = 0; i < num_samples; ++i)
	    	  {
	    		  transmit_buffer->samples[i] = ring_buffer_remove_sample(pRingBuf);
	    	  }
	          transmit_buffer->package_number++;
	          transmit_buffer->ring_buffer_data_count = ring_buffer_samples;
	          transmit_buffer->num_samples = num_samples;
	          transmit_buffer->ring_buffer_overflows = 0;

	          uint16_t len = sizeof(usb_package) + transmit_buffer->num_samples * sizeof(TR_BUF_SAMPLE_T);

	          int result = -1;
// transfer the package to the USB Host
	          result = CDC_Transmit_FS((uint8_t*)transmit_buffer, len);

	          debug_write_string("    => PP");
//	          debug_write_int(transmit_buffer->package_number);
//	          debug_write_string(" ");
//	          debug_write_int(result);
	          debug_write_newline();
	      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 250;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2003200; //921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM4 )
	{
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
	}
}

void Physio_Start()
{
  sensorState.startFlipped = 1;
  sensorState.startTicks = HAL_GetTick();
}

void ExecutePendingCommands(ring_buffer *buffer)
{
	if (sensorState.startFlipped && sensorState.stopFlipped)
	{
		int startPriority = sensorState.startTicks > sensorState.stopTicks;
		if (startPriority)
		{
			ring_buffer_free(buffer);
			sensorState.started = 1;
		}
		else
		{
			sensorState.started = 0;
		}
		sensorState.startFlipped = 0;
		sensorState.stopFlipped = 0;
		sensorState.stopTicks = 0;
		sensorState.startTicks = 0;
	}
}

void Physio_Stop()
{
  sensorState.stopFlipped = 1;
  sensorState.stopTicks = HAL_GetTick();
}

void Physio_UseRamp()
{
  sensorState.usingPpg = 0;
}

void Physio_UsePpg()
{
  sensorState.usingPpg = 1;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* USER CODE END Error_Handler_Debug */
  debug_write_string("FATAL ERROR OCCURRED! BOARD HALTED.");
  debug_write_string("==============================================================================");
  debug_write_newline();
  __set_BASEPRI(1 << 4);
  while(1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
