/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "debug.h"
#include "revision.h"
#include "x20_defines.h"


/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint32_t x20_capabilities_descriptor_buffer[512 / sizeof(uint32_t)];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

static x20_capabilities_descriptor *capabilitiesDescriptor;
static WAVEFORM_SENSOR_STATE *pSensorState;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

static int8_t CDC_Control(
				USBD_SetupReqTypedef *_req,
				uint8_t **responseData,
				uint16_t len);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  debug_write_string("CDC_Init_FS"); debug_write_newline();

  uint8_t revision_len = strlen(REVISION_INFO);
  uint8_t size = sizeof(x20_capabilities_descriptor) + revision_len + 1;
  if(size > sizeof(x20_capabilities_descriptor_buffer))
  {
	  debug_write_string(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
	  debug_write_string("x20_capabilities_descriptor structure is too large!");
	  debug_write_newline();
	  while(1);
  }
  capabilitiesDescriptor = (x20_capabilities_descriptor*)x20_capabilities_descriptor_buffer;

  capabilitiesDescriptor->size = size;
  capabilitiesDescriptor->generation = 0;
  capabilitiesDescriptor->bits_per_sample = pSensorState->bitsPerSample;
  capabilitiesDescriptor->sampling_rate = pSensorState->samplingRate;
  capabilitiesDescriptor->bytes_per_physio_transfer = pSensorState->physioTransferSize;

  // copy build date in safe manner
  char* build_date = BUILD_DATE;
  uint8_t build_date_str_len = strlen(build_date);
  uint8_t build_date_size = sizeof(capabilitiesDescriptor->firmware_build_date);
  strncpy(capabilitiesDescriptor->firmware_build_date,
          build_date,
          build_date_size - 1);
  uint8_t last = build_date_size > build_date_str_len ? build_date_str_len : build_date_size - 1;
  capabilitiesDescriptor->firmware_build_date[last] = 0;

  // copy revision info in safe manner
  strncpy(capabilitiesDescriptor->revision_info,
		  REVISION_INFO,
		  revision_len);
  capabilitiesDescriptor->revision_info[revision_len] = 0;


  debug_write_string("capabilitiesDescriptor ");
  debug_write_int(size);
  debug_write_newline();
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  debug_write_string("CDC_DeInit_FS"); debug_write_newline();

  return (USBD_OK);
  /* USER CODE END 4 */
}

static int8_t CDC_Control(
		USBD_SetupReqTypedef *req,
		uint8_t **responseData,
		uint16_t len)
{
  debug_write_string("CDC_Ctrl ");
  debug_write_int(req->bmRequest);
  debug_write_string(" ");
  debug_write_int(req->bRequest);
  debug_write_newline();

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;

  const uint8_t command_request_type = USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;

  if( command_request_type == (req->bmRequest & command_request_type))
  {
	  debug_write_string(" => X20 Command detected"); debug_write_newline();

	  // we probably received a commands
	  switch(req->bRequest)
	  {
		  case X20_GET_CAPABILITIES_DESCRIPTOR:
			  debug_write_string("   => X20_GET_CAPABILITIES_DESCRIPTOR ");
			  debug_write_int(capabilitiesDescriptor->size);
			  debug_write_newline();
              *responseData = (uint8_t*)capabilitiesDescriptor;
              return USBD_OK;

		  case X20_USE_PPG:
			  debug_write_string("   => X20_USE_PPG"); debug_write_newline();
			  pSensorState->UsePpg();
              return USBD_OK;

		  case X20_USE_RAMP:
			  debug_write_string("   => X20_USE_RAMP"); debug_write_newline();
			  pSensorState->UseRamp();
              return USBD_OK;

		  case X20_START:
			  debug_write_string("   => X20_START"); debug_write_newline();
			  pSensorState->Start();
              return USBD_OK;

		  case X20_STOP:
			  debug_write_string("   => X20_STOP"); debug_write_newline();
			  pSensorState->Stop();
              return USBD_OK;

		  default:
			  return USBD_FAIL;
	  }
  }

  return (USBD_FAIL);
}



/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  debug_write_string("CDC_Transmit_FS ");

  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

uint8_t CDC_FreeToTransmit()
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED){
    return 0;
  }
  if (hcdc->TxState != 0) {
	// busy, not ready to transmit
	return 0;
  }
  return 1;
}

void CDC_SetSensorInterface(WAVEFORM_SENSOR_STATE *state)
{
	pSensorState = state;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
