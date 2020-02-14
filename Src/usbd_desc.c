/**
  ******************************************************************************
  * @file           : usbd_desc.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB device descriptors.
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
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* USER CODE BEGIN INCLUDE */
#include "winusb.h"
#include "winusb_defs.h"
#include "debug.h"
#include "revision.h"
#include "psk_x20.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @addtogroup USBD_DESC
  * @{
  */

/** @defgroup USBD_DESC_Private_TypesDefinitions USBD_DESC_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Defines USBD_DESC_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

extern char serialNumber[20];

#define USBD_VID                        0x1209
#define USBD_LANGID_STRING              1033
#define USBD_PID_FS                     0x5252

// Product & manufacturer strings come from the 'usbd_desc.h'
// This way it may be reused somewhere else
//#define USBD_MANUFACTURER_STRING        "https://www.psk-online.ru"
//#define USBD_PRODUCT_STRING_FS          "PskOnline PSK-X20 PPG Waveform Sensor"
#define USBD_CONFIGURATION_STRING_FS    USBD_PRODUCT_STRING_FS
#define USBD_INTERFACE_STRING_FS        USBD_PRODUCT_STRING_FS

//#define USBD_SERIALNUMBER_STRING_FS     "00000000001A"
#define USBD_SERIALNUMBER_STRING_FS     serialNumber

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/** @defgroup USBD_DESC_Private_Macros USBD_DESC_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_FunctionPrototypes USBD_DESC_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_OsDescriptor(USBD_SpeedTypeDef speed, USBD_SetupReqTypedef *req, uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t * USBD_FS_USRStringDesc(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif /* USB_SUPPORT_USER_STRING_DESC */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Variables USBD_DESC_Private_Variables
  * @brief Private variables.
  * @{
  */

USBD_DescriptorsTypeDef FS_Desc =
{
  .GetDeviceDescriptor = USBD_FS_DeviceDescriptor
, .GetLangIDStrDescriptor = USBD_FS_LangIDStrDescriptor
, .GetManufacturerStrDescriptor = USBD_FS_ManufacturerStrDescriptor
, .GetProductStrDescriptor = USBD_FS_ProductStrDescriptor
, .GetSerialStrDescriptor = USBD_FS_SerialStrDescriptor
, .GetConfigurationStrDescriptor = USBD_FS_ConfigStrDescriptor
, .GetInterfaceStrDescriptor = USBD_FS_InterfaceStrDescriptor
, .GetOsDescriptor = USBD_FS_OsDescriptor
};

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
/** USB standard device descriptor. */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 1.00 -- hardware revision*/
  0x01,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

/* USB_DeviceDescriptor */

/* BEGIN --> OS Descriptors */

const struct winusb_compatible_id_descriptor winusb_wcid = {
    .dwLength = (WINUSB_COMPATIBLE_ID_HEADER_SIZE +
                 1*WINUSB_COMPATIBLE_ID_FUNCTION_SECTION_SIZE),
    .bcdVersion = 0x0100,
    .wIndex = 0x0004,
    .bNumSections = 1,
    .reserved = { 0, 0, 0, 0, 0, 0, 0 },
    .functions = {
        {
            .bInterfaceNumber = 0,
            .reserved0 = { 1 },
            .compatibleId = "WINUSB",
            .subCompatibleId = "",
            .reserved1 = { 0, 0, 0, 0, 0, 0}
        }
    }
};

const struct winusb_extended_property_device_interface_guid_descriptor winusb_device_interface = {
    .dwLength = 146, //sizeof(winusb_device_interface), // should be 146, including    // 4
    .bcdVersion = 0x0100,                                                       // 2
    .wIndex = 0x0005,                                                           // 2
    .wCount = 1,                                                                // 2
    .dwSize = 136,                                  // should be 136, including // 4
    .dwPropertyDataType = WINUSB_PROP_DATA_TYPE_REG_REG_MULTI_SZ,               // 4
    .wPropertyNameLength = 42,                                                  // 2
    .bPropertyName = {
    		'D','e','v','i','c','e','I','n',
			't','e','r','f','a','c','e','G',
			'U','I','D','s', 0
    },                                                                          // 42 (21 16-bit chars)
    .dwPropertyDataLength = 80,                                                 // 4
    .bPropertyData = {
    		'{','C','C','7','4','5','8','7',
			'9','-','9','1','C','7','-','4',
			'E','8','B','-','8','F','6','6',
			'-','0','F','A','6','9','7','4',
			'8','9','0','9','B','}', 0,  0 }                                    // 80 (40 16-bit chars)
};


/* END --> OS Descriptors */


/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Variables USBD_DESC_Private_Variables
  * @brief Private variables.
  * @{
  */

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */

/** USB lang indentifier descriptor. */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
/* Internal string descriptor. */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Functions USBD_DESC_Private_Functions
  * @brief Private functions.
  * @{
  */

/**
  * @brief  Return the device descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}


/**
  * @brief  Processes the request for the OS descriptor and returns the appropriate descriptor
  * @param  speed : Current device speed
  * @param  req : The setup packet containing the descriptor request
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_OsDescriptor(USBD_SpeedTypeDef speed, USBD_SetupReqTypedef *req, uint16_t *length)
{
  switch (req->wIndex)
  {
    case WINUSB_REQ_GET_COMPATIBLE_ID_FEATURE_DESCRIPTOR:
      debug_write_string("  ");
      debug_write_string("WINUSB_REQ_GET_COMPATIBLE_ID_FEATURE_DESCRIPTOR");
      debug_write_newline();
      *length = winusb_wcid.dwLength;
      return (uint8_t*)&winusb_wcid;

    case WINUSB_REQ_GET_EXTENDED_PROPERTIES_OS_FEATURE_DESCRIPTOR:
        // Notice that this control request is sent to 'interface'!
        debug_write_string("  ");
        debug_write_string("WINUSB_REQ_GET_EXTENDED_PROPERTIES_OS_FEATURE_DESCRIPTOR");
        debug_write_newline();
        *length = winusb_device_interface.dwLength;
        return (uint8_t*)&winusb_device_interface;

    default:
        *length = 0;
        break;
  }
  return 0;
}


/**
  * @brief  Return the LangID string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  debug_write_string("GET_LangIdStr"); debug_write_newline();

  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
  * @brief  Return the product string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the manufacturer string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
  * @brief  Return the serial number string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the configuration string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the interface string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}


/**
 * @brief
  * @param pdev:device instance
  * @param index: index of the string descriptor (only the single language is supported here)
  * @param length: pointer to the variable containing the size of the descriptor to transfer
  * @retval pointer to descriptor buffer}
  */
uint8_t  *USBD_Sensor_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev ,uint8_t index,  uint16_t *pOutLen)
{
	  debug_write_string("GUSD "); debug_write_int(index);

	  if( MS_OS_DESCRIPTOR_INDEX == index )
	  {
		  // '!' is the request code that will be later sent
		  // by the host to retrieve OS feature descriptors
		  USBD_GetString((uint8_t*)"MSFT100!", USBD_StrDesc, pOutLen);
	  }

	  else if( X20_REVISION_STRING_IDX == index )
	  {
		  USBD_GetString((uint8_t*)REVISION_INFO, USBD_StrDesc, pOutLen);
	  }

	  else if( X20_BUILD_DATE_STRING_IDX == index )
	  {
		  USBD_GetString((uint8_t*)BUILD_DATE, USBD_StrDesc, pOutLen);
	  }
	  else
	  {
		  *pOutLen = 0;
	  }

	  debug_write_string(" => "); debug_write_int(*pOutLen); debug_write_newline();

	  return USBD_StrDesc;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
