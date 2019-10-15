/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

#include "debug.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_Sensor_GetCfgDesc_FS(uint16_t *length);

static uint8_t  *USBD_Sensor_GetCfgDesc_HS(uint16_t *length);

static uint8_t  *USBD_Sensor_GetCfgDesc_Other(uint16_t *length);

uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length);

extern uint8_t *USBD_Sensor_GetUsrStrDescriptor(
        struct _USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length);


/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */ 


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC = 
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,     
  USBD_Sensor_GetCfgDesc_HS,
  USBD_Sensor_GetCfgDesc_FS,
  USBD_Sensor_GetCfgDesc_Other,
  USBD_CDC_GetDeviceQualifierDescriptor,
  USBD_Sensor_GetUsrStrDescriptor
};

__ALIGN_BEGIN static uint8_t USBD_Sensor_CfgDesc[PHYSIO_SENSOR_CONFIG_DESC_SIZE] __ALIGN_END
        =
        { 0x09, /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
		PHYSIO_SENSOR_CONFIG_DESC_SIZE, /* wTotalLength: Bytes returned */
        0x00, 0x01, /*bNumInterfaces: 1 interface*/
        0x01, /*bConfigurationValue: Configuration value*/
        0x00, /*iConfiguration: Index of string descriptor describing the configuration*/
        0xC0, /*bmAttributes: bus powered */
        0x32, /*MaxPower 100 mA: this current is used for detecting Vbus*/

        /************** Descriptor of Physiological interface ****************/
        /* 09 */
        0x09, /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x00, /*bInterfaceNumber: Number of Interface*/
        0x00, /*bAlternateSetting: Alternate setting*/
        0x01, /*bNumEndpoints*/
        0xFF, /*bInterfaceClass: Vendor-specific*/
        0x00, /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x00, /*nInterfaceProtocol : 0=none */
        0, /*iInterface: Index of string descriptor*/
        /******************** Descriptor of Custom HID endpoints ********************/
        /* 18 */
        0x07, /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
        PHYSIO_EPIN_ADDR, /*bEndpointAddress: Endpoint Address (IN)*/
        0x02, /*bmAttributes: Interrupt endpoint*/
		PHYSIO_EPIN_MAX_PACKET_SIZE, /*wMaxPacketSize: 64 Byte max */
        0x00, 0x5, /*bInterval: Polling Interval (5 ms)*/
        /* 25 */
        };

/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  debug_write_string("USBD_CDC_Init"); debug_write_newline();

  uint8_t ret = 0;
  USBD_CDC_HandleTypeDef   *hcdc;
  
  if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
  {  
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
    			   PHYSIO_EPIN_ADDR,
				   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_IN_PACKET_SIZE);
    
//    /* Open EP OUT */
//    USBD_LL_OpenEP(pdev,
//                   CDC_OUT_EP,
//                   USBD_EP_TYPE_BULK,
//                   CDC_DATA_HS_OUT_PACKET_SIZE);
    
  }
  else
  {
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
    			   PHYSIO_EPIN_ADDR,
				   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_IN_PACKET_SIZE);
    
//    /* Open EP OUT */
//    USBD_LL_OpenEP(pdev,
//                   CDC_OUT_EP,
//                   USBD_EP_TYPE_BULK,
//                   CDC_DATA_FS_OUT_PACKET_SIZE);
  }
//  /* Open Command IN EP */
//  USBD_LL_OpenEP(pdev,
//                 CDC_CMD_EP,
//                 USBD_EP_TYPE_INTR,
//                 CDC_CMD_PACKET_SIZE);
  
    
  pdev->pClassData = USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    
    /* Init  physical Interface components */
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();
    
    /* Init Xfer states */
    hcdc->TxState = 0;
//    hcdc->RxState = 0;
       
//    if(pdev->dev_speed == USBD_SPEED_HIGH  )
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_HS_OUT_PACKET_SIZE);
//    }
//    else
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_FS_OUT_PACKET_SIZE);
//    }
    
    
  }
  return ret;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  debug_write_string("USBD_CDC_DeInit"); debug_write_newline();

  uint8_t ret = 0;
  
  /* Close EP IN */
  USBD_LL_CloseEP(pdev, PHYSIO_EPIN_ADDR);
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  debug_write_string("USBD_CDC_Setup "); debug_write_newline();

  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  static uint8_t ifalt = 0;

  uint8_t *dataPtr = hcdc->data;
    
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
	// does the control transfer has a data stage?
    if (req->wLength)
    {
      // yes, there is a data stage?

      if (req->bmRequest & 0x80)
      {
    	debug_write_string("in ");
    	USBD_CDC_ItfTypeDef *interface = ((USBD_CDC_ItfTypeDef *)pdev->pUserData);
        uint8_t status = interface->Control(req,
        									&dataPtr,
                                            req->wLength);

        if (USBD_OK == status)
        {
          memcpy(hcdc->data, dataPtr, req->wLength);
          USBD_CtlSendData(pdev,
        		  	  	   hcdc->data,
                           req->wLength);
          return USBD_OK;
        }
        return USBD_FAIL;

      }
      else
      {
      	debug_write_string("out ");
    	debug_write_string("Unexpected! setup request with OUT data stage");
    	debug_write_newline();
        return USBD_FAIL;
      }
      
    }
    else
    {
      // no, there is no data stage
      debug_write_string("nd ");
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req, 0, 0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {      
    case USB_REQ_GET_INTERFACE :
      debug_write_string("USB_REQ_GET_INTERFACE"); debug_write_newline();
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
    break;
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  debug_write_string("USBD_CDC_DataIn"); debug_write_newline();

  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {

    debug_write_string("USBD_CDC_DataIn => OK"); debug_write_newline();
    hcdc->TxState = 0;

    return USBD_OK;
  }
  else
  {
    debug_write_string("USBD_CDC_DataIn => FAIL"); debug_write_newline();
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  debug_write_string("Unexpected! USBD_CDC_DataOut"); debug_write_newline();

  /* Get the received data length */
//  hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
//    ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Receive(hcdc->RxBuffer, &hcdc->RxLength);
//
//    return USBD_OK;
    return USBD_FAIL;
  }
  else
  {
    return USBD_FAIL;
  }
}



/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
//  if((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFF))
//  {
  	debug_write_string("Unexpected! in USBD_CDC_EP0_RxReady"); debug_write_newline();
//
//  	((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(hcdc->CmdOpCode,
//                                                      (uint8_t *)hcdc->data,
//                                                      hcdc->CmdLength);
//      hcdc->CmdOpCode = 0xFF;
      
//  }
  return USBD_OK;
}

/**
 * @brief  USBD_Sensor_GetCfgDesc
 *         return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t *USBD_Sensor_GetCfgDesc_FS(uint16_t *length)
{
	debug_write_string("USBD_Sensor_GetCfgDesc_FS => FULL SPEED"); debug_write_newline();
    *length = sizeof(USBD_Sensor_CfgDesc);
    return USBD_Sensor_CfgDesc;
}

static uint8_t *USBD_Sensor_GetCfgDesc_HS(uint16_t *length)
{
	debug_write_string("USBD_Sensor_GetCfgDesc_HS => HIGH SPEED"); debug_write_newline();
    *length = sizeof(USBD_Sensor_CfgDesc);
    return USBD_Sensor_CfgDesc;
}

static uint8_t *USBD_Sensor_GetCfgDesc_Other(uint16_t *length)
{
	debug_write_string("USBD_Sensor_GetCfgDesc_Other => OTHER SPEED"); debug_write_newline();
    *length = sizeof(USBD_Sensor_CfgDesc);
    return USBD_Sensor_CfgDesc;
}


/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length)
{
  debug_write_string("USBD_CDC_GetDeviceQualifierDescriptor"); debug_write_newline();
  *length = sizeof (USBD_CDC_DeviceQualifierDesc);
  return USBD_CDC_DeviceQualifierDesc;
}

/**
* @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_CDC_ItfTypeDef *fops)
{
  debug_write_string("USBD_CDC_RegisterInterface"); debug_write_newline();

  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData = fops;
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  debug_write_string("USBD_CDC_SetTxBuffer "); debug_write_int(pbuff); debug_write_int(length);debug_write_newline();

  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;  
  
  return USBD_OK;  
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  hcdc->RxBuffer = pbuff;
//  return USBD_OK;
  debug_write_string("Unexpected: USBD_CDC_SetRxBuffer"); debug_write_newline();
  
  return USBD_FAIL;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if(hcdc->TxState == 0)
    {
      /* Tx Transfer in progress */
      hcdc->TxState = 1;
      
      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
    		  	  	   PHYSIO_EPIN_ADDR,
                       hcdc->TxBuffer,
                       hcdc->TxLength);
      
      debug_write_string("  USBD_CDC_TransmitPacket => OK"); debug_write_newline();
      return USBD_OK;
    }
    else
    {
      debug_write_string("  USBD_CDC_TransmitPacket => BUSY"); debug_write_newline();
      return USBD_BUSY;
    }
  }
  else
  {
    debug_write_string("  USBD_CDC_TransmitPacket => FAILED"); debug_write_newline();
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  debug_write_string("Unexpected! in USBD_CDC_ReceivePacket"); debug_write_newline();
  return USBD_FAIL;

//  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
//
//  /* Suspend or Resume USB Out process */
//  if(pdev->pClassData != NULL)
//  {
//    if(pdev->dev_speed == USBD_SPEED_HIGH  )
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_HS_OUT_PACKET_SIZE);
//    }
//    else
//    {
//      /* Prepare Out endpoint to receive next packet */
//      USBD_LL_PrepareReceive(pdev,
//                             CDC_OUT_EP,
//                             hcdc->RxBuffer,
//                             CDC_DATA_FS_OUT_PACKET_SIZE);
//    }
//    return USBD_OK;
//  }
//  else
//  {
//    return USBD_FAIL;
//  }
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
