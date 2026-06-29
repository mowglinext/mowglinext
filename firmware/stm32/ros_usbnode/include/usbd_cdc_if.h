/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* USER CODE BEGIN EXPORTED_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* Powers of 2 are a good choice so that modulo operations become faster mask operations */
/* If you overwrite the CDC_DataReceivedHandler and always process the data, APP_RX_DATA_SIZE should be set to 0 to save RAM */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  8192
#define CDC_RX_DATA_HANDLED 1
#define CDC_RX_DATA_NOTHANDLED 0

#ifndef USE_USB_FS
// if you are using USB_HS uncomment the following define
// it is here because ST forgot to define it for USB FS
#define USE_USB_FS 1
#endif

// define USB_FS_HS_PERIPHERAL if your device has a 'HS' peripheral but runs in FS mode
//#define USB_FS_HS_PERIPHERAL
#ifdef USB_FS_HS_PERIPHERAL
#undef USE_USB_FS
#undef CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_MAX_PACKET_SIZE CDC_DATA_FS_MAX_PACKET_SIZE
#endif

// define CDC_REENTRANT if you use CDC_Transmit_xx in a reentrant way, i.e. from
// nested interrupts or from both main context and interrupt context
// if defined, interrupts will be disabled while copying data into internal buffer (critical section)
//#define CDC_REENTRANT
#ifdef CDC_REENTRANT
    #define CDC_ENTER_CRITICAL_SECTION()   \
       uint32_t PriMsk;                    \
       PriMsk = __get_PRIMASK();           \
       __set_PRIMASK(1);

    #define CDC_EXIT_CRITICAL_SECTION()    \
       __set_PRIMASK(PriMsk);
#else
    #define CDC_ENTER_CRITICAL_SECTION()
    #define CDC_EXIT_CRITICAL_SECTION()
#endif

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */
#ifdef USE_USB_FS
    _Static_assert(APP_TX_DATA_SIZE >= CDC_DATA_FS_MAX_PACKET_SIZE, "tx buffer should hold at least 1 full usb packet");
#else
    _Static_assert(APP_TX_DATA_SIZE >= CDC_DATA_HS_MAX_PACKET_SIZE, "tx buffer should hold at least 1 full usb packet");
#endif
/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
#ifdef USE_USB_FS
    extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
#else
    extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
#endif


/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit(const void* Buf, uint32_t Len);
uint8_t CDC_TransmitTimed(const void* Buf, uint32_t Len, uint32_t TimeoutMs);

void CDC_ResumeTransmit(void);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */
uint8_t CDC_IsBusy();
uint32_t CDC_RXQueue_Dequeue(void* Dst, uint32_t MaxLen);
uint32_t CDC_TXQueue_GetReadAvailable();
uint32_t CDC_TXQueue_GetWriteAvailable();
uint32_t CDC_RXQueue_GetReadAvailable();
uint32_t CDC_RXQueue_GetWriteAvailable();
uint32_t CDC_GetDroppedTxPackets();
uint32_t CDC_GetDroppedRxPackets();
void CDC_ResetDroppedTxPackets();
void CDC_ResetDroppedRxPackets();
uint8_t CDC_DataReceivedHandler(const uint8_t *Data, uint32_t len);
uint32_t CDC_GetLastTransmitStartTick();
uint32_t CDC_GetLastTransmitCompleteTick();
uint8_t CDC_IsComportOpen();

/**
 * @brief  CDC_TransmitString
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *
 *
 * @param  string: 0-terminated C-string to send
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
static inline uint8_t CDC_TransmitString(const char *string)
{
    return CDC_Transmit(string, strlen(string));
}

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

