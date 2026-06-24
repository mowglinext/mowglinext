/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v1.0_Cube
 * @brief          : Usb device for Virtual Com Port.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include <stdatomic.h>
#include <signal.h>

/* USER CODE BEGIN INCLUDE */

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
#if APP_RX_DATA_SIZE == 0
#undef APP_RX_DATA_SIZE
#define APP_RX_DATA_SIZE 2
#endif
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
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

static uint32_t s_txhead = 0;
static uint32_t s_txtail = 0;
static uint32_t s_rxhead = 0;
static uint32_t s_rxtail = 0;

static uint32_t s_lastTransmitStart = 0;
static uint32_t s_lastTransmitComplete = 0;
static uint32_t s_txBusyCounter = 0;
static uint32_t s_txQueueHighWatermark = 0;
static uint32_t s_rxDropCounterHead = 0;
static uint32_t s_rxDropCounterTail = 0;
static uint32_t s_txDropCounterHead = 0;
static uint32_t s_txDropCounterTail = 0;

#ifdef USE_USB_FS
    static uint8_t ReceiveBuffer[CDC_DATA_FS_MAX_PACKET_SIZE];
#else
    static uint8_t ReceiveBuffer[CDC_DATA_HS_MAX_PACKET_SIZE];
#endif


static USBD_CDC_LineCodingTypeDef s_linecoding = {
    115200,       /* baud rate*/
    0x00,         /* stop bits-1*/
    0x00,         /* parity - none*/
    0x08          /* nb. of bits 8*/
};
/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

#ifdef USE_USB_FS
    extern USBD_HandleTypeDef hUsbDeviceFS;
    #define hUsbDevice hUsbDeviceFS
    #define CDC_DATA_MAX_PACKET_SIZE CDC_DATA_FS_MAX_PACKET_SIZE
#else
    extern USBD_HandleTypeDef hUsbDeviceHS;
    #define hUsbDevice hUsbDeviceHS
    #define CDC_DATA_MAX_PACKET_SIZE CDC_DATA_HS_MAX_PACKET_SIZE
#endif

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CDC_Init(void);
static int8_t CDC_DeInit(void);
static int8_t CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static uint8_t CDC_RXQueue_Enqueue(const uint8_t *buffer, uint32_t length);
static uint8_t CDC_TXQueue_Enqueue(const uint8_t *buffer, uint32_t length);
static const uint8_t* CDC_TXQueue_Dequeue(uint32_t *length);
//static void CDC_ResumeTransmit(void);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

#ifdef USE_USB_FS
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive,
    CDC_TransmitCplt
};
#else
USBD_CDC_ItfTypeDef USBD_Interface_fops_HS = {
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive,
    CDC_TransmitCplt
};
#endif

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the CDC media low layer over the FS USB IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init(void)
{
    /* USER CODE BEGIN 3 */
    /* Set Application Buffers */
    USBD_CDC_SetTxBuffer(&hUsbDevice, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDevice, ReceiveBuffer);
    return (USBD_OK);
    /* USER CODE END 3 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit(void)
{
    /* USER CODE BEGIN 4 */
    return (USBD_OK);
    /* USER CODE END 4 */
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    UNUSED(length);

    /* USER CODE BEGIN 5 */
    switch (cmd) {
    case CDC_SEND_ENCAPSULATED_COMMAND:

        break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

        break;

    case CDC_SET_COMM_FEATURE:

        break;

    case CDC_GET_COMM_FEATURE:

        break;

    case CDC_CLEAR_COMM_FEATURE:

        break;

        /*******************************************************************************/
        /* Line Coding Structure                                                       */
        /*-----------------------------------------------------------------------------*/
        /* Offset | Field       | Size | Value  | Description                          */
        /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
        /* 4      | bCharFormat |   1  | Number | Stop bits                            */
        /*                                        0 - 1 Stop bit                       */
        /*                                        1 - 1.5 Stop bits                    */
        /*                                        2 - 2 Stop bits                      */
        /* 5      | bParityType |  1   | Number | Parity                               */
        /*                                        0 - None                             */
        /*                                        1 - Odd                              */
        /*                                        2 - Even                             */
        /*                                        3 - Mark                             */
        /*                                        4 - Space                            */
        /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
        /*******************************************************************************/
    case CDC_SET_LINE_CODING:
        s_linecoding.bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
        s_linecoding.format = pbuf[4];
        s_linecoding.paritytype = pbuf[5];
        s_linecoding.datatype = pbuf[6];
        break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t) (s_linecoding.bitrate);
        pbuf[1] = (uint8_t) (s_linecoding.bitrate >> 8);
        pbuf[2] = (uint8_t) (s_linecoding.bitrate >> 16);
        pbuf[3] = (uint8_t) (s_linecoding.bitrate >> 24);
        pbuf[4] = s_linecoding.format;
        pbuf[5] = s_linecoding.paritytype;
        pbuf[6] = s_linecoding.datatype;
        break;

    case CDC_SET_CONTROL_LINE_STATE:

        break;

    case CDC_SEND_BREAK:

        break;

    default:
        break;
    }

    return (USBD_OK);
    /* USER CODE END 5 */
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will issue a NAK packet on any OUT packet received on
 *         USB endpoint until exiting this function. If you exit this function
 *         before transfer is complete on CDC interface (ie. using DMA controller)
 *         it will result in receiving more data while previous ones are still
 *         not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Receive(uint8_t *Buf, uint32_t *Len)
{
    /* USER CODE BEGIN 6 */
    uint8_t dataHandled = CDC_DataReceivedHandler(Buf, *Len);

    if (dataHandled == CDC_RX_DATA_NOTHANDLED) {
        if (CDC_RXQueue_Enqueue(Buf, *Len) != USBD_OK) {
            atomic_signal_fence(memory_order_acquire);
            s_rxDropCounterHead++;
            atomic_signal_fence(memory_order_release);
        }
    }

    USBD_CDC_ReceivePacket(&hUsbDevice);
    return (USBD_OK);
    /* USER CODE END 6 */
}

/**
 * @brief  CDC_Transmit
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note see respective header for interrupt safety and reentrancy explanation
 *
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit(const void *Buf, uint32_t Len)
{
    CDC_ENTER_CRITICAL_SECTION();
    uint8_t result = USBD_OK;
    /* USER CODE BEGIN 7 */

    s_lastTransmitStart = HAL_GetTick();
    result = CDC_TXQueue_Enqueue((const uint8_t*)Buf, Len);

    if (result != USBD_OK) {
        s_txDropCounterHead++;
        /* Keep the drop counter, but do not print every overflow over the
         * debug UART: when the host disconnects or the CDC TX queue backs up,
         * this path can trigger at IMU rate and turn a transient USB stall into
         * enough debug-UART backpressure to starve the main loop watchdog. */
    }

    CDC_ResumeTransmit();

    CDC_EXIT_CRITICAL_SECTION();
    /* USER CODE END 7 */
    return result;
}

/**
 * @brief  CDC_TransmitTimed
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         Unlike CDC_Transmit this function can also send data with lengths greater than APP_TX_DATA_SIZE
 *         @note see respective header for interrupt safety and reentrancy explanation
 *         @note if reentrency is enabled, interrupts are disabled for up to the provided timeout!
 *         @note due to the nature of the timeout, it is possible that only the first part of the data is enqueued/sent
 *
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @param  TimeoutMs: maximum time to wait in ms to enqueue the data
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_TransmitTimed(const void* Buf, uint32_t Len, uint32_t TimeoutMs)
{
    if (TimeoutMs == 0) {
        return CDC_Transmit(Buf, Len);
    }

    CDC_ENTER_CRITICAL_SECTION();

    uint32_t start = HAL_GetTick();
    uint8_t result = USBD_OK;
    while (Len > 0) {
        if (HAL_GetTick() - start > TimeoutMs) {
            result = USBD_BUSY;
            break;
        }

        uint32_t enqueueSize = MIN(Len, APP_TX_DATA_SIZE);
        result = CDC_Transmit(Buf, enqueueSize);
        if (result == USBD_OK) {
            Len -= enqueueSize;
            Buf += Len;
        }

    }

    CDC_EXIT_CRITICAL_SECTION();

    return result;
}

/**
 * @brief  CDC_TransmitCplt_FS
 *         Data transmited callback
 *
 *         @note
 *         This function is IN transfer complete callback used to inform user that
 *         the submitted Data is successfully sent over USB.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    uint8_t result = USBD_OK;
    /* USER CODE BEGIN 13 */
    UNUSED(Buf);
    UNUSED(epnum);

    atomic_signal_fence(memory_order_acquire);
    s_txtail += *Len;
    s_lastTransmitComplete = HAL_GetTick();
    atomic_signal_fence(memory_order_release);

    CDC_ResumeTransmit();

    /* USER CODE END 13 */
    return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @brief  CDC_ResumeTransmit
 *         Resume transmission by dequeing data from transmission queue if possible and if usb is not busy
 *
 */
void CDC_ResumeTransmit(void)
{
    if (CDC_IsBusy()) {
        s_txBusyCounter++;
        return;
    }

    uint32_t queueLength;
    const uint8_t *queueData = CDC_TXQueue_Dequeue(&queueLength);
    if (queueLength > 0) {
        USBD_CDC_SetTxBuffer(&hUsbDevice, (uint8_t*) queueData, queueLength);
        USBD_CDC_TransmitPacket(&hUsbDevice);
    }
}

/**
 * @brief  CDC_TXQueue_GetReadAvailable
 *         Check how many bytes are waiting for sending in the transmission queue
 *
 *         @note this is usually not of concern for the user
 *
 *
 * @retval number of enqueued bytes in the transmission queue
 */
uint32_t CDC_TXQueue_GetReadAvailable()
{
    atomic_signal_fence(memory_order_acquire);
    return s_txhead - s_txtail;
}

/**
 * @brief  CDC_TXQueue_GetWriteAvailable
 *         Check how much space is left in the transmission queue
 *
 *
 * @retval number of available bytes in the transmission queue
 */
uint32_t CDC_TXQueue_GetWriteAvailable()
{
    return APP_TX_DATA_SIZE - CDC_TXQueue_GetReadAvailable();
}

/**
 * @brief  CDC_RXQueue_GetReadAvailable
 *         Check how many bytes are enqueued in the reception queue
 *
 *
 * @retval number of available bytes in the reception queue
 */
uint32_t CDC_RXQueue_GetReadAvailable()
{
    atomic_signal_fence(memory_order_acquire);
    return s_rxhead - s_rxtail;
}

/**
 * @brief  CDC_RXQueue_GetWriteAvailable
 *         Check how much space is left in the reception queue
 *
 *         @note this is usually not of concern for the user
 *
 *
 * @retval number of available bytes in the reception queue
 */
uint32_t CDC_RXQueue_GetWriteAvailable()
{
    return APP_RX_DATA_SIZE - CDC_RXQueue_GetReadAvailable();
}

/**
 * @brief  CDC_TXQueue_Enqueue
 *         Enqueue data into the transmission queue
 *
 * @param  buffer: data to enqueue (must not be null)
 * @param  length: length of data
 * @retval USBD_OK if ok USBD_BUSY otherwise (queue full)
 */
uint8_t CDC_TXQueue_Enqueue(const uint8_t *buffer, uint32_t length)
{
    if (length > CDC_TXQueue_GetWriteAvailable()) {
        return USBD_BUSY;
    }

    atomic_signal_fence(memory_order_acquire);
    uint32_t head = s_txhead;
    uint32_t sizeTillWrapAround = APP_TX_DATA_SIZE - (head % APP_TX_DATA_SIZE);
    uint32_t firstLength = MIN(length, sizeTillWrapAround);
    uint32_t secondLength = length - firstLength;

    // first part
    uint32_t insertIndex = head % APP_TX_DATA_SIZE;
    memcpy(UserTxBufferFS + insertIndex, buffer, firstLength);
    head += firstLength;

    // second part after wrap around
    memcpy(UserTxBufferFS, buffer + firstLength, secondLength);
    head += secondLength;

    atomic_signal_fence(memory_order_acquire);
    s_txhead = head;
    if ((s_txhead - s_txtail) > s_txQueueHighWatermark) {
        s_txQueueHighWatermark = s_txhead - s_txtail;
    }
    atomic_signal_fence(memory_order_release);

    return USBD_OK;
}

/**
 * @brief  CDC_RXQueue_Enqueue
 *         Enqueue data into the reception queue
 *
 * @param  buffer: data to enqueue (must not be null)
 * @param  length: length of data
 * @retval USBD_OK if ok USBD_BUSY otherwise (queue full)
 */
uint8_t CDC_RXQueue_Enqueue(const uint8_t *buffer, uint32_t length)
{
    if (length > CDC_RXQueue_GetWriteAvailable()) {
        return USBD_BUSY;
    }

    atomic_signal_fence(memory_order_acquire);
    uint32_t head = s_rxhead;
    uint32_t sizeTillWrapAround = APP_RX_DATA_SIZE - (head % APP_RX_DATA_SIZE);
    uint32_t firstLength = MIN(length, sizeTillWrapAround);
    uint32_t secondLength = length - firstLength;

    // first part
    uint32_t insertIndex = head % APP_RX_DATA_SIZE;
    memcpy(UserRxBufferFS + insertIndex, buffer, firstLength);
    head += firstLength;

    // second part after wrap around
    memcpy(UserRxBufferFS, buffer + firstLength, secondLength);
    head += secondLength;

    atomic_signal_fence(memory_order_acquire);
    s_rxhead = head;
    atomic_signal_fence(memory_order_release);

    return USBD_OK;
}

/**
 * @brief  CDC_TXQueue_Dequeue
 *         Dequeue data from the transmission queue
 *
 *         @note
 *         This is intended to be called from the transmission complete interrupt
 *         The next transmission complete interrupt _must_ increase s_txtail accordingly to actually free up the
 *         dequeued data space
 *
 * @param  length: pointer where the length of dequeued data is written to (must not be null)
 * @retval buffer to dequeud data
 */
const uint8_t* CDC_TXQueue_Dequeue(uint32_t *length)
{
    uint32_t queueSize = CDC_TXQueue_GetReadAvailable();
    if (queueSize == 0) {
        *length = 0;
        return NULL;
    }

    atomic_signal_fence(memory_order_acquire);

    // length is capped so that we get no buffer wrap around
    // this reduces complexity and reduces memory requirements, but decreases throughput
    // length is also capped to 4096 due to ST internals
    uint32_t tail = s_txtail;
    uint32_t sizeTillWrapAround = APP_TX_DATA_SIZE - (tail % APP_TX_DATA_SIZE);
    uint32_t dequeueLength = MIN(MIN(queueSize, sizeTillWrapAround), 4096);

    // this is a small optimization: if we send a packet multiple of 64 bytes (512 bytes for HS)
    // the next USB transfer slot is wasted with a ZLP, so instead send 1 byte in next slot
    // or possibly even more, if new data got enqueued
    // this increases throughput at the cost of latency
    if (dequeueLength % CDC_DATA_MAX_PACKET_SIZE == 0) {
        dequeueLength--;
    }

    *length = dequeueLength;
    const uint8_t *dequeueData = UserTxBufferFS + (tail % APP_TX_DATA_SIZE);

    return dequeueData;
}

/**
 * @brief  CDC_RXQueue_Dequeue
 *         Dequeue all data from the reception queue, but at most MaxLen bytes
 *         @note this is not reentrant safe, i.e. do not call this from both normal and interrupt context
 *
 *
 * @param  Dst: Destination buffer
 * @param  MaxLen: Maximum number of bytes to read
 * @retval Number of bytes dequeued. The value is 0 if no bytes are available
 */
uint32_t CDC_RXQueue_Dequeue(void *Dst, uint32_t MaxLen)
{
    uint32_t queueSize = CDC_RXQueue_GetReadAvailable();
    if (queueSize == 0 || MaxLen == 0) {
        return 0;
    }

    atomic_signal_fence(memory_order_acquire);
    uint32_t tail = s_rxtail;
    uint32_t dequeueLength = MIN(queueSize, MaxLen);
    uint32_t sizeTillWrapAround = APP_RX_DATA_SIZE - (tail % APP_RX_DATA_SIZE);
    uint32_t firstLength = MIN(dequeueLength, sizeTillWrapAround);
    uint32_t secondLength = dequeueLength - firstLength;

    // first part
    const uint8_t *dequeueData = UserRxBufferFS + (tail % APP_RX_DATA_SIZE);
    memcpy(Dst, dequeueData, firstLength);
    tail += firstLength;

    // second part after wrap around
    dequeueData = UserRxBufferFS;
    char * dstPtr = ((char*)Dst) + firstLength;
    memcpy(dstPtr, dequeueData, secondLength);
    tail += secondLength;

    atomic_signal_fence(memory_order_acquire);
    s_rxtail = tail;
    atomic_signal_fence(memory_order_release);

    return dequeueLength;
}

/**
 * @brief  CDC_IsBusy
 *         Check if the CDC is busy transmitting data
 *         @note If CDC is busy, one can still enqueue data
 *
 * @retval 1 if busy transmitting data
 */
uint8_t CDC_IsBusy()
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDevice.pClassData;
    
    return hcdc->TxState != 0;
}

/**
 * @brief  CDC_GetDroppedTxPackets
 *         Get the number of dropped packets which could not be sent (and also not enqueued),
 *         because the TX buffer did not have enough space left
 *
 * @retval number of dropped packets
 */
uint32_t CDC_GetDroppedTxPackets()
{
    atomic_signal_fence(memory_order_acquire);
    return s_txDropCounterHead - s_txDropCounterTail;
}

uint32_t CDC_GetBusyCount()
{
    atomic_signal_fence(memory_order_acquire);
    return s_txBusyCounter;
}

/**
 * @brief  CDC_GetDroppedRxPackets
 *         Get the number of dropped received packets (neither handled nor enqueued)
 *
 * @retval number of dropped packets
 */
uint32_t CDC_GetDroppedRxPackets()
{
    atomic_signal_fence(memory_order_acquire);
    return s_rxDropCounterHead - s_rxDropCounterTail;
}

uint32_t CDC_GetTxQueueHighWatermark()
{
    atomic_signal_fence(memory_order_acquire);
    return s_txQueueHighWatermark;
}

/**
 * @brief  CDC_ResetDroppedTxPackets
 *         Reset the dropped packed counter for the transmission
 *
 * @retval
 */
void CDC_ResetDroppedTxPackets()
{
    atomic_signal_fence(memory_order_acquire);
    s_txDropCounterTail = s_txDropCounterHead;
    atomic_signal_fence(memory_order_release);
}

/**
 * @brief  CDC_ResetDroppedRxPackets
 *         Reset the dropped packed counter for the reception
 *
 * @retval
 */
void CDC_ResetDroppedRxPackets()
{
    atomic_signal_fence(memory_order_acquire);
    s_rxDropCounterTail = s_rxDropCounterHead;
    atomic_signal_fence(memory_order_release);
}

/**
 * @brief  CDC_GetLastTransmitStartTick
 *         Get the timestamp of when the last packet's _attempted_ transmission
 *         was started
 *
 *
 * @retval timestamp in HAL ticks
 */
uint32_t CDC_GetLastTransmitStartTick()
{
    return s_lastTransmitStart;
}

/**
 * @brief  CDC_GetLastTransmitCompleteTick
 *         Get the timestamp of when the last packet was successfully sent to the
 *         connected computer
 *
 * @retval timestamp in HAL ticks
 */
uint32_t CDC_GetLastTransmitCompleteTick()
{
    atomic_signal_fence(memory_order_acquire);
    return s_lastTransmitComplete;
}

/**
 * @brief  CDC_IsComportOpen
 *         Check if the connectd computer is reading data through the virtual com port
 *
 *         @note This functions relies on the start and complete timestamp of the last sent packet,
 *         its information is as recent as the last sent packet (or attempted to send packet)
 *         This also means you cannot wait for CDC_IsComportOpen to return true and then start
 *         sending data.
 *
 *
 * @retval 1 if the com port is open on the computer side and the computer is reading data
 */
uint8_t CDC_IsComportOpen()
{
    atomic_signal_fence(memory_order_acquire);
    return s_lastTransmitStart != 0 && s_lastTransmitStart - s_lastTransmitComplete < 500;
}

/**
 * @brief  CDC_DataReceivedHandler
 *         Called when data is received via USB CDC
 *         This can be overwritten via user provided function
 *         @note this is called from USB interrupt context
 *
 *
 * @param  Buf: data
 * @param  len: number of bytes received (<= 64 for USB FS, <= 512 for USB HS)
 * @retval CDC_RX_DATA_HANDLED if the data is handled and should not be placed in the receive queue
 */
__weak uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len)
{
    UNUSED(Buf);
    UNUSED(len);
    return CDC_RX_DATA_NOTHANDLED;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

