/****************************************************************************
* Title                 :   
* Filename              :   blademotor.c
* Author                :   Nekraus
* Origin Date           :   17/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file blademotor.c
*  \brief 
*
*/
/******************************************************************************
* Includes
*******************************************************************************/
#include <string.h>
#include <stdbool.h>

#include "stm32f_board_hal.h"

#include "main.h"
#include "board.h"
#include "emergency.h"
#include "actuator_authorization.h"

#include "blademotor.h" 

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define BLADEMOTOR_LENGTH_INIT_MSG 22
#define BLADEMOTOR_LENGTH_RQST_MSG 7
#define BLADEMOTOR_LENGTH_VERSION_MSG 7
/* The RM1000 PAC5223 replies to the version query with a 12-byte frame
 * (length byte 0x08 plus four framing/checksum bytes). This is distinct from
 * the 16-byte steady-state motor telemetry frame. */
#define BLADEMOTOR_LENGTH_VERSION_RESPONSE 12u
/* Reversal requires an accepted OFF transmission and qualifying ESC reports.
 * The tested 500 holds its speed word after OFF, then clears it; it reports no
 * progressive coast-down curve. These guards are not a mechanical stop model. */
#define BLADEMOTOR_REVERSE_OFF_MS 1000u
#define BLADEMOTOR_ZERO_CONFIRM_MS 300u
#define BLADEMOTOR_FEEDBACK_MAX_AGE_MS 300u
#define BLADEMOTOR_REVERSE_REPORT_MS 5000u
#define BLADEMOTOR_RX_TIMEOUT_MS 100u
#ifndef BLADEMOTOR_COASTDOWN_VALIDATION
#define BLADEMOTOR_COASTDOWN_VALIDATION 0
#endif
#if BLADEMOTOR_COASTDOWN_VALIDATION && !BOARD_YARDFORCE500_VARIANT_ORIG
#error "Coast-down validation uses the Yardforce500 UART debug output"
#endif
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef enum {
#if defined(BLADEMOTOR_SEQUENCED_POWER)
    BLADEMOTOR_RESET_HOLD,
    BLADEMOTOR_LOGIC_BOOT,
    BLADEMOTOR_VERSION_WAIT,
    BLADEMOTOR_MATRIX_WAIT,
    BLADEMOTOR_POWER_WAIT,
#endif
    BLADEMOTOR_INIT_1,
    BLADEMOTOR_INIT_2,
    BLADEMOTOR_RUN
}BLADEMOTOR_STATE_e;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
UART_HandleTypeDef BLADEMOTOR_USART_Handler; // UART  Handle

DMA_HandleTypeDef hdma_uart_blade_rx;
DMA_HandleTypeDef hdma_uart_blade_tx;

static BLADEMOTOR_STATE_e blademotor_eState = BLADEMOTOR_INIT_1;

bool BLADEMOTOR_bActivated = false;
uint16_t BLADEMOTOR_u16RPM = 0;
uint16_t BLADEMOTOR_u16Power = 0;
uint32_t BLADEMOTOR_u32Error = 0;

static uint8_t blademotor_pu8ReceivedData[BLADEMOTOR_LENGTH_RECEIVED_MSG] = {0};
static uint8_t blademotor_pu8RqstMessage[BLADEMOTOR_LENGTH_RQST_MSG]  = {0x55, 0xaa, 0x03, 0x20, 0x80, 0x00, 0xA2};
static uint8_t blademotor_u8OnOff = 0;
static uint8_t blademotor_u8Direction = 0;
static uint8_t blademotor_u8RunDirection = 0;
static uint32_t blademotor_u32OnAuthorizationEpoch = 0u;
#if defined(BLADEMOTOR_SEQUENCED_POWER)
static uint32_t blademotor_state_started_tick = 0u;
static bool blademotor_startup_retry_pending = false;
static volatile bool blademotor_startup_version_rx_armed = false;
static volatile bool blademotor_startup_version_response_complete = false;
static volatile bool blademotor_startup_version_response_valid = false;
#endif
static volatile bool blademotor_rx_armed = false;
static volatile uint32_t blademotor_rx_started_tick = 0u;
static volatile uint32_t blademotor_last_valid_tick = 0u;
static volatile uint32_t blademotor_fault_sequence = 0u;
static volatile uint8_t blademotor_seen_valid = 0u;
static volatile uint8_t blademotor_last_error = 0u;
static bool blademotor_reverse_pending = false;
static bool blademotor_off_sent = false;
static bool blademotor_zero_seen = false;
static uint32_t blademotor_stop_since, blademotor_zero_since;
static uint32_t blademotor_last_feedback_seq;
static uint32_t blademotor_zero_epoch;
static uint32_t blademotor_pending_since, blademotor_pending_report_tick;
#if BLADEMOTOR_COASTDOWN_VALIDATION
static uint32_t blademotor_trace_seq, blademotor_trace_tx_tick;
static uint8_t blademotor_trace_command;
#endif

typedef struct {
    uint32_t seq, tick, zero_epoch;
    uint16_t reported_speed;
    uint8_t valid, activated, error;
} blademotor_feedback_t;
/* Updated only after RX completion; snapshot with IRQs masked in the foreground.
 * seq/tick identify UART replies, not new physical speed measurements inside the
 * ESC. Never infer deceleration from a held word or use public cached RPM alone.
 * Invalid replies break qualification. Hardware evidence is in BLADE-REVERSE.md. */
static volatile blademotor_feedback_t blademotor_feedback;

static void blademotor_cancel_on_request(void)
{
    blademotor_u8OnOff = 0u;
    blademotor_u32OnAuthorizationEpoch = 0u;
    blademotor_reverse_pending = false;
    blademotor_off_sent = blademotor_zero_seen = false;
}

static bool blademotor_on_request_is_authorized(void)
{
    return blademotor_u8OnOff != 0u &&
        Emergency_State() == 0u &&
        main_eOpenmowerStatus != OPENMOWER_STATUS_IDLE &&
        MOTORLINK_OutputInhibited() == 0u &&
        ActuatorAuthorization_DriveRequestIsCurrent(
            blademotor_u32OnAuthorizationEpoch);
}

const uint8_t blademotor_pcu8Preamble[5]  = {0x55,0xAA,0x0A,0x2,0xD0};
const uint8_t blademotor_pcu8VersionMsg[BLADEMOTOR_LENGTH_VERSION_MSG] =
    {0x55, 0xaa, 0x03, 0x20, 0x5a, 0x06, 0x82};
const uint8_t blademotor_pcu8InitMsg[BLADEMOTOR_LENGTH_INIT_MSG] =  { 0x55, 0xaa, 0x12, 0x20, 0x80, 0x00, 0xac, 0x0d, 0x00, 0x02, 0x32, 0x50, 0x1e, 0x04, 0x00, 0x15, 0x21, 0x05, 0x0a, 0x19, 0x3c, 0xaa };
/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
*  Public Functions
*******************************************************************************/

static bool blademotor_feedback_qualified_for_reverse(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    blademotor_feedback_t feedback = blademotor_feedback;
    __set_PRIMASK(primask);

    /* Do not miss a bad/nonzero reply overwritten by a later good reply before
     * this foreground cycle. RX also breaks the epoch across a feedback gap. */
    if (feedback.zero_epoch != blademotor_zero_epoch)
    {
        blademotor_zero_seen = false;
        blademotor_zero_epoch = feedback.zero_epoch;
    }
    if (!blademotor_off_sent || !feedback.valid || feedback.activated ||
        feedback.error || feedback.reported_speed != 0 ||
        (uint32_t)(now - feedback.tick) > BLADEMOTOR_FEEDBACK_MAX_AGE_MS)
    {
        blademotor_zero_seen = false;
        blademotor_last_feedback_seq = feedback.seq;
        return false;
    }
    /* Release only while consuming a new reply. The minimum OFF dwell must
     * not expire into an enable using the same cached zero as the last pass. */
    if (feedback.seq == blademotor_last_feedback_seq)
        return false;
    blademotor_last_feedback_seq = feedback.seq;
    if (!blademotor_zero_seen)
    {
        blademotor_zero_since = feedback.tick;
        blademotor_zero_seen = true;
    }
    /* A latched nonzero word never contributes to this zero-report interval.
     * A direct nonzero-to-zero transition needs the full confirmation window. */
    /* The bench image must never reverse, even when the ESC reports zero.
     * Its purpose is to compare that report with independent rotor observation. */
    return !BLADEMOTOR_COASTDOWN_VALIDATION && blademotor_zero_seen &&
        (uint32_t)(feedback.tick - blademotor_zero_since) >= BLADEMOTOR_ZERO_CONFIRM_MS &&
        (uint32_t)(now - blademotor_stop_since) >= BLADEMOTOR_REVERSE_OFF_MS;
}

void blademotor_prepareMsg(void)
{
    uint8_t command = BLADEMOTOR_STOP_COMMAND_VALUE;
    if (!BLADEMOTOR_FeedbackHealthy())
    {
        MOTORLINK_ForceInhibit();
    }
    /* Recheck the request token and system-level actuator gates where the
     * final packet is built. Emergency assert+release, IDLE->MOWING, or link
     * inhibit can invalidate a cached ON before the upper loop runs again.
     * BLADEMOTOR_App holds IRQs through packet construction and DMA start. */
    if (blademotor_u8OnOff && !blademotor_on_request_is_authorized())
    {
        blademotor_cancel_on_request();
    }
    if (!blademotor_u8OnOff)
    {
        /* OFF from the existing emergency/idle/heartbeat gates cancels a
         * pending start. Never retain a queued enable across that request. */
        blademotor_reverse_pending = false;
        blademotor_off_sent = blademotor_zero_seen = false;
    }
    else if (!MOTORLINK_OutputInhibited() && BLADEMOTOR_FeedbackHealthy())
    {
        if (!blademotor_reverse_pending &&
            blademotor_u8Direction != blademotor_u8RunDirection)
        {
            blademotor_reverse_pending = true;
            blademotor_off_sent = blademotor_zero_seen = false;
            blademotor_pending_since = blademotor_pending_report_tick = HAL_GetTick();
        }
        if (!blademotor_reverse_pending || blademotor_feedback_qualified_for_reverse(HAL_GetTick()))
            command = blademotor_u8Direction
                ? BLADEMOTOR_REVERSE_COMMAND_VALUE
                : BLADEMOTOR_FORWARD_COMMAND_VALUE;
    }
    /* Choose the board-specific direction here, where every transmitted frame
     * is built, and compute its checksum from the final command byte.
     * Controller feedback semantics and hardware evidence: BLADE-REVERSE.md. */
    blademotor_pu8RqstMessage[5] = command;
    blademotor_pu8RqstMessage[6] = crcCalc(blademotor_pu8RqstMessage, BLADEMOTOR_LENGTH_RQST_MSG - 1);
}

/**
 * @brief Init the Blade Motor Serial Port (PAC5223)
 * @retval None
 */
void BLADEMOTOR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PAC 5223 Reset Line (Blade Motor) */
    PAC5223RESET_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = PAC5223RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#if defined(BLADEMOTOR_SEQUENCED_POWER)
    /* Keep the PAC logic and active-low blade power gate safe through setup. */
    HAL_GPIO_WritePin(PAC5223RESET_GPIO_PORT, PAC5223RESET_PIN, 0);
#else
    HAL_GPIO_WritePin(PAC5223RESET_GPIO_PORT, PAC5223RESET_PIN, 1);
#endif
    HAL_GPIO_Init(PAC5223RESET_GPIO_PORT, &GPIO_InitStruct);
#if defined(BLADEMOTOR_SEQUENCED_POWER)
    BLADEMOTOR_POWER_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = BLADEMOTOR_POWER_PIN;
    HAL_GPIO_WritePin(BLADEMOTOR_POWER_GPIO_PORT, BLADEMOTOR_POWER_PIN, 1);
    HAL_GPIO_Init(BLADEMOTOR_POWER_GPIO_PORT, &GPIO_InitStruct);
#endif

    // enable port and usart clocks
    BLADEMOTOR_USART_GPIO_CLK_ENABLE();

	// Initiale USART3 for STM32f1 and USART6 for STM32f4
#if BOARD_YARDFORCE500_VARIANT_ORIG
	__HAL_RCC_USART3_CLK_ENABLE();
#elif BOARD_YARDFORCE500_VARIANT_B
	__HAL_RCC_USART6_CLK_ENABLE();
#endif

#if BOARD_YARDFORCE500_VARIANT_ORIG
    // RX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(BLADEMOTOR_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(BLADEMOTOR_USART_TX_PORT, &GPIO_InitStruct);

    // Alternate Pin Set ?
    __HAL_AFIO_REMAP_USART2_ENABLE();
#elif BOARD_YARDFORCE500_VARIANT_B
    // RX TX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_TX_PIN | BLADEMOTOR_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(BLADEMOTOR_USART_TX_PORT, &GPIO_InitStruct);
#endif

    BLADEMOTOR_USART_Handler.Instance = BLADEMOTOR_USART_INSTANCE;
    BLADEMOTOR_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    BLADEMOTOR_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    BLADEMOTOR_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    BLADEMOTOR_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    BLADEMOTOR_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    BLADEMOTOR_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

    HAL_UART_Init(&BLADEMOTOR_USART_Handler);

    DB_TRACE(" * Blade Motor UART initialized\r\n");

    /* UART4 DMA Init */
    /* UART4_RX Init */
#if BOARD_YARDFORCE500_VARIANT_ORIG
	hdma_uart_blade_rx.Instance = DMA1_Channel3;
#elif BOARD_YARDFORCE500_VARIANT_B
	hdma_uart_blade_rx.Instance = DMA2_Stream1;
	hdma_uart_blade_rx.Init.Channel = DMA_CHANNEL_5;
	hdma_uart_blade_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
	hdma_uart_blade_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart_blade_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart_blade_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart_blade_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart_blade_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart_blade_rx.Init.Mode = DMA_NORMAL;
	hdma_uart_blade_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart_blade_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&BLADEMOTOR_USART_Handler, hdmarx, hdma_uart_blade_rx);
    
    /* UART4 DMA Init */
    /* UART4_TX Init */
#if BOARD_YARDFORCE500_VARIANT_ORIG
	hdma_uart_blade_tx.Instance = DMA1_Channel2;
#elif BOARD_YARDFORCE500_VARIANT_B
	hdma_uart_blade_tx.Instance = DMA2_Stream6;
	hdma_uart_blade_tx.Init.Channel = DMA_CHANNEL_5;
	hdma_uart_blade_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
	hdma_uart_blade_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_uart_blade_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart_blade_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart_blade_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart_blade_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart_blade_tx.Init.Mode = DMA_NORMAL;
	hdma_uart_blade_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_uart_blade_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&BLADEMOTOR_USART_Handler, hdmatx, hdma_uart_blade_tx);
    
    // enable IRQ
#if BOARD_YARDFORCE500_VARIANT_ORIG
	IRQn_Type usart_irq = USART3_IRQn;
#elif BOARD_YARDFORCE500_VARIANT_B
	IRQn_Type usart_irq = USART6_IRQn;
#endif

    HAL_NVIC_SetPriority(usart_irq, 0, 0);
	HAL_NVIC_EnableIRQ(usart_irq);
    __HAL_UART_ENABLE_IT(&BLADEMOTOR_USART_Handler, UART_IT_TC);

#if defined(BLADEMOTOR_SEQUENCED_POWER)
    blademotor_state_started_tick = HAL_GetTick();
    blademotor_eState = BLADEMOTOR_RESET_HOLD;
#else
    blademotor_eState = BLADEMOTOR_INIT_1;
#endif
}

/// @brief handle drive motor messages
/// @param  
void  BLADEMOTOR_App(void){
    if (!BLADEMOTOR_FeedbackHealthy())
        MOTORLINK_ForceInhibit();
    switch (blademotor_eState)
    {
#if defined(BLADEMOTOR_SEQUENCED_POWER)
    case BLADEMOTOR_RESET_HOLD:
        if ((uint32_t)(HAL_GetTick() - blademotor_state_started_tick) >= 50u)
        {
            HAL_GPIO_WritePin(PAC5223RESET_GPIO_PORT, PAC5223RESET_PIN, 1);
            blademotor_state_started_tick = HAL_GetTick();
            blademotor_eState = BLADEMOTOR_LOGIC_BOOT;
        }
        break;

    case BLADEMOTOR_LOGIC_BOOT:
        if ((uint32_t)(HAL_GetTick() - blademotor_state_started_tick) >=
                (blademotor_startup_retry_pending ? 1000u : 100u) &&
            BLADEMOTOR_USART_Handler.gState == HAL_UART_STATE_READY &&
            BLADEMOTOR_USART_Handler.RxState == HAL_UART_STATE_READY)
        {
            /* Arm RX before querying the PAC5223. A missing response is
             * bounded and retried while the blade inverter stays unpowered. */
            blademotor_startup_version_response_complete = false;
            blademotor_startup_version_response_valid = false;
            blademotor_startup_version_rx_armed = true;
            if (HAL_UART_Receive_DMA(&BLADEMOTOR_USART_Handler,
                    blademotor_pu8ReceivedData,
                    BLADEMOTOR_LENGTH_VERSION_RESPONSE) == HAL_OK)
            {
                if (HAL_UART_Transmit_DMA(&BLADEMOTOR_USART_Handler,
                        (uint8_t*)blademotor_pcu8VersionMsg,
                        BLADEMOTOR_LENGTH_VERSION_MSG) == HAL_OK)
                {
                    blademotor_state_started_tick = HAL_GetTick();
                    blademotor_eState = BLADEMOTOR_VERSION_WAIT;
                }
                else
                {
                    blademotor_startup_version_rx_armed = false;
                    (void)HAL_UART_AbortReceive(&BLADEMOTOR_USART_Handler);
                    blademotor_startup_retry_pending = true;
                    blademotor_state_started_tick = HAL_GetTick();
                    ++BLADEMOTOR_u32Error;
                    MOTORLINK_ForceInhibit();
                }
            }
            else
            {
                blademotor_startup_version_rx_armed = false;
                blademotor_startup_retry_pending = true;
                blademotor_state_started_tick = HAL_GetTick();
                ++BLADEMOTOR_u32Error;
                MOTORLINK_ForceInhibit();
            }
        }
        break;

    case BLADEMOTOR_VERSION_WAIT:
        if ((uint32_t)(HAL_GetTick() - blademotor_state_started_tick) >= 20u &&
            BLADEMOTOR_USART_Handler.gState == HAL_UART_STATE_READY &&
            BLADEMOTOR_USART_Handler.RxState == HAL_UART_STATE_READY)
        {
            if (blademotor_startup_version_response_complete &&
                blademotor_startup_version_response_valid)
            {
                if (HAL_UART_Transmit_DMA(&BLADEMOTOR_USART_Handler,
                                          (uint8_t*)blademotor_pcu8InitMsg,
                                          BLADEMOTOR_LENGTH_INIT_MSG) == HAL_OK)
                {
                    blademotor_startup_version_rx_armed = false;
                    blademotor_startup_retry_pending = false;
                    blademotor_state_started_tick = HAL_GetTick();
                    blademotor_eState = BLADEMOTOR_MATRIX_WAIT;
                }
                else
                {
                    ++BLADEMOTOR_u32Error;
                    blademotor_startup_version_rx_armed = false;
                    blademotor_startup_retry_pending = true;
                    blademotor_state_started_tick = HAL_GetTick();
                    blademotor_eState = BLADEMOTOR_LOGIC_BOOT;
                    MOTORLINK_ForceInhibit();
                }
            }
            else if (blademotor_startup_version_response_complete)
            {
                blademotor_startup_version_rx_armed = false;
                blademotor_startup_retry_pending = true;
                blademotor_state_started_tick = HAL_GetTick();
                blademotor_eState = BLADEMOTOR_LOGIC_BOOT;
            }
        }
        if (blademotor_eState == BLADEMOTOR_VERSION_WAIT &&
            (uint32_t)(HAL_GetTick() - blademotor_state_started_tick) > 350u)
        {
            blademotor_startup_version_rx_armed = false;
            (void)HAL_UART_AbortReceive(&BLADEMOTOR_USART_Handler);
            (void)HAL_UART_AbortTransmit(&BLADEMOTOR_USART_Handler);
            blademotor_startup_retry_pending = true;
            blademotor_state_started_tick = HAL_GetTick();
            blademotor_eState = BLADEMOTOR_LOGIC_BOOT;
            ++BLADEMOTOR_u32Error;
            MOTORLINK_ForceInhibit();
        }
        break;

    case BLADEMOTOR_MATRIX_WAIT:
        if ((uint32_t)(HAL_GetTick() - blademotor_state_started_tick) >= 20u &&
            BLADEMOTOR_USART_Handler.gState == HAL_UART_STATE_READY)
        {
            HAL_GPIO_WritePin(BLADEMOTOR_POWER_GPIO_PORT,
                              BLADEMOTOR_POWER_PIN, 0);
            blademotor_state_started_tick = HAL_GetTick();
            blademotor_eState = BLADEMOTOR_POWER_WAIT;
        }
        break;

    case BLADEMOTOR_POWER_WAIT:
        if ((uint32_t)(HAL_GetTick() - blademotor_state_started_tick) >= 50u)
        {
            blademotor_eState = BLADEMOTOR_RUN;
            debug_printf(" * RM1000 Blade Motor Controller initialized\r\n");
        }
        break;
#endif

    case BLADEMOTOR_INIT_1:

        HAL_UART_Transmit_DMA(&BLADEMOTOR_USART_Handler, (uint8_t*)blademotor_pcu8InitMsg, BLADEMOTOR_LENGTH_INIT_MSG);
        blademotor_eState = BLADEMOTOR_RUN;
        debug_printf(" * Blade Motor Controller initialized\r\n");     
        break;
    
    case BLADEMOTOR_RUN:

        /*error detected*/
        if(blademotor_pu8ReceivedData[6] != 0){
            blademotor_u8OnOff = 0;
            blademotor_reverse_pending = false;
            blademotor_off_sent = blademotor_zero_seen = false;
            BLADEMOTOR_u32Error++;
        }
        /* Each command gets exactly one bounded response window. Do not issue
         * another request while DMA still owns the RX buffer; a silent or
         * malformed controller must not wedge polling indefinitely. */
        if (blademotor_rx_armed)
        {
            if (BLADEMOTOR_USART_Handler.RxState != HAL_UART_STATE_READY)
            {
                if ((uint32_t)(HAL_GetTick() - blademotor_rx_started_tick) <
                    BLADEMOTOR_RX_TIMEOUT_MS)
                    break;
                (void)HAL_UART_AbortReceive(&BLADEMOTOR_USART_Handler);
                if (BLADEMOTOR_USART_Handler.RxState != HAL_UART_STATE_READY)
                {
                    /* Keep the timeout active if HAL could not reclaim DMA. */
                    blademotor_rx_started_tick = HAL_GetTick();
                    ++BLADEMOTOR_u32Error;
                    MOTORLINK_ForceInhibit();
                    break;
                }
                blademotor_rx_armed = false;
                ++BLADEMOTOR_u32Error;
                MOTORLINK_ForceInhibit();
            }
            else
            {
                /* A full frame completed; BLADEMOTOR_ReceiveIT also clears
                 * this flag, while this path covers HAL completion ordering. */
                blademotor_rx_armed = false;
            }
        }
        if (BLADEMOTOR_USART_Handler.RxState != HAL_UART_STATE_READY ||
            HAL_UART_Receive_DMA(&BLADEMOTOR_USART_Handler,
                blademotor_pu8ReceivedData,
                BLADEMOTOR_LENGTH_RECEIVED_MSG) != HAL_OK)
        {
            ++BLADEMOTOR_u32Error;
            MOTORLINK_ForceInhibit();
            break;
        }
        blademotor_rx_armed = true;
        blademotor_rx_started_tick = HAL_GetTick();

        if (blademotor_reverse_pending &&
            (uint32_t)(HAL_GetTick() - blademotor_pending_report_tick) >= BLADEMOTOR_REVERSE_REPORT_MS)
        {
            blademotor_pending_report_tick = HAL_GetTick();
            BLADEMOTOR_u32Error++;
            debug_printf("Blade reversal waiting: OFF retained (%lu ms)\r\n",
                (unsigned long)(HAL_GetTick() - blademotor_pending_since));
        }
#if BLADEMOTOR_COASTDOWN_VALIDATION
        /* Foreground only; never print in the RX interrupt. Sequence gaps
         * reveal overwritten samples or dropped best-effort UART debug lines. */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        blademotor_feedback_t trace = blademotor_feedback;
        __set_PRIMASK(primask);
        if (trace.seq != blademotor_trace_seq)
        {
            blademotor_trace_seq = trace.seq;
            debug_printf("blade coast t=%lu tx=%02x tx_t=%lu seq=%lu rx_t=%lu valid=%u active=%u speed_word=%u err=%u\r\n",
                (unsigned long)HAL_GetTick(), (unsigned)blademotor_trace_command,
                (unsigned long)blademotor_trace_tx_tick, (unsigned long)trace.seq,
                (unsigned long)trace.tick, (unsigned)trace.valid, (unsigned)trace.activated,
                (unsigned)trace.reported_speed, (unsigned)trace.error);
        }
#endif
        /* Do not rewrite a request buffer still owned by the UART DMA. */
        if (BLADEMOTOR_USART_Handler.gState != HAL_UART_STATE_READY) break;
        const uint32_t output_primask = __get_PRIMASK();
        __disable_irq();
        blademotor_prepareMsg();
        if (BLADEMOTOR_USART_Handler.gState == HAL_UART_STATE_READY &&
            HAL_UART_Transmit_DMA(&BLADEMOTOR_USART_Handler,
                (uint8_t*)blademotor_pu8RqstMessage,
                BLADEMOTOR_LENGTH_RQST_MSG) == HAL_OK)
        {
#if BLADEMOTOR_COASTDOWN_VALIDATION
            if (blademotor_trace_command != blademotor_pu8RqstMessage[5])
            {
                blademotor_trace_command = blademotor_pu8RqstMessage[5];
                blademotor_trace_tx_tick = HAL_GetTick();
            }
#endif
            if (blademotor_pu8RqstMessage[5] & 0x80)
            {
                blademotor_u8RunDirection = blademotor_u8Direction;
                blademotor_reverse_pending = false;
            }
            else if (blademotor_reverse_pending && !blademotor_off_sent)
            {
                /* A failed/busy transmission must not start the stop timer.
                 * Only subsequent feedback can qualify this direction change. */
                blademotor_stop_since = HAL_GetTick();
                blademotor_last_feedback_seq = blademotor_feedback.seq;
                blademotor_off_sent = true;
            }
        }
        __set_PRIMASK(output_primask);
        break;
    
    default:
        break;
    }
}

/// @brief control blade motor (there is no speed control for this motor)
/// @param on_off 1 to turn on, 0 to turn off
void BLADEMOTOR_Set(uint8_t on_off, uint8_t direction,
                    uint32_t authorization_epoch)
{
    /* Keep the cached request and its token atomic with respect to safety
     * interrupts. The caller's snapshot must still be current when accepted. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    blademotor_u8Direction = direction != 0u;
    if (on_off == 0u)
    {
        blademotor_cancel_on_request();
    }
    else if (Emergency_State() != 0u ||
             main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE ||
             MOTORLINK_OutputInhibited() != 0u ||
             !ActuatorAuthorization_DriveRequestIsCurrent(
                 authorization_epoch))
    {
        blademotor_cancel_on_request();
    }
    else
    {
        /* Latch ON and its caller authorization token; never modify the
         * DMA-owned message here. The token is checked again before TX. */
        blademotor_u32OnAuthorizationEpoch = authorization_epoch;
        blademotor_u8OnOff = 1u;
    }
    __set_PRIMASK(primask);
}

/// @brief drive motor receive interrupt handler
/// @param  
void BLADEMOTOR_ReceiveIT(void)
{
    blademotor_rx_armed = false;
    const uint32_t now = HAL_GetTick();
#if defined(BLADEMOTOR_SEQUENCED_POWER)
    if (blademotor_startup_version_rx_armed)
    {
        /* This reply is the PAC5223 version response, not steady-state motor
         * telemetry. Validate its own framed length and additive checksum;
         * never pass it through the 16-byte status decoder. */
        const bool valid =
            blademotor_pu8ReceivedData[0] == 0x55u &&
            blademotor_pu8ReceivedData[1] == 0xaau &&
            blademotor_pu8ReceivedData[2] ==
                (BLADEMOTOR_LENGTH_VERSION_RESPONSE - 4u) &&
            blademotor_pu8ReceivedData[3] == 0x02u &&
            blademotor_pu8ReceivedData[4] == 0xdau &&
            blademotor_pu8ReceivedData[BLADEMOTOR_LENGTH_VERSION_RESPONSE - 1u] ==
                crcCalc(blademotor_pu8ReceivedData,
                        BLADEMOTOR_LENGTH_VERSION_RESPONSE - 1u);
        blademotor_startup_version_rx_armed = false;
        blademotor_startup_version_response_valid = valid;
        blademotor_startup_version_response_complete = true;
        if (!valid)
        {
            ++BLADEMOTOR_u32Error;
            ++blademotor_fault_sequence;
            MOTORLINK_ForceInhibit();
        }
        return;
    }
    /* A late completion from an aborted startup query is not a motor-status
     * frame. Accept 16-byte feedback only after startup reaches RUN. */
    if (blademotor_eState != BLADEMOTOR_RUN)
        return;
#endif
    blademotor_feedback.valid = 0;
    /* decode the frame */    
    if(memcmp(blademotor_pcu8Preamble, blademotor_pu8ReceivedData, 2) == 0){        
        uint8_t l_u8crc = crcCalc(blademotor_pu8ReceivedData, BLADEMOTOR_LENGTH_RECEIVED_MSG-1);

        if(blademotor_pu8ReceivedData[BLADEMOTOR_LENGTH_RECEIVED_MSG-1] == l_u8crc ){
            if((blademotor_pu8ReceivedData[5] & 0x80) == 0x80){
                BLADEMOTOR_bActivated = true;
            }
            else{
                BLADEMOTOR_bActivated = false;
            }
            /* Legacy public RPM field: raw ESC speed word, held after OFF on
             * the tested 500. A new UART reply need not contain a new estimate. */
            BLADEMOTOR_u16RPM = blademotor_pu8ReceivedData[7] + (blademotor_pu8ReceivedData[8]<<8);
            BLADEMOTOR_u16Power = blademotor_pu8ReceivedData[9] + (blademotor_pu8ReceivedData[10]<<8) ;           
            blademotor_feedback.reported_speed = BLADEMOTOR_u16RPM;
            blademotor_feedback.activated = BLADEMOTOR_bActivated;
            blademotor_feedback.error = blademotor_pu8ReceivedData[6];
            blademotor_feedback.valid = 1;
            if (blademotor_seen_valid != 0u &&
                (uint32_t)(now - blademotor_last_valid_tick) >
                    BLADEMOTOR_FEEDBACK_MAX_AGE_MS)
            {
                ++blademotor_fault_sequence;
                MOTORLINK_ForceInhibit();
            }
            blademotor_seen_valid = 1u;
            blademotor_last_valid_tick = now;
            blademotor_last_error = blademotor_pu8ReceivedData[6] != 0u;
            if (blademotor_last_error != 0u)
            {
                ++blademotor_fault_sequence;
                MOTORLINK_ForceInhibit();
            }
        }
        else
        {
            ++blademotor_fault_sequence;
            MOTORLINK_ForceInhibit();
        }

    }
    else
    {
        ++blademotor_fault_sequence;
        MOTORLINK_ForceInhibit();
    }
    if (!blademotor_feedback.valid || blademotor_feedback.activated ||
        blademotor_feedback.error || blademotor_feedback.reported_speed != 0 ||
        (uint32_t)(now - blademotor_feedback.tick) > BLADEMOTOR_FEEDBACK_MAX_AGE_MS)
        blademotor_feedback.zero_epoch++;
    blademotor_feedback.tick = now;
    blademotor_feedback.seq++;
}

bool BLADEMOTOR_FeedbackHealthy(void)
{
    return blademotor_seen_valid != 0u && blademotor_last_error == 0u &&
        (uint32_t)(HAL_GetTick() - blademotor_last_valid_tick) <=
            BLADEMOTOR_FEEDBACK_MAX_AGE_MS;
}

uint32_t BLADEMOTOR_FaultSequence(void)
{
    return blademotor_fault_sequence;
}

/******************************************************************************
*  Private Functions
*******************************************************************************/
