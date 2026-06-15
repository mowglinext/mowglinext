/****************************************************************************
* Title                 :   drive motor module
* Filename              :   drivemotor.c
* Author                :   Nekraus
* Origin Date           :   18/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file drivemotor.c
 *  \brief drive motor module
 *
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include <string.h>
#include <stdlib.h>

#include "stm32f_board_hal.h"

#include "main.h"
#include "ros/ros_custom/cpp_main.h"
#include "board.h"
#include "adc.h"

#include "drivemotor.h"

/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/
#define DRIVEMOTOR_LENGTH_INIT_MSG 38
#define DRIVEMOTOR_LENGTH_RQST_MSG 12
#define DRIVEMOTOR_LENGTH_RECEIVED_MSG 20

/* Kinematic ceiling on per-frame encoder motion. cmd_vel is capped to MAX_MPS,
 * so in one ~20 ms controller frame a wheel advances at most
 *   MAX_MPS * TICKS_PER_M * 0.02 s  ticks.
 * The x3 factor is slack for frame-time jitter; a "reset" whose remainder
 * exceeds this is not real motion (a glitch) and is dropped, not accumulated. */
#define DRIVEMOTOR_MAX_TICKS_PER_FRAME ((uint32_t)(MAX_MPS * TICKS_PER_M * 0.02f * 3.0f))
/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/
typedef enum
{
    DRIVEMOTOR_INIT_1,
    DRIVEMOTOR_INIT_2,
    DRIVEMOTOR_RUN,
    DRIVEMOTOR_BACKWARD,
    DRIVEMOTOR_WAIT
} DRIVEMOTOR_STATE_e;

typedef struct
{
    /* 0*/ uint16_t u16_preambule;
    /* 2*/ uint8_t u8_length;
    /* 3*/ uint16_t u16_id;
    /* 5*/ uint8_t u8_direction;
    /* 6*/ uint8_t u8_left_speed;
    /* 7*/ uint8_t u8_right_speed;
    /* 8*/ uint16_t u16_ukndata0;
    /*10*/ uint8_t u8_left_power;
    /*11*/ uint8_t u8_right_power;
    /*12*/ uint8_t u8_error;
    /*13*/ uint16_t u16_left_ticks;
    /*15*/ uint16_t u16_right_ticks;
    /*17*/ uint8_t u8_left_ukn;
    /*18*/ uint8_t u8_right_ukn;
    /*19*/ uint8_t u8_CRC;
} __attribute__((__packed__)) DRIVEMOTORS_data_t;

/* Per-wheel travel-direction filter (see resolve_direction / DRIVEMOTOR_App_Rx).
 * Holds the last *confirmed* physical direction so the motor controller's
 * one-frame-early direction flip on a reversal cannot mis-sign coast ticks. */
typedef struct
{
    int8_t  s8Eff;    /* committed/published travel direction (-1/0/+1)        */
    uint8_t u8Resets; /* counter resets counted during an unconfirmed reversal */
} DRIVEMOTOR_dirfilter_t;

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
UART_HandleTypeDef DRIVEMOTORS_USART_Handler; // UART  Handle

DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

static DRIVEMOTOR_STATE_e drivemotor_eState = DRIVEMOTOR_INIT_1;
static rx_status_e drivemotors_eRxFlag = RX_WAIT;

static DRIVEMOTORS_data_t drivemotor_psReceivedData = {0};
static uint8_t drivemotor_pu8RqstMessage[DRIVEMOTOR_LENGTH_RQST_MSG] = {0x55, 0xaa, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t drivemotor_pcu8Preamble[5] = {0x55, 0xAA, 0x10, 0x01, 0xE0};
// const uint8_t drivemotor_pcu8InitMsg[DRIVEMOTOR_LENGTH_INIT_MSG] = { 0x55, 0xaa, 0x08, 0x10, 0x80, 0xa0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x37};
const uint8_t drivemotor_pcu8InitMsg[DRIVEMOTOR_LENGTH_INIT_MSG] = {0x55, 0xaa, 0x22, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0xC8, 0x46, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0F, 0x14, 0x96, 0x0A, 0x1E, 0x5a, 0xfa, 0x05, 0x0A, 0x14, 0x32, 0x40, 0x04, 0x20, 0x01, 0x00, 0x00, 0x2C, 0x01, 0xEE};

uint16_t prev_right_encoder_val = 0;
uint16_t prev_left_encoder_val = 0;
/* Last *confirmed* per-wheel direction + reset-bracket counters. Replaces the
 * old prev_*_direction / prev_*_wheel_speed_val edge-prediction fence, which
 * raced the controller's reset latency and mis-signed reversal coast ticks. */
static DRIVEMOTOR_dirfilter_t left_dir_filter = {0, 0};
static DRIVEMOTOR_dirfilter_t right_dir_filter = {0, 0};
uint32_t right_encoder_ticks = 0;
uint32_t left_encoder_ticks = 0;
int32_t  right_ticks_signed = 0;  /**< Cumulative SIGNED encoder ticks (polarity = direction) */
int32_t  left_ticks_signed  = 0;
int8_t left_direction = 0;
int8_t right_direction = 0;
uint16_t right_encoder_val = 0;
uint16_t left_encoder_val = 0;
int16_t right_wheel_speed_val = 0;
int16_t left_wheel_speed_val = 0;
uint8_t right_power = 0;
uint8_t left_power = 0;

uint32_t DRIVEMOTOR_u32ErrorCnt = 0;

static uint8_t left_speed_req;
static uint8_t right_speed_req;
static uint8_t left_dir_req;
static uint8_t right_dir_req;

/* No host-side PWM deadband: the PAC5210 drive-motor controller runs its
 * own internal loop and handles sub-threshold commands gracefully (this
 * is how the original cedbossneo/mowgli firmware behaved, and it worked
 * fine). Adding a host-side deadband promotion on 2026-04-19 introduced
 * a 2.5× angular overshoot at low wz — any command between [-34, -1] ∪
 * [1, 34] got snapped to ±35, so wz=0.30 rad/s produced physical rotation
 * at ~0.72 rad/s. Confirmed in Voie C Test 2026-04-24 (commanded 90°
 * → measured 225° physical). The fix is to pass PWM through unchanged
 * and let the motor controller handle its own dynamics. */

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
__STATIC_INLINE void drivemotor_prepareMsg(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir);

/******************************************************************************
 *  Public Functions
 *******************************************************************************/

/// @brief Initialize STM32 hardware UART to control drive motors
/// @param
void DRIVEMOTOR_Init(void)
{
    PAC5210RESET_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PAC5210RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PAC5210RESET_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PAC5210RESET_GPIO_PORT, PAC5210RESET_PIN, 0); // take Drive Motor PAC out of reset if LOW

    // PD7 (->PAC5210 PC4), PD8 (->PAC5210 PC3)
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7 | GPIO_PIN_8, 1);

    // enable port and usart clocks
    DRIVEMOTORS_USART_GPIO_CLK_ENABLE();
    DRIVEMOTORS_USART_USART_CLK_ENABLE();

#if BOARD_YARDFORCE500_VARIANT_ORIG
    // RX
    GPIO_InitStruct.Pin = DRIVEMOTORS_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DRIVEMOTORS_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = DRIVEMOTORS_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DRIVEMOTORS_USART_TX_PORT, &GPIO_InitStruct);

    // Alternate Pin Set ?
    __HAL_AFIO_REMAP_USART2_ENABLE();
#elif BOARD_YARDFORCE500_VARIANT_B
    // RX TX
    GPIO_InitStruct.Pin = DRIVEMOTORS_USART_TX_PIN | DRIVEMOTORS_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(DRIVEMOTORS_USART_TX_PORT, &GPIO_InitStruct);
#endif

    DRIVEMOTORS_USART_Handler.Instance = DRIVEMOTORS_USART_INSTANCE; // USART2
    DRIVEMOTORS_USART_Handler.Init.BaudRate = 115200;                // Baud rate
    DRIVEMOTORS_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B;  // The word is  8  Bit format
    DRIVEMOTORS_USART_Handler.Init.StopBits = USART_STOPBITS_1;      // A stop bit
    DRIVEMOTORS_USART_Handler.Init.Parity = UART_PARITY_NONE;        // No parity bit
    DRIVEMOTORS_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // No hardware flow control
    DRIVEMOTORS_USART_Handler.Init.Mode = USART_MODE_TX_RX;          // Transceiver mode

    HAL_UART_Init(&DRIVEMOTORS_USART_Handler);

    /* USART2 DMA Init */
    /* USART2_RX Init */
#if BOARD_YARDFORCE500_VARIANT_ORIG
    hdma_usart2_rx.Instance = DMA1_Channel6;
#elif BOARD_YARDFORCE500_VARIANT_B
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(&DRIVEMOTORS_USART_Handler, hdmarx, hdma_usart2_rx);

    // USART2_TX Init */
#if BOARD_YARDFORCE500_VARIANT_ORIG
	hdma_usart2_tx.Instance = DMA1_Channel7;
#elif BOARD_YARDFORCE500_VARIANT_B
	hdma_usart2_tx.Instance = DMA1_Stream6;
	hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&DRIVEMOTORS_USART_Handler, hdmatx, hdma_usart2_tx);

    // enable IRQ
    HAL_NVIC_SetPriority(DRIVEMOTORS_USART_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(DRIVEMOTORS_USART_IRQ);

    __HAL_UART_ENABLE_IT(&DRIVEMOTORS_USART_Handler, UART_IT_TC);

    right_encoder_ticks = 0;
    left_encoder_ticks = 0;
    prev_right_encoder_val = 0;
    prev_left_encoder_val = 0;
    left_dir_filter = (DRIVEMOTOR_dirfilter_t){0, 0};
    right_dir_filter = (DRIVEMOTOR_dirfilter_t){0, 0};
}

/// @brief handle drive motor messages
/// @param
void DRIVEMOTOR_App_10ms(void)
{

    static uint32_t l_u32Timestamp = 0;

    switch (drivemotor_eState)
    {
    case DRIVEMOTOR_INIT_1:

        HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)drivemotor_pcu8InitMsg, DRIVEMOTOR_LENGTH_INIT_MSG);
        drivemotor_eState = DRIVEMOTOR_RUN;
        debug_printf(" * Drive Motor Controller initialized\r\n");
        break;

    case DRIVEMOTOR_RUN:

        /* prepare to receive the message before to launch the command */
        HAL_UART_Receive_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)&drivemotor_psReceivedData, sizeof(DRIVEMOTORS_data_t));

        drivemotor_prepareMsg(left_speed_req, right_speed_req, left_dir_req, right_dir_req);
        /* error State*/
        if (drivemotor_psReceivedData.u8_error != 0)
        {
            drivemotor_prepareMsg(0, 0, 0, 0);
            DRIVEMOTOR_u32ErrorCnt++;
        }

        /* todo add also accelerometer detection*/
        if ((HALLSTOP_Left_Sense() || HALLSTOP_Right_Sense()) && (left_dir_req || right_dir_req))
        {

            switch (main_eOpenmowerStatus)
            {
            case OPENMOWER_STATUS_MOWING:
                /*hit something goes back */
                drivemotor_eState = DRIVEMOTOR_BACKWARD;
                l_u32Timestamp = HAL_GetTick();
                break;
            case OPENMOWER_STATUS_DOCKING:
                /* Get voltage from dock, stop the mower*/
                if (chargerInputVoltage > MIN_DOCKED_VOLTAGE)
                {
                    drivemotor_prepareMsg(0, 0, 0, 0);
                }
                else
                { /*hit something goes back */
                    drivemotor_eState = DRIVEMOTOR_BACKWARD;
                    l_u32Timestamp = HAL_GetTick();
                }

                break;
            case OPENMOWER_STATUS_UNDOCKING:
            case OPENMOWER_STATUS_IDLE:
            case OPENMOWER_STATUS_RECORD:
            default:
                /* nothing to do in these modes*/
                break;
            }
        }

        HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)drivemotor_pu8RqstMessage, DRIVEMOTOR_LENGTH_RQST_MSG);

        break;

    case DRIVEMOTOR_BACKWARD:
        /* prepare to receive the message before to launch the command */
        HAL_UART_Receive_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)&drivemotor_psReceivedData, sizeof(DRIVEMOTORS_data_t));
        drivemotor_prepareMsg(100, 100, 0, 0); /* set to -0.33m/s  */
        HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)drivemotor_pu8RqstMessage, DRIVEMOTOR_LENGTH_RQST_MSG);

        if ((HAL_GetTick() - l_u32Timestamp) > 2000)
        {
            drivemotor_eState = DRIVEMOTOR_WAIT;
            l_u32Timestamp = HAL_GetTick();
        }

        break;

    case DRIVEMOTOR_WAIT:
        /* prepare to receive the message before to launch the command */
        HAL_UART_Receive_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)&drivemotor_psReceivedData, sizeof(DRIVEMOTORS_data_t));
        drivemotor_prepareMsg(0, 0, 0, 0);
        HAL_UART_Transmit_DMA(&DRIVEMOTORS_USART_Handler, (uint8_t *)drivemotor_pu8RqstMessage, DRIVEMOTOR_LENGTH_RQST_MSG);

        if ((HAL_GetTick() - l_u32Timestamp) > 1000)
        {
            drivemotor_eState = DRIVEMOTOR_RUN;
        }

        break;

    default:
        break;
    }

    /* TODO error management */
    switch (drivemotors_eRxFlag)
    {
    case RX_VALID:
        break;

    case RX_WAIT:

        /* todo check for timeout */

        break;

    case RX_CRC_ERROR:
    case RX_INVALID_ERROR:
    case RX_TIMEOUT_ERROR:
    default:
        /* inform for error */
        break;
    }
}

/// @brief Resolve the physical travel direction of a wheel for odometry.
///
/// The drive controller flips its reported direction bit one to two frames
/// BEFORE the wheel has actually reversed. For those frames it reports the new
/// direction with a speed byte of 0 while the wheel is really still coasting in
/// the OLD direction. Believing that bit feeds odometry a few ticks with the
/// wrong sign on every reversal (which here corrupts BOTH /wheel_odom and the
/// wheel-PI loop, since both consume left/right_ticks_signed).
///
/// So a reported reversal (opposite sign to the committed direction) is only
/// adopted once the new segment is confirmed by EITHER
///   (B) the reported speed byte going non-zero (controller now actively
///       driving the new direction), OR
///   (A) a second counter reset (the controller's documented double-reset that
///       brackets the real new segment).
/// Until then the previously committed direction keeps being reported. Resumes
/// in the same direction and the first motion from rest carry no stale motion
/// to mis-sign, so they are adopted immediately.
///
/// @param f         per-wheel filter state (updated in place)
/// @param reported  decoded reported direction this frame (-1/0/+1)
/// @param speed     reported speed byte this frame
/// @param reset     non-zero if the raw counter decreased this frame (a reset)
/// @return the committed (publishable) travel direction
static int8_t resolve_direction(DRIVEMOTOR_dirfilter_t *f, int8_t reported, uint8_t speed, int reset)
{
    if (reported != 0 && (speed != 0 || f->s8Eff == 0))
    {
        /* (B) the speed byte confirms real motion in the reported direction -
         * or this is the first motion from rest, with no old direction to
         * protect. Either way, believe the reported direction immediately. */
        f->s8Eff = reported;
        f->u8Resets = 0;
    }
    else if (reported == -f->s8Eff)
    {
        /* Unconfirmed reversal: the controller reports the opposite direction
         * but the speed byte is still 0 while the wheel coasts the old way.
         * Keep the old direction until (A) a second counter reset confirms the
         * real new segment. (Frames here have speed 0, handled above first.) */
        if (reset && ++f->u8Resets >= 2)
        {
            f->s8Eff = reported;
            f->u8Resets = 0;
        }
    }
    else
    {
        /* stop report or same-direction frame: no reversal in progress */
        f->u8Resets = 0;
    }
    return f->s8Eff;
}

/// @brief Fold one controller frame into a wheel's tick totals + travel dir.
///
/// Rule 1 - detect the counter reset DIRECTLY: any decrease in the controller's
/// per-wheel counter means it reset to 0, so the progress since the reset is the
/// new value; otherwise the counter rose and we add the difference. (The old
/// code PREDICTED the reset from direction/speed edges and pre-synced prev_*,
/// which raced the controller's one-frame reset latency and scored the pre-reset
/// count as a phantom jump.) A reset whose remainder exceeds one frame's
/// kinematic limit is a glitch and is dropped rather than accumulated.
///
/// Rule 2 - the magnitude is signed by the CONFIRMED direction from
/// resolve_direction(), not the reported byte, so coast ticks on a reversal
/// keep the old sign until the wheel has truly turned around.
///
/// Accumulates both the legacy unsigned-abs counter (*ticks) and the signed
/// cumulative counter (*ticks_signed, consumed by the host + wheel-PI loop).
///
/// @return the committed (publishable) travel direction for this wheel.
static int8_t DRIVEMOTOR_UpdateWheel(DRIVEMOTOR_dirfilter_t *f, int8_t reported, uint8_t speed,
                                     uint16_t val, uint16_t *prev,
                                     uint32_t *ticks, int32_t *ticks_signed)
{
    int32_t l_s32Delta = (int32_t)val - (int32_t)*prev;
    /* Confirmed travel direction for THIS frame's motion. */
    int8_t l_s8Eff = resolve_direction(f, reported, speed, l_s32Delta < 0);

    uint32_t l_u32Mag = 0;
    if (reported != 0)
    {
        if (l_s32Delta >= 0)
        {
            l_u32Mag = (uint32_t)l_s32Delta;             /* progress within a segment */
        }
        else if ((uint32_t)val <= DRIVEMOTOR_MAX_TICKS_PER_FRAME)
        {
            l_u32Mag = (uint32_t)val;                    /* reset: new-segment progress so far */
        }
        /* else: implausibly large reset remainder => glitch, drop it */
    }
    *prev = val;
    *ticks += l_u32Mag;                                  /* legacy unsigned-abs counter */
    *ticks_signed += (int32_t)l_u32Mag * (int32_t)l_s8Eff;  /* signed by CONFIRMED dir */
    return l_s8Eff;
}

/// @brief Decode received drive motor messages
/// @param
void DRIVEMOTOR_App_Rx(void)
{
    if (drivemotors_eRxFlag == RX_VALID)
    {
        /* decode */
        uint8_t direction = drivemotor_psReceivedData.u8_direction;
        // we need to adjust for direction (+/-) !
        if ((direction & 0xc0) == 0xc0)
        {
            left_direction = 1;
        }
        else if ((direction & 0x80) == 0x80)
        {
            left_direction = -1;
        }
        else
        {
            left_direction = 0;
        }
        if ((direction & 0x30) == 0x30)
        {
            right_direction = 1;
        }
        else if ((direction & 0x20) == 0x20)
        {
            right_direction = -1;
        }
        else
        {
            right_direction = 0;
        }

        left_encoder_val = drivemotor_psReceivedData.u16_left_ticks;
        right_encoder_val = drivemotor_psReceivedData.u16_right_ticks;

        // power consumption
        left_power = drivemotor_psReceivedData.u8_left_power;
        right_power = drivemotor_psReceivedData.u8_right_power;

        /*
         * Fold each wheel's controller frame into its tick totals and resolve
         * the true travel direction (DRIVEMOTOR_UpdateWheel / resolve_direction).
         * Reset detection is now DIRECT — any drop in the per-wheel counter is a
         * reset, so the new value is the post-reset progress (a glitch beyond one
         * frame's kinematic limit is dropped). The publishable direction is held
         * at the old sign through a reversal until the speed byte or a second
         * reset confirms the wheel has truly turned around, so the coast ticks
         * the controller emits one frame early are never mis-signed. This
         * replaces the old predict-the-reset fence + raw-direction signing, which
         * raced the controller's reset latency and corrupted left/right_ticks_
         * signed (and thus /wheel_odom + the wheel-PI loop) on every reversal.
         */
        int8_t l_s8EffLeftDir = DRIVEMOTOR_UpdateWheel(
            &left_dir_filter, left_direction, drivemotor_psReceivedData.u8_left_speed,
            left_encoder_val, &prev_left_encoder_val,
            &left_encoder_ticks, &left_ticks_signed);
        int8_t l_s8EffRightDir = DRIVEMOTOR_UpdateWheel(
            &right_dir_filter, right_direction, drivemotor_psReceivedData.u8_right_speed,
            right_encoder_val, &prev_right_encoder_val,
            &right_encoder_ticks, &right_ticks_signed);

        /* Sign the reported speed magnitude by the CONFIRMED direction too, so
         * the host's velocity sign agrees with the signed-tick trend. */
        left_wheel_speed_val = l_s8EffLeftDir * drivemotor_psReceivedData.u8_left_speed;
        right_wheel_speed_val = l_s8EffRightDir * drivemotor_psReceivedData.u8_right_speed;

        wheelTicks_handler(left_ticks_signed, right_ticks_signed,
                           left_wheel_speed_val, right_wheel_speed_val);

        drivemotors_eRxFlag = RX_WAIT; // ready for next message
    }
}

/**
 * @brief  Set drive motor speeds from a signed PWM command per wheel.
 *
 * Single scalar per wheel encodes both magnitude and direction: positive =
 * forward, negative = reverse, 0 = stop. The motor-controller PCB's legacy
 * (|speed|, direction-bit) interface is produced internally by this function.
 *
 * No host-side deadband: the PAC5210 motor controller handles sub-threshold
 * commands on its own. Saturates to 8-bit motor-controller magnitude range.
 *
 * @param  left_pwm_signed   signed PWM command for the left wheel
 * @param  right_pwm_signed  signed PWM command for the right wheel
 */
void DRIVEMOTOR_SetSpeedSigned(int16_t left_pwm_signed, int16_t right_pwm_signed)
{
    /* Saturate to the 8-bit motor-controller magnitude. */
    if (left_pwm_signed  >  255) left_pwm_signed  =  255;
    if (left_pwm_signed  < -255) left_pwm_signed  = -255;
    if (right_pwm_signed >  255) right_pwm_signed =  255;
    if (right_pwm_signed < -255) right_pwm_signed = -255;

    left_speed_req  = (uint8_t)(left_pwm_signed  < 0 ? -left_pwm_signed  : left_pwm_signed);
    right_speed_req = (uint8_t)(right_pwm_signed < 0 ? -right_pwm_signed : right_pwm_signed);

    /* Motor-controller convention: dir=1 → forward at |speed|, dir=0 →
     * reverse at |speed| (or stop when |speed|=0). Only forward maps to
     * a non-zero dir byte. */
    left_dir_req  = (left_pwm_signed  > 0) ? 1 : 0;
    right_dir_req = (right_pwm_signed > 0) ? 1 : 0;
}

/**
 * @brief  Legacy 4-argument form kept as a thin shim over the signed API so
 *         any caller that still uses `(speed, speed, dir, dir)` keeps
 *         working. New code should call DRIVEMOTOR_SetSpeedSigned directly.
 */
void DRIVEMOTOR_SetSpeed(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir)
{
    const int16_t l_signed = left_dir  ? (int16_t)left_speed  : -(int16_t)left_speed;
    const int16_t r_signed = right_dir ? (int16_t)right_speed : -(int16_t)right_speed;
    /* If both speeds are 0, the deadband path passes 0 through unchanged,
     * matching the old "0,0,0,0 == stop" contract. */
    DRIVEMOTOR_SetSpeedSigned(
        (left_speed  == 0) ? 0 : l_signed,
        (right_speed == 0) ? 0 : r_signed);
}

/// @brief drive motor receive interrupt handler
/// @param
void DRIVEMOTOR_ReceiveIT(void)
{
    /* decode the frame */
    if (memcmp(drivemotor_pcu8Preamble, (uint8_t *)&drivemotor_psReceivedData, 5) == 0)
    {
        uint8_t l_u8crc = crcCalc((uint8_t *)&drivemotor_psReceivedData, DRIVEMOTOR_LENGTH_RECEIVED_MSG - 1);
        if (drivemotor_psReceivedData.u8_CRC == l_u8crc)
        {
            drivemotors_eRxFlag = RX_VALID;
        }
        else
        {
            drivemotors_eRxFlag = RX_CRC_ERROR;
        }
    }
    else
    {
        drivemotors_eRxFlag = RX_INVALID_ERROR;
    }
}

/******************************************************************************
 *  Private Functions
 *******************************************************************************/

__STATIC_INLINE void drivemotor_prepareMsg(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir)
{

    uint8_t direction = 0x0;

    // calc direction bits
    if (right_dir == 1)
    {
        direction |= (0x20 + 0x10);
    }
    else
    {
        direction |= 0x20;
    }
    if (left_dir == 1)
    {
        direction |= (0x40 + 0x80);
    }
    else
    {
        direction |= 0x80;
    }

    drivemotor_pu8RqstMessage[0] = 0x55;
    drivemotor_pu8RqstMessage[1] = 0xaa;
    drivemotor_pu8RqstMessage[2] = 0x08;
    drivemotor_pu8RqstMessage[3] = 0x10;
    drivemotor_pu8RqstMessage[4] = 0x80;
    drivemotor_pu8RqstMessage[5] = direction;
    drivemotor_pu8RqstMessage[6] = left_speed;
    drivemotor_pu8RqstMessage[7] = right_speed;
    drivemotor_pu8RqstMessage[9] = 0;
    drivemotor_pu8RqstMessage[8] = 0;
    drivemotor_pu8RqstMessage[10] = 0;
    drivemotor_pu8RqstMessage[11] = crcCalc(drivemotor_pu8RqstMessage, DRIVEMOTOR_LENGTH_RQST_MSG - 1);
}
