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

int8_t prev_left_direction = 0;
int8_t prev_right_direction = 0;
uint16_t prev_right_encoder_val = 0;
uint16_t prev_left_encoder_val = 0;
int16_t prev_right_wheel_speed_val = 0;
int16_t prev_left_wheel_speed_val = 0;
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
    prev_left_direction = 0;
    prev_right_direction = 0;
    prev_right_encoder_val = 0;
    prev_left_encoder_val = 0;
    prev_right_wheel_speed_val = 0;
    prev_left_wheel_speed_val = 0;
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
         * Motor-controller encoder quirk: the 16-bit encoder register can
         * reset to 0 when the wheel's commanded direction changes (and again
         * when the speed crosses from 0 to non-zero). Detect those events
         * and restart the delta window so we don't register a massive phantom
         * jump.
         */
        /*
         * Direction-change fence: when `left_encoder_reset` is true the
         * motor-controller PCB is in the middle of a direction/speed
         * transient and its encoder register may or may not have been
         * zeroed by the controller yet. Rather than GUESS (which the old
         * `prev_left_encoder_val = 0` did, and failed catastrophically
         * when the controller had NOT reset yet: delta_raw =
         * current_encoder_val - 0 = tens of thousands of ticks
         * mistakenly attributed to the new direction, producing
         * ±1000 m/s bogus velocities downstream), we SYNC `prev` to
         * whatever the current encoder value is. Delta for this one
         * packet is then 0 — we concede up to ~21 ms of fine-grained
         * odometry tracking during the transient, which at 0.5 m/s max
         * is 10 mm — in exchange for never emitting a bogus tick-burst
         * on direction change.
         *
         * Subsequent packets compute deltas against the post-transient
         * baseline, so the rest of the trajectory is uncorrupted.
         */
        left_wheel_speed_val = left_direction * drivemotor_psReceivedData.u8_left_speed;
        const uint8_t left_encoder_reset =
            (left_direction == 0) ||
            (left_direction != prev_left_direction) ||
            (prev_left_wheel_speed_val == 0 && left_wheel_speed_val != 0);
        if (left_encoder_reset)
        {
            prev_left_encoder_val = left_encoder_val;  /* sync, not zero */
        }

        /* Encoder register wraps around 0xFFFF. We compute the delta as a
         * SIGNED int16 difference — natural wrap handling that works in
         * BOTH directions, provided the wheel hasn't spun >32767 ticks in
         * one 20 ms sample (~100 m/s — impossible for a mower).
         *
         * Previously this was an unsigned subtraction, which is only correct
         * for monotonically-increasing forward motion. Any backward motion
         * (mechanical backlash, motor coast through zero, PI overshoot) made
         * the encoder briefly count down — e.g. 100 → 91 read as uint16
         * delta=65527 instead of -9. That uint16 was then multiplied by the
         * motor direction byte, injecting a ±65,527-tick spike into
         * left_ticks_signed every time the wheel jittered through zero.
         * Once the wheel-level PI loop started consuming left_ticks_signed
         * for feedback, those spikes saturated the PI output and the chassis
         * spun at full ω while reporting ~0 m motion to the ROS host.
         */
        const int16_t left_delta_signed = (int16_t)(left_encoder_val - prev_left_encoder_val);
        const uint16_t left_delta_raw = (left_delta_signed >= 0)
                                            ? (uint16_t)left_delta_signed
                                            : (uint16_t)(-left_delta_signed);
        /* Legacy cumulative counter (unsigned-abs) kept for any existing
         * consumer that still expects monotonic positive ticks. */
        left_encoder_ticks += (uint32_t)left_delta_raw;
        /* Signed cumulative count — the encoder timer here is up-only, so we
         * still scale by the commanded motor direction. The signed delta cast
         * above only protects against backward-jitter spikes; for true reverse
         * motion the direction byte does the polarity flip. */
        left_ticks_signed += (int32_t)left_delta_signed * (int32_t)left_direction;
        prev_left_encoder_val = left_encoder_val;
        prev_left_wheel_speed_val = left_wheel_speed_val;
        prev_left_direction = left_direction;

        /* Same fence for the right wheel — direction-change transient is
         * per-wheel on a differential-drive platform (e.g. an in-place
         * pivot has left and right transitions at different times). */
        right_wheel_speed_val = right_direction * drivemotor_psReceivedData.u8_right_speed;
        const uint8_t right_encoder_reset =
            (right_direction == 0) ||
            (right_direction != prev_right_direction) ||
            (prev_right_wheel_speed_val == 0 && right_wheel_speed_val != 0);
        if (right_encoder_reset)
        {
            prev_right_encoder_val = right_encoder_val;  /* sync, not zero */
        }
        const int16_t right_delta_signed = (int16_t)(right_encoder_val - prev_right_encoder_val);
        const uint16_t right_delta_raw = (right_delta_signed >= 0)
                                             ? (uint16_t)right_delta_signed
                                             : (uint16_t)(-right_delta_signed);
        right_encoder_ticks += (uint32_t)right_delta_raw;
        right_ticks_signed += (int32_t)right_delta_signed * (int32_t)right_direction;
        prev_right_encoder_val = right_encoder_val;
        prev_right_wheel_speed_val = right_wheel_speed_val;
        prev_right_direction = right_direction;

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
