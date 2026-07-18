
/**
  ******************************************************************************
  * @file    emergency.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/22
  * @version 1.0.0
  * @brief   Emergency handling, buttons, lift sensors, tilt sensors
  ******************************************************************************  
  * 
  ******************************************************************************
  */
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "stm32f_board_hal.h"

// stm32 custom
#include "board.h"
#include "main.h"
#include "i2c.h"

//#define EMERGENCY_DEBUG 1

/* Written by EmergencyController() in the main loop (bit-OR of sensor flags)
 * AND by Emergency_SetState() from the USB RX interrupt (host assert/release).
 * volatile so neither context caches a stale copy; a plain uint8_t load/store
 * is already atomic on Cortex-M3, but the read-modify-write |= is NOT, so
 * every OR is done under a __disable_irq guard (see emergency_set_bits). */
static volatile uint8_t emergency_state = 0;
static uint32_t stop_emergency_started = 0;
static uint32_t blue_wheel_lift_emergency_started = 0;
static uint32_t red_wheel_lift_emergency_started = 0;
static uint32_t both_wheels_lift_emergency_started = 0;
static uint32_t tilt_emergency_started = 0;
static uint32_t accelerometer_int_emergency_started = 0;
static uint32_t play_button_started = 0;

/* Runtime emergency-sensor timeouts [ms] (PKT_ID_SET_SAFETY_LIMITS). Seeded with
 * the compile-time board_defaults values, which stay the power-on fallback so an
 * unconnected host runs the vetted safe defaults. The four TRIP timeouts can only
 * be SHORTENED (faster e-stop); the play-button CLEAR hold can only be LENGTHENED
 * (harder to un-latch). See emergency_set_timeouts(). */
static volatile uint32_t g_one_wheel_lift_ms = ONE_WHEEL_LIFT_EMERGENCY_MILLIS;
static volatile uint32_t g_both_wheels_lift_ms = BOTH_WHEELS_LIFT_EMERGENCY_MILLIS;
static volatile uint32_t g_tilt_ms = TILT_EMERGENCY_MILLIS;
static volatile uint32_t g_stop_button_ms = STOP_BUTTON_EMERGENCY_MILLIS;
static volatile uint32_t g_play_clear_ms = PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS;

#define EMERGENCY_MIN_TRIP_MS 10u
#define EMERGENCY_MAX_CLEAR_MS 10000u

/* Trip timeouts clamp to [MIN, compiled] — the wire can only SHORTEN them (a
 * faster e-stop is safer); it can never make a fault take longer to trip. */
static uint32_t emergency_clamp_trip(uint32_t v, uint32_t compiled) {
  if (v < EMERGENCY_MIN_TRIP_MS) return EMERGENCY_MIN_TRIP_MS;
  if (v > compiled) return compiled;
  return v;
}
/* The clear-hold clamps to [compiled, MAX] — the wire can only LENGTHEN it
 * (harder to un-latch the emergency). Shortening it would make accidental
 * clearing of the e-stop EASIER = weaker protection, so that direction is
 * forbidden (the one field whose safe direction is the opposite of the trips). */
static uint32_t emergency_clamp_clear(uint32_t v, uint32_t compiled) {
  if (v < compiled) return compiled;
  if (v > EMERGENCY_MAX_CLEAR_MS) return EMERGENCY_MAX_CLEAR_MS;
  return v;
}

void emergency_set_timeouts(uint32_t one_wheel_lift_ms,
                            uint32_t both_wheels_lift_ms, uint32_t tilt_ms,
                            uint32_t stop_button_ms, uint32_t play_clear_ms) {
  /* EmergencyController() reads these in the main loop; this runs in USB RX
   * interrupt context. uint32 stores are atomic on Cortex-M3, but apply the set
   * as a group under the same guard the module uses elsewhere. */
  __disable_irq();
  g_one_wheel_lift_ms =
      emergency_clamp_trip(one_wheel_lift_ms, ONE_WHEEL_LIFT_EMERGENCY_MILLIS);
  g_both_wheels_lift_ms = emergency_clamp_trip(
      both_wheels_lift_ms, BOTH_WHEELS_LIFT_EMERGENCY_MILLIS);
  g_tilt_ms = emergency_clamp_trip(tilt_ms, TILT_EMERGENCY_MILLIS);
  g_stop_button_ms =
      emergency_clamp_trip(stop_button_ms, STOP_BUTTON_EMERGENCY_MILLIS);
  g_play_clear_ms =
      emergency_clamp_clear(play_clear_ms, PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS);
  __enable_irq();
}


/**
 * @brief return Emergency State bits
 * @retval >0 if there is an emergency, 0 if all i good
 */
uint8_t Emergency_State(void)
{
    return(emergency_state);
}

/**
 * @brief Set Emergency State (host/ROS API, runs in USB RX interrupt context)
 * @note  Only assert (any non-zero -> 1) or release (0). The former
 *        EMERGENCY_CHECKING_DISABLE/ENABLE opcodes let a host packet globally
 *        disable ALL physical safety sensors (e-stop, wheel-lift, tilt) — that
 *        bypass has been removed. Firmware is the sole safety authority and its
 *        sensor checking must never be disableable over comms. A single-byte
 *        store to the volatile state is atomic on Cortex-M3.
 * @retval none
 */
void  Emergency_SetState(uint8_t new_emergency_state)
{
    emergency_state = (new_emergency_state != 0) ? 1u : 0u;
}

/**
 * @brief OR sensor bits into the emergency state atomically w.r.t. the USB RX
 *        interrupt (Emergency_SetState). Without the guard, the load-OR-store
 *        could drop a concurrent host assert/release.
 */
static void emergency_set_bits(uint8_t bits)
{
    __disable_irq();
    emergency_state |= bits;
    __enable_irq();
}

/**
 * @brief Poll mechanical Tilt Sensor
 * @retval 1 if tilt is detected, 0 if all is good
 */
int Emergency_Tilt(void)
{
   return(HAL_GPIO_ReadPin(TILT_PORT, TILT_PIN));
}

/**
 * @brief Poll yellow connector stop button
 * @retval 1 if press is detected, 0 if not pressed
 */
int Emergency_StopButtonYellow(void)
{
   return(HAL_GPIO_ReadPin(STOP_BUTTON_YELLOW_PORT, STOP_BUTTON_YELLOW_PIN));
}

/**
 * @brief Poll yellow connector stop button
 * @retval 1 if press is detected, 0 if not pressed
 */
int Emergency_StopButtonWhite(void)
{
   return(HAL_GPIO_ReadPin(STOP_BUTTON_WHITE_PORT, STOP_BUTTON_WHITE_PIN));
}

/**
 * @brief Wheel lift blue sensor
 * @retval 1 if lift is detected, 0 if not lifted
 */
int Emergency_WheelLiftBlue(void)
{
   return(HAL_GPIO_ReadPin(WHEEL_LIFT_BLUE_PORT, WHEEL_LIFT_BLUE_PIN));
}

/**
 * @brief Wheel lift red sensor
 * @retval 1 if lift is detected, 0 if not lifted
 */
int Emergency_WheelLiftRed(void)
{
   return(HAL_GPIO_ReadPin(WHEEL_LIFT_RED_PORT, WHEEL_LIFT_RED_PIN));
}


/**
 * @brief Wheel lift red sensor
 * @retval 1 if lift is detected, 0 if not lifted
 */
int Emergency_LowZAccelerometer(void)
{
   return(I2C_TestZLowINT());
}

/*
 * Manages the emergency sensors
 */
void EmergencyController(void)
{
    uint8_t stop_button_yellow = Emergency_StopButtonYellow();
    uint8_t stop_button_white = Emergency_StopButtonWhite();
    uint8_t wheel_lift_blue = Emergency_WheelLiftBlue();
    uint8_t wheel_lift_red = Emergency_WheelLiftRed();
    uint8_t tilt = Emergency_Tilt();
    GPIO_PinState play_button = !HAL_GPIO_ReadPin(PLAY_BUTTON_PORT, PLAY_BUTTON_PIN); // pullup, active low    
    uint8_t accelerometer_int_triggered = Emergency_LowZAccelerometer();

    uint32_t now = HAL_GetTick();
    static uint32_t l_u32timestamp = 0;

#ifdef EMERGENCY_DEBUG
    debug_printf("EmergencyController()\r\n");
    debug_printf("  >> stop_button_yellow: %d\r\n", Emergency_StopButtonYellow());
    debug_printf("  >> stop_button_white: %d\r\n", Emergency_StopButtonWhite());
    debug_printf("  >> wheel_lift_blue: %d\r\n", Emergency_WheelLiftBlue());
    debug_printf("  >> wheel_lift_red: %d\r\n", Emergency_WheelLiftRed());
    debug_printf("  >> tilt: %d\r\n", Emergency_Tilt());
    debug_printf("  >> accelerometer_int_triggered: %d\r\n", Emergency_LowZAccelerometer());
    debug_printf("  >> play_button: %d\r\n",play_button);
#endif

    if (stop_button_yellow || stop_button_white)
    {
        if (stop_emergency_started == 0)
        {
            stop_emergency_started = now;
        }
        else
        {
            if (now - stop_emergency_started >= g_stop_button_ms)
            {
                if (stop_button_yellow)
                {
                    emergency_set_bits(0b00010);
                    debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - STOP BUTTON (\e[33myellow\e[0m) triggered\r\n");
                }
                if (stop_button_white) {
                    emergency_set_bits(0b00100);
                    debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - STOP BUTTON (\e[37m0mwhite\e[) triggered\r\n");
                }
            }
        }
    }
    else
    {
        stop_emergency_started = 0;
    }

    if (wheel_lift_blue && wheel_lift_red)
    {
        if (both_wheels_lift_emergency_started==0)
        {
            both_wheels_lift_emergency_started=now;
        }
        else if (now-both_wheels_lift_emergency_started>=g_both_wheels_lift_ms)
        {
            emergency_set_bits(0b11000);
            debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - WHEEL LIFT (\e[31mred\e[0m and \e[34mblue\e[0m) triggered\r\n");
        }
    } else {
        both_wheels_lift_emergency_started=0;
    }
    if (wheel_lift_blue)
    {
        if (blue_wheel_lift_emergency_started==0)
        {
            blue_wheel_lift_emergency_started=now;
        }
        else if (now-blue_wheel_lift_emergency_started>=g_one_wheel_lift_ms)
        {
            emergency_set_bits(0b01000);
            debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - WHEEL LIFT (\e[34mblue\e[0m) triggered\r\n");
        }
    } else {
        blue_wheel_lift_emergency_started=0;
    }
    if (wheel_lift_red)
    {
        if (red_wheel_lift_emergency_started==0)
        {
            red_wheel_lift_emergency_started=now;
        }
        else if (now-red_wheel_lift_emergency_started>=g_one_wheel_lift_ms)
        {
            emergency_set_bits(0b10000);
            debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - WHEEL LIFT (\e[31mred\e[0m) triggered\r\n");
        }
    } else {
        red_wheel_lift_emergency_started=0;
    }

    if (accelerometer_int_triggered)
    {
        if(accelerometer_int_emergency_started == 0)
        {
            accelerometer_int_emergency_started = now;
        }
        else
        {
            if (now - accelerometer_int_emergency_started >= g_tilt_ms) {
                emergency_set_bits(0b100000);
                debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - ACCELEROMETER TILT triggered\r\n");
            }
        }     
    }
    else
    {
        accelerometer_int_emergency_started = 0;
    }
    
    if (tilt)
    {
        if(tilt_emergency_started == 0)
        {
            tilt_emergency_started = now;
        }
        else
        {
            if (now - tilt_emergency_started >= g_tilt_ms) {
                emergency_set_bits(0b100000);
                debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - MECHANICAL TILT triggered\r\n");
            }
        }
    }
    else
    {
        tilt_emergency_started = 0;
    }

    if (emergency_state && play_button)
    {
        if(play_button_started == 0)
        {
            play_button_started = now;
        }
        else
        {
            if (now - play_button_started >= g_play_clear_ms) {
                emergency_state = 0;
                debug_printf(" \e[01;31m## EMERGENCY ##\e[0m - manual reset\r\n");
				StatusLEDUpdate();
                do_chirp=1;
            }
        }
    }
    else
    {
        play_button_started = 0;
    }
    /* play buzzer when emergency every 5s*/
    if(emergency_state  && ((HAL_GetTick()-l_u32timestamp) > 5000)){
        l_u32timestamp = HAL_GetTick();
        do_chirp=5;
    }
}

/**
 * @brief Emergency sensors
 * @retval None
 */
void Emergency_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    STOP_BUTTON_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = STOP_BUTTON_YELLOW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(STOP_BUTTON_YELLOW_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = STOP_BUTTON_WHITE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(STOP_BUTTON_WHITE_PORT, &GPIO_InitStruct);

    TILT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = TILT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(TILT_PORT, &GPIO_InitStruct);

    WHEEL_LIFT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = WHEEL_LIFT_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WHEEL_LIFT_BLUE_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = WHEEL_LIFT_RED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WHEEL_LIFT_RED_PORT, &GPIO_InitStruct);

}
