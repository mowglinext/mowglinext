

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C"
{
#endif

    // this is the sofware version that any other Mowgli components like MowgliRover will match against

#define MOWGLI_SW_VERSION_MAJOR 1
#define MOWGLI_SW_VERSION_BRANCH 11 /* even = stable, odd = testing/unstable */
#define MOWGLI_SW_VERSION_MINOR 1

/********************************************************************************
 * BOARD SELECTION
 * the specific board setting are set a the end of this file
 ********************************************************************************/
//#define BOARD_LUV1000RI 1

/* definition type don't modify */
#define DEBUG_TYPE_NONE 0
#define DEBUG_TYPE_UART 1
#define DEBUG_TYPE_SWO 2

/* Publish Mowgli Topics */
//#define ROS_PUBLISH_MOWGLI

/* different type of panel are possible */
#define PANEL_TYPE_NONE 0
#define PANEL_TYPE_YARDFORCE_500_CLASSIC 1
#define PANEL_TYPE_YARDFORCE_LUV1000RI 2
#define PANEL_TYPE_YARDFORCE_900_ECO 3
#define PANEL_TYPE_YARDFORCE_500B_CLASSIC 4
#define PANEL_TYPE_YARDFORCE_NX100I 5 /* "RM EC3" LCD panel, USART1, panel_nx100i.c */

/* ultrasonic backends, used when OPTION_ULTRASONIC == 1 */
#define ULTRASONIC_TYPE_UART 1       /* LUV1000RI: serial module on MASTER_USART (ultrasonic_sensor.c) */
#define ULTRASONIC_TYPE_NX100I_I2C 2 /* NX100i: "RM CSB" board on bit-banged I2C PC10/PC11 (ultrasonic_nx100i.c) */

#if BOARD_YARDFORCE500_VARIANT_ORIG
///////////////////////////
// Yardforce 500 CLASSIC //
///////////////////////////
#define BLADEMOTOR_USART_INSTANCE USART3

#define VALID_BOARD_DEFINED 1
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500_CLASSIC
#define BLADEMOTOR_LENGTH_RECEIVED_MSG 16
#define BLADEMOTOR_STOP_COMMAND_VALUE 0x00u
#define BLADEMOTOR_FORWARD_COMMAND_VALUE 0x80u
#define BLADEMOTOR_REVERSE_COMMAND_VALUE 0xC0u
#define BLADEMOTOR_POLL_INTERVAL_MS 100u
#define DEBUG_TYPE DEBUG_TYPE_UART

#define MAX_MPS 0.5		  // Allow maximum speed of 1.0 m/s
#define PWM_PER_MPS 337.0 // PWM value of 300 means 1 m/s bot speed so we divide by 4 to have correct robot speed but still progressive speed
#define TICKS_PER_M 339.0 // Power-on fallback encoder ticks per meter; ROS runtime tuning overrides this after host connection
#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters

#define OPTION_ULTRASONIC 0
#define OPTION_BUMPER 0

#define BOARD_HAS_MASTER_USART 1
#elif BOARD_YARDFORCE500_VARIANT_B
/////////////////////
// Yardforce 500 B //
/////////////////////

#define BLADEMOTOR_USART_INSTANCE USART6

#define VALID_BOARD_DEFINED 1
#if defined(BOARD_BILTEMA_RM1000)
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_900_ECO
#define BLADEMOTOR_STOP_COMMAND_VALUE 0x02u
#define BLADEMOTOR_FORWARD_COMMAND_VALUE 0x81u
#define BLADEMOTOR_REVERSE_COMMAND_VALUE 0xC1u
#define BLADEMOTOR_POLL_INTERVAL_MS 20u
#define BLADEMOTOR_SEQUENCED_POWER 1
#else
#if defined(BOARD_YARDFORCE_NX100I)
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_NX100I
#else
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500B_CLASSIC
#endif
#define BLADEMOTOR_STOP_COMMAND_VALUE 0x00u
#define BLADEMOTOR_FORWARD_COMMAND_VALUE 0x80u
#define BLADEMOTOR_REVERSE_COMMAND_VALUE 0xC0u
#define BLADEMOTOR_POLL_INTERVAL_MS 100u
#endif
#define BLADEMOTOR_LENGTH_RECEIVED_MSG 16
#define DEBUG_TYPE DEBUG_TYPE_SWO

#define MAX_MPS 0.5		  // Allow maximum speed of 1.0 m/s
#define PWM_PER_MPS 300.0 // PWM value of 300 means 1 m/s bot speed so we divide by 4 to have correct robot speed but still progressive speed
#define TICKS_PER_M 277.0 // Power-on fallback encoder ticks per meter; ROS runtime tuning overrides this after host connection
#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters

#if defined(BOARD_YARDFORCE_NX100I)
/* The NX100i hood carries an ultrasonic board ("RM CSB", two sensors) that
 * the mainboard reads over a bit-banged I2C bus; see ultrasonic_nx100i.c. */
#define OPTION_ULTRASONIC 1
#define ULTRASONIC_TYPE ULTRASONIC_TYPE_NX100I_I2C
#else
#define OPTION_ULTRASONIC 0
#endif
/* OPTION_BUMPER 0 compiles HALLSTOP_{Left,Right}_Sense() to a constant 0
 * (main.c), so PD2/PD3 never influence drivemotor.c. */
#define OPTION_BUMPER 0

#if defined(BOARD_YARDFORCE_NX100I)
/* YardForce NX100i: same GForce STM32F401VC mainboard and pinout as the 500B
 * (verified live against the stock firmware, 2026-10-13), same PAC5210 /
 * PAC5223 UART protocols. Differences handled here:
 *  - PD2/PD3 hall-stop (bumper) lines are not connected on the NX100i and sit
 *    permanently HIGH on the board; they stay compiled out (OPTION_BUMPER 0).
 *    Collision sensing comes from the hood's ultrasonic board instead: an I2C
 *    slave (addr 0x51, "RM CSB") on PC10 SCL / PC11 SDA that reports two
 *    distances; ultrasonic_nx100i.c reads it and the values travel to the
 *    host in pkt_status.uss_ranges_m[0..1] (protocol reverse-engineered from
 *    the stock firmware, see docs in the PR).
 *  - The hood stop button has two contacts: PC0 is driven HIGH when pressed
 *    (works with the default pull-down); PC8 is a normally-closed contact to
 *    GND that OPENS when pressed, so it needs a pull-up to read 1 = pressed.
 *    With the pull-up, a disconnected hood cable also reads as pressed
 *    (fail-safe). Verified with the stock firmware's own pull configuration.
 *  - The NX100i LCD/18-button panel ("RM EC3") speaks its own protocol on
 *    USART1: panel_nx100i.c (PANEL_TYPE_YARDFORCE_NX100I) does the handshake
 *    and keepalive, forwards the panel's START/HOME as UI events, lets START
 *    release a latched emergency like the play button, and can push a message
 *    code to the LCD. */
#define STOP_BUTTON_WHITE_PULL GPIO_PULLUP
/* The J17/J18 accessory 5 V rail that powers an external IMU only comes up
 * once the motor controllers and TF4 have been initialised; it is not a
 * single GPIO (PD7, PD8, PE11, PE14, PE15 and PC5 were each tried alone and in
 * combination at reset-halt, 2026-10-15). An unpowered IMU module's pull-ups
 * hold J18 SCL/SDA low, so the stock boot order (IMU probe before the motor
 * init) spends its whole 20 s window on a dead bus and never finds the
 * sensor. main.c therefore probes the IMU after that init on this board. */
#define IMU_PROBE_AFTER_POWER_INIT 1
/* Hood ultrasonic board bus (bit-banged, open-drain). */
#define NX100I_USS_SCL_PIN GPIO_PIN_10
#define NX100I_USS_SCL_PORT GPIOC
#define NX100I_USS_SDA_PIN GPIO_PIN_11
#define NX100I_USS_SDA_PORT GPIOC
#define NX100I_USS_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
/* Dock handshake. The NX100i station holds a current-limited ~14 V probe on
 * its contacts (32 V open circuit) until the mower connects its battery bus;
 * the stock firmware treats the charger as present above ~13 V (ADC > 1000),
 * raises TF4 and starts the buck, after which the station ramps to full
 * voltage. Mowgli's 500B thresholds (connect >= 30 V, docked >= 20 V) would
 * never fire on this station. Charger topology, measurements and limits are
 * otherwise identical to the 500B (verified against the stock firmware). */
#define MIN_DOCKED_VOLTAGE 12.0f
#define CHARGER_CONNECT_VOLTAGE 12.0f
#endif

#define BOARD_HAS_MASTER_USART 0
#elif defined(BOARD_LUV1000RI) // TODO: This currently can't be selected via platformio
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_LUV1000RI
#define BLADEMOTOR_LENGTH_RECEIVED_MSG 14
#define BLADEMOTOR_STOP_COMMAND_VALUE 0x00u
#define BLADEMOTOR_FORWARD_COMMAND_VALUE 0x80u
#define BLADEMOTOR_REVERSE_COMMAND_VALUE 0xC0u
#define BLADEMOTOR_POLL_INTERVAL_MS 100u

#define DEBUG_TYPE 0

#define OPTION_ULTRASONIC 1
#define OPTION_BUMPER 0

#define MAX_MPS 0.5		  // Allow maximum speed of 1.0 m/s
#define PWM_PER_MPS 300.0 // PWM value of 300 means 1 m/s bot speed so we divide by 4 to have correct robot speed but still progressive speed
#define TICKS_PER_M 300.0 // Power-on fallback encoder ticks per meter; ROS runtime tuning overrides this after host connection
#define WHEEL_BASE 0.285   // The distance between the center of the wheels in meters

#define BOARD_HAS_MASTER_USART 0
#endif

// I_DONT_NEED_MY_FINGERS, when DEFINED (value is irrelevant — the guards use
// #ifdef/#ifndef), compiles out EmergencyController() so the firmware NEVER
// polls the physical stop button / wheel-lift / tilt sensors. That contradicts
// "the STM32 firmware is the sole blade safety authority", so it is left
// UNDEFINED to keep hardware e-stop sensing active.
// WARNING: before flashing to a robot, validate per-chassis that the stop-button,
// wheel-lift and tilt GPIOs are actually wired and not floating/noisy — otherwise
// EmergencyController() may latch a spurious emergency. Re-#define this only if a
// given chassis lacks those sensors.
// #define I_DONT_NEED_MY_FINGERS           1      // disables EmergencyController()

// Battery/charge envelope, emergency-sensor timeouts and the onboard-IMU tilt
#if (OPTION_ULTRASONIC == 1) && !defined(ULTRASONIC_TYPE)
#define ULTRASONIC_TYPE ULTRASONIC_TYPE_UART
#endif

// threshold are single-sourced in board_defaults.h (shared with board.h.template
// so CI and the GUI-flashed firmware can never drift). This file defines none of
// them itself; the #ifndef fallbacks in board_defaults.h supply the blessed
// values. To change one, edit board_defaults.h (NOT here).
#include "board_defaults.h"

// Enable Emergency debugging
//#define EMERGENCY_DEBUG

// IMU configuration options
#define EXTERNAL_IMU_ACCELERATION  1
#define EXTERNAL_IMU_ANGULAR       1

// Force disable IMU to be detected - CURRENTLY THIS SETTING DOES NOT WORK!
//#define DISABLE_LSM6
//#define DISABLE_MPU6050
//#define DISABLE_WT901

// we use J18 (Red 9 pin connector as Master Serial Port)
#define MASTER_J18 1

// enable Drive and Blade Motor UARTS
#define DRIVEMOTORS_USART_ENABLED 1
#define BLADEMOTOR_USART_ENABLED 1
#define PANEL_USART_ENABLED 1

// our IMU hangs of a bigbanged I2C bus on J18
#define SOFT_I2C_ENABLED 1

#define LED_PIN GPIO_PIN_2
#define LED_GPIO_PORT GPIOB
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

/* 24V Supply */
#define TF4_PIN GPIO_PIN_5
#define TF4_GPIO_PORT GPIOC
#define TF4_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

/* Blade Motor nRESET - (HIGH for no RESET) */
#define PAC5223RESET_PIN GPIO_PIN_14
#define PAC5223RESET_GPIO_PORT GPIOE
#define PAC5223RESET_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()

#if defined(BLADEMOTOR_SEQUENCED_POWER)
#define BLADEMOTOR_POWER_PIN GPIO_PIN_11
#define BLADEMOTOR_POWER_GPIO_PORT GPIOD
#define BLADEMOTOR_POWER_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#endif

/* Drive Motors - HC366 OE Pins (LOW to enable) */
#define PAC5210RESET_PIN GPIO_PIN_15
#define PAC5210RESET_GPIO_PORT GPIOE
#define PAC5210RESET_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()

/* Charge Control Pins - HighSide/LowSide MosFET */
#define CHARGE_LOWSIDE_PIN GPIO_PIN_8
#define CHARGE_HIGHSIDE_PIN GPIO_PIN_9
#define CHARGE_GPIO_PORT GPIOE
#define CHARGE_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE();

/* Stop button - (HIGH when pressed) */
#define STOP_BUTTON_YELLOW_PIN GPIO_PIN_0
#define STOP_BUTTON_YELLOW_PORT GPIOC
#define STOP_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define STOP_BUTTON_WHITE_PIN GPIO_PIN_8
#define STOP_BUTTON_WHITE_PORT GPIOC

/* Mechanical tilt - (HIGH when set) */
#define TILT_PIN GPIO_PIN_8
#define TILT_PORT GPIOA
#define TILT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* Wheel lift - (HIGH when set) */
#define WHEEL_LIFT_BLUE_PIN GPIO_PIN_0
#define WHEEL_LIFT_BLUE_PORT GPIOD
#define WHEEL_LIFT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define WHEEL_LIFT_RED_PIN GPIO_PIN_1
#define WHEEL_LIFT_RED_PORT GPIOD

/* Play button - (LOW when pressed) */
#if BOARD_YARDFORCE500_VARIANT_B
#define PLAY_BUTTON_PIN GPIO_PIN_9
#else
#define PLAY_BUTTON_PIN GPIO_PIN_7
#endif
#define PLAY_BUTTON_PORT GPIOC
#define PLAY_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

/* Home button - (LOW when pressed) */
#define HOME_BUTTON_PIN GPIO_PIN_13
#define HOME_BUTTON_PORT GPIOB
#define HOME_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()


/* Rain Sensor - (LOW when active) */
#define RAIN_SENSOR_PIN GPIO_PIN_2
#define RAIN_SENSOR_PORT GPIOE
#define RAIN_SENSOR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()

/* STOP HALL Sensor - (HIGH when set) */
#define HALLSTOP_RIGHT_PIN GPIO_PIN_2
#define HALLSTOP_LEFT_PIN GPIO_PIN_3
#define HALLSTOP_PORT GPIOD
#define HALLSTOP_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

#if BOARD_HAS_MASTER_USART
    /* either J6 or J18 can be the master USART port */
#ifdef MASTER_J6
/* USART1 (J6 Pin 1 (TX) Pin 2 (RX)) */
#define MASTER_USART_INSTANCE USART1
#define MASTER_USART_RX_PIN GPIO_PIN_10
#define MASTER_USART_RX_PORT GPIOA
#define MASTER_USART_TX_PIN GPIO_PIN_9
#define MASTER_USART_TX_PORT GPIOA
#define MASTER_USART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define MASTER_USART_USART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#define MASTER_USART_IRQ USART1_IRQn
#endif
#ifdef MASTER_J18
/* UART4 (J18 Pin 7 (TX) Pin 8 (RX)) */
#define MASTER_USART_INSTANCE UART4
#define MASTER_USART_RX_PIN GPIO_PIN_11
#define MASTER_USART_RX_PORT GPIOC
#define MASTER_USART_TX_PIN GPIO_PIN_10
#define MASTER_USART_TX_PORT GPIOC
#define MASTER_USART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define MASTER_USART_USART_CLK_ENABLE() __HAL_RCC_UART4_CLK_ENABLE()
#define MASTER_USART_IRQ UART4_IRQn
#endif
#endif

#ifdef DRIVEMOTORS_USART_ENABLED
/* drive motors PAC 5210 (USART2) */
#define DRIVEMOTORS_USART_INSTANCE USART2

#define DRIVEMOTORS_USART_RX_PIN GPIO_PIN_6
#define DRIVEMOTORS_USART_RX_PORT GPIOD

#define DRIVEMOTORS_USART_TX_PIN GPIO_PIN_5
#define DRIVEMOTORS_USART_TX_PORT GPIOD

#define DRIVEMOTORS_USART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define DRIVEMOTORS_USART_USART_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE()

#define DRIVEMOTORS_USART_IRQ USART2_IRQn
#define DRIVEMOTORS_MSG_LEN 12
#endif

#ifdef BLADEMOTOR_USART_ENABLED
#if BOARD_YARDFORCE500_VARIANT_ORIG
/* blade motor PAC 5223 (USART3) */
#define BLADEMOTOR_USART_RX_PIN GPIO_PIN_11
#define BLADEMOTOR_USART_RX_PORT GPIOB

#define BLADEMOTOR_USART_TX_PIN GPIO_PIN_10
#define BLADEMOTOR_USART_TX_PORT GPIOB

#define BLADEMOTOR_USART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BLADEMOTOR_USART_USART_CLK_ENABLE() __HAL_RCC_USART3_CLK_ENABLE()
#elif BOARD_YARDFORCE500_VARIANT_B
/* blade motor PAC 5223 (USART6) */
#define BLADEMOTOR_USART_RX_PIN GPIO_PIN_7
#define BLADEMOTOR_USART_RX_PORT GPIOC

#define BLADEMOTOR_USART_TX_PIN GPIO_PIN_6
#define BLADEMOTOR_USART_TX_PORT GPIOC

#define BLADEMOTOR_USART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define BLADEMOTOR_USART_USART_CLK_ENABLE() __HAL_RCC_USART6_CLK_ENABLE()
#endif
#endif

#ifdef PANEL_USART_ENABLED
#define PANEL_USART_INSTANCE USART1

#define PANEL_USART_RX_PIN GPIO_PIN_10
#define PANEL_USART_RX_PORT GPIOA

#define PANEL_USART_TX_PIN GPIO_PIN_9
#define PANEL_USART_TX_PORT GPIOA

#define PANEL_USART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define PANEL_USART_USART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#define PANEL_USART_IRQ USART1_IRQn
#endif

// J18 has the SPI3 pins, as we dont use SPI3, we recycle them for I2C Bitbanging (for our IMU)
#ifdef SOFT_I2C_ENABLED
#define SOFT_I2C_SCL_PIN GPIO_PIN_3
#define SOFT_I2C_SCL_PORT GPIOB
#define SOFT_I2C_SDA_PIN GPIO_PIN_4
#define SOFT_I2C_SDA_PORT GPIOB

#define SOFT_I2C_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE();
#endif

#if !VALID_BOARD_DEFINED
#error "No valid board has been defined, this likely is a mismatch between this file and platformio.ini"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
