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

    #define BOARD_YARDFORCE500_VARIANT_ORIG 1




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

#if BOARD_YARDFORCE500_VARIANT_ORIG
///////////////////////////
// Yardforce 500 CLASSIC //
///////////////////////////
#define BLADEMOTOR_USART_INSTANCE USART3

#define VALID_BOARD_DEFINED 1
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500_CLASSIC
#define BLADEMOTOR_LENGTH_RECEIVED_MSG 16
#define DEBUG_TYPE DEBUG_TYPE_UART

#define MAX_MPS 0.6          // Allow maximum speed of 1.0 m/s
#define PWM_PER_MPS 300.0 // PWM value of 300 means 1 m/s bot speed so we divide by 4 to have correct robot speed but still progressive speed
#define TICKS_PER_M 0 // Motor Encoder ticks per meter
#define WHEEL_BASE  0        // The distance between the center of the wheels in meters

#define OPTION_ULTRASONIC 0
#define OPTION_BUMPER 0

#define BOARD_HAS_MASTER_USART 1
#elif BOARD_YARDFORCE500_VARIANT_B
/////////////////////
// Yardforce 500 B //
/////////////////////
#define BLADEMOTOR_USART_INSTANCE USART6

#define VALID_BOARD_DEFINED 1
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500_CLASSIC
#define BLADEMOTOR_LENGTH_RECEIVED_MSG 16
#define DEBUG_TYPE DEBUG_TYPE_SWO

#define MAX_MPS 0.6          // Allow maximum speed of 1.0 m/s
#define PWM_PER_MPS 300.0 // PWM value of 300 means 1 m/s bot speed so we divide by 4 to have correct robot speed but still progressive speed
#define TICKS_PER_M 0 // Motor Encoder ticks per meter
#define WHEEL_BASE  0        // The distance between the center of the wheels in meters

#define OPTION_ULTRASONIC 0
#define OPTION_BUMPER 0

#define BOARD_HAS_MASTER_USART 0
#elif defined(BOARD_LUV1000RI)
/////////////////////////
// Yardforce LUV1000Ri //
/////////////////////////
#define VALID_BOARD_DEFINED 1
#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500_CLASSIC
#define BLADEMOTOR_LENGTH_RECEIVED_MSG 14
#define DEBUG_TYPE 0
#define OPTION_ULTRASONIC 1
#define OPTION_BUMPER 0

#define MAX_MPS 0.6          // Allow maximum speed of 1.0 m/s
#define PWM_PER_MPS 300.0 // PWM value of 300 means 1 m/s bot speed so we divide by 4 to have correct robot speed but still progressive speed
#define TICKS_PER_M 0 // Motor Encoder ticks per meter
#define WHEEL_BASE 0   // The distance between the center of the wheels in meters

#define BOARD_HAS_MASTER_USART 0
#else

#error "No valid board has been defined, this likely is a mismatch between this file and the board selection"
#endif





/// nominal max charge current is 1.0 Amp
#define MAX_CHARGE_CURRENT 1.50f
/// Voltage threshold for CC to CV transition
#define LIMIT_VOLTAGE_150MA 29.00f
/// Max voltage allowed
#define MAX_CHARGE_VOLTAGE 29.00f
/// Max battery voltage allowed
#define BAT_CHARGE_CUTOFF_VOLTAGE 29.00f
/// We consider the battery is full when in CV mode the current below 0.1A
#define CHARGE_END_LIMIT_CURRENT 0.08f
// if voltage is greater than this assume we are docked
#define MIN_DOCKED_VOLTAGE 20.0f
// if voltage is lower this assume battery is disconnected
#define MIN_BATTERY_VOLTAGE 5.0f

// if current is greater than this assume the battery is charging
#define MIN_CHARGE_CURRENT 0.1f
#define LOW_BAT_THRESHOLD 25.2f /* near 20% SOC */
#define LOW_CRI_THRESHOLD 23.5f /* near 0% SOC */

// Emergency sensor timeouts
#define ONE_WHEEL_LIFT_EMERGENCY_MILLIS 10000
#define BOTH_WHEELS_LIFT_EMERGENCY_MILLIS 100
#define TILT_EMERGENCY_MILLIS 1000 // used for both the mechanical and accelerometer based detection
#define STOP_BUTTON_EMERGENCY_MILLIS 100
#define PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS 1000
#define IMU_ONBOARD_INCLINATION_THRESHOLD 0x38 // stock firmware uses 0x2C (way more allowed inclination)

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
#error "No valid board has been defined, this likely is a mismatch between this file and the board selection"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
