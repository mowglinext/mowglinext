/**
 ******************************************************************************
 * @file    main.c
 * @author  Georg Swoboda <cn@warp.at>
 * @date    21/09/2022
 * @version 1.0.0
 * @brief   main / bootup and initialization, motor control routines, usb init
 ******************************************************************************
 *
 * compile with -DBOARD_YARDFORCE500 to enable the YF500 GForce pinout
 *
 * ROS integration howto taken from here: https://github.com/Itamare4/ROS_stm32f1_rosserial_USB_VCP (Itamar Eliakim)
 *
 ******************************************************************************
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include "stm32f_board_hal.h"
#include "main.h"
// stm32 custom
#include "board.h"
#include "panel.h"
#include "panel.h"
#include "blademotor.h"
#include "drivemotor.h"
#include "emergency.h"
#include "blademotor.h"
#include "drivemotor.h"
#include "ultrasonic_sensor.h"
#include "perimeter.h"
#include "adc.h"
#include "charger.h"
#include "soft_i2c.h"
#include "i2c.h"
#include "imu/imu.h"
#include "mowgli_protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "nbt.h"

// ros
#include "cpp_main.h"
#include "ringbuffer.h"

static void WATCHDOG_vInit(void);
static void WATCHDOG_Refresh(void);
static const char *WATCHDOG_StageName(uint8_t stage);
static uint8_t WATCHDOG_ReadPersistedStage(void);
static void WATCHDOG_WritePersistedStage(uint8_t stage);
static void WATCHDOG_LoadBootBreadcrumb(void);
void TIM4_Init(void);
void HALLSTOP_Sensor_Init(void);

static nbt_t main_chargecontroller_nbt;
static nbt_t main_statusled_nbt;
static nbt_t main_emergency_nbt;
static nbt_t main_blademotor_nbt;
static nbt_t main_drivemotor_nbt;
static nbt_t main_wdg_nbt;
static nbt_t main_buzzer_nbt;
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
static nbt_t main_ultrasonicsensor_nbt;
#endif
#if BOARD_HAS_MASTER_USART
volatile uint8_t master_tx_busy = 0;
static uint8_t master_tx_buffer_len;
static char master_tx_buffer[255];
#endif

uint8_t do_chirp_duration_counter;
uint8_t do_chirp = 0;

openmower_status_e main_eOpenmowerStatus = OPENMOWER_STATUS_IDLE;

#if BOARD_YARDFORCE500_VARIANT_ORIG
UART_HandleTypeDef MASTER_USART_Handler; // UART  Handle
#endif

// Drive Motors DMA
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
TIM_HandleTypeDef TIM3_Handle; // PWM Beeper
TIM_HandleTypeDef TIM4_Handle; // PWM Buzzer
IWDG_HandleTypeDef IwdgHandle = {0};
WWDG_HandleTypeDef WwdgHandle = {0};
static uint32_t g_boot_reset_csr = 0;
uint8_t g_boot_reset_cause_code = 0;
uint8_t g_boot_last_watchdog_stage_code = WATCHDOG_STAGE_NONE;
static volatile uint8_t g_main_loop_stage = WATCHDOG_STAGE_NONE;

static const char *BOOT_PrimaryResetCause(uint32_t csr)
{
#ifdef RCC_CSR_WWDGRSTF
  if ((csr & RCC_CSR_WWDGRSTF) != 0u)
  {
    return "WWDG";
  }
#endif
#ifdef RCC_CSR_IWDGRSTF
  if ((csr & RCC_CSR_IWDGRSTF) != 0u)
  {
    return "IWDG";
  }
#endif
#ifdef RCC_CSR_SFTRSTF
  if ((csr & RCC_CSR_SFTRSTF) != 0u)
  {
    return "SFTRST";
  }
#endif
#ifdef RCC_CSR_BORRSTF
  if ((csr & RCC_CSR_BORRSTF) != 0u)
  {
    return "BOR";
  }
#endif
#ifdef RCC_CSR_PORRSTF
  if ((csr & RCC_CSR_PORRSTF) != 0u)
  {
    return "POR/PDR";
  }
#endif
#ifdef RCC_CSR_PINRSTF
  if ((csr & RCC_CSR_PINRSTF) != 0u)
  {
    return "PIN";
  }
#endif
#ifdef RCC_CSR_LPWRRSTF
  if ((csr & RCC_CSR_LPWRRSTF) != 0u)
  {
    return "LPWR";
  }
#endif
  return "UNKNOWN";
}

static uint8_t BOOT_ResetCauseCode(uint32_t csr)
{
  /* LED code mapping used by BOOT_BlinkResetCause():
   * 1=PIN, 2=POR/PDR, 3=BOR, 4=SFTRST, 5=IWDG, 6=WWDG, 7=LPWR. */
#ifdef RCC_CSR_WWDGRSTF
  if ((csr & RCC_CSR_WWDGRSTF) != 0u)
  {
    return RESET_CAUSE_WWDG;
  }
#endif
#ifdef RCC_CSR_IWDGRSTF
  if ((csr & RCC_CSR_IWDGRSTF) != 0u)
  {
    return RESET_CAUSE_IWDG;
  }
#endif
#ifdef RCC_CSR_SFTRSTF
  if ((csr & RCC_CSR_SFTRSTF) != 0u)
  {
    return RESET_CAUSE_SFTRST;
  }
#endif
#ifdef RCC_CSR_BORRSTF
  if ((csr & RCC_CSR_BORRSTF) != 0u)
  {
    return RESET_CAUSE_BOR;
  }
#endif
#ifdef RCC_CSR_PORRSTF
  if ((csr & RCC_CSR_PORRSTF) != 0u)
  {
    return RESET_CAUSE_POR_PDR;
  }
#endif
#ifdef RCC_CSR_PINRSTF
  if ((csr & RCC_CSR_PINRSTF) != 0u)
  {
    return RESET_CAUSE_PIN;
  }
#endif
#ifdef RCC_CSR_LPWRRSTF
  if ((csr & RCC_CSR_LPWRRSTF) != 0u)
  {
    return RESET_CAUSE_LPWR;
  }
#endif
  return RESET_CAUSE_UNKNOWN;
}

static void BOOT_PrintResetFlagIfSet(uint32_t csr, uint32_t mask, const char *name, bool *first)
{
  if ((csr & mask) == 0u)
  {
    return;
  }
  if (!*first)
  {
    DB_TRACE("|");
  }
  DB_TRACE("%s", name);
  *first = false;
}

static void BOOT_LogResetCause(uint32_t csr)
{
  bool first = true;
  const uint8_t code = BOOT_ResetCauseCode(csr);
  DB_TRACE(" * Reset cause: CSR=0x%08lx primary=%s led_code=%u flags=[",
           (unsigned long)csr,
           BOOT_PrimaryResetCause(csr),
           (unsigned int)code);
#ifdef RCC_CSR_LPWRRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_LPWRRSTF, "LPWR", &first);
#endif
#ifdef RCC_CSR_WWDGRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_WWDGRSTF, "WWDG", &first);
#endif
#ifdef RCC_CSR_IWDGRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_IWDGRSTF, "IWDG", &first);
#endif
#ifdef RCC_CSR_SFTRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_SFTRSTF, "SFTRST", &first);
#endif
#ifdef RCC_CSR_BORRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_BORRSTF, "BOR", &first);
#endif
#ifdef RCC_CSR_PORRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_PORRSTF, "POR/PDR", &first);
#endif
#ifdef RCC_CSR_PINRSTF
  BOOT_PrintResetFlagIfSet(csr, RCC_CSR_PINRSTF, "PIN", &first);
#endif
  if (first)
  {
    DB_TRACE("none");
  }
  DB_TRACE("]\r\n");
}

static void BOOT_ClearResetFlags(void)
{
#ifdef RCC_CSR_RMVF
  RCC->CSR |= RCC_CSR_RMVF;
#endif
}

static void BOOT_BlinkResetCause(uint32_t csr)
{
  const uint8_t code = BOOT_ResetCauseCode(csr);
  if (code == 0u)
  {
    return;
  }

  for (uint8_t i = 0; i < code; ++i)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(80);
  }
}

void WATCHDOG_SetMainLoopStage(uint8_t stage)
{
  g_main_loop_stage = stage;
}

static const char *WATCHDOG_StageName(uint8_t stage)
{
  switch (stage)
  {
    case WATCHDOG_STAGE_NONE:
      return "NONE";
    case WATCHDOG_STAGE_CHATTER:
      return "CHATTER";
    case WATCHDOG_STAGE_MOTORS:
      return "MOTORS";
    case WATCHDOG_STAGE_PANEL:
      return "PANEL";
    case WATCHDOG_STAGE_ROS_SPIN:
      return "ROS_SPIN";
    case WATCHDOG_STAGE_BROADCAST:
      return "BROADCAST";
    case WATCHDOG_STAGE_DRIVEMOTOR_RX:
      return "DRIVEMOTOR_RX";
    case WATCHDOG_STAGE_PERIMETER:
      return "PERIMETER";
    case WATCHDOG_STAGE_ADC:
      return "ADC";
    case WATCHDOG_STAGE_CHARGER:
      return "CHARGER";
    case WATCHDOG_STAGE_STATUS_LED:
      return "STATUS_LED";
    case WATCHDOG_STAGE_ULTRASONIC_HANDLER:
      return "ULTRASONIC_HANDLER";
    case WATCHDOG_STAGE_ULTRASONIC_APP:
      return "ULTRASONIC_APP";
    case WATCHDOG_STAGE_WATCHDOG_REFRESH:
      return "WATCHDOG_REFRESH";
    case WATCHDOG_STAGE_DRIVEMOTOR_10MS:
      return "DRIVEMOTOR_10MS";
    case WATCHDOG_STAGE_BLADEMOTOR:
      return "BLADEMOTOR";
    case WATCHDOG_STAGE_BUZZER:
      return "BUZZER";
    case WATCHDOG_STAGE_EMERGENCY:
      return "EMERGENCY";
    case WATCHDOG_STAGE_BROADCAST_ENTER:
      return "BROADCAST_ENTER";
    case WATCHDOG_STAGE_BROADCAST_IMU_BUILD:
      return "BROADCAST_IMU_BUILD";
    case WATCHDOG_STAGE_BROADCAST_IMU_SEND:
      return "BROADCAST_IMU_SEND";
    case WATCHDOG_STAGE_BROADCAST_RESET_SEND:
      return "BROADCAST_RESET_SEND";
    case WATCHDOG_STAGE_BROADCAST_STATUS_SEND:
      return "BROADCAST_STATUS_SEND";
    case WATCHDOG_STAGE_BROADCAST_BLADE_SEND:
      return "BROADCAST_BLADE_SEND";
    case WATCHDOG_STAGE_BROADCAST_EXIT:
      return "BROADCAST_EXIT";
    case WATCHDOG_STAGE_CDC_TX_ENTER:
      return "CDC_TX_ENTER";
    case WATCHDOG_STAGE_CDC_TX_QUEUE:
      return "CDC_TX_QUEUE";
    case WATCHDOG_STAGE_CDC_TX_RESUME:
      return "CDC_TX_RESUME";
    case WATCHDOG_STAGE_CDC_TX_EXIT:
      return "CDC_TX_EXIT";
    case WATCHDOG_STAGE_IMU_ACCEL:
      return "IMU_ACCEL";
    case WATCHDOG_STAGE_IMU_GYRO:
      return "IMU_GYRO";
    case WATCHDOG_STAGE_IMU_MAG:
      return "IMU_MAG";
    case WATCHDOG_STAGE_IMU_PACKET_FILL:
      return "IMU_PACKET_FILL";
    case WATCHDOG_STAGE_USB_IRQ_ENTER:
      return "USB_IRQ_ENTER";
    case WATCHDOG_STAGE_USB_IRQ_EXIT:
      return "USB_IRQ_EXIT";
    case WATCHDOG_STAGE_CDC_RX_ENTER:
      return "CDC_RX_ENTER";
    case WATCHDOG_STAGE_CDC_RX_PROCESS:
      return "CDC_RX_PROCESS";
    case WATCHDOG_STAGE_CDC_RX_EXIT:
      return "CDC_RX_EXIT";
    case WATCHDOG_STAGE_CDC_TX_PACKET:
      return "CDC_TX_PACKET";
    case WATCHDOG_STAGE_CDC_TX_COMPLETE:
      return "CDC_TX_COMPLETE";
    case WATCHDOG_STAGE_USB_RESET:
      return "USB_RESET";
    case WATCHDOG_STAGE_USB_SUSPEND:
      return "USB_SUSPEND";
    case WATCHDOG_STAGE_USB_RESUME:
      return "USB_RESUME";
    case WATCHDOG_STAGE_CDC_TX_PACKET_FAIL:
      return "CDC_TX_PACKET_FAIL";
    case WATCHDOG_STAGE_CDC_TX_BUSY_STUCK:
      return "CDC_TX_BUSY_STUCK";
    case WATCHDOG_STAGE_CDC_TX_QUEUE_FULL:
      return "CDC_TX_QUEUE_FULL";
    case WATCHDOG_STAGE_CDC_HOST_CLOSED:
      return "CDC_HOST_CLOSED";
    default:
      return "UNKNOWN";
  }
}

static uint8_t WATCHDOG_ReadPersistedStage(void)
{
#if BOARD_YARDFORCE500_VARIANT_ORIG
  return (uint8_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5) & 0xFFu);
#elif BOARD_YARDFORCE500_VARIANT_B
  uint32_t backup_register = (uint32_t)&(RTC->BKP0R);
  backup_register += (RTC_BKP_DR5 * 4u);
  return (uint8_t)(*(__IO uint32_t *)backup_register & 0xFFu);
#else
  return WATCHDOG_STAGE_NONE;
#endif
}

static void WATCHDOG_WritePersistedStage(uint8_t stage)
{
  HAL_PWR_EnableBkUpAccess();
#if BOARD_YARDFORCE500_VARIANT_ORIG
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, stage);
#elif BOARD_YARDFORCE500_VARIANT_B
  uint32_t backup_register = (uint32_t)&(RTC->BKP0R);
  backup_register += (RTC_BKP_DR5 * 4u);
  *(__IO uint32_t *)backup_register = (uint32_t)stage;
#else
  (void)stage;
#endif
}

static void WATCHDOG_LoadBootBreadcrumb(void)
{
  if (g_boot_reset_cause_code == RESET_CAUSE_WWDG)
  {
    g_boot_last_watchdog_stage_code = WATCHDOG_ReadPersistedStage();
    DB_TRACE(" * Last stage before WWDG reset: %s (%u)\r\n",
             WATCHDOG_StageName(g_boot_last_watchdog_stage_code),
             (unsigned int)g_boot_last_watchdog_stage_code);
  }
  else
  {
    g_boot_last_watchdog_stage_code = WATCHDOG_STAGE_NONE;
  }

  WATCHDOG_WritePersistedStage(WATCHDOG_STAGE_NONE);
}

#if DB_ACTIVE
int debug_assert(int condition, const char *msg)
{
  if (condition){
    return 0;
  }
  debug_printf(msg);
  return 1;
}
#endif

int main(void)
{
  g_boot_reset_csr = RCC->CSR;
  g_boot_reset_cause_code = BOOT_ResetCauseCode(g_boot_reset_csr);

  HAL_Init();
  SystemClock_Config();
  BOOT_ClearResetFlags();

#if BOARD_YARDFORCE500_VARIANT_ORIG
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
#endif

  MX_DMA_Init();

#if BOARD_HAS_MASTER_USART
  // Init debug USART
  MASTER_USART_Init();
#endif

  BOOT_LogResetCause(g_boot_reset_csr);

  DB_TRACE("\r\n");
  DB_TRACE("    __  ___                    ___\r\n");
  DB_TRACE("   /  |/  /___ _      ______ _/ (_)\r\n");
  DB_TRACE("  / /|_/ / __ \\ | /| / / __ `/ / / \r\n");
  DB_TRACE(" / /  / / /_/ / |/ |/ / /_/ / / /  \r\n");
  DB_TRACE("/_/  /_/\\____/|__/|__/\\__, /_/_/   \r\n");
  DB_TRACE("                     /____/        \r\n");
  DB_TRACE("\r\n\r\n");
  DB_TRACE(" * Master USART (debug) initialized\r\n");
  LED_Init();
  BOOT_BlinkResetCause(g_boot_reset_csr);
  DB_TRACE(" * LED initialized\r\n");
  TIM2_Init();
  ADC_Charging_Init();
  WATCHDOG_LoadBootBreadcrumb();
#ifdef OPTION_PERIMETER
  Perimeter_vInit();
#endif
  DB_TRACE(" * Charging ADC initialized\r\n");
  TIM3_Init();
  HAL_TIM_PWM_Start(&TIM3_Handle, TIM_CHANNEL_4);
  TIM4_Init();
  HAL_TIM_PWM_Start(&TIM4_Handle, TIM_CHANNEL_3);
  DB_TRACE(" * Timer3 (Beeper) initialized\r\n");
  TF4_Init();
  DB_TRACE(" * 24V switched on\r\n");
  RAIN_Sensor_Init();
  DB_TRACE(" * RAIN Sensor enabled\r\n");
  HALLSTOP_Sensor_Init();
  DB_TRACE(" * HALL Sensor enabled\r\n");

  I2C_Init();
  DB_TRACE(" * Hard I2C initialized\r\n");
  if (I2C_Acclerometer_TestDevice())
  {
    I2C_Accelerometer_Setup();
  }
  else
  {
    chirp(3);
    DB_TRACE("\e[01;31m * WARNING: initalization of onboard accelerometer for tilt protection failed !\e[0m\r\n");
  }

  DB_TRACE(" * Accelerometer (onboard/tilt safety) initialized\r\n");
  SW_I2C_Init();
  DB_TRACE(" * Soft I2C (J18) initialized\r\n");
  DB_TRACE(" * Testing supported IMUs:\r\n");
  IMU_Init();
  // IMU calibration is handled on the ROS2 side (hardware_bridge_node)
  // when the robot is on the dock.
  PANEL_Init();
  DB_TRACE(" * Panel initialized\r\n");
  Emergency_Init();
  DB_TRACE(" * Emergency sensors initialized\r\n");
  TIM1_Init();
  DB_TRACE(" * Timer1 (Charge PWM) initialized\r\n");
  MX_USB_DEVICE_Init();
  DB_TRACE(" * USB CDC initialized\r\n");
// Init Drive Motors and Blade Motor
#ifdef DRIVEMOTORS_USART_ENABLED
  DRIVEMOTOR_Init();
  DB_TRACE(" * Drive Motors USART initialized\r\n");
#endif
#ifdef BLADEMOTOR_USART_ENABLED
  BLADEMOTOR_Init();
#endif
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
  ULTRASONICSENSOR_Init();
#endif

  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, 0);
  HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1);

  // Initialize Main Timers
  NBT_init(&main_chargecontroller_nbt, 10);
  NBT_init(&main_statusled_nbt, 1000);
  NBT_init(&main_emergency_nbt, 10);
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
  NBT_init(&main_ultrasonicsensor_nbt, 50);
#endif
  NBT_init(&main_blademotor_nbt, 100);
  NBT_init(&main_drivemotor_nbt, 20);
  NBT_init(&main_wdg_nbt, 10);
  NBT_init(&main_buzzer_nbt, 200);

  DB_TRACE(" * NBT Main timers initialized\r\n");

#ifdef I_DONT_NEED_MY_FINGERS
  DB_TRACE("\r\n\e[01;31m");
  DB_TRACE("=========================================================\r\n");
  DB_TRACE("   EMERGENCY/SAFETY FEATURES ARE DISABLED IN board.h ! \r\n");
  DB_TRACE("=========================================================\r\n");
  DB_TRACE("\e[0m\r\n");
#endif
  // Initialize ROS
  init_ROS();
  DB_TRACE(" * ROS serial node initialized\r\n");
  DB_TRACE("\r\n\e[01;36m >>> entering main loop ...\e[0m\r\n\r\n");
  // <chirp><chirp> means we are in the main loop
  chirp(2);

  WATCHDOG_vInit();
  WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_NONE);

  while (1)
  {
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_CHATTER);
    chatter_handler();
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_MOTORS);
    motors_handler();
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_PANEL);
    panel_handler();
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_ROS_SPIN);
    spinOnce();
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST);
    broadcast_handler();

    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_DRIVEMOTOR_RX);
    DRIVEMOTOR_App_Rx();
#ifdef OPTION_PERIMETER
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_PERIMETER);
    Perimeter_vApp();
#endif

    if (NBT_handler(&main_chargecontroller_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_ADC);
      ADC_input();
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_CHARGER);
      ChargeController();
    }
    if (NBT_handler(&main_statusled_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_STATUS_LED);
      StatusLEDUpdate();

      // DB_TRACE("master_rx_STATUS: %d  drivemotors_rx_buf_idx: %d  cnt_usart2_overrun: %x\r\n", master_rx_STATUS, drivemotors_rx_buf_idx, cnt_usart2_overrun);
    }
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
    /* try to send ros message without delay*/
    if (ULTRASONIC_MessageReceived() == 1)
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_ULTRASONIC_HANDLER);
      ultrasonic_handler();
    }
    if (NBT_handler(&main_ultrasonicsensor_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_ULTRASONIC_APP);
      ULTRASONICSENSOR_App();
    }
#endif
    if (NBT_handler(&main_wdg_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_WATCHDOG_REFRESH);
      WATCHDOG_Refresh();
    }

    if (NBT_handler(&main_drivemotor_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_DRIVEMOTOR_10MS);
      DRIVEMOTOR_App_10ms();
    }

    if (NBT_handler(&main_blademotor_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BLADEMOTOR);
      BLADEMOTOR_App();
    }

    if (NBT_handler(&main_buzzer_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BUZZER);
      // TODO
      if (do_chirp)
      {
        TIM3_Handle.Instance->CCR4 = 10; // chirp on
        TIM4_Handle.Instance->CCR3 = 10; // chirp on
        do_chirp = 0;
        do_chirp_duration_counter = 0;
      }
      if (do_chirp_duration_counter == 1)
      {
        TIM3_Handle.Instance->CCR4 = 0; // chirp off
        TIM4_Handle.Instance->CCR3 = 0; // chirp off
      }
      do_chirp_duration_counter++;
    }

#ifndef I_DONT_NEED_MY_FINGERS
    if (NBT_handler(&main_emergency_nbt))
    {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_EMERGENCY);
      EmergencyController();
    }
#endif
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // we never get here ...
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

#if BOARD_HAS_MASTER_USART
// The STM32f1 has enough USARTs to use one for debugging

/**
 * @brief Init the Master Serial Port  - this what connects to the upstream controller
 * @retval None
 */
void MASTER_USART_Init()
{
  // enable port and usart clocks
  MASTER_USART_GPIO_CLK_ENABLE();
  MASTER_USART_USART_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  // RX
  GPIO_InitStruct.Pin = MASTER_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(MASTER_USART_RX_PORT, &GPIO_InitStruct);

  // TX
  GPIO_InitStruct.Pin = MASTER_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(MASTER_USART_TX_PORT, &GPIO_InitStruct);

  MASTER_USART_Handler.Instance = MASTER_USART_INSTANCE;     // USART1 (DEV)
  MASTER_USART_Handler.Init.BaudRate = 115200;               // Baud rate
  MASTER_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
  MASTER_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
  MASTER_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
  MASTER_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
  MASTER_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode

  HAL_UART_Init(&MASTER_USART_Handler); // HAL_UART_Init() Will enable  UART1

  /* UART4 DMA Init */
  /* UART4_RX Init */
  hdma_uart4_rx.Instance = DMA2_Channel3;
  hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart4_rx.Init.Mode = DMA_NORMAL;
  hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&MASTER_USART_Handler, hdmarx, hdma_uart4_rx);

  /* UART4 DMA Init */
  /* UART4_TX Init */
  hdma_uart4_tx.Instance = DMA2_Channel5;
  hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart4_tx.Init.Mode = DMA_NORMAL;
  hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&MASTER_USART_Handler, hdmatx, hdma_uart4_tx);

  // enable IRQ
  HAL_NVIC_SetPriority(MASTER_USART_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(MASTER_USART_IRQ);

  __HAL_UART_ENABLE_IT(&MASTER_USART_Handler, UART_IT_TC);
}
#endif

/**
 * @brief Init LED
 * @retval None
 */
void LED_Init()
{
  LED_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Poll RAIN Sensor
 * @retval 1 if rain is detected, 0 if no rain
 */
int RAIN_Sense(void)
{
  return (!HAL_GPIO_ReadPin(RAIN_SENSOR_PORT, RAIN_SENSOR_PIN)); // pullup, active low
}

/**
 * @brief Init RAIN Sensor (PE2) Input
 * @retval None
 */
void RAIN_Sensor_Init()
{
  RAIN_SENSOR_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = RAIN_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(RAIN_SENSOR_PORT, &GPIO_InitStruct);
}

/**
 * @brief Init HALL STOP Sensor (PD2&3) Inputs
 * @retval None
 */
void HALLSTOP_Sensor_Init()
{
  HALLSTOP_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(HALLSTOP_PORT, &GPIO_InitStruct);
}

/**
 * @brief Poll HALLSTOP_Left_Sense Sensor
 * @retval  1 if trigger , 0 if no stop sensor trigger
 */
int HALLSTOP_Left_Sense(void)
{
#if OPTION_BUMPER == 1
  return (HAL_GPIO_ReadPin(HALLSTOP_PORT, GPIO_PIN_2));
#else
  return 0;
#endif
}

/**
 * @brief Poll HALLSTOP_Right_Sense
 * @retval 1 if trigger , 0 if no stop sensor trigger
 */
int HALLSTOP_Right_Sense(void)
{
#if OPTION_BUMPER == 1
  return (HAL_GPIO_ReadPin(HALLSTOP_PORT, GPIO_PIN_3));
#else
  return 0;
#endif
}

/**
 * @brief Init TF4 (24V Power Switch)
 * @retval None
 */
void TF4_Init()
{
  TF4_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = TF4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(TF4_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    DB_TRACE("Error Handler reached, oops\r\n");
    chirp(1);
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

#if BOARD_YARDFORCE500_VARIANT_B
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
#endif
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#if BOARD_YARDFORCE500_VARIANT_ORIG
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
#endif
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if BOARD_YARDFORCE500_VARIANT_ORIG
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
#elif BOARD_YARDFORCE500_VARIANT_B
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 6;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

#if BOARD_YARDFORCE500_VARIANT_ORIG
  // TODO: Is something like this needed for variant B?
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_RTC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/**
 * @brief TIM3 Initialization Function
 *
 * Beeper is on PB1 (PWM)
 *
 * @param None
 * @retval None
 */
void TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM3_Handle.Instance = TIM3;
  TIM3_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM3_Handle.Init.Period = 50;
  TIM3_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM3_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM3_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM3_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM3_Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
#if BOARD_YARDFORCE500_VARIANT_B
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
#endif
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief TIM4 Initialization Function
 *
 * Buzzer is on PD14 (PWM)
 *
 * @param None
 * @retval None
 */
void TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  TIM4_Handle.Instance = TIM4;
  TIM4_Handle.Init.Prescaler = 36000; // 72Mhz -> 2khz
  TIM4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM4_Handle.Init.Period = 50;
  TIM4_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM4_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM4_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&TIM4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM4_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM4_Handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

#if BOARD_YARDFORCE500_VARIANT_ORIG
  // TODO: check if something equivalent is needed for the STM32f4
  __HAL_AFIO_REMAP_TIM4_ENABLE(); // to use PD14 it is a full remap
#endif

  __HAL_RCC_GPIOD_CLK_ENABLE();
  /**TIM4 GPIO Configuration
  PD14    ------> TIM4_CH3
  */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
#if BOARD_YARDFORCE500_VARIANT_B
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
#endif
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

#if BOARD_YARDFORCE500_VARIANT_ORIG
  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration (DRIVE MOTORS)  */

  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

  /* DMA1_Channel2_IRQn interrupt configuration  (BLADE MOTOR)  */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration (BLADE MOTOR) */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
#elif BOARD_YARDFORCE500_VARIANT_B
  /* DMA interrupt init */

  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

#endif
}

/*
 * Update the states for the Emergency, Charge and Low Bat LEDs
 */
void StatusLEDUpdate(void)
{
  if (Emergency_State())
  {
    DB_TRACE("\e[2K\e[01;31m EMERGENCY WAS TRIGGERED !\e[0m\r\n");
    PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_FLASH_FAST);
  }
  else
  {
    PANEL_Set_LED(PANEL_LED_LIFTED, PANEL_LED_OFF);
  }

  if (chargecontrol_is_charging == 1) // Connected to charger
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_ON);
  }
  else if (chargecontrol_is_charging == 2) // CC mode 1A
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_FAST);
  }
  else if (chargecontrol_is_charging == 3) // CV mode
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_FLASH_SLOW);
  }
  else
  {
    PANEL_Set_LED(PANEL_LED_CHARGING, PANEL_LED_OFF);
  }

  // show a lowbat warning if battery voltage drops below LOW_BAT_THRESHOLD ? (random guess, needs more testing or a compare to the stock firmware)
  if (battery_voltage <= LOW_CRI_THRESHOLD)
  {
    PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_FAST); // low
  }
  else if (battery_voltage <= LOW_BAT_THRESHOLD)
  {
    PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_FLASH_SLOW); // really low
  }
  else
  {
    PANEL_Set_LED(PANEL_LED_BATTERY_LOW, PANEL_LED_OFF); // bat ok
  }

  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); // flash LED
}

/*
 * print hex bytes
 */
void msgPrint(uint8_t *msg, uint8_t msg_len)
{
  int i;
  DB_TRACE("msg: ");
  for (i = 0; i < msg_len; i++)
  {
    DB_TRACE(" %02x", msg[i]);
  }
  DB_TRACE("\r\n");
}

/*
 * calc crc byte
 */
uint8_t crcCalc(const uint8_t *msg, uint8_t msg_len)
{
  uint8_t crc = 0x0;
  uint8_t i;

  for (i = 0; i < msg_len; i++)
  {
    crc += msg[i];
  }
  return (crc);
}

/*
 * 2khz chirps
 */
void chirp(uint8_t count)
{
  uint8_t i;

  for (i = 0; i < count; i++)
  {
    TIM3_Handle.Instance->CCR4 = 10;
    TIM4_Handle.Instance->CCR3 = 10;
    HAL_Delay(100);
    TIM3_Handle.Instance->CCR4 = 0;
    TIM4_Handle.Instance->CCR3 = 0;
    HAL_Delay(50);
  }
}

/*
 * Debug print via MASTER USART
 */
void vprint(const char *fmt, va_list argp)
{
  char string[200];
  if (0 < vsnprintf(string, sizeof(string), fmt, argp)) // build string
  {
#if DEBUG_TYPE == DEBUG_TYPE_SWO
    for (int i = 0; i < strlen(string); i++)
    {
      ITM_SendChar(string[i]);
    }
#elif DEBUG_TYPE == DEBUG_TYPE_UART
#if BOARD_YARDFORCE500_VARIANT_ORIG
    MASTER_Transmit((unsigned char *)string, strlen(string));
#else
#error "This board does not suport debugging via UART"
#endif
#endif
  }
}

/*
 * Debug print
 */
void debug_printf(const char *fmt, ...)
{
  va_list argp;
  va_start(argp, fmt);
  vprint(fmt, argp);
  va_end(argp);
}

#if BOARD_HAS_MASTER_USART
/*
 * Send message via MASTER USART (DMA Normal Mode).
 * Debug output is strictly best-effort: if a previous DMA transfer is still in
 * flight, drop this message immediately so logging can never block the main
 * loop or delay the watchdog refresh.
 */
void MASTER_Transmit(uint8_t *buffer, uint8_t len)
{
  if (master_tx_busy)
  {
    return;
  }

  master_tx_busy = 1;
  // copy into our master_tx_buffer
  master_tx_buffer_len = len;
  memcpy(master_tx_buffer, buffer, master_tx_buffer_len);
  if (HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t *)master_tx_buffer, master_tx_buffer_len) != HAL_OK)
  {
    master_tx_busy = 0;
  }
}
#endif

/*
 * Initialize Watchdog - not tested yet (by Nekraus)
 */
static void WATCHDOG_vInit(void)
{
#if defined(DB_ACTIVE)
  /* setup DBGMCU block - stop IWDG at break in debug mode */
  __HAL_FREEZE_IWDG_DBGMCU();
#endif /* DB_ACTIVE */

  /* change the period to 50ms */
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload = 0xFFF;
  /* Enable IWDG (LSI automatically enabled by HW) */

  /* if window feature is not applied Init() precedes Start() */
  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
#ifdef DB_ACTIVE
    DB_TRACE(" IWDG init Error\r\n");
#endif /* DB_ACTIVE */
  }

#if defined(DB_ACTIVE)
  /* setup DBGMCU block - stop WWDG at break in debug mode */
  __HAL_FREEZE_WWDG_DBGMCU();
#endif /* DB_ACTIVE */

  /* Setup period - 20ms */
  __WWDG_CLK_ENABLE();
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Counter = 0x7F; /* 40.02 ms*/
  WwdgHandle.Init.Window = 0x7F;  /* 0ms */
  WwdgHandle.Init.EWIMode = WWDG_EWI_ENABLE;
  HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(WWDG_IRQn);
  if( HAL_WWDG_Init(&WwdgHandle) != HAL_OK )
  {
#ifdef DB_ACTIVE
    DB_TRACE(" WWDG init Error\r\n");
#endif /* DB_ACTIVE */
  }
} /* WATCHDOG_vInit() */

/*
 * Feed the watchdog every 10ms
 */
static void WATCHDOG_Refresh(void)
{
  /* Update WWDG counter */
  WwdgHandle.Instance = WWDG;
  if (HAL_WWDG_Refresh(&WwdgHandle) != HAL_OK)
  {
#ifdef DB_ACTIVE
    DB_TRACE(" WWDG refresh error\r\n");
#endif /* DB_ACTIVE */
  }
  /* Reload IWDG counter */
  IwdgHandle.Instance = IWDG;
  if (HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
  {
#ifdef DB_ACTIVE
    DB_TRACE(" IWDG refresh error\r\n");
#endif /* DB_ACTIVE */
  }
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
  (void)hwwdg;
  WATCHDOG_WritePersistedStage((uint8_t)g_main_loop_stage);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  // do nothing here
}

/*
 * called when DMA transfer completes
 * update <xxxx>_tx_busy to let XXXX_Transmit function now when the DMA buffer is free
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if BOARD_HAS_MASTER_USART
  if (huart->Instance == MASTER_USART_INSTANCE)
  {
    /* HAL invokes this callback on TX completion, so release the busy flag
     * unconditionally. It was previously gated on a re-read of USART_FLAG_TC;
     * when TC had not yet latched at callback time the flag was left set
     * forever, so the next MASTER_Transmit() spun in its busy-wait, starved the
     * main loop, and the window watchdog reset the board. */
    master_tx_busy = 0;
  }
#endif
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  // do nothing here
}

/*
 * Master UART receive ISR
 * DriveMotors UART receive ISR
 * PANEL UART receive ISR
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if BOARD_HAS_MASTER_USART
  if (huart->Instance == MASTER_USART_INSTANCE)
  {
#if (DEBUG_TYPE != DEBUG_TYPE_UART) && (OPTION_ULTRASONIC == 1)
    ULTRASONICSENSOR_ReceiveIT();
#endif
  }
  else
#endif
  if (huart->Instance == BLADEMOTOR_USART_INSTANCE)
  {
    BLADEMOTOR_ReceiveIT();
  }
  else if (huart->Instance == DRIVEMOTORS_USART_INSTANCE)
  {
    DRIVEMOTOR_ReceiveIT();
  }
}
