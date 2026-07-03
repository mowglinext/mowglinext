/****************************************************************************
* Title                 :   charger module
* Filename              :   charger.c
* Author                :   Nekraus
* Origin Date           :   01/04/2023
* Version               :   1.0.0

*****************************************************************************/
/** \file charger.c
 *  \brief
 *
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include "main.h"
#include "board.h"
#include "adc.h"
#include "charger.h"
/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/

typedef enum{
    CHARGER_STATE_IDLE,
    CHARGER_STATE_CONNECTED,
    CHARGER_STATE_CHARGING_CC,
    CHARGER_STATE_CHARGING_CV,
    CHARGER_STATE_END_CHARGING,
} CHARGER_STATE_e;

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/

TIM_HandleTypeDef TIM1_Handle;  // PWM Charge Controller

float SOC                           = 0;
uint16_t chargecontrol_pwm_val      = 0;
uint8_t  chargecontrol_is_charging  = 0;

static CHARGER_STATE_e charger_state = CHARGER_STATE_IDLE;
static float charge_end_voltage=BAT_CHARGE_CUTOFF_VOLTAGE ;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 *  Public Functions
 *******************************************************************************/


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
 void TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM1_Handle.Instance = TIM1;
  TIM1_Handle.Init.Prescaler = 0;
  TIM1_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM1_Handle.Init.Period = 1400;
  TIM1_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM1_Handle.Init.RepetitionCounter = 0;
  TIM1_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&TIM1_Handle) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM1_Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM1_Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&TIM1_Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handle, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  GPIO_InitTypeDef GPIO_InitStruct = {0};  
  CHARGE_GPIO_CLK_ENABLE();
  /** TIM1 GPIO Configuration
  PA7 or PE8     -----> TIM1_CH1N
  PA8 oe PE9    ------> TIM1_CH1
  */
  GPIO_InitStruct.Pin = CHARGE_LOWSIDE_PIN|CHARGE_HIGHSIDE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
#if BOARD_YARDFORCE500_VARIANT_B
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
#endif
  HAL_GPIO_Init(CHARGE_GPIO_PORT, &GPIO_InitStruct);

#if BOARD_YARDFORCE500_VARIANT_ORIG
  // TODO: Is something equivalent needed for the STM32f4?
  __HAL_AFIO_REMAP_TIM1_ENABLE();        // to use PE8/9 it is a full remap
#endif


    // Charge CH1/CH1N PWM Timer
  TIM1->CCR1 = 0;  
  HAL_TIM_PWM_Start(&TIM1_Handle, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&TIM1_Handle, TIM_CHANNEL_1);
  DB_TRACE(" * Charge Controler PWM Timers initialized\r\n");
}

 void charger_set_end_voltage(float v) {
    /* Limit input to reasonable values. */
    if (v>MAX_CHARGE_VOLTAGE) {
      v=MAX_CHARGE_VOLTAGE;
    } else if (v<LOW_BAT_THRESHOLD) {
      v=LOW_BAT_THRESHOLD;
    }
    /* Go back to constant current, if voltage is increased. */
    if (v>charge_end_voltage && charger_state==CHARGER_STATE_CHARGING_CV) {
      charger_state=CHARGER_STATE_CHARGING_CC;
    }
    charge_end_voltage=v;
 }

/*
 * manages the charge voltage, and charge, lowbat LED
 * improvementt need to be done to avoid sparks when connected charger and disconnected 
 * todo PID current measure
 * needs to be called frequently
 */
void ChargeController(void)
{                        
  static uint32_t timestamp = 0;

  /*charger disconnected force idle state*/
  if(( chargerInputVoltage < MIN_DOCKED_VOLTAGE) ){
    charger_state = CHARGER_STATE_IDLE;
  }
    
    switch (charger_state)
    {
    case CHARGER_STATE_CONNECTED:
        
        /* when connected the 3.3v and 5v is provided by the charger so we get the real biais of the current measure */
        chargecontrol_pwm_val = 0;

        /* wait 100ms to read current */
        if( (HAL_GetTick() - timestamp) > 100){
          charge_current_offset.f = current_without_offset;
          // Writes a data in a RTC Backup data Register 3&4
          HAL_PWR_EnableBkUpAccess();
          HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, charge_current_offset.u[0]);    
          HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, charge_current_offset.u[1]);   
          HAL_PWR_DisableBkUpAccess(); 
          HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 1); /* Power on the battery  Powerbus */
          charger_state = CHARGER_STATE_CHARGING_CC;
        }

        break;

    case CHARGER_STATE_CHARGING_CC:
        // cap charge current at 1.5 Amps
        if ((battery_voltage > charge_end_voltage && (chargecontrol_pwm_val > 0)) || ((current > MAX_CHARGE_CURRENT) && (chargecontrol_pwm_val > 39)))
        {
            chargecontrol_pwm_val--;
        }
        if ((battery_voltage < charge_end_voltage) && (current < MAX_CHARGE_CURRENT) && (chargecontrol_pwm_val < 1350))
        {
            chargecontrol_pwm_val++;
        }

        if(charge_voltage >= charge_end_voltage) {
            charger_state = CHARGER_STATE_CHARGING_CV;
        }

        break;

    case CHARGER_STATE_CHARGING_CV:
        // set PWM to approach 29.4V  charge voltage
        if ((battery_voltage < charge_end_voltage) && (charge_voltage < (MAX_CHARGE_VOLTAGE)) && (chargecontrol_pwm_val < 1350))
        {
          chargecontrol_pwm_val++;
        }            
        if ((battery_voltage > charge_end_voltage && (chargecontrol_pwm_val > 0)) || (charge_voltage > (MAX_CHARGE_VOLTAGE) && (chargecontrol_pwm_val > 39)))
        {
          chargecontrol_pwm_val--;
        }

        /* the current is limited to 150ma */
        if ((current > (MAX_CHARGE_CURRENT/10)) && chargecontrol_pwm_val > 0)
        {
            chargecontrol_pwm_val--;
        }

        /* battery full ? */
        if (current < CHARGE_END_LIMIT_CURRENT) {
          //charger_state = CHARGER_STATE_END_CHARGING;
          /*consider as the battery full */
          ampere_acc.f = 2.8;
        }

        break;

    case CHARGER_STATE_END_CHARGING:

        chargecontrol_pwm_val = 0;

        break;


    case CHARGER_STATE_IDLE:
    default:
       
        if (chargerInputVoltage >= 30.0 ) {
            charger_state = CHARGER_STATE_CONNECTED;
            HAL_GPIO_WritePin(TF4_GPIO_PORT, TF4_PIN, 0); /* Power off the battery  Powerbus */
            timestamp = HAL_GetTick();
        }
        chargecontrol_pwm_val = 0;
        break;
    }
    
    ampere_acc.f += ((current - charge_current_offset.f)/(100*60*60));
    if(ampere_acc.f >= 2.8)ampere_acc.f = 2.8;
    SOC = ampere_acc.f/2.8;

    // Writes a data in a RTC Backup data Register 1
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, ampere_acc.u[0]);    
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, ampere_acc.u[1]);   
    HAL_PWR_DisableBkUpAccess(); 

    chargecontrol_is_charging = charger_state;

    /*Check the PWM value for safety */
    if (chargecontrol_pwm_val > 1350){
        chargecontrol_pwm_val = 1350;
    }
    TIM1->CCR1 = chargecontrol_pwm_val;  
    
}

/******************************************************************************
 *  Private Functions
 *******************************************************************************/
