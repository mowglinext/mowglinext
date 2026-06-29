/****************************************************************************
* Title                 :   adc module
* Filename              :   adc.c
* Author                :   Nekraus
* Origin Date           :   01/04/2023
* Version               :   1.0.0

*****************************************************************************/
/** \file adc.c
 *  \brief
 *
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include "main.h"
#include "perimeter.h"
#include "adc.h"
#include <math.h>
/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/
const float f_RTO = 10000;
const float beta = 3380;

/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/
typedef enum
{
    ADC_CHARGING_CHANNEL_CURRENT = 0,
    ADC_CHARGING_CHANNEL_CHARGEVOLTAGE,
    ADC_CHARGING_CHANNEL_BATTERYVOLTAGE,
    ADC_CHARGING_CHANNEL_CHARGERINPUTVOLTAGE,
    ADC_CHARGING_CHANNEL_NTC,
    ADC_CHARGING_CHANNEL_MAX,
} ADC_Charging_channelSelection_e;

/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/
TIM_HandleTypeDef TIM2_Handle; // Time Base for ADC
ADC_HandleTypeDef ADC_Charging_Handle;
RTC_HandleTypeDef hrtc = {0};

ADC_Charging_channelSelection_e adc_charging_eChannelSelection = ADC_CHARGING_CHANNEL_CURRENT;

volatile uint16_t adc_u16BatteryVoltage       = 0;
volatile uint16_t adc_u16Current              = 0;
volatile uint16_t adc_u16ChargerVoltage       = 0;
volatile uint16_t adc_u16ChargerInputVoltage  = 0;
volatile uint16_t adc_u16Input_NTC            = 0;

float battery_voltage;
float charge_voltage;
float current;
float current_without_offset;
float ntc_voltage;
float blade_temperature;
float chargerInputVoltage;

union FtoU ampere_acc;
union FtoU charge_current_offset;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void adc_charging_SetChannel(ADC_Charging_channelSelection_e channel);

/******************************************************************************
 *  Public Functions
 *******************************************************************************/

/**
 * @brief TIM2 Initialization Function
 *
 * Used to start ADC every 250µs
 *
 * @param None
 * @retval None
 */
void TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    TIM2_Handle.Instance = TIM2;
    TIM2_Handle.Init.Prescaler = 18 - 1; // 72Mhz -> 4Mhz
    TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM2_Handle.Init.Period = 1000 - 1; /*1khz*/
    TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&TIM2_Handle) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&TIM2_Handle, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&TIM2_Handle, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 5;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&TIM2_Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief ADC Initialization Function
 *
 * Init GPIO to read
 *
 * @param None
 * @retval None
 */
void ADC_Charging_Init(void)
{
	// Configuration: ADC1 for Yardforce 500B
	// 				  ADC2 for Yardforce 500 original
#if BOARD_YARDFORCE500_VARIANT_ORIG
    __HAL_RCC_ADC2_CLK_ENABLE();
	ADC_TypeDef *Charging_ADC = ADC2;
#elif BOARD_YARDFORCE500_VARIANT_B
	__HAL_RCC_ADC1_CLK_ENABLE();
	ADC_TypeDef *Charging_ADC = ADC1;
#endif
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**ADC1 GPIO Configuration
    PA1     ------> Charge Current
    PA2     ------> Charge Voltage
    PA3     ------> Battery Voltage
    PA7     ------> Charger Voltage
    PC2     ------>  Blade NTC
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    ADC_Charging_Handle.Instance = Charging_ADC;
	// technically ADC_SCAN_DISABLE on STM32f1, but this is compatible with STM32f1 und STM32f4, and zero is zero
	ADC_Charging_Handle.Init.ScanConvMode = DISABLE;
	ADC_Charging_Handle.Init.ContinuousConvMode = DISABLE;
	ADC_Charging_Handle.Init.DiscontinuousConvMode = DISABLE;
	ADC_Charging_Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
	ADC_Charging_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC_Charging_Handle.Init.NbrOfConversion = 1;

#if BOARD_YARDFORCE500_VARIANT_B
	ADC_Charging_Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	ADC_Charging_Handle.Init.Resolution = ADC_RESOLUTION_12B;
	ADC_Charging_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	ADC_Charging_Handle.Init.DMAContinuousRequests = DISABLE;
	ADC_Charging_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
#endif

    if (HAL_ADC_Init(&ADC_Charging_Handle) != HAL_OK)
    {
        Error_Handler();
    }

	adc_charging_eChannelSelection = ADC_CHARGING_CHANNEL_CURRENT;
	adc_charging_SetChannel(adc_charging_eChannelSelection);

#if BOARD_YARDFORCE500_VARIANT_ORIG
	IRQn_Type used_ADC_irq = ADC1_2_IRQn;
#elif BOARD_YARDFORCE500_VARIANT_B
	IRQn_Type used_ADC_irq = ADC_IRQn;
#endif

    HAL_NVIC_SetPriority(used_ADC_irq, 0, 0);
    HAL_NVIC_EnableIRQ(used_ADC_irq);

#if BOARD_YARDFORCE500_VARIANT_ORIG
    // calibrate  - important for accuracy !
    HAL_ADCEx_Calibration_Start(&ADC_Charging_Handle);

	// TODO: The STM32f4 does not have a function to calibrate the ADC,
	//		 so we either need manual calibration or just assume it is
	// 		 calibrated correctly all the time
#endif
    HAL_ADC_Start_IT(&ADC_Charging_Handle);
    HAL_TIM_OC_Start(&TIM2_Handle, TIM_CHANNEL_2);

    /* USER CODE BEGIN RTC_MspInit 0 */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* USER CODE END RTC_MspInit 0 */
    /* Enable BKP CLK enable for backup registers */

#if BOARD_YARDFORCE500_VARIANT_ORIG
	// The STM32f4 seems to not require this, but the STM32f1 does
	// TODO: Check if this is true
    __HAL_RCC_BKP_CLK_ENABLE();
#endif
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    /* USER CODE BEGIN RTC_MspInit 1 */
    HAL_PWR_EnableBkUpAccess();

    ampere_acc.u[0] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
    ampere_acc.u[1] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);

    charge_current_offset.u[0] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
    charge_current_offset.u[1] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
}

/**
 * @brief ADC Input Function
 *
 * get the raw data and transform to human readeable values (V,A,T)
 *
 * @param None
 * @retval None
 */

void ADC_input(void)
{
    float l_fTmp;

    /* Snapshot volatile ADC values under interrupt lock to avoid torn reads */
    __disable_irq();
    uint16_t raw_battery       = adc_u16BatteryVoltage;
    uint16_t raw_charger       = adc_u16ChargerVoltage;
    uint16_t raw_current       = adc_u16Current;
    uint16_t raw_chargerInput  = adc_u16ChargerInputVoltage;
    uint16_t raw_ntc           = adc_u16Input_NTC;
    __enable_irq();

    /* battery volatge calculation */
    l_fTmp = ((float)raw_battery / 4095.0f) * 3.3f * 10.09 + 0.6f;
    battery_voltage = 0.2 * l_fTmp + 0.8 * battery_voltage;

     /*charger voltage calculation */
    l_fTmp = ((float)raw_charger / 4095.0f) * 3.3f * 16;
    charge_voltage = 0.8 * l_fTmp + 0.2 * charge_voltage;

    /*charge current calculation */
    l_fTmp = (((float)raw_current / 4095.0f) * 3.3f - 2.5f) * 100 / 12.0;
    current_without_offset =   0.8 * l_fTmp + 0.2 * current_without_offset;

    /*remove offset*/
    current = current_without_offset - charge_current_offset.f;

    /*blade motor temperature calculation */
    l_fTmp = (raw_ntc/4095.0f)*3.3f;
    ntc_voltage = 0.5*l_fTmp + 0.5*ntc_voltage;

    /*calculation for NTC temperature*/
    l_fTmp = ntc_voltage * 10000;               //Resistance of RT
    l_fTmp = log(l_fTmp / f_RTO);
    l_fTmp = (1 / ((l_fTmp / beta) + (1 / (273.15+25)))); //Temperature from thermistor
    blade_temperature = l_fTmp - 273.15;                 //Conversion to Celsius

    /* Input voltage from the external supply*/
    l_fTmp = (raw_chargerInput / 4095.0f) * 3.3f * (32 / 2);
    chargerInputVoltage = 0.5 * l_fTmp + 0.5 * chargerInputVoltage;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#ifdef OPTION_PERIMETER
    if (hadc == &ADC_Handle)
    {
        PERIMETER_vITHandle();
    }
#endif

    if (hadc == &ADC_Charging_Handle)
    {
        uint16_t l_u16Rawdata = ADC_Charging_Handle.Instance->DR;

        switch (adc_charging_eChannelSelection)
        {
        case ADC_CHARGING_CHANNEL_CURRENT:
            adc_u16Current = l_u16Rawdata;
            break;

        case ADC_CHARGING_CHANNEL_CHARGEVOLTAGE:
            adc_u16ChargerVoltage = l_u16Rawdata;
            break;

        case ADC_CHARGING_CHANNEL_BATTERYVOLTAGE:
            adc_u16BatteryVoltage = l_u16Rawdata;
            break;

        case ADC_CHARGING_CHANNEL_CHARGERINPUTVOLTAGE:
            adc_u16ChargerInputVoltage = l_u16Rawdata;
            break;

        case ADC_CHARGING_CHANNEL_NTC:
            adc_u16Input_NTC = l_u16Rawdata;

            break;

        case ADC_CHARGING_CHANNEL_MAX:
        default:
            /* should not get here */
            break;
        }

        adc_charging_eChannelSelection++;
        if (adc_charging_eChannelSelection == ADC_CHARGING_CHANNEL_MAX)
			adc_charging_eChannelSelection = ADC_CHARGING_CHANNEL_CURRENT;
		adc_charging_SetChannel(adc_charging_eChannelSelection);

        HAL_ADC_Start_IT(&ADC_Charging_Handle);
    }
}



/******************************************************************************
 *  Private Functions
 *******************************************************************************/

void adc_charging_SetChannel(ADC_Charging_channelSelection_e channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

#if BOARD_YARDFORCE500_VARIANT_ORIG
	uint32_t adc_SampleTime = ADC_SAMPLETIME_239CYCLES_5;
#elif BOARD_YARDFORCE500_VARIANT_B
	uint32_t adc_SampleTime = ADC_SAMPLETIME_480CYCLES;
#endif

    switch (channel)
    {
    case ADC_CHARGING_CHANNEL_CURRENT:
        sConfig.Channel = ADC_CHANNEL_1; // PA1 Charge Current
        sConfig.Rank = 1;
        sConfig.SamplingTime = adc_SampleTime;
        if (HAL_ADC_ConfigChannel(&ADC_Charging_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;

    case ADC_CHARGING_CHANNEL_CHARGEVOLTAGE:
        sConfig.Channel = ADC_CHANNEL_2; // PA2 Charge Voltage
        sConfig.Rank = 1;
        sConfig.SamplingTime = adc_SampleTime;
        if (HAL_ADC_ConfigChannel(&ADC_Charging_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;

    case ADC_CHARGING_CHANNEL_BATTERYVOLTAGE:
        sConfig.Channel = ADC_CHANNEL_3; // PA3 Battery
        sConfig.Rank = 1;
        sConfig.SamplingTime = adc_SampleTime;
        if (HAL_ADC_ConfigChannel(&ADC_Charging_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;

    case ADC_CHARGING_CHANNEL_CHARGERINPUTVOLTAGE:
        sConfig.Channel = ADC_CHANNEL_7; // PA7 Charger Input voltage
        sConfig.Rank = 1;
        sConfig.SamplingTime = adc_SampleTime;
        if (HAL_ADC_ConfigChannel(&ADC_Charging_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;

    case ADC_CHARGING_CHANNEL_NTC:
        sConfig.Channel = ADC_CHANNEL_13; // PC2
        sConfig.Rank = 1;
        sConfig.SamplingTime = adc_SampleTime;
        if (HAL_ADC_ConfigChannel(&ADC_Charging_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;

    case ADC_CHARGING_CHANNEL_MAX:
    default:
        /* should not get here */
        sConfig.Channel = ADC_CHANNEL_3; // PA3 Battery
        sConfig.Rank = 1;
        sConfig.SamplingTime = adc_SampleTime;
        if (HAL_ADC_ConfigChannel(&ADC_Charging_Handle, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    }
}
