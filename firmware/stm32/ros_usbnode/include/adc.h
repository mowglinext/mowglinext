/****************************************************************************
* Title                 :   adc module
* Filename              :   adc.h
* Author                :   Nekraus
* Origin Date           :   01/04/2023
* Version               :   1.0.0

*****************************************************************************/
/** \file adc.h
*  \brief 
*
*/
#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>

#if defined(BOARD_YARDFORCE500_VARIANT_B)
#include "stm32f4xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Includes
*******************************************************************************/

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
/* union to store a float in the U16 backup register */
union FtoU{
  float  f;
  uint16_t u[2];
};

/******************************************************************************
* Variables
*******************************************************************************/

extern RTC_HandleTypeDef hrtc;

extern float battery_voltage;
extern float charge_voltage;
extern float current;
extern float current_without_offset;
extern float blade_temperature;
extern float chargerInputVoltage;

extern union FtoU ampere_acc;
extern union FtoU charge_current_offset;

/******************************************************************************
* PUBLIC Function Prototypes
*******************************************************************************/

void TIM2_Init(void);
void ADC_Charging_Init(void);

void ADC_input(void);

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc);



#ifdef __cplusplus
}
#endif
#endif /*__ADC_H*/ 

/*** End of File **************************************************************/