/****************************************************************************
* Title                 :   drive motor module
* Filename              :   drivemotor.h
* Author                :   Nekraus
* Origin Date           :   18/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file drivemotor.h
*  \brief 
*
*/
#ifndef __DRIVEMOTOR_H
#define __DRIVEMOTOR_H

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

/******************************************************************************
* Variables
*******************************************************************************/

extern int16_t   right_wheel_speed_val;
extern int16_t   left_wheel_speed_val;
extern uint32_t  right_encoder_ticks;   // legacy cumulative-magnitude counter
extern uint32_t  left_encoder_ticks;    // legacy cumulative-magnitude counter
extern int32_t   right_ticks_signed;    // cumulative signed ticks (polarity = direction)
extern int32_t   left_ticks_signed;     // cumulative signed ticks (polarity = direction)
extern uint16_t  right_encoder_val;     // non accumulating
extern uint16_t  left_encoder_val;      // non accumulating
extern uint8_t   right_power;
extern uint8_t   left_power;
extern uint32_t  DRIVEMOTOR_u32ErrorCnt;


/******************************************************************************
* PUBLIC Function Prototypes
*******************************************************************************/

void DRIVEMOTOR_Init(void);
void DRIVEMOTOR_App_10ms(void);
void DRIVEMOTOR_App_Rx(void);
void DRIVEMOTOR_ReceiveIT(void);
/**
 * Preferred API — signed PWM per wheel. Positive = forward, negative =
 * reverse, 0 = stop. Pure passthrough: the value is saturated to ±255 and
 * sent to the PAC5210 as-is (no host-side deadband compensation — the host
 * wheel-velocity PI owns feedforward/deadband now; see wheel_rate_controller).
 */
void DRIVEMOTOR_SetSpeedSigned(int16_t left_pwm_signed, int16_t right_pwm_signed);

/** Legacy 4-arg API kept as a shim over DRIVEMOTOR_SetSpeedSigned. */
void DRIVEMOTOR_SetSpeed(uint8_t left_speed, uint8_t right_speed, uint8_t left_dir, uint8_t right_dir);

#ifdef __cplusplus
}
#endif
#endif /*__DRIVEMOTOR_H*/ 

/*** End of File **************************************************************/