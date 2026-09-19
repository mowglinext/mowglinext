/****************************************************************************
* Title                 :   Blade motor driver
* Filename              :   blademotor.h
* Author                :   Nekraus
* Origin Date           :   17/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file blademotor.h
*  \brief 
*
*/
#ifndef __BLADEMOTOR_H
#define __BLADEMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>

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
// global variables used by ROS
extern bool BLADEMOTOR_bActivated;
/* Raw ESC bytes 7..8 under the legacy RPM name. On the tested 500 this holds
 * the last nonzero report after OFF, then clears; it is not live coast RPM. */
extern uint16_t BLADEMOTOR_u16RPM;
extern uint16_t BLADEMOTOR_u16Power;
extern uint32_t BLADEMOTOR_u32Error;
/******************************************************************************
* PUBLIC Function Prototypes
*******************************************************************************/

void BLADEMOTOR_Init(void);
void BLADEMOTOR_App(void);
void BLADEMOTOR_ReceiveIT(void);

void BLADEMOTOR_Set(uint8_t on_off, uint8_t direction);


#ifdef __cplusplus
}
#endif

#endif /*BLADEMOTOR_H*/ 

/*** End of File **************************************************************/
