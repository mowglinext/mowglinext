#ifndef __PANEL_H
#define __PANEL_H

#include "board.h"
#include "stm32f_board_hal.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
        PANEL_LED_OFF,
        PANEL_LED_ON,
        PANEL_LED_FLASH_SLOW,
        PANEL_LED_FLASH_FAST
} PANEL_LED_STATE;


/* 
 * different yardforce models have different keyboard/led panels 
 */

#if PANEL_TYPE==PANEL_TYPE_YARDFORCE_900_ECO         // YardForce SA900ECO
    #define PANEL_LED_LIFTED 0
    #define PANEL_LED_SIGNAL 1
    #define PANEL_LED_BATTERY_LOW 2
    #define PANEL_LED_CHARGING 3
    #define PANEL_LED_4H 4
    #define PANEL_LED_6H 5
    #define PANEL_LED_8H 6
    #define PANEL_LED_10H 7
    #define PANEL_LED_S1 8
    #define PANEL_LED_S2 9
    #define PANEL_LED_LOCK 10

     #define PANEL_LED_GPS 1
    
    #define LED_STATE_SIZE 12       // model has 12-1 different leds to control ?   
    #define LED_CMD 0x508b


#elif PANEL_TYPE==PANEL_TYPE_YARDFORCE_LUV1000RI   
    #define PANEL_LED_LIFTED 0
    #define PANEL_LED_SIGNAL 1
    #define PANEL_LED_BATTERY_LOW 2
    #define PANEL_LED_CHARGING 3
    #define PANEL_LED_4H 4
    #define PANEL_LED_6H 5
    #define PANEL_LED_8H 6
    #define PANEL_LED_10H 7
    #define PANEL_LED_S1 8
    #define PANEL_LED_S2 9
    #define PANEL_LED_LOCK 10
    #define PANEL_LED_WIFI 11
    
    #define PANEL_LED_GPS 11
    
    #define LED_STATE_SIZE 12       // model has 12-1 different leds to control ?   
    #define LED_CMD 0x508b

#elif PANEL_TYPE==PANEL_TYPE_YARDFORCE_500_CLASSIC   // Yardforce 500 CLASSIC

 /* Byte Mapping of Bytes 5-16
      0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16
      
      0x55  0xaa  0x02  0x50  0x00  0x02  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x02  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x02  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x02  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x02  0x00  0x00  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x02  0x00  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x02  0x00  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x02  0x00  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x02  0x00  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x02  0x00  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x02  0x00
      0x55  0xaa  0x02  0x50  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x00  0x02
      
      55    aa    02    50    00    timer S1    S2    Lock  OK	  MON   TUE   WED   THU   FRI   SAT   SUN
    */      
    #define PANEL_LED_LIFTED 0
    #define PANEL_LED_SIGNAL 1
    #define PANEL_LED_BATTERY_LOW 2
    #define PANEL_LED_CHARGING 3
    #define PANEL_LED_2H 4
    #define PANEL_LED_4H 5
    #define PANEL_LED_6H 6
    #define PANEL_LED_8H 7
    #define PANEL_LED_S1 8
    #define PANEL_LED_S2 9
    #define PANEL_LED_LOCK 10
    #define PANEL_LED_MON 11
    #define PANEL_LED_TUE 12
    #define PANEL_LED_WED 13
    #define PANEL_LED_THR 14
    #define PANEL_LED_FRI 15
    #define PANEL_LED_SAT 16
    #define PANEL_LED_SUN 17
    #define PANEL_LED_UNKNOWN 18

    #define PANEL_LED_GPS 11
    
    #define LED_STATE_SIZE 19       // model has 19-2 different leds to control ?
    #define LED_CMD 0x508e

#elif PANEL_TYPE==PANEL_TYPE_YARDFORCE_500B_CLASSIC   // Yardforce 500B CLASSIC (Bluetooth)

    #define PANEL_LED_LIFTED 0
    #define PANEL_LED_SIGNAL 1
    #define PANEL_LED_BATTERY_LOW 2
    #define PANEL_LED_CHARGING 3
    #define PANEL_LED_2H 4
    #define PANEL_LED_4H 5
    #define PANEL_LED_6H 6
    #define PANEL_LED_8H 7
    #define PANEL_LED_S1 8
    #define PANEL_LED_S2 9
    #define PANEL_LED_LOCK 10
    #define PANEL_LED_MON 11
    #define PANEL_LED_TUE 12
    #define PANEL_LED_WED 13
    #define PANEL_LED_THR 14
    #define PANEL_LED_FRI 15
    #define PANEL_LED_SAT 16
    #define PANEL_LED_SUN 17
    #define PANEL_LED_UNKNOWN 18

    #define PANEL_LED_GPS 11

    #define LED_STATE_SIZE 19       // model has 19-2 different leds to control ?
    #define LED_CMD 0x508e
#endif

/* Panel Buttons published in /button_state rostopic via Serial */
#define PANEL_BUTTON_BYTES 12
#define PANEL_BUTTON_DEF_S1 1
#define PANEL_BUTTON_DEF_S2 2
#define PANEL_BUTTON_DEF_LOCK 3
#define PANEL_BUTTON_DEF_OK 4
#define PANEL_BUTTON_DEF_MON 5
#define PANEL_BUTTON_DEF_TUE 6
#define PANEL_BUTTON_DEF_WED 7
#define PANEL_BUTTON_DEF_THU 8
#define PANEL_BUTTON_DEF_FRI 9
#define PANEL_BUTTON_DEF_SAT 10
#define PANEL_BUTTON_DEF_SUN 11

#define PANEL_BUTTON_DEF_START PANEL_BUTTON_BYTES
#define PANEL_BUTTON_DEF_HOME (PANEL_BUTTON_BYTES+1)

   

extern UART_HandleTypeDef PANEL_USART_Handler;

void PANEL_Init(void);
void PANEL_Tick(void);

void PANEL_Set_LED(uint8_t led, PANEL_LED_STATE state);
int PANEL_Get_Key_Pressed(void);

void PANEL_ReceiceIT(void);

void PANEL_Send_Message(const uint8_t *data, uint8_t dataLength, uint16_t command);

extern uint8_t buttonstate[PANEL_BUTTON_BYTES+2];
extern uint8_t buttonupdated;
extern uint8_t buttoncleared;
extern uint8_t Led_States[LED_STATE_SIZE];


#ifdef __cplusplus
}
#endif

#endif
