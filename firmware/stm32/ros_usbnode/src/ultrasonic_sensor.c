#include <stdio.h>
#include <string.h>
#include "stm32f_board_hal.h"

// stm32 custom
#include "board.h"
#include "main.h"
#include "ultrasonic_sensor.h"

/* The NX100i reads its hood ultrasonic board over I2C (ultrasonic_nx100i.c),
 * which provides this same interface; keep this UART backend out then. */
#if !((OPTION_ULTRASONIC == 1) && defined(ULTRASONIC_TYPE) && (ULTRASONIC_TYPE == ULTRASONIC_TYPE_NX100I_I2C))

extern UART_HandleTypeDef MASTER_USART_Handler; // UART  Handle

typedef enum {
    ULTRASONIC_INIT_1,
    ULTRASONIC_INIT_2,
    ULTRASONIC_RUN
}ULTRASONIC_STATE_e;

static ULTRASONIC_STATE_e ultrasonic_state = ULTRASONIC_INIT_1;

const uint8_t ultrasonic_InitMessage1[6] = {0x55,0xAA,0x02,0xFF,0xFF,0xFF};
const uint8_t ultrasonic_InitMessage2[6] = {0x55,0xAA,0x02,0xFF,0xFB,0xFB};
const uint8_t ultrasonic_RqstMessage[6]  = {0x55,0xAA,0x02,0x70,0x39,0xAA};

const uint8_t ultrasonic_PreAmbule[5]  = {0x55,0xAA,0x06,0x70,0x39};

static uint8_t ultrasonic_pu8ReceivedData[10] = {0};

static uint8_t ultrasonic_RxFlag = 0;

static uint32_t ultrasonic_u32LeftDistance = 0;
static uint32_t ultrasonic_u32RightDistance = 0;

void ULTRASONICSENSOR_Init(void){
    ultrasonic_state = ULTRASONIC_INIT_1;
    ultrasonic_u32LeftDistance = 0;
    ultrasonic_u32RightDistance = 0;
}

void ULTRASONICSENSOR_App(void){

    switch (ultrasonic_state)
    {
    case ULTRASONIC_INIT_1:
        HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)ultrasonic_InitMessage1, 6);
        ultrasonic_state = ULTRASONIC_INIT_2;
        break;
    
    case ULTRASONIC_INIT_2:
        HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)ultrasonic_InitMessage2, 6);
        ultrasonic_state = ULTRASONIC_RUN;
        break;

    case ULTRASONIC_RUN:
        
        /* prepare to receive the message before to lauch the command */
        HAL_UART_Receive_DMA(&MASTER_USART_Handler,ultrasonic_pu8ReceivedData,10);
        HAL_UART_Transmit_DMA(&MASTER_USART_Handler, (uint8_t*)ultrasonic_RqstMessage, 6);


        break;
    
    default:
        break;
    }
}

void ULTRASONICSENSOR_ReceiveIT(void)
{
    /* decode the frame */
    if(memcmp(ultrasonic_PreAmbule,ultrasonic_pu8ReceivedData,5) == 0){
        /* todo calculate the CRC */
        ultrasonic_u32LeftDistance = (ultrasonic_pu8ReceivedData[5] << 8) + ultrasonic_pu8ReceivedData[6];
        ultrasonic_u32RightDistance  = (ultrasonic_pu8ReceivedData[7] << 8) + ultrasonic_pu8ReceivedData[8];

        DB_TRACE(" R: %dmm, L: %dmm \r\n",ultrasonic_u32RightDistance/10,ultrasonic_u32LeftDistance/10);
        ultrasonic_RxFlag = 1;
    }
}

uint32_t ULTRASONIC_MessageReceived(void){
    if(ultrasonic_RxFlag == 1){
        ultrasonic_RxFlag = 0; /* reset flags*/
        return 1;
    }
    else{
        return 0;
    }
}

uint32_t ULTRASONICSENSOR_u32GetLeftDistance(void){
    return ultrasonic_u32LeftDistance;
}

uint32_t ULTRASONICSENSOR_u32GetRightDistance(void){
    return ultrasonic_u32RightDistance;
}

#endif /* not the NX100i I2C backend */
