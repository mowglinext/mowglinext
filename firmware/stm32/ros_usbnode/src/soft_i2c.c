/**
  ******************************************************************************
  * @file    soft_i2c.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/22
  * @version 1.0.0
  * @brief   Software I2C driver for pins on J18
  ******************************************************************************
  * @attention
  *
  * mostly reworked code from: https://schkorea.tistory.com/437
  * note that you need to turn off JTAG (but not SWD) to free the pins that go to
  * J18 !
  ******************************************************************************
  */

#include "stm32f_board_hal.h"
#include "soft_i2c.h"
#include "board.h"



//#define  SW_I2C_WAIT_TIME  25	//(11.0us)
//#define  SW_I2C_WAIT_TIME  23	//(10.4us)
#define  SW_I2C_WAIT_TIME  22	//100Khz(10.0us)	100Khz	==	10us
//#define  SW_I2C_WAIT_TIME  10	//195Khz
//#define  SW_I2C_WAIT_TIME  9	//205Khz	200Khz	==	5us
//#define  SW_I2C_WAIT_TIME  8	//237Khz
//#define  SW_I2C_WAIT_TIME  7	//240Khz	250Khz	==	4us
//#define  SW_I2C_WAIT_TIME  6	//275Khz
//#define  SW_I2C_WAIT_TIME  5	//305Khz
//#define  SW_I2C_WAIT_TIME  4	//350Khz(3.84us)
//#define  SW_I2C_WAIT_TIME  3	//400Khz(3.44us)
//#define  SW_I2C_WAIT_TIME  2	//425Khz(3.04us)	333Khz	==	3us
//#define  SW_I2C_WAIT_TIME  1	//425Khz(2.64us)	400Khz	==	2.5us

#define TRUE 1
#define FALSE 0

#define  I2C_READ       0x01
#define  READ_CMD       1
#define  WRITE_CMD      0
#define  I2C_ACK_SPIN_LIMIT  256

// map from board.h to what soft_i2c uses
#define SW_I2C1_SCL_GPIO  SOFT_I2C_SCL_PORT
#define SW_I2C1_SDA_GPIO  SOFT_I2C_SDA_PORT
#define SW_I2C1_SCL_PIN   SOFT_I2C_SCL_PIN
#define SW_I2C1_SDA_PIN   SOFT_I2C_SDA_PIN


void  __attribute__ ((optimize(0))) TIMER__Wait_us (uint32_t nCount) 
{
    for (; nCount != 0;nCount--);
}

/* init soft i2c pins */
void SW_I2C_Init(void)
{
    /* PB3, PB4 are used by the JTAG - we need to disable it, as we use SWD anyhow we dont need it */

#if BOARD_YARDFORCE500_VARIANT_ORIG
	// TODO: Check if some equivalent of all of this is needed for the STM32f4
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable A.F. clock

    if( (SW_I2C1_SDA_PIN == GPIO_PIN_3) || (SW_I2C1_SCL_PIN == GPIO_PIN_3)){
        AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // JTAG is disabled, SW0 is enabled
    }
    else
    {
        AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NOJNTRST; // JTAG is disabled
    }
#endif

	SOFT_I2C_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;


    GPIO_InitStruct.Pin   = SW_I2C1_SCL_PIN;
    HAL_GPIO_Init(SW_I2C1_SCL_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = SW_I2C1_SDA_PIN;
    HAL_GPIO_Init(SW_I2C1_SDA_GPIO, &GPIO_InitStruct);    
}

/* de-init soft i2c pins */
void SW_I2C_DeInit(void)
{ 
   HAL_GPIO_DeInit(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN);
   HAL_GPIO_DeInit(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN);
}

// SDA High
void sda_high(void)
{ 
    HAL_GPIO_WritePin(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN, GPIO_PIN_SET);
}

// SDA low
void sda_low(void)
{
    HAL_GPIO_WritePin(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN, GPIO_PIN_RESET);
}

// SCL High
void scl_high(void)
{   
    HAL_GPIO_WritePin(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN, GPIO_PIN_SET);
}

// SCL low
void scl_low(void)
{    
    HAL_GPIO_WritePin(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN, GPIO_PIN_RESET);
}

void sda_out(uint8_t out)
{
    if (out)
    {
        sda_high();
    }
    else
    {
        sda_low();
    }
}

void sda_in_mode(void)
{
  //  debug_printf("sda_in_mode()\r\n");
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;	//IPD->IPU
	GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin   = SW_I2C1_SDA_PIN;
    HAL_GPIO_Init(SW_I2C1_SDA_GPIO, &GPIO_InitStruct);  
}

void sda_out_mode(void)
{
    //debug_printf("sda_out_mode()\r\n");

    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;


    GPIO_InitStruct.Pin   = SW_I2C1_SDA_PIN;
    HAL_GPIO_Init(SW_I2C1_SDA_GPIO, &GPIO_InitStruct);  
}

void scl_in_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;	//IPD->IPU
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin   = SW_I2C1_SCL_PIN;
    HAL_GPIO_Init(SW_I2C1_SCL_GPIO, &GPIO_InitStruct);
  
}

void scl_out_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;		// error point GPIO_Mode_Out_PP	
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin   = SW_I2C1_SCL_PIN;
    HAL_GPIO_Init(SW_I2C1_SCL_GPIO, &GPIO_InitStruct);
}

void i2c_clk_data_out(void)
{
    scl_high();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_low();
    //TIMER__Wait_us(SW_I2C_WAIT_TIME>>2);
}

void i2c_port_initial(void)
{
    sda_high();
    scl_high();
}

void i2c_start_condition(void)
{
    sda_high();
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_low();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_low();
}

void i2c_stop_condition(void)
{
    sda_low();
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_high();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_low();
}

static int i2c_check_ack(void)
{
    int             i;

    sda_in_mode();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    scl_high();

    for (i = I2C_ACK_SPIN_LIMIT; i > 0; i--)
    {
        if (!SW_I2C_ReadVal_SDA()) break;	//0=ack , 1=nack
    }
    scl_low();
    sda_out_mode();

    return i;
}

static uint8_t i2c_bus_is_idle(void)
{
    return (uint8_t)(SW_I2C_ReadVal_SDA() && SW_I2C_ReadVal_SCL());
}

static void i2c_abort_transaction(void)
{
    sda_out_mode();
    i2c_stop_condition();
}

void i2c_send_not_ack(void)
{
    i2c_clk_data_out();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

void i2c_send_final_nack(void)
{
    sda_out_mode();
    sda_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    scl_low();
}

void i2c_check_final_read_not_ack(void)
{
    i2c_clk_data_out();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_out_mode();
}


void i2c_check_not_ack_continue(void)
{
//    sda_in_mode();
    i2c_clk_data_out();
//    sda_out_mode();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

void i2c_byte_out(uint8_t addr)
{
    int  x;

    for (x = 128; x ; x>>=1)
    {
        sda_out(addr & x);
        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        i2c_clk_data_out();
//        TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }
}

void i2c_slave_address(uint8_t IICID, uint8_t readwrite)
{
    i2c_byte_out(IICID | (readwrite ? I2C_READ : 0));
}

void i2c_send_ack(void)
{
    sda_out_mode();
    sda_low();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    scl_low();
    sda_in_mode();
    
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

/* external functions */
uint8_t SW_I2C_ReadVal_SDA(void)
{    
    return HAL_GPIO_ReadPin(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN);       
}

uint8_t SW_I2C_ReadVal_SCL(void)
{ 
    return HAL_GPIO_ReadPin(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN);    
}

void SW_I2C_Write_Data(uint8_t data)
{
    int  x;

    scl_low();

    for (x = 7; x >= 0; x--)
    {
        sda_out(data & (1 << x));
        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        i2c_clk_data_out();
//        TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }
}

uint8_t SW_I2C_Read_Data_in_in_mode(void)
{
    int      x;
    uint8_t  readdata = 0;

    sda_in_mode();
    for (x = 128; x; x>>=1)
    {
        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        scl_high();

        TIMER__Wait_us(SW_I2C_WAIT_TIME);

        if (SW_I2C_ReadVal_SDA())
            readdata |= x;

        scl_low();

    }

    return readdata;
}

uint8_t SW_I2C_WriteControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t data)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    i2c_byte_out(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    SW_I2C_Write_Data(data);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_WriteControl_8Bit_OnlyRegAddr(uint8_t IICID, uint8_t regaddr)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    i2c_byte_out(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

//    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_WriteControl_16Bit(uint8_t IICID, uint8_t regaddr, uint16_t data)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_byte_out(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    SW_I2C_Write_Data((data >> 8) & 0xFF);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    SW_I2C_Write_Data(data & 0xFF);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_ReadControl_8Bit_OnlyRegAddr(uint8_t IICID, uint8_t regaddr)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_byte_out(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_ReadControl_8Bit_OnlyData(uint8_t IICID)
{
    uint8_t  readdata = 0;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = SW_I2C_Read_Data_in_in_mode();

    i2c_check_final_read_not_ack();

    i2c_stop_condition();

    return readdata;
}

uint16_t SW_I2C_ReadControl_16Bit_OnlyData(uint8_t IICID)
{
    uint8_t  readimsi = 0;
    uint16_t  readdata = 0;

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_send_not_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readimsi = SW_I2C_Read_Data_in_in_mode();
    i2c_check_not_ack_continue();

    readdata = readimsi<<8;

    readimsi = SW_I2C_Read_Data_in_in_mode();
    i2c_check_final_read_not_ack();

    readdata |= readimsi;

    i2c_stop_condition();

    return readdata;
}

uint8_t SW_I2C_ReadControl_8Bit(uint8_t IICID, uint8_t regaddr)
{
    uint8_t  readdata = 0;

    i2c_port_initial();
    if (!i2c_bus_is_idle()) return 0x80;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return 0x80; } // TODO error handling

    i2c_byte_out(regaddr);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return 0x40; } // TODO error handling

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return 0x20; } // TODO error handling

    readdata = SW_I2C_Read_Data_in_in_mode();

    i2c_check_final_read_not_ack();

    i2c_stop_condition();

    return readdata;
}

uint16_t SW_I2C_ReadControl_16Bit(uint8_t IICID, uint8_t regaddr)
{
    uint16_t  readdata = 0;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    i2c_check_ack();

    i2c_byte_out(regaddr);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = SW_I2C_Read_Data_in_in_mode();
    i2c_send_ack();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = ((readdata << 8) | SW_I2C_Read_Data_in_in_mode());

    i2c_check_final_read_not_ack();

    i2c_stop_condition();

    return readdata;
}

uint8_t SW_I2C_ReadnControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata))
{
    uint8_t   returnack = TRUE;
    uint8_t  index;

    if ((rcnt == 0) || (pdata == NULL)) {
        return FALSE;
    }

    i2c_port_initial();
    if (!i2c_bus_is_idle()) return FALSE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return FALSE; }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_byte_out(regaddr);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return FALSE; }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return FALSE; }

    for ( index = 0 ; index < rcnt ; index++){
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    	pdata[index] = SW_I2C_Read_Data_in_in_mode();
    }

    pdata[rcnt-1] = SW_I2C_Read_Data_in_in_mode();

    i2c_send_final_nack();

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_Multi_ReadnControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata))
{
    uint8_t  index;

    if ((rcnt == 0) || (pdata == NULL)) {
        return FALSE;
    }

    i2c_port_initial();
    if (!i2c_bus_is_idle()) return FALSE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return FALSE; }

    i2c_byte_out(regaddr);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return FALSE; }

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    if (!i2c_check_ack()) { i2c_abort_transaction(); return FALSE; }

    for ( index = 0 ; index < (rcnt-1) ; index++){
    	pdata[index] = SW_I2C_Read_Data_in_in_mode();
		    i2c_send_ack();
    }

    pdata[rcnt-1] = SW_I2C_Read_Data_in_in_mode();

    i2c_check_final_read_not_ack();
    i2c_stop_condition();

    return TRUE;
}

uint8_t SW_I2C_Check_SlaveAddr(uint8_t IICID)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }
	
    return returnack;
}

uint8_t SW_I2C_UTIL_WRITE(uint8_t IICID, uint8_t regaddr, uint8_t data)
{
	return SW_I2C_WriteControl_8Bit(IICID<<1, regaddr, data);
}

uint8_t SW_I2C_UTIL_Read(uint8_t IICID, uint8_t regaddr)
{
	return SW_I2C_ReadControl_8Bit(IICID<<1, regaddr);
}

uint8_t SW_I2C_UTIL_Read_Multi(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata))
{
	return SW_I2C_Multi_ReadnControl_8Bit(IICID<<1, regaddr, rcnt, pdata);
}
