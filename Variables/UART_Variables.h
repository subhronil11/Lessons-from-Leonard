/*
 * UART_Variables.h
 *
 *  Created on: 14 jul. 2020
 *      Author: SCI
 */

#ifndef UART_VARIABLES_H_
#define UART_VARIABLES_H_


unsigned char UART_Flag = 0;
unsigned char UART_Data_Received[8] = {0};
unsigned char i = 0;
unsigned char j = 0;

static enum
{
    State_INIT,
    State_IDLE,
    State_Receive,
    State_ERROR
}RX_State = State_INIT;

#endif /* UART_VARIABLES_H_ */
