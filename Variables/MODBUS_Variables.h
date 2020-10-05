/*
 * MODBUS_Variables.h
 *
 *  Created on: 14 jul. 2020
 *      Author: SCI & EJL
 */

#ifndef MODBUS_VARIABLES_H_
#define MODBUS_VARIABLES_H_

#define MODBUS_TIMER_

unsigned char Slave_Address = 64;
unsigned int Holding_Register[10] = {0};
unsigned char Return_Package[25] = {0};
unsigned int Start_Address = 0;
unsigned int No_of_Registers = 0;
unsigned int Received_CRC = 0;
unsigned int Send_CRC = 0;
unsigned char Package_Flag = 0;
unsigned char n = 0;
unsigned char m = 0;
unsigned char Timer_Value = 0;
unsigned char Timer_Flag = 0;

static enum
{
    Address_Val,
    Function_Val,
    Start_Val,
    No_of_Reg_Val,
    CRC_Val,
    Error_Gen,
    Reply_Gen,
    Transmission
}MB_State = Address_Val;


#endif /* MODBUS_VARIABLES_H_ */
