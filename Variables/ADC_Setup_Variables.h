/*
 * ADC_Variables.h
 *
 *  Created on: 13 jul. 2020
 *      Author: SCI & EJL
 */

#ifndef ADC_SETUP_VARIABLES_H_
#define ADC_SETUP_VARIABLES_H_

const unsigned int OSR_CHn = 0b01; // 00b = 256, 01b = 128, 10b = 64, 11b = 32
unsigned int Max_Counter = 200; //100 original
const unsigned char Gain_CH0 = 4; // 0 = x1, 1 = x2, 2 = x4, 3 = x8, 4 = x16
const unsigned char Gain_CH1 = 0;
const unsigned char Gain_CH2 = 0;
long Current_Offset = 0;
long Voltage_Offset = 0;
int Current_ADC_Offset = 0; // Should be acquired by calibration
int Voltage_ADC_Offset = 0; // Should be acquired by calibration
unsigned int Address = 0; // Address of the information memory



#endif /* ADC_SETUP_VARIABLES_H_ */
