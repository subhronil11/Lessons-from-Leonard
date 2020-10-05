/*
 * Voltage_Measurement_Variables.h
 *
 *  Created on: 13 jul. 2020
 *      Author: SCI
 */

#ifndef VOLTAGE_MEASUREMENT_VARIABLES_H_
#define VOLTAGE_MEASUREMENT_VARIABLES_H_


unsigned int Results_ADC_Voltage;
int Results_Voltage[100]; // SD24 conversion and temporary results
unsigned int i_V = 0; // Index used for the Results_Voltage array
int New_Voltage;
int Prev_Voltage;
unsigned long Voltage_Square;
unsigned int Voltage_Counter = 0;
unsigned int Voltage_Counter_main = 0;
float Voltage_Counter_divided = 0.0;
unsigned long long Voltage_Sum = 0;
unsigned long long Voltage_Sum_main = 0;
unsigned long Voltage_Mean = 0;
unsigned long Voltage_Roll[Roll_Avg_Size]={0};
unsigned long long Voltage_Roll_Sum = 0;
unsigned long Voltage_Roll_Avg = 0;
unsigned char Voltage_Roll_Counter = 0;
unsigned int Voltage_RMS = 0;
unsigned int Actual_Voltage_RMS = 0;
unsigned char Voltage_Zero_Crossing = 0;
unsigned char Voltage_Flag = 0xFF;



#endif /* VOLTAGE_MEASUREMENT_VARIABLES_H_ */
