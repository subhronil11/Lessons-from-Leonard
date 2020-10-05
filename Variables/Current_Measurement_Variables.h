/*
 * Current_Measurement_Variables.h
 *
 *  Created on: 13 jul. 2020
 *      Author: SCI
 */

#ifndef CURRENT_MEASUREMENT_VARIABLES_H_
#define CURRENT_MEASUREMENT_VARIABLES_H_

int Results_ADC_Current;
int Results_Current[100]; // SD24 conversion and temporary results
unsigned int i_C = 0; // Index used for the Results_Current array
int New_Current;
int Prev_Current;
unsigned long Current_Square;
unsigned int Current_Counter = 0;
unsigned int Current_Counter_main = 0;
float Current_Counter_divided = 0.0;
unsigned long long Current_Sum = 0;
unsigned long long Current_Sum_main = 0;
unsigned long Current_Mean = 0;
unsigned long Current_Roll[Roll_Avg_Size]={0};
unsigned long long Current_Roll_Sum = 0;
unsigned long Current_Roll_Avg = 0;
unsigned char Current_Roll_Counter = 0;
unsigned int Current_RMS = 0;
unsigned int Actual_Current_RMS = 0;
unsigned int Current_RMS_DAC = 0;
unsigned char Current_Zero_Crossing = 0;
unsigned char Current_Flag = 0xFF;



#endif /* CURRENT_MEASUREMENT_VARIABLES_H_ */
