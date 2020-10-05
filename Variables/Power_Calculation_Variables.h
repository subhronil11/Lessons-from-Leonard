/*
 * Power_Calculation_Variables.h
 *
 *  Created on: 13 jul. 2020
 *      Author: SCI
 */

#ifndef POWER_CALCULATION_VARIABLES_H_
#define POWER_CALCULATION_VARIABLES_H_

long Instant_Power = 0;
long long Power_Sum = 0;
long long Power_Sum_main = 0;
unsigned long Apparent_Power = 0;
unsigned int Actual_Apparent_Power = 0;
unsigned long long Apparent_Power_Square = 0;
long True_Power_Mean = 0;
long Power_Roll[Roll_Avg_Size] = {0};
long long Power_Roll_Sum = 0;
unsigned char Power_Roll_Counter = 0;
long True_Power = 0;
int Actual_True_Power = 0;
long long True_Power_Square = 0;
unsigned long long Reactive_Power_Square = 0;
unsigned long Reactive_Power = 0;
int Actual_Reactive_Power = 0;
int PF = 0;



#endif /* POWER_CALCULATION_VARIABLES_H_ */
