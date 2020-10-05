//******************************************************************************
//
//  ACLK = 32kHz, MCLK = SMCLK = Calibrated DCO = 16.384MHz
//
//
//
//
//  SCI and EJL
//
//  ELEQ Steenwijk B.V.
//  2020
//  Built with Code Composer Studio v8.2
//  Alpha 0.3.2.7:
//  Created: 1-10-2020
//  Author: SCI & EJL
//  Modbus update 6
//
//******************************************************************************
#include <msp430.h>
#include <math.h>
#include <stdio.h>
#include <The_DAC_Header.h>

// Define
#define Roll_Avg_Size 10
#define Slave_ID 12345

#define TRUE 1

#define DEBUG 0
#define USE_MODBUS 1

#define BAUD_RATE 19200
#define MESSAGE_SILENT_TIME

#ifdef VARIANT_A
  #include <processors/variant_a>
#endif
#ifdef VARIANT_B
  #include <processors/variant_b>
#endif

#include <variables.h>

// Other variables
const float Numerator = 1; // Const keeps the variable from changing in the code
int Calibration;

// Initialization functions
void initSD24();
void initSPI();
#ifdef USE_MODBUS
void initUART();
#endif
void initTIMER_A0();
void initTIMER_A1();
void initPORT_1();
void initPORT_2();

// Functions
void Erase_InfoSeg();
void Write_InfoSeg(char Value);
char Read_InfoSeg(unsigned int Address2);
void SPI_Send(unsigned char Command, unsigned int Data);
unsigned int Sqrt_32(unsigned long Input_Value);
unsigned long Sqrt_64(unsigned long long Input_Value);
unsigned int CRC_Calculator(unsigned int Data_Length, unsigned char Buffer[]);
unsigned char MODBUS_Address();
void Start_Timer_A1(unsigned char Time);

void main(void)
{
//================================================================================================================
//
// TLV
//
//================================================================================================================
	/*// Store the TLV data
	unsigned int TLV_checksum = Read_InfoSeg(0x13C0) + ((int)Read_InfoSeg(0x13C1) << 8); // Address 0x13C0
	unsigned char Die_Record_Tag = Read_InfoSeg(0x13C2); // Address 0x13C2
	unsigned char Die_Record_Length = Read_InfoSeg(0x13C3); // Address 0x13C3
	unsigned long LotWafer_ID = Read_InfoSeg(0x13C4) + ((int)Read_InfoSeg(0x13C5) << 8) + ((long)Read_InfoSeg(0x13C6) << 16) + ((long)Read_InfoSeg(0x13C7) << 24); // Address 0x13C4
	unsigned int Die_X_position = Read_InfoSeg(0x13C8) + ((int)Read_InfoSeg(0x13C9) << 8); // Address 0x13C8
	unsigned int Die_Y_position = Read_InfoSeg(0x13CA) + ((int)Read_InfoSeg(0x13CB) << 8); // Address 0x13CA
	unsigned int Test_results = Read_InfoSeg(0x13CC) + ((int)Read_InfoSeg(0x13CD) << 8); // Address 0x13CC
	unsigned char REF_Calibration_Tag = Read_InfoSeg(0x13CE); // Address 0x13CE
	unsigned char REF_Calibration_Length = Read_InfoSeg(0x13CF); // Address 0x13CF
	unsigned char Calibrate_REF_REFCAL1 = Read_InfoSeg(0x13D0); // Address 0x13D0
	unsigned char Calibrate_REF_REFCAL0 = Read_InfoSeg(0x13D1); // Address 0x13D1
	unsigned char DCO_Calibration_Tag = Read_InfoSeg(0x13D2); // Address 0x13D2
	unsigned char DCO_Calibration_Length = Read_InfoSeg(0x13D3); // Address 0x13D3
	unsigned char Calibrate_DCO_CSIRFCAL = Read_InfoSeg(0x13D4); // Address 0x13D4
	unsigned char Calibrate_DCO_CSIRTCAL = Read_InfoSeg(0x13D5); // Address 0x13D5
	unsigned char Calibrate_DCO_CSERFCAL = Read_InfoSeg(0x13D6); // Address 0x13D6
	unsigned char Calibrate_DCO_CSERTCAL = Read_InfoSeg(0x13D7); // Address 0x13D7
	unsigned char SD24_Calibration_Tag = Read_InfoSeg(0x13D8); // Address 0x13D8
	unsigned char SD24_Calibration_Length = Read_InfoSeg(0x13D9); // Address 0x13D9
	unsigned char Calibrate_SD24 = Read_InfoSeg(0x13DA); // Address 0x13DA
	unsigned char Tag_Empty = Read_InfoSeg(0x13DC); // Address 0x13DC
	unsigned char Empty_Length = Read_InfoSeg(0x13DD); // Address 0x13DD

	FCTL2 = FWKEY | FSSEL_1 | FN1 | FN3 | FN5;  // MCLK/42 for Flash Timing Generator

	//Erase_InfoSeg(); // !!! Erases the entire information segment (Also the TLV) !!!
	Address = 0x1000;
	Write_InfoSeg(Current_ADC_Offset >> 8); // Write value to Information Segment
	Write_InfoSeg(Current_ADC_Offset); // Write value to Information Segment
	Address++;
	Write_InfoSeg(Voltage_ADC_Offset >> 8); // Write value to Information Segment
	Write_InfoSeg(Voltage_ADC_Offset); // Write value to Information Segment
	Address++;

	// Restore TLV
	Address = 0x013C0;
	Write_InfoSeg(TLV_checksum);
	Write_InfoSeg(TLV_checksum >> 8);
	Write_InfoSeg(Die_Record_Tag);
	Write_InfoSeg(Die_Record_Length);
	Write_InfoSeg(LotWafer_ID);
	Write_InfoSeg(LotWafer_ID >> 8);
	Write_InfoSeg(LotWafer_ID >> 16);
	Write_InfoSeg(LotWafer_ID >> 24);
	Write_InfoSeg(Die_X_position);
	Write_InfoSeg(Die_X_position >> 8);
	Write_InfoSeg(Die_Y_position);
	Write_InfoSeg(Die_Y_position >> 8);
	Write_InfoSeg(Test_results);
	Write_InfoSeg(Test_results >> 8);
	Write_InfoSeg(REF_Calibration_Tag);
	Write_InfoSeg(REF_Calibration_Length);
	Write_InfoSeg(Calibrate_REF_REFCAL1);
	Write_InfoSeg(Calibrate_REF_REFCAL0);
	Write_InfoSeg(DCO_Calibration_Tag);
	Write_InfoSeg(DCO_Calibration_Length);
	Write_InfoSeg(Calibrate_DCO_CSIRFCAL);
	Write_InfoSeg(Calibrate_DCO_CSIRTCAL);
	Write_InfoSeg(Calibrate_DCO_CSERFCAL);
	Write_InfoSeg(Calibrate_DCO_CSERTCAL);
	Write_InfoSeg(SD24_Calibration_Tag);
	Write_InfoSeg(SD24_Calibration_Length);
	Write_InfoSeg(Calibrate_SD24);
	Write_InfoSeg(Tag_Empty);
	Write_InfoSeg(Empty_Length);
*/

//================================================================================================================
//
// Register setup
//
//================================================================================================================
	WDTCTL = WDTPW | WDTHOLD; // Stop WDT

    initSPI(); // Configure eUSCI_B0 for SPI operation
    initUART();
    initSD24();
    initTIMER_A0();
    initTIMER_A1();
    initPORT_1();
    initPORT_2();
    SD24CCTL2 |= SD24SC; // Start conversion

    Timer_Value = 47; // 47 = 3.5 cycle @ 19200 baud
    Start_Timer_A1(Timer_Value); // Start Timer 3.5

    __bis_SR_register(GIE); // General interrupt enable

//================================================================================================================
//
// Calibration
//
//================================================================================================================
    if (OSR_CHn == 0b01)
        Max_Counter *= 4; // 2 X amount of cycles
    if (OSR_CHn == 0b10)
        Max_Counter *= 8; // 4 X amount of cycles
    if (OSR_CHn == 0b11)
        Max_Counter *= 16; // 8 X amount of cycles

    float Current_Calibration = 1 / (28296.63 * 16 * 0.05); // CTratio / (28296.63 * GAIN * Rshunt(0.05))
    float Voltage_Calibration = 1208 / (28296.63 * 1); // Rdivider(1208) / (28296.63 * GAIN)
    float Power_Calibration = Current_Calibration * Voltage_Calibration;

//================================================================================================================
//
// Main loop
//
//================================================================================================================
    P1DIR |= BIT1;
    P1OUT &= ~BIT1;

    while (1)
    {
//================================================================================================================
//
// Current calculation
//
//================================================================================================================
        if (Current_Flag == 1)
        {
            calculate_voltage();
            if (Current_Counter_main > 0) // Avoid division by 0
            {
                Current_Counter_divided = (Numerator / Current_Counter_main); // Calculate 1/Counter (this is faster than Sum/Counter)
                Current_Mean = (Current_Sum_main * Current_Counter_divided);

                // Rolling average
                Current_Roll_Sum = Current_Roll_Sum - Current_Roll[Current_Roll_Counter] + Current_Mean;
                Current_Roll[Current_Roll_Counter] = Current_Mean;
                Current_Roll_Counter++;
                if (Current_Roll_Counter == Roll_Avg_Size)
                {
                    Current_Roll_Counter = 0;
                }
                Current_Roll_Avg = Current_Roll_Sum * 0.1;

                // Final RMS calculation
                Current_RMS = Sqrt_32(Current_Roll_Avg);
                Actual_Current_RMS = Current_RMS * (Current_Calibration * 100);
                Current_RMS_DAC = Sqrt_32(Current_Roll_Avg) * 2.82; //3.96; //2.84 (previous value) // Sqrt of long type is always an int type//rolling avg

                // Reset
                Current_Sum_main = 0;
                Current_Counter_main = 0;
            }
        }

//================================================================================================================
//
// Voltage calculation
//
//================================================================================================================
        if (Voltage_Flag == 1)
        {
            if (Voltage_Counter_main > 0) // Avoid division by 0
            {
                Voltage_Counter_divided = (Numerator / Voltage_Counter_main); // Calculate 1/Counter (this is faster than Sum/Counter)
                Voltage_Mean = (Voltage_Sum_main * Voltage_Counter_divided);

                // Rolling average
                Voltage_Roll_Sum = Voltage_Roll_Sum - Voltage_Roll[Voltage_Roll_Counter] + Voltage_Mean;
                Voltage_Roll[Voltage_Roll_Counter] = Voltage_Mean;
                Voltage_Roll_Counter++;
                if (Voltage_Roll_Counter == Roll_Avg_Size)
                {
                    Voltage_Roll_Counter = 0;
                }
                Voltage_Roll_Avg = Voltage_Roll_Sum * 0.1;

                // Final RMS calculation
                Voltage_RMS = Sqrt_32(Voltage_Roll_Avg); // Sqrt of long type is always an int type
                Actual_Voltage_RMS = Voltage_RMS * (Voltage_Calibration * 100);

                // Reset
                Voltage_Sum_main = 0;
                Voltage_Counter_main = 0;

//================================================================================================================
//
// Power calculation
//
//================================================================================================================
                True_Power_Mean = (Power_Sum_main * Voltage_Counter_divided);

                // Rolling average
                Power_Roll_Sum = Power_Roll_Sum - Power_Roll[Power_Roll_Counter] + True_Power_Mean;
                Power_Roll[Power_Roll_Counter] = True_Power_Mean;
                Power_Roll_Counter++;
                if (Power_Roll_Counter == Roll_Avg_Size)
                {
                    Power_Roll_Counter = 0;
                }
                True_Power = Power_Roll_Sum * 0.1;
                Actual_True_Power = True_Power * (Power_Calibration * 100);

                // Reset
                Power_Sum_main = 0;

                __disable_interrupt(); // Disable all interrupts for hardware multiplication
                MPY = Current_RMS; // Hardware multiplication (unsigned) to calculate apparent power
                OP2 = Voltage_RMS;
                Apparent_Power = (((long)RESHI << 16) + RESLO); // Get result

                MPY = Apparent_Power; // Hardware multiplication (unsigned)
                OP2 = Apparent_Power;
                Apparent_Power_Square = (((long)RESHI << 16) + RESLO); // Get result

                MPY = (Apparent_Power >> 16); // Hardware multiplication (unsigned)
                OP2 = (Apparent_Power >> 16);
                Apparent_Power_Square += (long long)(((long)RESHI << 16) + RESLO) << 32; // Get result

                MPY = True_Power; // Hardware multiplication (unsigned)
                OP2 = True_Power;
                True_Power_Square = (((long)RESHI << 16) + RESLO); // Get result

                MPYS = (True_Power >> 16); // Hardware multiplication (unsigned)
                OP2 = (True_Power >> 16);
                True_Power_Square += (long long)(((long)RESHI << 16) + RESLO) << 32; // Get result
                __enable_interrupt(); // Enable all interrupts --> GIE = 1 (HIGH)

                Actual_Apparent_Power = Apparent_Power * (Power_Calibration * 100);

                if (Apparent_Power_Square == 0 || Apparent_Power_Square <= True_Power_Square)
                {
                    Actual_Reactive_Power = 0;
                }
                else
                {
                    Reactive_Power_Square = Apparent_Power_Square - True_Power_Square;
                    Reactive_Power = Sqrt_64(Reactive_Power_Square);
                    Actual_Reactive_Power = Reactive_Power * (Power_Calibration * 100);
                    if (True_Power < 0)
                    {
                        Actual_Reactive_Power *= -1;
                    }
                }

//================================================================================================================
//
// Power factor calculation
//
//================================================================================================================
                if (Apparent_Power == 0) // Prevents the power factor from reaching weird values.
                {
                    PF = 0;
                }
                else
                {
                    PF = ((float)True_Power / (float)Apparent_Power) * 100; // Apparently floating stuff fixes the calculation...
                }

//================================================================================================================
//
// Prepare values for energy calculation
//
//================================================================================================================
                Sum_Actual_True_Power += Actual_True_Power;
                if (PF > 0)
                {
                    Sum_Consumed += Actual_True_Power;
                }
                else
                {
                    Sum_Supplied += (Actual_True_Power * -1);
                }
                Counter_Energy++;

//================================================================================================================
//
// Update the values within the holding registers
//
//================================================================================================================
                Holding_Register[0] = Actual_Current_RMS;
                Holding_Register[1] = Actual_Voltage_RMS;
                Holding_Register[2] = Actual_True_Power;
                Holding_Register[3] = Actual_Apparent_Power;
                Holding_Register[4] = Actual_Reactive_Power;
                Holding_Register[5] = PF;
                Holding_Register[6] = Netto_Energy;
                Holding_Register[7] = Consumed_Energy;
                Holding_Register[8] = Supplied_Energy;
                Holding_Register[9] = Slave_ID;
            }
        }

//================================================================================================================
//
// MODBUS loop
//
//================================================================================================================
        while (UART_Flag == 1)
        {
            switch(MB_State)
            {
                case Address_Val: // Address validation
                    P1OUT |= BIT1; // Set transmit bit
                    if (UART_Data_Received[0] == Slave_Address)
                    {
                        MB_State = Function_Val;
                    }
                    else if (UART_Data_Received == 0)
                    {
                        // Broadcast mode
                        MB_State = Address_Val;
                        UART_Flag = 0;
                        P1OUT &= ~BIT1; // Reset transmit bit
                    }
                    else
                    {
                        // Wrong address
                        MB_State = Address_Val;
                        UART_Flag = 0;
                        P1OUT &= ~BIT1; // Reset transmit bit
                    }
                case Function_Val: // Function validation
                    if (UART_Data_Received[1] == 3)
                    {
                        MB_State = Start_Val;
                    }
                    else
                    {
                        MB_State = Error_Gen;
                    }
                    break;
                case Start_Val: // Start address validation
                    Start_Address = ((unsigned int)UART_Data_Received[2] << 8) + UART_Data_Received[3];
                    if (Start_Address < 10)
                    {
                        MB_State = No_of_Reg_Val;
                    }
                    else
                    {
                        MB_State = Error_Gen;
                    }
                    break;
                case No_of_Reg_Val: // Check if number of asked registers are available
                    No_of_Registers = ((unsigned int)UART_Data_Received[4] << 8) + UART_Data_Received[5];
                    if (No_of_Registers < 11 - Start_Address)
                    {
                        MB_State = CRC_Val;
                    }
                    else
                    {
                        MB_State = Error_Gen;
                    }
                    break;
                case CRC_Val: // CRC validation
                    Received_CRC = UART_Data_Received[6] + ((unsigned int)UART_Data_Received[7] << 8);
                    if (CRC_Calculator(6, UART_Data_Received) == Received_CRC)
                    {
                        MB_State = Reply_Gen;
                    }
                    else
                    {
                        MB_State = Error_Gen;
                    }
                    break;
                case Error_Gen: // Error package
                    Return_Package[0] = Slave_Address;
                    Return_Package[1] = UART_Data_Received[1] + 0x80; // Add error code to function code
                    Return_Package[2] = 0; // Number of data bytes
                    Send_CRC = CRC_Calculator(3, Return_Package); // Calculate CRC
                    Return_Package[3] = (unsigned char)(Send_CRC & 0xFF);
                    Return_Package[4] = (unsigned char)(Send_CRC >> 8);
                    n = 4;
                    MB_State = Transmission;
                    break;
                case Reply_Gen: // Reply package
                    Return_Package[0] = Slave_Address; // Address
                    Return_Package[1] = UART_Data_Received[1]; // Function code
                    Return_Package[2] = No_of_Registers * 2; // Number of data bytes
                    m = 0;
                    for (n = 3; n < (No_of_Registers * 2 + 3); n++) // Load requested registers into the Return_Package array
                    {
                        Return_Package[n] = (unsigned char)(Holding_Register[Start_Address + m] >> 8);
                        Return_Package[n++] = (unsigned char)(Holding_Register[Start_Address + m] & 0xFF);
                        m++;
                    }
                    Send_CRC = CRC_Calculator(n, Return_Package); // Calculate CRC
                    Return_Package[n] = (unsigned char)(Send_CRC & 0xFF);
                    n++;
                    Return_Package[n] = (unsigned char)(Send_CRC >> 8);
                    MB_State = Transmission;
                    break;
                case Transmission: // Transmission
                    Package_Flag = 1;
                    __disable_interrupt(); // Disable all interrupts
                    UCA0IE |= UCTXIE;
                    __enable_interrupt(); // Enable all interrupts --> GIE = 1 (HIGH)
                    MB_State = Address_Val;
                    UART_Flag = 0;
                    break;
                default:
                    MB_State = Address_Val;
                    break;
            }
        }

//================================================================================================================
//
// Temperature calculation
//
//================================================================================================================
        Temperature = ((float)((unsigned long)Results_ADC_Temperature * 1200) / 70711) - 273; // In degrees celcius

//================================================================================================================
//
// Send data to the DAC through SPI
//
//================================================================================================================
        Data = DAC_Data_Converter(Current_RMS_DAC); // Convert data for 4-20mA use
        SPI_Send(Command,Data); // SPI function
    }
}

//################################################################################################################
//
// Interrupt handlers
//
//################################################################################################################

//================================================================================================================
//
// SD24_ISR
//
// Input: -
// Output: -
// Effect:  Processes the samples and keeps count of the number of samples, for both current and voltage.
//          It calculates the sum of the squares of both instantaneous values.
//          It also calculates the instantaneous power.
//
//================================================================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SD24_VECTOR
__interrupt void SD24_ISR(void) {
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SD24_VECTOR))) SD24_ISR (void)
{
#else
#error Compiler not supported!
#endif
    switch (__even_in_range(SD24IV,SD24IV_SD24MEM3))
    {
        case SD24IV_NONE: break;
        case SD24IV_SD24OVIFG: break;
        case SD24IV_SD24MEM0:  break;
        case SD24IV_SD24MEM1:  break;
        case SD24IV_SD24MEM2:

            // Process ADC temperature data
            Results_ADC_Temperature = SD24MEM2; // Save Channel 2 results (clears IFG)

            // Process ADC current data
            Results_ADC_Current = SD24MEM0; // Save Channel 0 results
            //Calibration = (Read_InfoSeg(0x1000) + ((int)Read_InfoSeg(0x1001) << 8));
            //Results_Current[i_C] = (Results_ADC_Current - Calibration); // Adjust for the zero point 256 OSR 16 gain
            Results_Current[i_C] = (Results_ADC_Current - Current_ADC_Offset); // Adjust for the zero point 256 OSR 16 gain
            New_Current = Results_Current[i_C];
            Current_Offset += Results_Current[i_C]; // Used for average calculation for calibration

            // Process ADC voltage data
            Results_ADC_Voltage = SD24MEM1; // Save Channel 1 results
            //Calibration = (Read_InfoSeg(0x1003) + ((int)Read_InfoSeg(0x1004) << 8));
            //Results_Voltage[i_V] = (Results_ADC_Voltage - Calibration); // Adjust for the zero point
            Results_Voltage[i_V] = (Results_ADC_Voltage - Voltage_ADC_Offset); // Adjust for the zero point
            New_Voltage = Results_Voltage[i_V];
            Voltage_Offset += Results_Voltage[i_V]; // Used for average calculation for calibration

            // Hardware multiplication
            __disable_interrupt(); // Disable all interrupts for hardware multiplication
            MPYS = New_Current; // Hardware multiplication (signed) to calculate the square of the sample
            OP2 = New_Current;
            Current_Square = (((long)RESHI << 16) + RESLO);
            MPYS = New_Current; // Hardware multiplication (signed) to calculate instantaneous power
            OP2 = New_Voltage;
            Instant_Power = (((long)RESHI << 16) + RESLO); // Get result
            MPYS = New_Voltage; // Hardware multiplication (signed) to calculate the square of the sample
            OP2 = New_Voltage;
            Voltage_Square = (((long)RESHI << 16) + RESLO); // Get result
            __enable_interrupt(); // Enable all interrupts --> GIE = 1 (HIGH)

            // Current processing
            if (Current_Counter > 0) // Two ADC values have been acquired
            {
                  if (Prev_Current <= 0 && New_Current > 0) // A new cycle has begun (zero crossing detected)
                  {
                      Current_Zero_Crossing++;
                      if (Current_Zero_Crossing >= 2) // 20ms is too short for DAC to process data
                      {
                          Current_Flag = 1; // Set flag for main loop
                          Current_Counter_main = Current_Counter; // Store values in new variable
                          Current_Sum_main = Current_Sum; // Store values in new variable

                          // Reset for new cycle
                          Current_Sum = Current_Square;
                          Current_Counter = 1;
                          Current_Zero_Crossing = 0;
                      }
                      else // 2 cycles have not yet passed
                      {
                          Current_Counter++;
                          Current_Sum += Current_Square;
                          Current_Flag = 0; // Reset flag for zero crossing
                      }
                  }
                  else // An old cycle is going on
                  {
                      Current_Counter++;
                      Current_Sum += Current_Square;
                      Current_Flag = 0; // Reset flag for zero crossing

                      if (Current_Counter > Max_Counter) //0hz issue; set a lower frequency detection limit =~ 37.5Hz
                      {
                          Current_Flag = 1; // Set flag for main loop, finish the periodic cycle
                          Current_Counter_main = Current_Counter; // Store values in new variable to be accessed in main loop
                          Current_Sum_main = Current_Sum; // Store values in new variable to be accessed in main loop

                          // Reset for new cycle
                          Current_Sum = Current_Square;
                          Current_Counter = 1;
                          Current_Zero_Crossing = 0;
                      }
                  }
                  Prev_Current = New_Current; // Assign the current value to the last value for next round
            }
            else // Only one ADC value has been acquired
            {
                Prev_Current = New_Current; // Assign the current value to the last value for next round
                Current_Counter = 1;
            }

            // Voltage processing
            if (Voltage_Counter > 0) // Two ADC values have been acquired
            {
                  if (Prev_Voltage <= 0 && New_Voltage > 0) // A new cycle has begun (zero crossing detected)
                  {
                      Voltage_Zero_Crossing++;
                      if (Voltage_Zero_Crossing >= 2) // 2 cycles have passed
                      {
                          Voltage_Flag = 1; // Set flag for main loop
                          Voltage_Counter_main = Voltage_Counter; // Store values in new variable
                          Voltage_Sum_main = Voltage_Sum; // Store values in new variable
                          Power_Sum_main = Power_Sum; // Store values in new variable

                          // Reset for new cycle
                          Voltage_Sum = Voltage_Square;
                          Power_Sum = Instant_Power;
                          Voltage_Counter = 1;
                          Voltage_Zero_Crossing = 0; // Reset flag for zero crossing
                      }
                      else // 2 cycles have not yet passed
                      {
                          Voltage_Counter++;
                          Voltage_Sum += Voltage_Square;
                          Power_Sum += Instant_Power;
                          Voltage_Flag = 0; // Reset flag for zero crossing
                      }
                  }
                  else // An old cycle is going on
                  {
                      Voltage_Counter++;
                      Voltage_Sum += Voltage_Square;
                      Power_Sum += Instant_Power;
                      Voltage_Flag = 0; // Reset flag for zero crossing

                      if (Voltage_Counter > Max_Counter) //0hz issue; set a lower frequency detection limit =~ 37.5Hz
                      {
                          Voltage_Flag = 1; // Set flag for main loop, finish the periodic cycle
                          Voltage_Counter_main = Voltage_Counter; // Store values in new variable to be accessed in main loop
                          Voltage_Sum_main = Voltage_Sum; // Store values in new variable to be accessed in main loop
                          Power_Sum_main = Power_Sum; // Store values in new variable

                          // Reset for new cycle
                          Voltage_Sum = Voltage_Square;
                          Power_Sum = Instant_Power;
                          Voltage_Counter = 1;
                          Voltage_Zero_Crossing = 0; // Reset flag for zero crossing
                      }
                  }
                  Prev_Voltage = New_Voltage; // Assign the voltage value to the last value for next round
            }
            else // Only one ADC value has been acquired
            {
                Prev_Voltage = New_Voltage; // Assign the current value to the last value for next round
                Voltage_Counter = 1;
            }

            // Update counters
            i_C++;
            if (i_C >= 101)
            {
                //Current_Offset *= 0.01; // Division for average calculation for calibration
                i_C = 0; // Reset index
                Current_Offset = 0;
            }
            i_V++;
            if (i_V >= 101)
            {
                //Voltage_Offset *= 0.01; // Division for average calculation for calibration
                i_V = 0; // Reset index
                Voltage_Offset = 0;
            }
            break;
        case SD24IV_SD24MEM3: break;
        default: break;
    }
}

//================================================================================================================
//
// USCI_A0_ISR
//
// Input: -
// Output: -
// Effect:  The receive interrupt handles the MODBUS data collection and the start byte detection
//          The transmit interrupt handles the MODBUS data transmission and resets afterwards for MODBUS data collection
//
//================================================================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void USCI_A0_ISR(void)
{
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            UART_Data_Received[i] = UCA0RXBUF; // Get character (Clears IFG)
            switch (RX_State)
            {
                case State_INIT:
                    break;
                case State_IDLE:
                    Timer_Value = 20; // 20 = 1.5 cycles @ 19200 baud
                    Start_Timer_A1(Timer_Value); // Start Timer 1.5
                    i++;
                    RX_State = State_Receive;
                    break;
                case State_Receive:
                    if (Timer_Flag == 1)
                    {
                        i = 0;
                        Timer_Value = 47; // 47 = 3.5 cycles @ 19200 baud
                        Start_Timer_A1(Timer_Value); // Start Timer 3.5
                        RX_State = State_ERROR;
                        Timer_Flag = 0;
                    }
                    else
                    {
                        if (i >= 7)
                        {
                            i = 0;
                            Timer_Value = 47; // 47 = 3.5 cycles @ 19200 baud
                            Start_Timer_A1(Timer_Value); // Start Timer 3.5
                            UART_Flag = 1;
                        }
                        else
                        {
                            Timer_Value = 20; // 20 = 1.5 cycles @ 19200 baud
                            Start_Timer_A1(Timer_Value); // Start Timer 1.5
                            i++;
                        }
                    }
                    break;
                case State_ERROR:
                    i = 0;
                    Timer_Value = 47; // 47 = 3.5 cycles @ 19200 baud
                    Start_Timer_A1(Timer_Value); // Start Timer 2
                    break;
            }
            break;
        case USCI_UART_UCTXIFG:
            // Modbus data transmission
            if (Package_Flag == 1)
            {
                if (j <= n)
                {
                    UCA0TXBUF = Return_Package[j];
                    j++;
                }
                else // Reset
                {
                    j = 0;
                    Package_Flag = 0;
                    for (n = 26; n > 0; n--) // Reset the Return_Package array
                    {
                        Return_Package[n] = 0;
                    }
                    while (UCA0STATW&UCBUSY); // Wait until data has been send
                    __disable_interrupt(); // Disable all interrupts
                    initUART(); // Re-initializes the UART
                    __enable_interrupt(); // Enable all interrupts --> GIE = 1 (HIGH)
                    P1OUT &= ~BIT1; // Reset transmit bit
                }
            }
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

//================================================================================================================
//
// TA0_ISR
//
// Input: -
// Output: -
// Effect: Gives an interrupt after one second
//
//================================================================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TA0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (Counter_Energy > 0) // Avoid division by 0
    {
        Netto_Energy += (Sum_Actual_True_Power / Counter_Energy);
        Consumed_Energy += (Sum_Consumed / Counter_Energy);
        Supplied_Energy += (Sum_Supplied / Counter_Energy);

        // Reset the sums and counter
        Sum_Actual_True_Power = 0;
        Sum_Consumed = 0;
        Sum_Supplied = 0;
        Counter_Energy = 0;
    }
}

//================================================================================================================
//
// TA1_ISR
//
// Input: -
// Output: -
// Effect: Detects if 1.5 or 3.5 characters of pasue have passed
//
//================================================================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TA1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TA1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    PollModbus();
    if (Timer_Value == 47) // 3.5 characters
    {
        TA1CCTL0 &= ~CCIE; // Stop timer
        TA1CTL &= ~(TASSEL_1 | MC_1 | ID_0);
        RX_State = State_IDLE;
    }
    if (Timer_Value == 27) // 2 characters
    {
        TA1CCTL0 &= ~CCIE; // Stop timer
        TA1CTL &= ~(TASSEL_1 | MC_1 | ID_0);
        RX_State = State_IDLE;
    }
    if (Timer_Value == 20) // 1.5 character
    {
        Timer_Flag = 1;
        Timer_Value = 27;
        Start_Timer_A1(Timer_Value);
    }

}

//================================================================================================================
//
// PORT1_ISR
//
// Input: -
// Output: -
// Effect: Updates the Slave_Address and changes the direction of the interrupt
//
//================================================================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) PORT1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(P1IV,0x10))
    {
        case 0x00: break;               // No interrupt
        case 0x02:                      // P1.0 interrupt
            Slave_Address = MODBUS_Address();
            __disable_interrupt();
            P1IES ^= BIT0; // Flip the interrupt direction
            __enable_interrupt();
            P1IFG &= ~BIT0;
            break;
        case 0x04: break;               // P1.1 interrupt
        case 0x06: break;               // P1.2 interrupt
        case 0x08: break;               // P1.3 interrupt
        case 0x0A: break;               // P1.4 interrupt
        case 0x0C: break;               // P1.5 interrupt
        case 0x0E: break;               // P1.6 interrupt
        case 0x10: break;               // P1.7 interrupt
    }
}

//================================================================================================================
//
// PORT2_ISR
//
// Input: -
// Output: -
// Effect: Updates the Slave_Address and changes the direction of the interrupts
//
//================================================================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) PORT2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(P2IV,0x08))
    {
        case 0x00: break;               // No interrupt
        case 0x02:                      // P2.0 interrupt
            Slave_Address = MODBUS_Address();
            __disable_interrupt();
            P2IES ^= BIT0; // Flip the interrupt direction
            __enable_interrupt();
            P2IFG &= ~BIT0; // Clear IFG
            break;
        case 0x04:              // P2.1 interrupt
            Slave_Address = MODBUS_Address();
            __disable_interrupt();
            P2IES ^= BIT1; // Flip the interrupt direction
            __enable_interrupt();
            P2IFG &= ~BIT1; // Clear IFG
            break;
        case 0x06:              // P2.2 interrupt
            Slave_Address = MODBUS_Address();
            __disable_interrupt();
            P2IES ^= BIT2; // Flip the interrupt direction
            __enable_interrupt();
            P2IFG &= ~BIT2; // Clear IFG
            break;
        case 0x08:              // P2.3 interrupt
            Slave_Address = MODBUS_Address();
            __disable_interrupt();
            P2IES ^= BIT3; // Flip the interrupt direction
            __enable_interrupt();
            P2IFG &= ~BIT3; // Clear IFG
            break;
    }
}

//################################################################################################################
//
// Initialization functions
//
//################################################################################################################

//================================================================================================================
//
// initSD24
//
// Input: -
// Output: -
// Effect: Initializes 3 ADC channels with an interrupt on channel 2
//
//================================================================================================================
void initSD24()
{
    SD24CTL = SD24REFS; // Internal ref? yes

    // Channel 0 single mode, Group with Channel 2
    SD24CCTL0 |= (OSR_CHn << 8) | SD24GRP | SD24DF; // OSR, Group, 2s compliment
    SD24INCTL0 |= (Gain_CH0 << 3); // "n"x gain

    // Channel 1 single mode, Group with Channel 2
    SD24CCTL1 |= (OSR_CHn << 8) | SD24GRP | SD24DF; // OSR, Group, 2s compliment
    SD24INCTL1 |= (Gain_CH1 << 3); // "n"x gain

    // Channel 2 temperature, enable interrupt
    SD24CCTL2 |= (OSR_CHn << 8) | SD24DF; // OSR, 2s compliment
    SD24INCTL2 |= (Gain_CH2 << 3) | SD24INCH_6; // Temperature sensor
    SD24CCTL2 |= SD24IE; // Enable interrupt

    // Delay ~200us for 1.2V ref to settle
    __delay_cycles(3200);
}

//================================================================================================================
//
// initSPI
//
// Input: -
// Output: -
// Effect: Initializes UCB0 for SPI operation
//
//================================================================================================================
void initSPI()
{
    // GPIO
    P1DIR |= BIT4; // Slave select pin
    P1SEL0 |=   BIT5 | BIT6 | BIT7;         // eUSCI_B0 Pin Function
    P1SEL1 &= ~(BIT5 | BIT6 | BIT7);

    // Configure eUSCI_B0 for SPI operation
    UCB0CTLW0 = UCSWRST;                    // **Put state machine in reset**
    UCB0CTLW0 |= UCMSB | UCMST | UCSYNC | UCCKPH; // Set MSB first, Set to master mode,  Set to synchronous mode, SPI mode 3
    UCB0CTLW0 |= UCSSEL_2;                  // SMCLK
    UCB0BR0 = 0x02;                         // /2
    UCB0BR1 = 0;                            //
    UCB0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
}

//================================================================================================================
//
// initUART
//
// Input: -
// Output: -
// Effect: Initializes UCA0 for UART operation
//
//================================================================================================================
void initUART()
{
    // GPIO
    P1SEL0 |=   BIT2 | BIT3;                // P1.2/3 eUSCI_A Function & eUSCI_B
    P1SEL1 &= ~(BIT2 | BIT3);

    UCA0CTL1 |= UCSWRST;                    // Hold eUSCI in reset
    UCA0CTLW0 |= UCPEN | UCPAR;              // Even parity
    UCA0CTL1 |= UCSSEL_2;                   // SMCLK
    UCA0BR0   = 0x55;                       // 19200 baud
    UCA0BR1   = 0x03;
    UCA0MCTLW = 0x4900;                     // 16.384MHz/19200 = 853.333(See UG) error
    UCA0CTL1 &= ~UCSWRST;                   // Release from reset
    UCA0IE   |= UCRXIE;                     // Enable RX interrupt
}

//================================================================================================================
//
// initTIMER_A0
//
// Input: -
// Output: -
// Effect: Initializes Timer A0 based of ACLK
//
//================================================================================================================
void initTIMER_A0()
{
    TA0CCTL0 = CCIE;  // CCR0 Interrupt Enabled
    TA0CCR0 = 32756;  // 32756 (frequency of ACLK)
    TA0CTL = TASSEL_1 | MC_1 | ID_0;    // ACLK/1, Up Mode
}

//================================================================================================================
//
// initTIMER_A1
//
// Input: -
// Output: -
// Effect: Initializes Timer A1 based of ACLK, used to detect the 1.5 and 3.5 characters of pause in MODBUS
//
//================================================================================================================
void initTIMER_A1()
{
    TA1CCTL0 = CCIE;  // CCR0 Interrupt Enabled
    TA1CCR0 = 47;  // 28 clock cycles at 19200 baud equals 47.8 clock cycles at 32768 Hz.
    TA1CTL = TASSEL_1 | MC_1 | ID_0;    // ACLK/1, Up Mode
}

//================================================================================================================
//
// initPORT_1
//
// Input: -
// Output: -
// Effect: Enables the interrupt on P1.0
//
//================================================================================================================
void initPORT_1()
{
    P1IES &= ~BIT0; // Interrupt on lo/hi edge
    P1IE |= BIT0; // Enable interrupt
    P1IFG &= ~BIT0; // Clear IFG
}

//================================================================================================================
//
// initPORT_2
//
// Input: -
// Output: -
// Effect: Enables the interrupt on P2.0, P2.1, P2.2 and P2.3
//
//================================================================================================================
void initPORT_2()
{
    P2IES &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Interrupt on lo/hi edge
    P2IE |= BIT0 | BIT1 | BIT2 | BIT3; // Enable interrupt
    P2IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3); // Clear IFG
}

//################################################################################################################
//
// Other functions
//
//################################################################################################################

//================================================================================================================
//
// Erase_InfoSeg
//
// Input: -
// Output: -
// Effect: Erases the information memory
//
//================================================================================================================
void Erase_InfoSeg()
{
    // Local variables
	unsigned char *Flash_ptr; // Flash pointer

	Flash_ptr = (unsigned char *)Address; // Initialize Flash pointer

    if (FCTL3 & LOCKSEG) // If Info Seg is still locked
    {
        FCTL3 = FWKEY | LOCKSEG; // Clear LOCKSEG bit
    }
    FCTL1 = FWKEY | ERASE; // Set ERASE bit for write operation

    *Flash_ptr = 0; // Write dummy value to flash

    FCTL1 = FWKEY; // Clear WRT bit
    FCTL3 = FWKEY | LOCKSEG; // Set LOCKSEG bit
}

//================================================================================================================
//
// Write_InfoSeg
//
// Input: Value to store inside the information memory
// Output: -
// Effect: Writes data to the information memory
//
//================================================================================================================
void Write_InfoSeg(char Value)
{
    // Local variables
	unsigned char *Flash_ptr; // Flash pointer

	Flash_ptr = (unsigned char *)Address; // Initialize Flash pointer

    if (FCTL3 & LOCKSEG) // If Info Seg is still locked
    {
        FCTL3 = FWKEY | LOCKSEG; // Clear LOCKSEG bit
    }
    FCTL1 = FWKEY | WRT; // Set WRT bit for write operation

    *Flash_ptr = Value; // Write value to flash
    Address++;

    FCTL1 = FWKEY; // Clear WRT bit
    FCTL3 = FWKEY | LOCKSEG; // Set LOCKSEG bit
}

//================================================================================================================
//
// Read_InfoSeg
//
// Input: Address
// Output: -
// Effect: Reads data from the information memory
//
//================================================================================================================
char Read_InfoSeg(unsigned int Address2)
{
    // Local variables
	unsigned char *Flash_ptr; // Flash pointer

	Flash_ptr = (unsigned char *)Address2;
	return *Flash_ptr;
}

//================================================================================================================
//
// SPI_Send
//
// Input: Command, Data
// Output: -
// Effect: Sends 24 bits of data through the SPI protocol
//
//================================================================================================================
void SPI_Send(unsigned char Command, unsigned int Data)
{
    // Local variables
    unsigned char Trash;
    unsigned char SPI_State = 0;

    while (SPI_State < 3)
    {
        switch (SPI_State)
        {
            case 0:
                P1OUT &= ~BIT4; // Set slave select low
                if ((UCB0STATW & UCFE) == UCFE) // Check for framing error
                {
                    UCB0STATW &= 0x0040; // Clear framing error
                }
                else
                {
                    SPI_State = 1;
                }
                break;
            case 1:
                if ((UCB0STATW & UCOE) == UCOE) // Check for overrun error flag (RX)
                {
                    Trash = UCB0RXBUF; // Empty the RX buffer
                }
                else
                {
                    SPI_State = 2;
                }
                break;
            case 2:
                UCB0TXBUF = Command; // Transmit data (8 bit)
                while (UCB0STATW&UCBUSY);
                UCB0TXBUF = (Data >> 8); // Transmit data (8 bit)
                while (UCB0STATW&UCBUSY);
                UCB0TXBUF = (Data & 0xFF); // Transmit data (8 bit)
                while (UCB0STATW&UCBUSY);
                P1OUT |= BIT4; // Set slave select high
                SPI_State = 3;
                break;
            case 3:
                break;
            default:
                SPI_State = 0;
        }
    }
}

//================================================================================================================
//
// Sqrt_32
//
// Input: An unsigned long value (32 bit)
// Output: Square root of the input value
// Effect: Calculates the square root of the input value using the Newton-Raphson method (kind of)
//
//================================================================================================================
unsigned int Sqrt_32(unsigned long Input_Value)
{
    // Local variables
	unsigned int c = 0x8000;
    unsigned int g = 0x8000;
    unsigned long g_Square;

    while (c > 0)
    {
        __disable_interrupt(); // Disable all interrupts for hardware multiplication
    	MPY = g; // Hardware multiplication (unsigned)
        OP2 = g;
        g_Square = (((long)RESHI << 16) + RESLO); // Get result
        __enable_interrupt(); // Enable all interrupts --> GIE = 1 (HIGH)
    	if (g_Square > Input_Value)
    	{
            g ^= c; // g = g xor c
    	}
        c >>= 1; // Bitshift c
        g |= c;
    }
    return g;
}

//================================================================================================================
//
// Sqrt_64
//
// Input: An unsigned long long value (64 bit)
// Output: Square root of the input value
// Effect: Calculates the square root of the input value using the Newton-Raphson method (kind of)
//
//================================================================================================================
unsigned long Sqrt_64(unsigned long long Input_Value)
{
    // Local variables
    unsigned long c = 0x80000000;
    unsigned long g = 0x80000000;
    unsigned long long g_Square;

    while (c > 0)
    {
        __disable_interrupt(); // Disable all interrupts for hardware multiplication
        MPY = g; // Hardware multiplication (unsigned)
        OP2 = g;
        g_Square = (((long)RESHI << 16) + RESLO); // Get result

        MPY = (g >> 16); // Hardware multiplication (unsigned)
        OP2 = (g >> 16);
        g_Square += (long long)(((long)RESHI << 16) + RESLO) << 32; // Get result and store in the upper part
        __enable_interrupt(); // Enable all interrupts --> GIE = 1 (HIGH)

        if (g_Square > Input_Value)
        {
            g ^= c; // g = g xor c
        }
        c >>= 1; // Bitshift c
        g |= c;
    }
    return g;
}

//================================================================================================================
//
// CRC_Calculator
//
// Input: Data length of the received MODBUS message
// Output: The CRC result
// Effect: Calculates the CRC based on the CRC-16-IBM polynomial
//
//================================================================================================================
unsigned int CRC_Calculator(unsigned int Data_Length, unsigned char Buffer[])
{
    // Local variables
    unsigned int Checksum = 0xFFFF; // Start value
    unsigned int g;
    unsigned char f;

    for (g = 0; g < Data_Length; g++)
    {
        Checksum ^= (unsigned int)Buffer[g];
        for (f = 8; f > 0; f--)
        {
            if (Checksum & 0x0001)
            {
                Checksum = (Checksum >> 1) ^ 0xA001;
            }
            else
            {
                Checksum >>= 1;
            }
        }
    }
    return Checksum;
}

//================================================================================================================
//
// MODBUS_Address
//
// Input: -
// Output: -
// Effect: Calculates the slave address based on the port input values
//
//================================================================================================================
unsigned char MODBUS_Address()
{
    return ((P2IN & 0x0F) << 1) + (P1IN & BIT0);
}

//================================================================================================================
//
// Start_Timer_A1
//
// Input: Time
// Output: -
// Effect: Starts Timer A1
//
//================================================================================================================
void Start_Timer_A1(unsigned char Time)
{
    __disable_interrupt();
    TA1CTL |= TACLR; // Reset timer A1
    TA1CCR0 = Time;
    TA1CCTL0 = CCIE;  // CCR0 Interrupt Enabled
    TA1CTL = TASSEL_1 | MC_1 | ID_0;    // ACLK/1, Up Mode
    __enable_interrupt();
}
