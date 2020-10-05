#define THE_DAC_HEADER_H_

unsigned int DAC_Data_Converter(long RMS)
{
    // local variables
    unsigned int DAC_Data;
    unsigned long DAC_Data_Adjusted;
    DAC_Data = (unsigned int)RMS;
    DAC_Data_Adjusted = DAC_Data + 0x2AAA;
    if (DAC_Data_Adjusted >= 0xFFFF) //D555 = 20mA
        DAC_Data_Adjusted = 0xFFFF;
    return (DAC_Data_Adjusted);
    //return (DAC_Data+0x2AAA);
}
