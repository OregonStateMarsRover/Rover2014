/*
 * adc.cpp
 *
 * Created: 5/6/2014 6:39:20 PM
 *  Author: Nick
 */ 

#include "adc.h"


uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}

uint16_t ReadADC(uint8_t Channel, uint8_t ADCMode) // Mode = 1 for single ended, 0 for internal
{
	if ((ADCA.CTRLA & ADC_ENABLE_bm) == 0)
	{
		ADCA.CTRLA = ADC_ENABLE_bm ; // Enable the ADC
		ADCA.CTRLB = ADC_RESOLUTION_8BIT_gc; // Signed Mode
		ADCA.REFCTRL = ADC_REFSEL_VCC_gc; // Internal 1v ref
		ADCA.EVCTRL = 0; // no events
		ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc ;
		ADCA.CALL = ReadSignatureByte(0x20) ; //ADC Calibration Byte 0
		ADCA.CALH = ReadSignatureByte(0x21) ; //ADC Calibration Byte 1
		_delay_us(400); // Wait at least 25 clocks
	}
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADCMode ; // Gain = 1, Single Ended
	ADCA.CH0.MUXCTRL = (Channel<<3);
	ADCA.CH0.INTCTRL = 0 ; // No interrupt
	for(uint8_t Waste = 0; Waste<2; Waste++)
	{
		ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
		while (ADCA.INTFLAGS==0) ; // Wait for complete
		ADCA.INTFLAGS = ADCA.INTFLAGS ;
	 }
	return ADCA.CH0RES ;
}