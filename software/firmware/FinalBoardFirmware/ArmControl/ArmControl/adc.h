/*
 * adc.h
 *
 * Contains headers for ADC
 *
 * Created: 5/6/2014 6:36:17 PM
 *  Author: Nick
 */ 


#ifndef ADC_H_
#define ADC_H_

#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <ctype.h>

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

uint8_t ReadSignatureByte(uint16_t Address);
uint16_t ReadADC(uint8_t Channel, uint8_t ADCMode); // Mode = 1 for single ended, 0 for internal

#endif /* ADC_H_ */