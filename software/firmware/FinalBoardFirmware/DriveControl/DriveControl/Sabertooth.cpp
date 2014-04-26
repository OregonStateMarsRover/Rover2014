/*
 * Sabertooth.cpp
 *
 * Created: 4/25/2014 3:59:35 PM
 *  Author: corwin
 */ 
#define F_CPU 32000000UL

#include "Sabertooth.h"
#include <util/delay.h>
#include <avr/io.h>
#include "usart_driver.h"
#include "avr_compiler.h"

void SabertoothInit(void){
	PORTD.OUTSET = PIN3_bm;																//Set TX to output
	USART_Format_Set(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//USARTD0, 8 Data bits, No Parity, 1 Stop bit.
	USART_Baudrate_Set(&USARTD0, 207 , 0);												//Set USART to 9600 baud for 32MHz core system clock
	USART_Tx_Enable(&USARTD0);															//Enable transmission
	_delay_ms(100);																		//Wait for things to settle
	USART_PutChar(&USARTD0);
	
}