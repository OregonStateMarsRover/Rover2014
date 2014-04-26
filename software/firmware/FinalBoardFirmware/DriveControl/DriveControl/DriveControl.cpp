/*
 * DriveControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
 *  Author: corwin
 */ 

#define F_CPU 32000000UL

#include <util/delay.h>
#include <avr/io.h>
#include "usart_driver.h"
#include "avr_compiler.h"

/*! Define that selects the Usart used in example. */
#define USART USARTC0
#define XBEEDIO0 PIN5_bm //PORTA
#define SabertoothTx PIN3_bm //PORTD
#define StatusLight PIN3_bm //PORTE

/*! Success variable, used to test driver. */
bool success;

int main(void)
{
	CCP = CCP_IOREG_gc;              // disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
	CCP = CCP_IOREG_gc;              // disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock

	 
	CCP = CCP_IOREG_gc;
	OSC.CTRL |= OSC_RC32KEN_bm;
	while(!(OSC.STATUS & OSC_RC32KRDY_bm)); // wait for oscillator to be ready
	OSC.DFLLCTRL &= ~OSC_RC32MCREF_bm;
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;  
	 
	 
	 
	 
	 
	/* Variable used to send and receive data. */
	uint8_t sendData[] = "This is a string\r\n";
	uint8_t receivedData;

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTC.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR = PIN2_bm;
	PORTA.DIRCLR = XBEEDIO0;
	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 207 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);


	/* Assume that everything is OK. */
	success = true;
	/* Send data from 255 down to 0*/
	
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm);
	while(1){
		int i = 0;
		while(sendData[i] != '\0') {
			while(!USART_IsTXDataRegisterEmpty(&USART));
			USART_PutChar(&USART, sendData[i]);
			i++;
		}
		if((PORTA.IN & XBEEDIO0)){
			PORTC.OUTSET = (PIN5_bm | PIN6_bm | PIN7_bm);
		}else if((!(PORTA.IN & XBEEDIO0))){
			PORTC.OUTCLR = (PIN5_bm | PIN6_bm | PIN7_bm);
		}
	}
}