/*
 * DriveControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
 *  Author: corwin
 */ 

#define F_CPU 2000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "usart_driver.h"
#include "avr_compiler.h"

/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! Success variable, used to test driver. */
bool success;

int main(void)
{
	/* Variable used to send and receive data. */
	uint8_t sendData[] = "This is a string\r\n";
	uint8_t receivedData;

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTC.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR = PIN2_bm;

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);


	/* Assume that everything is OK. */
	success = true;
	/* Send data from 255 down to 0*/
	
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm);
	while(1)
		    {
		int i = 0;
		while(sendData[i] != '\0') {
	    /* Send one char. */
		do{
		/* Wait until it is possible to put data into TX data register.
		 * NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK. */
		}while(!USART_IsTXDataRegisterEmpty(&USART));
		USART_PutChar(&USART, sendData[i]);
		i++;
		uint16_t timeout = 1000;
		/* Receive one char. */
		do{
		/* Wait until data received or a timeout.*/
		timeout--;
		}while(!USART_IsRXComplete(&USART) && timeout!=0);
		receivedData = USART_GetChar(&USART);
		//if(receivedData != 0){
		//USART_PutChar(&USART, receivedData);
		//receivedData = 0;
		//}
		/* Check the received data. */
	}

	/* Disable both RX and TX. */


        PORTC.OUTSET = (PIN5_bm | PIN6_bm | PIN7_bm);
		_delay_ms(500);
		PORTC.OUTCLR = (PIN5_bm | PIN6_bm | PIN7_bm);   
		_delay_ms(500);
	}
}