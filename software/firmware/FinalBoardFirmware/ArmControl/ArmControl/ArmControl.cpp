/*
 * ArmControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
 *  Author: Nick
 */ 

#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "usart_driver.h"
#include "avr_compiler.h"
#include "XMegaMacros.h"

/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! Success variable, used to test driver. */
bool success;


int main(void)
{
	
	CCP = CCP_IOREG_gc;              // disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
	CCP = CCP_IOREG_gc;              // disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
	
	/* Variable used to send and receive data. */
	uint8_t sendData[] = "This is a string\r\n";
	//uint8_t receivedData;

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
	USART_Baudrate_Set(&USART, 207 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);


	/* Assume that everything is OK. */
	success = true;
	/* Send data from 255 down to 0*/
	
	//Setup Status and Error LEDS
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm);
	
	//Setup Outputs
	PORTD.DIRSET = (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm);
	PORTA.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm);  //First set of M settings
	PORTB.DIRSET = (PIN0_bm | PIN1_bm | PIN2_bm);  //Second set of M settings

	//Setup Inputs
	PORTA.DIRCLR = (PIN2_bm);

	//SETUP "UPPER" DRIVER
	//Set the enable pin low (disable high)
	MD1_DISABLE();
	
	//Setup Microstepping
	MD1_M0_CLR();
	MD1_M1_CLR();
	MD1_M2_CLR();
	
	MD1_DIR_CLR();
	MD1_STEP_CLR();
	
	//Motor Driver 2 setup
	MD2_ENABLE();
	
	//Setup Microstepping
	MD2_M0_CLR();
	MD2_M1_CLR();
	MD2_M2_CLR();
	
	MD2_DIR_CLR();
	MD2_STEP_CLR();
	
	
	int swap = 0;
	
	while(1) {
		int i = 0;
		while(sendData[i] != '\0') {
		    /* Send one char. */
			do{
				/* Wait until it is possible to put data into TX data register.
				 * NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK. */
			}while(!USART_IsTXDataRegisterEmpty(&USART));
			USART_PutChar(&USART, sendData[i]);
			i++;
			//uint16_t timeout = 1000;
			/* Receive one char. */
			//do{
			//	/* Wait until data received or a timeout.*/
			//	timeout--;
			//}while(!USART_IsRXComplete(&USART) && timeout!=0);
			//receivedData = USART_GetChar(&USART);
			
			//if(receivedData != 0){
			//USART_PutChar(&USART, receivedData);
			//receivedData = 0;
			//}
			/* Check the received data. */
		}

		/* Disable both RX and TX. */


		//PORTC.OUTSET = (PIN5_bm | PIN6_bm | PIN7_bm);
		//PORTD.OUTSET = PIN5_bm;
		MD2_STEP_SET();
		_delay_us(60);
		MD2_STEP_CLR();
		//PORTD.OUTCLR = PIN5_bm;
		//PORTC.OUTCLR = (PIN5_bm | PIN6_bm | PIN7_bm);
		_delay_us(60);
		
		++swap;
		
		if(swap > 250){
			MD2_DIR_SET();
			STATUS1_CLR();
			STATUS2_SET();
		}
		else {
			MD2_DIR_CLR();
			STATUS1_SET();
			STATUS2_CLR();
		}
		if(swap > 500){
			swap = 0;
		}
		
		if((PORTA.IN & (1 << PIN0_bp)) == 0){
			ERROR_SET();
		}
		else {
			ERROR_CLR();
		}
	}
}