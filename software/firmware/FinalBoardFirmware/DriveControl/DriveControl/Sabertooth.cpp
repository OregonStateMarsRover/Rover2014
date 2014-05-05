<<<<<<< HEAD
/* 
* Sabertooth.cpp
*
* Created: 4/30/2014 1:08:16 PM
* Author: Corwin
*/


#include "Sabertooth.h"
#include <avr/io.h>
#include <util/delay.h>

Sabertooth::Sabertooth(USART_t *USART_SaberUsart, PORT_t * SaberPORT)
{
	Sabertooth_USART = USART_SaberUsart;				//Sets the private variable to the USART being used
	Sabertooth_PORT = SaberPORT;						//Sets the private variable for the PORT the USART is on
	
	Sabertooth_PORT->DIRSET = PIN3_bm;					//Sets the TX pin for the USART to an output
	USART_Format_Set(Sabertooth_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Sets the Sabertooth USART to run in 8 bit data, no parity, and 1 stop bit, 
	USART_Baudrate_Set(Sabertooth_USART, 207 , 0);		//Sets the Sabertooth baud rate to 9600 when running at 32Mhz system clock
	USART_Tx_Enable(Sabertooth_USART);					//Enable the USART transmit capabilities
	_delay_ms(100);										//Delay to let things settle
	
	USART_PutChar(Sabertooth_USART, AUTOBAUD_BYTE);		//Send the autobaud byte to get the sabertooth communicating
	SendDriveCmd(14, 20);								//Sets the communication watchdog on the sabertooth to (x*100ms) It's currently set to two seconds.
	StopAll();											//Everything is now initialized, stop all motor movement to account for random noise or failed startups
}

void Sabertooth::DriveTest(){
	
	int i;
	for(i = 0; i < 128 ; i++){
		SendDriveCmd(LEFT_FORWARD, i);
		SendDriveCmd(RIGHT_FORWARD, i);
		_delay_ms(30);
	}
	for( ; i > 0 ; i--){
		SendDriveCmd(LEFT_FORWARD, i);
		SendDriveCmd(RIGHT_FORWARD, i);
		_delay_ms(30);
	}
	
		for(i = 0; i < 128 ; i++){
			SendDriveCmd(LEFT_BACK, i);
			SendDriveCmd(RIGHT_BACK, i);
			_delay_ms(30);
		}
		for( ; i > 0 ; i--){
			SendDriveCmd(LEFT_BACK, i);
			SendDriveCmd(RIGHT_BACK, i);
			_delay_ms(30);
		}
		
}

void Sabertooth::ParsePacket(unsigned char left, unsigned char right){
	unsigned char command_left = LEFT_FORWARD;
	unsigned char value_left = 0;
	unsigned char command_right = RIGHT_FORWARD;
	unsigned char value_right = 0;
	
	if(left == 127){
		command_left = LEFT_FORWARD;
		value_left = 0;
	}else if(left < 127){
		command_left = LEFT_BACK;
		value_left = (127-left);
	}else if(left > 127){
		command_left = LEFT_FORWARD;
		value_left = (left-127);
	}
	
	if(right == 127){
		command_right = RIGHT_FORWARD;
		value_right = 0;
		}else if(right < 127){
		command_right = RIGHT_BACK;
		value_right = (127-right);
		}else if(right > 127){
		command_right = RIGHT_FORWARD;
		value_right = (right-127);
	}
	
	SendDriveCmd(command_left, value_left);
	SendDriveCmd(command_right, value_right);
}

void Sabertooth::StopAll(){
	SendDriveCmd(LEFT_FORWARD, 0);
	SendDriveCmd(RIGHT_FORWARD, 0);
}

unsigned char Sabertooth::SaberChecksum(unsigned char command, unsigned char value){
	return ((SABERTOOTHADDRESS+command+value) & 127);
}

void Sabertooth::SendDriveCmd(char command, char value){
	while(!USART_IsTXDataRegisterEmpty(Sabertooth_USART));						//Necessary to make sure we don't overwrite data in the buffer
	USART_PutChar(Sabertooth_USART, SABERTOOTHADDRESS);							//Sends the address to the sabertooth
	while(!USART_IsTXDataRegisterEmpty(Sabertooth_USART));
	USART_PutChar(Sabertooth_USART, command);									//Sends the command to the sabertooth
	while(!USART_IsTXDataRegisterEmpty(Sabertooth_USART));
	USART_PutChar(Sabertooth_USART, value);										//Sends the value or speed to the sabertooth
	while(!USART_IsTXDataRegisterEmpty(Sabertooth_USART));
	USART_PutChar(Sabertooth_USART, SaberChecksum(command, value));				//Send the checksum of all these values to the sabertooth
}
=======
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
>>>>>>> 938f67737ee788cf3e5ee03b44ec65d67da05f05
