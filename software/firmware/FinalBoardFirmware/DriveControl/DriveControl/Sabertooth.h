/* 
* Sabertooth.h
*
* Created: 4/30/2014 1:08:16 PM
* Author: Corwin
*/


#ifndef __SABERTOOTH_H__
#define __SABERTOOTH_H__

#define F_CPU 32000000UL
#define SABERTOOTHADDRESS 128
#define AUTOBAUD_BYTE 170

#define LEFT_FORWARD 0
#define LEFT_BACK 1
#define RIGHT_FORWARD 4
#define RIGHT_BACK 5


#include <avr/io.h>
#include <util/delay.h>

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

class Sabertooth
{
//variables
public:
protected:
private:

//functions
public:
	Sabertooth(USART_t *USART_SaberUsart, PORT_t * SaberPORT);
	void DriveTest();
	void StopAll();
	unsigned char SaberChecksum(unsigned char command, unsigned char value);
	void ParsePacket(unsigned char left, unsigned char right);
	void ResetSaber();
protected:
private:
	void SendDriveCmd(char command, char value);
	Sabertooth( const Sabertooth &c );
	Sabertooth& operator=( const Sabertooth &c );
	USART_t *Sabertooth_USART;
	PORT_t *Sabertooth_PORT;

}; //Sabertooth

#endif //__SABERTOOTH_H__
