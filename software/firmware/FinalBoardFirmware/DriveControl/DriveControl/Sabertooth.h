<<<<<<< HEAD
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
protected:
private:
	void SendDriveCmd(char command, char value);
	Sabertooth( const Sabertooth &c );
	Sabertooth& operator=( const Sabertooth &c );
	USART_t *Sabertooth_USART;
	PORT_t *Sabertooth_PORT;

}; //Sabertooth

#endif //__SABERTOOTH_H__
=======
/*
 * Sabertooth.h
 *
 * Created: 4/25/2014 3:58:54 PM
 *  Author: corwin
 */ 


#ifndef SABERTOOTH_H_
#define SABERTOOTH_H_





#endif /* SABERTOOTH_H_ */
>>>>>>> 938f67737ee788cf3e5ee03b44ec65d67da05f05
