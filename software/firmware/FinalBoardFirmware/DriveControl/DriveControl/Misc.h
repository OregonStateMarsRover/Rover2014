/*
 * Misc.h
 *
 * Created: 4/30/2014 3:10:05 PM
 *  Author: Corwin
 */ 


#ifndef MISC_H_
#define MISC_H_
#include <avr/io.h>

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

extern USART_data_t USART_PC_Data;

void SetXMEGA32MhzCalibrated();
void SetupPCComms();
unsigned char PCComsChecksum(unsigned char command, unsigned char left, unsigned char right);
void SendDriveControlStatus(USART_t *PCComs, bool IsRoving, bool Checksum);
void FlushSerialBuffer(USART_data_t *UsartBuffer);

#endif /* MISC_H_ */