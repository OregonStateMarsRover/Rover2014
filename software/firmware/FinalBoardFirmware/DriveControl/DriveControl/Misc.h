/*
 * Misc.h
 *
 * Created: 4/30/2014 3:10:05 PM
 *  Author: Corwin
 */ 


#ifndef MISC_H_
#define MISC_H_
#include <avr/io.h>

//Custom Defined Macros
#define STATUS1_SET(void) (PORTC.OUTSET = PIN6_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN6_bm)
#define STATUS1_TOGGLE(void) (PORTC.OUTTGL = PIN6_bm)

#define STATUS2_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS2_CLR(void) (PORTC.OUTCLR = PIN5_bm)
#define STATUS2_TOGGLE(void) (PORTC.OUTTGL = PIN5_bm)

#define ERROR_SET(void) (PORTC.OUTSET = PIN7_bm)
#define ERROR_CLR(void) (PORTC.OUTCLR = PIN7_bm)
#define ERROR_TOGGLE(void) (PORTC.OUTTGL = PIN7_bm)

#define JUDGELED_SET(void) (PORTE.OUTSET = PIN3_bm)
#define JUDGELED_CLR(void) (PORTE.OUTCLR = PIN3_bm)
#define JUDGELED_TOGGLE(void) (PORTE.OUTTGL = PIN3_bm)

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