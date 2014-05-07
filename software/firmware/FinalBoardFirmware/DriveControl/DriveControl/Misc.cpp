/*
 * Misc.cpp
 *
 * Created: 4/30/2014 3:09:52 PM
 *  Author: Corwin
 */ 
#include "Misc.h"
#include <avr/io.h>

void SetXMEGA32MhzCalibrated(){
	CCP = CCP_IOREG_gc;						//Disable register security for oscillator update
	OSC.CTRL = OSC_RC32MEN_bm;				//Enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); //Wait for oscillator to be ready
	CCP = CCP_IOREG_gc;						//Disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;		//Switch to 32MHz clock


	CCP = CCP_IOREG_gc;						//Disable register security for oscillator update
	OSC.CTRL |= OSC_RC32KEN_bm;				//Enable 32Khz oscillator
	while(!(OSC.STATUS & OSC_RC32KRDY_bm)); //Wait for oscillator to be ready
	OSC.DFLLCTRL &= ~OSC_RC32MCREF_bm;		//Set up calibration source to be 32Khz crystal
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;		//Enable calibration of 32Mhz oscillator
}

void SetupPCComms(){
	USART_InterruptDriver_Initialize(&USART_PC_Data, &USARTC0, USART_DREINTLVL_LO_gc);				//Initialize USARTC0 as interrupt driven serial and clear it's buffers
	USART_Format_Set(USART_PC_Data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(USART_PC_Data.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTC0, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(USART_PC_Data.usart);															//Enable receiving over serial
	USART_Tx_Enable(USART_PC_Data.usart);															//Enable transmitting over serial
	PMIC.CTRL |= PMIC_LOLVLEX_bm;																	//Enable PMIC interrupt level low (No idea what this does, but is necessary)
}

unsigned char PCComsChecksum(unsigned char command, unsigned char left, unsigned char right){
	return (command ^ left ^ right);
	
}

void SendDriveControlStatus(USART_t *PCComs, bool IsRoving, bool Checksum){
	while(!USART_IsTXDataRegisterEmpty(PCComs));
	USART_PutChar(PCComs, 255);
	while(!USART_IsTXDataRegisterEmpty(PCComs));
	USART_PutChar(PCComs, ((IsRoving << 0) | (Checksum << 1)));
	while(!USART_IsTXDataRegisterEmpty(PCComs));
	USART_PutChar(PCComs, 255);
}

void FlushSerialBuffer(USART_data_t *UsartBuffer){
	while(USART_RXBufferData_Available(UsartBuffer)){
		USART_RXBuffer_GetByte(UsartBuffer);
	}
}