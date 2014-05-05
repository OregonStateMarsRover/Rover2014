/*
 * DriveControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
 *  Author: corwin
 */ 

#define F_CPU 32000000UL

#include <util/delay.h>
<<<<<<< HEAD
#include "Sabertooth.h"
#include "Misc.h"

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};

=======
#include <avr/io.h>
#include "usart_driver.h"
#include "avr_compiler.h"
>>>>>>> 938f67737ee788cf3e5ee03b44ec65d67da05f05

/*! Define that selects the Usart used in example. */
#define XBEEDIO0 PIN5_bm //PORTA
#define SabertoothTx PIN3_bm //PORTD
#define StatusLight PIN3_bm //PORTE

<<<<<<< HEAD
#define RECEIVE_PACKET_SIZE  6
#define	SEND_PACKET_SIZE  4
=======
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
>>>>>>> 938f67737ee788cf3e5ee03b44ec65d67da05f05

uint8_t sendArray[SEND_PACKET_SIZE] = {0x55, 0xaa, 0xf0};
uint8_t receiveArray[RECEIVE_PACKET_SIZE];

USART_data_t USART_PC_Data;

bool IsRoving = false;

int main(void)
{
	SetXMEGA32MhzCalibrated();									//Set XMega to user 32Mhz internal oscillator with 32Khz crystal calibration
	
	///////Setup Inputs and Outputs///////
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm | PIN3_bm);		//Sets outputs on port C
	PORTC.DIRCLR = PIN2_bm;										//Sets inputs on PORT C
	PORTA.DIRCLR = XBEEDIO0;									//Sets inputs on PORTA
	
	
	///////Initialize Serial Communcations///////
	SetupPCComms();												//Initializes PC Communications at 9600 baud	
	Sabertooth DriveSaber(&USARTD0, &PORTD);					//Initializes Sabertooth Communications at 9600 Baud
	
	sei();														//Enables global interrupts so the interrupt serial can work
	
	////Semi-global vars//////
	unsigned char BufferIdx = 0;
	const char XMegaID[] = "ID: MainDrive\r\n";
	enum MicroState{
		WaitForHost,
		Driving	
	}XMegaState = WaitForHost;

	while(1){
		
		switch(XMegaState){
			case WaitForHost:
				for(int i = 0 ; XMegaID[i] != '\0'; i++){
					while(!USART_IsTXDataRegisterEmpty(&USARTC0));
					USART_PutChar(&USARTC0, XMegaID[i]);
				}
				_delay_ms(500);
				if(USART_RXBufferData_Available(&USART_PC_Data)){
					if(USART_RXBuffer_GetByte(&USART_PC_Data) == 'D'){
						XMegaState = Driving;
						USART_PutChar(&USARTC0, 'r');
					}
				}
				break;	
				
			case Driving:
				if(USART_RXBufferData_Available(&USART_PC_Data)){
					receiveArray[BufferIdx] = USART_RXBuffer_GetByte(&USART_PC_Data);
					BufferIdx++;
				}
			
				if(BufferIdx == RECEIVE_PACKET_SIZE){
					if(IsRoving){
						if(receiveArray[4] == PCComsChecksum(receiveArray[1], receiveArray[2], receiveArray[3])){
							DriveSaber.ParsePacket(receiveArray[2], receiveArray[3]);
						}else{
							DriveSaber.StopAll();
						}
					}
					BufferIdx = 0;
					SendDriveControlStatus(&USARTC0, true);
				}
			
				if(!IsRoving){
					DriveSaber.StopAll();
				}
			
				if((PORTA.IN & XBEEDIO0)){
					PORTC.OUTSET = (PIN5_bm | PIN6_bm | PIN7_bm);
					IsRoving = true;
				}else if((!(PORTA.IN & XBEEDIO0))){
					PORTC.OUTCLR = (PIN5_bm | PIN6_bm | PIN7_bm);
					IsRoving = false;
				}
				break;
				
		};	
	}
}

ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&USART_PC_Data);
}


ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_PC_Data);
}