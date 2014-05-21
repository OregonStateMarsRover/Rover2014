/*
 * DriveControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
 *  Author: corwin
 PE3 - Status LED
 */ 

#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "Sabertooth.h"
#include "Misc.h"

extern "C"{
	#include "usart_driver.h"
	#include "avr_compiler.h"
};
#define TIMEOUTMAX 5				//Time in seconds before state change back to init

/*! Define that selects the Usart used in example. */
#define XBEEDIO0 PIN5_bm //PORTA
#define SabertoothTx PIN3_bm //PORTD
#define StatusLight PIN3_bm //PORTE

#define RECEIVE_PACKET_SIZE  6
#define	SEND_PACKET_SIZE  4

uint8_t sendArray[SEND_PACKET_SIZE] = {0x55, 0xaa, 0xf0};
uint8_t receiveArray[RECEIVE_PACKET_SIZE];

USART_data_t USART_PC_Data;
 
bool IsRoving = false;

volatile long unsigned int TimeSinceInit = 0;
long unsigned int TimePrevious = 0;

int main(void)
{
	SetXMEGA32MhzCalibrated();									//Set XMega to user 32Mhz internal oscillator with 32Khz crystal calibration
	
	///////Setup Inputs and Outputs///////
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm | PIN3_bm);		//Sets outputs on port C
	PORTC.DIRCLR = PIN2_bm;										//Sets inputs on PORT C
	PORTA.DIRCLR = XBEEDIO0;
	PORTE.DIRSET = PIN3_bm;									//Sets inputs on PORTA
	
	
	///////Initialize Serial Communcations///////
	SetupPCComms();												//Initializes PC Communications at 9600 baud0
	_delay_ms(500);												//Delay to make sabertooth initialize
	Sabertooth DriveSaber(&USARTD0, &PORTD);					//Initializes Sabertooth Communications at 9600 Baud
	
	
	//////////////////Timers///////////////
	TCC0.CTRLA = TC_CLKSEL_DIV1024_gc; //31250 counts per second with 32Mhz Processor
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCC0.PER = 15625;
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	
	TCD0.CTRLA = TC_CLKSEL_DIV1024_gc; //31250 counts per second with 32Mhz Processor
	TCD0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCD0.PER = 31250;
	TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	///////////////////Timers//////////////
	
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
					if(USART_RXBuffer_GetByte(&USART_PC_Data) == 'r'){
						XMegaState = Driving;
						USART_PutChar(&USARTC0, 'r');
						BufferIdx = 0;
					}
				}
				TimePrevious = TimeSinceInit;
				break;	
				
			case Driving:
				if(USART_RXBufferData_Available(&USART_PC_Data)){
					receiveArray[BufferIdx] = USART_RXBuffer_GetByte(&USART_PC_Data);
					BufferIdx++;
				}
			
				if(BufferIdx == RECEIVE_PACKET_SIZE){
					FlushSerialBuffer(&USART_PC_Data);
					
					if(IsRoving){
						if(receiveArray[4] == PCComsChecksum(receiveArray[1], receiveArray[2], receiveArray[3])){
							DriveSaber.ParsePacket(receiveArray[2], receiveArray[3]);
							STATUS1_SET();
						}
						else{STATUS1_CLR();}
					}else if(!IsRoving){
						_delay_ms(10);
					}
					
					BufferIdx = 0;
					SendDriveControlStatus(&USARTC0, IsRoving, false);
					TimePrevious = TimeSinceInit;
				}					
					
				
				if((TimeSinceInit - TimePrevious) > TIMEOUTMAX){
					DriveSaber.StopAll();
					XMegaState = WaitForHost;
					TimePrevious = TimeSinceInit;
				}
				break;
				
		};	
	
		if(!IsRoving){
			DriveSaber.StopAll();
		}
	
		if((PORTA.IN & XBEEDIO0)){
			ERROR_CLR();
			IsRoving = true;
		}else if((!(PORTA.IN & XBEEDIO0))){
			ERROR_SET();
			IsRoving = false;
		}
	
	}
}


ISR(USARTC0_RXC_vect){
	USART_RXComplete(&USART_PC_Data);
}


ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&USART_PC_Data);
} 

ISR(TCC0_OVF_vect){
	if(IsRoving){
		JUDGELED_TOGGLE();	
	}else if(!IsRoving){
		JUDGELED_SET();
	}
	
}

ISR(TCD0_OVF_vect){
	STATUS2_TOGGLE();
	TimeSinceInit++;
}

