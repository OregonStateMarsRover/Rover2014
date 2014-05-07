/*
 * ArmControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
 *  Author: NICK!
 * PA0 is Step2POT
 * PA1 is Step1POT
 */ 

#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

extern "C" {
	#include "avr_compiler.h"
	#include "usart_driver.h"
};

#include "Sabertooth.h"
#include "XMegaMacros.h"

int swap = 0;
USART_data_t USART_PC_Data;

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
	PORTC.DIRSET = PIN3_bm;																			//Sets TX Pin as output
	PORTC.DIRCLR = PIN2_bm;																			//Sets RX pin as input
	
	USART_InterruptDriver_Initialize(&USART_PC_Data, &USARTC0, USART_DREINTLVL_LO_gc);				//Initialize USARTC0 as interrupt driven serial and clear it's buffers
	USART_Format_Set(USART_PC_Data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);	//Set the data format of 8 bits, no parity, 1 stop bit
	USART_RxdInterruptLevel_Set(USART_PC_Data.usart, USART_RXCINTLVL_LO_gc);						//Enable the receive interrupt
	USART_Baudrate_Set(&USARTC0, 207 , 0);															//Set baudrate to 9600 with 32Mhz system clock
	USART_Rx_Enable(USART_PC_Data.usart);															//Enable receiving over serial
	USART_Tx_Enable(USART_PC_Data.usart);															//Enable transmitting over serial
	PMIC.CTRL |= PMIC_LOLVLEX_bm;																	//Enable PMIC interrupt level low (No idea what this does, but is necessary)
}

void DemStuffYouBeenDoingBefore(){
	
	MD2_STEP_SET();
	_delay_us(60);
	MD2_STEP_CLR();

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

void DemInitThingsYouBeenDoing(){
	SetXMEGA32MhzCalibrated();
	SetupPCComms();
	
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
}

void SendStringPC(char *stufftosend){
	for(int i = 0 ; stufftosend[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTC0));
		USART_PutChar(&USARTC0, stufftosend[i]);	
	}
}

uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}

uint16_t ReadADC(uint8_t Channel, uint8_t ADCMode) // Mode = 1 for single ended, 0 for internal
{
	if ((ADCA.CTRLA & ADC_ENABLE_bm) == 0)
	{
		ADCA.CTRLA = ADC_ENABLE_bm ; // Enable the ADC
		ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc; // Signed Mode
		ADCA.REFCTRL = ADC_REFSEL_VCC_gc; // Internal 1v ref
		ADCA.EVCTRL = 0 ; // no events
		ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc ;
		ADCA.CALL = ReadSignatureByte(0x20) ; //ADC Calibration Byte 0
		ADCA.CALH = ReadSignatureByte(0x21) ; //ADC Calibration Byte 1
		_delay_us(400); // Wait at least 25 clocks
	}
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADCMode ; // Gain = 1, Single Ended
	ADCA.CH0.MUXCTRL = (Channel<<3);
	ADCA.CH0.INTCTRL = 0 ; // No interrupt
	for(uint8_t Waste = 0; Waste<2; Waste++)
	{
		ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
		while (ADCA.INTFLAGS==0) ; // Wait for complete
		ADCA.INTFLAGS = ADCA.INTFLAGS ;
	}
	return ADCA.CH0RES ;
}

int main(void)
{
		DemInitThingsYouBeenDoing();							//All init moved to nicer spot
		_delay_ms(2500);
		char SendBuffer[200];

		
	while(1) {
		DemStuffYouBeenDoingBefore();							//Your stepper code
		int resultPA0 = ReadADC(0,1);
		int resultPA1 = ReadADC(1,1);
		
		sprintf(SendBuffer, "ADC Value for PA0 is: %d.\r\nADC Value for PA1 is: %d.\r\n\r\n", resultPA0, resultPA1);	//Store the result in a string to be sent
		SendStringPC(SendBuffer);								//Send Dem Strings
		_delay_ms(100);											//Wait so things don't die
	}
}