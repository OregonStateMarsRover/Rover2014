/*
 * ArmControl.cpp
 *
 * Created: 4/22/2014 8:54:20 PM
<<<<<<< HEAD
 *  Author: NICK!
 * PA0 is Step2POT
 * PA1 is Step1POT
=======

PB3 = Limit 1 = Grip Limit
PA3 = Limit 2 = Grip Close
PA2 = Limit 3 = Rotation Calibration


 *  Author: Nick
>>>>>>> 034fb4b9ec9e5b3ad4812938ac8725fedd57e6b0
 */ 

#define F_CPU 32000000UL
#define MAXTIMEOUT 20

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

extern "C" {
	#include "avr_compiler.h"
	#include "usart_driver.h"
};

#include "Sabertooth.h"
#include "XMegaMacros.h"
#include "adc.h"  //Include the ADC functions
#include "motorInfo.h"  //Include the motor information
#include "stepperInfo.h"
#include "rotateStepper.h"
#include "Misc.h"

int swap = 0;
USART_data_t USART_PC_Data;


motorInfo lowerAct;
motorInfo upperAct;
stepperInfo gripStepper;
rotateStepper baseStepper;

#define LOWER 0
#define UPPER 1

#define GRIP 0
#define RELEASE 1

#define GRIP_BM_SERIAL (1 << 1)

volatile bool canAcceptPackets = true;
volatile bool IsPacketToParse = false;

volatile unsigned char ARM_Dock_State = 0;
unsigned char ARM_Dock_State_Prev = 0;

volatile bool ShouldRECAL = 0;

volatile unsigned char bufferIndex = 0;

volatile long unsigned int TimeSinceInit = 0;
long unsigned int TimePrevious = 0;

volatile int v = 0;

#define PACKETSIZE 10
volatile char recieveBuffer[PACKETSIZE];
volatile char SendBuffer[100];

enum { HEADER, COMMAND, BASEROTVAL1, BASEROTVAL2, ACT1VAL1, ACT1VAL2, ACT2VAL1, ACT2VAL2, CHECKSUM, TAIL};
	
enum XMegaStates{
	WaitForHost,
	ARMControl
} CurrentState = WaitForHost;

void SetupResetTimer(){
	TCD0.CTRLA = TC_CLKSEL_DIV1024_gc; //31250 counts per second with 32Mhz Processor
	TCD0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCD0.PER = 31250;
	TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;
}
	
void FlushSerialBuffer(USART_data_t *UsartBuffer){
	while(USART_RXBufferData_Available(UsartBuffer)){
		USART_RXBuffer_GetByte(UsartBuffer);
	}
}

ISR(USARTC0_RXC_vect){
	USART_RXComplete(&USART_PC_Data);
	
	if(USART_RXBufferData_Available(&USART_PC_Data)){
		recieveBuffer[bufferIndex] = USART_RXBuffer_GetByte(&USART_PC_Data);
		bufferIndex++;
	}
	
	if((bufferIndex == PACKETSIZE)){
		FlushSerialBuffer(&USART_PC_Data);
		if(recieveBuffer[8] == (recieveBuffer[1] ^ recieveBuffer[2] ^ recieveBuffer[3] ^ recieveBuffer[4] ^ recieveBuffer[5] ^ recieveBuffer[6] ^ recieveBuffer[7])){
			ShouldRECAL = recieveBuffer[1] & 0b00001000;
  			ARM_Dock_State = recieveBuffer[1] & 0b00000100;
			gripStepper.desiredGripState = !(recieveBuffer[1] & GRIP_BM_SERIAL); //0b00000010	
			baseStepper.desiredPos = (recieveBuffer[3]+recieveBuffer[2]);
			lowerAct.setDesired((double(recieveBuffer[5]+recieveBuffer[4]) / double(100)));
			upperAct.setDesired((double(recieveBuffer[7]+recieveBuffer[6]) / double(100)));
			IsPacketToParse = true;
		}else{
			while(!USART_IsTXDataRegisterEmpty(&USARTC0));
			USART_PutChar(&USARTC0, 255);
			while(!USART_IsTXDataRegisterEmpty(&USARTC0));
			USART_PutChar(&USARTC0,0);  //Checksum failed
			while(!USART_IsTXDataRegisterEmpty(&USARTC0));
			USART_PutChar(&USARTC0,255);

			bufferIndex = 0;	
		}
		
	}

}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&USART_PC_Data);
}


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


//Motor 1 is Gripper
//Motor 2 is Base Stepper
void DemInitThingsYouBeenDoing(){
	SetXMEGA32MhzCalibrated();
	SetupPCComms();
	SetupResetTimer();
	
	//Setup Status and Error LEDS
	PORTC.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm);
	
	//Setup Outputs
	PORTD.DIRSET = (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm);
	PORTA.DIRSET = (PIN5_bm | PIN6_bm | PIN7_bm);  //First set of M settings
	PORTB.DIRSET = (PIN0_bm | PIN1_bm | PIN2_bm);  //Second set of M settings

	//Setup Inputs
	PORTA.DIRCLR = (PIN2_bm); //Rotation Calibration
	PORTA.DIRCLR = (PIN3_bm); //Grip Close
	PORTB.DIRCLR = (PIN3_bm); //Grip Limit
	PORTA.DIRCLR = (PIN4_bm); //'IsRoving' check
		
	PORTA.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN3CTRL = PORT_OPC_PULLUP_gc;	

	//GRIP STEPPER is MD1

	//SETUP "UPPER" DRIVER
	MD1_ENABLE();
	
	//Setup Microstepping
	MD1_M0_CLR();
	MD1_M1_CLR();
	MD1_M2_SET();
	
	MD1_DIR_CLR();
	MD1_STEP_CLR();
	
	
	//BASE STEPPER is MD2
	
	//Motor Driver 2 setup
	MD2_ENABLE();
	
	//Setup Microstepping
	MD2_M0_SET();  //Small amount of micro stepping is sufficient 
	MD2_M1_SET();
	MD2_M2_CLR();
	
	MD2_DIR_CLR();
	MD2_STEP_CLR();
	
	sei();
}

void SendStringPC(char *stufftosend){
	for(int i = 0 ; stufftosend[i] != '\0' ; i++){
		while(!USART_IsTXDataRegisterEmpty(&USARTC0));
		USART_PutChar(&USARTC0, stufftosend[i]);	
	}
}


//DOCUMENTATION NEEDED :D
double abs(double input){
	if(input > 0)
		return input;
	else
		return input * -1;
}

//PA1 is lower act

//If a 0 is passed in, then the lower act is read
//0 = LOWER ACT
//1 = UPPER ACT
int smoothADC(int act){
	const int smoothFactor = 7;
	int count = 0;
	for(int i = 0; i < smoothFactor; ++i){
		if(act == LOWER){ 
			count += ReadADC(1,1);	
		}
		else if(act == UPPER) {
			count += ReadADC(0,1);
		}
		_delay_ms(1);
	}
	return count/smoothFactor;
}

//lowerAct   upperAct
void checkActPosition(){
	
	if (abs(lowerAct.currentPos - lowerAct.desiredPos) < lowerAct.acceptableError){
		++lowerAct.acceptableCount;
	}
	else{
		lowerAct.acceptableCount = 0;
	}
	if (abs(upperAct.currentPos - upperAct.desiredPos) < upperAct.acceptableError){
		++upperAct.acceptableCount;
	}
	else{
		upperAct.acceptableCount = 0;
	}
	
	if(upperAct.acceptableCount >= upperAct.acceptableCountMax){
		upperAct.disable();
	}
	if(lowerAct.acceptableCount >= upperAct.acceptableCountMax){
		lowerAct.disable();
	}
	
	
	lowerAct.currentPos = smoothADC(LOWER)/58.13 -.41;
	upperAct.currentPos = smoothADC(UPPER)/58.13 -.41;
}

int getMotorSpeed(int act){
		
	if(act == LOWER){
		if(abs(lowerAct.currentPos - lowerAct.desiredPos) < lowerAct.slowRange/2)
			return lowerAct.speed / 3;
		else if(abs(lowerAct.currentPos - lowerAct.desiredPos) < lowerAct.slowRange)
			return lowerAct.speed / 2;
		else
			return lowerAct.speed;
	}
	else if (act == UPPER){
		if(abs(upperAct.currentPos - upperAct.desiredPos) < upperAct.slowRange/2)
			return upperAct.speed / 3;
		else if(abs(upperAct.currentPos - upperAct.desiredPos) < upperAct.slowRange)
			return upperAct.speed / 2;
		else
			return upperAct.speed;
	}
	
	
	////////
	return 0;
}

/*Returns a 1 or a -1, depending on whether the actuator needs to retract 
  or extend
*/
int getMotorDir(int act){
	if(act == LOWER){
		if(!lowerAct.enabled)
			return 0;
		
		if(lowerAct.currentPos > lowerAct.desiredPos)
			return -1;
		else
			return 1;
	}
	else if(act == UPPER){
		if(!upperAct.enabled)
			return 0;
		
		if(upperAct.currentPos > upperAct.desiredPos)
			return -1;
		else
			return 1;
	}
	/////////
	return 0;
}

int main(void)
{
	DemInitThingsYouBeenDoing();							//All init moved to nicer spot
	_delay_ms(1000);

	
	Sabertooth DriveSaber(&USARTD0, &PORTD);
	
	
	upperAct.desiredPos = 3.0;
	lowerAct.desiredPos = 3.5;
	
	//Wait until rover is unpaused
	while(!CHECK_ISROVING());
	
	lowerAct.enable();
	upperAct.enable();
	
	
	/////////////Initial Calibration and Default Positions//////////////////////
	while((lowerAct.enabled || upperAct.enabled)){
		checkActPosition();
		DriveSaber.ParsePacket(127+getMotorSpeed(LOWER)*getMotorDir(LOWER), 127+getMotorSpeed(UPPER)*getMotorDir(UPPER));
		while(!CHECK_ISROVING()){
			DriveSaber.ParsePacket(127,127);
		}	
	}
	baseStepper.calibrateBase();
	MD2_DIR_CLR();
	baseStepper.rotateBase(0);  //Note that this function takes an angle relative
	
	
	gripStepper.enable();							 //to the absolute 0 on the robot
	gripStepper.processCommand(RELEASE);
	/*
	_delay_ms(5000);
	gripStepper.enable();
	gripStepper.processCommand(GRIP);
	*/ 
	/////////////////   DEBUG (and not wasting power) purposes!
	//MD2_DISABLE();
	/////////////////

	/////////////Initial Calibration and Default Positions//////////////////////

//	sprintf(SendBuffer, "Multiplier: %d \r\n  \r\n", (int) baseStepper.multiplier);
//	SendStringPC(SendBuffer);								//Send Dem Strings
	while(1){
		if(CurrentState == WaitForHost){
			SendStringPC("ID: ArmControl\r\n");
			_delay_ms(500);
			if(recieveBuffer[0] == 'r'){
				CurrentState = ARMControl;
				while(!USART_IsTXDataRegisterEmpty(&USARTC0));
				USART_PutChar(&USARTC0, 'r');
				TimePrevious = TimeSinceInit;
			}
			bufferIndex = 0;
		}else if(CurrentState == ARMControl){
			if(IsPacketToParse){
				if(ShouldRECAL == true){

							
					upperAct.desiredPos = 3.0;
					lowerAct.desiredPos = 3.5;
						
					lowerAct.enable();
					upperAct.enable();
						
					/////////////Initial Calibration and Default Positions//////////////////////
					while((lowerAct.enabled || upperAct.enabled)){
						checkActPosition();
						DriveSaber.ParsePacket(127+getMotorSpeed(LOWER)*getMotorDir(LOWER), 127+getMotorSpeed(UPPER)*getMotorDir(UPPER));
						while(!CHECK_ISROVING()){
							DriveSaber.ParsePacket(127,127);
						}
					}

					baseStepper.calibrateBase();
					MD2_DIR_CLR();
					baseStepper.rotateBase(0);  //Note that this function takes an angle relative
						
					if(gripStepper.desiredGripState){
						gripStepper.enable();							 //to the absolute 0 on the robot
						gripStepper.processCommand(RELEASE);	
					}
					
					ShouldRECAL = false;

				}else{
					ERROR_SET();									//Show light when done with actuators
					lowerAct.enable();						//Re-enable lower actuator
					upperAct.enable();						//Re-enabled lower actuator

					baseStepper.rotateBase(baseStepper.desiredPos);	//Move base to position
					
					checkActPosition();								//Check once to avoid loop is possible
					while(lowerAct.enabled || upperAct.enabled){	//If a motor needs to move, do below
						checkActPosition();							//Check positions
						DriveSaber.ParsePacket(127+getMotorSpeed(LOWER)*getMotorDir(LOWER), 127+getMotorSpeed(LOWER)*getMotorDir(UPPER));	//Move to position
						while(!CHECK_ISROVING()){
							DriveSaber.ParsePacket(127,127);
						}  //e-stop check
					}												//Exit when done moving
					

					DriveSaber.ParsePacket(127,127);				//Stop actuators from moving any more
				
					if((gripStepper.desiredGripState == GRIP)){
						gripStepper.enable();
						gripStepper.processCommand(GRIP);
					}else if(gripStepper.desiredGripState == RELEASE){
						gripStepper.enable();
						gripStepper.processCommand(RELEASE);

					}
				}
				
				IsPacketToParse = false;
				ERROR_CLR();
				while(!USART_IsTXDataRegisterEmpty(&USARTC0));
				USART_PutChar(&USARTC0, 255);
				while(!USART_IsTXDataRegisterEmpty(&USARTC0));
				USART_PutChar(&USARTC0, 0b00000010 | CHECK_GRIP_CLOSE());
				while(!USART_IsTXDataRegisterEmpty(&USARTC0));
				USART_PutChar(&USARTC0,255);
				bufferIndex = 0;
				TimePrevious = TimeSinceInit;
			}
			
			//if((TimePrevious - TimeSinceInit) > MAXTIMEOUT){
			//	CurrentState = WaitForHost;
			//	bufferIndex = 0;
			//}
		}
	}

}

ISR(TCD0_OVF_vect){
	TimeSinceInit++;
}




