/* 
* stepperInfo.cpp
*
* Created: 5/7/2014 6:19:00 AM
* Author: Nick
*/

#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "stepperInfo.h"

extern "C" {
	#include "avr_compiler.h"
	#include "usart_driver.h"
	
};

extern void SendStringPC(char *stufftosend);

// default constructor
stepperInfo::stepperInfo() {
	enabled = 0;
	currentState = -1;
	init = 1; //Init state (ignore first push)
} //stepperInfo

// default destructor
stepperInfo::~stepperInfo()
{
} //~stepperInfo


//Used for the gripping stepper
void stepperInfo::processCommand(int cmd){
	if(!enabled)
		return;  //TODO: MAKE BETTER
		
	if(cmd != GRIP && cmd != RELEASE)
		return;
		
	if(cmd == currentState)
		return;
		
	currentState = cmd;
	
	//char sendBuffer[200];
	//sprintf(sendBuffer, "Reached process command");
	//SendStringPC(sendBuffer);
	
	
	//GET DIRECTION
	//CLR IS OUT
	
	//SET is grip
	//CLR is release
	if(cmd == GRIP)
		MD1_DIR_SET();
	else if (cmd == RELEASE)
		MD1_DIR_CLR();
		
	if(!init){
		for(int i = 0; i < 10000; ++i){
			
			while(!CHECK_ISROVING());  //e-stop check

			MD1_STEP_SET();
			_delay_us(50);
			MD1_STEP_CLR();
			_delay_us(500);
		}
	}
	else {
		init = 0;
	}
	
	//MOVE UNTIL LIMIT OR GRIP
	if(cmd == GRIP){
		for(long i = 0; i < 185000; ++i){
		
			while(!CHECK_ISROVING());  //e-stop check

			MD1_STEP_SET();
			_delay_us(20);
			MD1_STEP_CLR();
			_delay_us(40);
		}		
	}else if(cmd == RELEASE){
		while(!CHECK_GRIP_LIMIT()){
			while(!CHECK_ISROVING());  //e-stop check
			MD1_STEP_SET();
			_delay_us(20);
			MD1_STEP_CLR();
			_delay_us(40);
		}	
	}	
	enabled = 0;
	
}


void stepperInfo::enable(){
	enabled = 1;
}