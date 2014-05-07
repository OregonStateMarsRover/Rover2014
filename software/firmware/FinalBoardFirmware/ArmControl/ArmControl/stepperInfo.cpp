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
	
	
	//GET DIRECTION
	//CLR IS OUT
	
	//SET is grip
	//CLR is release
	if(cmd == GRIP)
		MD1_DIR_SET();
	else if (cmd == RELEASE)
		MD1_DIR_CLR();
		
	if(!init){
		for(int i = 0; i < 1500; ++i){
			MD1_STEP_SET();
			_delay_us(500);
			MD1_STEP_CLR();
			_delay_us(500);
			_delay_ms(1);
		}
	}
	else {
		init = 0;
	}
	
	//MOVE UNTIL LIMIT OR GRIP
	while(!CHECK_GRIP_LIMIT() && !CHECK_GRIP_CLOSE()){
		MD1_STEP_SET();
		_delay_us(500);
		MD1_STEP_CLR();
		_delay_us(500);
		
		_delay_us(500);
	}
	
	enabled = 0;
	
}


void stepperInfo::enable(){
	enabled = 1;
}