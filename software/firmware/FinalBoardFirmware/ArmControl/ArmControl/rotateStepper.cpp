/* 
* rotateStepper.cpp
*
* Created: 5/7/2014 7:12:28 AM
* Author: Nick
*/

#define F_CPU 32000000UL

#include "rotateStepper.h"
#include "XMegaMacros.h"


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "stepperInfo.h"


extern "C" {
	#include "avr_compiler.h"
	#include "usart_driver.h"
};

// default constructor
rotateStepper::rotateStepper() {
	calSpan = 255; //TODO: Set to actual value
	multiplier = -1; //Invalid (not set) state
	
	currentAngle = 0; //It will have its reference based off of the 2nd limit switch,
					  //but the interface function will minus the amount to make 0 forward
} //rotateStepper

// default destructor
rotateStepper::~rotateStepper()
{
} //~rotateStepper

void rotateStepper::rotateBase(int desiredAngle){
	//NEED INPUT CHEKCING
	
	int zeroedAngle = desiredAngle + 36.09;
	
	moveBase(zeroedAngle - currentAngle);
	currentAngle = currentAngle + (zeroedAngle - currentAngle);
	
}


void rotateStepper::calibrateBase(){
	bool calInProgress = true;
	bool calFirstPress = false;

	//bool calSecondPress = false;
	
	int calButtonState;
	
	int stepCount = 0;
	
	MD2_DIR_CLR(); //Set arm to turn counter-clockwise
	
	while (calInProgress){
		while(!CHECK_ISROVING());
		calButtonState = CHECK_CAL();

		if(calButtonState && !calFirstPress){
			
			calFirstPress = true;
			MD2_DIR_SET();  //Sets arm to clockwise
			_delay_ms(1200);  //For gracefulness
		}
		
		if(calFirstPress == true)
			++stepCount;
			
		if(calFirstPress && stepCount > 150 && CHECK_CAL()){
			calInProgress = false;
		}
		
		MD2_STEP_CLR();
		_delay_us(600);
		MD2_STEP_SET();
		_delay_us(600);	
		
	}	
	currentAngle = 0;
	multiplier = stepCount / calSpan;
	_delay_ms(1200);  //For gracefulness
}


//Multiplier is steps per degree

//Helper function
void rotateStepper::moveBase(int degreesToMove){

	if (degreesToMove > 0)
		MD2_DIR_CLR();  //Counter Clockwise
	else
		MD2_DIR_SET();  //Clockwise
	
	int stepsToMove = abs(degreesToMove) * multiplier;
	
	for(int i = 0; i < stepsToMove; ++i){
		while(!CHECK_ISROVING());  //e-stop check
		
		MD2_STEP_CLR();
		_delay_us(400);
		MD2_STEP_SET();
		_delay_us(500);
	}
	
}