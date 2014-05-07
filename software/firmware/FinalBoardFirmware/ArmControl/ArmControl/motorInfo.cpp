/* 
* motorInfo.cpp
*
* Created: 5/7/2014 3:17:28 AM
* Author: Nick
*/


#include "motorInfo.h"

// default constructor
motorInfo::motorInfo() {
	acceptableCount = 0;
	acceptableCountMax = 5;
	acceptableError = .1;  //Needs calibration
	slowRange = .5;
	speed = 50;
	enabled = 0;
	
	//currentPos = 0;
} //motorInfo

// default destructor
motorInfo::~motorInfo() {
	
} //~motorInfo


void motorInfo::enable(){
	enabled = 1;
}

void motorInfo::disable(){
	enabled = 0;
}


void motorInfo::setDesired(float desired){
	desiredPos = desired; //Mutate desiredPos
}