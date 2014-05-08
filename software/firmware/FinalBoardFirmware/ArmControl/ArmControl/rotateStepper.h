/* 
* rotateStepper.h
*
* Created: 5/7/2014 7:12:28 AM
* Author: Nick
*/


#ifndef __ROTATESTEPPER_H__
#define __ROTATESTEPPER_H__


class rotateStepper
{
//variables
public:
	double multiplier;
	double calSpan;
	
	int currentAngle;
	volatile int desiredPos;
protected:
private:

//functions
public:
	rotateStepper();
	~rotateStepper();
	
	void rotateBase(int desiredAngle);
	void calibrateBase();
	
	void moveBase(int degreesToMove); //Helper function
protected:
private:
	
	
	rotateStepper( const rotateStepper &c );
	rotateStepper& operator=( const rotateStepper &c );

}; //rotateStepper

#endif //__ROTATESTEPPER_H__
