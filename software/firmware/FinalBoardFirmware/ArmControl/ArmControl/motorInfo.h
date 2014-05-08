/* 
* motorInfo.h
*
* Created: 5/7/2014 3:17:28 AM
* Author: Nick
*/


#ifndef __MOTORINFO_H__
#define __MOTORINFO_H__


class motorInfo
{
//variables
public:
	int acceptableCountMax;
	int acceptableCount;
	double acceptableError;
	double slowRange;
	int speed;
	int enabled;
	
	double currentPos;  //Current pos in inches
	volatile double desiredPos;  //Desired pos in inches
	
protected:
private:

//functions
public:
	motorInfo();
	~motorInfo();
	void enable();
	void disable();
	void setDesired(float desired);
protected:
private:
	motorInfo( const motorInfo &c );
	motorInfo& operator=( const motorInfo &c );

}; //motorInfo

#endif //__MOTORINFO_H__
