/* 
* stepperInfo.h
*
* Created: 5/7/2014 6:19:00 AM
* Author: Nick
*/

#include "XMegaMacros.h"

#ifndef __STEPPERINFO_H__
#define __STEPPERINFO_H__

#define GRIP 0
#define RELEASE 1

class stepperInfo
{
//variables
public:
	int enabled;
	int currentState;
	int init;
	volatile unsigned char desiredGripState;
protected:
private:

//functions
public:
	stepperInfo();
	~stepperInfo();
	void processCommand(int cmd);
	void enable();
protected:
private:
	stepperInfo( const stepperInfo &c );
	stepperInfo& operator=( const stepperInfo &c );

}; //stepperInfo

#endif //__STEPPERINFO_H__
