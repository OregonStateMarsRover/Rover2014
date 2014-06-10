/*
 * Misc.h
 *
 * Created: 5/27/2014 10:19:12 PM
 *  Author: Corwin
 */ 

#ifndef MISC_H_
#define MISC_H_

#define DOCK_ARM 1
#define UNDOCK_ARM 0

#include <avr/io.h>

void DockArm(unsigned char dockState);


#endif /* MISC_H_ */