/*
 * CPPFile1.cpp
 *
 * Created: 5/27/2014 10:19:21 PM
 *  Author: Corwin
 */ 
#include "Misc.h"
#include <avr/io.h>
#include "XMegaMacros.h"

void DockArm(unsigned char dockState){
	if(dockState == DOCK_ARM){
		ERROR_SET();
	}else if(dockState == UNDOCK_ARM){
		
	}
}