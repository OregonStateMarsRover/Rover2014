#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define F_CPU 16000000UL

#define NOINTERRUPT 255

#define INITIALIZATION 0
#define WAITFORHOST 1
#define DRIVING 2

#define DRIVE_FORWARD 1
#define DRIVE_BACKWARD 0
#define DRIVE_STOP 2

volatile unsigned char nextState;

#endif // MAIN_H_INCLUDED
