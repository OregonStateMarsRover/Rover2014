#ifndef SABERTOOTH_H_INCLUDED
#define SABERTOOTH_H_INCLUDED

#define SABERTOOTHADDRESS 128
#define AUTOBAUD_BYTE 170

#define LEFT_FORWARD 0
#define LEFT_BACK 1
#define RIGHT_FORWARD 4
#define RIGHT_BACK 5


void Initialize_Sabertooth(void);
void Sabertooth_WriteCommand(unsigned char address, unsigned char command, unsigned char value);
void Sabertooth_SetMotors(unsigned char address, unsigned char LeftDir, unsigned char LeftSpeed, unsigned char RightDir, unsigned char RightSpeed);
void Sabertooth_LeftDrive(unsigned char direction, unsigned char speed);

void Sabertooth_RightDrive(unsigned char direction, unsigned char speed);
unsigned char Sabertooth_Checksum(unsigned char address, unsigned char command, unsigned char speed);
void Sabertooth_HardStop(void);
void Sabertooth_DriveTest(void);

#endif // SABERTOOTH_H_INCLUDED
