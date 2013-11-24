#ifndef SABERTOOTH_H_INCLUDED
#define SABERTOOTH_H_INCLUDED

#define SABERTOOTHADDRESS 128


void Initialize_Sabertooth(void);
void Sabertooth_LeftDrive(unsigned char direction, unsigned char speed);

void Sabertooth_RightDrive(unsigned char direction, unsigned char speed);
unsigned char Sabertooth_Checksum(unsigned char address, unsigned char command, unsigned char speed);
void Sabertooth_HardStop(void);
void Sabertooth_DriveTest(void);

#endif // SABERTOOTH_H_INCLUDED
