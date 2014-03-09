#include <usart.h>
#include <stdint.h>
#include <Sabertooth.h>
#include <util/delay.h>
#include <main.h>
#include <Mega2560.h>

void Initialize_Sabertooth(void){
    SendByteUSART1(AUTOBAUD_BYTE);                          //This byte tells the controller to automatically determine the usart communication speed.
    Sabertooth_WriteCommand(SABERTOOTHADDRESS, 14, 20);      //This setting enables the sabertooth watchdog, which will stop the motors after a certain period of no control packets.
    Sabertooth_HardStop();
}

void Sabertooth_WriteCommand(unsigned char address, unsigned char command, unsigned char value){

    SendByteUSART1(address);
    SendByteUSART1(command);
    SendByteUSART1(value);
    SendByteUSART1(Sabertooth_Checksum(address, command, value));
}

void Sabertooth_SetMotors(unsigned char address, unsigned char LeftDir, unsigned char LeftSpeed, unsigned char RightDir, unsigned char RightSpeed){

    if(LeftDir == DRIVE_FORWARD){
        LeftDir = LEFT_FORWARD;
    }else if(LeftDir == DRIVE_BACKWARD){
        LeftDir = LEFT_BACK;
    }else if(LeftDir == DRIVE_STOP){
        LeftDir = LEFT_FORWARD;
        LeftSpeed = 0;
    }

    if(RightDir == DRIVE_FORWARD){
        RightDir = RIGHT_FORWARD;
    }else if(RightDir == DRIVE_BACKWARD){
        RightDir = RIGHT_BACK;
    }else if(RightDir == DRIVE_STOP){
        RightDir = RIGHT_FORWARD;
        RightSpeed = 0;
    }

    unsigned char LeftChecksum = Sabertooth_Checksum(address, LeftDir, LeftSpeed);
    unsigned char RightChecksum = Sabertooth_Checksum(address, RightDir, RightSpeed);


    SendByteUSART1(address);
    SendByteUSART1(LeftDir);
    SendByteUSART1(LeftSpeed);
    SendByteUSART1(LeftChecksum);

    SendByteUSART1(address);
    SendByteUSART1(RightDir);
    SendByteUSART1(RightSpeed);
    SendByteUSART1(RightChecksum);
}

void Sabertooth_LeftDrive(unsigned char direction, unsigned char speed){
    unsigned char address = SABERTOOTHADDRESS;                              //Assigns our define to a nice variable so the compiler won't complain
    unsigned char command = command;                                        //Cannot be initialized unassigned due to compiler optimizations

    if(direction == 2){
        command = 0;
        speed = 0;
    }else if(direction == 1){
        command = 0;                                                        //Sabertooth command for driving motor 1 forward
    }else if(direction == 0){
        command = 1;                                                        //Sabertooth command for driving motor 1 backwards
    }

    SendByteUSART1(SABERTOOTHADDRESS);
    SendByteUSART1(command);
    SendByteUSART1(speed);
    SendByteUSART1(Sabertooth_Checksum(address, command, speed));
}

void Sabertooth_RightDrive(unsigned char direction, unsigned char speed){
    unsigned char address = SABERTOOTHADDRESS;                              //Assigns our define to a nice variable so the compiler won't complain
    unsigned char command = command;                                        //Cannot be initialized unassigned due to compiler optimizations

    if(direction == 2){
        command = 0;
        speed = 0;
    }else if(direction == 1){
        command = 4;        //Sabertooth command for driving motor 1 forward
    }else if(direction == 0){
        command = 5;        //Sabertooth command for driving motor 1 backwards
    }

    SendByteUSART1(SABERTOOTHADDRESS);
    SendByteUSART1(command);
    SendByteUSART1(speed);
    SendByteUSART1(Sabertooth_Checksum(address, command, speed));
}

unsigned char Sabertooth_Checksum(unsigned char address, unsigned char command, unsigned char speed){
    uint8_t sum = (address + command + speed);
    uint8_t checksum = (sum & 127);
    return checksum;
}

void Sabertooth_HardStop(void){
    Sabertooth_LeftDrive(1, 0);
    Sabertooth_RightDrive(1, 0);
}

void Sabertooth_DriveTest(void){
    int i;
    for(i = 0 ; i < 128 ; i++){
        Sabertooth_SetMotors(SABERTOOTHADDRESS, DRIVE_FORWARD, i, DRIVE_FORWARD, i);
            _delay_ms(20);
    }
    for( ; i > 0 ; i--){
        Sabertooth_SetMotors(SABERTOOTHADDRESS, DRIVE_FORWARD, i, DRIVE_FORWARD, i);
        _delay_ms(20);
    }
    for(i = 0 ; i < 128 ; i++){
        Sabertooth_SetMotors(SABERTOOTHADDRESS, DRIVE_BACKWARD, i, DRIVE_BACKWARD, i);
        _delay_ms(20);
    }
    for( ; i > 0 ; i--){
        Sabertooth_SetMotors(SABERTOOTHADDRESS, DRIVE_BACKWARD, i, DRIVE_BACKWARD, i);
        _delay_ms(20);
    }
}
