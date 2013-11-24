#include <usart.h>
#include <stdint.h>
#include <Sabertooth.h>
#include <util/delay.h>

void Initialize_Sabertooth(void){
    SendByteUSART1(170);        //Autobaud Byte for Sabertooth
    Sabertooth_HardStop();
}

void Sabertooth_LeftDrive(unsigned char direction, unsigned char speed){
    unsigned char address = SABERTOOTHADDRESS;
    unsigned char command;

    if(direction == 2){
        command = 0;
        speed = 0;
    }else if(direction == 1){
        command = 0;        //Sabertooth command for driving motor 1 forward
    }else if(direction == 0){
        command = 1;        //Sabertooth command for driving motor 1 backwards
    }

    SendByteUSART1(SABERTOOTHADDRESS);
    SendByteUSART1(command);
    SendByteUSART1(speed);
    SendByteUSART1(Sabertooth_Checksum(address, command, speed));
}

void Sabertooth_RightDrive(unsigned char direction, unsigned char speed){
    unsigned char address = SABERTOOTHADDRESS;
    unsigned char command;

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
        Sabertooth_LeftDrive(1, i);
        Sabertooth_RightDrive(1, i);
        _delay_ms(20);
    }
    for( ; i > 0 ; i--){
        Sabertooth_LeftDrive(1, i);
        Sabertooth_RightDrive(1, i);
        _delay_ms(20);
    }
    for(i = 0 ; i < 128 ; i++){
        Sabertooth_LeftDrive(0, i);
        Sabertooth_RightDrive(0, i);
        _delay_ms(20);
    }
    for( ; i > 0 ; i--){
        Sabertooth_LeftDrive(0, i);
        Sabertooth_RightDrive(0, i);
        _delay_ms(20);
    }
}
