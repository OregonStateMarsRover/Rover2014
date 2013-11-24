#include <avr/io.h>
#include <usart.h>
#include <avr/interrupt.h>
#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <Sabertooth.h>

void Initialize_USART0(double newbaud){
    PRR0 &= ~(1<<PRUSART0);  //Disables power saving mode

    int baud_prescaller = ((F_CPU / (newbaud * 16UL)) - 1);
    UBRR0H = (baud_prescaller >> 8);
    UBRR0L = baud_prescaller;

    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0 | 1 << RXEN0 | 1 << RXCIE0);
    // Set frame format: 8data, 1stop bit
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));


}

void SendByteUSART0(char data){
    while (!( UCSR0A & (1<<UDRE0)));    //BLOCKING!!!!!
    UDR0 = data;
}

void SendStringUSART0(unsigned char *data){

	for (int loop = 0 ; data[loop] != '\0'; loop++){
	//Storage for return value of SendByteUART

		/* Sends the current byte based on the loop umber to SendByte */
		SendByteUSART0(data[loop]);

		/* Stops sending if SendByteUART had an error */
	}
}

unsigned char GetByteUART(void){
	if((UCSR0A & (1 << RXC0))){
        return(UDR0);
	}else{
        return -1;
	}
}

void Initialize_USART1(double newbaud){
    PRR1 &= ~(1<<PRUSART1);

    int baud_prescaller = ((F_CPU / (newbaud * 16UL)) - 1);
    UBRR1H = (baud_prescaller >> 8);
    UBRR1L = baud_prescaller;

    // Enable transmitter and receiver
    UCSR1B = (1 << TXEN1 | 1 << RXEN1);
    // Set frame format: 8data, 1stop bit
    UCSR1C = ((1<<UCSZ10)|(1<<UCSZ11));
}

void SendByteUSART1(char data){
    while (!( UCSR1A & (1<<UDRE1)));    //BLOCKING!!!!!
    UDR1 = data;
}

ISR (USART0_RX_vect){
    receive_buffer[bufferpos] = UDR0;
    bufferpos++;

    if(bufferpos == PACKETSIZE){
        char orig_estop[2] = {255, 255};
        char orig_leftspeed[2] = {255, 255};
        char orig_rightspeed[2] = {255, 255};
        char orig_checksum[2] = {255, 255};
        unsigned char new_estop = {255};
        unsigned char new_leftspeed = {255};
        unsigned char new_rightspeed = {255};
        unsigned char new_checksum = {255};
        unsigned char leftDir = leftDir;
        unsigned char rightDir = rightDir;

        bufferpos = 0;

        orig_estop[0] = receive_buffer[2];
        orig_estop[1] = receive_buffer[3];
        orig_leftspeed[0] = receive_buffer[4];
        orig_leftspeed[1] = receive_buffer[5];
        orig_rightspeed[0] = receive_buffer[6];
        orig_rightspeed[1] = receive_buffer[7];
        orig_checksum[0] = receive_buffer[8];
        orig_checksum[1] = receive_buffer[9];

        new_estop = strtol(orig_estop, NULL, 16);
        new_leftspeed = strtol(orig_leftspeed, NULL, 16);
        new_rightspeed = strtol(orig_rightspeed, NULL, 16);
        new_checksum = strtol(orig_checksum, NULL, 16);

        if(new_leftspeed == 127){
            new_leftspeed = 0;
            leftDir = DRIVE_FORWARD;
        }else if(new_leftspeed < 127){
            new_leftspeed = 127 - new_leftspeed;
            leftDir = DRIVE_BACKWARD;
        }else if(new_leftspeed > 127){
            new_leftspeed -= 127;
            leftDir = DRIVE_FORWARD;
        }

        if(new_rightspeed == 127){
            new_rightspeed = 0;
            rightDir = DRIVE_FORWARD;
        }else if(new_rightspeed < 127){
            new_rightspeed = 127 - new_rightspeed;
            rightDir = DRIVE_BACKWARD;
        }else if(new_rightspeed > 127){
            new_rightspeed -= 127;
            rightDir = DRIVE_FORWARD;
        }
        Sabertooth_SetMotors(SABERTOOTHADDRESS, leftDir, new_leftspeed, rightDir, new_rightspeed);

    }

    //UCSR0B &= ~(1 << RXCIE0); //Manually clears interrupt flag if you don't read the data on the port
    return;
}
