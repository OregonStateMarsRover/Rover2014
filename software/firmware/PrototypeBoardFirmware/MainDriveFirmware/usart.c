#include <avr/io.h>
#include <usart.h>
#include <avr/interrupt.h>
#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <Sabertooth.h>
#include <Mega2560.h>

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

unsigned char ChecksumUSART0(unsigned char estop, unsigned char left, unsigned char right){
    return (estop ^ left ^ right);
}

ISR (USART0_RX_vect){
    receive_buffer[bufferpos] = UDR0;
    bufferpos++;

    if(bufferpos == PACKETSIZE){
        unsigned char leftDir = leftDir;
        unsigned char rightDir = rightDir;

        bufferpos = 0;

        if(receive_buffer[1] & ESTOP){
            Sabertooth_HardStop();
        }else if(receive_buffer[4] == ChecksumUSART0(receive_buffer[1], receive_buffer[2], receive_buffer[3])){

            if(receive_buffer[2] == 127){
                receive_buffer[2] = 0;
                leftDir = DRIVE_STOP;
            }else if(receive_buffer[2] < 127){
                receive_buffer[2] = 127 - receive_buffer[2];
                leftDir = DRIVE_BACKWARD;
            }else if(receive_buffer[2] > 127){
                receive_buffer[2] -= 127;
                leftDir = DRIVE_FORWARD;
            }

            if(receive_buffer[3] == 127){
                receive_buffer[3] = 0;
                rightDir = DRIVE_STOP;
            }else if(receive_buffer[3] < 127){
                receive_buffer[3] = 127 - receive_buffer[3];
                rightDir = DRIVE_BACKWARD;
            }else if(receive_buffer[3] > 127){
                receive_buffer[3] -= 127;
                rightDir = DRIVE_FORWARD;
            }
            Sabertooth_SetMotors(SABERTOOTHADDRESS, leftDir, receive_buffer[2], rightDir, receive_buffer[3]);
        }
        SendByteUSART0('r');
    }

    return;
}
