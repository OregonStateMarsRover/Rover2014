/*
 */
#include <stdio.h>
#include <main.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <usart.h>
#include <Sabertooth.h>
#include <Mega2560.h>

int main(void)
{
    unsigned char currentState = 0;
    nextState = NOINTERRUPT;

    for(int i = 0 ; i < PACKETSIZE ; i++){
        receive_buffer[i] = 255;
    }

    bufferpos = 0;

    while(1){
        if(nextState != NOINTERRUPT){
            currentState = nextState;
        }
        switch(currentState){
            case INITIALIZATION:
                Initialize_Mega2560();
                Initialize_USART0(9600);   //Fastest Stable Clock is 38400
                Initialize_USART1(9600);
                Initialize_Sabertooth();
                currentState = WAITFORHOST;
                break;

            case WAITFORHOST:
                cli();
                SendStringUSART0((unsigned char *)"ID: MainDrive\r\n");
                if(GetByteUART() == 'D'){
                    SendStringUSART0((unsigned char *)"Master Found. Switching to Drive Mode.\r\n");
                    currentState = DRIVING;
                    SendByteUSART0('r');
                    sei();
                    break;
                }
                _delay_ms(500);
                PORTB |= (1<<PB7);
                _delay_ms(100);
                PORTB &= ~(1<<PB7);
                _delay_ms(100);
                break;

            case DRIVING:
                break;
            case 3:
                break;

            default:
                break;


        }
    }
    return 0;
}
