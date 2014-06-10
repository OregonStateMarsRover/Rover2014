/*
 * PauseSwitch.cpp
 *
 * Created: 4/23/2014 2:41:55 AM
 *  Author: corwin
 */ 
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define STATUSLED (1 << PB4)
#define BUTTON (1 << PB0)
#define XBEEIO (1 << PB3)

#define ROVING 1
#define NOT_ROVING 0

#define LIT 1
#define NOT_LIT 0

volatile unsigned char InterruptCounter = 0;							//A counter for our interrupt function
const unsigned char InterruptCountMax = 15;								//Hold our interrupt count max to make up for only having an 8 bit timer and needing 1Hz led flash
volatile unsigned char LEDState = LIT;									//Holds the current Led state
unsigned char RoverState = ROVING;									//Holds the current rover roving state
	
unsigned char ButtonPushed(void){
	return ((BUTTON & PINB) == 0);										//Returns whether the button has been pushed (active low)
}
	
int main(void)
{
	DDRB = (STATUSLED | XBEEIO);										//Set Outputs
	DDRB &= ~(BUTTON);													//Set Inputs
	PORTB |= (BUTTON);													//Enable Internal pull up resistor for Button

	TCCR0A = 0;															//Clear settings for timer counter 0 register A
	TCCR0B = 0;															//Clear settings for timer counter 0 register B
	TCNT0 = 0;															//zeros the count register

	TCCR0B |= ((1 << CS02) | (1 << CS00));								//Sets the counter prescaler to 1024
	TIMSK = (1 << OCIE0A);												//Sets an interrupt to trigger on counter hitting 255 for timer 0

	sei();		
															//Enable global interrupts
	PORTB |= XBEEIO;
	
    while(1){	
		if(ButtonPushed()){
			if(RoverState == ROVING){									//If the button was pushed and the state was previously roving, switch to non-roving
				PORTB &= ~XBEEIO;										//Send new roving state to rover
				RoverState = NOT_ROVING;								//We are now not roving
			}else if(RoverState == NOT_ROVING){							//If the button was pushed and the state was previously not roving, switch to roving
				PORTB |= XBEEIO;										//Send new roving state to rover
				RoverState = ROVING;									//We are now roving
			}
			_delay_ms(500);	
		}											//Wait 500ms for button debounce
    }
}


ISR(TIMER0_COMPA_vect){								
	if(RoverState == NOT_ROVING){										//If the rover is not roving
		PORTB |= STATUSLED;												//Rover is paused, make status light solid
	}else if(RoverState == ROVING){										//If the rover is roving
		if(InterruptCounter == InterruptCountMax){						//Check to see if the interrupt counter has hit max (needed because you can't get 1Hz from an 8 bit counter)
			if(LEDState == LIT){										//If it has and the led was previously lit
				PORTB &= ~STATUSLED;									//Turn off the led
				LEDState = NOT_LIT;										//Set led state to off
			}else if(LEDState == NOT_LIT){								//If it was previously off
				PORTB |= STATUSLED;										//Turn the led on	
				LEDState = LIT;											//Set led state to on
			}
			InterruptCounter = 0;										//Zero our interrupt counter
		}else{
			InterruptCounter++;											//If the interrupt counter is not at max, increment it
		}
	}
}