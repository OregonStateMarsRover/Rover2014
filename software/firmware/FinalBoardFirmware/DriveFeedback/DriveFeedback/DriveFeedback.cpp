/*
 * DriveFeedback.cpp
 *
 * Created: 4/23/2014 2:37:57 AM
 *  Author: corwin
 */ 

#define AVG_DIN0 PC0
#define AVG_DIN0_INIT() (DDRC &= ~(1 << AVG_DIN0))
#define AVG_DIN0_READ() (PINC & (1 << AVG_DIN0))

#define AVG_DIN1 PC1
#define AVG_DIN1_INIT() (DDRC &= ~(1 << AVG_DIN1))
#define AVG_DIN1_READ() (PINC & (1 << AVG_DIN1))

#define AVG_DIN2 PC2
#define AVG_DIN2_INIT() (DDRC &= ~(1 << AVG_DIN2))
#define AVG_DIN2_READ() (PINC & (1 << AVG_DIN2))

#define AVG_DIN3 PC3
#define AVG_DIN3_INIT() (DDRC &= ~(1 << AVG_DIN3))
#define AVG_DIN3_READ() (PINC & (1 << AVG_DIN3))

#define AVG_DIN4 PC4
#define AVG_DIN4_INIT() (DDRC &= ~(1 << AVG_DIN4))
#define AVG_DIN4_READ() (PINC & (1 << AVG_DIN4))

#define AVG_DIN5 PC5
#define AVG_DIN5_INIT() (DDRC &= ~(1 << AVG_DIN5))
#define AVG_DIN5_READ() (PINC & (1 << AVG_DIN5))

#define AVG_DIN6 PD2	//Note change to PORTD
#define AVG_DIN6_INIT() (DDRD &= ~(1 << AVG_DIN6))
#define AVG_DIN6_READ() (PIND & (1 << AVG_DIN6))

#define AVG_DIN7 PD3
#define AVG_DIN7_INIT() (DDRD &= ~(1 << AVG_DIN7))
#define AVG_DIN7_READ() (PIND & (1 << AVG_DIN7))



#define AVG_OE PD4
#define AVG_OE_INIT() (DDRD |= (1 << AVG_OE))
#define AVG_OE_SET() (PORTD |= (1 << AVG_OE))
#define AVG_OE_CLR() (PORTD &= ~(1 << AVG_OE))

#define AVG_SEL1 PD5
#define AVG_SEL1_INIT() (DDRD |= (1 << AVG_SEL1))
#define AVG_SEL1_SET() (PORTD |= (1 << AVG_SEL1))
#define AVG_SEL1_CLR() (PORTD &= ~(1 << AVG_SEL1))

#define AVG_SEL2 PD6
#define AVG_SEL2_INIT() (DDRD |= (1 << AVG_SEL2))
#define AVG_SEL2_SET() (PORTD |= (1 << AVG_SEL2))
#define AVG_SEL2_CLR() (PORTD &= ~(1 << AVG_SEL2))

#define AVG_RSTY PD7
#define AVG_RSTY_INIT() (DDRD |= (1 << AVG_RSTY))
#define AVG_RSTY_SET() (PORTD |= (1 << AVG_RSTY))
#define AVG_RSTY_CLR() (PORTD &= ~(1 << AVG_RSTY))

#define AVG_RSTX PB1
#define AVG_RSTX_INIT() (DDRB |= (1 << AVG_RSTX))
#define AVG_RSTX_SET() (PORTB |= (1 << AVG_RSTX))
#define AVG_RSTX_CLR() (PORTB &= ~(1 << AVG_RSTX))

#define AVG_XYSEL PB2
#define AVG_XYSEL_INIT() (DDRB |= (1 << AVG_XYSEL))
#define AVG_XYSEL_SET() (PORTB |= (1 << AVG_XYSEL))
#define AVG_XYSEL_CLR() (PORTB &= ~(1 << AVG_XYSEL))

#define SERIALTX PD1
#define SERIALTX_INIT() (DDRD |= (1 << SERIALTX))

#define SERIALRX PD0
#define SERIALRX_INIT() (DDRD &= ~(1 << SERIALRX))

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

unsigned long LeftValBuffer = 0UL;
unsigned long RightValBuffer = 0UL;

char SendBuffer[100];

void PinsInit(){
	AVG_DIN0_INIT();
	AVG_DIN1_INIT();
	AVG_DIN2_INIT();
	AVG_DIN3_INIT();
	AVG_DIN4_INIT();
	AVG_DIN5_INIT();
	AVG_DIN6_INIT();
	AVG_DIN7_INIT();
	
	AVG_OE_INIT();
	AVG_SEL1_INIT();
	AVG_SEL2_INIT();
	AVG_RSTY_INIT();
	AVG_RSTX_INIT();
	AVG_XYSEL_INIT();
	
	AVG_RSTX_SET();
	AVG_RSTY_SET();
	
	AVG_XYSEL_CLR();
	
	
	SERIALTX_INIT();
	SERIALRX_INIT();
	
}

void SerialInit(double newbaud){
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

void SendStringUSART0(char *data){

	for (int loop = 0 ; data[loop] != '\0'; loop++){
		SendByteUSART0(data[loop]);
	}
}

unsigned char ReadAVG(){
	unsigned char temp = 0;
	temp |= (AVG_DIN0_READ() << 0);
	temp |= (AVG_DIN1_READ() << 1);
	temp |= (AVG_DIN2_READ() << 2);
	temp |= (AVG_DIN3_READ() << 3);
	temp |= (AVG_DIN4_READ() << 4);
	temp |= (AVG_DIN5_READ() << 5);
	temp |= (AVG_DIN6_READ() << 6);
	temp |= (AVG_DIN7_READ() << 7);
	return temp;
}

int main(void)
{
	PinsInit();
	_delay_ms(500);
	SerialInit(9600);
	_delay_ms(500);
	
	SendStringUSART0("Device Initialized!!!\r\n\r\n\r\n");
	_delay_ms(2000);

    while(1){
		unsigned char msb = 0;
		unsigned char trd = 0;
		unsigned char scd = 0;
		unsigned char lsb = 0;
		
		AVG_OE_CLR();
		AVG_SEL1_CLR();
		AVG_SEL2_SET();
		
		_delay_ms(10);
		msb = ReadAVG();
		
		AVG_SEL1_SET();
		_delay_ms(10);
		trd = ReadAVG();
		
		AVG_SEL1_CLR();
		AVG_SEL2_CLR();
		_delay_ms(10);
		scd = ReadAVG();
		
		AVG_SEL1_SET();
		_delay_ms(10);
		lsb = ReadAVG();
		
		AVG_OE_SET();
		
		sprintf(SendBuffer,"The lsb value is: %d.\r\n", lsb);
		SendStringUSART0(SendBuffer);
		_delay_ms(100);
    }
}