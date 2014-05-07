#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#define ESTOP 0b00000001

void Initialize_USART0(double newbaud);
void SendByteUSART0(char data);
void SendStringUSART0 (unsigned char *data);
unsigned char GetByteUART(void);

void Initialize_USART1(double newbaud);
void SendByteUSART1(char data);
unsigned char ChecksumUSART0(unsigned char estop, unsigned char left, unsigned char right);
#endif // USART_H_INCLUDED
