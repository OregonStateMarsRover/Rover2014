/*
 */
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{

    DDRB = 0b11111111;

    while(1){
        PORTB = 0b11111111;
        //_delay_ms(1);
        PORTB = 0;
        //_delay_ms(1);
    }

    return 0;
}
