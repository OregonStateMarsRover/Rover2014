#include <Mega2560.h>
#include <avr/io.h>

void Initialize_Mega2560(void){
    DDRB |= (1<<PB7);
}
