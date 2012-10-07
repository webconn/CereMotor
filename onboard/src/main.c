#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// In this module, we will be only running in interrupts
// Other time will be in sleep mode

uint8_t current_adc = 2; // using ADC3 for right and ADC2 for left

int main(void)
{
    // 1. Init i/o
    DDRB = (1<<1)|(1<<2); // outputs: PB1 to right channel, PB2 to left

    // 2. init ADC
    ADMUX = (1<<REFS0)|(1<<ADLAR)|current_adc; // ignoring two lower bits
    ADCSRA = (1<<ADEN)|(1<<ADIE);
    ADCSRB = (1<<ADTS0);

    // 3. init ADC noise reduction sleep
    MCUCR |= (1<<SM0);

    sei();

    // 4. run conversion and sleep
    ADCSRA |= (1<<ADSC);

    while(1)
        MCUCR |= (1<<SE); // goto sleep

    // make gcc happy
    return 0;
}

ISR(ADC_vect)
{
    // 1. get value
    if(ADCH > CONFIG_CRITICAL_VALUE)
    {
        if(current_adc == 2) PORTB |= (1<<2);
        else PORTB |= (1<<1);
    }
    else
    {
        if(current_adc == 2) PORTB &= ~(1<<2);
        else PORTB &= ~(1<<1);
    }

    // 2. init next conversion
    current_adc++;
    if(current_adc == 4)
        current_adc = 2;

    ADMUX &= ~(3);
    ADMUX |= current_adc;
    ADCSRA |= (1<<ADSC);

}
