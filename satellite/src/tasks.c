#include <sat/tasks.h>

#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <sat/uart.h>

typedef struct{
    void (*function)(void);
    uint16_t delay;
} task_t;

volatile task_t tasks[8];

#ifdef DEBUG_NT
volatile uint8_t num_tasks = 0;
#endif
volatile uint8_t flags = 0;

void task_init(void)
{
    // For task manager, we will use Timer1
    // Let's init it!
    
    TCCR1B = (1<<WGM12)|(1<<CS12); // clock divided by 256
    OCR1A = 63; // the top of counter; gives near 1 kHz frequency

    TIMSK |= 1<<OCIE1A; // enable CompA interrupt

    uint8_t i;

    for(i=0; i<8; i++)
    {
	    tasks[i].delay = 65535;
        tasks[i].function = 0;
    }
}

void task_manager(void)
{
    uint8_t i;

    for(i=0; i!=8; i++)
    {
        if(flags & (1<<i))
        {
            tasks[i].delay = 65535;
            flags &= ~(1<<i);
            tasks[i].function();
            #ifdef DEBUG_NT
            num_tasks--;
            #endif
        }
    }
}

void task_add(void * func, uint16_t delay)
{
    uint8_t i;

    for(i=0; i!=8; i++)
    {
        if(tasks[i].delay == 65535) // if timer is not used
        {
            tasks[i].function = func;
            tasks[i].delay = delay;
            #ifdef DEBUG_NT
            num_tasks++;
            #endif
            return;
        }
    }
}

void sys_reset(void)
{
    wdt_enable(WDTO_15MS);
    _delay_ms(20);
}

// Timer0 interrupt vector: get tasks from timers

ISR(TIMER1_COMPA_vect)
{
    // We need to check all tasks
    uint8_t i;

    for(i=0; i!=8; i++)
    {
        if(tasks[i].delay != 65535)
        {
            if(tasks[i].delay == 0)
            {
                flags |= 1<<i;
            }
            else tasks[i].delay--;
        }
    }
}
