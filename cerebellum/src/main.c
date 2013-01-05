#include <cerebellum/chassis.h>
#include <stm32f10x.h>

void delay(void)
{
    uint32_t i = 9999;
    while(i-- > 0)
    {
        asm volatile("nop");
    }
}

int main(void)
{
    chassis_init();

    while(1)
    {
        int16_t i;
        for(i = -8191; i < 8192; i++)
        {
            chassis_write(i, i);
            delay();
        }
    }
    
    return 0;
}
