#include <arch/antares.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x.h>
#include <lib/tasks.h>

/*
void SysTick_Handler(void)
{
    tmgr_tick();
}
*/
int main(void)
{
    GPIO_InitTypeDef LED_Init = {
        .GPIO_Pin = GPIO_Pin_5,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_Out_PP
    };

    // enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // init GPIOA
    GPIO_Init(GPIOA, &LED_Init);

    // init SysTick
    SysTick_Config(SystemCoreClock / 1000);
    
    while(1)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
        sleep_ticks(1000);
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        sleep_ticks(1000);
    }
    
    return 0;
}

void mdelay(void)
{

}
