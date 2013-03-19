#include <cerebellum/snippet.h>

void _snippet_runGPIORCC(GPIO_TypeDef * gpio)
{
    uint32_t cmd = 0;
    if(gpio == GPIOA)
        cmd = RCC_APB2Periph_GPIOA;
    else if(gpio == GPIOB)
        cmd = RCC_APB2Periph_GPIOB;
    else if(gpio == GPIOC)
        cmd = RCC_APB2Periph_GPIOC;
    else if(gpio == GPIOD)
        cmd = RCC_APB2Periph_GPIOD;
    else if(gpio == GPIOE)
        cmd = RCC_APB2Periph_GPIOE;
    else if(gpio == GPIOF)
        cmd = RCC_APB2Periph_GPIOF;
        
    RCC_APB2PeriphClockCmd(cmd, ENABLE);
}

void _snippet_runTIMRCC(TIM_TypeDef * tim)
{
    uint32_t cmd = 0;
    
    if(tim == TIM2)
        cmd = RCC_APB1Periph_TIM2;
    else if(tim == TIM3)
        cmd = RCC_APB1Periph_TIM3;
    else if(tim == TIM4)
        cmd = RCC_APB1Periph_TIM4;
    else if(tim == TIM5)
        cmd = RCC_APB1Periph_TIM5;
    else if(tim == TIM6)
        cmd = RCC_APB1Periph_TIM6;
    else if(tim == TIM7)
        cmd = RCC_APB1Periph_TIM7;
    else if(tim == TIM12)
        cmd = RCC_APB1Periph_TIM12;
    else if(tim == TIM13)
        cmd = RCC_APB1Periph_TIM13;
    else if(tim == TIM14)
        cmd = RCC_APB1Periph_TIM14;

    if(cmd) // if timer at APB1
        RCC_APB1PeriphClockCmd(cmd, ENABLE);
    else // look at APB2
    {
        if(tim == TIM1)
            cmd = RCC_APB2Periph_TIM1;
        else if(tim == TIM8)
            cmd = RCC_APB2Periph_TIM8;
        else if(tim == TIM9)
            cmd = RCC_APB2Periph_TIM9;
        else if(tim == TIM10)
            cmd = RCC_APB2Periph_TIM10;
        else if(tim == TIM11)
            cmd = RCC_APB2Periph_TIM11;
        else if(tim == TIM15)
            cmd = RCC_APB2Periph_TIM15;
        else if(tim == TIM16)
            cmd = RCC_APB2Periph_TIM16;
        else if(tim == TIM17)
            cmd = RCC_APB2Periph_TIM17;
            
        RCC_APB2PeriphClockCmd(cmd, ENABLE);
    }


}
