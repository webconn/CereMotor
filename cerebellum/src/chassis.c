/**
 * @file chassis.c
 * @brief STM32F103RET6 init code
 * @author WebConn
 * @date 4 Jan 2013
 * @version 0.1
 */

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>

#include <cerebellum/chassis.h>

#define LEFT 1
#define RIGHT 0
#define FORWARD 1
#define BACKWARD 0

#ifdef CONFIG_ROBOT_2013
    #define CHASSIS_APB1 RCC_APB1Periph_TIM4
    #define CHASSIS_APB2 RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC

    #define DIR_LEFT_FWD_PIN GPIO_Pin_10
    #define DIR_LEFT_FWD_GPIO GPIOC
    
    #define DIR_LEFT_BWD_PIN GPIO_Pin_2
    #define DIR_LEFT_BWD_GPIO GPIOD
    
    #define DIR_RIGHT_FWD_PIN GPIO_Pin_3
    #define DIR_RIGHT_FWD_GPIO GPIOC
    
    #define DIR_RIGHT_BWD_PIN GPIO_Pin_2
    #define DIR_RIGHT_BWD_GPIO GPIOC

    #define PWM_LEFT_PIN GPIO_Pin_8
    #define PWM_LEFT_GPIO GPIOB

    #define PWM_RIGHT_PIN GPIO_Pin_9
    #define PWM_RIGHT_GPIO GPIOB

    #define PWM_LEFT_TIMER TIM4
    #define PWM_LEFT_OC 3

    #define PWM_RIGHT_TIMER TIM4
    #define PWM_RIGHT_OC 4
#endif

/**
 * Init of movement system
 */

void chassis_init(void)
{
    /*
     * Initialision of chassis
     * is configuring GPIO and PWM timers
     */

    // Clock start
    RCC_APB1PeriphClockCmd(CHASSIS_APB1, ENABLE);
    RCC_APB2PeriphClockCmd(CHASSIS_APB2, ENABLE);

    /*
     * Init GPIO of the directors
     */

    GPIO_InitTypeDef directors = {
        .GPIO_Speed = GPIO_Speed_10MHz,
        .GPIO_Mode = GPIO_Mode_Out_PP
    };

    // Left FWD pin
    directors.GPIO_Pin = DIR_LEFT_FWD_PIN;
    GPIO_Init(DIR_LEFT_FWD_GPIO, &directors);

    // Left BWD pin
    directors.GPIO_Pin = DIR_LEFT_BWD_PIN;
    GPIO_Init(DIR_LEFT_BWD_GPIO, &directors);

    // Right FWD pin
    directors.GPIO_Pin = DIR_RIGHT_FWD_PIN;
    GPIO_Init(DIR_RIGHT_FWD_GPIO, &directors);

    // Right BWD pin
    directors.GPIO_Pin = DIR_RIGHT_BWD_PIN;
    GPIO_Init(DIR_RIGHT_BWD_GPIO, &directors);

    /*
     * Init GPIO of PWM outputs
     */

    GPIO_InitTypeDef PWM_Outputs = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_AF_PP
    };

    // Left PWM pin
    PWM_Outputs.GPIO_Pin = PWM_LEFT_PIN;
    GPIO_Init(PWM_LEFT_GPIO, &PWM_Outputs);
    
    // Right PWM pin
    PWM_Outputs.GPIO_Pin = PWM_RIGHT_PIN;
    GPIO_Init(PWM_RIGHT_GPIO, &PWM_Outputs);

    /*
     * Init PWM timers
     */

    // Init time base
    TIM_TimeBaseInitTypeDef timer = {
        .TIM_Period = 8191,                     // PWM accuracy is 8192 variants
        .TIM_Prescaler = 0,                     // Prescaler off
        .TIM_ClockDivision = 0,                 // Clock divider if off
        .TIM_CounterMode = TIM_CounterMode_Up   // Upcounting
    };

    TIM_TimeBaseInit(PWM_LEFT_TIMER, &timer);
    TIM_TimeBaseInit(PWM_RIGHT_TIMER, &timer);

    // Output channels
    TIM_OCInitTypeDef TIM_Out = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_Pulse = 0
    };

    // Left timer
    #if PWM_LEFT_OC == 1
        TIM_OC1Init(PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC1PreloadConfig(PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #elif PWM_LEFT_OC == 2
        TIM_OC2Init(PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC2PreloadConfig(PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #elif PWM_LEFT_OC == 3
        TIM_OC3Init(PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC3PreloadConfig(PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #elif PWM_LEFT_OC == 4
        TIM_OC4Init(PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC4PreloadConfig(PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #else
        #error "Invalid Output Compare number for left PWM: must be 1-4"
    #endif

    // Right timer
    #if PWM_RIGHT_OC == 1
        TIM_OC1Init(PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC1PreloadConfig(PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #elif PWM_RIGHT_OC == 2
        TIM_OC2Init(PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC2PreloadConfig(PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #elif PWM_RIGHT_OC == 3
        TIM_OC3Init(PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC3PreloadConfig(PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #elif PWM_RIGHT_OC == 4
        TIM_OC4Init(PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC4PreloadConfig(PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #else
        #error "Invalid Output Compare number for right PWM: must be 1-4"
    #endif

    // Enable timers
    TIM_ARRPreloadConfig(PWM_LEFT_TIMER, ENABLE);
    TIM_ARRPreloadConfig(PWM_RIGHT_TIMER, ENABLE);

    TIM_Cmd(PWM_LEFT_TIMER, ENABLE);
    TIM_Cmd(PWM_RIGHT_TIMER, ENABLE);
}

void chassis_write(int16_t left, int16_t right)
{
    // 1. Parsing left engine info
    if(left < 0)
    {
        left = -left;
        chassis_set_dir(LEFT, BACKWARD);
    }
    else
    {
        chassis_set_dir(LEFT, FORWARD);
    }

    // 2. Parsing right engine info
    if(right < 0)
    {
        right = -right;
        chassis_set_dir(RIGHT, BACKWARD);
    }
    else
    {
        chassis_set_dir(RIGHT, FORWARD);
    }

    // 3. Updating PWM timers
    
    // Left timer
    #if PWM_LEFT_OC == 1
        TIM_SetCompare1(PWM_LEFT_TIMER, left);
    #elif PWM_LEFT_OC == 2
        TIM_SetCompare2(PWM_LEFT_TIMER, left);
    #elif PWM_LEFT_OC == 3
        TIM_SetCompare3(PWM_LEFT_TIMER, left);
    #elif PWM_LEFT_OC == 4
        TIM_SetCompare4(PWM_LEFT_TIMER, left);
    #endif
    
    // Right timer
    #if PWM_RIGHT_OC == 1
        TIM_SetCompare1(PWM_RIGHT_TIMER, right);
    #elif PWM_RIGHT_OC == 2
        TIM_SetCompare2(PWM_RIGHT_TIMER, right);
    #elif PWM_RIGHT_OC == 3
        TIM_SetCompare3(PWM_RIGHT_TIMER, right);
    #elif PWM_RIGHT_OC == 4
        TIM_SetCompare4(PWM_RIGHT_TIMER, right);
    #endif
}

void chassis_set_dir(uint8_t engine, uint8_t direction)
{
    if(engine)
    {
        if(direction)
        {
            GPIO_SetBits(DIR_LEFT_FWD_GPIO, DIR_LEFT_FWD_PIN);
            GPIO_ResetBits(DIR_LEFT_BWD_GPIO, DIR_LEFT_BWD_PIN);
        }
        else
        {
            GPIO_SetBits(DIR_LEFT_BWD_GPIO, DIR_LEFT_BWD_PIN);
            GPIO_ResetBits(DIR_LEFT_FWD_GPIO, DIR_LEFT_FWD_PIN);
        }
    }
    else
    {
        if(direction)
        {
            GPIO_SetBits(DIR_RIGHT_FWD_GPIO, DIR_RIGHT_FWD_PIN);
            GPIO_ResetBits(DIR_RIGHT_BWD_GPIO, DIR_RIGHT_BWD_PIN);
        }
        else
        {
            GPIO_SetBits(DIR_RIGHT_BWD_GPIO, DIR_RIGHT_BWD_PIN);
            GPIO_ResetBits(DIR_RIGHT_FWD_GPIO, DIR_RIGHT_FWD_PIN);
        }
    }
}
