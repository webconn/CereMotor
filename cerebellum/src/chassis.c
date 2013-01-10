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
#include <robots/config.h>

#define LEFT 1
#define RIGHT 0
#define FORWARD 1
#define BACKWARD 0

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
    RCC_APB1PeriphClockCmd(CONFIG_CHASSIS_APB1, ENABLE);
    RCC_APB2PeriphClockCmd(CONFIG_CHASSIS_APB2, ENABLE);

    /*
     * Init GPIO of the directors
     */

    GPIO_InitTypeDef directors = {
        .GPIO_Speed = GPIO_Speed_10MHz,
        .GPIO_Mode = GPIO_Mode_Out_PP
    };

    // Left FWD pin
    directors.GPIO_Pin = 1<<CONFIG_DIR_LEFT_FWD_PIN;
    GPIO_Init(CONFIG_DIR_LEFT_FWD_GPIO, &directors);

    // Left BWD pin
    directors.GPIO_Pin = 1<<CONFIG_DIR_LEFT_BWD_PIN;
    GPIO_Init(CONFIG_DIR_LEFT_BWD_GPIO, &directors);

    // Right FWD pin
    directors.GPIO_Pin = 1<<CONFIG_DIR_RIGHT_FWD_PIN;
    GPIO_Init(CONFIG_DIR_RIGHT_FWD_GPIO, &directors);

    // Right BWD pin
    directors.GPIO_Pin = 1<<CONFIG_DIR_RIGHT_BWD_PIN;
    GPIO_Init(CONFIG_DIR_RIGHT_BWD_GPIO, &directors);

    /*
     * Init GPIO of PWM outputs
     */

    GPIO_InitTypeDef CONFIG_PWM_Outputs = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_AF_PP
    };

    // Left PWM pin
    CONFIG_PWM_Outputs.GPIO_Pin = 1<<CONFIG_PWM_LEFT_PIN;
    GPIO_Init(CONFIG_PWM_LEFT_GPIO, &CONFIG_PWM_Outputs);
    
    // Right PWM pin
    CONFIG_PWM_Outputs.GPIO_Pin = 1<<CONFIG_PWM_RIGHT_PIN;
    GPIO_Init(CONFIG_PWM_RIGHT_GPIO, &CONFIG_PWM_Outputs);

    /*
     * Init PWM timers
     */

    // Init time base
    TIM_TimeBaseInitTypeDef timer = {
        .TIM_Period = CONFIG_PWM_ACCURACY - 1,  // PWM accuracy
        .TIM_Prescaler = 0,                     // Prescaler off
        .TIM_ClockDivision = 0,                 // Clock divider if off
        .TIM_CounterMode = TIM_CounterMode_Up   // Upcounting
    };

    TIM_TimeBaseInit(CONFIG_PWM_LEFT_TIMER, &timer);
    TIM_TimeBaseInit(CONFIG_PWM_RIGHT_TIMER, &timer);

    // Output channels
    TIM_OCInitTypeDef TIM_Out = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_Pulse = 0
    };

    // Left timer
    #if CONFIG_PWM_LEFT_OC == 1
        TIM_OC1Init(CONFIG_PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC1PreloadConfig(CONFIG_PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #elif CONFIG_PWM_LEFT_OC == 2
        TIM_OC2Init(CONFIG_PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC2PreloadConfig(CONFIG_PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #elif CONFIG_PWM_LEFT_OC == 3
        TIM_OC3Init(CONFIG_PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC3PreloadConfig(CONFIG_PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #elif CONFIG_PWM_LEFT_OC == 4
        TIM_OC4Init(CONFIG_PWM_LEFT_TIMER, &TIM_Out);
        TIM_OC4PreloadConfig(CONFIG_PWM_LEFT_TIMER, TIM_OCPreload_Enable);
    #else
        #error "Invalid Output Compare number for left PWM: must be 1-4"
    #endif

    // Right timer
    #if CONFIG_PWM_RIGHT_OC == 1
        TIM_OC1Init(CONFIG_PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC1PreloadConfig(CONFIG_PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #elif CONFIG_PWM_RIGHT_OC == 2
        TIM_OC2Init(CONFIG_PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC2PreloadConfig(CONFIG_PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #elif CONFIG_PWM_RIGHT_OC == 3
        TIM_OC3Init(CONFIG_PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC3PreloadConfig(CONFIG_PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #elif CONFIG_PWM_RIGHT_OC == 4
        TIM_OC4Init(CONFIG_PWM_RIGHT_TIMER, &TIM_Out);
        TIM_OC4PreloadConfig(CONFIG_PWM_RIGHT_TIMER, TIM_OCPreload_Enable);
    #else
        #error "Invalid Output Compare number for right PWM: must be 1-4"
    #endif

    // Enable timers
    TIM_ARRPreloadConfig(CONFIG_PWM_LEFT_TIMER, ENABLE);
    TIM_ARRPreloadConfig(CONFIG_PWM_RIGHT_TIMER, ENABLE);

    TIM_Cmd(CONFIG_PWM_LEFT_TIMER, ENABLE);
    TIM_Cmd(CONFIG_PWM_RIGHT_TIMER, ENABLE);
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
    #if CONFIG_PWM_LEFT_OC == 1
        TIM_SetCompare1(CONFIG_PWM_LEFT_TIMER, left);
    #elif CONFIG_PWM_LEFT_OC == 2
        TIM_SetCompare2(CONFIG_PWM_LEFT_TIMER, left);
    #elif CONFIG_PWM_LEFT_OC == 3
        TIM_SetCompare3(CONFIG_PWM_LEFT_TIMER, left);
    #elif CONFIG_PWM_LEFT_OC == 4
        TIM_SetCompare4(CONFIG_PWM_LEFT_TIMER, left);
    #endif
    
    // Right timer
    #if CONFIG_PWM_RIGHT_OC == 1
        TIM_SetCompare1(CONFIG_PWM_RIGHT_TIMER, right);
    #elif CONFIG_PWM_RIGHT_OC == 2
        TIM_SetCompare2(CONFIG_PWM_RIGHT_TIMER, right);
    #elif CONFIG_PWM_RIGHT_OC == 3
        TIM_SetCompare3(CONFIG_PWM_RIGHT_TIMER, right);
    #elif CONFIG_PWM_RIGHT_OC == 4
        TIM_SetCompare4(CONFIG_PWM_RIGHT_TIMER, right);
    #endif
}

void chassis_set_dir(uint8_t engine, uint8_t direction)
{
    if(engine)
    {
        if(direction)
        {
            GPIO_SetBits(CONFIG_DIR_LEFT_FWD_GPIO, 1<<CONFIG_DIR_LEFT_FWD_PIN);
            GPIO_ResetBits(CONFIG_DIR_LEFT_BWD_GPIO, 1<<CONFIG_DIR_LEFT_BWD_PIN);
        }
        else
        {
            GPIO_SetBits(CONFIG_DIR_LEFT_BWD_GPIO, 1<<CONFIG_DIR_LEFT_BWD_PIN);
            GPIO_ResetBits(CONFIG_DIR_LEFT_FWD_GPIO, 1<<CONFIG_DIR_LEFT_FWD_PIN);
        }
    }
    else
    {
        if(direction)
        {
            GPIO_SetBits(CONFIG_DIR_RIGHT_FWD_GPIO, 1<<CONFIG_DIR_RIGHT_FWD_PIN);
            GPIO_ResetBits(CONFIG_DIR_RIGHT_BWD_GPIO, 1<<CONFIG_DIR_RIGHT_BWD_PIN);
        }
        else
        {
            GPIO_SetBits(CONFIG_DIR_RIGHT_BWD_GPIO, 1<<CONFIG_DIR_RIGHT_BWD_PIN);
            GPIO_ResetBits(CONFIG_DIR_RIGHT_FWD_GPIO, 1<<CONFIG_DIR_RIGHT_FWD_PIN);
        }
    }
}
