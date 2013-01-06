/**
 * @file encoders.c
 * @brief Encoders configuration and manipulation API
 * @author WebConn
 * @date 5 Jan 2012
 */

#include <cerebellum/encoders.h>

#include <stm32f10x.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#ifdef CONFIG_ROBOT_2013

    #define ENC_APB1 RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5
    #define ENC_APB2 RCC_APB2Periph_GPIOA

    #define ENC_LEFT_A_PIN GPIO_Pin_0
    #define ENC_LEFT_A_GPIO GPIOA

    #define ENC_LEFT_B_PIN GPIO_Pin_1
    #define ENC_LEFT_B_GPIO GPIOA
    
    #define ENC_RIGHT_A_PIN GPIO_Pin_7
    #define ENC_RIGHT_A_GPIO GPIOA

    #define ENC_RIGHT_B_PIN GPIO_Pin_6
    #define ENC_RIGHT_B_GPIO GPIOA

    #define ENC_LEFT_TIMER TIM5
    #define ENC_LEFT_A_IC 1
    #define ENC_LEFT_B_IC 2
    
    #define ENC_RIGHT_TIMER TIM3
    #define ENC_RIGHT_A_IC 1
    #define ENC_RIGHT_B_IC 2

#endif

volatile encState_t _leftEncoder, _rightEncoder;

void encoders_init(void)
{
    /*
     * Initialisation of encoders is
     * configuring RCC, GPIO and counters
     */



    /*
     * Configure RCC
     */
    RCC_APB1PeriphClockCmd(ENC_APB1, ENABLE);
    RCC_APB2PeriphClockCmd(ENC_APB2, ENABLE);

    /*
     * Configure GPIO as input floating
     */
    
    GPIO_InitTypeDef input = {
        .GPIO_Mode = GPIO_Mode_IN_FLOATING
    };

    input.GPIO_Pin = ENC_LEFT_A_PIN;
    GPIO_Init(ENC_LEFT_A_GPIO, &input);

    input.GPIO_Pin = ENC_LEFT_B_PIN;
    GPIO_Init(ENC_LEFT_B_GPIO, &input);

    input.GPIO_Pin = ENC_RIGHT_A_PIN;
    GPIO_Init(ENC_RIGHT_A_GPIO, &input);

    input.GPIO_Pin = ENC_RIGHT_B_PIN;
    GPIO_Init(ENC_RIGHT_B_GPIO, &input);

    /*
     * Configure timers
     * Encoder interface configured manually (by CMSIS)
     */
    
    #if ENC_LEFT_A_IC == 1
        #define ENC_LEFT_A_CCER TIM_CCER_CC1P       // depends on encoder reading polarity
        #define ENC_LEFT_A_CCMR1 TIM_CCMR1_CC1S_0   // for channels 1 and 2 only
        #define ENC_LEFT_A_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_LEFT_A_IC == 2
        #define ENC_LEFT_A_CCER TIM_CCER_CC2P       // depends on encoder reading polarity
        #define ENC_LEFT_A_CCMR1 TIM_CCMR1_CC2S_0   // for channels 1 and 2 only
        #define ENC_LEFT_A_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_LEFT_A_IC == 3
        #define ENC_LEFT_A_CCER TIM_CCER_CC3P       // depends on encoder reading polarity
        #define ENC_LEFT_A_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_LEFT_A_CCMR2 TIM_CCMR2_CC3S_0   // for channels 3 and 4 only
    #elif ENC_LEFT_A_IC == 4
        #define ENC_LEFT_A_CCER TIM_CCER_CC4P       // depends on encoder reading polarity
        #define ENC_LEFT_A_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_LEFT_A_CCMR2 TIM_CCMR2_CC4S_0   // for channels 3 and 4 only
    #else
        #error "Left encoder A channel: wrong input number (must be 1-4)"
    #endif    

    #if ENC_LEFT_B_IC == 1
        #define ENC_LEFT_B_CCER TIM_CCER_CC1P       // depends on encoder reading polarity
        #define ENC_LEFT_B_CCMR1 TIM_CCMR1_CC1S_0   // for channels 1 and 2 only
        #define ENC_LEFT_B_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_LEFT_B_IC == 2
        #define ENC_LEFT_B_CCER TIM_CCER_CC2P       // depends on encoder reading polarity
        #define ENC_LEFT_B_CCMR1 TIM_CCMR1_CC2S_0   // for channels 1 and 2 only
        #define ENC_LEFT_B_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_LEFT_B_IC == 3
        #define ENC_LEFT_B_CCER TIM_CCER_CC3P       // depends on encoder reading polarity
        #define ENC_LEFT_B_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_LEFT_B_CCMR2 TIM_CCMR2_CC3S_0   // for channels 3 and 4 only
    #elif ENC_LEFT_B_IC == 4
        #define ENC_LEFT_B_CCER TIM_CCER_CC4P       // depends on encoder reading polarity
        #define ENC_LEFT_B_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_LEFT_B_CCMR2 TIM_CCMR2_CC4S_0   // for channels 3 and 4 only
    #else
        #error "Left encoder B channel: wrong input number (must be 1-4)"
    #endif

    #if ENC_RIGHT_A_IC == 1
        #define ENC_RIGHT_A_CCER TIM_CCER_CC1P       // depends on encoder reading polarity
        #define ENC_RIGHT_A_CCMR1 TIM_CCMR1_CC1S_0   // for channels 1 and 2 only
        #define ENC_RIGHT_A_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_RIGHT_A_IC == 2
        #define ENC_RIGHT_A_CCER TIM_CCER_CC2P       // depends on encoder reading polarity
        #define ENC_RIGHT_A_CCMR1 TIM_CCMR1_CC2S_0   // for channels 1 and 2 only
        #define ENC_RIGHT_A_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_RIGHT_A_IC == 3
        #define ENC_RIGHT_A_CCER TIM_CCER_CC3P       // depends on encoder reading polarity
        #define ENC_RIGHT_A_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_RIGHT_A_CCMR2 TIM_CCMR2_CC3S_0   // for channels 3 and 4 only
    #elif ENC_RIGHT_A_IC == 4
        #define ENC_RIGHT_A_CCER TIM_CCER_CC4P       // depends on encoder reading polarity
        #define ENC_RIGHT_A_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_RIGHT_A_CCMR2 TIM_CCMR2_CC4S_0   // for channels 3 and 4 only
    #else
        #error "Right encoder A channel: wrong input number (must be 1-4)"
    #endif

    #if ENC_RIGHT_B_IC == 1
        #define ENC_RIGHT_B_CCER TIM_CCER_CC1P       // depends on encoder reading polarity
        #define ENC_RIGHT_B_CCMR1 TIM_CCMR1_CC1S_0   // for channels 1 and 2 only
        #define ENC_RIGHT_B_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_RIGHT_B_IC == 2
        #define ENC_RIGHT_B_CCER TIM_CCER_CC2P       // depends on encoder reading polarity
        #define ENC_RIGHT_B_CCMR1 TIM_CCMR1_CC2S_0   // for channels 1 and 2 only
        #define ENC_RIGHT_B_CCMR2 0                  // for channels 3 and 4 only
    #elif ENC_RIGHT_B_IC == 3
        #define ENC_RIGHT_B_CCER TIM_CCER_CC3P       // depends on encoder reading polarity
        #define ENC_RIGHT_B_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_RIGHT_B_CCMR2 TIM_CCMR2_CC3S_0   // for channels 3 and 4 only
    #elif ENC_RIGHT_B_IC == 4
        #define ENC_RIGHT_B_CCER TIM_CCER_CC4P       // depends on encoder reading polarity
        #define ENC_RIGHT_B_CCMR1 0                  // for channels 1 and 2 only
        #define ENC_RIGHT_B_CCMR2 TIM_CCMR2_CC4S_0   // for channels 3 and 4 only
    #else
        #error "Right encoder B channel: wrong input number (must be 1-4)"
    #endif
    

    // Configuring CCERs
    ENC_LEFT_TIMER->CCER &= ~(ENC_LEFT_A_CCER | ENC_LEFT_B_CCER);
    ENC_LEFT_TIMER->CCER |= (ENC_LEFT_A_CCER | ENC_LEFT_B_CCER);

    ENC_RIGHT_TIMER->CCER &= ~(ENC_RIGHT_A_CCER | ENC_RIGHT_B_CCER);
    ENC_RIGHT_TIMER->CCER |= (ENC_RIGHT_A_CCER | ENC_RIGHT_B_CCER);

    // Configuring CCMR1s
    ENC_LEFT_TIMER->CCMR1 &= ~(ENC_LEFT_A_CCMR1 | ENC_LEFT_B_CCMR1);
    ENC_LEFT_TIMER->CCMR1 |= (ENC_LEFT_A_CCMR1 | ENC_LEFT_B_CCMR1);
    
    ENC_RIGHT_TIMER->CCMR1 &= ~(ENC_RIGHT_A_CCMR1 | ENC_RIGHT_B_CCMR1);
    ENC_RIGHT_TIMER->CCMR1 |= (ENC_RIGHT_A_CCMR1 | ENC_RIGHT_B_CCMR1);

    // Configuring CCMR2s
    ENC_LEFT_TIMER->CCMR2 &= ~(ENC_LEFT_A_CCMR2 | ENC_LEFT_B_CCMR2);
    ENC_LEFT_TIMER->CCMR2 |= (ENC_LEFT_A_CCMR2 | ENC_LEFT_B_CCMR2);
    
    ENC_RIGHT_TIMER->CCMR2 &= ~(ENC_RIGHT_A_CCMR2 | ENC_RIGHT_B_CCMR2);
    ENC_RIGHT_TIMER->CCMR2 |= (ENC_RIGHT_A_CCMR2 | ENC_RIGHT_B_CCMR2);

    // Configuring SMCR
    ENC_LEFT_TIMER->SMCR &= ~(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    ENC_LEFT_TIMER->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);

    ENC_RIGHT_TIMER->SMCR &= ~(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    ENC_RIGHT_TIMER->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);

    // Configuring timer update value
    ENC_LEFT_TIMER->ARR = 65535;
    ENC_RIGHT_TIMER->ARR = 65535;

    // Configuring timer counters values to middle for greater analysing
    ENC_LEFT_TIMER->CNT = 32768;
    ENC_RIGHT_TIMER->CNT = 32768;

    // Enabling timers
    TIM_Cmd(ENC_LEFT_TIMER, ENABLE);
    TIM_Cmd(ENC_RIGHT_TIMER, ENABLE);
}

void encoders_parser(void)
{
    /*
     * Encoder parsing process:
     * - get counted value
     * - clear timers
     * - fill in encoder structures
     */

    // Get counted values
    _leftEncoder.delta = ENC_LEFT_TIMER->CNT - 32768;
    _rightEncoder.delta = ENC_RIGHT_TIMER->CNT - 32768;

    // Clear timers
    ENC_LEFT_TIMER->CNT = 32768;
    ENC_RIGHT_TIMER->CNT = 32768;

    // Update encoders structures
    _leftEncoder.value += _leftEncoder.delta;
    _rightEncoder.value += _rightEncoder.delta;

}

encState_t * encoder_get(uint8_t encoder)
{
    if(encoder) // left encoder
        return (encState_t *) &(_leftEncoder);
    else        // right encoder
        return (encState_t *) &(_rightEncoder);
}

int32_t encoder_getDelta(uint8_t encoder)
{
    if(encoder)
        return _leftEncoder.delta;
    else
        return _rightEncoder.delta;
}

int32_t encoder_getPath(uint8_t encoder)
{
    if(encoder)
        return _leftEncoder.value;
    else
        return _rightEncoder.value;
}

void encoder_reset(uint8_t encoder)
{
    if(encoder)
    {
        _leftEncoder.value = 0;
        _leftEncoder.delta = 0;
    }
    else
    {
        _rightEncoder.value = 0;
        _rightEncoder.delta = 0;
    }
}
