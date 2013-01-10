/**
 * @file encoders.c
 * @brief Encoders configuration and manipulation API
 * @author WebConn
 * @date 5 Jan 2012
 */

#include <cerebellum/encoders.h>
#include <robots/config.h>

#include <stm32f10x.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

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
    RCC_APB1PeriphClockCmd(CONFIG_ENC_APB1, ENABLE);
    RCC_APB2PeriphClockCmd(CONFIG_ENC_APB2, ENABLE);

    /*
     * Configure GPIO as input floating
     */
    
    GPIO_InitTypeDef input = {
        .GPIO_Mode = GPIO_Mode_IN_FLOATING
    };

    input.GPIO_Pin = 1<<CONFIG_ENC_LEFT_A_PIN;
    GPIO_Init(CONFIG_ENC_LEFT_A_GPIO, &input);

    input.GPIO_Pin = 1<<CONFIG_ENC_LEFT_B_PIN;
    GPIO_Init(CONFIG_ENC_LEFT_B_GPIO, &input);

    input.GPIO_Pin = 1<<CONFIG_ENC_RIGHT_A_PIN;
    GPIO_Init(CONFIG_ENC_RIGHT_A_GPIO, &input);

    input.GPIO_Pin = 1<<CONFIG_ENC_RIGHT_B_PIN;
    GPIO_Init(CONFIG_ENC_RIGHT_B_GPIO, &input);

    /*
     * Configure timers
     */
    
    #ifndef CONFIG_ENC_LEFT_INV
        TIM_EncoderInterfaceConfig(CONFIG_ENC_LEFT_TIMER, TIM_EncoderMode_TI1, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    #else
        TIM_EncoderInterfaceConfig(CONFIG_ENC_LEFT_TIMER, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    #endif

    #ifndef CONFIG_ENC_RIGHT_INV
        TIM_EncoderInterfaceConfig(CONFIG_ENC_RIGHT_TIMER, TIM_EncoderMode_TI1, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    #else
        TIM_EncoderInterfaceConfig(CONFIG_ENC_RIGHT_TIMER, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    #endif

    // Configuring timer update value
    CONFIG_ENC_LEFT_TIMER->ARR = 65535;
    CONFIG_ENC_RIGHT_TIMER->ARR = 65535;

    // Configuring timer counters values to middle for greater analysing
    CONFIG_ENC_LEFT_TIMER->CNT = 32768;
    CONFIG_ENC_RIGHT_TIMER->CNT = 32768;

    // Enabling timers
    TIM_Cmd(CONFIG_ENC_LEFT_TIMER, ENABLE);
    TIM_Cmd(CONFIG_ENC_RIGHT_TIMER, ENABLE);
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
    
    _leftEncoder.delta = CONFIG_ENC_LEFT_TIMER->CNT - 32768;
    _rightEncoder.delta = CONFIG_ENC_RIGHT_TIMER->CNT - 32768;

    // Clear timers
    CONFIG_ENC_LEFT_TIMER->CNT = 32768;
    CONFIG_ENC_RIGHT_TIMER->CNT = 32768;

    // Change direction if required
    #ifdef CONFIG_ENC_LEFT_SWAP
        _leftEncoder.delta = -_leftEncoder.delta;
    #endif

    #ifdef CONFIG_ENC_RIGHT_SWAP
        _rightEncoder.delta = -_rightEncoder.delta;
    #endif

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
