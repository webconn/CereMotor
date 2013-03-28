#ifndef ROBOT_2013PROTO_H
#define ROBOT_2013PROTO_H

/**
* @file include/robots/robot_2013proto.h
* @brief Configuration file for Eurobot 2013 Main robot prototype
* @author WebConn
* @date 10 Jan 2013
*/

/**
 * @addgroup LEDs configuraion
 * @{
 */

// LED1 config
#define CONFIG_LED1_RCC RCC_APB2Periph_GPIOA
#define CONFIG_LED1_GPIO GPIOA
#define CONFIG_LED1_PIN 5

// LED2 config
#define CONFIG_LED2_RCC RCC_APB2Periph_GPIOC
#define CONFIG_LED2_GPIO GPIOC
#define CONFIG_LED2_PIN 15

// LED3 config
#define CONFIG_LED3_RCC RCC_APB2Periph_GPIOC
#define CONFIG_LED3_GPIO GPIOC
#define CONFIG_LED3_PIN 10

/**
 * @}
 */


/**
* @addgroup Chassis configuration
* @{
*/

#define CONFIG_CHASSIS_APB1 RCC_APB1Periph_TIM4
#define CONFIG_CHASSIS_APB2 RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD

#define CONFIG_DIR_LEFT_FWD_PIN 4
#define CONFIG_DIR_LEFT_FWD_GPIO GPIOB

#define CONFIG_DIR_LEFT_BWD_PIN 5
#define CONFIG_DIR_LEFT_BWD_GPIO GPIOB

#define CONFIG_DIR_RIGHT_FWD_PIN 2
#define CONFIG_DIR_RIGHT_FWD_GPIO GPIOD

#define CONFIG_DIR_RIGHT_BWD_PIN 3
#define CONFIG_DIR_RIGHT_BWD_GPIO GPIOB

#define CONFIG_PWM_LEFT_PIN 9
#define CONFIG_PWM_LEFT_GPIO GPIOB

#define CONFIG_PWM_RIGHT_PIN 8
#define CONFIG_PWM_RIGHT_GPIO GPIOB

#define CONFIG_PWM_LEFT_TIMER TIM4
#define CONFIG_PWM_LEFT_OC 4

#define CONFIG_PWM_RIGHT_TIMER TIM4
#define CONFIG_PWM_RIGHT_OC 3

#define CONFIG_PWM_ACCURACY 8192

#define CONFIG_CHASSIS_BREAK_LEVEL_VCC
// CONFIG_CHASSIS_BREAK_LEVEL_GND is not set

/**
* @}
*/

/**
* @addgroup Encoders configuration
* @{
*/

#define CONFIG_ENC_APB1 RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2
#define CONFIG_ENC_APB2 RCC_APB2Periph_GPIOA

#define CONFIG_ENC_LEFT_A_PIN 0
#define CONFIG_ENC_LEFT_A_GPIO GPIOA

#define CONFIG_ENC_LEFT_B_PIN 1
#define CONFIG_ENC_LEFT_B_GPIO GPIOA

#define CONFIG_ENC_RIGHT_A_PIN 7
#define CONFIG_ENC_RIGHT_A_GPIO GPIOA

#define CONFIG_ENC_RIGHT_B_PIN 6
#define CONFIG_ENC_RIGHT_B_GPIO GPIOA

#define CONFIG_ENC_LEFT_TIMER TIM2
// CONFIG_ENC_LEFT_INV is not set
//#define CONFIG_ENC_LEFT_SWAP 1

#define CONFIG_ENC_RIGHT_TIMER TIM3
// CONFIG_ENC_RIGHT_INV is not set
#define CONFIG_ENC_RIGHT_SWAP 1

#define CONFIG_ENC_WHEEL_RADIUS 22.5
#define CONFIG_ENC_RESOLUTION 3000*2

/**
* @}
*/

/**
 * @addgroup Servo controller
 * @{
 */

// CONFIG_SERVO_USE_TIM1 is not set
// CONFIG_SERVO_USE_TIM2 is not set
// CONFIG_SERVO_USE_TIM3 is not set
// CONFIG_SERVO_USE_TIM4 is not set
#define CONFIG_SERVO_USE_TIM5
// CONFIG_SERVO_USE_TIM8 is not set

#define CONFIG_SERVO_TIMER_COMPARE 1
#define CONFIG_MAX_SERVO 16
#define CONFIG_SERVO_MIN_VALUE 0
#define CONFIG_SERVO_MAX_VALUE 2047
#define CONFIG_SERVO_SAFE_MIN 2047

/**
 * @}
 */

/**
 * @addgroup Sensors library
 * @{
 */

#define CONFIG_SENSORS_ADC_MAX_CHANNELS 16
#define CONFIG_SENSORS_ADC_SAMPLETIME ADC_SampleTime_41Cycles5
#define CONFIG_SENSORS_USE_ADC1
// CONFIG_SENSORS_USE_ADC3 is not set

/**
 * @}
 */

/**
* @addgroup Stabilisation config
* @{
*/

#define CONFIG_CHASSIS_RADIUS 93
#define CONFIG_STAB_ALGO_PATH
// CONFIG_STAB_ALGO_ANGLE is not set

#define CONFIG_ANGLE_COEFF 0.980334 //1.007203
#define CONFIG_PATH_RCOEFF 0.980334

/**
* @}
*/

#endif
