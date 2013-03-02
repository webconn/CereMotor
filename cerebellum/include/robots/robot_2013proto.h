#ifndef ROBOT_2013PROTO_H
#define ROBOT_2013PROTO_H

/**
 * @file include/robots/robot_2013proto.h
 * @brief Configuration file for Eurobot 2013 Main robot prototype
 * @author WebConn
 * @date 10 Jan 2013
 */

/**
 * @addgroup Chassis configuration
 * @{
 */

#define CONFIG_CHASSIS_APB1 RCC_APB1Periph_TIM4
#define CONFIG_CHASSIS_APB2 RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD

#define CONFIG_DIR_LEFT_FWD_PIN 4
#define CONFIG_DIR_LEFT_FWD_GPIO GPIOB

#define CONFIG_DIR_LEFT_BWD_PIN 3
#define CONFIG_DIR_LEFT_BWD_GPIO GPIOB

#define CONFIG_DIR_RIGHT_FWD_PIN 2
#define CONFIG_DIR_RIGHT_FWD_GPIO GPIOD

#define CONFIG_DIR_RIGHT_BWD_PIN 5
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
// CONFIG_ENC_LEFT_SWAP is not set

#define CONFIG_ENC_RIGHT_TIMER TIM3
// CONFIG_ENC_RIGHT_INV is not set
#define CONFIG_ENC_RIGHT_SWAP

#define CONFIG_ENC_WHEEL_RADIUS 24.5
#define CONFIG_ENC_RESOLUTION 3000

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

/**
 * @}
 */

#endif
