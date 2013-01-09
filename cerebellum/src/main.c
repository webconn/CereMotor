#include <cerebellum/chassis.h>
#include <cerebellum/encoders.h>
#include <cerebellum/led.h>
#include <cerebellum/uart.h>
#include <cerebellum/pid.h>

#include <stm32f10x.h>
#include <stm32f10x_usart.h>

#include <stdio.h>

volatile uint32_t _running = 0;
volatile int32_t _pwm = 0;
volatile int32_t _radius = 0;

void SysTick_Handler(void)
{
    encoders_parser(); // update encoders values

    int32_t leftSpeed, rightSpeed, error, leftPWM, rightPWM;
    leftSpeed = encoder_getPath(1);
    rightSpeed = encoder_getPath(0);

    // PID stabilisation algo
    if(_running == 1) // line
    {
        error = calculateLinError(leftSpeed, rightSpeed);
        updatePID(error, _pwm, &leftPWM, &rightPWM);
        chassis_write(leftPWM, rightPWM);
    }
    else if(_running == 2) // radius
    {
        error = calculateRadError(leftSpeed, rightSpeed, _radius);
        updatePID(error, _pwm, &leftPWM, &rightPWM);
        chassis_write(leftPWM, rightPWM);
    }
    else
    {
     //   chassis_write(0, 0);
    }
}

void chassis_line(int32_t pwm)
{
    resetPID();
    encoder_reset(1);
    encoder_reset(0);
    _running = 1;
    _pwm = pwm;
}

void chassis_rad(int32_t pwm, int32_t radius)
{
    resetPID();
    _running = 2;
    _pwm = pwm;
    _radius = radius;
}

void chassis_stop(void)
{
    _running = 0;
}

void _delay()
{
    int i;
    for(i=0; i<9999; i++)
        asm volatile("nop");
}

int main(void)
{
    chassis_init();
    encoders_init();
    led_init();
    uart_init(1, 57600);

    SysTick_Config(SystemCoreClock / 100); // 10 ms timer period

    // Configuring PID
    pidConfig cnf = {
        .p_gain = 2,
        .i_rgain = 10000,
        .d_rgain = 100,
        
        .i_max = 8191,
        .i_min = -8191
    };

    configPID(&cnf);

    int spd = 4096, stab = 1;

            led_on();
    while(1)
    {
        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        {
            uint32_t ch = USART_ReceiveData(USART1);
            chassis_stop();

            switch(ch)
            {
                case 'w':
                    if(stab)
                        chassis_line(spd);
                    else
                        chassis_write(spd, spd);
                    break;
                case 'd':
                    chassis_write(-spd, spd);
                    break;
                case 's':
                    if(stab)
                        chassis_line(-spd);
                    else
                        chassis_write(-spd, -spd);
                    break;
                case 'a':
                    chassis_write(spd, -spd);
                    break;
                case 'z':
                    chassis_write(0, 0);
                    break;
                case 'm':
                    if(stab)
                    {
                        led_off();
                        stab = 0;
                    }
                    else
                    {
                        stab = 1;
                        led_on();
                    }
                    break;
                case '1':
                    spd = 1000;
                    break;
                case '2':
                    spd = 2500;
                    break;
                case '3':
                    spd = 5000;
                    break;
                case '4':
                    spd = 8000;
                    break;
            }
        }
    }
    
    return 0;
}
