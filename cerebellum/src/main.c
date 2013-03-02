#include <cerebellum/chassis.h>
#include <cerebellum/encoders.h>
#include <cerebellum/led.h>
#include <cerebellum/uart.h>
#include <cerebellum/pid.h>
#include <cerebellum/deltacoords.h>
#include <cerebellum/robot.h>
#include <cerebellum/movement.h>

#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>

#include <stdio.h>
#include <math.h>

volatile uint32_t _delay = 0;

void SysTick_Handler(void)
{
    encoders_parser(); // update encoders values

    updateCoords(encoder_getDelta(0), encoder_getDelta(1));
    move_tick();
    
    if(_delay > 0)
        _delay--;
}
/*
void goto_point(int32_t x, int32_t y, int32_t pwm)
{
    // Stage 1. Get current coordinates
    int32_t cx, cy;
    cx = getX();
    cy = getY();

    // Stage 2. Get current angle
    float cangle = getAngle();

    // Stage 3. Calculate path
    uint32_t path = (uint32_t) sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
    float angle = atan2((float) (y-cy), (float) (x-cx));

    // Stage 4. Rotate robot to the new angle
    chassis_setAngle(pwm, angle);

    // Stage 5. Go to the path
    chassis_line(pwm, path);

    // Here we should insert some coordinates stabilisation
}
*/

void delay(uint32_t time)
{
    _delay = time / 10;
    while(_delay > 0);;;
}

int main(void)
{
    chassis_init();
    encoders_init();
    led_init();
    uart_init(3, 9600);
    uart_init(1, 57600);

    SysTick_Config(SystemCoreClock / 100); // 10 ms timer period

    // Configuring PID
    pidConfig cnf = {
        .p_gain = 1,
        .i_rgain = 10000,
        .d_rgain = 100,
        
        .i_max = 8191,
        .i_min = -8191
    };

    pid_config(&cnf);

    // Turning on LED - end of initialisation
    led_on();

    // dirty-hack - disable JTAG using registers
    AFIO->MAPR &= ~(7 << 24);
    AFIO->MAPR |= (4 << 24);

        move_line(6000, 5, mmToTicks(1000));
    while(move_isBusy())
    {
        //chassis_write(2048, 2048);
        //delay(100);
        //chassis_write(0, 0);
        //delay(1000);
        // Trying to move forward by new library

        printf("PWM: %06d, %06d, ENC: %06d, %06d, ST: %d\n\r", (int) move_getPWM(0), (int) move_getPWM(1), (int) encoder_getDelta(0), (int) encoder_getDelta(1), in);

        // Debugging
        
        //printf("S:(%06d,%06d) \r", (int) leftSpeed, (int) rightSpeed);
        //printf("C:(%05d, %05d) A:%f\r", (int) getX(), (int) getY(), (float) getAngle());

    }

    while(1);;; // end of program
    
    return 0;
}
