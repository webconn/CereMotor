#include <cerebellum/chassis.h>
#include <cerebellum/encoders.h>
#include <cerebellum/led.h>
#include <cerebellum/uart.h>
#include <cerebellum/pid.h>
#include <cerebellum/deltacoords.h>
#include <cerebellum/robot.h>
#include <cerebellum/movement.h>
#include <cerebellum/sensors.h>
#include <cerebellum/servo.h>

#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <stdio.h>
#include <math.h>

#define LEFT 1
#define RIGHT 0

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

void _delay_ms(uint32_t time)
{
    _delay = time / 10;
    while(_delay > 0);;;
}

extern int32_t move_getMidAcc(void);

int main(void)
{
    chassis_init();
    encoders_init();
    led_init();
    uart_init(3, 9600);
    uart_init(1, 57600);
    servo_init();
    sensor_init();

    __enable_irq();

    // Init ADC10 on PC0
    /*sensor_t adc = {
        .gpio = GPIOC,
        .pin = 1,
        .mode = SENSOR_ANALOG,
        .channel = 10
    };

    sensor_t adc2 = {
        .gpio = GPIOA,
        .pin = (1<<1),
        .mode = SENSOR_ANALOG,
        .channel = 1
    };

    sensor_add(&adc2);
    sensor_add(&adc);*/

    SysTick_Config(SystemCoreClock / 100); // 10 ms timer period

    // Configuring PID
    pidConfig cnf = {
        .p_gain = 2,
        .i_rgain = 1000,
        .d_rgain = 10,
        
        .i_max = 8191,
        .i_min = -8191
    };

    pid_config(&cnf);

    // Turning on LED - end of initialisation
    led_on(1);
    led_on(2);
    led_on(3);

    // dirty-hack - disable JTAG using registers
    AFIO->MAPR &= ~(7 << 24);
    AFIO->MAPR |= (4 << 24);

    servo first = servo_add(GPIOB, 0);
    servo second = servo_add(GPIOC, 5);

    // In infinite loop - rotate servo 1 and 2
    while(1)
    {
        int i;
        for(i=450; i<=1300; i+=2)
        {
            servo_write(first, i);
            servo_write(second, i);
            delay(10);
        }
        for(i=1300; i>=450; i-=2)
        {
            servo_write(first, i);
            servo_write(second, i);
            delay(10);
        }

    }

    // Check ADC
/*    while(1)
    {
        if(sensor_read(&adc) > sensor_read(&adc2))
            led_on(1);
        else
            led_off(1);
    }*/

    //int i;
    /*
    for(i=0; i<4; i++)
    {
        move_line(6000, 10, mmToTicks(500));
        while(move_isBusy());
        move_rotate(2000, 30, 3.14159/2);
        while(move_isBusy());
    }
    */
    //printf("Required: %06d\n\r\n", (int) (2 * 3.14159 * getChassisRadius()));
    /*while(minBrake > 0)
    {    
        // 1. Set MinBrakeDelta
        move_setMinBrakeDelta(1);

        // 2. Try to make full revolution
        move_rotate(1000, 6, -3.14159);
        
        while(move_isBusy())
        {
        }// waiting for complete of action

        // 3. Wait while robot moving
        while(encoder_getDelta(1) > 0);
        while(encoder_getDelta(0) > 0);

        // 4.1. Send current angle in UART
        printf("%f, (%06d, %06d)\n\r", getAngle(), (int) getX(), (int) getY());

        // 4. Measure brakepath
        int32_t brakePath = encoder_getPath(0) - move_getBrakePath();

        // 5. Check brakepath for minimal luft
        if(brakePath <= mmToTicks(3))
        {
           // printf("MBL=%02d, SUCCESS! (BP=%06d, MBP=%06d)\n\r", (int) minBrake, (int) brakePath, (int) move_getBrakePath());
        }
        else
        {
           // printf("MBL=%02d, FAIL!    (BP=%06d, MBP=%06d)\n\r", (int) minBrake, (int) brakePath, (int) move_getBrakePath());
        }

        minBrake--;
    }*/

    while(1)
    {
        printf("A: %f; PATH: (%06d, %06d)\r\r", getAngle(), (int) encoder_getPath(LEFT), (int) encoder_getPath(RIGHT));
    }

    while(1);;; // end of program
    
    return 0;
}
