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

#include <robots/actions2013.h>

volatile uint32_t _delay = 0;

void SysTick_Handler(void)
{
    encoders_parser(); // update encoders values

    updateCoords(encoder_getDelta(0), encoder_getDelta(1));
    move_tick();
    
    if(_delay > 0)
        _delay--;
}

void _delay_ms(uint32_t time)
{
    _delay = time / 10;
    while(_delay > 0);;;
}

extern int32_t move_getMidAcc(void);

static inline void _init_io(void)
{
    chassis_init();
    encoders_init();
    led_init();
    uart_init(3, 9600);
    uart_init(1, 57600);
    servo_init();
    sensor_init();

    __enable_irq();

    SysTick_Config(SystemCoreClock / 100); // 10 ms timer period
}

/**
 * Periphery global variables
 */
servo elevator, bigpaw, smallpaw, grip_l, grip_r;
sensor_t limiter_l, limiter_r, elevator_h, elevator_l, line_l, line_r, wall_front, wall_rear; // robot sensors
sensor_t shmorgalka, field_select, button1, button2; // user interface

static inline void _init_periph(void)
{
    // 1. Servos
    elevator = servo_add(GPIOB, 1);
    bigpaw = servo_add(GPIOC, 5);
    smallpaw = servo_add(GPIOB, 0);
    grip_l = servo_add(GPIOB, 12);
    grip_r = servo_add(GPIOB, 13);

    // 2. Sensors
    limiter_l.mode = SENSOR_ACTIVE_GND;
    limiter_l.gpio = GPIOA;
    limiter_l.pin = GPIO_Pin_15;
    sensor_add(&limiter_l);
    
    limiter_r.mode = SENSOR_ACTIVE_GND;
    limiter_r.gpio = GPIOA;
    limiter_r.pin = GPIO_Pin_14;
    sensor_add(&limiter_r);
    
    button1.mode = SENSOR_PASSIVE_GND;
    button1.gpio = GPIOA;
    button1.pin = GPIO_Pin_13;
    sensor_add(&button1);
    
    button2.mode = SENSOR_PASSIVE_GND;
    button2.gpio = GPIOA;
    button2.pin = GPIO_Pin_8;
    sensor_add(&button1);

    field_select.mode = SENSOR_PASSIVE_GND;
    field_select.gpio = GPIOC;
    field_select.pin = GPIO_Pin_8;
    sensor_add(&field_select);
    
    shmorgalka.mode = SENSOR_PASSIVE_GND;
    shmorgalka.gpio = GPIOC;
    shmorgalka.pin = GPIO_Pin_7;
    sensor_add(&shmorgalka);
    
    elevator_h.mode = SENSOR_PASSIVE_GND;
    elevator_h.gpio = GPIOC;
    elevator_h.pin = GPIO_Pin_6;
    sensor_add(&elevator_h);
    
    elevator_l.mode = SENSOR_PASSIVE_GND;
    elevator_l.gpio = GPIOC;
    elevator_l.pin = GPIO_Pin_4;
    sensor_add(&elevator_l);

    wall_front.mode = SENSOR_ACTIVE_VCC;
    wall_front.gpio = GPIOA;
    wall_front.pin = GPIO_Pin_4;
    sensor_add(&wall_front);
    
    wall_rear.mode = SENSOR_ACTIVE_VCC;
    wall_rear.gpio = GPIOA;
    wall_rear.pin = GPIO_Pin_3;
    sensor_add(&wall_rear);

    line_l.mode = SENSOR_ANALOG_THRESHOLD_HIGH;
    line_l.gpio = GPIOA;
    line_l.pin = GPIO_Pin_2;
    line_l.channel = 2;
    sensor_add(&line_l);
    
    line_r.mode = SENSOR_ANALOG_THRESHOLD_HIGH;
    line_r.gpio = GPIOC;
    line_r.pin = GPIO_Pin_3;
    line_r.channel = 13;
    sensor_add(&line_r);

    // Throw required sensors into actions list
    actions_init(bigpaw, smallpaw, elevator, grip_l, grip_r, &elevator_h, &elevator_l);
    move_initLimiters(&limiter_l, &limiter_r);
}

int main(void)
{
    // 1. Init I/O
    _init_io();

    // 2. Configuring all sensors, servos etc.
    _init_periph();

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

    while(1);;;

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

    while(1);;; // end of program
    
    return 0;
}
