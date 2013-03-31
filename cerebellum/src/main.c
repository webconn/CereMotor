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
    uart_init(1, 115200);
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

    wall_front.mode = SENSOR_ACTIVE_GND;
    wall_front.gpio = GPIOA;
    wall_front.pin = GPIO_Pin_3;
    sensor_add(&wall_front);
    
    wall_rear.mode = SENSOR_ACTIVE_GND;
    wall_rear.gpio = GPIOA;
    wall_rear.pin = GPIO_Pin_4;
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
    move_initWallSensor(&wall_front, &wall_rear);
    paw_move(BIG, OPEN);
    paw_move(SMALL, CLOSE);
    grip_set(LEFT, OPEN);
    grip_set(RIGHT, OPEN);
    elevator_move(DOWN);

    move_setMinBrakeDelta(8);
}

void tactics_red(void);
void tactics_blue(void);

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
    //led_on(3);

    // dirty-hack - disable JTAG using registers
    AFIO->MAPR &= ~(7 << 24);
    AFIO->MAPR |= (4 << 24);

    // Manual elevator manipulation
    // Try to drop elevator
    /*grip_set(LEFT, HOLD);
    grip_set(RIGHT, HOLD);
    elevator_move(UP);
    elevator_move(DOWN);
    grip_set(LEFT, UNHOLD);
    grip_set(RIGHT, UNHOLD);*/
    // Here we describe cake candles blowing algo
    
    while(1);;;
    while(sensor_read(&shmorgalka));;; // shmorgalka
    
    // Select tactics switch
    if(sensor_read(&field_select))
    {
        tactics_red();
    }
    else
    {
        tactics_blue();
    }

    while(1);;; // end of program
    
    return 0;
}

void tactics_red(void)
{

}

#define degreesToRadians(dgrs) (dgrs*3.14159/180)

void tactics_blue(void)
{
    move_refreshAngle();
    /*
    // Start
    move_line(4000, 10, mmToTicks(580));
    while(move_isBusy());

    // Rotate right 45 degrees
    move_rotate(3000, 20, -3.14159 / 4);
    while(move_isBusy());

    // Go to first glass
    move_line(1700, 10, mmToTicks(360));
    while(move_isBusy());

    // Now first glass is under our right grip
    _delay_ms(500);
    grip_set(RIGHT, HOLD);

    // Go to second glass
    // Rotate left 15 degrees
    move_rotate(3000, 20, 3.14159 / 12);
    while(move_isBusy());

    // Go forward 16 cm
    move_line(1700, 10, mmToTicks(200));
    while(move_isBusy());

    // Hold right glass
    _delay_ms(500);
    grip_set(LEFT, HOLD);

    // Ok, now both grips have glasses.
    // Vira!
    _delay_ms(500);
    elevator_move(UP);

    // Go to next glass
    // ROtate to final position -88 degrees (em... delta = -28)
    move_rotate(3000, 20, -3.14159 * 56 / 180);
    while(move_isBusy());

    // Go forward to third glass, about 14 cm
    move_line(1700, 20, mmToTicks(160));
    while(move_isBusy());

    // Unhold right grip
    _delay_ms(500);
    grip_set(RIGHT, UNHOLD);

    // Meine!
    _delay_ms(300);
    elevator_move(DOWN);

    // Hold right grip
    _delay_ms(300);
    grip_set(RIGHT, HOLD);

    // Vira!
    _delay_ms(300);
    elevator_move(UP);

    // Rotate left at about 70 degrees
    move_rotate(3000, 20, degreesToRadians(70));
    while(move_isBusy());

    // Move 18 cm forward
    move_line(1700, 10, mmToTicks(200));
    while(move_isBusy());

    // Now fourth glass is under left grip
    _delay_ms(300);
    grip_set(LEFT, UNHOLD);

    // Meine!
    _delay_ms(300);
    elevator_move(DOWN);

    // Hold left
    _delay_ms(300);
    grip_set(LEFT, HOLD);

    // Vira!
    _delay_ms(300);
    elevator_move(UP);

    while(1);;;
    */
    // Cake
    paw_move(BIG, OPEN);
    paw_move(SMALL, OPEN);
    
    int32_t pos_top = 0, pos_bottom = 0;
    int32_t flag_blow = 0;
    int32_t candles_top[] = {0, mmToTicks(2*260), mmToTicks(4*264.3), mmToTicks(6*264.3)};
    int32_t candles_bottom[] = {0, mmToTicks(3*170), mmToTicks(4*180), mmToTicks(5*190), mmToTicks(6*180), mmToTicks(7*180), mmToTicks(9*180), mmToTicks(10*180)};
    
    move_setMinBrakeDelta(3);

    // To hit the candles:
    // 1. Clear angle (by crashing the wall)
    move_refreshAngle();

    // 1.1. Go away from the wall
    move_line(4000, 20, mmToTicks(40));
    while(move_isBusy());

    // 2. Rotate CCW to about 15 degrees
    move_rotate(2000, 30, 0.261799);
    while(move_isBusy());

    // 3. Hit the first candle
    if(candles_top[0] == 0)
    {
        pos_top = 1;
        paw_move(BIG, BLOW);
        _delay_ms(500);
        paw_move(BIG, OPEN);
    }
    if(candles_bottom[0] == 0)
    {
        pos_bottom = 1;
        paw_move(SMALL, BLOW);
        _delay_ms(500);
        paw_move(SMALL, OPEN);
    }

    move_rotate(2000, 30, -0.261799);
    while(move_isBusy());

    // 4. Reset the angle again
    move_refreshAngle();
    
    // 3. Move by cake and hit the candles from the list!
    encoder_reset(0);
    encoder_reset(1);
    move_wall(2500, 20, mmToTicks(1850));
    while(move_isBusy())
    {
        int32_t aripPath = (encoder_getPath(1) + encoder_getPath(0)) / 2;
        
        if(aripPath >= candles_top[pos_top])
        {
            pos_top++;
            paw_move(BIG, BLOW);
            flag_blow = 1;
        }

        if(aripPath >= candles_bottom[pos_bottom])
        {
            pos_bottom++;
            paw_move(SMALL, BLOW);
            flag_blow = 1;
        }

        if(flag_blow)
        {
            flag_blow = 0;
            _delay_ms(500);
            paw_move(BIG, OPEN);
            paw_move(SMALL, OPEN);
        }
    }
}
