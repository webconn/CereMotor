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
#include <cerebellum/odetect.h>
#include <cerebellum/uartgrab.h>

#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <stdio.h>
#include <math.h>

#define LEFT 1
#define RIGHT 0

#define COLOR_RED 1
#define COLOR_BLUE 2

#include <robots/actions2013.h>

void manualCtl(void);
volatile uint32_t _delay = 0;
volatile uint32_t time = 0;
volatile uint32_t starter = 0;
volatile uint8_t flag_lock = 0;

// Global PID configuration

// Direction corrector
pid_regulator_t chassis_pid = {
    .p_gain = 800,
    .i_rgain = 100000,
    .d_rgain = 5,
    
    .i_max = 10000,
    .i_min = -10000,

    .i_mem = 0,
    .d_mem = 0
};

// Speed corrector
pid_regulator_t speed_pid = {
    .p_gain = 1100,
    .i_rgain = 5000,
    .d_rgain = 10,
    
    .i_max = 10000,
    .i_min = -10000,

    .i_mem = 0,
    .d_mem = 0
};

/**
 * Periphery global variables
 */
servo elevator, bigpaw, smallpaw, grip_l, grip_r;
sensor_t limiter_l, limiter_r, elevator_h, elevator_l, line_l, line_r, wall_front, wall_rear; // robot sensors
sensor_t shmorgalka, field_select, button1, button2; // user interface

void SysTick_Handler(void)
{
    led_on(2);
    encoders_parser(); // update encoders values
    coords_update(encoder_getDelta(0), encoder_getDelta(1));
    
    if(flag_lock > 0 && flag_lock < 10)
        flag_lock++;
    // Check ODetect states
    if(odetect_getDirection(0/*move_getRelativeDirection()*/))
    {
        flag_lock = 1;
        //move_pause();
        led_on(3);
    }
    else if(flag_lock == 10)
    {
        flag_lock = 0;
        //move_continue();
        led_off(3);
    }

    if(time < 9000)
        __move_correct();

    
    if(_delay > 0)
        _delay--;

    if(starter == 1)
    {
        time++;
    }
    if(time == 9000) // stop moving and inflate the baloon
    {
        chassis_break(8192, 8192);
        GPIO_SetBits(GPIOC, GPIO_Pin_2);
    }
    if(time == 9700) // baloon ready
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);
        //manualCtl();
        //while(1);;; // end of battle
    }
    led_off(2);

    // Test for ColorDetector
    uint8_t * data = uartgrab_get(1);

    // 1. Get start zone color
    if(sensor_read(&field_select)) // red
    {
        // Scan top
        if((data[0] & 0b1100) == COLOR_RED << 2)
        {
            paw_move(BIG, OPEN);
        }
        else
        {
            paw_move(BIG, BLOW);
        }

        // Scan bottom
        if((data[0] & 3) == COLOR_RED)
        {
            paw_move(SMALL, OPEN);
        }
        else
        {
            paw_move(SMALL, BLOW);
        }
    }
    else // blue
    {
        // Scan top
        if((data[0] & (3 << 2)) == COLOR_BLUE << 2)
        {
            paw_move(BIG, OPEN);
        }
        else
        {
            paw_move(BIG, BLOW);
        }

        // Scan bottom
        if((data[0] & 3) == COLOR_BLUE)
        {
            paw_move(SMALL, OPEN);
        }
        else
        {
            paw_move(SMALL, BLOW);
        }
    }
}

void _delay_ms(uint32_t time)
{
    _delay = time / 10;
    while(_delay > 0);;;
}

static inline void _init_io(void)
{
    chassis_init();
    encoders_init();
    led_init();
    uart_init(3, 9600);
    servo_init();
    sensor_init();
    odetect_init();

    // Init ColorDetector (at UART1)
    uartgrab_init(1, 115200, 1); // UART1 at 115200 receives 1 byte

    // dirty-hack - disable JTAG using registers
    AFIO->MAPR &= ~(7 << 24);
    AFIO->MAPR |= (4 << 24);

    __enable_irq();
    // Camera lighting at PB15
    GPIO_InitTypeDef light = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Pin = GPIO_Pin_15
    };
    GPIO_Init(GPIOB, &light);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);

    SysTick_Config(SystemCoreClock / 100); // 10 ms timer period
}

static inline void _init_periph(void)
{
    // 0. Inflater
    GPIO_InitTypeDef inflater = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Pin = GPIO_Pin_2
    };
    GPIO_Init(GPIOC, &inflater);

    // 1. Servos
    elevator = servo_add(GPIOB, 1);
    bigpaw = servo_add(GPIOC, 5);
    smallpaw = servo_add(GPIOB, 0);
    grip_l = servo_add(GPIOB, 12);
    grip_r = servo_add(GPIOB, 13);

    servo_write(elevator, 894);

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

    wall_front.mode = SENSOR_ANALOG;
    wall_front.gpio = GPIOA;
    wall_front.pin = GPIO_Pin_3;
    wall_front.channel = 3;
    sensor_add(&wall_front);
    
    wall_rear.mode = SENSOR_ANALOG;
    wall_rear.gpio = GPIOA;
    wall_rear.pin = GPIO_Pin_4;
    wall_rear.channel = 4;
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


    // Configure movement stabilisation
    // Send pointers to PID regulators
    move_setPathPID(&chassis_pid);
    move_setSpeedPID(&speed_pid);

    //move_setMinBrakeDelta(8);
    //move_stop();

    // Throw required sensors into actions list
    actions_init(bigpaw, smallpaw, elevator, grip_l, grip_r, &elevator_h, &elevator_l);
    //move_initLimiters(&limiter_l, &limiter_r);
    //move_initWallSensor(&wall_front, &wall_rear);
    paw_move(BIG, CLOSE);
    paw_move(SMALL, CLOSE);
    grip_set(LEFT, OPEN);
    grip_set(RIGHT, OPEN);
}

void tactics_red(void);
void tactics_blue(void);

int main(void)
{
    // 1. Init I/O
    _init_io();

    // 2. Configuring all sensors, servos etc.
    _init_periph();

    // 3. Configuring chassis PID regulators

    // Turning on LED - end of initialisation
    led_on(1);
    //led_on(2);
    //led_on(3);

    while(sensor_read(&shmorgalka));;; // shmorgalka
    starter = 1;
    //move_saveSwitch(DISABLE);
    
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

void sendInfo(void)
{
    //printf("M:%d;P:%05ld,%05ld;E:%05ld,%05ld;A:%06f;C:%05ld,%05ld\n\r", move_isBusy(), move_getPWM(0), move_getPWM(1), encoder_getPath(1), encoder_getPath(0), getAngle(), getX(), getY());
    uart_send(1, '\n');
}

void tactics_red(void)
{
    // To start to move:
    // 1. Set calculators
    move_setSpeedValueCalculator(&__move_speedCalculator_acc);
    move_setSpeedErrorCalculator(&__move_speedErrorCalculator);
    move_setPathErrorCalculator(&__move_angleErrorCalculator);
    move_setStopCalculator(&__move_stopCalculator_basic);
    move_setStopCoefficient(16);

    // 2. Set PIDs
    move_setPathPID(&chassis_pid);
    move_setSpeedPID(&speed_pid);

    // 3. Set required speeds and path if required
    move_setRqSpeed(150);
    move_setRqAcceleration(3);
    move_setRqAccelerationDivider(1);
    move_setRqPath(mmToTicks(500));

    // 4. Apply values for algos
    move_applyUserValues();

    // Now we should move
    _delay_ms(10000);

    move_setPathErrorCalculator(&__move_pathErrorCalculator_stay);
}

void tactics_blue(void)
{

}
/*
void tactics_red(void)
{
    paw_move(BIG, OPEN);
    grip_set(LEFT, HOLD);
    move_wall(1500, 15, mmToTicks(1000));
    while(move_isBusy());
    move_wall(1300, 20, -mmToTicks(1000));
    while(move_isBusy());
}

void tactics_blue(void)
{
    // 1. Clear angle
    move_refreshAngle();

    // Refresh our correct position
    updateX(mmToTicks(80));
    updateY(mmToTicks(1400));
    
    // 2. Collecting glasses
    // 2.1. Go away from the start zone
    move_line(2000, 10, mmToTicks(30));
    while(move_isBusy());

    move_rotateAbsolute(1500, 20, -degreesToRadians(20));
    while(move_isBusy());

    // Collect 1st and 2nd glasses (to 1st floor)
    move_line(2500, 20, mmToTicks(1050));
    while(move_isBusy());
    _delay_ms(100);
    grip_set(LEFT, HOLD);
    grip_set(RIGHT, HOLD);
    _delay_ms(100);

    // Collect 3rd glass
    move_rotateAbsolute(2000, 20, -degreesToRadians(52));
    elevator_move(UP);
    while(move_isBusy());

    move_line(2000, 15, mmToTicks(250));
    while(move_isBusy());
    _delay_ms(100);
    take_glass(LEFT, DOWN);
    _delay_ms(100);

    // Collect 4th glass
    move_rotateAbsolute(2000, 30, -degreesToRadians(115));
    elevator_move(UP);
    while(move_isBusy());

    move_line(2000, 15, mmToTicks(190));
    while(move_isBusy());
    _delay_ms(100);
    take_glass(LEFT, DOWN);
    _delay_ms(100);

    // Collect 5th glass
    move_rotateAbsolute(2000, 30, -degreesToRadians(215));
    elevator_move(UP);
    while(move_isBusy());

    move_line(2000, 15, mmToTicks(140));
    while(move_isBusy());
    _delay_ms(100);
    take_glass(RIGHT, DOWN);
    _delay_ms(100);

    // Collect 6th glass
    move_rotateAbsolute(2000, 30, -degreesToRadians(155));
    elevator_move(UP);
    while(move_isBusy());

    move_line(2000, 15, mmToTicks(140));
    while(move_isBusy());
    _delay_ms(100);
    take_glass(LEFT, DOWN);
    _delay_ms(100);

    // Go to the base
    move_rotateAbsolute(2000, 30, -degreesToRadians(180));
    while(move_isBusy());

    move_line(2000, 15, mmToTicks(500));
    while(move_isBusy());

    grip_set(LEFT, OPEN);
    grip_set(RIGHT, OPEN);
}
*/
/*
void tactics_red(void)
{
    _delay_ms(5000); // while younger brother starts

    // Launching from 1st zone
    // 0. Clear angle
    move_refreshAngle();
    
    // Set coords
    updateX(mmToTicks(2920));
    updateY(mmToTicks(250));
    updateAngle(degreesToRadians(180));

    // 1. Move forward 10 cm

    move_line(2000, 10, mmToTicks(130));
    while(move_isBusy()) sendInfo();
    _delay_ms(300);

    move_rotateAbsolute(2000, 30, degreesToRadians(180));
    while(move_isBusy()) sendInfo();

    // Rotate 15 degrees CCW
    move_rotate(2000, 30, -degreesToRadians(19));
    while(move_isBusy()) sendInfo();

    // Forward 100 cm (we will take two glasses in non-stop mode)
    move_line(2500, 10, mmToTicks(1000));
    while(move_isBusy())
    {
        sendInfo();
    }

    grip_set(RIGHT, HOLD);
    grip_set(LEFT, HOLD);
    _delay_ms(300);
    
    // Now we have one glass in left and one in right

    // Rotate CCW 30 degrees
    // And set elevator to up
    move_rotate(2000, 30, -degreesToRadians(38));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Go forward 27 cm
    move_line(2000, 10, mmToTicks(270));
    while(move_isBusy()) sendInfo();

    // Grip at right/
    take_glass(LEFT, DOWN);

    // Rotate 75 degrees CCW and move up an elevator
    move_rotate(1200, 10, -degreesToRadians(68));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Go forward 15 cm
    move_line(2000, 10, mmToTicks(180));
    encoder_reset(0);
    while(move_isBusy()) sendInfo();

    // Hold at the right grip
    take_glass(LEFT, DOWN);

    // Rotate CCW 90 degrees
    move_rotate(1200, 10, -degreesToRadians(92));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Go forward 13 cm
    move_line(2000, 10, mmToTicks(150));
    while(move_isBusy()) sendInfo();

    // Take glass into the left grip
    take_glass(RIGHT, DOWN);

    // Rotate 115 degrees CW
    move_rotate(1200, 10, degreesToRadians(100));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Forward 15 cm
    move_line(2000, 10, mmToTicks(240));
    while(move_isBusy()) sendInfo();

    // Unhold left grip
    take_glass(RIGHT, DOWN);

    // Now go to the base
    move_rotateAbsolute(2000, 30, 0);
    while(move_isBusy()) sendInfo();

    move_line(4000, 15, mmToTicks(600));
    while(move_isBusy()) sendInfo();

    _delay_ms(300);

    grip_set(RIGHT, OPEN);
    grip_set(LEFT, OPEN);

    // Go to the ekac!
    // Go to point (1500, 800)
    paw_move(BIG, OPEN);

    odetect_switch(DISABLE);

    move_line(2000, 15, -mmToTicks(450));
    while(move_isBusy());

    move_rotateAbsolute(2000, 30, -degreesToRadians(19));
    while(move_isBusy());

    odetect_switch(ENABLE);

    move_line(5000, 20, -mmToTicks(950));
    while(move_isBusy());

    move_line(2000, 20, -mmToTicks(400));
    while(move_isBusy())
    {
        if(sensor_read(&wall_rear))
            break;
    }
    
    move_wall(3000, 20, -mmToTicks(1500));
    while(move_isBusy())
    {
        if(sensor_read(&limiter_r) || sensor_read(&limiter_l))
            move_stop();
    }

    int32_t pos_top = 0, pos_bottom = 0;
    int32_t flag_blow = 0;
    int32_t candles_top[] = {mmToTicks(260), mmToTicks(3*260), mmToTicks(5*264.3), mmToTicks(1840)};
    int32_t candles_bottom[] = {mmToTicks(175), mmToTicks(2*180), mmToTicks(4*180), mmToTicks(5*190), mmToTicks(6*180), mmToTicks(7*180), mmToTicks(8*180), mmToTicks(11*175)};


    // To hit the candles:
    // 1. Clear angle (by crashing the wall)
    move_refreshAngle();

    // 3. Move by cake and hit the candles from the list!
    move_wall(2000, 100, mmToTicks(1850));
    while(move_isBusy())
    {
        sendInfo();
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

    // Cake is ready. Now go home
    // First, run around the cake
    move_wall(2500, 20, -mmToTicks(850));
    while(move_isBusy()) sendInfo();

}

void tactics_blue(void)
{
    //_delay_ms(5000); // while younger brother starts

    // Launching from 1st zone
    // 0. Clear angle
    move_refreshAngle();

    move_line(2000, 20, mmToTicks(500));
    while(move_isBusy()) sendInfo();
    while(1);;;
    // Set coords
    updateX(mmToTicks(80));
    updateY(mmToTicks(1400));

    // 1. Move forward 10 cm
    move_line(3000, 15, mmToTicks(100));
    while(move_isBusy()) sendInfo();

    // Rotate 15 degrees CCW
    move_rotate(2000, 30, degreesToRadians(19));
    while(move_isBusy()) sendInfo();

    // Forward 100 cm (we will take two glasses in non-stop mode)
    move_line(2500, 10, mmToTicks(1000));
    while(move_isBusy())
    {
        sendInfo();
        //if((encoder_getPath(0) + encoder_getPath(1)) >= mmToTicks(1400)) // if we reached 1st glass
            //grip_set(LEFT, HOLD);
    }

    grip_set(LEFT, HOLD);
    grip_set(RIGHT, HOLD);
    _delay_ms(300);
    
    // Now we have one glass in left and one in right

    // Rotate CCW 30 degrees
    // And set elevator to up
    move_rotate(2000, 30, degreesToRadians(35));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Go forward 27 cm
    move_line(2000, 10, mmToTicks(270));
    while(move_isBusy()) sendInfo();

    // Grip at right
    take_glass(RIGHT, DOWN);

    // Rotate 75 degrees CCW and move up an elevator
    move_rotate(1200, 10, degreesToRadians(68));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Go forward 15 cm
    move_line(2000, 10, mmToTicks(180));
    encoder_reset(0);
    while(move_isBusy()) sendInfo();

    // Hold at the right grip
    grip_set(RIGHT, UNHOLD);
    _delay_ms(300);
    elevator_move(DOWN);
    _delay_ms(300);
    grip_set(LEFT, UNHOLD);
    _delay_ms(500);
    grip_set(LEFT, HOLD);
    grip_set(RIGHT, HOLD);
    _delay_ms(300);

    // Rotate CCW 90 degrees
    move_rotate(1200, 10, degreesToRadians(96));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Go forward 13 cm
    move_line(2000, 10, mmToTicks(150));
    while(move_isBusy()) sendInfo();

    // Take glass into the left grip
    grip_set(LEFT, UNHOLD);
    _delay_ms(300);
    elevator_move(DOWN);
    _delay_ms(300);
    grip_set(RIGHT,UNHOLD);
    _delay_ms(500);
    grip_set(RIGHT, HOLD);
    grip_set(LEFT, HOLD);
    _delay_ms(300);

    // Rotate 115 degrees CW
    move_rotate(1200, 10, degreesToRadians(-93));
    elevator_move(UP);
    while(move_isBusy()) sendInfo();

    // Forward 15 cm
    move_line(2000, 10, mmToTicks(170));
    while(move_isBusy()) sendInfo();

    // Unhold left grip
    grip_set(LEFT, UNHOLD);
    _delay_ms(300);
    elevator_move(DOWN);
    _delay_ms(300);
    grip_set(RIGHT, UNHOLD);
    _delay_ms(500);
    grip_set(RIGHT, HOLD);
    grip_set(LEFT, HOLD);
    _delay_ms(300);

    // Now go to the base
    move_rotate(1500, 20, degreesToRadians(70));
    while(move_isBusy()) sendInfo();

    move_line(4000, 15, mmToTicks(650));
    while(move_isBusy()) sendInfo();

    _delay_ms(300);

    grip_set(LEFT, OPEN);
    grip_set(RIGHT, OPEN);

    _delay_ms(300);

    // Glasses are collected. Now go to the cake
    move_line(4000, 20, -mmToTicks(750));
    while(move_isBusy()) sendInfo();

    move_rotate(3000, 30, degreesToRadians(90));
    while(move_isBusy()) sendInfo();

    move_line(3000, 20, -mmToTicks(900));
    while(move_isBusy())
    {
        sendInfo();
        // Detect wall
        if(sensor_read(&wall_rear))
        {
            move_stop();
        }

        // When ride 30 cm, open big paw
        if(encoder_getPath(0) + encoder_getPath(1) >= mmToTicks(400))
        {
            paw_move(BIG, OPEN);
        }
    }

    move_wall(2500, 20, -mmToTicks(400));
    while(move_isBusy())
    {
        sendInfo();
        // detect rear wall
        if(sensor_read(&limiter_l) || sensor_read(&limiter_r))
        {
            move_stop();
        }
    }

    move_refreshAngle();

    // Cake tactics!

    paw_move(BIG, OPEN);
    paw_move(SMALL, OPEN);
    
    int32_t pos_top = 0, pos_bottom = 0;
    int32_t flag_blow = 0;
    int32_t candles_top[] = {0, mmToTicks(2*260), mmToTicks(4*264.3), mmToTicks(6*264.3)};
    int32_t candles_bottom[] = {10, mmToTicks(3*175), mmToTicks(4*180), mmToTicks(5*190), mmToTicks(6*180), mmToTicks(7*180), mmToTicks(9*180), mmToTicks(10*183)};

    // To hit the candles:
    // 1. Clear angle (by crashing the wall)
    move_refreshAngle();

    // 1.1. Go away from the wall
    move_line(4000, 20, mmToTicks(50));
    while(move_isBusy()) sendInfo();

    // 2. Rotate CCW to about 15 degrees
    move_rotate(2000, 30, degreesToRadians(18));
    while(move_isBusy()) sendInfo();

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
    while(move_isBusy()) sendInfo();

    // 4. Reset the angle again
    move_refreshAngle();
    
    // 3. Move by cake and hit the candles from the list!
    encoder_reset(0);
    encoder_reset(1);
    move_wall(2000, 100, mmToTicks(1850));
    while(move_isBusy())
    {
        sendInfo();
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

    // Cake is ready. Now go home
    // First, run around the cake
    move_wall(2500, 20, -mmToTicks(850));
    while(move_isBusy()) sendInfo();

    // Now go to home coordinate
    // Correct angle
    move_rotate(3000, 20, -getAngle()); // to get 0 angle
    while(move_isBusy()) sendInfo();

    // Go back
    move_line(8192, 10, -mmToTicks(1300));
    while(move_isBusy()) sendInfo();
}

void tactics_blue_alt(void)
{
    move_refreshAngle();

    // Set correct coordinates
    updateX(0);
    updateY(mmToTicks(1400));
    
    // Start
    move_line(1500, 10, mmToTicks(580));
    while(move_isBusy()) sendInfo();

    // Rotate right 45 degrees
    move_rotate(3000, 20, -3.14159 / 4);
    while(move_isBusy()) sendInfo();

    // Go to first glass
    move_line(1500, 10, mmToTicks(360));
    while(move_isBusy()) sendInfo();

    // Now first glass is under our right grip
    _delay_ms(500);
    grip_set(RIGHT, HOLD);

    // Go to second glass
    // Rotate left 15 degrees
    move_rotate(3000, 20, 3.14159 / 12);
    while(move_isBusy()) sendInfo();

    // Go forward 16 cm
    move_line(1500, 10, mmToTicks(200));
    while(move_isBusy()) sendInfo();

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
    while(move_isBusy()) sendInfo();

    // Go forward to third glass, about 14 cm
    move_line(1500, 20, mmToTicks(160));
    while(move_isBusy()) sendInfo();

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
    move_rotate(3000, 20, degreesToRadians(60));
    while(move_isBusy()) sendInfo();

    // Move 18 cm forward
    move_line(1500, 10, mmToTicks(200));
    while(move_isBusy()) sendInfo();

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

    // Go to 5th glass
    // Rotate 90 degrees CW
    move_rotate(3000, 20, degreesToRadians(-90));
    while(move_isBusy()) sendInfo();

    // Go forward 20 cm
    move_line(2000, 10, mmToTicks(200));
    while(move_isBusy()) sendInfo();

    // Now glass is under the left grip
    // Unhold left grip
    grip_set(LEFT, UNHOLD);
    _delay_ms(300);

    // Meine!
    elevator_move(DOWN);
    _delay_ms(300);

    // Hold left grip
    grip_set(LEFT, HOLD);
    _delay_ms(300);

    // Vira!
    elevator_move(UP);
    _delay_ms(300);

    // Go to the last glass
    // CW about 70 degrees
    move_rotate(3000, 30, degreesToRadians(-40));
    while(move_isBusy()) sendInfo();

    // Forward about 20 cm
    move_line(2000, 10, mmToTicks(200));
    while(move_isBusy()) sendInfo();

    // Now glass is under right grip
    // Unhold right grip
    grip_set(RIGHT, UNHOLD);
    _delay_ms(300);

    // Meine!
    elevator_move(DOWN);
    _delay_ms(300);

    // Hold right grip
    grip_set(RIGHT, HOLD);
    _delay_ms(300);

    // Vira! All glasses are collected
    elevator_move(UP);
    _delay_ms(300);

    // Now go to the base to store glasses there
    // 1. Correct angle
    move_rotate(3000, 20, - getAngle() - 3.14159); // to get 0 angle
    while(move_isBusy()) sendInfo();

    // 2. Go to wall
    move_line(6000, 15, mmToTicks(650));
    while(move_isBusy()) sendInfo();

    // Move glasses down
    elevator_move(DOWN);
    _delay_ms(300);
    grip_set(LEFT, OPEN);
    grip_set(RIGHT, OPEN);
    _delay_ms(300);

    // Now go to the cake
    move_line(4000, 15, -mmToTicks(550));
    while(move_isBusy()) sendInfo();

    move_rotate(2000, 30, degreesToRadians(90));
    while(move_isBusy()) sendInfo();

   
    // Now we are ready to go to the cake
    // Rotate CCW 70 degrees
    move_line(4000, 15, -mmToTicks(1200));
    while(move_isBusy())
    {
        sendInfo();
        // Detect the wall by rear sensor
        if(sensor_read(&wall_rear))
        {
            move_stop();
        }
    }


    // Now we are near the cake
    // Go by wall while we are not near the real wall (sensed by limiters)
    move_wall(1500, 20, -mmToTicks(200));
    while(move_isBusy())
    {
        sendInfo();
        // Sense the limiters
        if(sensor_read(&limiter_l) || sensor_read(&limiter_r))
        {
            move_stop();
        }
    }

    // Cake
    paw_move(BIG, OPEN);
    paw_move(SMALL, OPEN);
    
    int32_t pos_top = 0, pos_bottom = 0;
    int32_t flag_blow = 0;
    int32_t candles_top[] = {0, mmToTicks(2*260), mmToTicks(4*264.3), mmToTicks(6*264.3)};
    int32_t candles_bottom[] = {0, mmToTicks(3*170), mmToTicks(4*180), mmToTicks(5*190), mmToTicks(6*180), mmToTicks(7*180), mmToTicks(9*180), mmToTicks(10*180)};

    // To hit the candles:
    // 1. Clear angle (by crashing the wall)
    move_refreshAngle();

    // 1.1. Go away from the wall
    move_line(4000, 20, mmToTicks(40));
    while(move_isBusy()) sendInfo();

    // 2. Rotate CCW to about 15 degrees
    move_rotate(2000, 30, 0.261799);
    while(move_isBusy()) sendInfo();

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
    while(move_isBusy()) sendInfo();

    // 4. Reset the angle again
    move_refreshAngle();
    
    // 3. Move by cake and hit the candles from the list!
    encoder_reset(0);
    encoder_reset(1);
    move_wall(2500, 20, mmToTicks(1850));
    while(move_isBusy())
    {
        sendInfo();
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

    // Cake is ready. Now go home
    // First, run around the cake
    move_wall(2500, 20, -mmToTicks(850));
    while(move_isBusy()) sendInfo();

    // Now go to home coordinate
    // Correct angle
    move_rotate(3000, 20, -getAngle()); // to get 0 angle
    while(move_isBusy()) sendInfo();

    // Go back
    move_line(8192, 10, -mmToTicks(1300));
    while(move_isBusy()) sendInfo();
    // That's all, guys!
}
*/
void manualCtl(void)
{
    //move_free();
    while(1)
    {
        uint8_t byte = uart_read(1);

        switch(byte)
        {
            case 'w':
                chassis_write(2000, 2000);
                break;
            case 's':
                chassis_write(-2000, -2000);
                break;
            case 'a':
                chassis_write(-1000, 1000);
                break;
            case 'd':
                chassis_write(1000, -1000);
                break;
            case ' ':
                chassis_write(0, 0);
                break;
            case 'i':
                GPIO_SetBits(GPIOC, GPIO_Pin_2);
                _delay_ms(7000);
                GPIO_ResetBits(GPIOC, GPIO_Pin_2);
                break;
            case 'y':
                grip_set(LEFT, OPEN);
                break;
            case 'h':
                grip_set(LEFT, UNHOLD);
                break;
            case 'n':
                grip_set(LEFT, HOLD);
                break;
            case 'u':
                grip_set(RIGHT, OPEN);
                break;
            case 'j':
                grip_set(RIGHT, UNHOLD);
                break;
            case 'm':
                grip_set(RIGHT, HOLD);
                break;
            case 'g':
                elevator_move(UP);
                break;
            case 'b':
                elevator_move(DOWN);
                break;
            case 'r':
                NVIC_SystemReset();
                break;
        }
    }
}
