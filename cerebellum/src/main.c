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

// Direction corrector (angle-based)
pid_regulator_t chassis_pid = {
    .p_gain = 180,
    .i_rgain = 500000,
    .d_rgain = 20,
    
    .i_max = 100000,
    .i_min = -100000,

    .i_mem = 0,
    .d_mem = 0
};
/*
pid_regulator_t chassis_pid = {
    .p_gain = 16,
    .i_rgain = 50000,
    .d_rgain = 10,
    
    .i_max = 100000,
    .i_min = -100000,

    .i_mem = 0,
    .d_mem = 0
};*/

// Speed corrector
pid_regulator_t speed_pid = {
    .p_gain = 80,
    .i_rgain = 2000,
    .d_rgain = 100,
    
    .i_max = 2000000,
    .i_min = -2000000,

    .i_mem = 0,
    .d_mem = 0
};

void SysTick_Handler(void)
{
    led_on(2);
    encoders_parser(); // update encoders values
    coords_update(encoder_getDelta(0), encoder_getDelta(1));
    
    __move_correct();
    
    if(_delay > 0)
        _delay--;
    led_off(2);

    time++;
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

void tactics_red(void);
void tactics_blue(void);

int main(void)
{
    // 1. Init I/O
    _init_io();

    // 3. Configuring chassis PID regulators

    // Turning on LED - end of initialisation
    led_on(1);
    //led_on(2);
    //led_on(3);

    starter = 1;
    //move_saveSwitch(DISABLE);
    
    // Select tactics switch
    tactics_red();

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
    move_setStopCalculator(&__move_stopCalculator_active);
    move_setStopCoefficient(16);

    // 2. Set PIDs
    move_setPathPID(&chassis_pid);
    move_setSpeedPID(&speed_pid);

    // 3. Set required speeds and path if required
    move_setRqSpeed(50);
    move_setRqAcceleration(1);
    move_setRqAccelerationDivider(2);
    move_setRqPath(mmToTicks(1500));

    // 4. Apply values for algos
    move_applyUserValues();
    move_stopDisable();

    // Now we should move
    time = 0;
    while(time < 10);;;
    while(move_isBusy())
    {
        printf("%ld %ld\n\r", encoder_getDelta(LEFT), encoder_getDelta(RIGHT));
        uart_send(1, '\n');
        _delay_ms(10);
    }

    move_setStopZeroValues();
    move_stopEnable();

    _delay_ms(1000);

    encoder_reset(0);
    encoder_reset(1);
    move_setRqPath(-mmToTicks(1500));
    move_applyUserValues();
    move_stopDisable();

    _delay_ms(100);
    while(move_isBusy());
    move_setStopZeroValues();
    move_stopEnable();
    move_setRqSpeed(0);
}
