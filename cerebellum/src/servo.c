#include <cerebellum/servo.h>

typedef struct {
    GPIO_TypeDef * gpio;
    uint16_t bitmask;
    uint16_t value;
} _servo_t;

volatile struct _sflags {
    unsigned update: 1;
    unsigned active: 1;
} servo_flags;

volatile _servo_t servo_queue[CONFIG_MAX_SERVO]; 
volatile _servo_t * p_servo_queue[CONFIG_MAX_SERVO];

volatile uint32_t servo_queue_len = 0;
volatile uint32_t num_add = 0;
volatile uint32_t curItem = 0;

void _servo_sort(void);
inline void _servo_timerOverflow(void);
inline void _servo_setPassive(void);
inline void _servo_setActive(void);
inline void _servo_timerCompare(void);
inline void _servo_switch(void);
inline void _servo_prepare(void);

// Prepare defined values
#if CONFIG_SERVO_TIMNO == 1
#define SERVO_TIMER_IRQn TIM1_UP_IRQn
#define SERVO_TIMER_CC_IRQn TIM1_CC_IRQn
#elif CONFIG_SERVO_TIMNO == 2
#define SERVO_TIMER_IRQn TIM2_IRQn
#elif CONFIG_SERVO_TIMNO == 3
#define SERVO_TIMER_IRQn TIM3_IRQn
#elif CONFIG_SERVO_TIMNO == 4
#define SERVO_TIMER_IRQn TIM4_IRQn
#elif CONFIG_SERVO_TIMNO == 5
#define SERVO_TIMER_IRQn TIM5_IRQn
#elif CONFIG_SERVO_TIMNO == 6
#define SERVO_TIMER_IRQn TIM6_IRQn
#elif CONFIG_SERVO_TIMNO == 7
#define SERVO_TIMER_IRQn TIM7_IRQn
#elif CONFIG_SERVO_TIMNO == 8
#define SERVO_TIMER_IRQn TIM8_UP_IRQn
#define SERVO_TIMER_CC_IRQn TIM8_CC_IRQn
#endif

#if CONFIG_SERVO_TIMER_COMPARE == 1
#define SERVO_TIMER_IT_CC TIM_IT_CC1
#elif CONFIG_SERVO_TIMER_COMPARE == 2
#define SERVO_TIMER_IT_CC TIM_IT_CC2
#elif CONFIG_SERVO_TIMER_COMPARE == 3
#define SERVO_TIMER_IT_CC TIM_IT_CC3
#elif CONFIG_SERVO_TIMER_COMPARE == 4
#define SERVO_TIMER_IT_CC TIM_IT_CC4
#endif


void servo_init(void)
{
    // In this function, we initialize
    // timer and timer interrupt
    
    // 0. Init RCC
    _snippet_runTIMRCC(CONFIG_SERVO_TIMER);
    
    // 1. Init timer
    TIM_TimeBaseInitTypeDef timer = {
        .TIM_Prescaler = 80, // prescaler will change sometimes
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 2047,
        .TIM_ClockDivision = TIM_CKD_DIV1
    };
    TIM_TimeBaseInit(CONFIG_SERVO_TIMER, &timer);

    // Configure output compare
    TIM_OCInitTypeDef compare = {
        .TIM_OCMode = TIM_OCMode_Timing,
        .TIM_Pulse = 0
    };

    // Set compare
    #if CONFIG_SERVO_TIMER_COMPARE == 1
    TIM_OC1Init(CONFIG_SERVO_TIMER, &compare);
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC1, ENABLE);
    #elif CONFIG_SERVO_TIMER_COMPARE == 2
    TIM_OC2Init(CONFIG_SERVO_TIMER, &compare);
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC2, ENABLE);
    #elif CONFIG_SERVO_TIMER_COMPARE == 3
    TIM_OC3Init(CONFIG_SERVO_TIMER, &compare);
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC3, ENABLE);
    #elif CONFIG_SERVO_TIMER_COMPARE == 4
    TIM_OC4Init(CONFIG_SERVO_TIMER, &compare);
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC4, ENABLE);
    #endif


    // Enable interrupts
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_Update, ENABLE);

    // Condfigure interrupt in NVIC
    NVIC_InitTypeDef interrupt = {
        .NVIC_IRQChannel = SERVO_TIMER_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 1,
        .NVIC_IRQChannelSubPriority = 1,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&interrupt);
    
    #ifdef SERVO_TIMER_CC_IRQn
    interrupt.NVIC_IRQChannel = SERVO_TIMER_CC_IRQn;
    NVIC_Init(&interrupt);
    #endif

    // That's so.
    // Enable timers
    TIM_Cmd(CONFIG_SERVO_TIMER, ENABLE);
}

servo servo_add(GPIO_TypeDef * gpio, uint8_t pin)
{
    uint32_t new_pos = servo_queue_len + num_add;
    // 0. Check for empty space in servo queue
    if(servo_queue_len >= CONFIG_MAX_SERVO)
        return CONFIG_MAX_SERVO;

    // 1. Init GPIO
    // 1.1. Run clock
    _snippet_runGPIORCC(gpio);

    // 2. Init GPIO
    static GPIO_InitTypeDef gpio_cnf = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_Out_PP
    };
    gpio_cnf.GPIO_Pin = (1<<pin);
    GPIO_Init(gpio, &gpio_cnf);

    // 2. Insert servo in servo queue
    servo_queue[new_pos].gpio = gpio;
    servo_queue[new_pos].bitmask = (1<<pin);
    servo_queue[new_pos].value = 1;

    p_servo_queue[new_pos] = &servo_queue[new_pos];

    // 3. Set flag to update servo queue
    servo_flags.update = 1;

    num_add++;
    
    if(new_pos == 0)
        TIM_Cmd(CONFIG_SERVO_TIMER, ENABLE);
    
    // 4. Return servo ID
    return (servo) new_pos;
}


/**
 * Set new value to the servo
 */
void servo_write(servo servo_id, uint16_t value)
{
    if(value == 0) value = 1;
    servo_queue[servo_id].value = value;
    _servo_sort();
}

/**
 * Sort servo array
 */
void _servo_sort(void)
{
    // In this function we just need to sort servo queue, according to number of items
    uint32_t a, b;
    
    for(a=0; a < servo_queue_len; a++)
    {
        uint8_t limit = servo_queue_len - a;
        for(b=0; b < limit; b++)
        {
            // While sorting, we modify array of pointers, so
            // Let's compare params
            if(p_servo_queue[b]->value > p_servo_queue[b+1]->value)
            {
                volatile _servo_t * temp = p_servo_queue[b];
                p_servo_queue[b] = p_servo_queue[b+1];
                p_servo_queue[b+1] = temp;
            }
        }
    }
}

inline void _servo_prepare(void)
{
    if(num_add > 0)
    {
        servo_queue_len += num_add;
        num_add = 0;
    }

    // 1. Update queue if required
    if(servo_flags.update)
    {
        _servo_sort();
        servo_flags.update = 0;
    }

    // 2. Set logical "1" on all servo outputs
    uint32_t i;
    for(i=0; i<servo_queue_len; i++)
        GPIO_SetBits(p_servo_queue[i]->gpio, p_servo_queue[i]->bitmask);

    // Now we are waiting for timer overflow to run in active mode
}

/**
 * Switch to active mode
 */
inline void _servo_setActive(void)
{
    servo_flags.active = 1;

    // Configure timer
    // Prescaler = 120 (if f=72 MHz)
    // Prescaler = 60 (if f=36 MHz)
    TIM_PrescalerConfig(CONFIG_SERVO_TIMER, 120, TIM_PSCReloadMode_Immediate);

    // Set 1st output compare and pointer to compare
    #if CONFIG_SERVO_TIMER_COMPARE == 1
    TIM_SetCompare1(CONFIG_SERVO_TIMER, p_servo_queue[0]->value);
    #elif CONFIG_SERVO_TIMER_COMPARE == 2
    TIM_SetCompare2(CONFIG_SERVO_TIMER, p_servo_queue[0]->value);
    #elif CONFIG_SERVO_TIMER_COMPARE == 3
    TIM_SetCompare3(CONFIG_SERVO_TIMER, p_servo_queue[0]->value);
    #elif CONFIG_SERVO_TIMER_COMPARE == 4
    TIM_SetCompare4(CONFIG_SERVO_TIMER, p_servo_queue[0]->value);
    #endif
    curItem = 0;
}

/**
 * Switch to passive mode
 */
inline void _servo_setPassive(void)
{
    servo_flags.active = 0;

    // Configure timer
    // Prescaler = 935 (if f = 72 MHz)
    // Prescaler = 468 (if f = 36 MHz)
    // Value to turn all servo on = 977
    // It means minimal pulse 0.6 ms
    TIM_PrescalerConfig(CONFIG_SERVO_TIMER, 120, TIM_PSCReloadMode_Immediate);
    TIM_SetCounter(CONFIG_SERVO_TIMER, 0); // clear timer if any

    #if CONFIG_SERVO_TIMER_COMPARE == 1
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC1, ENABLE);
    TIM_SetCompare1(CONFIG_SERVO_TIMER, 2047);
    #elif CONFIG_SERVO_TIMER_COMPARE == 2
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC2, ENABLE);
    TIM_SetCompare2(CONFIG_SERVO_TIMER, 2047);
    #elif CONFIG_SERVO_TIMER_COMPARE == 3
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC3, ENABLE);
    TIM_SetCompare3(CONFIG_SERVO_TIMER, 2047);
    #elif CONFIG_SERVO_TIMER_COMPARE == 4
    TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC4, ENABLE);
    TIM_SetCompare4(CONFIG_SERVO_TIMER, 2047);
    #endif
}

/**
 * Servo switcher
 */
inline void _servo_switch(void)
{
    uint16_t timerValue = TIM_GetCounter(CONFIG_SERVO_TIMER);
    // Turn off all current servos
    do
    {
        GPIO_ResetBits(p_servo_queue[curItem]->gpio, p_servo_queue[curItem]->bitmask);
        curItem++;
        if(curItem == servo_queue_len) break; // we reached end of queue
    }
    while(p_servo_queue[curItem]->value == timerValue);

    // Here we have an index of next servo item
    // Set compare value to next servo
    if(curItem == servo_queue_len)
        #if CONFIG_SERVO_TIMER_COMPARE == 1
        TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC1, DISABLE);
        #elif CONFIG_SERVO_TIMER_COMPARE == 2
        TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC2, DISABLE);
        #elif CONFIG_SERVO_TIMER_COMPARE == 3
        TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC3, DISABLE);
        #elif CONFIG_SERVO_TIMER_COMPARE == 4
        TIM_ITConfig(CONFIG_SERVO_TIMER, TIM_IT_CC4, DISABLE);
        #endif
    else
        #if CONFIG_SERVO_TIMER_COMPARE == 1
        TIM_SetCompare1(CONFIG_SERVO_TIMER, p_servo_queue[curItem]->value);
        #elif CONFIG_SERVO_TIMER_COMPARE == 2
        TIM_SetCompare2(CONFIG_SERVO_TIMER, p_servo_queue[curItem]->value);
        #elif CONFIG_SERVO_TIMER_COMPARE == 3
        TIM_SetCompare3(CONFIG_SERVO_TIMER, p_servo_queue[curItem]->value);
        #elif CONFIG_SERVO_TIMER_COMPARE == 4
        TIM_SetCompare4(CONFIG_SERVO_TIMER, p_servo_queue[curItem]->value);
        #endif
}

/**
 * Servo timer overflow
 */
inline void _servo_timerOverflow(void)
{
    // 1. Check if we have some items in queue
    if(servo_queue_len > 0)
    {
        // Check mode of operation
        if(servo_flags.active) // if we were in active mode
            _servo_setPassive(); // switch to passive
        else                   // if we were in passive mode
        {
            _servo_setActive();  // switch to active
        }
    }
    // 2. If there are no items, let's try to get 'em
    else if(num_add > 0) 
    {
        _servo_prepare();
        _servo_setActive();
    }
    // 3. If there is nothing to add, stay in passive for saving resourses
    else
    {
        _servo_setPassive();
    }
}

/**
 * Servo timer output compare
 */
inline void _servo_timerCompare(void)
{
    // Check mode of operation
    if(servo_flags.active) // in active mode, change levels
    {
        _servo_switch();
    }
    else
    {                       // in passive mode, it could be only setup interrupt
        _servo_prepare();  // prepare servos to run
    }
}

/**
 * Interrupt vector
 */

#if CONFIG_SERVO_TIMNO == 1
void TIM1_CC_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 2
void TIM2_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 3
void TIM3_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 4
void TIM4_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 5
void TIM5_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 6
void TIM6_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 7
void TIM7_IRQHandler(void)
#elif CONFIG_SERVO_TIMNO == 8
void TIM8_CC_IRQHandler(void)
#else
#error "Servo library supports TIM1-TIM8 only (other have no interrupts)"
#endif
{
    // Select required interrupt
    if(TIM_GetITStatus(CONFIG_SERVO_TIMER, SERVO_TIMER_IT_CC) != RESET) // output compare
    {
        _servo_timerCompare();
        TIM_ClearITPendingBit(CONFIG_SERVO_TIMER, SERVO_TIMER_IT_CC);
    }
#if CONFIG_SERVO_TIMNO == 1
}
void TIM1_UP_IRQHandler(void)
{
#elif CONFIG_SERVO_TIMNO == 8
}
void TIM8_UP_IRQHandler(void)
{
#endif
    if(TIM_GetITStatus(CONFIG_SERVO_TIMER, TIM_IT_Update) != RESET)// timer update
    {
        _servo_timerOverflow();
        TIM_ClearITPendingBit(CONFIG_SERVO_TIMER, TIM_IT_Update);
    }
}
