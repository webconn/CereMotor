#include <cerebellum/sensors.h>

// Prepare defines
#if defined(CONFIG_SENSORS_USE_ADC1)
#define SENSORS_ADC ADC1
#define SENSORS_DMA DMA1_Channel1
#define SENSORS_DMA_RCC RCC_AHBPeriph_DMA1
#define SENSORS_ADC_RCC RCC_APB2Periph_ADC1
#elif defined(CONFIG_SENSORS_USE_ADC3)
#define SENSORS_ADC ADC3
#define SENSORS_DMA DMA2_Channel5
#define SENSORS_DMA_RCC RCC_AHBPeriph_DMA2
#define SENSORS_ADC_RCC RCC_APB2Periph_ADC3
#endif


/**
 * Array of measured channel values
 */
uint16_t _adc_measures[CONFIG_SENSORS_ADC_MAX_CHANNELS];

/**
 * Number of used channels
 */
uint32_t _adc_numberOfChannels = 0; // incremented by every new sensor registered

/**
 * DMA init structure
 */
DMA_InitTypeDef _dma = {
    .DMA_Mode = DMA_Mode_Circular, // run in a loop
    .DMA_PeripheralBaseAddr = (uint32_t) & (SENSORS_ADC->DR), // take data from SENSORS_ADC Data Register
    .DMA_MemoryBaseAddr = (uint32_t) & (_adc_measures[0]), // write data to array starting from 0
    .DMA_DIR = DMA_DIR_PeripheralSRC, // data flow direction - obviously, from periph to memory
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable, // we read from only one register, so - no increment
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_BufferSize = 0, // number of values in buffer
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord, // in register - 16-bit value
    .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord, // in memory - 16-bit too
    .DMA_Priority = DMA_Priority_Medium,
    .DMA_M2M = DMA_M2M_Disable
};

/**
 * ADC init structure
 */
ADC_InitTypeDef _adc = {
    .ADC_Mode = ADC_Mode_Independent,
    .ADC_ScanConvMode = ENABLE, // channels will be scanned step-by-step
    .ADC_ContinuousConvMode = ENABLE, // return to 1st element in queue if end is reached
    .ADC_ExternalTrigConv = ADC_ExternalTrigConv_None, // no external trigger to run the measure
    .ADC_DataAlign = ADC_DataAlign_Right, // data align... if required
    .ADC_NbrOfChannel = 0 // number of initializing channels
};

void sensor_init(void)
{
    // 0. Run clocks
    RCC_AHBPeriphClockCmd(SENSORS_DMA_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(SENSORS_ADC_RCC, ENABLE);

    // 1. Init ADC
    ADC_Init(SENSORS_ADC, &_adc);

    // 2. Init DMA for writing measures directly to array
    DMA_Init(SENSORS_DMA, &_dma); // SENSORS_ADC is on DMA1 channel 1

    // 3. Setup SENSORS_ADC to send DMA requests
    ADC_DMACmd(SENSORS_ADC, ENABLE);

    // 4. Enable DMA
    DMA_Cmd(SENSORS_DMA, ENABLE);

    // 5. Enable ADC
    ADC_Cmd(SENSORS_ADC, ENABLE);

    // 6. Calibrate ADC
    ADC_ResetCalibration(SENSORS_ADC);
    while(ADC_GetResetCalibrationStatus(SENSORS_ADC));;;

    ADC_StartCalibration(SENSORS_ADC);
    while(ADC_GetCalibrationStatus(SENSORS_ADC));;;
}

void sensor_add(sensor_t * sensor)
{
    // 0. Init RCC
    _snippet_runGPIORCC(sensor->gpio);

    GPIO_InitTypeDef pin_GPIO;

    // 1. Set mode GPIO init structure
    switch(sensor->mode)
    {
        case SENSOR_PASSIVE_GND:
            pin_GPIO.GPIO_Mode = GPIO_Mode_IPU;
            break;
        case SENSOR_PASSIVE_VCC:
            pin_GPIO.GPIO_Mode = GPIO_Mode_IPD;
            break;
        case SENSOR_ANALOG:
            
            // 0. Check for free channel
            if(_adc_numberOfChannels >= 16) return; // no channel, sorry

            // 1. Setup GPIO
            pin_GPIO.GPIO_Mode = GPIO_Mode_AIN;

            // 2. Turn off ADC and DMA
            ADC_Cmd(SENSORS_ADC, DISABLE);
            DMA_Cmd(SENSORS_DMA, DISABLE);
            ADC_DMACmd(SENSORS_ADC, DISABLE);

            // 2. Configure ADC channel
            // 2.1. Reconfigure ADC for using one more channel
            _adc.ADC_NbrOfChannel = _adc_numberOfChannels + 1;
            ADC_Init(SENSORS_ADC, &_adc); // re-initialising goes right, hmmm... Let it be

            // 2.2. Reconfigure DMA for using one more channel
            _dma.DMA_BufferSize = _adc_numberOfChannels + 1;
            DMA_DeInit(SENSORS_DMA);
            DMA_Init(SENSORS_DMA, &_dma);

            // Enable DMA and ADC
            ADC_Cmd(SENSORS_ADC, ENABLE);
            DMA_Cmd(SENSORS_DMA, ENABLE);
            ADC_DMACmd(SENSORS_ADC, ENABLE);

            // Configure one more channel for ADC
            ADC_RegularChannelConfig(SENSORS_ADC, sensor->channel, _adc_numberOfChannels+1, CONFIG_SENSORS_ADC_SAMPLETIME); // hmmm... play with sample time

            // 3. Set channel ID in structure
            sensor->chid = _adc_numberOfChannels;

            // 4. Increase _adc_addChannels counter
            _adc_numberOfChannels++;

            // Launch ADC convertion
            ADC_SoftwareStartConvCmd(SENSORS_ADC, ENABLE);

            break;
        default:
            pin_GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }

    // 2. Set GPIO pin
    pin_GPIO.GPIO_Pin = sensor->pin;

    // 3. Init GPIO
    GPIO_Init(sensor->gpio, &pin_GPIO);
}

uint32_t sensor_read(sensor_t * sensor)
{
    if(sensor->mode == SENSOR_ANALOG)
    {
        return _adc_measures[sensor->chid];
    }
    else if(sensor->mode == SENSOR_ACTIVE_GND || sensor->mode == SENSOR_PASSIVE_GND)
    {
        return GPIO_ReadInputDataBit(sensor->gpio, sensor->pin) == 0;
    }
    else // active and passive to VCC
    {
        return GPIO_ReadInputDataBit(sensor->gpio, sensor->pin) > 0;
    }
}
