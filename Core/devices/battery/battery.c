#include "battery.h"

battery_data_t battery_data;
extern ADC_HandleTypeDef hadc1;
#define ADC_COEFF 0.00564102564

void update_battery_data(){
    HAL_ADC_Start(&hadc1);
    if( HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK )
    {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        battery_data.voltage = adc_value * ADC_COEFF;
    }
    HAL_ADC_Stop(&hadc1);
}