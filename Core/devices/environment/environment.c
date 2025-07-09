#include "environment.h"


extern struct bme280_t bme280;

env_data_t env_data;

float temp_median[5];
float press_median[5];
float hum_median[5];

int median_env_data_index = 0;

int init_env_data()
{
    BME280_SPI_routine();
    int comres = bme280_init(&bme280);
    if(comres != BME280_SUCCESS) {
        printf("bme280 init failed %d\n", comres);
        Error_Handler();
    }
    comres = bme280_set_power_mode(BME280_NORMAL_MODE);
    comres += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    comres += bme280_set_oversamp_pressure(BME280_OVERSAMP_8X);
    comres += bme280_set_oversamp_humidity(BME280_OVERSAMP_SKIPPED);
    comres += bme280_set_filter(BME280_FILTER_COEFF_16);
    comres += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    return comres;
}

int update_env_data()
{
    s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    /* API is used to read the compensated temperature, humidity and pressure*/
    int com_rslt = bme280_read_pressure_temperature_humidity(
    &v_comp_press_u32[1], &v_comp_temp_s32[1],  &v_comp_humidity_u32[1]);

    float imp_temp = ((float)(v_comp_temp_s32[1])/100);		// convert to fahrenheit
    //float temp = ((float)v_comp_temp_s32[1]/100);
    float imp_press = ((float)(v_comp_press_u32[1])/100); 	// convert to inches of mercury
    //float press = ((float)(v_comp_press_u32[1])/100);
    float imp_humi = ((float)(v_comp_humidity_u32[1])/1024);		// relative humidity
    if (com_rslt != BME280_SUCCESS) {
        return -1;
    }
    env_data.temp = imp_temp;
    env_data.press = imp_press;
    env_data.hum = imp_humi;
    env_data.timestamp = HAL_GetTick();
    return 0;
}