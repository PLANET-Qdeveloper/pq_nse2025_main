#include "environment.h"
#include "logger.h"
#include "arm_math.h"
extern struct bme280_t bme280;

env_data_t env_data;

#define NUM_TAPS 48
#define NUM_TAPS_AFTER_DIFF 60
const float fir_coeff_float_after_diff[] = {
    0.000000000000000000,
    0.000025695329810175,
    0.000109001586731170,
    0.000261383834469497,
    0.000497284187006939,
    0.000834034055775751,
    0.001291444409523049,
    0.001891070645768172,
    0.002655173095064218,
    0.003605418278370960,
    0.004761388110941483,
    0.006138982684280024,
    0.007748815659053334,
    0.009594708572398906,
    0.011672390798343837,
    0.013968505240990876,
    0.016460006299700354,
    0.019114016903342568,
    0.021888186575993940,
    0.024731564041054279,
    0.027585967536853457,
    0.030387805705078649,
    0.033070273559075526,
    0.035565823484471772,
    0.037808792095928072,
    0.039738051371860414,
    0.041299547699896848,
    0.042448595699213505,
    0.043151804843812071,
    0.043388535390380012,
    0.043151804843812071,
    0.042448595699213505,
    0.041299547699896848,
    0.039738051371860407,
    0.037808792095928072,
    0.035565823484471785,
    0.033070273559075540,
    0.030387805705078645,
    0.027585967536853460,
    0.024731564041054296,
    0.021888186575993951,
    0.019114016903342579,
    0.016460006299700354,
    0.013968505240990888,
    0.011672390798343854,
    0.009594708572398910,
    0.007748815659053343,
    0.006138982684280031,
    0.004761388110941481,
    0.003605418278370961,
    0.002655173095064216,
    0.001891070645768171,
    0.001291444409523047,
    0.000834034055775751,
    0.000497284187006940,
    0.000261383834469497,
    0.000109001586731170,
    0.000025695329810175,
    0.000000000000000000,
    0.000000000000000000,
};

const float fir_coeff_float[] = {
    0.000000000000000000,
    -0.000024024696576945,
    0.000101226956250659,
    -0.000242163323719646,
    0.000460999749780538,
    -0.000774774630602397,
    0.001202120701130650,
    -0.001761530971058314,
    0.002469307247330359,
    -0.003337371808104531,
    0.004371148912631902,
    -0.005567729920489168,
    0.006914522500969975,
    -0.008388551337371363,
    0.009956527400794210,
    -0.011575739536704845,
    0.013195751371189323,
    -0.014760814825634478,
    0.016212845473916451,
    -0.017494750818964289,
    0.018553865526707455,
    -0.019345231432795058,
    0.019834466537781616,
    0.979999801847076246,
    0.019834466537781620,
    -0.019345231432795058,
    0.018553865526707451,
    -0.017494750818964296,
    0.016212845473916455,
    -0.014760814825634478,
    0.013195751371189323,
    -0.011575739536704854,
    0.009956527400794212,
    -0.008388551337371361,
    0.006914522500969971,
    -0.005567729920489170,
    0.004371148912631905,
    -0.003337371808104533,
    0.002469307247330362,
    -0.001761530971058315,
    0.001202120701130649,
    -0.000774774630602398,
    0.000460999749780538,
    -0.000242163323719647,
    0.000101226956250659,
    -0.000024024696576946,
    0.000000000000000000,
    0.000000000000000000
};


q31_t fir_coeff_q31[NUM_TAPS + 2] = {0};
q31_t fir_coeff_q31_after_diff[NUM_TAPS_AFTER_DIFF + 2] = {0};

q31_t temp_median[5];
int temp_median_index = 0;
arm_fir_instance_q31 temp_fir;
q31_t temp_fir_state[2+NUM_TAPS-1] = {0};




q31_t press_median[5];
int press_median_index = 0;
arm_fir_instance_q31 press_fir;
arm_fir_instance_q31 press_fir_after_diff;
q31_t press_fir_state[2+NUM_TAPS-1] = {0};
q31_t press_fir_state_after_diff[2+NUM_TAPS_AFTER_DIFF-1] = {0};


q31_t press_previous_value = 0;
q31_t press_previous_value_2 = 0;
int press_delta_index = 0;

q31_t calculate_median(q31_t *array) {
    q31_t temp_array[5];
    for (int i = 0; i < 5; i++) {
        temp_array[i] = array[i];
    }
    
    for (int i = 0; i < 5 - 1; i++) {
        for (int j = 0; j < 5 - i - 1; j++) {
            if (temp_array[j] > temp_array[j + 1]) {
                q31_t temp = temp_array[j];
                temp_array[j] = temp_array[j + 1];
                temp_array[j + 1] = temp;
            }
        }
    }
    
    return temp_array[2];
}


int init_env_data()
{
    BME280_SPI_routine();
    int comres = bme280_init(&bme280);
    if(comres != BME280_SUCCESS) {
        output_log(LOG_LEVEL_ERROR, "bme280 init failed %d\n", comres);
        Error_Handler();
    }
    comres = bme280_set_power_mode(BME280_NORMAL_MODE);
    comres += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    comres += bme280_set_oversamp_pressure(BME280_OVERSAMP_8X);
    comres += bme280_set_oversamp_humidity(BME280_OVERSAMP_SKIPPED);
    comres += bme280_set_filter(BME280_FILTER_COEFF_16);
    comres += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);

    arm_float_to_q31(fir_coeff_float, fir_coeff_q31, NUM_TAPS-1);
    arm_float_to_q31(fir_coeff_float_after_diff, fir_coeff_q31_after_diff, NUM_TAPS_AFTER_DIFF-1);

    arm_fir_init_q31(&temp_fir, NUM_TAPS, fir_coeff_q31, temp_fir_state, 1);
    arm_fir_init_q31(&press_fir, NUM_TAPS, fir_coeff_q31, press_fir_state, 1);
    arm_fir_init_q31(&press_fir_after_diff, NUM_TAPS_AFTER_DIFF, fir_coeff_q31_after_diff, press_fir_state_after_diff, 1);
    
    return comres;
}

int update_env_data()
{
    s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    /* API is used to read the compensated temperature, humidity and pressure*/
    int com_rslt = bme280_read_uncomp_pressure(&v_comp_press_u32[0]);
    com_rslt += bme280_read_uncomp_temperature(&v_comp_temp_s32[0]);
    v_comp_press_u32[0] = bme280_compensate_pressure_int32(v_comp_press_u32[0]);
    v_comp_temp_s32[0] = bme280_compensate_temperature_int32(v_comp_temp_s32[0]);


    float imp_temp = ((float)(v_comp_temp_s32[0])/100);		// convert to fahrenheit
    //float temp = ((float)v_comp_temp_s32[1]/100);
    float imp_press = ((float)(v_comp_press_u32[0])/100); 	// convert to inches of mercury
    //float press = ((float)(v_comp_press_u32[1])/100);
    if (com_rslt != BME280_SUCCESS) {
        return -1;
    }
    env_data.temp = imp_temp;
    env_data.press = imp_press;
    float temp_normalized = (imp_temp - 20.0f) / 100.0f;
    float press_normalized = (imp_press - 800.0f) / 1000.0f;
    arm_float_to_q31(&temp_normalized, &temp_median[temp_median_index], 1);
    arm_float_to_q31(&press_normalized, &press_median[press_median_index], 1);
    temp_median_index++;
    press_median_index++;
    if (temp_median_index >= 5) {
        temp_median_index = 0;
    }
    if (press_median_index >= 5) {
        press_median_index = 0;
    }
    
    q31_t temp_median_unfiltered = calculate_median(temp_median);
    q31_t press_median_unfiltered = calculate_median(press_median);
    q31_t temp_res[1] = {0};
    q31_t press_res[1] = {0};
    arm_fir_q31(&temp_fir, &temp_median_unfiltered, temp_res, 1);
    arm_fir_q31(&press_fir, &press_median_unfiltered, press_res, 1);
    q31_t cascade_filtered[1] = {0};
    arm_fir_q31(&press_fir_after_diff, press_res, cascade_filtered, 1);

    


    q31_t press_delta_accum = cascade_filtered[0] * 3 - press_previous_value * 4 + press_previous_value_2;
    
    press_previous_value_2 = press_previous_value;
    press_previous_value = cascade_filtered[0];
    press_delta_accum /= 2;


    float press_delta_accum_float;
    q31_t press_delta_fir_res[1] = {0};
    
    arm_q31_to_float(&press_delta_accum, &press_delta_accum_float, 1);
    env_data.delta_press = press_delta_accum_float * 100.0f;


    arm_q31_to_float(temp_res, &env_data.temp_filtered, 1);
    arm_q31_to_float(press_res, &env_data.press_filtered, 1);

    env_data.temp_filtered = env_data.temp_filtered * 100.0f + 20.0f;
    env_data.press_filtered = env_data.press_filtered * 1000.0f + 800.0f;
    
    env_data.timestamp = HAL_GetTick();
    return 0;
}