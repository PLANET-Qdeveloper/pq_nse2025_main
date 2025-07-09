#include "apogee.h"

apogee_t apogee;


int apogee_detect(float pressure){
    apogee.median_pressure[apogee.median_pressure_index] = pressure;
    apogee.median_pressure_index++;
    if (apogee.median_pressure_index >= 5){
        apogee.median_pressure_index = 0;
    }
    float median_pressure = 0;
    return 0;
}








