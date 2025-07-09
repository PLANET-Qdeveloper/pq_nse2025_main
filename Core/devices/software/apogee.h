#ifndef APOGEE_H
#define APOGEE_H

#include "main.h"
#include "arm_math.h"

typedef struct {
    float median_pressure[5];
    int median_pressure_index;
    uint32_t time;
} apogee_t;

int apogee_detect(float pressure);
#endif