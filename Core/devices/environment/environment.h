#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "bme280.h"
#include "bme280_support.h"
#include "bme280.h"
#include "main.h"
#include "stdio.h"
#include "arm_math.h"

int init_env_data();
int update_env_data();
q31_t calculate_median(q31_t *array);

#endif

