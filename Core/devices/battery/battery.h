#ifndef BATTERY_H
#define BATTERY_H

#include "main.h"

typedef struct {
    float voltage;
} battery_data_t;



void update_battery_data();


#endif


