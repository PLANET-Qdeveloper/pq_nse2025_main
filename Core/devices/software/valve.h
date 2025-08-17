#ifndef __VALVE_H__
#define __VALVE_H__

#include "main.h"

typedef struct {
    int temperature;
    int pressure;
    int is_servo_open;
    int is_nos_open;
} valve_data_t;

void valve_init(void);
int valve_check();





#endif