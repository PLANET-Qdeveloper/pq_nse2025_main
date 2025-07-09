#ifndef __WIRELESS_H__
#define __WIRELESS_H__

#include "main.h"
#include "E22_900T22S.h"
#include "stdio.h"

typedef struct {
    uint8_t data[128];
    uint16_t size;
} wireless_data_t;

int init_wireless();
int send_data(uint8_t *data, uint16_t size);
int receive_data(uint8_t *data, uint16_t size);
int add_buffer_wireless(uint8_t *data, uint16_t size);
#endif