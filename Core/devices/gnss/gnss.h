#ifndef __GNSS_H__
#define __GNSS_H__

#include "main.h"
#include "lwgps.h"
#define GNSS_I2C_ADDR 0x84

#define GNSS_I2C_NUM_BYTES 4096

#define GNSS_I2C_NUM_BYTE_HIGH_ADDR 0xFD
#define GNSS_I2C_NUM_BYTE_LOW_ADDR 0xFE
#define GNSS_I2C_DATA_ADDR 0xFF

typedef struct {
    double latitude;
    double longitude;
    float altitude;
    int num_sats;
    float hdop;
} gnss_data_t;

int init_gnss();
int update_gnss_data();

#endif


