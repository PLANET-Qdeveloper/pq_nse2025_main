#include "gnss.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

extern UART_HandleTypeDef huart5;
extern I2C_HandleTypeDef hi2c2;
lwgps_t gh;

gnss_data_t gnss_data;
uint8_t data_buffer[GNSS_I2C_NUM_BYTES];
int init_gnss()
{
    HAL_GPIO_WritePin(RESET_GNSS_GPIO_Port, RESET_GNSS_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(RESET_GNSS_GPIO_Port, RESET_GNSS_Pin, GPIO_PIN_SET);

    while(1) {
        int result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(GNSS_I2C_ADDR), 2, 2);
        if(result == HAL_OK) {
            break;
        }
        osDelay(100);
    }

    uint8_t config[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x06, 0xF3, 0x58};
    int comres = HAL_I2C_Master_Transmit(&hi2c2, GNSS_I2C_ADDR, config, sizeof(config), 100);
    if(comres != HAL_OK) {
        return -1;
    }

    lwgps_init(&gh);
    
    return 0;
}

int update_gnss_data()
{
    uint8_t data_available[4] = {0};
    uint16_t data_available_16 = 0;
    int comres = HAL_I2C_Mem_Read(&hi2c2, GNSS_I2C_ADDR, GNSS_I2C_NUM_BYTE_HIGH_ADDR, 1, data_available, 2, 100);
    if(comres != HAL_OK) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        return -1;
    }
    data_available_16 = (data_available[0] << 8) | data_available[1];
    if (data_available_16 == 0) {
        return -1;
    }
    do{
        int data_len = data_available_16 > GNSS_I2C_NUM_BYTES ? GNSS_I2C_NUM_BYTES: data_available_16;
        comres = HAL_I2C_Master_Receive(&hi2c2, GNSS_I2C_ADDR, data_buffer, data_len, 100);
        comres = !lwgps_process(&gh, data_buffer, data_available_16);
        if(!comres) {
            gnss_data.latitude = gh.latitude;
            gnss_data.longitude = gh.longitude;
            gnss_data.altitude = gh.altitude;
            gnss_data.num_sats = gh.sats_in_view;
            gnss_data.hdop = gh.dop_h;
        }
        data_available_16 -= data_len;
    }while(data_available_16 > 0);
    

    
    return comres;
}