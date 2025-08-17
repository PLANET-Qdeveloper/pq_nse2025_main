#include "valve.h"
#include "wireless.h"

extern UART_HandleTypeDef huart3;
valve_data_t valve_data = {0};
uint8_t valve_received_data[64] = {0};
int valve_received_data_index = 0;

int valve_check(void) {
    // Poll UART for incoming data
    uint8_t received_data[1] = {0};
    
    if (HAL_UART_Receive(&huart3, received_data, 1, 1) == HAL_OK) {
        // Process received data
        do{
            if(received_data[0] == '\n'){
                valve_received_data[valve_received_data_index] = '\0';
                int res = sscanf(valve_received_data, "%d,%d,%d,%d", &valve_data.temperature, &valve_data.pressure, &valve_data.is_servo_open, &valve_data.is_nos_open);
                valve_received_data_index = 0;
                memset(valve_received_data, 0, sizeof(valve_received_data));
            }
            if(valve_received_data_index < sizeof(valve_received_data) - 1){
                valve_received_data[valve_received_data_index++] = received_data[0];
            }else{
                valve_received_data_index = 0;
                memset(valve_received_data, 0, sizeof(valve_received_data));
            }
        }while(HAL_UART_Receive(&huart3, received_data, 1, 1) == HAL_OK);
        return 0;
    }
    return -1; // No data received
}