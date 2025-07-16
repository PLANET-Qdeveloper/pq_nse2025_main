#include "wireless.h"


struct LoRa_Handler LoRaTX = {0};
extern UART_HandleTypeDef huart1;
extern uint8_t lora_receive_byte;
wireless_data_t wireless_data;

int init_wireless(){
    LoRaTX.UART_HANDLER = &huart1;
    LoRaTX.M0_GPIO_PORT = M0_GPIO_Port;
    LoRaTX.M1_GPIO_PORT = M1_GPIO_Port;
    LoRaTX.AUX_GPIO_PORT = AUX_GPIO_Port;
    LoRaTX.RESET_GPIO_PORT = RESET_LORA_GPIO_Port;
    LoRaTX.M0_PIN = M0_Pin;
    LoRaTX.M1_PIN = M1_Pin;
    LoRaTX.MAUX_PIN = AUX_Pin;
    LoRaTX.RESET_PIN = RESET_LORA_Pin;


    struct LoRa_Configuration LoRaConf = {0};


    HAL_GPIO_WritePin(POW_COM_GPIO_Port, POW_COM_Pin, GPIO_PIN_SET);
    E22_Reset(&LoRaTX, &LoRaConf, 0);
    E22_Config_Init(&LoRaConf);
    LoRaConf.Uart_baud_rate = UART_BAUDRATE_9600;
    LoRaConf.Air_data_rate = AIR_DATA_RATE_62500;
    E22_Config_Set(&LoRaTX, &LoRaConf);
    uint8_t data[8] = {0};
    E22_Register_Read_all(&LoRaTX, &LoRaConf, data);
    
    return 0;
}

int send_data(uint8_t *data, uint16_t size){
    return E22_Payload_Transmit_Transparent(&LoRaTX, data, size);
}

int receive_data(uint8_t *data, uint16_t size){
    return E22_Payload_Receive(&LoRaTX, data, size);
}

int add_buffer_wireless(uint8_t *data, uint16_t size){
    if(wireless_data.size + size > sizeof(wireless_data.data)){
        return -1;
    }
    memcpy(wireless_data.data + wireless_data.size, data, size);
    wireless_data.size += size;
    return 0;
}

int check_wireless(){
    uint8_t data[1] = {0};
    int res = E22_Payload_Receive(&LoRaTX, data, 1);
    while(res == HAL_OK){
        add_buffer_wireless(data, 1);
        res = E22_Payload_Receive(&LoRaTX, data, 1);
    }
    return 0;
}