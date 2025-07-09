///-----------------------------------------------
///  Ebyte E22-900T-22S Library for STM32
///  E22-900T30Dと互換性はないです.
///  2025/04/26 Ver 1.0.0
///  Author: Unknown
///	 
/// 
///
///   ~1A 出力できる電源が必要.
///   連続送信するとぬくぬくになるのでヒートシンク必要.
///
///   電源を入れる前に必ずアンテナを接続しよう☆☆
///   電源を入れる前に必ずアンテナを接続しよう☆☆
///   電源を入れる前に必ずアンテナを接続しよう☆☆
///-----------------------------------------------

#ifndef INCLUDED_E22_900T22S
#define INCLUDED_E22_900T22S

#include "main.h"


/// @brief UART Baud rate setting
enum UART_BAUDRATE {
	UART_BAUDRATE_1200 = 0b00000000,
	UART_BAUDRATE_2400 = 0b00100000,
	UART_BAUDRATE_4800 = 0b01000000,
	UART_BAUDRATE_9600 = 0b01100000, // default
	UART_BAUDRATE_19200 = 0b10000000,
	UART_BAUDRATE_38400 = 0b10100000,
	UART_BAUDRATE_57600 = 0b11000000,
	UART_BAUDRATE_115200 = 0b11100000
};

/// @brief Serial parity setting
enum SERIAL_PARITY {
	SERIAL_PARITY_8N1 = 0b00000000, // default
	SERIAL_PARITY_8O1 = 0b00001000,
	SERIAL_PARITY_8E1 = 0b00010000,
	SERIAL_PARITY_8N1_2 = 0b00011000
};

/// @brief Wireless air data rate setting. The higher it is, the shorter the transmission distance is.
/// @attention Difference between AIR_DATA_RATE_2400_1, 2 and 3 is Unclear. May be a Spreading factor?
enum AIR_DATA_RATE {
	AIR_DATA_RATE_2400_1 = 0b00000000,
	AIR_DATA_RATE_2400_2 = 0b00000001,
	AIR_DATA_RATE_2400_3 = 0b00000010, // default
	AIR_DATA_RATE_4800 = 0b00000011,
	AIR_DATA_RATE_9600 = 0b00000100,
	AIR_DATA_RATE_19200 = 0b00000101,
	AIR_DATA_RATE_38400 = 0b00000110,
	AIR_DATA_RATE_62500 = 0b00000111
};

/// @brief When the data sent is smaller than the subpacketlength, the serial output of the receiving end is an uninterrupted continuous output. When the data sent is larger than the subpacket length, the serial port in receiving endwillsubpacket the data and then output them.
enum SUB_PACKET_SIZE {
	SUB_PACKET_SIZE_240 = 0b00000000, // default
	SUB_PACKET_SIZE_128 = 0b01000000,
	SUB_PACKET_SIZE_64 = 0b10000000,
	SUB_PACKET_SIZE_32 = 0b11000000
};

/// @brief RSSI function. Check the datasheet for details.
enum RSSI_NOISE {
	RSSI_NOISE_DISABLED = 0b00000000, // default
	RSSI_NOISE_ENABLE = 0b00100000
};

/// @brief System Reserved
enum LORA_RESERVED {
	LORA_RESERVED = 0b00000000
};

/// @brief If you dont wanna use the M0 and M1 pins to switch working modes, you can enable this function. Check the datasheet for details.
enum SOFTWARE_MODE_FUNCTION {
	SOFTWARE_MODE_FUNCTION_DISABLED = 0b00000000, // default
	SOFTWARE_MODE_FUNCTION_ENABLE = 0b00000100
};

/// @brief Set transmission power. 22dBm ≒ 160mW
enum TX_POWER {
	TX_POWER_22DBM = 0b00000000, // default
	TX_POWER_17DBM = 0b00000001,
	TX_POWER_13DBM = 0b00000010,
	TX_POWER_10DBM = 0b00000011
};


/// @brief Check the datasheet for more details.
enum RSSI_BYTES {
	RSSI_BYTES_DISABLED = 0b00000000, // default
	RSSI_BYTES_ENABLE = 0b10000000
};

/// @brief During fixed transmission, the module will identify the first 3 bytes as: addr_high + addr_low + channel, and use them as wireless transmission targets.
enum TRANSMISSION_METHOD {
	TRANSMISSION_METHOD_TRANSPARENT = 0b00000000, // default
	TRANSMISSION_METHOD_FIXED = 0b01000000
};



/// @brief Check the datasheet for more information
enum WOR_MODE {
	WOR_MODE_RECEIVER = 0b00000000, // default
	WOR_MODE_TRANSMITTER = 0b00001000
};

/// @brief  Cycle time T = (1 + WOR) * 500ms. Both transmitter and the receiver must be set as the same cycle time(ms)
enum WOR_CYCLE_TIME {
	WOR_CYCLE_TIME_500 = 0b00000000,
	WOR_CYCLE_TIME_1000 = 0b00000001,
	WOR_CYCLE_TIME_1500 = 0b00000010,
	WOR_CYCLE_TIME_2000 = 0b00000011,
	WOR_CYCLE_TIME_2500 = 0b00000100,
	WOR_CYCLE_TIME_3000 = 0b00000101,
	WOR_CYCLE_TIME_3500 = 0b00000110,
	WOR_CYCLE_TIME_4000 = 0b00000111
};

struct LoRa_Handler {
    UART_HandleTypeDef *UART_HANDLER;
	GPIO_TypeDef *M0_GPIO_PORT;
	GPIO_TypeDef *M1_GPIO_PORT;
	GPIO_TypeDef *AUX_GPIO_PORT;
	GPIO_TypeDef *RESET_GPIO_PORT;
	uint16_t M0_PIN;
	uint16_t M1_PIN;
	uint16_t MAUX_PIN;
	uint16_t RESET_PIN;
};

/// @brief LoRa Config 構造体. enum内で定義した値を代入する. Add_h Add_l Netid Channel Crypt_h Crypt_l は16進数で指定する.
struct LoRa_Configuration {
	uint8_t Add_h;
	uint8_t Add_l;
	uint8_t Netid;
	uint8_t Channel;
	uint8_t Crypt_h;
	uint8_t Crypt_l;
	uint8_t Uart_baud_rate;
	uint8_t Serial_parity;
	uint8_t Air_data_rate;
	uint8_t Subpacket_size;
	uint8_t Rssi_noise;
	uint8_t Reserved;
	uint8_t Software_mode_func;
	uint8_t Txpower;
	uint8_t Rssi_bytes;
	uint8_t Transmission_method;
	uint8_t Repeater_func;
	uint8_t Lbt_func;
	uint8_t Wor_mode;
	uint8_t Wor_cycle_time;
};


/*
struct LoRa_Configuration {
	// default values
	uint8_t Add_h = 0x00;
	uint8_t Add_l = 0x00;
	uint8_t Netid = 0x00;
	uint8_t Channel = 0x00;
	uint8_t Crypt_h = 0x00;
	uint8_t Crypt_l = 0x00;
	uint8_t Uart_baud_rate = UART_BAUDRATE_9600;
	uint8_t Serial_parity = SERIAL_PARITY_8N1;
	uint8_t Air_data_rate = AIR_DATA_RATE_2400_1;
	uint8_t Subpacket_size = SUB_PACKET_SIZE_240;
	uint8_t Rssi_noise = RSSI_NOISE_DISABLED;
	uint8_t Reserved = LORA_RESERVED;
	uint8_t Software_mode_func = SOFTWARE_MODE_FUNCTION_DISABLED;
	uint8_t Txpower = TX_POWER_22DBM;
	uint8_t Rssi_bytes = RSSI_BYTES_DISABLED;
	uint8_t Transmission_method = TRANSMISSION_METHOD_TRANSPARENT;
	uint8_t Repeater_func = REPEATER_FUNCTION_DISABLED;
	uint8_t Lbt_func = LBT_FUNCTION_DISABLED;
	uint8_t Wor_mode = WOR_MODE_RECEIVER;
	uint8_t Wor_cycle_time = WOR_CYCLE_TIME_1000;
};
*/

#define CMD_SET_REG 0xC0 // COMMAND FOR SETTING REGISTER
#define CMD_READ_REG 0xC1 // COMMAND FOR READING REGISTER
#define CMD_SET_REG_TEMP 0xC2 // COMMAND FOR SETTING TEMPORARY REGISTER
#define REG_ADD_H 0x00 // DEVICE ADDRESS HIGH BYTE
#define REG_ADD_L 0x01 // DEVICE ADDRESS LOW BYTE
#define REG_NETID 0x02 // Network address, used to distingish the the network
#define REG0 0x03 // UART CONFIGURATION REGISTER
#define REG1 0x04 // RF CONFIGURATION REGISTER
#define REG2 0x05 // CHANNEL CONTROL REGISTER
#define REG3 0x06 // TRANSMISSION PARAMETER CONTROL
#define CRYPT_H 0x07 // PASSWORD KEY HIGH BYTE
#define CRYPT_L 0x08 // PASSWORD KEY LOW BYTE
//#define PID 0x80 ~ 0x86 Product information 7 bytes. Read only. 未実装です.

void E22_Config_Init(struct LoRa_Configuration *conf);
int8_t E22_Change_Working_Mode(struct LoRa_Handler *LoRa, uint8_t mode);
void E22_Config_Set(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf);
void E22_Reset(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t mode);
void E22_Baudrate_Set(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t mode);
int8_t E22_Register_Read(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t reg);
int8_t E22_Register_Write(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t reg, uint8_t parameter);
int8_t E22_Register_Read_all(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t *data);
int8_t E22_Payload_Transmit_Transparent(struct LoRa_Handler *LoRa, uint8_t *data, uint16_t datasize);
int8_t E22_Payload_Transmit_Fixed(struct LoRa_Handler *LoRa, uint8_t *data, uint16_t datasize, uint8_t t_addr_h, uint8_t t_addr_l, uint8_t t_channel);
int8_t E22_Payload_Receive(struct LoRa_Handler *LoRa, uint8_t *data, uint16_t datasize);
int8_t E22_Rssi_Get(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, int8_t *rssi, int8_t *snr);

#endif