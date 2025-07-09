#include "main.h"
#include "E22_900T22S.h"

/*
 * Created on: April 26, 2025
 * Author: tsuru
 * 注意！E220シリーズには互換性がありません registerの指定アドレスが異なるからです.
 * E22は電力効率は良くないですが、より長距離で通信可能(らしい)です
 * 
 */



/// @brief Lora_Configurationに初期値を代入する
/// @param conf 定義された構造体(LoRa_Configuration)
void E22_Config_Init(struct LoRa_Configuration *conf) {
	conf->Add_h = 0x00;
	conf->Add_l = 0x00;
	conf->Channel = 0x46;
	conf->Crypt_h = 0x00;
	conf->Crypt_l = 0x00;
	conf->Uart_baud_rate = UART_BAUDRATE_9600;
	conf->Serial_parity = SERIAL_PARITY_8N1;
	conf->Air_data_rate = AIR_DATA_RATE_2400_1;
	conf->Subpacket_size = SUB_PACKET_SIZE_240;
	conf->Rssi_noise = RSSI_NOISE_DISABLED;
	conf->Reserved = LORA_RESERVED;
	conf->Software_mode_func = SOFTWARE_MODE_FUNCTION_DISABLED;
	conf->Txpower = TX_POWER_22DBM;
	conf->Rssi_bytes = RSSI_BYTES_DISABLED;
	conf->Transmission_method = TRANSMISSION_METHOD_TRANSPARENT;
	conf->Wor_mode = WOR_MODE_RECEIVER;
	conf->Wor_cycle_time = WOR_CYCLE_TIME_1000;
}


/// @brief モジュールの動作モード変更
/// @param LoRa LoRaハンドラ構造体
/// @param mode Normal=0, WOR=1, Configuration=2, Deep_Sleep=3
/// @return 成功=1, エラー=0
int8_t E22_Change_Working_Mode(struct LoRa_Handler *LoRa, uint8_t mode) {

    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
    osDelay(50);

    switch (mode) {
        case 0:
            HAL_GPIO_WritePin(LoRa->M0_GPIO_PORT, LoRa->M0_PIN, RESET);
            HAL_GPIO_WritePin(LoRa->M1_GPIO_PORT, LoRa->M1_PIN, RESET);
            osDelay(50);
            break;
        case 1:
            HAL_GPIO_WritePin(LoRa->M0_GPIO_PORT, LoRa->M0_PIN, SET);
            HAL_GPIO_WritePin(LoRa->M1_GPIO_PORT, LoRa->M1_PIN, RESET);
            osDelay(50);
            break;
        case 2:
            HAL_GPIO_WritePin(LoRa->M0_GPIO_PORT, LoRa->M0_PIN, RESET);
            HAL_GPIO_WritePin(LoRa->M1_GPIO_PORT, LoRa->M1_PIN, SET);
            osDelay(50);
            break;
        case 3:
            HAL_GPIO_WritePin(LoRa->M0_GPIO_PORT, LoRa->M0_PIN, SET);
            HAL_GPIO_WritePin(LoRa->M1_GPIO_PORT, LoRa->M1_PIN, SET);
            osDelay(50);
            break;
        default:
            HAL_GPIO_WritePin(LoRa->M0_GPIO_PORT, LoRa->M0_PIN, RESET);
            HAL_GPIO_WritePin(LoRa->M1_GPIO_PORT, LoRa->M1_PIN, RESET);
            osDelay(50);
            return 0;
    }
    return 1;
}

/// @brief UARTのbaudrateを変更して適用する. Configモードは9600のみ受け付けるため、それに使用.
/// @param LoRa LoRaハンドラ
/// @param conf LoRa Config コンストラクタ
/// @param mode Baudrateを enum_UART_BAUDRATE で指定.
void E22_Baudrate_Set(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t mode) {
    /// busy状態からぬけるのを待つ
    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
    osDelay(50);
    /// HAL_UART_DEINIT
    HAL_UART_DeInit(LoRa->UART_HANDLER);
    switch (mode) {
        case UART_BAUDRATE_1200:
            LoRa->UART_HANDLER->Init.BaudRate = 1200;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_2400:
            LoRa->UART_HANDLER->Init.BaudRate = 2400;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_4800:
            LoRa->UART_HANDLER->Init.BaudRate = 4800;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_9600:
            LoRa->UART_HANDLER->Init.BaudRate = 9600;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_19200:
            LoRa->UART_HANDLER->Init.BaudRate = 19200;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_38400:
            LoRa->UART_HANDLER->Init.BaudRate = 38400;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_57600:
            LoRa->UART_HANDLER->Init.BaudRate = 57600;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        case UART_BAUDRATE_115200:
            LoRa->UART_HANDLER->Init.BaudRate = 115200;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
        default:
            LoRa->UART_HANDLER->Init.BaudRate = 9600;
            HAL_UART_Init(LoRa->UART_HANDLER);
            break;
    }

}


void E22_Config_Set(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf) {
    /// busy状態から抜けるのを待つ
    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
    osDelay(500);
    /// 動作モードをConfigに変更する
    E22_Change_Working_Mode(LoRa, 3);
    /// baudratgeを9600に変更する.
    E22_Baudrate_Set(LoRa, conf, UART_BAUDRATE_9600);
    /// sendデータの設定
    uint8_t send_data[11] = {
        CMD_SET_REG,
        0x00,
        0x08,
        conf->Add_h,
        conf->Add_l,
        conf->Uart_baud_rate | conf->Serial_parity | conf->Air_data_rate,
        conf->Subpacket_size | conf->Reserved | conf->Rssi_noise | conf->Txpower | conf->Software_mode_func,
        conf->Channel,
        conf->Rssi_bytes | conf->Transmission_method | conf->Wor_mode | conf->Wor_cycle_time,
        conf->Crypt_h,
        conf->Crypt_l
    };
    /// 送信
    HAL_UART_Transmit(LoRa->UART_HANDLER, send_data, 11, 500);
    /// 遅延
    osDelay(500);
    /// Normalモードにもどす
    E22_Change_Working_Mode(LoRa, 0);
    /// 遅延
    osDelay(100);
    /// UART再起動
    E22_Baudrate_Set(LoRa, conf, conf->Uart_baud_rate);
    
}


/// @brief モジュールのリセットを行う関数
/// @param LoRa LoRaハンドラ
/// @param conf Config
/// @param mode [関数拡張用]
void E22_Reset(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t mode) {
    // reset_pinをlow
    HAL_GPIO_WritePin(LoRa->RESET_GPIO_PORT, LoRa->RESET_PIN, RESET);
    /// 遅延
    osDelay(100);
    // reset_pinをhigh
    HAL_GPIO_WritePin(LoRa->RESET_GPIO_PORT, LoRa->RESET_PIN, SET);
}


// regを読んで値を吐くfunc
int8_t E22_Register_Read(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t reg) {
    
    /// busy状態から抜けるのを待つ
    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
    osDelay(50);

    /// 動作モードをConfigに変更する
    E22_Change_Working_Mode(LoRa, 3);

    /// baudratgeを9600に変更する.
    E22_Baudrate_Set(LoRa, conf, UART_BAUDRATE_9600);

    // send_data, receive_dataはmoduleからregを読む用
    uint8_t send_data[3]={CMD_READ_REG, reg, 1};
    uint8_t receive_data[4]={0};

    // 読み取りコマンド送信
    HAL_UART_Transmit(LoRa->UART_HANDLER, send_data, 3, 100);

    // モジュールがbusy状態から抜けるのを待つ
    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);

    // Receive data
    HAL_UART_Receive(LoRa->UART_HANDLER, receive_data, 4, 100);

    /// 遅延
    osDelay(50);
    /// Normalモードにもどす
    E22_Change_Working_Mode(LoRa, 0);
    /// 遅延
    osDelay(50);
    /// UART再起動
    E22_Baudrate_Set(LoRa, conf, conf->Uart_baud_rate);

    // Error検知
    if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1) {
        //読んだ値返す
        return receive_data[3];
    } else {
        // Error
        return -1;
    }

}


// regを書き込む関数
int8_t E22_Register_Write(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t reg, uint8_t parameter) {
    /// busy状態から抜けるのを待つ
    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
    osDelay(50);

    // Configモードにする
    E22_Change_Working_Mode(LoRa, 3);

    /// baudratgeを9600に変更する.
    E22_Baudrate_Set(LoRa, conf, UART_BAUDRATE_9600);

    // Delay
    osDelay(3);

    // send_data, receive_dataを設定
    uint8_t send_data[4]={CMD_SET_REG,reg,1,parameter};
	uint8_t receive_data[4]={0};
    
    // データの送受信
    HAL_UART_Transmit(LoRa->UART_HANDLER,send_data ,4, 100);
	HAL_UART_Receive(LoRa->UART_HANDLER, receive_data, 4, 100);

    // baudrateを戻す
    E22_Baudrate_Set(LoRa, conf, conf->Uart_baud_rate);

    // Normalモードに戻す
    E22_Change_Working_Mode(LoRa, 0);

    // Error検知
    if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1 && receive_data[3] == parameter) {
        return receive_data[3];
    } else {
        // Error
        return -1;
    }

}


// すべてのregを読む
int8_t E22_Register_Read_all(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, uint8_t *data) {
    //  iを0~7まで移動させてすべての設定値を読む
    for(int i=0; i<8; i++) {
        data[i]=E22_Register_Read(LoRa, conf, i);
		osDelay(2);
		if((int8_t)data[i]==(int8_t)-1) {
            // E22_read_register関数がエラーを吐いたらこちらもエラー吐く
            return -1;
        }
    }
    //return data <-後で変更
    return 1;
}


/// @brief Transparentモードで送信する関数.
/// @param LoRa 
/// @param data 
/// @param datasize 
/// @return 
int8_t E22_Payload_Transmit_Transparent(struct LoRa_Handler *LoRa, uint8_t *data, uint16_t datasize) {
    // dataを送信
    HAL_UART_Transmit(LoRa->UART_HANDLER, data ,datasize, 100);
	return 1;
}

/// @brief チャンネルとアドレスを指定して送信する関数
/// @param LoRa LoRaハンドラ
/// @param data 送信データ
/// @param datasize 送信データサイズ
/// @param t_addr_h ターゲットアドレス_ハイバイト
/// @param t_addr_l ターゲットアドレス_ローバイト
/// @param t_channel ターゲットチャンネル(周波数)
/// @return 
int8_t E22_Payload_Transmit_Fixed(struct LoRa_Handler *LoRa, uint8_t *data, uint16_t datasize, uint8_t t_addr_h, uint8_t t_addr_l, uint8_t t_channel) {
    // 送信先パラメータの設定
    uint8_t send_data[datasize + 3];
    send_data[0] = t_addr_h;
    send_data[1] = t_addr_l;
    send_data[2] = t_channel;
    for(int i = 0; i < datasize; i++) {
        send_data[i + 3] = data[i];
    }
    // 送信
    HAL_UART_Transmit(LoRa->UART_HANDLER, send_data, datasize+3, 100);
    return 1;

}

/// @brief 受信する関数
/// @param LoRa 
/// @param data 
/// @param datasize 
/// @return 
int8_t E22_Payload_Receive(struct LoRa_Handler *LoRa, uint8_t *data, uint16_t datasize) {
    //while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
	HAL_UART_Receive(LoRa->UART_HANDLER, data, datasize, 100);
	return 1;
}

/// @brief RSSI値を取得
/// @param LoRa LoRa_Handler
/// @param rssi 取得したRSSI値を格納.
/// @param snr SNR値を格納.
/// @return 
int8_t E22_Rssi_Get(struct LoRa_Handler *LoRa, struct LoRa_Configuration *conf, int8_t *rssi, int8_t *snr) {
    if(conf->Rssi_noise == RSSI_NOISE_ENABLE) {
        /// send_data
        uint8_t send_data[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
        uint8_t receive_data[5] = {0};
        
        ///書き込み
        HAL_UART_Transmit(LoRa->UART_HANDLER, send_data, 6, 100);

        /// 待機
        /// while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
        osDelay(50);

        /// 読み込み
        HAL_UART_Receive(LoRa->UART_HANDLER, receive_data, 5, 100);

        /// エラーチェック＆計算
        if(receive_data[0]==0xC1 && receive_data[1]==0x00 && receive_data[2]==0x02) {
            *rssi = -(256 - receive_data[3]);
            *snr = -(256 - receive_data[4]) + (256 - receive_data[3]);
            return 1;
        } else {
            return 0;
        }
    } else {
        return -1;
    }
}
