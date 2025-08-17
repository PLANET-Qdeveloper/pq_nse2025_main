/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "environment.h"
#include "FreeRTOS.h"
#include "imu.h"
#include "wireless.h"
#include "gnss.h"
#include "state.h"
#include "logger.h"
#include "flash.h"
#include "wireless.h"
#include "float_print.h"
#include "battery.h"
#include "can.h"
#include "state.h"
#include "cyclecounter.h"
#include "valve.h"
#include "pq_com_format/pq_com_format.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct LoRa_Handler LoRaTX;

extern UART_HandleTypeDef huart2;
extern state_t state;
extern env_data_t env_data;
extern imu_data_t imu_data;
extern battery_data_t battery_data;
extern wireless_data_t wireless_data;
extern UART_HandleTypeDef huart1;
extern uint8_t lora_receive_byte;
extern UART_HandleTypeDef huart3;
extern gnss_data_t gnss_data;
uint32_t last_loop_time = 0;
uint32_t average_loop_time = 0;
uint32_t average_loop_time_count = 0;

pq_com_format_t transmit_packet;
uint8_t transmit_data[512];
uint8_t packet_data[255];

typedef struct{
  uint8_t key1;
  uint8_t length1;
  float altitude;
  uint8_t key2;
  uint8_t length2;
  float latitude;
  uint8_t key3;
  uint8_t length3;
  float longitude;
  uint8_t key4;
  uint8_t length4;
  float pressure;
  uint8_t key5;
  uint8_t length5;
  float temperature;
  uint8_t key6;
  uint8_t length6;
  uint8_t state;
  uint8_t key7;
  uint8_t length7;
  uint8_t flags;
  uint8_t key8;
  uint8_t length8;
  uint32_t time;
  uint8_t key9;
  uint8_t length9;
  float voltage;
  uint8_t key10;
  uint8_t length10;
  float x_acc;
  uint8_t key11;
  uint8_t length11;
  float y_acc;
  uint8_t key12;
  uint8_t length12;
  float z_acc;
  uint8_t key13;
  uint8_t length13;
  uint8_t num_sats;
}__attribute__((__packed__)) fast_downlink_data_t;

int init_cplt = 0;
int liftoff = 0;
int nos = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 1024 * 4
};
/* Definitions for communicationTask */
osThreadId_t communicationTaskHandle;
const osThreadAttr_t communicationTask_attributes = {
  .name = "communicationTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for uartPollTask */
osThreadId_t uartPollTaskHandle;
const osThreadAttr_t uartPollTask_attributes = {
  .name = "uartPollTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for gnssTask */
osThreadId_t gnssTaskHandle;
const osThreadAttr_t gnssTask_attributes = {
  .name = "gnssTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for sepTask */
osThreadId_t sepTaskHandle;
const osThreadAttr_t sepTask_attributes = {
  .name = "sepTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for fastDownlinkTask */
osThreadId_t fastDownlinkTaskHandle;
const osThreadAttr_t fastDownlinkTask_attributes = {
  .name = "fastDownlinkTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartDefaultTask, NULL, &mainTask_attributes);

  /* creation of communicationTask */
  communicationTaskHandle = osThreadNew(StartCommunicationTask, NULL, &communicationTask_attributes);

  /* creation of uartPollTask */
  uartPollTaskHandle = osThreadNew(StartUartPollTask, NULL, &uartPollTask_attributes);

  /* creation of gnssTask */
  gnssTaskHandle = osThreadNew(StartGnssTask, NULL, &gnssTask_attributes);

  /* creation of sepTask */
  sepTaskHandle = osThreadNew(StartSepTask, NULL, &sepTask_attributes);

  /* creation of fastDownlinkTask */
  fastDownlinkTaskHandle = osThreadNew(StartFastDownlinkTask, NULL, &fastDownlinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the mainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN mainTask */

  init_wireless();

  init_flash();

  output_log(LOG_LEVEL_IMPORTANT, "Booted core function: %ld", HAL_GetTick());


  // init gnss
  HAL_GPIO_WritePin(RESET_GNSS_GPIO_Port, RESET_GNSS_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(RESET_GNSS_GPIO_Port, RESET_GNSS_Pin, GPIO_PIN_SET);
  init_gnss();
  // init_imu();
  init_env_data();
  can_init();
  HAL_GPIO_WritePin(POW_VALVE_GPIO_Port, POW_VALVE_Pin, GPIO_PIN_SET);
  output_log(LOG_LEVEL_IMPORTANT, "Init completed: %ld", HAL_GetTick());
  init_cplt = 1;
  
  
  /* Infinite loop */
  for(;;)
  {
    volatile uint8_t *reg8 = (volatile uint8_t *)0xE0001000;
    *reg8 = 0x01;
    volatile uint32_t *reg32 = (volatile uint32_t *)0xE0001004;
    *reg32 = 0x00;
    update_env_data();
    // update_imu_data();
    state_check();
    if(wireless_data.data[0] == 'r'){
      state_update(STATE_READY);
    }else if(wireless_data.data[0] == 'e'){
      state_update(STATE_EMERGENCY);
      nos = 1;
    }else if(wireless_data.data[0] == 's'){
      state_update(STATE_SAFETY);
    }else if(wireless_data.data[0] == 'f'){
      state_update(STATE_FLIGHT);
    }else if(wireless_data.data[0] == 'd'){
      state_update(STATE_DECERELATION);
    }else if(wireless_data.data[0] == 'b'){
      state_update(STATE_BURNING);
    }else if(wireless_data.data[0] == 'l'){
      liftoff = 1;
    }
    memset(wireless_data.data, 0, sizeof(wireless_data.data));
    update_battery_data();
    average_loop_time += HAL_GetTick() - last_loop_time;
    average_loop_time_count++;
    last_loop_time = HAL_GetTick();

    if(liftoff == 1){
      HAL_UART_Transmit(&huart3, (uint8_t *)"L", 1, 10);
      liftoff = 0;
    }
    osDelay(10);
  }
  /* USER CODE END mainTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
* @brief Function implementing the communicationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void *argument)
{
  /* USER CODE BEGIN communicationTask */
    while(init_cplt == 0){
      osDelay(100);
    }
  /* Infinite loop */
  for(;;)
  {
    /*
    output_log(LOG_LEVEL_INFO, "{\"time\":%ld, \"state\":%d, \"pressure\":%d.%03d, \"voltage\":%d.%03d, \"delta_press\":%d.%03d}x", HAL_GetTick(), state.state, FLOAT_PRINT(env_data.press_filtered), FLOAT_PRINT(battery_data.voltage), FLOAT_PRINT((env_data.delta_press*100000)));
    osDelay(500);
    output_log(LOG_LEVEL_INFO, "{\"wireless_data\":%s, \"average_loop_time\":%ld, \"liftoff\":%d}", wireless_data.data, average_loop_time * 1000 / average_loop_time_count, liftoff);
    osDelay(500);
    output_log(LOG_LEVEL_INFO, "{\"latitude\":%d.%03d, \"longitude\":%d.%03d, \"altitude\":%d.%03d, \"num_sats\":%d, \"hdop\":%d.%03d}", FLOAT_PRINT(gnss_data.latitude), FLOAT_PRINT(gnss_data.longitude), FLOAT_PRINT(gnss_data.altitude), gnss_data.num_sats, FLOAT_PRINT(gnss_data.hdop));
    */
    average_loop_time = 0;
    average_loop_time_count = 0;
    last_loop_time = HAL_GetTick();

    if(state.state == STATE_DECERELATION){
      uint8_t data[] = "S";
      can_send(0x123, data, 1);
    }
    if(state.state == STATE_READY){
      HAL_UART_Transmit(&huart3, (uint8_t *)"R", 1, 10);
    }else if(state.state == STATE_SAFETY){
      HAL_UART_Transmit(&huart3, (uint8_t *)"S", 1, 10);
    }else if(nos == 1){
      HAL_UART_Transmit(&huart3, (uint8_t *)"E", 1, 10);
      nos = 0;
    }


    wireless_data.size = 0;
    memset(wireless_data.data, 0, sizeof(wireless_data.data));
    osDelay(500);

  }
  /* USER CODE END communicationTask */
}

/* USER CODE BEGIN Header_StartUartPollTask */
/**
* @brief Function implementing the uartPollTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartPollTask */
void StartUartPollTask(void *argument)
{
  /* USER CODE BEGIN uartPollTask */
  while(init_cplt == 0){
    osDelay(100);
  }
  /* Infinite loop */
  for(;;)
  {
    if(state.state == STATE_READY || state.state == STATE_EMERGENCY){
      uint8_t data[1] = {0};
      HAL_UART_Receive(&huart2, data, 1, 1);
      if(data[0] == 'L'){
        liftoff = 1;
      }else if(data[0] == 'E'){
        state_update(STATE_EMERGENCY);
        nos = 1;
      }
    }
    check_wireless();
    valve_check();

    
    osDelay(1);
  }
  /* USER CODE END uartPollTask */
}

/* USER CODE BEGIN Header_StartGnssTask */
/**
* @brief Function implementing the gnssTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGnssTask */
void StartGnssTask(void *argument)
{
  /* USER CODE BEGIN gnssTask */
  while(init_cplt == 0){
    osDelay(100);
  }
  /* Infinite loop */
  for(;;)
  {
    update_gnss_data();
    osDelay(1000);
  }
  /* USER CODE END gnssTask */
}

/* USER CODE BEGIN Header_StartSepTask */
/**
* @brief Function implementing the sepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSepTask */
void StartSepTask(void *argument)
{
  /* USER CODE BEGIN sepTask */
  while(init_cplt == 0){
    osDelay(100);
  }
  /* Infinite loop */
  for(;;)
  {
    uint8_t data[4] = {0};
    if(state.state == STATE_DECERELATION || state.state == STATE_LANDED){
      memcpy(data, &env_data.press_filtered, 4);
      can_send(0x124, data, 4);
    }
    osDelay(100);
  }
  /* USER CODE END sepTask */
}

/* USER CODE BEGIN Header_StartFastDownlinkTask */
/**
* @brief Function implementing the fastDownlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFastDownlinkTask */
void StartFastDownlinkTask(void *argument)
{
  /* USER CODE BEGIN fastDownlinkTask */
  while(init_cplt == 0){
    osDelay(100);
  }
  int count = 100;
  /* Infinite loop */
  for(;;)
  {
      fast_downlink_data_t downlink_data = {};
      pq_com_format_clear(&transmit_packet);
      transmit_packet.source_id = 0x0A;
      transmit_packet.destination_id = 0x0B;
      transmit_packet.payload_length = sizeof(fast_downlink_data_t);
      
      downlink_data.altitude = gnss_data.altitude;
      downlink_data.latitude = gnss_data.latitude;
      downlink_data.longitude = gnss_data.longitude;
      downlink_data.pressure = env_data.press;
      downlink_data.temperature = env_data.temp;
      downlink_data.state = state.state;
      downlink_data.flags = (liftoff << 1) | nos;
      downlink_data.time = HAL_GetTick();
      downlink_data.voltage = battery_data.voltage;
      downlink_data.x_acc = imu_data.accel_x;
      downlink_data.y_acc = imu_data.accel_y;
      downlink_data.z_acc = imu_data.accel_z;
      downlink_data.num_sats = (uint8_t)gnss_data.num_sats;


      downlink_data.length1 = sizeof(downlink_data.altitude);
      downlink_data.length2 = sizeof(downlink_data.latitude);
      downlink_data.length3 = sizeof(downlink_data.longitude);
      downlink_data.length4 = sizeof(downlink_data.pressure);
      downlink_data.length5 = sizeof(downlink_data.temperature);
      downlink_data.length6 = sizeof(downlink_data.state);
      downlink_data.length7 = sizeof(downlink_data.flags);
      downlink_data.length8 = sizeof(downlink_data.time);
      downlink_data.length9 = sizeof(downlink_data.voltage);
      downlink_data.length10 = sizeof(downlink_data.x_acc);
      downlink_data.length11 = sizeof(downlink_data.y_acc);
      downlink_data.length12 = sizeof(downlink_data.z_acc);
      downlink_data.length13 = sizeof(uint8_t);

      downlink_data.key1 = 0xA1;
      downlink_data.key2 = 0xA2;
      downlink_data.key3 = 0xA3;
      downlink_data.key4 = 0xA4;
      downlink_data.key5 = 0xA5;
      downlink_data.key6 = 0xA6;
      downlink_data.key7 = 0xA7;
      downlink_data.key8 = 0xA8;
      downlink_data.key9 = 0xA9;
      downlink_data.key10 = 0xAA;
      downlink_data.key11 = 0xAB;
      downlink_data.key12 = 0xAC;
      downlink_data.key13 = 0xAD;

      memcpy(transmit_packet.payload, &downlink_data, sizeof(fast_downlink_data_t));
      pq_com_format_encode_result_t res;
      for(int i = 0; i < 512; i++){
        uint8_t transmit_data_segment;
        res = pq_com_format_encode(&transmit_packet, &transmit_data_segment);
        transmit_data[i] = transmit_data_segment;
        if(res == PQ_COM_FORMAT_ENCODE_COMPLETED){
          if(count == 0){
            send_data(transmit_data, i+1);
            if(state.state == STATE_BURNING || state.state == STATE_DECERELATION || state.state == STATE_FLIGHT || state.state == STATE_LANDED){
              count = 10;
            }else{
              count = 100;
            }
          }
          write_flash_log(transmit_data, i+1);
          break;
        }else if (res == PQ_COM_FORMAT_ENCODE_ERROR_UNDEFINED){
          break;
        }
      }
      count--;
      osDelay(10);
  }
  /* USER CODE END fastDownlinkTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

