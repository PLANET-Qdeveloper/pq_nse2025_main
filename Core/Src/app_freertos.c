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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct LoRa_Handler LoRaTX;
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
int init_cplt = 0;
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
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
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
  init_imu();
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
    update_imu_data();
    state_check();
    if(wireless_data.data[0] == 'r'){
      state_update(STATE_READY);
    }else if(wireless_data.data[0] == 'e'){
      state_update(STATE_EMERGENCY);
    }else if(wireless_data.data[0] == 's'){
      state_update(STATE_SAFETY);
    }else if(wireless_data.data[0] == 'f'){
      state_update(STATE_FLIGHT);
    }else if(wireless_data.data[0] == 'd'){
      state_update(STATE_DECERELATION);
    }else if(wireless_data.data[0] == 'b'){
      state_update(STATE_BURNING);
    }else if(wireless_data.data[0] == 'l'){
      HAL_UART_Transmit(&huart3, (uint8_t *)"L", 1, 10);
    }
    update_battery_data();
    average_loop_time += HAL_GetTick() - last_loop_time;
    average_loop_time_count++;
    last_loop_time = HAL_GetTick();
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

    output_log(LOG_LEVEL_IMPORTANT, "{\"time\":%ld, \"state\":%d, \"pressure\":%d.%03d, \"voltage\":%d.%03d, \"delta_press\":%d.%03d}x", HAL_GetTick(), state.state, FLOAT_PRINT(env_data.press_filtered), FLOAT_PRINT(battery_data.voltage), FLOAT_PRINT((env_data.delta_press*100000)));
    osDelay(500);
    output_log(LOG_LEVEL_IMPORTANT, "{\"wireless_data\":%s, \"average_loop_time\":%ld}", wireless_data.data, average_loop_time * 1000 / average_loop_time_count);
    osDelay(500);
    output_log(LOG_LEVEL_IMPORTANT, "{\"latitude\":%d.%03d, \"longitude\":%d.%03d, \"altitude\":%d.%03d, \"num_sats\":%d, \"hdop\":%d.%03d}", FLOAT_PRINT(gnss_data.latitude), FLOAT_PRINT(gnss_data.longitude), FLOAT_PRINT(gnss_data.altitude), gnss_data.num_sats, FLOAT_PRINT(gnss_data.hdop));
    average_loop_time = 0;
    average_loop_time_count = 0;
    last_loop_time = HAL_GetTick();

    if(state.state == STATE_DECERELATION){
      uint8_t data[] = "S";
      can_send(0x123, data, 1);
    }
    if(state.state == STATE_READY){
      HAL_UART_Transmit(&huart3, (uint8_t *)"R", 1, 10);
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
    check_wireless();
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
    memcpy(data, &env_data.press_filtered, 4);
    can_send(0x124, data, 4);
    osDelay(100);
  }
  /* USER CODE END sepTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

