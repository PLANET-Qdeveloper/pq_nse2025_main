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
#include "apogee.h"
#include "wireless.h"
#include "float_print.h"
#include "battery.h"
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
  init_imu();
  init_env_data();
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
    if(wireless_data.data[0] == 'r'){
      state_update(STATE_READY);

    }else if(wireless_data.data[0] == 'e'){
      state_update(STATE_EMERGENCY);
    }else if(wireless_data.data[0] == 's'){
      state_update(STATE_SAFETY);
    }else if(wireless_data.data[0] == 'f'){
      state_update(STATE_FLIGHT);
    }
    output_log(LOG_LEVEL_IMPORTANT, "{\"time\":%ld, \"state\":%d, \"pressure\":%d.%03d, \"voltage\":%d.%03d, \"delta_press\":%d.%03d}", HAL_GetTick(), state.state, FLOAT_PRINT(env_data.press_filtered), FLOAT_PRINT(battery_data.voltage), FLOAT_PRINT((env_data.delta_press*100000)));
    osDelay(500);
    output_log(LOG_LEVEL_IMPORTANT, "{\"wireless_data\":%s, \"average_loop_time\":%ld}", wireless_data.data, average_loop_time / average_loop_time_count);
    average_loop_time = 0;
    average_loop_time_count = 0;
    last_loop_time = HAL_GetTick();
    
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

