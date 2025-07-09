/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef	uint8_t u8;/**< used for unsigned 8bit */
typedef	uint16_t u16;/**< used for unsigned 16bit */
typedef	uint32_t u32;/**< used for unsigned 32bit */
typedef	uint64_t u64;/**< used for unsigned 64bit */

/*signed integer types*/
typedef	int8_t s8;/**< used for signed 8bit */
typedef	int16_t s16;/**< used for signed 16bit */
typedef	int32_t s32;/**< used for signed 32bit */
typedef	int64_t s64;/**< used for signed 64bit */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_IMU_Pin GPIO_PIN_13
#define RESET_IMU_GPIO_Port GPIOC
#define M0_Pin GPIO_PIN_0
#define M0_GPIO_Port GPIOC
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOA
#define EMOUT_Pin GPIO_PIN_4
#define EMOUT_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_5
#define M1_GPIO_Port GPIOC
#define SS2_Pin GPIO_PIN_1
#define SS2_GPIO_Port GPIOB
#define POW_COM_Pin GPIO_PIN_6
#define POW_COM_GPIO_Port GPIOC
#define RESET_GNSS_Pin GPIO_PIN_7
#define RESET_GNSS_GPIO_Port GPIOC
#define POW_VALVE_Pin GPIO_PIN_8
#define POW_VALVE_GPIO_Port GPIOC
#define PG_Pin GPIO_PIN_9
#define PG_GPIO_Port GPIOC
#define AUX_Pin GPIO_PIN_8
#define AUX_GPIO_Port GPIOA
#define RESET_LORA_Pin GPIO_PIN_9
#define RESET_LORA_GPIO_Port GPIOA
#define LOOUT_Pin GPIO_PIN_10
#define LOOUT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
