/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "fdcan.h"
#include "i2c.h"
#include "memorymap.h"
#include "octospi.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "imu.h"
#include "gnss.h"
#include "environment.h"
#include "wireless.h"
#include "logger.h"
#include "FreeRTOS.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern struct bno055_t bno055;
extern struct bme280_t bme280;
extern struct LoRa_Handler LoRaTX;
volatile uint8_t lora_receive_byte[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Set_OSPI_MemoryMappedMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern XSPI_HandleTypeDef hospi1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // Flash memory and LittleFS test code
  /// 0x1
  

  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in app_freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 99;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 1024;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_CSI;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 60;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 20;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVP|RCC_PLL2_DIVQ;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2Q;
  PeriphClkInitStruct.Spi2ClockSelection = RCC_SPI2CLKSOURCE_PLL2P;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI8_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI8_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);
}

/* USER CODE BEGIN 4 */
void Set_OSPI_MemoryMappedMode(void)
{
	XSPI_RegularCmdTypeDef sCommand = {0};
	XSPI_MemoryMappedTypeDef sMemMappedCfg = {0};
	uint8_t reg_data =0;

	/* Enable Reset --------------------------- */
	/* Common Commands */
	sCommand.OperationType      		= HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.IOSelect           		= HAL_XSPI_SELECT_IO_3_0;
	sCommand.InstructionDTRMode 		= HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode     		= HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.DataDTRMode				= HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DQSMode            		= HAL_XSPI_DQS_DISABLE;
	sCommand.SIOOMode          			= HAL_XSPI_SIOO_INST_EVERY_CMD;
	sCommand.AlternateBytesMode 		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytes				= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesWidth		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesDTRMode		= HAL_XSPI_ALT_BYTES_DTR_DISABLE;
	sCommand.InstructionMode   			= HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionWidth    		= HAL_XSPI_INSTRUCTION_8_BITS;
	sCommand.AddressWidth 				= HAL_XSPI_ADDRESS_24_BITS;
	/* Instruction */
	sCommand.Instruction 				= 0x66;	/* Reset Enable W25Q128JVSIQ */
	/* Address */
	sCommand.AddressMode       			= HAL_XSPI_ADDRESS_NONE;
	sCommand.Address					= 0;						
	/* Data */
	sCommand.DataMode          			= HAL_XSPI_DATA_NONE;
	sCommand.DataLength       			= 0;
	sCommand.DummyCycles       			= 0;				
	
	if (HAL_XSPI_Command(&hospi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		for(;;);
	}
	
	/* Reset Device --------------------------- */
	/* Common Commands */
	sCommand.OperationType      		= HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.IOSelect           		= HAL_XSPI_SELECT_IO_3_0;
	sCommand.InstructionDTRMode 		= HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode     		= HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.DataDTRMode				= HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DQSMode            		= HAL_XSPI_DQS_DISABLE;
	sCommand.SIOOMode          			= HAL_XSPI_SIOO_INST_EVERY_CMD;
	sCommand.AlternateBytesMode 		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytes				= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesWidth		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesDTRMode		= HAL_XSPI_ALT_BYTES_DTR_DISABLE;
	sCommand.InstructionMode   			= HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionWidth    		= HAL_XSPI_INSTRUCTION_8_BITS;
	sCommand.AddressWidth 				= HAL_XSPI_ADDRESS_24_BITS;
	/* Instruction */
	sCommand.Instruction 				= 0x99;	/* Reset W25Q128JVSIQ */
	/* Address */
	sCommand.AddressMode       			= HAL_XSPI_ADDRESS_NONE;
	sCommand.Address					= 0;
	/* Data */
	sCommand.DataMode          			= HAL_XSPI_DATA_NONE;
	sCommand.DataLength       			= 0;
	sCommand.DummyCycles       			= 0;
	
	if (HAL_XSPI_Command(&hospi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		for(;;);
	}

	/* Enter Quad-SPI Mode --------------------------- */
	/* Common Commands */
	sCommand.OperationType      		= HAL_XSPI_OPTYPE_COMMON_CFG;
	sCommand.IOSelect           		= HAL_XSPI_SELECT_IO_3_0;
	sCommand.InstructionDTRMode 		= HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode     		= HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.DataDTRMode				= HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DQSMode            		= HAL_XSPI_DQS_DISABLE;
	sCommand.SIOOMode          			= HAL_XSPI_SIOO_INST_EVERY_CMD;
	sCommand.AlternateBytesMode 		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytes				= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesWidth		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesDTRMode		= HAL_XSPI_ALT_BYTES_DTR_DISABLE;
	sCommand.InstructionMode   			= HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionWidth    		= HAL_XSPI_INSTRUCTION_8_BITS;
	sCommand.AddressWidth 				= HAL_XSPI_ADDRESS_24_BITS;
	/* Instruction */
	sCommand.Instruction 				= 0x31;	/* Set Status2 W25Q128JVSIQ */
	/* Address */
	sCommand.AddressMode       			= HAL_XSPI_ADDRESS_NONE;
	sCommand.Address					= 0;
	/* Data */
	sCommand.DataMode          			= HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.DataLength       			= 1;
	sCommand.DummyCycles       			= 0;
	reg_data 							= 0x02;	/* Enable QuadI/O Mode */
	
	if (HAL_XSPI_Command(&hospi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		for(;;);
	}
	uint8_t dummy_data = {0};
	if (HAL_XSPI_Transmit(&hospi1, &dummy_data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		for(;;);
	}

	/* Enter MemoryMappedMode --------------------------- */
	/* Read Commands */
	sCommand.OperationType      		= HAL_XSPI_OPTYPE_READ_CFG;
	sCommand.IOSelect           		= HAL_XSPI_SELECT_IO_3_0;
	sCommand.InstructionDTRMode 		= HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode     		= HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.DataDTRMode				= HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DQSMode            		= HAL_XSPI_DQS_DISABLE;
	sCommand.SIOOMode          			= HAL_XSPI_SIOO_INST_EVERY_CMD;
	sCommand.AlternateBytesMode 		= HAL_XSPI_ALT_BYTES_4_LINES;
	sCommand.AlternateBytes				= 0xFF;	/* Need for Fast Read QUAD W25Q128JVSIQ */
	sCommand.AlternateBytesWidth		= HAL_XSPI_ALT_BYTES_8_BITS;
	sCommand.AlternateBytesDTRMode		= HAL_XSPI_ALT_BYTES_DTR_DISABLE;
	sCommand.InstructionMode   			= HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionWidth    		= HAL_XSPI_INSTRUCTION_8_BITS;
	sCommand.AddressWidth				= HAL_XSPI_ADDRESS_24_BITS;
	/* Instruction */
	sCommand.Instruction 				= 0xEB;	/* Fast Read QUAD W25Q128JVSIQ */
	/* Address */
	sCommand.AddressMode       			= HAL_XSPI_ADDRESS_4_LINES;
	sCommand.Address					= 0;								
	/* Data */	
	sCommand.DataMode          			= HAL_XSPI_DATA_4_LINES;
	sCommand.DataLength       			= 0;
	sCommand.DummyCycles       			= 4;	/* DUMMY 4Cycle for Fast Read QUAD W25Q128JVSIQ */

	if(HAL_XSPI_Command(&hospi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		for(;;);
	}
	
	/* Write Commands */
	sCommand.OperationType      		= HAL_XSPI_OPTYPE_WRITE_CFG;
	sCommand.IOSelect           		= HAL_XSPI_SELECT_IO_3_0;
	sCommand.InstructionDTRMode 		= HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode     		= HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.DataDTRMode				= HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DQSMode            		= HAL_XSPI_DQS_DISABLE;
	sCommand.SIOOMode          			= HAL_XSPI_SIOO_INST_EVERY_CMD;
	sCommand.AlternateBytesMode 		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytes				= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesWidth		= HAL_XSPI_ALT_BYTES_NONE;
	sCommand.AlternateBytesDTRMode		= HAL_XSPI_ALT_BYTES_DTR_DISABLE;
	sCommand.InstructionMode   			= HAL_XSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionWidth    		= HAL_XSPI_INSTRUCTION_8_BITS;
	sCommand.AddressWidth				= HAL_XSPI_ADDRESS_24_BITS;
	/* Instruction */
	sCommand.Instruction 				= 0x32;	/* Page Write QUAD W25Q128JVSIQ */
	/* Address */
	sCommand.AddressMode       			= HAL_XSPI_ADDRESS_1_LINE;
	sCommand.Address					= 0;		
	/* Data */
	sCommand.DataMode          			= HAL_XSPI_DATA_4_LINES;
	sCommand.DataLength       			= 0;
	sCommand.DummyCycles       			= 0;							
	
	if(HAL_XSPI_Command(&hospi1, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		for(;;);
	}

	/* Set OCTO-SPI as MemoryMappedMode */
	sMemMappedCfg.TimeOutActivation 	= HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
	sMemMappedCfg.TimeoutPeriodClock 	= 0;
	if(HAL_XSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
	{
        for(;;);
    }

}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int _write(int file, char *ptr, int len)
{
  //HAL_UART_Transmit(&huart4,(uint8_t *)ptr,len,10);
  return len;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (huart->Instance == USART1)
  {
    add_buffer_wireless(lora_receive_byte, 1);
    HAL_UART_Receive_IT(&huart1, lora_receive_byte, 1);
  }
  if(xHigherPriorityTaskWoken == pdTRUE)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
