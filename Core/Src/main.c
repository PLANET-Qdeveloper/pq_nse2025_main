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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_FILES        8                                       // max 32
#define FILE_SIZE            8192
#define FILE_DEBUG            1                                        // Show test file messages, disable for benchmark


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t tick_count = 0;  // Simple tick counter for benchmarking
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  // Flash memory and LittleFS test code
  const char* fn_templ1 = "F%u.tst";
  const char* fn_templ2 = "R%u.tst";
  char fn[32], fn2[32];
  uint8_t buffer[FILE_SIZE]={0};
  lfs_file_t fp;
  uint32_t starttime, runtime;

  
  printf("\n\nOCTOSPI Flash Test, CPU clk=%luMHz\n", HAL_RCC_GetSysClockFreq()/1000000);
  // Initialize XSPI Flash
  if (CSP_XSPI_Init() != HAL_OK) {
      printf("*** CSP_XSPI_INIT Failed\n");
      Error_Handler();
  }
  
  HAL_Delay(100);
  
  printf("\nlittlefs version  = %x\n", LFS_VERSION);
  
  // Read Flash ID
  uint32_t id=0;
  XSPI_ReadID(&id);
  printf("Flash Identifier  = 0x%08lx\n", id);
  
  // Read Unique ID
  uint8_t uid[8]={0};
  XSPI_ReadUniqueID(uid);
  printf("64bits Identifier = 0x");
  for (int i=0; i<8; i++) printf("%02x", uid[i]);
  printf("\n");
  
  // Read SFDP Table
  printf("\nRead SFDP Table:\n");
  uint8_t sfdp[256]={0};
  XSPI_ReadSFDP(sfdp);
  printf("%2c %2c %2c %2c ", sfdp[0], sfdp[1], sfdp[2], sfdp[3]);
  for (int i=4; i<256; i++) {
      if (i%32==0) printf("\n");
      printf("%02x ", sfdp[i]);
  }
  printf("\n\n");
  
  // Optional: Erase entire chip (uncomment if needed)
  // printf("Erasing Chip.....\n");
  // if (CSP_XSPI_Erase_Chip() != HAL_OK) {
  //     printf("*** CSP_XSPI_Erase_Chip Failed\n");
  //     Error_Handler();
  // }
  // printf("Chip Erased\n");
  
  // Start Timer for benchmarking (using system tick)
  HAL_TIM_Base_Start_IT(&htim1);
  
  // Test timer accuracy
  starttime = HAL_GetTick();
  HAL_Delay(500);
  runtime = HAL_GetTick() - starttime;
  if (runtime != 500) printf("Timer clock incorrect? expected 500 got %lu\n", runtime);
  
  printf("\n\nMount littlfs and start timer\n\n");
  starttime = HAL_GetTick();  // Start benchmark timer
  
  // Mount filesystem with format
  stmlfs_mount(true);
  
  //---------------------------------------------------------------------------------------------
  // We'll create NUMBER_OF_FILES files, verify them, rename them, reverify, and delete them.
  //---------------------------------------------------------------------------------------------
  for (int i = 0; i < NUMBER_OF_FILES; i++) {
      sprintf(fn, fn_templ1, i);  // Create file name string
      memset(buffer, i, FILE_SIZE);
      
      int err = stmlfs_file_open(&fp, fn, LFS_O_WRONLY | LFS_O_CREAT);  // Create the file
      if (err < 0) {
          printf("open failed\n");
          Error_Handler();
      }
      
      printf("Write to File %s\n", fn);
      uint32_t wrsize = (uint32_t)stmlfs_file_write(&fp, buffer, FILE_SIZE);  // Write the file
      if (FILE_SIZE != wrsize) {
          printf("write fails, %lu bytes written out of %d\n", wrsize, FILE_SIZE);
          Error_Handler();
      }
      
      if (stmlfs_file_close(&fp) < 0) {  // flush and close the file
          printf("close failed\n");
          Error_Handler();
      }
  }
  
  #ifdef FILE_DEBUG
      dump_dir();  // Show directory
  #endif
  
  stmlfs_unmount();  // Unmount & remount
  stmlfs_mount(false);
  
  struct littlfs_fsstat_t stat;  // Display file system sizes
  stmlfs_fsstat(&stat);
  printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size, (int)stat.blocks_used);
  
  // Rename files test
  for (int i = 0; i < NUMBER_OF_FILES; i++) {
      sprintf(fn, fn_templ1, i);
      sprintf(fn2, fn_templ2, i);
      
      printf("Rename from %s to %s\n", fn, fn2);
      if (stmlfs_rename(fn, fn2) < 0) {  // rename
          printf("rename failed\n");
          fflush(stdout);
          Error_Handler();
      }
  }
  
  #ifdef FILE_DEBUG
      dump_dir();  // Show directory
  #endif
  
  stmlfs_fsstat(&stat);  // Display file system sizes
  printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size, (int)stat.blocks_used);
  
  // Read and verify files test
  for (int i = 0; i < NUMBER_OF_FILES; i++) {
      sprintf(fn, fn_templ1, i);
      sprintf(fn2, fn_templ2, i);
      
      printf("Reopen Filename=%s\n", fn2);
      int err = stmlfs_file_open(&fp, fn2, LFS_O_RDONLY);  // verify the file's content
      if (err < 0) {
          printf("lfs open failed\n");
          Error_Handler();
      } else {
          stmlfs_file_read(&fp, buffer, FILE_SIZE);
          bool err = false;
          for (int j=0; j<FILE_SIZE; j++) if (buffer[j] != i) err = true;
          if (err) printf("Read failed for %d\n", i);
          stmlfs_file_close(&fp);
          
          if (stmlfs_remove(fn2) < 0) {  // Delete the file
              printf("remove failed\n");
              Error_Handler();
          } else printf("File %s removed\n", fn2);
      }
  }
  
  #ifdef FILE_DEBUG
      dump_dir();  // Show directory
  #endif
  
  stmlfs_fsstat(&stat);  // Display file system sizes
  printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size, (int)stat.blocks_used);
  
  stmlfs_unmount();  // Release any resources we were using
  
  runtime = HAL_GetTick() - starttime;
  printf("lfs test done, runtime %lu ms\n", runtime);
  fflush(stdout);
  
  printf("\nEntering standby mode...\n");
  HAL_PWR_EnterSTANDBYMode();  // System stops here
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Simple delay loop if system doesn't enter standby
      HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  HAL_UART_Transmit(&huart4,(uint8_t *)ptr,len,10);
  return len;
}

/* USER CODE END 4 */

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
