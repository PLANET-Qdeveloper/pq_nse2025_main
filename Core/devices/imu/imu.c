#include "imu.h"

imu_data_t imu_data;
extern I2C_HandleTypeDef hi2c1;

int init_imu(){
    HAL_GPIO_WritePin(RESET_IMU_GPIO_Port, RESET_IMU_Pin, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(RESET_IMU_GPIO_Port, RESET_IMU_Pin, GPIO_PIN_SET);

  while(1) {
  int result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(BNO055_I2C_ADDR1<<1), 2, 2);
  if(result == HAL_OK) {
    break;
  }
  osDelay(100);
  }

  I2C_routine();
  s32 comres = bno055_init(&bno055);
  
  comres = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);


  if(comres != BNO055_SUCCESS) {
    output_log(LOG_LEVEL_ERROR, "bno055 init failed %d\n", comres);
    Error_Handler();
  }
  return comres;
}
int update_imu_data()
{
    return 0;
}