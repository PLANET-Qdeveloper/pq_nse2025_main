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
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_ACCMAG);
  comres += bno055_set_accel_range(BNO055_ACCEL_RANGE_8G);
  comres += bno055_set_accel_bw(BNO055_ACCEL_BW_250HZ);
  comres += bno055_set_accel_power_mode(BNO055_POWER_MODE_NORMAL);
  comres += bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
  comres += bno055_set_gyro_range(BNO055_GYRO_RANGE_2000DPS);
  comres += bno055_set_gyro_bw(BNO055_GYRO_BW_230HZ);
  comres += bno055_set_gyro_power_mode(BNO055_POWER_MODE_NORMAL);
  comres += bno055_set_gyro_unit(BNO055_GYRO_UNIT_DPS);


  if(comres != BNO055_SUCCESS) {
    output_log(LOG_LEVEL_ERROR, "bno055 init failed %d\n", comres);
    Error_Handler();
  }
  return comres;
}
int update_imu_data()
{
  struct bno055_accel_double_t accel_xyz;
  struct bno055_gyro_double_t gyro_xyz;
  bno055_convert_double_accel_xyz_msq(&accel_xyz);
  bno055_convert_double_gyro_xyz_dps(&gyro_xyz);
  imu_data.accel_x = (float)accel_xyz.x;
  imu_data.accel_y = (float)accel_xyz.y;
  imu_data.accel_z = (float)accel_xyz.z;
  imu_data.gyro_x = (float)gyro_xyz.x;
  imu_data.gyro_y = (float)gyro_xyz.y;
  imu_data.gyro_z = (float)gyro_xyz.z;
    return 0;
}