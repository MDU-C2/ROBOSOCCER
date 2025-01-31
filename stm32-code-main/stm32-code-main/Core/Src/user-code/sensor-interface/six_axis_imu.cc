/* nine_axis_imu.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the six axis WSEN_ISDS IMU. This driver
 * functions as a wrapper for Wurth Elektronik's WSEN_ISDS driver which can be
 * found here:
 * https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_ISDS_2536030320001
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "../../user-code/sensor-interface/six_axis_imu.h"

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../../Drivers/WSEN_ISDS_2536030320001/WSEN_ISDS_2536030320001.h"

namespace stm32_code
{
namespace sensor_interface
{

/* Default constructor */
SixAxisImu::SixAxisImu() {}

/* Configure the sensor to begin taking measuremets */
Status SixAxisImu::Init(I2C_HandleTypeDef* i2c_handle)
{
  uint8_t device_id_value = 0;
  ISDS_state_t sw_reset;

  ISDS_getDefaultInterface(&we_driver_);
  we_driver_.interfaceType = WE_i2c;
  we_driver_.options.i2c.burstMode = 1;
  we_driver_.options.i2c.address = i2c_address_;
  we_driver_.handle = i2c_handle;

  /* Wait for boot */
  HAL_Delay(50);

  /* Communication test */
  if (WE_SUCCESS != ISDS_getDeviceID(&we_driver_, &device_id_value) ||
		  (device_id_value != ISDS_DEVICE_ID_VALUE))
  {
	  return Status::kNotOk;
  }

  /* Perform soft reset of the sensor */
  ISDS_softReset(&we_driver_, ISDS_enable);
  do
  {
    ISDS_getSoftResetState(&we_driver_, &sw_reset);
  }
  while (sw_reset);

  /* Perform reboot (retrieve trimming parameters from nonvolatile memory) */
  ISDS_reboot(&we_driver_, ISDS_enable);
  HAL_Delay(15);

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&we_driver_, ISDS_enable);

  /* Sampling rate (104 Hz) */
  ISDS_setAccOutputDataRate(&we_driver_, ISDS_accOdr104Hz);
  ISDS_setGyroOutputDataRate(&we_driver_, ISDS_gyroOdr104Hz);

  /* Accelerometer 2g range */
  ISDS_setAccFullScale(&we_driver_, ISDS_accFullScaleTwoG);

  /* Gyroscope 2000 dps range */
  ISDS_setGyroFullScale(&we_driver_, ISDS_gyroFullScale2000dps);

  return Status::kOk;
}

/* Get the measured acceleration */
Vector3d<float> SixAxisImu::GetAcceleration()
{
  ISDS_state_t data_ready;
  Vector3d<float> acceleration;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isAccelerationDataReady(&we_driver_, &data_ready);
  }
  while (data_ready == ISDS_disable);

  /* Read acceleration values */
  if (ISDS_getAccelerations_float(&we_driver_, &acceleration.x, &acceleration.y,
		  &acceleration.z) != WE_SUCCESS)
  {
    acceleration.status = Status::kNotOk;
  }
  else
  {
	acceleration.status = Status::kOk;
  }

  return acceleration;
}

/* Get the measured angular speed */
Vector3d<float> SixAxisImu::GetAngularSpeed()
{
  ISDS_state_t data_ready;
  Vector3d<float> acceleration;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isGyroscopeDataReady(&we_driver_, &data_ready);
  }
  while (data_ready == ISDS_disable);

  /* Read acceleration values */
  if (ISDS_getAngularRates_float(&we_driver_, &acceleration.x, &acceleration.y,
		  &acceleration.z) != WE_SUCCESS)
  {
    acceleration.status = Status::kNotOk;
  }
  else
  {
	acceleration.status = Status::kOk;
  }

  return acceleration;
}

/* Get the measured temperature */
Scalar<float> SixAxisImu::GetTemperature()
{
  ISDS_state_t data_ready;
  Scalar<float> temperature;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isTemperatureDataReady(&we_driver_, &data_ready);
  }
  while (data_ready == ISDS_disable);

  /* Read temperature value */
  if (ISDS_getTemperature_float(&we_driver_, &temperature.value) != WE_SUCCESS)
  {
    temperature.status = Status::kNotOk;
  }
  else
  {
    temperature.status = Status::kOk;
  }

  return temperature;
}


} /* namespace sensor_interface */
} /* namespace stm32_code */




