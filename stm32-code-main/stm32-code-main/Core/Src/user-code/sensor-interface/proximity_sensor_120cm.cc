/* proximity_sensor_120cm.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the VL53L4CD 120cm proximity sensor. This driver
 * functions as a wrapper for ST's Ultra Lite Driver which can be found
 * here: https://www.st.com/en/embedded-software/stsw-img026.html
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h file */
#include "../../user-code/sensor-interface/proximity_sensor_120cm.h"

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"

namespace stm32_code
{
namespace sensor_interface
{

/* Default constructor */
ProximitySensor120cm::ProximitySensor120cm() {}

/* Configure the sensor to begin taking measurements */
Status ProximitySensor120cm::Init(I2C_HandleTypeDef* i2c_handle)
{
  VL53L4CD_Error status;
  i2c_handle_ = i2c_handle;
  status = VL53L4CD_SensorInit(i2c_handle, kI2cAddress_);

  if (status == 0)
  {
    status = VL53L4CD_StartRanging(i2c_handle, kI2cAddress_);
  }

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
	  return Status::kNotOk;
  }
}

/* Returns the measured distance in mm */
Scalar<uint16_t> ProximitySensor120cm::GetDistance()
{
  uint8_t data_ready;
  VL53L4CD_ResultsData_t result;
  Scalar<uint16_t> distance;

  VL53L4CD_CheckForDataReady(i2c_handle_, kI2cAddress_, &data_ready);

  if (data_ready == 1)
  {
    VL53L4CD_GetResult(i2c_handle_, kI2cAddress_, &result);
    VL53L4CD_ClearInterrupt(i2c_handle_, kI2cAddress_);
    distance.value = result.distance_mm;

    if (result.range_status == 0)
    {
      distance.status = Status::kOk;
    }
    else
    {
      distance.status = Status::kNotOk;
    }
  }
  else
  {
    distance.status;
  }

  return distance;
}

} /* namespace sensor_interface */
} /* namespace stm32_code */
