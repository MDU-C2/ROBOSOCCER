/* proximity_sensor_62cm.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the VL6180X 62cm proximity sensor. This driver
 * functions as a wrapper for Adafruit's VL6180X Driver which can be found
 * here: https://github.com/adafruit/Adafruit_VL6180X
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h file */
#include "../../user-code/sensor-interface/proximity_sensor_62cm.h"

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../../Drivers/Adafruit_VL6180X/Adafruit_VL6180X.h"

namespace stm32_code
{
namespace sensor_interface
{

/* Default constructor */
ProximitySensor62cm::ProximitySensor62cm() {}


/* Configure the sensor to begin taking measurements */
Status ProximitySensor62cm::Init(I2C_HandleTypeDef* i2c_handle)
{
  bool status;

  adafruit_driver_ = Adafruit_VL6180X(i2c_address_);
  status = adafruit_driver_.begin(i2c_handle);

  if (status == true)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

/* Returns the measured distance in mm */
Scalar<uint8_t> ProximitySensor62cm::GetDistance()
{
  Scalar<uint8_t> distance;
  uint8_t status;

  status = adafruit_driver_.readRangeStatus();
  distance.value = adafruit_driver_.readRange();

  if (status == VL6180X_ERROR_NONE)
  {
    distance.status = Status::kOk;
  }
  else
  {
    distance.status = Status::kNotOk;
  }

  return distance;
}

/* Returns the measured illuminence in lux */
Scalar<float> ProximitySensor62cm::GetIlluminance()
{
  Scalar<float> luminosity;
  uint8_t status;

  status = adafruit_driver_.readRangeStatus();
  luminosity.value = adafruit_driver_.readLux(VL6180X_ALS_GAIN_5);

  if (status == VL6180X_ERROR_NONE)
  {
    luminosity.status = Status::kOk;
  }
  else
  {
    luminosity.status = Status::kNotOk;
  }

  return luminosity;
}

} /* namespace sensor_interface */
} /* namespace stm32_code */
