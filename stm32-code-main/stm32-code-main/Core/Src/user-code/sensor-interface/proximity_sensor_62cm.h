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

#ifndef STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR62CM_H
#define STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR62CM_H

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../../Drivers/Adafruit_VL6180X/Adafruit_VL6180X.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Class representing driver control of one VL6180X proximity sensor.
 *
 * Class representing driver control of one VL6180X proximity sensor.
 *
 * @note Not copyable, not moveable.
 */
class ProximitySensor62cm
{
 public:
  /*!
   * @brief Default constructor
   *
   * The default constructor.
   */
  ProximitySensor62cm();

  /*!
   * @brief Initializes the sensor.
   *
   * Configure the sensor to begin taking measuremets.
   *
   * @param[in] i2c_handle Pointer to the handle of the I2C peripheral.
   * 
   * @return Enum indicating wheter initialization was successful.
   */
  Status Init(I2C_HandleTypeDef* i2c_handle);

  /*!
   * @brief Get the measured distance.
   *
   * Returns the measured distance in mm.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the distance and status
   * enum indicating whether sensor read was successful.
   */
  Scalar<uint8_t> GetDistance();

  /*!
   * @brief Get the measured illuminence.
   *
   * Returns the measured illuminence in lux.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the illuminence and status
   * enum indicating whether sensor read was successful.
   */
  Scalar<float> GetIlluminance();

 private:
  /*!
   * @brief An instance of Adafruit's VL6180X driver.
   */
  Adafruit_VL6180X adafruit_driver_;

  /*!
   * @brief I2C address of the VL6180X sensor.
   */
  uint8_t i2c_address_ = (0x29 << 1);
};

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR62CM_H */
