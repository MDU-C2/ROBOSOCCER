/* proximity_sensor_120cm.h
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

#ifndef STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR120CM_H
#define STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR120CM_H

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Class representing driver control of one VL53L4CD 120cm proximity
 * sensor.
 *
 * Class representing driver control of one VL53L4CD 120cm proximity sensor.
 *
 * @note Not copyable, not moveable.
 */
class ProximitySensor120cm
{
 public:
  /*!
   * @brief Default constructor
   *
   * The default constructor.
   */
  ProximitySensor120cm();

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
  Scalar<uint16_t> GetDistance();

 private:
  /*!
   * @brief I2C address of the distance sensor.
   */
  static constexpr Dev_t kI2cAddress_ = 0x52;

  /*!
   * @brief Pointer to the handle of the I2C peripheral.
   */
  I2C_HandleTypeDef* i2c_handle_;
};

} /* namespace sensor_interface */
} /* namespace stm32_code */


#endif /* STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR120CM_H */
