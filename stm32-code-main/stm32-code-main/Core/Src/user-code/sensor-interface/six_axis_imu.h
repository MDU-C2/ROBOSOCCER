/* nine_axis_imu.h
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

#ifndef STM32CODE_SENSORDRIVERS_SIXAXISIMU_H
#define STM32CODE_SENSORDRIVERS_SIXAXISIMU_H

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../../Drivers/WSEN_ISDS_2536030320001/WSEN_ISDS_2536030320001.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Class representing driver control of one WSEN_ISDS six axis imu.
 *
 * Class representing driver control of one WSEN_ISDS six axis imu.
 *
 * @note Not copyable, not moveable.
 */
class SixAxisImu
{
 public:
  /*!
   * @brief Default constructor
   *
   * The default constructor.
   */
  SixAxisImu();

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
   * @brief Get the measured acceleration.
   *
   * Returns the measured acceleration in mg.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the acceleration vector and status
   * enum indicating whether sensor read was successful.
   */
  Vector3d<float> GetAcceleration();

  /*!
   * @brief Get the measured angular speed.
   *
   * Returns the measured angular speed in milli degrees/s.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the angular velocity vector and status
   * enum indicating whether sensor read was successful.
   */
  Vector3d<float> GetAngularSpeed();

  /*!
   * @brief Get the measured temperature.
   *
   * Returns the measured temperature in degrees Celsius.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing temperature value and status enum indicating
   * whether sensor read was successful.
   */
  Scalar<float> GetTemperature();

 private:
  /*!
   * @brief An instance of Wurth Electronic's WSEN_ISDS driver.
   */
  WE_sensorInterface_t we_driver_;

  /*!
   * @brief I2C address of the WSEN_ISDS sensor.
   */
  uint8_t i2c_address_ = 0x6b;
};

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORDRIVERS_SIXAXISIMU_H */

