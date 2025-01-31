/* rgb_apds_9960.cc
 *==============================================================================
 * Author: Mudar Ibrahim
 * Creation date: 2024-11-12
 * Last modified: 2024-12-13 by Emil Åberg
 * Description: This code defines a driver for the APDS9960 sensor,
 * which is an advanced proximity and RGB sensor used for various applications,
 * including proximity detection and ambient light sensing.
 * The driver is designed for use on an STM32 microcontroller with the HAL
 * library for I2C communication.
 * The code implements functions to initialize the APDS9960 sensor
 * by enabling its power and proximity detection capabilities and to read
 * proximity data from the sensor.
 *
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "../../user-code/sensor-interface/rgb_apds_9960.h"

/* Projects .h files. */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../user-code/sensor-interface/read_write_functions.h"

/*stm32_code*/
namespace stm32_code
{
/* sensor_interface */
namespace sensor_interface
{

/* Initializes the APDS9960 sensor by enabling power and proximity detection. */
Status InitializeApds9960(I2C_HandleTypeDef* i2c_handle)
{
  /* Enable power (PON) and proximity../../user-code/ detection (PEN)*/
  uint8_t enable_value = 0x05;

  /* Write the value to the ENABLE register to enable power and proximity
   * detection*/
  return WriteByte(i2c_handle, APDS9960_ADDR, ENABLE_REG, enable_value);
  HAL_Delay(10);
}

/* Reads proximity data from the APDS9960 sensor.*/
Status ReadProximityData(I2C_HandleTypeDef* i2c_handle, uint8_t* proximity_data)
{
  /* Read proximity data from the PDATA register*/
  return ReadByte(i2c_handle, APDS9960_ADDR, PDATA_REG, proximity_data);
  HAL_Delay(10);
}

} /* namespace sensor_interface */
} /* namespace stm32_code */
