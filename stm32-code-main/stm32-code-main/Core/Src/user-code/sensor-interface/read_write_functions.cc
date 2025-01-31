/* read_write_functions.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-19 by Emil Åberg
 * Description: Functions to read and write data over I2C.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h file */
#include "../../user-code/sensor-interface/read_write_functions.h"

/* Projects .h files. */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"

namespace stm32_code
{
namespace sensor_interface
{

/* Read a byte of data from an external component via I2C */
Status ReadByte(I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address,
    uint8_t register_address, uint8_t* value)
{
  uint8_t data_write[1];
  uint8_t data_read[1];
  uint8_t status = 0;

  data_write[0] = register_address;
  status = HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_write, 1, 100);
  status = HAL_I2C_Master_Receive(i2c_handle, i2c_address, data_read, 1, 100);
  *value = data_read[0];

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

/* Read a block of data from an external component via I2C */
Status ReadBytes(I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address,
    uint8_t register_address, uint8_t buffer[], int buffer_size)
{
  uint8_t data_write[1];
  uint8_t status = 0;

  data_write[0] = register_address;
  status = HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_write, 1, 100);
  status = HAL_I2C_Master_Receive(i2c_handle, i2c_address, buffer, buffer_size, 100);

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

/* Write a byte of data to an external component via I2C */
Status WriteByte(I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address,
    uint8_t register_address, uint8_t value)
{
  uint8_t data_write[2];
  uint8_t status = 0;

  data_write[0] = register_address;
  data_write[1] = value;
  status = HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_write, 2, 100);

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

} /* namespace sensor_interface */
} /* namespace stm32_code */
