/* read_write_functions.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-19 by Emil Åberg
 * Description: Functions to read and write data over I2C.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_SENSORINTERFACE_READWRITEFUNCTIONS_H
#define STM32CODE_SENSORINTERFACE_READWRITEFUNCTIONS_H

/* Projects .h files. */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Read a byte of data from an external component via I2C.
 *
 * Read a byte of data from a register on an external component via I2C and
 * write it to a specified variable.
 *
 * @param[in] i2c_handle Pointer to the handle of the I2C peripheral.
 * @param[in] i2c_address I2C address of the component to read from.
 * @param[in] register_address Address of the register to read from.
 * @param[out] value Pointer to the variable to write the data to.
 *
 * @return Status Enum indicating whether the read was successful.
 *
 * @pre The following preconditions must be met before using this function:
 * - The register is using 8 bit register addresses. This function will not
 * work on components with 16 bit register addresses.
 * - For 7-bit I2C addresses, the i2c_address parameter must be bit shifted
 * one bit to the left.
 */
Status ReadByte(I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address,
    uint8_t register_address, uint8_t* value);

/*!
 * @brief Read a block of data from an external component via I2C.
 *
 * Read a block of data from an external component via I2C and write it
 * to a byte array.
 *
 * @param[in] i2c_handle Pointer to the handle of the I2C peripheral.
 * @param[in] i2c_address I2C address of the component to read from.
 * @param[in] register_address Address of the first register to read from.
 * @param[out] buffer Byte array to write the output data to.
 * @param[in] buffer_size Number of bytes to read, this value should match the
 * size of the buffer.
 *
 * @return Status Enum indicating whether the read was successful.
 *
 * @pre The following preconditions must be met before using this function:
 * - The register is using 8 bit register addresses. This function will not
 * work on components with 16 bit register addresses.
 * - For 7-bit I2C addresses, the i2c_address parameter must be bit shifted
 * one bit to the left.
 * - The data to read is arranged sequentially with the register_address
 * argument indicating the first byte to read.
 * - The component has auto-incrementing register pointer; the register
 * pointer is automatically incremented after each byte read.
 */
Status ReadBytes(I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address,
    uint8_t register_address, uint8_t buffer[], int buffer_size);

/*!
 * @brief Write a byte of data to an external component via I2C.
 *
 * Write a byte of data to a register on an external component via I2C.
 *
 * @param[in] i2c_handle Pointer to the handle of the I2C peripheral.
 * @param[in] i2c_address I2C address of the component to write to.
 * @param[in] register_address Address of the register to write to.
 * @param[in] value Data to write.
 *
 * @return Status Enum indicating whether the write was successful.
 *
 * @pre The following preconditions must be met before using this function:
 * - The register is using 8 bit register addresses. This function will not
 * work on components with 16 bit register addresses.
 * - For 7-bit I2C addresses, the i2c_address parameter must be bit shifted
 * one bit to the left.
 */
Status WriteByte(I2C_HandleTypeDef* i2c_handle, uint8_t i2c_address,
    uint8_t register_address, uint8_t value);

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORINTERFACE_READWRITEFUNCTIONS_H */
