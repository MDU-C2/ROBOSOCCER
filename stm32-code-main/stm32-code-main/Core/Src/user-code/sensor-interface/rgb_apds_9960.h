/* rgb_apds_9960.h
 *==============================================================================
 * Author: Mudar Ibrahim
 * Creation date: 2024-11-12
 * Last modified: 2024-12-13 by Emil Åberg
 * Description: This code defines a driver for the APDS9960 sensor,
 * which is an advanced proximity and RGB sensor used for various applications,
 * including proximity detection and ambient light sensing.
 * The driver is designed for use on an STM32 microcontroller with the HAL library
 * for I2C communication.
 * The code implements functions to initialize the APDS9960 sensor
 * by enabling its power and proximity detection capabilities and to read
 * proximity data from the sensor.
 *
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef STM32CODE_SENSORINTERFACE_RGBAPDS9960_H
#define STM32CODE_SENSORINTERFACE_RGBAPDS9960_H

/* Projects .h files. */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../user-code/sensor-interface/read_write_functions.h"

/*I2C address of the APDS9960*/
#define APDS9960_ADDR (0x39 << 1)
/*ENABLE register address*/
#define ENABLE_REG 0x80
/*Proximity data register address*/
#define PDATA_REG 0x9C

/* stm32_code*/
namespace stm32_code
{
/* sensor_drivers */
namespace sensor_interface
{
/*!
 * @brief Initializes the APDS9960 sensor by enabling power and proximity
 * detection.
 *
 * This function configures the APDS9960 sensor by writing to the ENABLE
 * register to turn on the power and enable proximity sensing functionality.
 * This operation requires communication over the I2C bus.
 *
 * @param[in] i2c_handle Handle to the I2C peripheral used for communication
 * with the APDS9960 sensor.
 *
 * @return Status Indicates whether the initialization was successful or
 * encountered an error:
 * - Status::kOk if the operation completed successfully.
 * - Status::kNotOk if there was an error during communication or setup.
 *
 * @note The I2C peripheral must be initialized before calling this function.
 * @note This function uses the APDS9960 I2C address defined as `APDS9960_ADDR`
 * and writes the value `0x05` to enable both power (PON) and proximity
 * detection (PEN).
 *
 * @see ReadProximityData for reading data from the proximity sensor.
 *
 * @pre Ensure that:
 * - The I2C bus is properly configured.
 * - The APDS9960 sensor is correctly connected and powered.
 *
 * @warning Incorrect I2C address or bus issues may cause communication failures.
 * Verify that the APDS9960 sensor is properly connected and powered before
 * calling this function.
 */
Status InitializeApds9960(I2C_HandleTypeDef* i2c_handle);
/*!
 * @brief Reads proximity data from the APDS9960 sensor.
 *
 * This function reads the proximity data from the PDATA register of the
 * APDS9960 sensor over the I2C bus. The data read provides information about
 * the detected proximity levels, allowing further processing or decision-making
 * based on the proximity value.
 *
 * @param[in] i2c_handle Handle to the I2C peripheral used for communication
 * with the APDS9960 sensor.
 * @param[out] proximity_data Pointer to a variable where the read proximity
 * value will be stored.
 *
 * @return Status Indicates whether the read operation was successful or
 * encountered an error:
 * - Status::kOk if the data was read successfully.
 * - Status::kNotOk if there was an error during communication.
 *
 * @note The I2C peripheral must be properly initialized and configured before
 * calling this function.
 * @note This function reads from the PDATA register (address `0x9C`) of the
 * APDS9960 sensor.
 *
 * @see InitializeApds9960 for initializing the APDS9960 sensor.
 *
 * @pre Ensure that:
 * - The APDS9960 sensor has been initialized and enabled for proximity sensing.
 * - The I2C bus is correctly configured.
 *
 * @warning Incorrect I2C configuration or improper initialization of the
 * APDS9960 sensor may lead to communication errors or invalid proximity data.
 */
Status ReadProximityData(I2C_HandleTypeDef* i2c_handle, uint8_t* proximity_data);

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORINTERFACE_RGBAPDS9960_H */
