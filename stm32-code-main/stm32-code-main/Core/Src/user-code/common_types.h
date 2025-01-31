/* common_types.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-12-13 by Emil Åberg
 * Description: Common types used in the stm32-code project.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_COMMONTYPES_H
#define STM32CODE_COMMONTYPES_H

namespace stm32_code
{

/*!
 * @brief Enum representing status of an operation or value.
 */
enum class Status
{
  /*!
   * @brief No error has been detected.
   */
  kOk = 0,

  /*!
   * @brief An error has been detected. Accompanying operation or value
   * is unreliable.
   */
  kNotOk = -1
};

/*!
 * @brief Struct representing a 3D vector with status indicator.
 */
template<typename T> struct Vector3d
{
  T x;
  T y;
  T z;
  Status status;
};

/*!
 * @brief Struct representing a quarternion value with status indicator.
 */
template<typename T> struct Quarternion
{
  T w;
  T x;
  T y;
  T z;
  Status status;
};

/*!
 * @brief Struct representing a rotation with status indicator.
 */
template<typename T> struct Rotation
{
  T heading;
  T roll;
  T pitch;
  Status status;
};

/*!
 * @brief Struct representing a scalar value with status indicator.
 */
template<typename T> struct Scalar
{
  T value;
  Status status;
};

} /* namespace stm32_code */

#endif /* STM32CODE_COMMONTYPES_H */
