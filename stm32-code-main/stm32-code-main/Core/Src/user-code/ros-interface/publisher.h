/* publisher.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-12-05
 * Last modified: 2024-12-05 by Emil Åberg
 * Description: Class representing a ROS node and publisher that can transmit
 * data to Raspberry Pi connected to peripheral UART5.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_ROSINTERFACE_PUBLISHER_H
#define STM32CODE_ROSINTERFACE_PUBLISHER_H

/* C++ standard library headers */
#include "string"

/* Project .h files */
#include "../../user-code/common_types.h"
#include "stm32h7xx_hal.h"

/* Other .h files */
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/float32.h"

namespace stm32_code
{
namespace ros_interface
{

/* external declaration for UART5 handle, which is connected to Raspberry Pi */
extern UART_HandleTypeDef huart5;

/*!
 * @brief Class for publishing data to Raspberry Pi as a ROS topic.
 *
 * Class for publishing data to Raspberry Pi over UART5 as a ROS topic.
 *
 * @note Neither copyable nor moveable.
 * @note As of writing this, no functionality for receiving the published topic
 * on the Raspberry Pi side has been developed yet. Anyone wanting to receive the
 * data sent with this class, will have to look up and implement the neccessary
 * steps on the Raspberry Pi.
 * 
 * @note Not copyable, not moveable.
 */
class Publisher
{
 public:
  /*! 
   * @brief Creates a ROS publisher node and topic.
   * 
   * Creates a ROS node and topic. The topic will be of message
   * type Float32.
   *
   * @param[in] name Name of the topic. The created node  will be named
   * '<topic_name>_node'.
   */
  Publisher(const std::string topic_name);

  /*! 
   * @brief Publish a value.
   * 
   * Publishes a new value. The published value will be of message type
   * Float32.
   *
   * @param[in] value Value to publish.
   * 
   * @return Status indicating wheter value was successfully published.
   */
  Status Publish(float value);

 private:
  /*! 
   * @brief Object to handle freertos memory allocation. Datatype defined by
   * microROS.
   */
  rcl_allocator_t freertos_allocator_;

  /*! 
   * @brief Object representing the microros publisher. Datatype defined by
   * microROS.
   */
  rcl_publisher_t publisher_;

  /*! 
   * @brief Object representing the ROS message. Datatype defined by
   * microROS.
   */
  std_msgs__msg__Float32 msg_;

  /*! 
   * @brief Object representing the node options. Datatype defined by
   * microROS.
   */
  rclc_support_t support_;

  /*! 
   * @brief Object to handle node memory allocation. Datatype defined by
   * microROS.
   */
  rcl_allocator_t allocator_;

  /*! 
   * @brief Object representing the node. Datatype defined by microROS.
   */
  rcl_node_t node_;
};

} /* namespace ros_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_ROSINTERFACE_PUBLISHER_H */
