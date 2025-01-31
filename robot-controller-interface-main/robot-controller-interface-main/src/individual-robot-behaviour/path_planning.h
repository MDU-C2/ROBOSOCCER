/* path_planning.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-19
 * Last modified: 2024-10-27 by Carl Larsson
 * Description: Path planning header file.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_


/* Related .h files */

/* C++ standard library headers */
#include <algorithm>
#include <memory>
#include <mutex>

/* Other .h files */
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

/* Project .h files */
#include "../individual-robot-behaviour/ball.h"
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/*!
 * @brief DWB controller class
 * 
 * Class for creating a DWB action node for controlling DWB, offers functions
 * to send target position to DWB which DWB then will navigate to.
 *
 * @note Neither copyable nor move-only. 
 * @note This class depends on external systems, including Nav2, this holds for
 * all its members aswell unless stated otherwise.
 *
 * @pre The following preconditions must be met before using this class:
 * - Nav2 stack must be running. (https://docs.nav2.org/)
 * - `rclcpp` must be initialized.
 * - A valid `baselink -> odom` transform must be available.
 * - A valid `baselink -> map` transform must be available.
 *
 * @warning Failure to launch Nav2, intializing rclcpp, not having a baselink to 
 * odom transform or baselink to map transform before using this class may 
 * result in the the class and its members failing to function as intended.
 */
class DwbController : public rclcpp::Node
{
 public:
   /*!
    * @brief Default constructor creating an action server node.
    *
    * Default constructor which creates an NavigateToPose action server node 
    * and then waits until the action server is ready.
    *
    * @throws RCLCPP_ERROR if the action server does not become ready within 
    * 10 seconds.
    *
    * @pre The following preconditions are necessary for the action server to 
    * become ready:
    * - Nav2 stack must be running. (https://docs.nav2.org/)
    * - `rclcpp` must be initialized.
    * - A valid `baselink -> odom` transform must be available.
    * - A valid `baselink -> map` transform must be available. If a map is 
    *   used (this is the case unless global costmap, planner is disabled).
    */
  DwbController();

  /*!
   * @brief Function for sending the target pose to DWB controller.
   *
   * Function for sending the target pose to DWB controller. This should 
   * automatically have DWB run in the background and publish velocities on 
   * cmd_vel. If a new target_pose is sent, then the old will be abandoned
   * and the most recent will be the goal.
   *
   * @param[in] target_pose The target pose which the path planner should
   * navigate to.
   *
   * @see DwbController for dependencies, requirements and preconditions
   * to use this function.
   */
  void SendTargetPose(Pose target_pose);

 private:
  /*!
   * @brief Callback function indicating if target position has been reached.
   *
   * Callback function which indicates if the target pose was reached, aborted,
   * canceled or encounters some unknown error.
   *
   * @param[in] result Contains result code describing the status of the task.
   */
  void ResultCallback(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>
          ::WrappedResult &result);

  /*!
   * @brief Action client node for sending target pose to DWB controller 
   *
   * NavigateToPose action client node object. The object is private and is not
   * intended to be accessed.
   */
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr 
      navigate_to_pose_client_;
};

/*============================================================================*/

/*!
 * @brief Performs local path planning using DWA.
 *
 * A loop which continously checks if a new target position has been set and 
 * sends the new target pose to the DWA if it has been updated.
 *
 * @param[in] target_pose Target position using class Pose. Can not be nullptr.
 *
 * @throws std::invalid_argument if argument is nullptr. Robot has not been 
 * initialized correctly if argument is nullptr.
 *
 * @note This function depends on external systems, including Nav2.
 *
 * @pre The following preconditions must be met before calling this function:
 * - Nav2 stack must be running. (https://docs.nav2.org/)
 * - `rclcpp` must be initialized.
 * - A valid `baselink -> odom` transform must be available.
 * - A valid `baselink -> map` transform must be available.
 *
 * @warning Failure to launch Nav2, intializing rclcpp, not having a baselink to 
 * odom transform or baselink to map transform before calling this function may 
 * result in the function failing to function as intended.
 */
void LocalPathPlanning(Pose *target_pose);

/*============================================================================*/

/*!
 * @brief Used to indicate if path planning work is completed.
 *
 * Indicates which callers work has been completed:
 * 0 indicates work has not been completed
 * 1 then work called by local_path_planner is completed
 * 2 then work called by shoot_setup is completed
 *
 * This variable is accessed by multiple threads and hence must be global.
 */
extern std::atomic_int atomic_target_reached_flag;
/*!
 * @brief Informs the path planner that we are commanded to move to a new target
 *
 * Informs the callback function for the DWB controller that it was a move
 * command which called and wanted path planning work to be done.
 *
 * Must be a global variable since multiple threads need to be able to access 
 * it.
 */
extern std::atomic_bool atomic_move_to_target;

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_ */
