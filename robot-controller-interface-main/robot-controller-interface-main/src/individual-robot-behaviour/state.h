/* state.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-22
 * Last modified: 2024-12-12 by Carl Larsson
 * Description: Robot state header file.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_STATE_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_STATE_H_


/* Related .h files */

/* C++ standard library headers */
#include <cmath>
#include <mutex>

/* Other .h files */
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

/* Project .h files */
#include "../common_types.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/*!
 * @brief Class for a pose in 2D Cartesian space. 
 * 
 * Class containing a pose in 2D space and methods for accessing and setting
 * these values in a thread safe way. 
 * Pose in 2D Cartesian space consists of:
 * - x coordinate
 * - y coordinate
 * - angle theta (also known as yaw) in radians
 *
 * @note Copyable, not moveable.
 */
class Pose
{
 public:
  /*!
   * @brief Default constructor 
   *
   * The default constructor sets x_ to 0.0, y_ to 0.0 and theta_ to 0.0.
   */
  Pose();

  /*! 
   * @brief Parameterized constructor
   * 
   * The parametrized constructor sets x_, y_ and theta_ to the provided 
   * values.
   *
   * @param[in] x The x value to be assigned to the classes x_ member.
   * @param[in] y The y value to be assigned to the classes y_ member.
   * @param[in] theta The theta value to be assigned to the classes theta_ 
   * member.
   */
  Pose(double x, double y, double theta);

  /*! 
   * @brief Copy constructor 
   *
   * Creates a Pose, copying the x_, y_ and theta_ values from another Pose.
   * Does not copy the mutex.
   *
   * @param[in] other A const pointer to another Pose which we are to copy all 
   * member values from to create the new Pose.
   *
   * @note Mutex is not copied.
   */
  Pose(const Pose& other);

  /*! 
   * @brief Get class x_ value.
   *
   * Gets the x_ value of the class. Thread safe.
   *
   * @return Returns the value of the private member x_.
   *
   * @note Thread safe.
   */
  double GetX() const;
  /*! 
   * @brief Get class y_ value.
   *
   * Gets the y_ value of the class. Thread safe.
   *
   * @return Returns the value of the private member y_.
   *
   * @note Thread safe.
   */
  double GetY() const;
  /*! 
   * @brief Get class theta_ value.
   *
   * Gets the theta_ value of the class. Thread safe.
   *
   * @return Returns the value of the private member theta_.
   *
   * @note Thread safe.
   */
  double GetTheta() const;

  /*!
   * @brief Set the x_ value of this class.
   *
   * Sets the x_ value of the class, value must be within the playing field,
   * if it is outside then it is set to the closest limit (negative or 
   * positive).
   *
   * @param[in] x The x value that is to be assigned to the classes x_ member.
   *
   * @note Thread safe.
   */
  void SetX(double x);
  /*!
   * @brief Set the y_ value of this class.
   *
   * Sets the y_ value of the class, value must be within the playing field,
   * if it is outside then it is set to the closest limit (negative or 
   * positive).
   *
   * @param[in] y The y value that is to be assigned to the classes y_ member.
   *
   * @note Thread safe.
   */
  void SetY(double y);
  /*!
   * @brief Set the theta_ value of this class.
   *
   * Sets the theta angle and wraps it to [-pi, pi].
   *
   * @param[in] theta The theta value that is to be assigned to the classes 
   * theta_ member.
   *
   * @note Thread safe.
   */
  void SetTheta(double theta);

  /*!
   * @brief A = (assignment) operator for this class.
   *
   * A thread safe assignment operator for the class.
   *
   * @param[in] other Another Pose which member values we are to assigned to 
   * "our own" Pose.
   * @return Returns address to "our own" Pose with its member values set to 
   * the same as the other Pose member values.
   *
   * @note Thread safe, both Poses mutexes are locked.
   * @note The classes mutex member value is NOT set to the others mutex value.
   * @note Does not seem to be able to assign already defined/declared 
   * variables.
   */
  Pose& operator=(const Pose& other);

  /*! 
   * @brief A != (inequality) operator for this class.
   *
   * A thread safe inequality operator for the class.
   *
   * @param[in] other The other Pose which we are to compare with.
   * @return Returns True if any member values are not equal (with a difference 
   * greater than the tolerance value). Returns False if all member values are 
   * equal (within the tolerance value).
   *
   * @note Thread safe, both classes mutexes are locked.
   */
  bool operator!=(const Pose& other) const;

 private:
  /*!
   * @brief X coordinate.
   *
   * X coordinate, can only be accessed with the thread safe methods GetX and 
   * SetX.
   */
  double x_;
  /*!
   * @brief Y coordinate.
   *
   * Y coordinate, can only be accessed with the thread safe methods GetY and 
   * SetY.
   */
  double y_;
  /*!
   * @brief Theta angle, also known as yaw.
   *
   * Theta angle (yaw), can only be accessed with the thread safe methods 
   * GetTheta and SetTheta.
   */
  double theta_;
  /*!
   * @brief Mutex ensuring the class methods are thread safe.
   *
   * @note Can not be accessed directly.
   *
   * @warning This mutex does not ensure there are no interruptions inbetween 
   * subsequent or multiple calls to different class functions.
   */
  mutable std::mutex pose_mutex_;

  /*!
   * @brief Tolerance for != operation 
   */
  const double tolerance_ = 1e-6;
};

/*============================================================================*/

/*! 
 * @brief Class describing the current state of a robot. 
 *
 * Class containing the robot state, including x and y coordinates, theta (or 
 * yaw) (in radians) and whether it has the ball or not. Additionally it contains thread 
 * safe ways of accessing and setting these values.
 *
 * @note Copyable, not moveable.
 */
class RobotState 
{
 public:
  /*! 
   * @brief Default constructor.
   *
   * The default constructor sets x_ to 0.0, y_ to 0.0, theta_ to 0.0 and 
   * ball_ to false. 
   */
  RobotState();

  /*!
   * @brief Parameterized constructor.
   *
   * Parametrized constructor setting x_ ,y_ ,theta_ and ball_ to the provided
   * values.
   *
   * @param[in] x The x value which is to be assigned to the classes x_ member.
   * @param[in] y The y value which is to be assigned to the classes y_ member.
   * @param[in] theta The theta value which is to be assigned to the classes 
   * theta_ member.
   * @param[in] ball The ball value which is to be assigned to the classes 
   * ball_ member.
   */
  RobotState(double x, double y, double theta, bool ball);

  /*! 
   * @brief Copy constructor.
   *
   * Creates a RobotState, copying the x_, y_, theta_ and ball_ values from 
   * another RobotState. Does not copy the mutex.
   *
   * @param[in] other Another RobotState from which we copy the member values 
   * to the new RobotState (except mutex).
   *
   * @note Mutex value is not copied to new Pose.
   */
  RobotState(const RobotState& other);

  /*! 
   * @brief Get class x_ value.
   *
   * Gets the x_ value of the class. Thread safe.
   *
   * @return Returns the value of the private member x_.
   *
   * @note Thread safe.
   */
  double GetX() const;
  /*!
   * @brief Get class y_ value.
   *
   * Gets the y_ value of the class. Thread safe.
   *
   * @return Returns the value of the private member y_.
   *
   * @note Thread safe.
   */
  double GetY() const;
  /*!
   * @brief Get class theta_ value.
   *
   * Gets the theta_ value of the class. Thread safe.
   *
   * @return Returns the value of the private member theta_.
   *
   * @note Thread safe.
   */
  double GetTheta() const;
  /*!
   * @brief Gets the status of whether the robot has the ball or not.
   *
   * Indicates whether the robot has the ball or not. If true then robot
   * has the ball such that activating the kicker will hit and launch the
   * ball.
   *
   * @return Returns the value of the private member ball_.
   *
   * @note Thread safe.
   */
  bool GetBall() const;

  /*!
   * @brief Set the x_ value of this class. Thread safe.
   *
   * @param[in] x The x value that is to be assigned to the classes x_ member.
   *
   * @note Thread safe.
   */
  void SetX(double x);
  /*!
   * @brief Set the y_ value of this class. Thread safe.
   *
   * @param[in] y The y value that is to be assigned to the classes y_ member.
   *
   * @note Thread safe.
   */
  void SetY(double y);
  /*!
   * @brief Set the theta_ value of this class. Thread safe.
   *
   * Sets the theta_ value and wraps it to [-pi, pi].
   *
   * @param[in] theta The theta value that is to be assigned to the classes 
   * theta_ member.
   * 
   * @note thread safe.
   */
  void SetTheta(double theta);
  /*!
   * @brief Set the status of whether the robot has the ball or not. Thread 
   * safe.
   *
   * @param[in] ball The ball value that is to be assigned to the classes 
   * ball_ member.
   *
   * @note Thread safe.
   */
  void SetBall(bool ball);

 private:
  /*!
   * @brief X coordinate of the robot.
   *
   * @note Member can only be accessed with GetX and SetX.
   */
  double x_;
  /*!
   * @brief Y coordinate of the robot.
   *
   * @note Member can only be accessed with GetY and SetY.
   */
  double y_;
  /*!
   * @brief Theta, also known as yaw, of the robot.
   *
   * @note Member can only be accessed with GetTheta and SetTheta.
   */
  double theta_;
  /*!
   * @brief This member indicates whether a robot has the ball or not.
   *
   * This member should be true if the robot has the ball in a position where
   * activating kicker this moment would result in the ball getting kicked, 
   * otherwise it should be false.
   *
   * @note Member can only be accessed with GetBall and SetBall.
   */
  bool ball_;
  /*!
   * @brief Mutex ensuring all class functions are thread safe.
   *
   * @note Can not be accessed directly.
   *
   * @warning This mutex does not ensure there are no interruptions inbetween 
   * subsequent or multiple calls to different class functions.
   */
  mutable std::mutex robot_state_mutex_;
};

/*============================================================================*/

/*!
 * @brief Odom Subscriber class
 *
 * Class which creates a odom subscriber node and has a callback whenever a 
 * message is received to store odometry data in current_state.
 * 
 * @note Neither copyable nor move-only. 
 * @note Dependent on external systems, including ROS.
 *
 * @pre For the class to work as intended the following preconditions must
 * be met:
 * - `rclcpp` must be initialized.
 *
 * @warning Failure to meet the preconditions could result in the class not 
 * working.
 */
class OdomSubscriber : public rclcpp::Node
{
 public:
   /*!
    * @brief Default constructor creating a odom subscriber node.
    *
    * Default constructor, creates an odom subscriber node and binds a
    * callback function to it.
    *
    * @see OdomCallback for the callback function.
    */
  OdomSubscriber();

 private:
  /*!
   * @brief Odometry subscriber callback function storing odom in 
   * current_state.
   *
   * @param[in] msg nav2 message contain Cartesian 3D positional data, 
   * orientation data expressed in quaternion system, linear velocity and 
   * angular velocity. 
   *
   * @note If msg is nullptr then the callback returns and does not store 
   * anything.
   *
   * @see OdomSubscriber for dependencies, requirements and preconditions.
   */
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  /*!
   * @brief Pointer to odom subscriber node.
   *
   * Pointer to odom subscriber node. It is private and cannot be accessed.
   * It is not intended to be accessed.
   */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

/*============================================================================*/

/*!
 * @brief Calculates angle between current possition and target possition.
 *
 * Calculates angle (radians) between current position (current_x, current_y) 
 * and target position (target_x, target_y), assuming a playing field which 
 * follows unit circle coordinates with four quadrants.
 *
 * @param[in] current_x Current position x coordinate.
 * @param[in] current_y Current position y coordinate.
 * @param[in] target_x Target position x coordinate.
 * @param[in] target_y Target position y coordinate.
 *
 * @throws std::invalid_argument if the current and target positions are the 
 * same, since atan2 is undefined for this case.
 *
 * @return The angle in radians between the two points wrapped to [-pi, pi].
 *
 * @note current and target can not be the same since atan2 is undefined in 
 * this case.
 */
double CalculateAngle(double current_x, double current_y, 
    double target_x, double target_y);

/*============================================================================*/

/*!
 * @brief Global variable for keeping track of current robot state.
 * 
 * Needs to be global since its used between multiple threads to keep track
 * of current robot state.
 */
extern RobotState current_state;

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namespace robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_ */
