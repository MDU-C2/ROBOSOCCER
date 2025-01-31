/* state.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-22
 * Last modified: 2024-12-12 by Carl Larsson
 * Description: Robot state source file. Odometry, drift correction etc.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */
#include "../individual-robot-behaviour/state.h"

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

/* 
 * Global variable for keeping track of current robot state.
 *
 * Needs to be global since its used between multiple threads to keep track
 * of current robot state.
 */
RobotState current_state;

/*============================================================================*/
/* 
 * Class for a pose in 2D Cartesian space.
 * Copyable but not movable.
 */

/* Default constructor */
Pose::Pose() : x_(0.0), y_(0.0), theta_(0.0) {}

/* Parameterized constructor */
Pose::Pose(double x, double y, double theta) 
{
  SetX(x);
  SetY(y);
  SetTheta(theta);
}

/* Copy constructor */
Pose::Pose(const Pose& other)
  : x_(other.x_), y_(other.y_), theta_(other.theta_) {} 

/* Get the members value */
double Pose::GetX() const 
{ 
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  return x_; 
}
double Pose::GetY() const 
{ 
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  return y_; 
}
double Pose::GetTheta() const 
{ 
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  return theta_; 
}

/*
 * Set the members values
 * Values must be within playing field
 */
void Pose::SetX(double x)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  x_ = (x > PlayingField::kFrameX / 2) ? PlayingField::kFrameX / 2 :
       (x < -PlayingField::kFrameX / 2) ? -PlayingField::kFrameX / 2 :
       x;
}
void Pose::SetY(double y) 
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  y_ = (y > PlayingField::kFrameY / 2) ? PlayingField::kFrameY / 2 :
       (y < -PlayingField::kFrameY / 2) ? -PlayingField::kFrameY / 2 :
       y;
}
void Pose::SetTheta(double theta) 
{
  std::lock_guard<std::mutex> lock(pose_mutex_);

  /* Wrap angle to [-pi, pi] */
  theta_ = atan2(sin(theta), cos(theta));
}

/* A = operator for this class */
Pose& Pose::operator=(const Pose& other)
{
  /* Lock both mutexes to prevent race conditions */
  std::lock_guard<std::mutex> lock(pose_mutex_);
  std::lock_guard<std::mutex> other_lock(other.pose_mutex_);

  if(this == &other)
  {
    return *this;
  }

  (*this).SetX(other.GetX());
  (*this).SetY(other.GetY());
  (*this).SetTheta(other.GetTheta());

  return *this;
}

/* A != operator for this class */
bool Pose::operator!=(const Pose& other) const 
{
  /* Lock both mutexes to prevent race conditions */
  std::lock_guard<std::mutex> lock(pose_mutex_);
  std::lock_guard<std::mutex> other_lock(other.pose_mutex_);

  return (std::fabs(x_ - other.x_) > tolerance_) ||
         (std::fabs(y_ - other.y_) > tolerance_) ||
         (std::fabs(theta_ - other.theta_) > tolerance_);
}

/*============================================================================*/
/* 
 * Class describing the current state of the robot
 * Copyable, not movable.
 */

/* Default constructor */
RobotState::RobotState() : x_(0.0), y_(0.0), theta_(0.0), ball_(false) {}

/* Parameterized constructor */
RobotState::RobotState(double x, double y, double theta, bool ball) 
{
  SetX(x);
  SetY(y);
  SetTheta(theta);
  SetBall(ball);
}

/* Copy constructor */
RobotState::RobotState(const RobotState& other)
  : x_(other.x_), y_(other.y_), theta_(other.theta_), ball_(other.ball_) {} 

/* Get the members value */
double RobotState::GetX() const 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  return x_; 
}
double RobotState::GetY() const 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  return y_; 
}
double RobotState::GetTheta() const 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  return theta_; 
}
bool RobotState::GetBall() const 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);

  return ball_; 
}

/* Set the members values */
void RobotState::SetX(double x) 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  x_ = x; 
}
void RobotState::SetY(double y) 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  y_ = y; 
}
void RobotState::SetTheta(double theta)
{
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  /* Wrap angle to [-pi, pi] */
  theta_ = atan2(sin(theta), cos(theta));
}
void RobotState::SetBall(bool ball) 
{ 
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  
  ball_ = ball; 
}

//==============================================================================

/* 
 * Odom Subscriber
 * Neither copyable nor move-only.
 */
OdomSubscriber::OdomSubscriber()
  : rclcpp::Node("odom_subscriber")
{
  /* Subscribe to odom topic */
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomSubscriber::OdomCallback, this, 
          std::placeholders::_1));
}

/* 
 * Odometry callback function, stores current state in global
 * variable current_state.
 */
void OdomSubscriber::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr 
    msg) const
{
  /* If msg is null then skip */
  if(msg == nullptr)
  {
    return;
  }

  /* Store globally */
  current_state.SetX(msg->pose.pose.position.x);
  current_state.SetY(msg->pose.pose.position.y);
  /* Transform from quaternion to euler angles */
  current_state.SetTheta(tf2::getYaw(msg->pose.pose.orientation));
}

/*============================================================================*/

/* 
 * Calculates angle between current and target pose, assuming a playing field 
 * which follows unit circle coordinations with four quadrants.
 */
double CalculateAngle(double current_x, double current_y, 
    double target_x, double target_y)
{
  double delta_x = target_x - current_x;
  double delta_y = target_y - current_y;

  /* atan2 is not defined for atan2(0.0,0.0) */
  if((delta_x == 0.0) && (delta_y == 0.0)) 
  {
    throw std::invalid_argument(
        "Undefined angle: current and target positions are the same.");
  }

  double theta = std::atan2(delta_y, delta_x);

  return theta;
}

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */
