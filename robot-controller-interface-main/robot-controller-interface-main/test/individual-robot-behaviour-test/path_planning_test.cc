/* path_planning_test.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-23
 * Last modified: 2024-11-16 by Carl Larsson
 * Description: Test file for path planning.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */
#include "../../src/individual-robot-behaviour/path_planning.h"

/* C++ standard library headers */

/* Other .h files */
#include "gtest/gtest.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

/* Project .h files */

/*============================================================================*/
/* DwbController class tests */

/*----------------------------------------------------------------------------*/

/* Test Fixture for DwbController */
class DwbControllerTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    /* Initialize rclcpp */
    rclcpp::init(0, nullptr);

    /* Instantiate DwbController */
    dwb_controller_ = std::make_shared<robot_controller_interface
        ::individual_robot_behaviour::DwbController>();

    /* Reset global flag */
    robot_controller_interface::individual_robot_behaviour::
        atomic_target_reached_flag = 0;
  }

  void TearDown() override
  {
    /* Shutdown rclcpp */
    rclcpp::shutdown();
  }

  std::shared_ptr<robot_controller_interface::individual_robot_behaviour
      ::DwbController> dwb_controller_;
};

/*----------------------------------------------------------------------------*/

/* 
 * Test case for SendTargetPose.
 * To run this test you need baselink to odom transform, baselink to map 
 * transform and nav2 stack up and running beforehand.
 * Easiest way to do this:
 * 1. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
 * 2. ros2 launch slam_toolbox online_async_launch.py
 * 3. ros2 launch nav2_bringup navigation_launch.py
 * Specify the following to run the test: 
 * ./main_test_exe --gtest_also_run_disabled_tests --gtest_filter=DwbControllerTest.DISABLED_SendTargetPoseTest
 */
TEST_F(DwbControllerTest, DISABLED_SendTargetPoseTest)
{
  /* Set target pose */
  robot_controller_interface::individual_robot_behaviour::Pose 
      target_pose(0.6, 0.43, 2.42);
  /* Signal that we have commanded dwb to move to target pose */
  robot_controller_interface::individual_robot_behaviour::
      atomic_move_to_target = true;

  /* Send the target_pose using SendTargetPose */
  dwb_controller_->SendTargetPose(target_pose);

  /*
   * Wait for navigating to target
   */
  rclcpp::Rate rate(10);  // 10 Hz
  while (rclcpp::ok() && (robot_controller_interface::
      individual_robot_behaviour::atomic_target_reached_flag != 1)) 
  {
    rclcpp::spin_some(dwb_controller_->get_node_base_interface());
    rate.sleep();
  }

  /*
   * Once target has been reached we have confirmation it works as it should.
   */
  SUCCEED();
}

/*============================================================================*/
/* LocalPathPlanning function tests */

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* 
 * TODO fix test.
 * Contains forever while loop, not certain if it can be tested using google 
 * test.
 */

/*============================================================================*/
