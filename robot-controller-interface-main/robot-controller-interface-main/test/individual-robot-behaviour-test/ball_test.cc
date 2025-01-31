/* ball_test.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-23
 * Last modified: 2024-11-16 by Carl Larsson
 * Description: Test file for ball source and header files.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */
#include "../../src/individual-robot-behaviour/ball.h"

/* C++ standard library headers */

/* Other .h files */
#include "gtest/gtest.h"

/* Project .h files */
#include "../../src/individual-robot-behaviour/state.h"
#include "../../src/common_types.h"


/*============================================================================*/
/* FindShootTarget function tests */

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* Test playing left and goalie is top */
TEST(FindShootTargetTest, PlayingLeftTop)
{
  bool playing_left = true;
  /* Enemy goal is right */
  double x = robot_controller_interface::PlayingField::kTouchLineX/2;
  /* Goalie located top half of goal */
  double y = robot_controller_interface::PlayingField::kGoalY*3/8;
  /* Irrelevant */
  double theta = 1.356;

  /* Create goalie pose with the given position */
  robot_controller_interface::individual_robot_behaviour::Pose 
      goalie_pose(x, y, theta);
  robot_controller_interface::individual_robot_behaviour::Pose target = 
      robot_controller_interface::individual_robot_behaviour
      ::FindShootTarget(goalie_pose, playing_left);
  EXPECT_DOUBLE_EQ(target.GetX(), x);
  /* We expect to aim in opposite corner of the goal to where the goalie is */
  EXPECT_DOUBLE_EQ(target.GetY(), -y);
}

/* Test playing left and goalie is bottom */
TEST(FindShootTargetTest, PlayingLeftBottom)
{
  bool playing_left = true;
  /* Enemy goal is right */
  double x = robot_controller_interface::PlayingField::kTouchLineX/2;
  /* Goalie located bottom half of goal */
  double y = -robot_controller_interface::PlayingField::kGoalY*3/8;
  /* Irrelevant */
  double theta = -2.0;

  /* Create goalie pose with the given position */
  robot_controller_interface::individual_robot_behaviour::Pose 
      goalie_pose(x, y, theta);
  robot_controller_interface::individual_robot_behaviour::Pose target = 
      robot_controller_interface::individual_robot_behaviour
      ::FindShootTarget(goalie_pose, playing_left);
  EXPECT_DOUBLE_EQ(target.GetX(), x);
  /* We expect to aim in opposite corner of the goal to where the goalie is */
  EXPECT_DOUBLE_EQ(target.GetY(), -y);
}

/* Test playing right and goalie is top */
TEST(FindShootTargetTest, PlayingRightTop)
{
  bool playing_left = false;
  /* Enemy goal is left */
  double x = -robot_controller_interface::PlayingField::kTouchLineX/2;
  /* Goalie located top half of goal */
  double y = robot_controller_interface::PlayingField::kGoalY*3/8;
  /* Irrelevant */
  double theta = -0.9112;

  /* Create goalie pose with the given position */
  robot_controller_interface::individual_robot_behaviour::Pose 
      goalie_pose(x, y, theta);
  robot_controller_interface::individual_robot_behaviour::Pose target = 
      robot_controller_interface::individual_robot_behaviour
      ::FindShootTarget(goalie_pose, playing_left);
  EXPECT_DOUBLE_EQ(target.GetX(), x);
  /* We expect to aim in opposite corner of the goal to where the goalie is */
  EXPECT_DOUBLE_EQ(target.GetY(), -y);

}

/* Test playing right and goalie is bottom */
TEST(FindShootTargetTest, PlayingRightBottom)
{
  bool playing_left = false;
  /* Enemy goal is left */
  double x = -robot_controller_interface::PlayingField::kTouchLineX/2;
  /* Goalie located bottom half of goal */
  double y = -robot_controller_interface::PlayingField::kGoalY*3/8;
  /* Irrelevant */
  double theta = 1.780;

  /* Create goalie pose with the given position */
  robot_controller_interface::individual_robot_behaviour::Pose 
      goalie_pose(x, y, theta);
  robot_controller_interface::individual_robot_behaviour::Pose target = 
      robot_controller_interface::individual_robot_behaviour
      ::FindShootTarget(goalie_pose, playing_left);
  EXPECT_DOUBLE_EQ(target.GetX(), x);
  /* We expect to aim in opposite corner of the goal to where the goalie is */
  EXPECT_DOUBLE_EQ(target.GetY(), -y);
}

/*============================================================================*/
/* ShootSetup function tests */

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* 
 * TODO fix tests.
 * Contains forever while loop, uncertain if i can be tested through a google 
 * test.
 */

/*============================================================================*/
