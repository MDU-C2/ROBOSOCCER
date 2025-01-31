/* ball.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-22
 * Last modified: 2024-12-12 by Carl Larsson
 * Description: Header file for everything that relates to the ball.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_


/* Related .h files */

/* C++ standard library headers */
#include <atomic>
#include <chrono>
#include <thread>

/* Other .h files */

/* Project .h files */
#include "../individual-robot-behaviour/path_planning.h"
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/*! 
 * @brief Find where in goal to aim, aim where goalie is not.
 * 
 * This function takes goalies position and which side of the field is friendly
 * and returns where in goal to aim for kicking the ball.
 *
 * @param[in] goalie_pose The pose of the goalie.
 * @param[in] playing_left Whether the left side of the field is friendly or 
 * not.
 * @return The target position where we want to shoot.
 */
Pose FindShootTarget(Pose goalie_pose, bool playing_left);

/*============================================================================*/

/*!
 * @brief Direct robot towards target and kicks the ball.
 *
 * This function angles the robot towards the commanded target and then 
 * kicks the ball.
 *
 * @param[in] goalie_pose Pointer to goalies pose, not allowed to be nullptr.
 * @param[in] atomic_goal_target Pointer to atomic bool indicating if the
 * target is to shoot at the goal, not allowed to be nullptr.
 * @param[in] atomic_shoot_ball Pointer to atomic bool indicating if the command 
 * to shoot the ball has been given, not allowed to be nullptr.
 * @param[in] atomic_playing_left Pointer to atomic bool indicating if left side 
 * of field is friendly side or not, not allowed to be nullptr.
 * @param[in] atomic_shoot_target Pointer to the shoot target pose, this is 
 * where we aim to shoot, not allowed to be nullptr.
 * @param[in, out] target_pose Pointer to the target position for path 
 * planning, not allowed to be nullptr.
 *
 * @throws std::invalid_argument when arguments are nullptr (robot has not 
 * been initialized correctly)
 *
 * @note This function is dependent on other functions running on seperate 
 * threads.
 *
 * @pre This function requires the following before use:
 * - InitializeRobot must have been called prior to use 
 * - LocalPathPlanning must be running in another thread
 *
 * @warning The function is not guaranteed to work as intended if the 
 * preconditions are not met.
 * @warning If the pointer arguments are null, then robot has not been 
 * initialized and an exception will be thrown.
 */
void ShootSetup(Pose *goalie_pose, std::atomic_bool *atomic_goal_target, 
    std::atomic_bool *atomic_shoot_ball, std::atomic_bool *atomic_playing_left, 
    Pose *shoot_target, Pose *target_pose);

/*============================================================================*/

/*! 
 * @brief Global atomic variable which indicates if shoot_setup function set
 * path planning to work.
 *
 * This global atomic variable indicates whether the shoot_setup function set
 * LocalPathPlanning to work so that the callback function knows which 
 * function it should tell that the task has been completed. 
 *
 * It needs to be global since multiple threads need to access it.
 */
extern std::atomic_bool atomic_shoot_setup_work;

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_BALL_H_ */
