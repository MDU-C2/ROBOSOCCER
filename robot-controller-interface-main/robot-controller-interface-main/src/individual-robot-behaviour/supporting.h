/* supporting.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-26
 * Last modified: 2024-10-12 by Carl Larsson
 * Description: Supporting functions header file. Everything that is required
 * to maintain and allow operation for the main tasks of the robots are 
 * considered supporting and can be found here.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */

/* C++ standard library headers */
#include <atomic>

/* Other .h files */

/* Project .h files */
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

/*============================================================================*/

/*!
 * @brief Intializes robot.
 *
 * Intializes robot by obtaining:
 * - Robot ID
 * - Starting pose
 *
 * @param[in,out] robot_id Pointer to the robot ID. Can be nullptr before 
 * initialization.
 * @param[in,out] target_position Pointer to the target position for path 
 * planning. Can be nullptr before initialization.
 *
 * @warning Failing to run this prior to any robot commands could result in
 * unintended behaviour.
 */
void InitializeRobot(std::atomic_int *robot_id, Pose *target_position);

/*============================================================================*/

/*!
 * @brief Listens to central computer.
 */
void Listener();

/*============================================================================*/

/*!
 * @brief Sends to central computer.
 */
void Sender();

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */
