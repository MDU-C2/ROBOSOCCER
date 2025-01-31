/* supporting.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-26
 * Last modified: 2024-10-12 by Carl Larsson
 * Description: Supporting functions source file. Everything that is required
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

/* Initializes robot. Be provided with ID, initial pose, */
/* Must be done to be able to start running */
void InitializeRobot(std::atomic_int *robot_id, Pose *target_position)
{
  /* TODO be provided ID for this robot */

  /* TODO get initial pose. */

  /* TODO set target_pose to inital pose to begin with to prevent strange 
   * behaviour */
  /*
  (*target_position).SetX(current_position.GetX());
  (*target_position).SetY(current_position.GetY());
  (*target_position).SetTheta(current_position.GetTheta());
  */
}

/*============================================================================*/

/* Listens to centralized computer. Gets commands and correction data. */
void Listener()
{
  /* TODO get commands*/

  /* TODO get correction data when drift gets to large (timer?) */
}

/*============================================================================*/

/* Sends data to centralized computer. Sends sensor data and robot status. */
void Sender()
{
  /* TODO send sensor data and robot status information (cpu temp etc) to 
   * centralized computer */
}

/*============================================================================*/

} /* namespace individual_robot_behaviour */
} /* namesapce robot_controller_interface */
