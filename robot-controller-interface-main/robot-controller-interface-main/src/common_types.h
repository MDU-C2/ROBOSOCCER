/* common_types.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-19
 * Last modified: 2024-10-15 by Emil Åberg
 * Description: Common types used by the individual robot behaviour program.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_
#define ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_


/* Related .h files */

/* C++ standard library headers */

/* Other .h files */

/* Project .h files */


namespace robot_controller_interface
{

/*============================================================================*/

/*! 
 * @brief Struct describing the playing field dimensions in millimeters (mm).
 */
struct PlayingField
{
  /*!
   * @brief The total length of the frame of the playing field in the X
   * dimension, in millimeters (mm).
   */
  static constexpr double kFrameX = 9600.0;
  /*!
   * @brief The total height of the frame of the playing field in the Y
   * dimension, in millimeters (mm).
   */
  static constexpr double kFrameY = 6600.0;
  /*!
   *@brief The total length of the touch line in the X dimension, in 
   millimeters (mm).
   */
  static constexpr double kTouchLineX = 9000.0;
  /*!
   * @brief The total length of the goal line in the Y dimension, in 
   * millimeters (mm).
   */
  static constexpr double kGoalLineY = 6000.0;
  /*!
   * @brief The total length of the defense area in the X dimension, in 
   * millimeters (mm).
   */
  static constexpr double kDefenseAreaX = 1000.0;
  /*!
   * @brief The total height of the defense area in the Y dimension, in 
   * millimeters (mm).
   */
  static constexpr double kDefenseAreaY = 2000.0;
  /*!
   * @brief The radius of the center circle, in millimeters (mm).
   */
  static constexpr double kCenterCircleRadius = 500.0;
  /*!
   * @brief The total size of the goal in the Y dimension, in millimeters (mm).
   */
  static constexpr double kGoalY = 1000.0;
};

/*============================================================================*/

} /* namespace robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_COMMONTYPES_H_ */
