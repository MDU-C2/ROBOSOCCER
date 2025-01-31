/* automated_referee.h
 *==============================================================================
 * Author: Aaiza A. Khan, Shruthi P. Kunnon, Emil Åberg
 * Creation date: 2024-10-10
 * Last modified: 2024-10-30 by Emil Åberg
 * Description: Automates referee commands based on robot and ball positions.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef CENTRALISEDAI_SSLINTERFACE_AUTOMATEDREFEREE_H_
#define CENTRALISEDAI_SSLINTERFACE_AUTOMATEDREFEREE_H_

/* C++ standard library headers */
#include "cmath"
#include <cstdlib>
#include "iostream"
#include "string"

/* Project .h files */
#include "../ssl-interface/referee_command_functions.h"
#include "../ssl-interface/ssl_vision_client.h"
#include "../ssl-interface/simulation_reset.h"
#include "../common_types.h"

namespace centralised_ai
{
namespace ssl_interface
{

/*!
 * @brief Class representing an automated referee.
 * 
 * Class representing an automated referee, which @return referee commands for
 * kickoff, freekicks/cornerkicks and keeps track of the score. Mainly intended
 * to be used during AI training so that it can be done without human
 * supervision.
 * 
 * @note not copyable, not moveable.
 */
class AutomatedReferee
{
 public:
  /*!
   * @brief Constructor for instantiating the automated referee.
   *
   * @param[in] vision_client A reference to the vision client.
   *
   * @param[in] ip address of the computer that is running grSim. When running
   * grSim on the same computer that the simulation interface is running on
   * this value should be localhost i.e. "127.0.0.1".
   *
   * @param[in] port The command listen port of grSim. This should
   * be set to the same value as that which is set in the grSim configuration.
   */
  AutomatedReferee(VisionClient& vision_client, std::string grsim_ip,
      uint16_t grsim_port);

  /*!
   * @brief Analyze the game state, needs to be called continously.
   * 
   * @throws std::runtime_error in the event that an unhandled state is reached.
   */
  void AnalyzeGameState();

  /*!
   * @brief Start the automated referee, reset score, referee command,
   * robot and ball positions.
   *
   * @param[in] starting_team The team that has the first kickoff.
   * 
   * @param[in] team_on_positive_half The team that has its goal on the half
   * of the field that is positive in ssl vision's coordinate system
   * 
   * @param[in] prepare_kickoff_duration Time in seconds that the game stays
   * in state the 'Prepare Kickoff Blue/Yellow' before transitioning to the
   * state 'Normal Start'. 
   * 
   * @param[in] stage_time The stage time duration.
   */
  void StartGame(enum Team starting_team, enum Team team_on_positive_half,
      double prepare_kickoff_duration, int64_t stage_time);

  /*!
   * @brief Stop the automated referee, outputs will no longer be updated.
   */
  void StopGame();

  /*!
   * @brief Prints the game controller data that has been read by this client.
   * 
   * Prints the game controller data that has been read by this client including
   * referee command, next referee command, score, ball designated position,
   * remaining stage time and which team has been assigned to the positive half
   * of the field. Used for debugging purposes.
   */
  void Print();

  /*!
   * @brief Returns the referee command.
   * 
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   *
   * @return The referee command.
   */
  enum RefereeCommand GetRefereeCommand();

  /*!
   * @brief Returns the blue team score.
   *
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   *
   * @return The score of the blue team.
   */
  int GetBlueTeamScore();

  /*!
   * @brief Returns the yellow team score.
   *
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called beforehand.
   *
   * @return The score of the yellow team.
   */
  int GetYellowTeamScore();

  /*!
   * @brief Returns the X coordinate of the ball designated position.
   * 
   * Return the X coordinate in mm of the ball designated position. This value
   * is relevant when the BALL_PLACEMENT_YELLOW or BALL_PLACEMENT_BlUE command
   * is issued by the referee, which means that a robot has to bring the ball
   * to the designated position.
   *
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   *
   * @return The X coordinate in mm of the ball designated position.
   */
  float GetBallDesignatedPositionX();

  /*!
   * @brief Returns the Y coordinate of the ball designated position.
   * 
   * Return the Y coordinate in mm of the ball designated position. This value
   * is relevant when the BALL_PLACEMENT_YELLOW or BALL_PLACEMENT_BlUE command
   * is issued by the referee, which means that a robot has to bring the ball
   * to the designated position.
   * 
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   *
   * @return The Y coordinate in mm of the ball designated position.
   */
  float GetBallDesignatedPositionY();

  /*!
   * @brief Returns the team that has been assigned to the positive half of the
   * field.
   *
   * @return The team assigned to the positive half of the field. Returns
   * `kUnknown' if no team is assigned to the positive half.
   *
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   */
  enum Team TeamOnPositiveHalf();

  /*!
   * @brief Returns the remaining stage time left.
   * 
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   *
   * @return The remaining stage time left in seconds. If the stage time is
   * passed this value become negative.
   */
  int64_t GetStageTimeLeft();

  /*!
   * @brief Returns true if the specified robot is currently touching the ball.
   * 
   * @return true if the robot with the given id and team is currently touching
   * the ball.
   * 
   * @param[in] id ID of the robot.
   * 
   * @param[in] team Team color of the robot.
   * 
   * @pre In order to have the data available AnalyzeGameState() needs to be
   * called continously.
   */
  bool IsTouchingBall(int id, enum Team team);

 protected:
  /*!
   * @brief Private struct to represent a point on the field.
   */
  struct Point
  {
    /*!
     * @brief X coordinate in mm.
     */
    float x;

    /*!
     * @brief Y coordinate in mm.
     */
    float y;
  };

  /***********************/
  /* Protected variables */
  /***********************/

  /*!
   * @brief Reference to the vision client.
   */
  VisionClient& vision_client_;

  /*!
   * @brief IP address of grSim.
   */
  std::string grsim_ip_;

  /*!
   * @brief grSim Command listen port.
   */
  uint16_t grsim_port_;

  /*!
   * @brief Time in seconds that the commands PREPARE_KICKOFF_BLUE/YELLOW
   * should stay at.
   */
  double prepare_kickoff_duration_;

  /*!
   * @brief Time at which latest PREPARE_KICKOFF_BLUE/YELLOW command is issued.
   */
  double prepare_kickoff_start_time_;

  /*!
   * @brief Time at which StartGame() was called.
   */
  double time_at_game_start_;

  /*!
   * @brief Flag indicating whether Automatic Referee is running.
   */
  bool game_running_;

  /*!
   * @brief Team that touched the ball last.
   */
  enum Team last_kicker_team_;

  /*!
   * @brief Remaining stage time.
   */
  int64_t stage_time_left_;

  /*!
   * @brief Stage time.
   */
  int64_t stage_time_;

  /*!
   * @brief The latest issued referee command.
   */
  RefereeCommand referee_command_;

  /*!
   * @brief Blue team's score.
   */
  int blue_team_score_;

  /*!
   * @brief Yellow team's score.
   */
  int yellow_team_score_;

  /*!
   * @brief When the BALL_PLACEMENT_YELLOW/BLUE commands are issued, indicates
   * where the ball should be brought for a free kick.
   */
  struct Point designated_position_;

  /*!
   * @brief Indicates which team is on the positive half of the field.
   */
  enum Team team_on_positive_half_;

  /*!
   * @brief distance in mm between robot and ball within which they are
   * considered to be touching each other.
   */
  static constexpr float kCollisionMargin_ = 12;

  /*!
   * @brief Indicates positive half X-coordinate for goal.
   */
  static constexpr float kGoalXPositiveHalf_ = 4500;

  /*!
   * @brief Indicates negative half X-coordinate for goal.
   */
  static constexpr float kGoalXNegativeHalf_ = -4500;

  /*!
   * @brief Indicates minimum Y-coordinate for goal width.
   */
  static constexpr float kGoalWidthMinY_ = -500;

  /*!
   * @brief Indicates maximum Y-coordinate for goal width.
   */
  static constexpr float kGoalWidthMaxY_ = 500;

  /*!
   * @brief Indicates maximum Y-coordinate for ball out of field.
   */
  static constexpr float kBallOutOfFieldMaxY_ = 3000;

  /*!
   * @brief Indicates minimum Y-coordinate for ball out of field.
   */
  static constexpr float kBallOutOfFieldMinY_ = -3000;

  /*********************/
  /* Protected methods */
  /*********************/

  /*!
   * @brief Returns true if the ball is out of the field.
   *
   * @param[in] ball_x The x-coordinate of the ball's position on the field.
   * @param[in] ball_y The y-coordinate of the ball's position on the field.
   *
   * @return `true` if the ball is out of the field, otherwise `false`.
   */
  bool IsBallOutOfField(float ball_x, float ball_y);

  /*!
   * @brief Returns which team is currently touching the ball.
   *
   * @return The team currently touching the ball, or `kUnknown` if no team is
   * in contact.
   */
  enum Team CheckForCollision();

  /*!
   * @brief Returns the distance between the specified robot and center of the
   * ball.
   *
   * @param[in] id The ID of the robot for which the distance is to be
   * calculated.
   * @param[in] team The team to which the robot belongs.
   *
   * @return The distance between the specified robot and the center of the
   * ball.
   */
  float DistanceToBall(int id, enum Team team);

  /*!
   * @brief Returns the distance to the center of the ball and specified point.
   *
   * @param[in] x The x-coordinate of the specified point.
   * @param[in] y The y-coordinate of the specified point.
   *
   * @return The distance between the center of the ball and the specified
   * point.
   */
  float DistanceToBall(float x, float y);

  /*!
   * @brief Returns true when the ball is in the goal of the specified team.
   *
   * @param[in] team The team whose goal is being checked.
   *
   * @return True if the ball is in the specified team's goal, otherwise false.
   */
  bool IsBallInGoal(enum Team team);

  /*!
   * @brief Assuming the ball is out of field, calculates the point where the
   * ball should be placed for a free kick or corner kick.
   *
   * @return The point where the ball should be placed for the free kick or
   * corner kick.
   */
  struct Point CalcBallDesignatedPosition();

  /*!
   * @brief Returns true when the ball is considered 'successfully placed'
   * according to SSL rules.
   *
   * @return True if the ball is successfully placed, false otherwise.
   */
  bool BallSuccessfullyPlaced();

  /*!
   * @brief Updates the current referee command, designated position, score and
   * resets ball and robot position when a goal is scored.
   */
  void RefereeStateHandler();
};

} /* namespace ssl_interface */
} /* namespace centralised_ai   */

#endif /* CENTRALISEDAI_SSLINTERFACE_AUTOMATEDREFEREE_H_ */
