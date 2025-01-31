/* common_types.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-09-24
 * Last modified: 2024-12-12 by Jacob Johansson
 * Description: Common types used by the centralized ai program.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef CENTRALISEDAI_COMMONTYPES_H_
#define CENTRALISEDAI_COMMONTYPES_H_

/* Related .h files */

/* C++ standard library headers */

/* Other .h files */

/* Project .h files */

namespace centralised_ai {

/*!
 * @brief Maximum size of UDP packets received from SSL Vision and
 * SSL Game Controller.
 */
static constexpr int kMaxUdpPacketSize = 65536;

/*!
 * @brief The maximum number of timesteps for the simulation or training.
 */
const int max_timesteps = 201;

/*!
 * @brief Length of the experience replay buffer.
 */
const int buffer_length = 2;

/*!
 * @brief Number of players per team in the simulation or game.
 */
const int amount_of_players_in_team = 6;

/*!
 * @brief Batch size for training, calculated as buffer length multiplied by the
 * number of players in the team.
 */
const int batch_size = buffer_length * amount_of_players_in_team;

/*!
 * @brief The coefficient representing how much the entropy should be considered
 * in the loss function.
 */
const float entropy_coefficient = 0.3;

/*!
 * @brief The clip value representing how much the policy should be clipped in
 * the loss function.
 */
const float clip_value = 0.2;

/*!
 * @brief Represents the number of global states or observation.
 */
const int num_global_states = 21;

/*!
 * @brief Represents the number of local states for each robot.
 */
const int num_local_states = 5;

/*!
 * @brief Number of possible actions each agent can take.
 */
const int num_actions = 6;

/*!
 * @brief Size of the hidden layer in LSTM or RNN-based neural networks.
 */
const int hidden_size = 64;

/*!
 * @brief The robot radius in mm.
 */
static constexpr float kRobotRadius = 85;

/*!
 * @brief The ball radius in mm.
 */
static constexpr float kBallRadius = 21.5;

/*!
 * @brief Enum representing player team selection.
 */
enum class Team {
  /*!
   * @brief Blue team.
   */
  kBlue = 0,
  /*!
   * @brief Yellow team.
   */
  kYellow = 1,
  /*!
   * @brief Team unknown or not set.
   */
  kUnknown = -1
};

/*!
 * @brief Enum with the possible referee commands that can be
 * received from the game controller.
 */
enum class RefereeCommand {
  /**!
   * @brief All robots should completely stop moving.
   */
  kHalt = 0,

  /**!
   * @brief Robots must keep 50 cm from the ball.
   */
  kStop = 1,

  /**!
   * @brief A prepared kickoff or penalty may now be taken.
   */
  kNormalStart = 2,

  /**!
   * @brief The ball is dropped and free for either team.
   */
  kForceStart = 3,

  /**!
   * @brief The yellow team may move into kickoff position.
   * Followed by Normal Start.
   */
  kPrepareKickoffYellow = 4,

  /**!
   * @brief The blue team may move into kickoff position.
   * Followed by Normal Start.
   */
  kPrepareKickoffBlue = 5,

  /**!
   * @brief The yellow team may move into penalty position.
   * Followed by Normal Start.
   */
  kPreparePenaltyYellow = 6,

  /**!
   * @brief The blue team may move into penalty position.
   * Followed by Normal Start.
   */
  kPreparePenaltyBlue = 7,

  /**!
   * @brief The yellow team may take a direct free kick.
   */
  kDirectFreeYellow = 8,

  /**!
   * @brief The blue team may take a direct free kick.
   */
  kDirectFreeBlue = 9,

  /**!
   * @brief The yellow team is currently in a timeout.
   */
  kTimeoutYellow = 12,

  /**!
   * @brief The blue team is currently in a timeout.
   */
  kTimeoutBlue = 13,

  /**!
   * @brief Equivalent to STOP, but the yellow team must pick up the
   * ball and drop it in the Designated Position.
   */
  kBallPlacementYellow = 16,

  /**!
   * @brief Equivalent to STOP, but the blue team must pick up the
   * ball and drop it in the Designated Position.
   */
  kBallPlacementBlue = 17,

  /**!
   * @brief Unknown Command, used when an unrecognized or undefined
   * command is encountered.
   */
  kUnknownCommand = -1
};
} /* namespace centralised_ai */

#endif /* CENTRALISEDAI_COMMONTYPES_H_ */
