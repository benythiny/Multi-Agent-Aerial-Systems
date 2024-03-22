#ifndef TASK_02_FORMATION_H
#define TASK_02_FORMATION_H

// load the message headers
#include <task_02_formation/formation_state.h>
#include <task_02_formation/ranging.h>
#include <task_02_formation/target.h>
#include <task_02_formation/action_handlers.h>

#include <tuple>

namespace task_02_formation
{

class Task02Formation {

public:
  // | ------- the interface to this library, DO NOT MODIFY ------- |

  /**
   * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
   * Use this method do do any heavy pre-computations.
   */
  virtual void init(void) = 0;

  /**
   * @brief This method calculates paths for each UAV from an initial state towards a desired final state.
   * This method is supposed to be filled in by the student.
   *
   * @param initial_states A vector of 3D initial positions for each UAV.
   * @param final_states A vector of 3D final positions of each UAV.
   *
   * @return A vector of paths, each path being a vector of 3D positions for each UAV. The initial and final states are supposed
   * to be the part of the path for each UAV. The expected result for point I, as the starting point for a UAV and point F as the final
   * point for a UAV, can be, e.g.:
   *   I -> F
   *   I -> B -> F
   *   I -> B -> C -> F
   * The following paths are considered invalid:
   *   I
   *   F
   *   D -> D
   *   I -> D
   *   F -> I
   */
  virtual std::vector<std::vector<Eigen::Vector3d>> getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                             const std::vector<Eigen::Vector3d> &final_states) = 0;

  /**
   * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
   *
   * @param uav_states Vector of 3D positions of each UAV.
   * @param distances Vector of the measured distances from each UAV to the source of signal.
   *
   * @return the estimated 3D position of the source of radiation.
   */
  virtual Eigen::Vector3d multilateration(const std::vector<Eigen::Vector3d> &uav_states, const Eigen::VectorXd &distances) = 0;

  /**
   * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz.
   *
   * @param formation_state The current state of the formation. The state contains:
   * - the absolute position of the virtual leader UAV (mean of all the UAV positions in the world frame),
   * - the positions of the follower UAVs relative the virtual leader
   * - flag, stating whether the formation is moving or whether it is stationary
   * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
   * @param time_stamp Current time in seconds.
   * @param action_handlers This structure provides users with functions to control the formation:
   *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
   *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
   * Moreover, the action_handlers structure provides additional methods for data visualization.
   */
  virtual void update(const FormationState_t &formation_state, const Ranging_t &ranging, const double &time_stamp, ActionHandlers_t &action_handlers) = 0;

  /**
   * @brief Set action handlers from the wrapper.
   *
   * @param action_handlers
   */
  void setActionHandlers(const ActionHandlers_t &action_handlers) {
    this->action_handlers_ = action_handlers;
  }

protected:
  /**
   * @brief This structure provides users with functions to control the formation:
   *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
   *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
   * Moreover, the action_handlers structure provides additional methods for data visualization.
   */
  ActionHandlers_t action_handlers_;
};

}  // namespace task_02_formation

#endif  // TASK_02_FORMATION_H
