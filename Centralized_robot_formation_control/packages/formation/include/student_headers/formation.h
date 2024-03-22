#ifndef FORMATION_H
#define FORMATION_H

#include <task_02_formation/task_02_formation.h>

#include <student_headers/astar.h>

namespace task_02_formation
{

class Formation : public Task02Formation {

public:
  /**
   * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
   * Use this method do do any heavy pre-computations.
   */
  void init();

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
  std::vector<std::vector<Eigen::Vector3d>> getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                     const std::vector<Eigen::Vector3d> &final_states);

  /**
   * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
   *
   * @param uav_states Vector of 3D positions of each UAV.
   * @param distances Vector of the measured distances from each UAV to the source of signal.
   *
   * @return the estimated 3D position of the source of radiation.
   */
  Eigen::Vector3d multilateration(const std::vector<Eigen::Vector3d> &uav_states, const Eigen::VectorXd &distances);

  /**
   * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz.
   *
   * @param formation_state The current state of the formation. The state contains:
   * - absolute position of the virtual leader UAV
   * - positions of the follower UAVs relative the virtual leader
   * - flag stating whether the formation is moving or whether it is stationary
   * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
   * @param time_stamp Current time in seconds.
   * @param action_handlers This structure provides users with functions to control the formation:
   *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
   *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
   * Moreover, the action_handlers structure provides additional methods for data visualization.
   */
  void update(const FormationState_t &formation_state, const Ranging_t &ranging, const double &time_stamp, ActionHandlers_t &action_handlers);

private:
  // | -------- Put any custom variables and methods here ------- |

  int user_defined_variable_ = 0;
  int current_cell_x = 0;
  int current_cell_y = 0;
  int previos_dir = 0; // 1: x+, 2: x-, 3: y+, 4: y-
  bool beginning = true;
  int attempts = 0;

  Eigen::Vector3d prev_good_s = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d estim_s_start = Eigen::Vector3d(0, 0, 0);
  int mean_iters = 0;
};

}  // namespace task_02_formation

#endif  // FORMATION_H
