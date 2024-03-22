#ifndef FORMATION_STATE_H
#define FORMATION_STATE_H

#include <eigen3/Eigen/Eigen>

namespace task_02_formation
{

typedef struct
{

  /**
   * @brief is true if the formation is static, stationary, not moving
   */
  bool is_static;

  /**
   * @brief the LEADER's position is absolute int he world frame
   */
  Eigen::Vector3d virtual_leader;

  /**
   * @brief the FOLLOWER's 3D positions are provided relative to the LEADER
   */
  std::vector<Eigen::Vector3d> followers;

} FormationState_t;

}  // namespace task_02_formation

#endif  // FORMATION_STATE_H
