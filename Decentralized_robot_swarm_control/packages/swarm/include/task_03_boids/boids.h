#ifndef BOIDS_H
#define BOIDS_H

#include <task_03_boids/task_03_boids.h>

namespace task_03_boids
{

class Boids : public Task03Boids {

  // DO NOT MODIFY THIS FILES, YOUR CHANGES WOULD BE OVERWRITTEN

public:
  /**
   * @brief Calculate a next-iteration action of one agent given relative information of its neighbors and the direction towards a target.
   *        This method is supposed to be filled in by the student.
   *
   * @param AgentState_t Current state of the agent as defined in agent_state.h.
   * @param user_params user-controllable parameters
   * @param action_handlers functions for visualization
   *  - visualizeArrow() will publish the given arrow in the agent frame within the visualization
   *
   * @return
   *    1) XYZ vector in frame of the agent to be set as velocity command. Beware that i) the vector z-axis component will be set to 0, ii) the vector magnitude
   * will be clamped into <v_min, v_max> limits and iii) azimuth of the vector's XY-projection will be saturated such that the azimuth between the agent's
   * current velocity and the vector does not exceed a maximal change.
   *
   *       Example 1: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 1, 1) will be clamped and saturated to
   * 0.2*(cos(d), sin(d), 0).
   *
   *       Example 2: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 0.05, 1) will
   * be clamped and saturated to 0.1*(cos(-d), sin(-d), 0).
   *
   *    2) Probability distribution of colors to be set to the agent for next iteration. Beware that output distribution.dim() has to equal input
   * state.distribution.dim().
   */
  std::tuple<Eigen::Vector3d, Distribution> updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                             const ActionHandlers_t &action_handlers);
};

}  // namespace task_03_boids

#endif  // BOIDS_H
