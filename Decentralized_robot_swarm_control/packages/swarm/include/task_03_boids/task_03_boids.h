#ifndef TASK_03_BOIDS_H
#define TASK_03_BOIDS_H

// load the message headers
#include <task_03_common/user_params.h>
#include <task_03_common/direction.h>
#include <task_03_boids/action_handlers.h>
#include <task_03_boids/agent_state.h>
#include <tuple>

namespace task_03_boids
{

class Task03Boids {

public:
  // |  LIBRARY INTERFACE: DO NOT MODIFY, YOUR CHANGES WILL BE OVERWRITTEN  |

  virtual std::tuple<Eigen::Vector3d, Distribution> updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                     const ActionHandlers_t &action_handlers) = 0;
};

}  // namespace task_03_boids

#endif  // TASK_03_BOIDS_H
