#ifndef TASK_03_SWARM_H
#define TASK_03_SWARM_H

// load the message headers
#include <task_03_common/direction.h>
#include <task_03_common/user_params.h>
#include <task_03_swarm/perception.h>
#include <task_03_swarm/action_handlers.h>
#include <task_03_common/utilities.h>

#include <tuple>
#include <set>
#include <map>

namespace task_03_swarm
{

class Task03Swarm {

public:
  // | ------- the interface to this library, DO NOT MODIFY ------- |

  /**
   * @brief the initialization method
   */
  virtual void init(const double visibility_radius) = 0;

  virtual Eigen::Vector3d updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers) = 0;
};

}  // namespace task_03_swarm

#endif  // TASK_03_SWARM_H
