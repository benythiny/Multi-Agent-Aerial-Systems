#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <vector>
#include <tuple>
#include <eigen3/Eigen/Eigen>

#include <task_03_common/distribution.h>

typedef struct
{
  Eigen::Vector3d velocity;      // current velocity (in the world frame)
  Eigen::Vector3d target;        // vector of max length 1 towards the target (in the agent frame)
  Distribution    distribution;  // current probability distribution of the colors

  // States of the neighbors:
  //  - position in the agent frame
  //  - velocity in the world frame
  //  - probability distribution of the neighbors' colors
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Distribution>> neighbors_states;

  bool         nearby_beacon;        // true if is nearby a beacon
  Distribution beacon_distribution;  // probability distribution of the nearby beacon

} AgentState_t;

#endif  // AGENT_STATE_H
