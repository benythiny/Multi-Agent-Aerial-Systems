#ifndef RANGING_H
#define RANGING_H

#include <eigen3/Eigen/Eigen>

namespace task_02_formation
{

typedef struct
{

  // each UAV's measurement of the distance to the robot
  Eigen::VectorXd distances;

} Ranging_t;

}  // namespace task_02_formation

#endif  // RANGING_H
