#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <eigen3/Eigen/Eigen>

namespace task_03_swarm
{

typedef struct
{

  Eigen::Vector3d                                          closest;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> gates;

} Obstacles_t;

typedef struct
{

  int    int1;
  int    int2;
  double dbl;

} SharedVariables_t;

typedef struct
{

  Eigen::Vector3d   position;          // in the body frame
  SharedVariables_t shared_variables;  // shared variables for free use

} Neighbor_t;

typedef struct
{
  double time;

  Eigen::Vector3d target_vector;  // in the body frame

  std::vector<Neighbor_t> neighbors;
  Obstacles_t             obstacles;

} Perception_t;

}  // namespace task_03_swarm

#endif  // PERCEPTION_H
