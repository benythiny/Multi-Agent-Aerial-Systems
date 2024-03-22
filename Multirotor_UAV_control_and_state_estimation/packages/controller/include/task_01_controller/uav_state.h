#ifndef UAV_STATE_H
#define UAV_STATE_H

#include <eigen3/Eigen/Eigen>

namespace task_01_controller
{

typedef struct
{

  Eigen::Vector3d position;
  Eigen::Vector3d acceleration;

  Eigen::Matrix3d orientation;

  double heading;

} UAVState_t;

}  // namespace task_01_controller

#endif  // UAV_STATE_H
