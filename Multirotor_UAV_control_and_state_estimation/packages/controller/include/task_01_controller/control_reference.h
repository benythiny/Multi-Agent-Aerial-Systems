#ifndef CONTROL_REFERENCE_H
#define CONTROL_REFERENCE_H

#include <eigen3/Eigen/Eigen>

namespace task_01_controller
{

typedef struct
{

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double          heading;

} ControlReference_t;

}  // namespace task_01_controller

#endif  // CONTROL_REFERENCE_H
