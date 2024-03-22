#ifndef ACTION_HANDLERS_H
#define ACTION_HANDLERS_H

#include <task_03_common/utilities.h>

namespace task_03_boids
{

typedef std::function<void(const std::string &name, const Eigen::Vector3d &endpoint, const Color_t &color)> visualizeArrow_t;

typedef struct
{
  visualizeArrow_t visualizeArrow;
} ActionHandlers_t;

}  // namespace task_03_boids

#endif  // ACTION_HANDLERS_H
