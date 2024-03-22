#ifndef ACTION_HANDLERS_H
#define ACTION_HANDLERS_H

#include <task_03_common/utilities.h>

namespace task_03_swarm
{

typedef struct
{

  Eigen::Vector3d velocity;
  double          heading_rate;

} Command_t;

typedef std::function<void(const int var_int1, const int var_int2, const double var_dbl)>                   shareVariables_t;
typedef std::function<void(const std::string &name, const Eigen::Vector3d &endpoint, const Color_t &color)> visualizeArrow_t;
typedef std::function<void(const std::string &name, const Eigen::Vector3d &startpoint, const Eigen::Vector3d &endpoint, const Color_t &color)>
                                                                                                                             visualizeArrowFrom_t;
typedef std::function<void(const std::string &name, const Eigen::Vector3d &center, const Color_t &color, const double size)> visualizeCube_t;

typedef struct
{
  shareVariables_t     shareVariables;
  visualizeArrow_t     visualizeArrow;
  visualizeArrowFrom_t visualizeArrowFrom;
  visualizeCube_t      visualizeCube;
} ActionHandlers_t;

}  // namespace task_03_swarm

#endif  // ACTION_HANDLERS_H
