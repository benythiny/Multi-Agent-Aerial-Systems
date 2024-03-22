#ifndef ACTION_HANDLERS_H
#define ACTION_HANDLERS_H

#include <math.h>
#include <eigen3/Eigen/Eigen>

namespace task_02_formation
{

typedef struct
{
  double x;
  double y;
  double z;
} Position_t;

typedef struct
{
  double red;
  double green;
  double blue;
  double alpha;
} Color_t;

typedef std::function<bool(const std::vector<std::vector<Eigen::Vector3d>> &paths)> reshapeFormation_t;

typedef std::function<bool(const Eigen::Vector3d &position)> setLeaderPosition_t;

typedef std::function<void(const Position_t &position, const Color_t &color, const double &size)> visualizeCube_t;

typedef struct
{
  reshapeFormation_t  reshapeFormation;
  setLeaderPosition_t setLeaderPosition;
  visualizeCube_t     visualizeCube;
} ActionHandlers_t;

}  // namespace task_02_formation

#endif  // ACTION_HANDLERS_H
