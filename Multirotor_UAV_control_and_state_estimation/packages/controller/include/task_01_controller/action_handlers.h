#ifndef ACTION_HANDLERS_H
#define ACTION_HANDLERS_H

#include <math.h>
#include <string>

namespace task_01_controller
{

typedef std::function<void(const std::stringstream &str)>                                                                 logLine_t;
typedef std::function<void(const std::string name, const double value)>                                                   plotValue_t;
typedef std::function<void(const std::string name, const double x, const double y, const double z, const double heading)> visualizePose_t;

typedef struct
{
  plotValue_t     plotValue;
  logLine_t       logLine;
  visualizePose_t visualizePose;
} ActionHandlers_t;

}  // namespace task_01_controller

#endif  // ACTION_HANDLERS_H
