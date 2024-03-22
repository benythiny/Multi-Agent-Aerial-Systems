#ifndef TASK_03_DIRECTION_H
#define TASK_03_DIRECTION_H

#include <iostream>
#include <eigen3/Eigen/Eigen>

typedef enum
{
  UP,
  DOWN,
  LEFT,
  RIGHT,
  NONE,
} Direction_t;

Direction_t intToDirection(const unsigned int &a) {

  return static_cast<Direction_t>(a);
}

int directionToInt(const Direction_t &dir) {

  return static_cast<int>(dir);
}

/* directionToString() //{ */

std::string directionToString(const Direction_t &dir) {

  switch (dir) {

    case UP: {
      return "UP";
      break;
    }

    case DOWN: {
      return "DOWN";
      break;
    }

    case LEFT: {
      return "LEFT";
      break;
    }

    case RIGHT: {
      return "RIGHT";
      break;
    }

    default: {
      return "NONE";
      break;
    }
  }
}

//}

/* directionToUnitVector() //{ */

Eigen::Vector3d directionToUnitVector(const Direction_t &dir) {

  switch (dir) {

    case UP: {
      return Eigen::Vector3d(0, 1, 0);
      break;
    }

    case DOWN: {
      return Eigen::Vector3d(0, -1, 0);
      break;
    }

    case LEFT: {
      return Eigen::Vector3d(-1, 0, 0);
      break;
    }

    case RIGHT: {
      return Eigen::Vector3d(1, 0, 0);
      break;
    }

    default: {
      break;
    }
  }

  return Eigen::Vector3d(0, 0, 0);
}

//}

#endif  // TASK_03_DIRECTION_H
