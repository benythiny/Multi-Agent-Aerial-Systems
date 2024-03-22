#include <task_01_controller/utils.h>

namespace task_01_controller
{

using namespace Eigen;

std::tuple<double, Matrix3d> augmentInputs(const double tilt_x, const double tilt_y, const double force_z, const double heading) {

  Matrix3d des_orientation = getOrientationFromTilts(tilt_x, tilt_y, heading);

  Vector3d body_thrust_force;

  // clang-format off
  body_thrust_force << tan(tilt_x) * force_z,
                       tan(tilt_y) * force_z,
                       force_z;
  // clang-format on

  double body_thrust = body_thrust_force.norm();

  return {body_thrust, des_orientation};
}

Matrix3d getOrientationFromTilts(const double tilt_x, const double tilt_y, const double heading) {

  // | -------------- calculate the prerequsisites -------------- |

  // thrust vector is based on the tilt_x and tilt_y
  // -> the thrust vector's projection to XZ plane has tilt = tilt_x
  // -> the thrust vector's projection to YZ plane has tilt = tilt_y
  // now we reconstruct the thrust vector back from known tilt_x and tilt_y
  Vector3d thrust_vector;

  // clang-format off
  thrust_vector << tan(tilt_x),
                   tan(tilt_y),
                   1.0;
  // clang-format on
  thrust_vector.normalize();

  // | ------------ construct the orientation matrix ------------ |

  Matrix3d des_orientation;

  // desired heading vector
  Vector3d heading_vector(cos(heading), sin(heading), 0);

  // the z axis = thrust vector
  des_orientation.col(2) = thrust_vector;

  // the y axis is is perpendicular to the thrust vector and the heading vector
  des_orientation.col(1) = thrust_vector.cross(heading_vector);

  // the x axis is perpendicular to the y axis and the thrust vector
  des_orientation.col(0) = des_orientation.col(1).cross(thrust_vector);

  // normalize all of the components
  des_orientation.col(0).normalize();
  des_orientation.col(1).normalize();
  des_orientation.col(2).normalize();

  return des_orientation;
}

std::tuple<double, double> getTilts(const MatrixXd &R) {

  double tilt_x;
  double tilt_y;

  const Vector3d thrust = R.col(2);

  tilt_x = atan2(thrust(0), thrust(2));
  tilt_y = atan2(thrust(1), thrust(2));

  return {tilt_x, tilt_y};
}

}  // namespace task_01_controller
