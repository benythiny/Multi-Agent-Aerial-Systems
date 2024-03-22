#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <tuple>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief combining the decoupled x-tilt, y-tilt and heading into desire 3D orientation
 *
 * @param tilt_x [rad], the component of the UAV tilt that is projected into the worl'd XZ plane
 * @param tilt_y [rad], the component of the UAV tilt that is projected into the worl'd YZ plane
 * @param heading [rad]
 *
 * @return 3D rotational matrix
 */
Matrix3d getOrientationFromTilts(const double tilt_x, const double tilt_y, const double heading);

/**
 * @brief convert between the augmented inputs and the desired orientation and total thrust
 *
 * @param tilt_x tilt in the x-axis direction [rad]
 * @param tilt_z tilt in the y-axis direction [rad]
 * @param force_z force along the z-axis [N]
 * @param heading heading [rad]
 *
 * @return total thrust, orientation R^3x3
 */
std::tuple<double, Matrix3d> augmentInputs(const double tilt_x, const double tilt_y, const double force_z, const double heading);

/**
 * @brief get tilts along world X and Y axes
 *
 * @param R orientation of the UAV
 *
 * @return tilt_x [rad], tilt_y [rad]
 */
std::tuple<double, double> getTilts(const MatrixXd &R);

}  // namespace task_01_controller

#endif  // UTILS_H
