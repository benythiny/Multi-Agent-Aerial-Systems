#include <student_headers/controller.h>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief the prediction step of the LKF
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param input current control input: input = [tilt_xz, tilt_yz, acceleration_z]^T, tilt_xz = desired tilt in the world's XZ [rad], tilt_yz = desired
 * tilt in the world's YZ plane [rad], acceleration_z = desired acceleration along world's z-axis [m/s^2]
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfPredict(const Vector9d &x, const Matrix9x9d &x_cov, const Vector3d &input, const double &dt) {

  // x[k+1] = A*x[k] + B*u[k]

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix

  new_x = A * x + B * input;                  // equation 10
  new_x_cov = A * x_cov * A.transpose() + Q;  // equation 11

  // Print the following values into a log file.
  // * the file will be place in "simulation/student_log.txt"
  // * use this for ploting in custom scipts, e.g., using Matlab or Python.
  //
  // std::stringstream string_to_be_logged;
  // string_to_be_logged << std::fixed << dt << ", " << x[0] << ", " << x[1] << ", " << x[2];
  // action_handlers_.logLine(string_to_be_logged);

  return {new_x, new_x_cov};
}

/**
 * @brief LKF filter correction step
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param measurement measurement vector: measurement = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfCorrect(const Vector9d &x, const Matrix9x9d &x_cov, const Vector6d &measurement, const double &dt) {

  Matrix9x9d I = Matrix9x9d::Identity();
  Matrix9x6d K;
  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix

  K = x_cov * H.transpose() * (H * x_cov * H.transpose() + R).inverse(); // eq. 13
  new_x = x + K * (measurement - H * x);  // eq. 14
  new_x_cov = (I - K * H) * x_cov;        // eq. 15

  return {new_x, new_x_cov};
}

}  // namespace task_01_controller
