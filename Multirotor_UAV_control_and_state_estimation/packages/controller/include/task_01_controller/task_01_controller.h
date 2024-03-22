#ifndef TASK_01_CONTROLLER_H
#define TASK_01_CONTROLLER_H

// load the message headers
#include <task_01_controller/control_reference.h>
#include <task_01_controller/uav_state.h>
#include <task_01_controller/user_params.h>
#include <task_01_controller/action_handlers.h>

#include <iostream>

#include <tuple>

namespace task_01_controller
{

using namespace Eigen;

typedef Eigen::Matrix<double, 9, 9> Matrix9x9d;
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;
typedef Eigen::Matrix<double, 6, 3> Matrix6x3d;
typedef Eigen::Matrix<double, 6, 9> Matrix6x9d;
typedef Eigen::Matrix<double, 9, 3> Matrix9x3d;
typedef Eigen::Matrix<double, 9, 6> Matrix9x6d;
typedef Eigen::Matrix<double, 3, 6> Matrix3x6d;
typedef Eigen::Matrix<double, 3, 9> Matrix3x9d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

class Task01Controller {

public:
  // | ------- the interface to this library, DO NOT MODIFY ------- |

  /**
   * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
   * Use this method to do any heavy pre-computations.
   *
   * @param mass UAV mass [kg]
   * @param g gravitational acceleration [m/s^2]
   * @param action_handlers methods for the user
   * @param action_handlers methods for the user
   */
  virtual void init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers) = 0;

  /**
   * @brief This method is called to reset the internal state of the controller, e.g., just before
   * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
   */
  virtual void reset() = 0;

  /**
   * @brief the main routine, is called to obtain the control signals
   *
   * @param uav_state the measured UAV state, contains position and acceleration
   * @param user_params user-controllable parameters
   * @param control_reference the desired state of the UAV, position, velocity, acceleration, heading
   * @param dt the time difference in seconds between now and the last time calculateControlSignal() got called
   *
   * @return the desired control signal: the total thrust force and the desired orientation
   */
  virtual std::pair<double, Matrix3d> calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params,
                                                             const ControlReference_t control_reference, const double dt) = 0;

  /**
   * @brief Linear Kalman Filter prediction step.
   *
   * @param x old state = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
   * @param x_cov old state covariance
   * @param input current control input: input = [tilt_xz, tilt_yz, acceleration_z]^T, tilt_xz = desired tilt in the world's XZ [rad], tilt_yz = desired
   * tilt in the world's YZ plane [rad], acceleration_z = desired acceleration along world's z-axis [m/s^2]
   * @param dt the time difference in seconds between now and the last iteration
   *
   * @return VectorXd = the new state, MatrixXd = the new state covariance
   */
  std::tuple<Vector9d, Matrix9x9d> lkfPredict(const Vector9d &x, const Matrix9x9d &x_cov, const Vector3d &input, const double &dt);

  /**
   * @brief Linear Kalman Filter correction step
   *
   * @param x old state = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
   * @param x_cov old state covariance
   * @param measurement measurement vector = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T
   * @param dt the time difference in seconds between now and the last iteration
   *
   * @return VectorXd = the new state, MatrixXd = the new state covariance
   */
  std::tuple<Vector9d, Matrix9x9d> lkfCorrect(const Vector9d &x, const Matrix9x9d &x_cov, const Vector6d &measurement, const double &dt);
};

}  // namespace task_01_controller

#endif  // TASK_01_CONTROLLER_H
