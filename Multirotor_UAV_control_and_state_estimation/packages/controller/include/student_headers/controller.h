#ifndef CONTROLLER_H
#define CONTROLLER_H

// load the message headers
#include <task_01_controller/task_01_controller.h>

namespace task_01_controller
{

using namespace Eigen;

class Controller : public Task01Controller {

public:
  // | ------- the interface to this library, DO NOT MODIFY ------- |

  /**
   * @brief method for reading the user parameters stored in user_params.yaml
  */
  void readInputs(const UserParams_t user_params);

  /**
   * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
   * Use this method to do any heavy pre-computations.
   *
   * @param mass UAV mass [kg]
   * @param g gravitational acceleration [m/s^2]
   * @param action_handlers methods for the user
   */
  void init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers);

  /**
   * @brief This method is called to reset the internal state of the controller, e.g., just before
   * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
   */
  void reset();

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
  std::pair<double, Matrix3d> calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params, const ControlReference_t control_reference,
                                                     const double dt);

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

private:
  // | -------------- put any member variables here ------------- |

  // vars for the controller
  double _mass_;  // the UAV mass
  double _g_;     // the gravitational acceleration
  double Fd;      // body thrust

  double Px;
  double Py;
  double Pz;

  double Dx;
  double Dy;
  double Dz;

  double Ix;
  double Iy;
  double Iz;

  Vector3d previous_error;
  Vector3d integral;

  // vars for the LKF
  Matrix9x9d A;
  Matrix9x3d B;
  Matrix6x9d H;

  Matrix6x6d R;
  Matrix9x9d Q;

  double coeff_r;
  double coeff_q;
  double delta_t;

  // methods for the user
  ActionHandlers_t action_handlers_;
};

}  // namespace task_01_controller

#endif  // CONTROLLER_H
