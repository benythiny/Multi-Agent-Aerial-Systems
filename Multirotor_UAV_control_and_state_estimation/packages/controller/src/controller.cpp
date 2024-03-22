#include <task_01_controller/utils.h>

#include <student_headers/controller.h>

#include <iostream>

namespace task_01_controller
{

using namespace Eigen;

void Controller::readInputs(const UserParams_t user_params){
  this->Pz = user_params.param1;
  this->Iz = user_params.param2;
  this->Dz = user_params.param3;


  this->Px = user_params.param4;
  this->Py = user_params.param4;

  this->Ix = user_params.param5;
  this->Iy = user_params.param5;

  this->Dx = user_params.param6;
  this->Dy = user_params.param6;
}


/**
 * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param mass UAV mass [kg]
 * @param user_params user-controllable parameters
 * @param g gravitational acceleration [m/s^2]
 * @param action_handlers methods for the user
 */
void Controller::init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers) {

  // copy the mass and the gravity acceleration
  this->_mass_ = mass;
  this->_g_    = g;
  this->Fd = 0;

  // the action handlers will allow you to plot data
  this->action_handlers_ = action_handlers;

  // controller initialization
  readInputs(user_params);
  previous_error = {0.0, 0.0, 0.0};
  integral = {0.0, 0.0, 0.0};

  // Kalman filter initialization
  this->delta_t = 0.01;
  double delta_t_2 = 0.5 * pow(delta_t,2);
  this->coeff_q = user_params.param7;
  this->coeff_r = user_params.param8;

  H << 1, 0, 0, 0, 0,0, 0, 0, 0,
       0, 1, 0, 0, 0,0, 0, 0, 0,
       0, 0, 1, 0, 0,0, 0, 0, 0,
       0, 0, 0, 0, 0,0, 1, 0, 0,
       0, 0, 0, 0, 0,0, 0, 1, 0,
       0, 0, 0, 0, 0,0, 0, 0, 1; 

  Matrix9x9d I9 = Matrix9x9d::Identity();
  Matrix6x6d I6 = Matrix6x6d::Identity();

  Q = I9 * coeff_q;
  R = I6 * coeff_r;

  A << 1, 0, 0, delta_t, 0, 0, delta_t_2, 0, 0,
       0, 1, 0, 0, delta_t, 0, 0, delta_t_2, 0,
       0, 0, 1, 0, 0, delta_t, 0, 0, delta_t_2,
       0, 0, 0, 1, 0, 0, delta_t, 0, 0,
       0, 0, 0, 0, 1, 0, 0, delta_t, 0,
       0, 0, 0, 0, 0, 1, 0, 0, delta_t,
       0, 0, 0, 0, 0, 0, 0.95, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0.95, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0.99;

  B << 0, 0, 0,
       0, 0, 0,
       0, 0, 0,
       0, 0, 0,
       0, 0, 0,
       0, 0, 0,
       0.05, 0, 0,
       0, 0.05, 0,
       0, 0, 0.01;

}

/**
 * @brief This method is called to reset the internal state of the controller, e.g., just before
 * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
 */
void Controller::reset() {

}



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
std::pair<double, Matrix3d> Controller::calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params,
                                                               const ControlReference_t control_reference, const double dt) {

  // Publish the following values as "ROS topics" such that they can be plotted
  // * plotting can be achived using, e.g., the tool called PlotJuggler
  // * try the "plot.sh" script, which will run PlotJuggler
  //
  // action_handlers_.plotValue("pos_x", uav_state.position[0]);
  // action_handlers_.plotValue("pos_y", uav_state.position[1]);
  // action_handlers_.plotValue("pos_z", uav_state.position[2]);

  // publish the following pose as "ROS topic", such that it can be plotted by Rviz
  //
  // action_handlers_.visualizePose("uav_pose_offset", uav_state.position[0], uav_state.position[1], uav_state.position[2] + 1.0, uav_state.heading);

  // Print the following values into a log file.
  // * the file will be place in "simulation/student_log.txt"
  // * use this for ploting in custom scipts, e.g., using Matlab or Python.
  //
  // std::stringstream string_to_be_logged;
  // string_to_be_logged << std::fixed << dt << ", " << uav_state.position[0] << ", " << uav_state.position[1] << ", " << uav_state.position[2];
  // action_handlers_.logLine(string_to_be_logged);

  // | ---------- calculate the output control signals ---------- |

  readInputs(user_params);
  Vector3d error = control_reference.position - uav_state.position;

  // PID:I
  integral += error * dt;

  // PID:D
  Vector3d derivated = (error - previous_error) / dt;

  // PID
  double alpha = Px * error[0] + Dx * derivated[0] + Ix * integral[0];
  double beta  = Py * error[1] + Dy * derivated[1] + Iy * integral[1];
  double r_zd  = Pz * error[2] + Dz * derivated[2] + Iz * integral[2];

  // feedforwards terms

  double f1 = atan2(control_reference.acceleration[0], Fd / _mass_);
  double f2 = atan2(control_reference.acceleration[1], Fd / _mass_);

  // summing up
  double des_tilt_x  = alpha + f1;  // [rad]
  double des_tilt_y  = beta + f2;  // [rad]
  double des_accel_z = r_zd + control_reference.acceleration[2];  // [m/s^2]

  double tilt_limit = 0.35; // 20 degrees

  // | ---------------- add gravity compensation ---------------- |

  des_accel_z += _g_;

  // | --------------- return the control signals --------------- |

  double   body_thrust;
  Matrix3d desired_orientation;

  std::tie(body_thrust, desired_orientation) = augmentInputs(des_tilt_x, des_tilt_y, des_accel_z * _mass_, control_reference.heading);

  previous_error = error;
  Fd = body_thrust;

  return {body_thrust, desired_orientation};
};

}  // namespace task_01_controller
