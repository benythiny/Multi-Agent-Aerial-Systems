#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <student_headers/controller.h>

#include <random>

#include <stdio.h>

//}

using namespace Eigen;

namespace task_01_evaluation
{

/* class Kalman //{ */

class KalmanTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  std::string     _version_;

  // | ----------------------- parameters ----------------------- |

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE*       _output_file_;

  double _requirement_rmse_pos_;
  double _requirement_rmse_vel_;

  task_01_controller::UserParams_t user_params_;

  // | ------------------ student's controller ------------------ |

  std::unique_ptr<task_01_controller::Controller> controller_;

  // | --------------------- action handlers -------------------- |

  task_01_controller::ActionHandlers_t action_handlers_;

  void visualizePose(const std::string name, const double x, const double y, const double z, const double heading);
  void plotValue(const std::string name, const double value);
  void logLine(const std::stringstream& str);

  // | ----------------- random number generator ---------------- |

  std::default_random_engine random_engine_;

  // | ---------------------- random engine --------------------- |

  double randd(const double from, const double to);
  int    randi(const int from, const int to);

  bool isVectorFinite(const Eigen::VectorXd& vector, const std::string name);
  bool isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name);

  void runtimeError();

  // | ------------------------ routines ------------------------ |

  std::tuple<double, double> getTilts(const MatrixXd& R);

  Matrix3d getOrientationFromTilts(const double tilt_x, const double tilt_y, const double heading);

  std::tuple<double, Matrix3d> augmentInputs(const double tilt_x, const double tilt_y, const double force_z, const double heading);

  // | ----------------------- log stream ----------------------- |

  FILE* student_log_;
};

//}

/* onInit() //{ */

void KalmanTest::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().nsec));

  mrs_lib::ParamLoader param_loader(nh_, "KalmanTest");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[KalmanTest]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("file", _file_path_);

  if (_file_path_ != "") {
    _output_to_file_ = true;
  }

  param_loader.loadParam("param1", user_params_.param1);
  param_loader.loadParam("param2", user_params_.param2);
  param_loader.loadParam("param3", user_params_.param3);
  param_loader.loadParam("param4", user_params_.param4);
  param_loader.loadParam("param5", user_params_.param5);
  param_loader.loadParam("param6", user_params_.param6);
  param_loader.loadParam("param7", user_params_.param7);
  param_loader.loadParam("param8", user_params_.param8);
  param_loader.loadParam("param9", user_params_.param9);
  param_loader.loadParam("param10", user_params_.param10);
  param_loader.loadParam("param11", user_params_.param11);
  param_loader.loadParam("param12", user_params_.param12);

  param_loader.loadParam("requirements/rmse_position", _requirement_rmse_pos_);
  param_loader.loadParam("requirements/rmse_velocity", _requirement_rmse_vel_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ParamServer]: Could not load all parameters!");
    ros::shutdown();
  }

  action_handlers_.plotValue     = std::bind(&KalmanTest::plotValue, this, std::placeholders::_1, std::placeholders::_2);
  action_handlers_.logLine       = std::bind(&KalmanTest::logLine, this, std::placeholders::_1);
  action_handlers_.visualizePose = std::bind(&KalmanTest::visualizePose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                             std::placeholders::_4, std::placeholders::_5);

  // | ------------------------ log file ------------------------ |

  std::string log_file_path = ros::package::getPath("task_01_controller") + "/../../simulation/student_log.txt";

  student_log_ = fopen(log_file_path.c_str(), "w+");

  const double dt   = 0.01;
  const double g    = 9.8;
  const double mass = 2.0;

  task_01_controller::Matrix9x9d A = task_01_controller::Matrix9x9d::Identity();
  A.diagonal().tail<3>()           = task_01_controller::Vector3d(0.95, 0.95, 0.99);
  A.diagonal(3)                    = dt * task_01_controller::Vector6d::Ones();
  A.diagonal(6)                    = 0.5 * dt * dt * task_01_controller::Vector3d::Ones();

  task_01_controller::Matrix9x3d B = task_01_controller::Matrix9x3d::Zero();
  B.block<3, 3>(6, 0).diagonal()   = task_01_controller::Vector3d(0.05, 0.05, 0.01);

  task_01_controller::Vector9d x = task_01_controller::Vector9d::Zero();

  // | ------------ initialize the student's library ------------ |

  controller_ = std::make_unique<task_01_controller::Controller>();
  controller_->init(mass, user_params_, g, action_handlers_);

  // | ---------------- prepare the input vector ---------------- |

  int input_n_steps = randi(5, 15);

  std::vector<double> input_step_lens;

  for (int i = 0; i < input_n_steps; i++) {
    input_step_lens.push_back(randd(1, 3));
  }

  std::vector<double> input_step_signal_x;
  std::vector<double> input_step_signal_y;
  std::vector<double> input_step_signal_z;

  for (int i = 0; i < input_n_steps; i++) {
    input_step_signal_x.push_back(randd(0.15, 0.15));
    input_step_signal_y.push_back(randd(-0.15, 0.15));
    input_step_signal_z.push_back(randd(-1, 1));
  }

  // | ------------------------ simulate ------------------------ |

  task_01_controller::Vector9d   x_student  = task_01_controller::Vector9d::Zero();
  task_01_controller::Matrix9x9d covariance = task_01_controller::Matrix9x9d::Identity();

  ROS_INFO("[KalmanTest]: simulating");

  std::normal_distribution<double> norm_robot_xyz_pos(0, 0.1 / 1.73);
  std::normal_distribution<double> norm_robot_xyz_acc(0, 0.1 / 1.73);

  double error_velocity = 0;
  double error_position = 0;

  int n_steps = 0;

  // for each input step
  /* for (int i = 0; i < input_n_steps; i++) { */
  for (int i = 0; i < 1; i++) {

    // how manu simulation samplea should be made
    int n_samples = int(input_step_lens[i] / dt);

    // for each simulation sample, apply the input and create artificial measurements
    /* for (int j = 0; j < 10; j++) { */
    for (int j = 0; j < n_samples; j++) {

      n_steps++;

      Eigen::Vector3d input = Eigen::Vector3d(input_step_signal_x[i], input_step_signal_y[i], input_step_signal_z[i]);

      std::tie(x_student, covariance) = controller_->lkfPredict(x_student, covariance, input, dt);

      if (x_student.size() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'x' with a wrong size");
        runtimeError();
      }

      if (covariance.cols() != 9 || covariance.rows() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'covariance' with wrong dimensions");
        runtimeError();
      }

      if (!isVectorFinite(x_student, "x_student")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's state vector, coming out of lkfPredict()");
        runtimeError();
      }

      if (!isMatrixFinite(covariance, "covariance")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's covariance matrix, coming out of lkfPredict()");
        runtimeError();
      }

      auto [thrust, R] = augmentInputs(input_step_signal_x[i], input_step_signal_y[i], (input_step_signal_z[i] + g) * mass, 0);

      Eigen::Vector3d des_acc = R * Eigen::Vector3d(0, 0, thrust / mass) - Eigen::Vector3d(0, 0, g);

      x = A * x + B * des_acc;

      // make the measurements noisy
      task_01_controller::Vector6d measurement;
      measurement.head<3>() = x.head<3>();
      measurement.tail<3>() = x.tail<3>();
      for (int it = 0; it < 3; it++)
        measurement(it) += norm_robot_xyz_pos(random_engine_);
      for (int it = 3; it < 6; it++)
        measurement(it) += norm_robot_xyz_acc(random_engine_);

      std::tie(x_student, covariance) = controller_->lkfCorrect(x_student, covariance, measurement, dt);

      if (x_student.size() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'x' with a wrong size");
        runtimeError();
      }

      if (covariance.cols() != 9 || covariance.rows() != 9) {
        ROS_ERROR("[KalmanTest]: lkfPredict() return 'covariance' with wrong dimensions");
        runtimeError();
      }

      if (!isVectorFinite(x_student, "x_student")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's state vector, coming out of lkfCorrect()");
        runtimeError();
      }

      if (!isMatrixFinite(covariance, "covariance")) {
        ROS_ERROR("[KalmanTest]: NaN/Inf found in the student's covariance matrix, coming out of lkfCorrect()");
        runtimeError();
      }

      error_position += (x.head<3>() - x_student.head<3>()).squaredNorm();
      error_velocity += (x.segment<3>(3) - x_student.segment<3>(3)).squaredNorm();
    }
  }

  double rmse_position = sqrt(error_position / n_steps);
  double rmse_velocity = sqrt(error_velocity / n_steps);

  if (_output_to_file_) {
    _output_file_ = fopen(_file_path_.c_str(), "w+");
  }

  if (rmse_position < _requirement_rmse_pos_ && rmse_velocity < _requirement_rmse_vel_) {

    ROS_INFO("[KalmanTest]: simulation finished, PASS, RMSE in pos = %.2f (limit %.2f), RMSE in vel = %.2f (limit %.2f)", rmse_position, _requirement_rmse_pos_,
             rmse_velocity, _requirement_rmse_vel_);

    if (_output_to_file_) {
      fprintf(_output_file_, "1 %.2f %.2f %.2f %.2f", rmse_position, _requirement_rmse_pos_, rmse_velocity, _requirement_rmse_vel_);
    }

  } else {

    ROS_ERROR("[KalmanTest]: simulation finished, FAIL, RMSE in pos = %.2f (limit %.2f), RMSE in vel = %.2f (limit %.2f)", rmse_position,
              _requirement_rmse_pos_, rmse_velocity, _requirement_rmse_vel_);

    if (_output_to_file_) {
      fprintf(_output_file_, "0 %.2f %.2f %.2f %.2f", rmse_position, _requirement_rmse_pos_, rmse_velocity, _requirement_rmse_vel_);
    }
  }

  if (_output_to_file_) {
    fclose(_output_file_);
  }

  fclose(student_log_);

  ros::Duration(1.0).sleep();

  ros::shutdown();
}

//}

// | ---------------- action handlers callbacks --------------- |

/* plotValue() //{ */

void KalmanTest::plotValue([[maybe_unused]] const std::string name, [[maybe_unused]] const double value) {
}

//}

/* logLine() //{ */

void KalmanTest::logLine(const std::stringstream& str) {
  fprintf(student_log_, "%s\n", str.str().c_str());
}

//}

/* visualizePose() //{ */

void KalmanTest::visualizePose([[maybe_unused]] const std::string name, [[maybe_unused]] const double x, [[maybe_unused]] const double y,
                               [[maybe_unused]] const double z, [[maybe_unused]] const double heading) {
}

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double KalmanTest::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int KalmanTest::randi(const int from, const int to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* isVectorFinite() //{ */

bool KalmanTest::isVectorFinite(const Eigen::VectorXd& vector, const std::string name) {

  for (int i = 0; i < vector.size(); i++) {
    if (!std::isfinite(vector[i])) {
      ROS_ERROR("[KalmanTest]: NaN detected in \"%s[%d]\"!!!", name.c_str(), i);
      return false;
    }
  }

  return true;
}

//}

/* isMatrixFinite() //{ */

bool KalmanTest::isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name) {

  for (int i = 0; i < matrix.cols(); i++) {
    for (int j = 0; j < matrix.rows(); j++) {
      if (!std::isfinite(matrix(i, j))) {
        ROS_ERROR("[KalmanTest]: NaN detected in \"%s[%d, %d]\"!!!", name.c_str(), i, j);
        return false;
      }
    }
  }

  return true;
}

//}

/* runtimeError() //{ */

void KalmanTest::runtimeError() {

  if (_output_to_file_) {

    _output_file_ = fopen(_file_path_.c_str(), "w+");

    fprintf(_output_file_, "0 %s %.2f %s %.2f", "Inf", _requirement_rmse_pos_, "Inf", _requirement_rmse_vel_);
    fclose(_output_file_);
  }

  ros::Duration(1.0).sleep();

  ros::shutdown();
}

//}

/* augmentInputs() //{ */

std::tuple<double, Matrix3d> KalmanTest::augmentInputs(const double tilt_x, const double tilt_y, const double force_z, const double heading) {

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

//}

/* getOrientationFromTilts() //{ */

Matrix3d KalmanTest::getOrientationFromTilts(const double tilt_x, const double tilt_y, const double heading) {

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

//}

/* getTilts() //{ */

std::tuple<double, double> KalmanTest::getTilts(const MatrixXd& R) {

  double tilt_x;
  double tilt_y;

  const Vector3d thrust = R.col(2);

  tilt_x = atan2(thrust(0), thrust(2));
  tilt_y = atan2(thrust(1), thrust(2));

  return {tilt_x, tilt_y};
}

//}

}  // namespace task_01_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_01_evaluation::KalmanTest, nodelet::Nodelet)
