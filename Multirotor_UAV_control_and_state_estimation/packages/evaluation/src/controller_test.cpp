#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <std_srvs/Trigger.h>

#include <stdio.h>

//}

namespace task_01_evaluation
{

/* class ControllerTest //{ */

class ControllerTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  std::string _file_path_;
  bool        output_to_file_ = false;
  FILE*       output_file_;

  // | ----------------------- parameters ----------------------- |

  double _main_timer_rate_;
  double _start_timer_rate_;

  std::string     _frame_id_;
  Eigen::MatrixXd _path_;
  bool            _stop_at_waypoints_ = false;
  bool            _use_heading_       = false;
  bool            _relax_heading_     = false;

  double _requirement_rmse_pos_;
  double _requirement_rmse_tilt_;

  // | ----------------------- member vars ---------------------- |

  std::atomic<bool> running_                    = false;
  double            square_pos_error_integral_  = 0;
  double            square_tilt_error_integral_ = 0;
  int               n_samples_                  = 0;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv> sch_path_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::UavState>                  sh_uav_state_;
  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>           sh_position_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_diag_;

  // | --------------------- service servers -------------------- |

  ros::ServiceServer service_server_start_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  ros::Timer timer_start_;
  void       timerStart(const ros::TimerEvent& event);
};

//}

/* onInit() //{ */

void ControllerTest::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ControllerTest");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[ControllerTest]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("file", _file_path_);

  _path_ = param_loader.loadMatrixDynamic2("path", -1, 4);

  param_loader.loadParam("main_timer/rate", _main_timer_rate_);
  param_loader.loadParam("start_timer/rate", _start_timer_rate_);
  param_loader.loadParam("relax_heading", _relax_heading_);
  param_loader.loadParam("use_heading", _use_heading_);
  param_loader.loadParam("stop_at_waypoints", _stop_at_waypoints_);

  param_loader.loadParam("requirements/rmse_position", _requirement_rmse_pos_);
  param_loader.loadParam("requirements/rmse_tilt", _requirement_rmse_tilt_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControllerTest]: could not load all parameters!");
    ros::shutdown();
  }

  if (_file_path_ != "") {
    output_to_file_ = true;
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ControllerTest";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_cmd_in");
  sh_uav_state_    = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in");

  // | --------------------- service clients -------------------- |

  sch_path_ = mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>(nh_, "path_out");

  // | ------------------------- timers ------------------------- |

  timer_start_ = nh_.createTimer(ros::Rate(_start_timer_rate_), &ControllerTest::timerStart, this);
  timer_main_  = nh_.createTimer(ros::Rate(_main_timer_rate_), &ControllerTest::timerMain, this, false, false);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[ControllerTest]: initialized, version %s", VERSION);
}

//}

// | ------------------------ callbacks ----------------------- |

/* timerStart() //{ */

void ControllerTest::timerStart([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  std::stringstream ss;

  if (!sh_uav_state_.hasMsg() || !sh_control_diag_.hasMsg() || !sh_position_cmd_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[ControllerTest]: waiting for data");
    return;
  }

  auto position_cmd = sh_position_cmd_.getMsg();
  auto uav_state    = sh_uav_state_.getMsg();
  auto control_diag = sh_control_diag_.getMsg();

  if (!control_diag->flying_normally) {
    ROS_INFO_THROTTLE(1.0, "[ControllerTest]: waiting for takeoff");
    return;
  }

  square_pos_error_integral_  = 0;
  square_tilt_error_integral_ = 0;
  n_samples_                  = 0;

  mrs_msgs::PathSrv srv;
  srv.request.path.header.frame_id      = _frame_id_;
  srv.request.path.header.stamp         = ros::Time(0);
  srv.request.path.fly_now              = true;
  srv.request.path.use_heading          = _use_heading_;
  srv.request.path.relax_heading        = _relax_heading_;
  srv.request.path.loop                 = false;
  srv.request.path.stop_at_waypoints    = _stop_at_waypoints_;
  srv.request.path.override_constraints = false;

  for (int i = 0; i < _path_.rows(); i++) {

    mrs_msgs::Reference reference;
    reference.position.x = _path_(i, 0);
    reference.position.y = _path_(i, 1);
    reference.position.z = _path_(i, 2);
    reference.heading    = _path_(i, 3);

    srv.request.path.points.push_back(reference);
  }

  bool called = sch_path_.call(srv);

  if (!called || !srv.response.success) {
    ROS_ERROR_THROTTLE(1.0, "[ControllerTest]: failed to set the path, '%s'", srv.response.message.c_str());
    return;
  }

  ROS_INFO("[ControllerTest]: Desired path was set, starting up evaluation timer.");

  ros::Duration(0.3).sleep();

  running_ = true;
  timer_main_.start();
  timer_start_.stop();

  ROS_INFO("[ControllerTest]: starting");
}

//}

/* timerMain() //{ */

void ControllerTest::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  auto position_cmd = *sh_position_cmd_.getMsg();
  auto uav_state    = *sh_uav_state_.getMsg();
  auto control_diag = *sh_control_diag_.getMsg();

  // calculate the control error
  double error_x = position_cmd.position.x - uav_state.pose.position.x;
  double error_y = position_cmd.position.y - uav_state.pose.position.y;
  double error_z = position_cmd.position.z - uav_state.pose.position.z;

  Eigen::Matrix3d attitude_cmd_transform = mrs_lib::AttitudeConverter(position_cmd.orientation);
  Eigen::Matrix3d uav_state_transform    = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

  Eigen::Vector3d uav_z_in_world_desired = attitude_cmd_transform * Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d uav_z_in_world         = uav_state_transform * Eigen::Vector3d(0, 0, 1);

  // calculate the angle between the drone's z axis and the world's z axis
  double tilt_error_ = acos(uav_z_in_world.dot(uav_z_in_world_desired));

  square_pos_error_integral_ += pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2);
  square_tilt_error_integral_ += pow(tilt_error_, 2);
  n_samples_++;

  double rmse_position = sqrt(square_pos_error_integral_ / n_samples_);
  double rmse_tilt     = sqrt(square_tilt_error_integral_ / n_samples_);

  ROS_INFO_THROTTLE(1.0, "[ControllerTest]: testing controller's performance: Position RMSE = %.2f, Tilt RMSE = %.2f", rmse_position, rmse_tilt);

  if (!control_diag.flying_normally) {

    if (output_to_file_) {
      output_file_ = fopen(_file_path_.c_str(), "w+");
    }

    ROS_INFO("[ControllerTest]: ");
    ROS_INFO("[ControllerTest]: ");
    ROS_INFO("[ControllerTest]: ");

    ROS_ERROR("[ControllerTest]: abnormal flight conditions detected, the UAV has probably crashed");

    if (output_to_file_) {
      fprintf(output_file_, "0 %.2f %.2f %.2f %.2f crashed", rmse_position, _requirement_rmse_pos_, rmse_tilt, _requirement_rmse_tilt_);
    }

    if (output_to_file_) {
      fclose(output_file_);
    }

    timer_main_.stop();

    ros::Duration(1.0).sleep();

    ros::requestShutdown();
  }

  if (!control_diag.tracker_status.have_goal) {

    if (output_to_file_) {
      output_file_ = fopen(_file_path_.c_str(), "w+");
    }

    if (rmse_position < _requirement_rmse_pos_ && rmse_tilt < _requirement_rmse_tilt_) {

      ROS_INFO("[ControllerTest]: test finished, PASS, Position RMSE = %.2f (limit %.2f), Tilt RMSE = %.2f (limit %.2f)", rmse_position, _requirement_rmse_pos_,
               rmse_tilt, _requirement_rmse_tilt_);

      if (output_to_file_) {
        fprintf(output_file_, "1 %.2f %.2f %.2f %.2f finished", rmse_position, _requirement_rmse_pos_, rmse_tilt, _requirement_rmse_tilt_);
      }

    } else {

      ROS_ERROR("[ControllerTest]: test finished, FAIL, Position RMSE = %.2f, Tilt RMSE = %.2f", rmse_position, rmse_tilt);

      if (output_to_file_) {
        fprintf(output_file_, "0 %.2f %.2f %.2f %.2f finished", rmse_position, _requirement_rmse_pos_, rmse_tilt, _requirement_rmse_tilt_);
      }
    }

    if (output_to_file_) {
      fclose(output_file_);
    }

    timer_main_.stop();

    ros::Duration(1.0).sleep();

    ros::shutdown();
  }
}  // namespace task_01_evaluation

//}

// | ------------------------ routines ------------------------ |

}  // namespace task_01_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_01_evaluation::ControllerTest, nodelet::Nodelet)
