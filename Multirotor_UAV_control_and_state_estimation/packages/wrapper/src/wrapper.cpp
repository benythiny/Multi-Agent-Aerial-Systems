/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>

#include <mrs_uav_managers/controller.h>

#include <student_headers/controller.h>

#include <random>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <dynamic_reconfigure/server.h>
#include <task_01_wrapper/wrapperConfig.h>

#include <task_01_wrapper/UserParams.h>
#include <task_01_wrapper/dynamic_publisher.h>

#include <mrs_msgs/Float64.h>

#include <ostream>
#include <fstream>

//}

namespace task_01_wrapper
{

/* class Wrapper //{ */

class Wrapper : public mrs_uav_managers::Controller {

public:
  ~Wrapper(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  double _uav_mass_;

  double hover_thrust_;

  ros::Time last_control_time_;

  std::string world_frame_id_;
  std::mutex  mutex_world_frame_id_;

  std::unique_ptr<task_01_controller::Controller> controller_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_measurement_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<task_01_wrapper::UserParams> sh_params_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_initialization_;

  void timerInitialization(const ros::TimerEvent &event);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef task_01_wrapper::wrapperConfig           DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(task_01_wrapper::wrapperConfig &params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;

  // | --------------------- action handlers -------------------- |

  task_01_controller::ActionHandlers_t action_handlers_;

  void plotValue(const std::string name, const double value);
  void logLine(const std::stringstream &stream);
  void visualizePose(const std::string name, const double x, const double y, const double z, const double heading);

  // | -------------------- dynamic publisher ------------------- |

  DynamicPublisher<mrs_msgs::Float64>          publishers_float_;
  DynamicPublisher<geometry_msgs::PoseStamped> publishers_pose_;

  // | ----------------- random number generator ---------------- |

  std::default_random_engine random_engine_;

  // | ----------------------- log stream ----------------------- |

  std::ofstream log_stream_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Wrapper::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space, const double uav_mass,
                         std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = uav_mass;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "Wrapper");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Task01Controller]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- bind the action handlers ---------------- |

  action_handlers_.plotValue     = std::bind(&Wrapper::plotValue, this, std::placeholders::_1, std::placeholders::_2);
  action_handlers_.logLine       = std::bind(&Wrapper::logLine, this, std::placeholders::_1);
  action_handlers_.visualizePose = std::bind(&Wrapper::visualizePose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                             std::placeholders::_4, std::placeholders::_5);

  // | -------------------- dynamic publisher ------------------- |

  publishers_float_ = DynamicPublisher<mrs_msgs::Float64>(nh_);
  publishers_pose_  = DynamicPublisher<geometry_msgs::PoseStamped>(nh_);

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, _uav_mass_ * common_handlers_->g);

  // | --------------------------- drs -------------------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  Drs_t::CallbackType f = boost::bind(&Wrapper::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- publishers ----------------------- |

  ph_measurement_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "measurement");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Wrapper";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_params_ = mrs_lib::SubscribeHandler<task_01_wrapper::UserParams>(shopts, "/param_server/params");

  // | ------------------------- timers ------------------------- |

  timer_initialization_ = nh_.createTimer(ros::Rate(1.0), &Wrapper::timerInitialization, this);

  // | ------------------------ log file ------------------------ |

  std::string log_file_path = ros::package::getPath("task_01_controller") + "/../../simulation/student_log.txt";

  log_stream_.open(log_file_path, std::ofstream::out | std::ofstream::trunc);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[Task01Controller]: initialized");

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool Wrapper::activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[Task01Controller]: activated without getting the last controller's command");

    return false;
  }

  is_active_ = true;

  last_control_time_ = ros::Time::now();

  controller_->reset();

  return true;
}

//}

/* deactivate() //{ */

void Wrapper::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[Task01Controller]: deactivated");
}

//}

/* update() //{ */

const mrs_msgs::AttitudeCommand::ConstPtr Wrapper::update([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                          [[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &control_reference) {

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (control_reference == mrs_msgs::PositionCommand::Ptr()) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  // calculate the dt

  double dt          = (ros::Time::now() - last_control_time_).toSec();
  last_control_time_ = ros::Time::now();

  if (dt > 1.0) {
    dt = 0.01;
    controller_->reset();
  }

  if (dt < 0.001) {
    dt = 0.01;
  }

  // copy the world frame id
  mrs_lib::set_mutexed(mutex_world_frame_id_, uav_state->header.frame_id, world_frame_id_);

  // set the uav state

  task_01_controller::UAVState_t controller_uav_state;

  std::normal_distribution<double> norm_robot_xyz_pos(0, params.position_measurement_std);
  std::normal_distribution<double> norm_robot_xyz_acc(0, params.acceleration_measurement_std);

  controller_uav_state.position[0] = uav_state->pose.position.x + norm_robot_xyz_pos(random_engine_);
  controller_uav_state.position[1] = uav_state->pose.position.y + norm_robot_xyz_pos(random_engine_);
  controller_uav_state.position[2] = uav_state->pose.position.z + norm_robot_xyz_pos(random_engine_);

  controller_uav_state.acceleration[0] = uav_state->acceleration.linear.x + norm_robot_xyz_acc(random_engine_);
  controller_uav_state.acceleration[1] = uav_state->acceleration.linear.y + norm_robot_xyz_acc(random_engine_);
  controller_uav_state.acceleration[2] = uav_state->acceleration.linear.z + norm_robot_xyz_acc(random_engine_);

  controller_uav_state.orientation = mrs_lib::AttitudeConverter(uav_state->pose.orientation);

  controller_uav_state.heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();

  // set the reference

  task_01_controller::ControlReference_t controller_reference;

  controller_reference.position[0] = control_reference->position.x;
  controller_reference.position[1] = control_reference->position.y;
  controller_reference.position[2] = control_reference->position.z;

  controller_reference.velocity[0] = control_reference->velocity.x;
  controller_reference.velocity[1] = control_reference->velocity.y;
  controller_reference.velocity[2] = control_reference->velocity.z;

  controller_reference.acceleration[0] = control_reference->acceleration.x;
  controller_reference.acceleration[1] = control_reference->acceleration.y;
  controller_reference.acceleration[2] = control_reference->acceleration.z;

  controller_reference.heading = control_reference->heading;

  // publish the artificial measurement

  {
    nav_msgs::Odometry measurement;
    measurement.header.stamp    = uav_state->header.stamp;
    measurement.header.frame_id = uav_state->header.frame_id;
    measurement.child_frame_id  = uav_state->header.frame_id;

    measurement.pose.pose.position.x = controller_uav_state.position[0];
    measurement.pose.pose.position.y = controller_uav_state.position[1];
    measurement.pose.pose.position.z = controller_uav_state.position[2];

    measurement.pose.pose.orientation = uav_state->pose.orientation;

    ph_measurement_.publish(measurement);
  }

  // | ----------------------- user params ---------------------- |

  auto user_params_msg = sh_params_.getMsg();

  task_01_controller::UserParams_t user_params;

  user_params.param1  = user_params_msg->param1;
  user_params.param2  = user_params_msg->param2;
  user_params.param3  = user_params_msg->param3;
  user_params.param4  = user_params_msg->param4;
  user_params.param5  = user_params_msg->param5;
  user_params.param6  = user_params_msg->param6;
  user_params.param7  = user_params_msg->param7;
  user_params.param8  = user_params_msg->param8;
  user_params.param9  = user_params_msg->param9;
  user_params.param10 = user_params_msg->param10;
  user_params.param11 = user_params_msg->param11;
  user_params.param12 = user_params_msg->param12;

  // obtain the control signal

  auto [f, Rd] = controller_->calculateControlSignal(controller_uav_state, user_params, controller_reference, dt);

  // convert the control signal

  /* geometry_msgs::Quaternion des_orientation = mrs_lib::AttitudeConverter(orientation); */
  /* double                    des_throttle    = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, thrust); */

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------

  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state->pose.orientation);

  Eigen::Array3d Kq = Eigen::Vector3d(6, 6, 6);

  // orientation error
  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;

  // clang-format off
  Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
  // clang-format on

  double thrust = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, f);

  // prepare the attitude feedback
  Eigen::Vector3d q_feedback = -Kq * Eq.array();

  // | --------------- prepare the attitude output -------------- |

  /* mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand); */
  /* output_command->header.stamp = ros::Time::now(); */

  /* output_command->thrust    = des_throttle; */
  /* output_command->mode_mask = output_command->MODE_ATTITUDE; */

  /* output_command->mass_difference = 0; */
  /* output_command->total_mass      = _uav_mass_; */

  /* output_command->attitude = des_orientation; */

  /* output_command->controller_enforcing_constraints = false; */

  /* output_command->controller = "Task01Controller"; */

  // | --------------- prepare the rate output -------------- |

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->thrust    = thrust;
  output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  output_command->mass_difference = 0;
  output_command->total_mass      = _uav_mass_;

  output_command->attitude = mrs_lib::AttitudeConverter(Rd);

  output_command->attitude_rate.x = q_feedback[0];
  output_command->attitude_rate.y = q_feedback[1];
  output_command->attitude_rate.z = q_feedback[2];

  output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "Task01Controller";

  return output_command;
}

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus Wrapper::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void Wrapper::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void Wrapper::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr Wrapper::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void Wrapper::callbackDrs(task_01_wrapper::wrapperConfig &params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  ROS_INFO("[Wrapper]: DRS updated");
}

//}

/* timerInitialization() //{ */

void Wrapper::timerInitialization([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_params_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[Wrapper]: waiting for user params");
    return;
  }

  auto user_params_msg = sh_params_.getMsg();

  task_01_controller::UserParams_t user_params;

  user_params.param1  = user_params_msg->param1;
  user_params.param2  = user_params_msg->param2;
  user_params.param3  = user_params_msg->param3;
  user_params.param4  = user_params_msg->param4;
  user_params.param5  = user_params_msg->param5;
  user_params.param6  = user_params_msg->param6;
  user_params.param7  = user_params_msg->param7;
  user_params.param8  = user_params_msg->param8;
  user_params.param9  = user_params_msg->param9;
  user_params.param10 = user_params_msg->param10;
  user_params.param11 = user_params_msg->param11;
  user_params.param12 = user_params_msg->param12;

  controller_ = std::make_unique<task_01_controller::Controller>();
  controller_->init(_uav_mass_, user_params, common_handlers_->g, action_handlers_);

  ROS_INFO("[Wrapper]: student's controller initialized");

  timer_initialization_.stop();
}

//}

// | ------------------------ routines ------------------------ |

/* plotValue() //{ */

void Wrapper::plotValue(const std::string name, const double value) {

  mrs_msgs::Float64 msg;
  msg.value = value;

  publishers_float_.publish(name, msg);
}

//}

/* logLine() //{ */

void Wrapper::logLine(const std::stringstream &stream) {
  log_stream_ << stream.str() << std::endl;
  log_stream_ << std::flush;
}

//}

/* visualizePose() //{ */

void Wrapper::visualizePose(const std::string name, const double x, const double y, const double z, const double heading) {

  auto world_frame_id = mrs_lib::get_mutexed(mutex_world_frame_id_, world_frame_id_);

  geometry_msgs::PoseStamped msg;

  msg.header.frame_id = world_frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;

  msg.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(heading);

  publishers_pose_.publish(name, msg);
}

//}

}  // namespace task_01_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_01_wrapper::Wrapper, mrs_uav_managers::Controller)
