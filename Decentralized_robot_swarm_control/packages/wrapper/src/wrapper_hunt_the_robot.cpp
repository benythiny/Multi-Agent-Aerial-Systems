#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <student_headers/swarm.h>

#include <random>

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <task_03_common/utilities.h>
#include <task_03_wrapper/UserParams.h>
#include <task_03_wrapper/dynamic_publisher.h>

#include <geometry_msgs/Pose.h>

#include <visualization_msgs/MarkerArray.h>

#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>

#include <task_03_wrapper/SwarmMemberState.h>
#include <task_03_wrapper/Diagnostics.h>
#include <task_03_wrapper/CorruptMeasurements.h>

//}

/* defines //{ */

#define TAU 2 * M_PI

//}

namespace task_03_wrapper
{

/* class WrapperHtR //{ */

class WrapperHtR : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  std::string _uav_name_;
  int         _uav_id_;

  int _cell_count_;

  int         _n_uavs_;
  std::string _swarm_member_state_topic_;
  double      _max_velocity_;
  double      _visibility_radius_;
  double      _cell_radius_;
  double      _gate_width_;
  double      _gate_acos_half_width_;

  double _target_noise_stdev_;

  std::mutex      _mutex_velocity_;
  Eigen::Vector3d _velocity_;

  bool       running_ = false;
  ros::Time  running_since_;
  std::mutex mutex_running_since_;

  double     distance_to_robot_;
  std::mutex mutex_distance_to_robot_;

  double _main_timer_rate_;

  std::mutex _mutex_shared_variables_;
  int        _shared_var_int1_ = 0;
  int        _shared_var_int2_ = 0;
  double     _shared_var_dbl_  = 0.0;

  const std::vector<Color_t> colors = {Color_t{1.0, 0.0, 0.0, 1.0}, Color_t{0.0, 1.0, 0.0, 1.0}, Color_t{0.0, 0.0, 1.0, 1.0}, Color_t{1.0, 0.0, 1.0, 1.0},
                                       Color_t{1.0, 1.0, 0.0, 1.0}};
  Color_t                    _uav_color_;

  std::mutex      mutex_position_;
  Eigen::Vector3d _position_ = Eigen::Vector3d::Zero();

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>                      sh_pos_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>            sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<geometry_msgs::Pose>                            sh_target_;
  mrs_lib::SubscribeHandler<task_03_wrapper::UserParams>                    sh_params_;
  mrs_lib::SubscribeHandler<task_03_wrapper::CorruptMeasurements>           sh_randomizer_;
  std::vector<mrs_lib::SubscribeHandler<task_03_wrapper::SwarmMemberState>> sh_swarm_member_state_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::VelocityReferenceStamped> ph_velocity_reference_;
  mrs_lib::PublisherHandler<task_03_wrapper::SwarmMemberState>  ph_swarm_state_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>    ph_vis_array_;
  mrs_lib::PublisherHandler<task_03_wrapper::Diagnostics>       ph_diagnostics_;

  // | ---------------------- visualization --------------------- |
  visualization_msgs::MarkerArray::Ptr msg_vis_array_;
  void                                 visualizeObstacles(const task_03_swarm::Obstacles_t &obstacles);

  // | -------------------- diagnostics timer ------------------- |

  ros::Timer timer_diagnostics_;
  void       timerDiagnostics(const ros::TimerEvent &event);
  double     _diagnostics_timer_rate_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  // | ----------------------- state timer ---------------------- |

  ros::Timer timer_state_;
  void       timerState(const ros::TimerEvent &event);

  // | -------------------- student's library ------------------- |

  std::unique_ptr<task_03_swarm::Swarm> swarm_;

  // | ------------------------- methods ------------------------ |

  double          dist(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
  double          randd(const double from, const double to);
  double          randdnorm(const double from, const double to);
  bool            angleInInterval(const double value, const double from, const double to);
  Eigen::Vector3d saturateVector(const Eigen::Vector3d &vec, const double max_len);

  // | --------------- methods for action handlers -------------- |

  bool setCommand(const Eigen::Vector3d &velocity, const double heading_rate);
  void shareVariables(const int var_int1, const int var_int2, const double var_dbl);
  void visualizeArrow(const std::string &name, const Eigen::Vector3d &endpoint, const Color_t &color);
  void visualizeArrowFrom(const std::string &name, const Eigen::Vector3d &startpoint, const Eigen::Vector3d &endpoint, const Color_t &color);
  void visualizeCube(const std::string &name, const Eigen::Vector3d &center, const Color_t &color, const double size);

  task_03_swarm::ActionHandlers_t action_handlers_;

  // | --------------------------- rng -------------------------- |

  std::default_random_engine random_engine_;
};

//}

/* onInit() //{ */

void WrapperHtR::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "WrapperHtR");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[WrapperHtR]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("main_rate", _main_timer_rate_);
  param_loader.loadParam("diagnostics_rate", _diagnostics_timer_rate_);
  param_loader.loadParam("n_uavs", _n_uavs_);
  param_loader.loadParam("environment/cell_count", _cell_count_);
  param_loader.loadParam("environment/cell_radius", _cell_radius_);
  param_loader.loadParam("environment/gate_width", _gate_width_);
  param_loader.loadParam("target_noise/stdev", _target_noise_stdev_);
  param_loader.loadParam("swarm_member_state_topic", _swarm_member_state_topic_);
  param_loader.loadParam("swarming/max_velocity", _max_velocity_);
  param_loader.loadParam("swarming/visibility_radius", _visibility_radius_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WrapperHtR]: Could not load all parameters!");
    ros::shutdown();
  }

  _gate_acos_half_width_ = std::acos(1.0 - std::pow(_gate_width_ / 2.0, 2) / (2.0 * std::pow(_cell_radius_, 2)));

  // | ----------------------- publishers ----------------------- |

  ph_velocity_reference_ = mrs_lib::PublisherHandler<mrs_msgs::VelocityReferenceStamped>(nh_, "velocity_reference_out", 1, true);
  ph_swarm_state_        = mrs_lib::PublisherHandler<task_03_wrapper::SwarmMemberState>(nh_, "swarm_member_state_out", 1, true);
  ph_vis_array_          = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "swarm_member_vis_array_out", 1, true);
  ph_diagnostics_        = mrs_lib::PublisherHandler<task_03_wrapper::Diagnostics>(nh_, "diagnostics_out");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "WrapperHtR";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_target_               = mrs_lib::SubscribeHandler<geometry_msgs::Pose>(shopts, "target_pose_in");
  sh_params_               = mrs_lib::SubscribeHandler<task_03_wrapper::UserParams>(shopts, "params_in");
  sh_pos_cmd_              = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_cmd_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in");
  sh_randomizer_           = mrs_lib::SubscribeHandler<task_03_wrapper::CorruptMeasurements>(shopts, "randomizer_in");

  // subscribe to position cmd
  for (int i = 0; i < _n_uavs_; i++) {

    unsigned int uav_id = i + 1;

    std::stringstream uav_name;
    uav_name << "uav" << uav_id;

    if (_uav_name_ == uav_name.str()) {

      _uav_id_ = i;

      if (i < int(colors.size())) {
        _uav_color_ = colors.at(i);
      } else {
        _uav_color_ = colors.at(0);
      }

      srand(uav_id);
      continue;
    }

    std::stringstream ss;
    ss << "/uav" << uav_id << "/" << _swarm_member_state_topic_;
    sh_swarm_member_state_.push_back(mrs_lib::SubscribeHandler<task_03_wrapper::SwarmMemberState>(shopts, ss.str()));
  }

  // | -------------------- student's library ------------------- |

  ROS_INFO("[WrapperHtR]: instantiating student's swarm library");
  swarm_ = std::make_unique<task_03_swarm::Swarm>();

  ROS_INFO("[WrapperHtR]: initializing student's swarm library");
  swarm_->init(_visibility_radius_);

  // | --------------------------- rng -------------------------- |

  /* srand(static_cast<unsigned int>(ros::Time::now().nsec)); */
  /* srand(time(NULL)); */

  // | ---------------- bind the action handlers ---------------- |

  /* action_handlers_.setCommand      = std::bind(&WrapperHtR::setCommand, this, std::placeholders::_1); */
  action_handlers_.visualizeArrow = std::bind(&WrapperHtR::visualizeArrow, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  action_handlers_.visualizeArrowFrom =
      std::bind(&WrapperHtR::visualizeArrowFrom, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
  action_handlers_.visualizeCube =
      std::bind(&WrapperHtR::visualizeCube, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
  action_handlers_.shareVariables = std::bind(&WrapperHtR::shareVariables, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // | ------------------------- timers ------------------------- |

  timer_main_        = nh_.createTimer(ros::Rate(_main_timer_rate_), &WrapperHtR::timerMain, this);
  timer_state_       = nh_.createTimer(ros::Rate(_main_timer_rate_), &WrapperHtR::timerState, this);
  timer_diagnostics_ = nh_.createTimer(ros::Rate(_diagnostics_timer_rate_), &WrapperHtR::timerDiagnostics, this);

  distance_to_robot_ = std::numeric_limits<double>::max();

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[WrapperHtR]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void WrapperHtR::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[WrapperHtR]: timerMain() spinning");

  if (!running_) {

    if (!sh_target_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: waiting for target coordinates");
      return;
    }

    if (!sh_params_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: waiting for params");
      return;
    }

    for (int i = 0; i < _n_uavs_ - 1; i++) {
      if (!sh_swarm_member_state_[i].hasMsg()) {
        ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: waiting for all swarm member states");
        return;
      }
    }

    ROS_INFO_ONCE("[WrapperHtR]: got all swarm member states");

    if (!sh_control_manager_diag_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: timerMain(): waiting for control manager diagnostics");
      return;
    }

    if (!sh_pos_cmd_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: timerMain(): waiting for pos cmd");
      return;
    }

    if (!sh_randomizer_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: waiting for randomizer");
      return;
    }

    auto control_diag = sh_control_manager_diag_.getMsg();

    if (!control_diag->flying_normally) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: timerMain(): waiting to be flying normally");
      return;
    }

    ROS_INFO_ONCE("[WrapperHtR]: got all data");

    {
      std::scoped_lock lock(mutex_running_since_);

      running_since_ = ros::Time::now();
      running_       = true;
    }
  }

  auto target  = sh_target_.getMsg();
  auto params  = sh_params_.getMsg();
  auto pos_cmd = sh_pos_cmd_.getMsg();

  std::vector<task_03_wrapper::SwarmMemberState::ConstPtr> swarm_members;
  for (int i = 0; i < _n_uavs_ - 1; i++) {
    swarm_members.push_back(sh_swarm_member_state_[i].getMsg());
  }

  for (int i = 0; i < _n_uavs_ - 1; i++) {
    if (!swarm_members[i]->flying_normally) {
      ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: timerMain(): waiting for all uavs to be flying normally");
      return;
    }
  }

  task_03_swarm::Perception_t perception;

  perception.time = ros::Time::now().toSec();

  // | --------------- my position and orientation -------------- |

  mrs_lib::set_mutexed(mutex_position_, Eigen::Vector3d(pos_cmd->position.x, pos_cmd->position.y, pos_cmd->position.z), _position_);

  /* perception.orientation = mrs_lib::AttitudeConverter(pos_cmd->orientation); */

  // | ---------------------- swarm members --------------------- |

  for (unsigned int i = 0; i < swarm_members.size(); i++) {

    const Eigen::Vector3d neigh_position    = Eigen::Vector3d(swarm_members[i]->position.x, swarm_members[i]->position.y, swarm_members[i]->position.z);
    const Eigen::Vector3d relative_position = neigh_position - _position_;

    const double distance = (neigh_position - _position_).norm();

    if (distance <= _visibility_radius_) {

      task_03_swarm::Neighbor_t neighbor;
      neighbor.position         = relative_position;
      neighbor.shared_variables = {swarm_members[i]->var_int1, swarm_members[i]->var_int2, swarm_members[i]->var_dbl};

      perception.neighbors.push_back(neighbor);
    }
  }

  // | ------------------------ obstacles ----------------------- |
  Eigen::Vector3d closest;

  const int             grid_x      = int(round(_position_.x() / 10)) * 10.0;
  const int             grid_y      = int(round(_position_.y() / 10)) * 10.0;
  const Eigen::Vector3d grid_center = Eigen::Vector3d(grid_x, grid_y, _position_.z());

  const double max_border_coord = 10.0 * _cell_count_ / 2.0;
  const bool   over_x           = std::fabs(_position_.x()) > max_border_coord;
  const bool   over_y           = std::fabs(_position_.y()) > max_border_coord;

  // If in one of the outer cells, place closest point on the border wall
  if (over_x) {
    closest = Eigen::Vector3d(0.0, _position_.y() - grid_center.y(), 0.0);
  } else if (over_y) {
    closest = Eigen::Vector3d(_position_.x() - grid_center.x(), 0.0, 0.0);
  } else {
    // Otherwise project closest point to cell border
    closest = _cell_radius_ * (_position_ - grid_center).normalized();
  }

  const double closest_azimuth = std::atan2(closest.y(), closest.x());

  /* const double ACOS_GATE_HALF_WIDTH = 0.304692654; // 3 m */
  /* const double ACOS_GATE_HALF_WIDTH = 0.201357921;  // 2 m */
  for (const auto &ang : {0.0, M_PI / 2, M_PI, -M_PI / 2}) {

    const double ang_p = ang + _gate_acos_half_width_;
    const double ang_m = ang - _gate_acos_half_width_;

    // In world
    Eigen::Vector3d gate_p = grid_center + _cell_radius_ * Eigen::Vector3d(std::cos(ang_p), std::sin(ang_p), 0.0);
    Eigen::Vector3d gate_m = grid_center + _cell_radius_ * Eigen::Vector3d(std::cos(ang_m), std::sin(ang_m), 0.0);

    // Relative to UAV position
    gate_p = gate_p - _position_;
    gate_m = gate_m - _position_;

    if (!over_x && !over_y && angleInInterval(closest_azimuth, ang_m, ang_p)) {

      const Eigen::Vector3d closer_gate = gate_p.norm() < gate_m.norm() ? gate_p : gate_m;
      closest                           = _position_ + closer_gate - grid_center;
    }

    perception.obstacles.gates.push_back({gate_p, gate_m});
  }

  perception.obstacles.closest = grid_center + closest - _position_;

  // | ------------------------- target ------------------------- |
  auto         randomizer            = sh_randomizer_.getMsg();
  double       azimuth               = std::atan2(target->position.y - pos_cmd->position.y, target->position.x - pos_cmd->position.x);
  const double default_azimuth_noise = randdnorm(0.0, _target_noise_stdev_);

  if (randomizer->uav_id == _uav_id_) {

    switch (randomizer->distribution) {
      case CorruptMeasurements::NONE: {
        azimuth += default_azimuth_noise;
        break;
      }
      case CorruptMeasurements::STATIC: {
        azimuth += randomizer->parameters[0] + default_azimuth_noise;
        break;
      }
      case CorruptMeasurements::UNIFORM: {
        const double offset = randd(randomizer->parameters[0], randomizer->parameters[1]);
        azimuth += offset;
        break;
      }
      case CorruptMeasurements::NORMAL: {
        const double offset = randdnorm(randomizer->parameters[0], randomizer->parameters[1]);
        azimuth += offset;
        break;
      }
      default: {
        break;
      }
    }
  } else {

    // Default noise for others
    azimuth += default_azimuth_noise;
  }

  perception.target_vector[0] = std::cos(azimuth);
  perception.target_vector[1] = std::sin(azimuth);
  perception.target_vector[2] = 0.0;
  /* perception.target_vector.normalize(); */

  // | ----------------------- user params ---------------------- |

  UserParams_t user_params;

  user_params.param1 = params->param1;
  user_params.param2 = params->param2;
  user_params.param3 = params->param3;
  user_params.param4 = params->param4;
  user_params.param5 = params->param5;
  user_params.param6 = params->param6;
  user_params.param7 = params->param7;
  user_params.param8 = params->param8;
  user_params.param9 = params->param9;

  // Clear visualization msg and fill it with obstacles
  visualizeObstacles(perception.obstacles);

  // | -------------- update the student's library -------------- |

  Eigen::Vector3d velocity = swarm_->updateAction(perception, user_params, action_handlers_);

  velocity[2] = 0.0;
  velocity    = saturateVector(velocity, _max_velocity_);

  setCommand(velocity, 0.0);

  {
    std::scoped_lock lock(_mutex_velocity_);
    _velocity_ = velocity;
  }

  ph_vis_array_.publish(msg_vis_array_);

  // | -------------------- distance to robot ------------------- |
  const Eigen::Vector3d vec =
      Eigen::Vector3d(target->position.x - pos_cmd->position.x, target->position.y - pos_cmd->position.y, target->position.z - pos_cmd->position.z);
  mrs_lib::set_mutexed(mutex_distance_to_robot_, vec.norm(), distance_to_robot_);
}

//}

/* timerState() //{ */

void WrapperHtR::timerState([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[WrapperHtR]: timerState() spinning");

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: timerState(): waiting for control manager diagnostics");
    return;
  }

  if (!sh_pos_cmd_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[WrapperHtR]: timerState(): waiting for pos cmd");
    return;
  }

  mrs_msgs::PositionCommand           pos_cmd      = *sh_pos_cmd_.getMsg();
  mrs_msgs::ControlManagerDiagnostics control_diag = *sh_control_manager_diag_.getMsg();

  task_03_wrapper::SwarmMemberState my_state;

  my_state.position        = pos_cmd.position;
  my_state.heading         = pos_cmd.heading;
  my_state.flying_normally = control_diag.flying_normally;
  my_state.uav_name        = _uav_name_;

  {
    std::scoped_lock lock(_mutex_shared_variables_);
    my_state.var_int1 = _shared_var_int1_;
    my_state.var_int2 = _shared_var_int2_;
    my_state.var_dbl  = _shared_var_dbl_;
  }

  ph_swarm_state_.publish(my_state);
}

//}

/* timerDiagnostics() //{ */

void WrapperHtR::timerDiagnostics([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[WrapperHtR]: timerDiagnostics() spinning");

  auto distance_to_robot = mrs_lib::get_mutexed(mutex_distance_to_robot_, distance_to_robot_);
  auto running           = mrs_lib::get_mutexed(mutex_running_since_, running_);
  auto running_since     = mrs_lib::get_mutexed(mutex_running_since_, running_since_);
  auto position          = mrs_lib::get_mutexed(mutex_position_, _position_);

  task_03_wrapper::Diagnostics diagnostics;

  diagnostics.active            = running;
  diagnostics.failure           = false;
  diagnostics.total_duration    = running ? (ros::Time::now() - running_since).toSec() : 0;
  diagnostics.distance_to_robot = distance_to_robot;
  diagnostics.position.x        = position.x();
  diagnostics.position.y        = position.y();
  diagnostics.position.z        = position.z();

  if (running) {
    auto control_diag   = sh_control_manager_diag_.getMsg();
    diagnostics.failure = !control_diag->flying_normally;
  }

  {
    std::scoped_lock lock(_mutex_velocity_);
    diagnostics.moving = _velocity_.norm() > _max_velocity_ / 10.0;
  }


  ph_diagnostics_.publish(diagnostics);
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* setCommand() //{ */

bool WrapperHtR::setCommand(const Eigen::Vector3d &velocity, const double heading_rate) {

  ROS_INFO_ONCE("[WrapperHtR]: setCommand() called");

  mrs_msgs::VelocityReferenceStamped velocity_reference;

  velocity_reference.reference.velocity.x = velocity[0];
  velocity_reference.reference.velocity.y = velocity[1];
  velocity_reference.reference.velocity.z = velocity[2];

  velocity_reference.reference.use_heading_rate = true;
  velocity_reference.reference.heading_rate     = heading_rate;

  ph_velocity_reference_.publish(velocity_reference);

  return true;
}

//}

/* dist() //{ */

double WrapperHtR::dist(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {

  return (a - b).norm();
}

//}
//
/* angleInInterval() //{ */

bool WrapperHtR::angleInInterval(const double angle, const double angle_from, const double angle_to) {
  const double from_to_dist    = mrs_lib::geometry::radians::pdist(angle_from, angle_to);
  const double from_angle_dist = mrs_lib::geometry::radians::pdist(angle_from, angle);
  return from_angle_dist < from_to_dist;
}

//}

/* randd() //{ */

double WrapperHtR::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randdnorm() //{ */

double WrapperHtR::randdnorm(const double mean, const double stdev) {

  std::normal_distribution<double> distribution(mean, stdev);
  return distribution(random_engine_);
}

//}

/* visualizeArrow() //{ */
void WrapperHtR::visualizeArrow(const std::string &name, const Eigen::Vector3d &endpoint, const Color_t &color) {
  visualizeArrowFrom(name, Eigen::Vector3d::Zero(), endpoint, color);
}

//}

/* visualizeArrowFrom() //{ */
void WrapperHtR::visualizeArrowFrom(const std::string &name, const Eigen::Vector3d &startpoint, const Eigen::Vector3d &endpoint, const Color_t &color) {

  visualization_msgs::Marker marker;
  marker.header.frame_id = "common_origin";
  marker.header.stamp    = ros::Time::now();

  marker.points.resize(2);
  marker.points.at(0).x = startpoint.x() + _position_.x();
  marker.points.at(0).y = startpoint.y() + _position_.y();
  marker.points.at(0).z = startpoint.z() + _position_.z();
  marker.points.at(1).x = startpoint.x() + _position_.x() + endpoint.x();
  marker.points.at(1).y = startpoint.y() + _position_.y() + endpoint.y();
  marker.points.at(1).z = startpoint.z() + _position_.z() + endpoint.z();

  marker.pose.orientation.w = 1.0;

  marker.color.a = color.alpha;
  marker.color.r = color.red;
  marker.color.g = color.green;
  marker.color.b = color.blue;

  marker.scale.x = 0.05;
  marker.scale.y = 0.15;
  marker.scale.z = 0.0;

  marker.type = visualization_msgs::Marker::ARROW;

  marker.ns       = name;
  marker.lifetime = ros::Duration(2.0 / _main_timer_rate_);

  msg_vis_array_->markers.push_back(marker);
}

//}

/* visualizeCube() //{ */

void WrapperHtR::visualizeCube(const std::string &name, const Eigen::Vector3d &center, const Color_t &color, const double size) {

  visualization_msgs::Marker marker;

  marker.header.frame_id = "common_origin";
  marker.header.stamp    = ros::Time::now();

  marker.pose.position.x = center.x() + _position_.x();
  marker.pose.position.y = center.y() + _position_.y();
  marker.pose.position.z = center.z() + _position_.z();

  marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  marker.color.a = color.alpha;
  marker.color.r = color.red;
  marker.color.g = color.green;
  marker.color.b = color.blue;

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.type = visualization_msgs::Marker::CUBE;

  marker.ns       = name;
  marker.lifetime = ros::Duration(2.0 / _main_timer_rate_);

  msg_vis_array_->markers.push_back(marker);
}

//}

/* visualizeObstacles() //{ */

void WrapperHtR::visualizeObstacles(const task_03_swarm::Obstacles_t &obstacles) {

  msg_vis_array_ = boost::make_shared<visualization_msgs::MarkerArray>();

  visualization_msgs::Marker marker_gates;
  marker_gates.header.frame_id = "common_origin";
  marker_gates.header.stamp    = ros::Time::now();

  marker_gates.pose.orientation.w = 1.0;

  marker_gates.color.r = _uav_color_.red;
  marker_gates.color.g = _uav_color_.green;
  marker_gates.color.b = _uav_color_.blue;
  marker_gates.color.a = _uav_color_.alpha;

  marker_gates.scale.x = 0.25;
  marker_gates.scale.y = marker_gates.scale.x;
  marker_gates.scale.z = marker_gates.scale.x;

  marker_gates.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_gates.ns   = "obstacles/gates";

  // Store in marker points
  for (const auto &p : obstacles.gates) {
    const auto gate_A = p.first + _position_;
    const auto gate_B = p.second + _position_;

    geometry_msgs::Point point_A;
    point_A.x = gate_A.x();
    point_A.y = gate_A.y();
    point_A.z = gate_A.z();

    geometry_msgs::Point point_B;
    point_B.x = gate_B.x();
    point_B.y = gate_B.y();
    point_B.z = gate_B.z();

    marker_gates.points.push_back(point_A);
    marker_gates.points.push_back(point_B);
  }

  visualization_msgs::Marker marker_closest;
  marker_closest.header  = marker_gates.header;
  marker_closest.color   = marker_gates.color;
  marker_closest.scale.x = 0.35;
  marker_closest.scale.y = marker_closest.scale.x;
  marker_closest.scale.z = marker_closest.scale.x;
  marker_closest.type    = visualization_msgs::Marker::SPHERE;
  marker_closest.ns      = "obstacles/closest";

  marker_closest.pose.position.x    = obstacles.closest.x() + _position_.x();
  marker_closest.pose.position.y    = obstacles.closest.y() + _position_.y();
  marker_closest.pose.position.z    = obstacles.closest.z() + _position_.z();
  marker_closest.pose.orientation.w = 1.0;

  msg_vis_array_->markers.push_back(marker_gates);
  msg_vis_array_->markers.push_back(marker_closest);
}

//}

/* shareVariables() //{ */

void WrapperHtR::shareVariables(const int var_int1, const int var_int2, const double var_dbl) {
  std::scoped_lock lock(_mutex_shared_variables_);
  _shared_var_int1_ = var_int1;
  _shared_var_int2_ = var_int2;
  _shared_var_dbl_  = var_dbl;
}

//}

/*//{ saturateVector() */
Eigen::Vector3d WrapperHtR::saturateVector(const Eigen::Vector3d &vec, const double max_len) {
  if (vec.norm() > max_len) {
    return max_len * vec.normalized();
  }
  return vec;
}
/*//}*/

}  // namespace task_03_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_wrapper::WrapperHtR, nodelet::Nodelet)
