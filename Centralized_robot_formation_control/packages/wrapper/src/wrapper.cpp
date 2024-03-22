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

#include <mrs_msgs/UavState.h>

#include <student_headers/formation.h>

#include <random>

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/PathSrv.h>
#include <geometry_msgs/Pose.h>

#include <dynamic_reconfigure/server.h>
#include <task_02_wrapper/wrapperConfig.h>

#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <task_02_wrapper/Diagnostics.h>

//}

namespace task_02_wrapper
{

/* class Wrapper //{ */

class Wrapper : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  int         _n_uavs_;
  std::string _position_cmd_topic_;
  std::string _control_manager_diag_topic_;
  std::string _path_srv_;
  std::string _variant_;

  bool       running_ = false;
  ros::Time  running_since_;
  std::mutex mutex_running_since_;

  double     distance_to_robot_;
  std::mutex mutex_distance_to_robot_;

  std::vector<double> noise_dist_means_;

  // | ----------------------- subscribers ---------------------- |

  std::vector<mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>>           sh_pos_cmd_;
  std::vector<mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<geometry_msgs::Pose>                              sh_target_;

  // | --------------------- service clients -------------------- |

  std::vector<mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>> sch_path_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_rviz_marker_;
  mrs_lib::PublisherHandler<visualization_msgs::Marker>      ph_true_target_;
  mrs_lib::PublisherHandler<task_02_wrapper::Diagnostics>    ph_diagnostics_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_student_markers_;

  // | -------------------- dynamic publisher ------------------- |

  visualization_msgs::MarkerArray student_markers_;
  int                             marker_id_ = 0;

  void visualizeCube(const task_02_formation::Position_t &position, const task_02_formation::Color_t &color, const double &size);

  // | ----------------------- rviz timer ----------------------- |

  ros::Timer timer_rviz_;
  void       timerRviz(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | -------------------- diagnostics timer ------------------- |

  ros::Timer timer_diagnostics_;
  void       timerDiagnostics(const ros::TimerEvent &event);
  double     _diagnostics_timer_rate_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  double _column_spacing_;
  int    _x_columns_;
  int    _y_columns_;

  std::string _obstacle_file_;

  // | -------------------- student's library ------------------- |

  std::unique_ptr<task_02_formation::Task02Formation> formation_;

  // | ------------------------- methods ------------------------ |

  double dist(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
  double randd(const double from, const double to);

  // | --------------- methods for action handlers -------------- |

  bool reshapeFormation(const std::vector<std::vector<Eigen::Vector3d>> &paths);
  bool setLeaderPosition(const Eigen::Vector3d &position);

  task_02_formation::ActionHandlers_t action_handlers_;

  task_02_formation::FormationState_t formation_state_;
  ros::Time                           is_static_time_;

  Eigen::Vector3d leader_offset_;

  std::vector<Eigen::Vector3d> abs_positions_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef task_02_wrapper::wrapperConfig           DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(task_02_wrapper::wrapperConfig &params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;

  // | --------------------------- rng -------------------------- |

  std::default_random_engine random_engine_;
};

//}

/* onInit() //{ */

void Wrapper::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "Wrapper");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Wrapper]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("main_rate", _main_timer_rate_);
  param_loader.loadParam("diagnostics_rate", _diagnostics_timer_rate_);
  param_loader.loadParam("n_uavs", _n_uavs_);
  param_loader.loadParam("position_cmd_topic", _position_cmd_topic_);
  param_loader.loadParam("control_manager_diag_topic", _control_manager_diag_topic_);
  param_loader.loadParam("path_srv", _path_srv_);
  param_loader.loadParam("variant", _variant_);

  param_loader.loadParam("robot/xy_deviation", params_.robot_xy_deviation);

  param_loader.loadParam("ranging/rel_deviation", params_.rel_deviation);
  param_loader.loadParam("ranging/abs_deviation", params_.abs_deviation);

  param_loader.loadParam("rviz/spacing", _column_spacing_);
  param_loader.loadParam("rviz/x_columns", _x_columns_);
  param_loader.loadParam("rviz/y_columns", _y_columns_);

  param_loader.loadParam("obstacle_file", _obstacle_file_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Wrapper]: Could not load all parameters!");
    ros::shutdown();
  }

  for (int i = 0; i < _n_uavs_; i++) {
    if (_variant_ == "difficult") {
      noise_dist_means_.push_back(randd(0.95, 1.05));
    } else {
      noise_dist_means_.push_back(1.0);
    }
  }

  // | ----------------------- publishers ----------------------- |

  ph_rviz_marker_     = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "rviz_marker", 1, true);
  ph_true_target_     = mrs_lib::PublisherHandler<visualization_msgs::Marker>(nh_, "true_target_out");
  ph_diagnostics_     = mrs_lib::PublisherHandler<task_02_wrapper::Diagnostics>(nh_, "diagnostics_out");
  ph_student_markers_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "student_markers", 1, true);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Wrapper";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_target_ = mrs_lib::SubscribeHandler<geometry_msgs::Pose>(shopts, "target_pose_in");

  // subscribe to position cmd
  for (int i = 0; i < _n_uavs_; i++) {

    int uav_id = i + 1;

    std::stringstream ss;
    ss << "/uav" << uav_id << "/" << _position_cmd_topic_;
    sh_pos_cmd_.push_back(mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, ss.str()));
  }

  // subscribe control manager diagnostics
  for (int i = 0; i < _n_uavs_; i++) {

    int uav_id = i + 1;

    std::stringstream ss;
    ss << "/uav" << uav_id << "/" << _control_manager_diag_topic_;
    sh_control_manager_diag_.push_back(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, ss.str()));
  }

  // prepare service clients for path
  for (int i = 0; i < _n_uavs_; i++) {

    int uav_id = i + 1;

    std::stringstream ss;
    ss << "/uav" << uav_id << "/" << _path_srv_;
    sch_path_.push_back(mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>(nh_, ss.str()));
  }

  // | --------------------------- drs -------------------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&Wrapper::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | -------------------- student's library ------------------- |

  ROS_INFO("[Wrapper]: instantiating student's formation library");
  formation_ = std::make_unique<task_02_formation::Formation>();

  ROS_INFO("[Wrapper]: initializing student's formation library");
  formation_->init();
  ROS_INFO("[Wrapper]: student's library initialized");

  // | --------------------------- rng -------------------------- |


  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  /* srand(time(NULL)); */

  // | ---------------- bind the action handlers ---------------- |

  ROS_INFO("[Wrapper]: binding the action handlers");

  action_handlers_.reshapeFormation  = std::bind(&Wrapper::reshapeFormation, this, std::placeholders::_1);
  action_handlers_.setLeaderPosition = std::bind(&Wrapper::setLeaderPosition, this, std::placeholders::_1);
  action_handlers_.visualizeCube     = std::bind(&Wrapper::visualizeCube, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  formation_->setActionHandlers(action_handlers_);

  formation_state_.virtual_leader = Eigen::Vector3d::Zero();

  for (int i = 0; i < _n_uavs_; i++) {
    formation_state_.followers.push_back(Eigen::Vector3d::Zero());
    abs_positions_.push_back(Eigen::Vector3d::Zero());
  }

  is_static_time_ = ros::Time(0);

  // | ------------------------- timers ------------------------- |

  ROS_INFO("[Wrapper]: starting timers");

  timer_main_        = nh_.createTimer(ros::Rate(_main_timer_rate_), &Wrapper::timerMain, this);
  timer_rviz_        = nh_.createTimer(ros::Rate(1.0), &Wrapper::timerRviz, this);
  timer_diagnostics_ = nh_.createTimer(ros::Rate(_diagnostics_timer_rate_), &Wrapper::timerDiagnostics, this);

  distance_to_robot_ = std::numeric_limits<double>::max();

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Wrapper]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void Wrapper::callbackDrs(task_02_wrapper::wrapperConfig &params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  ROS_INFO("[Wrapper]: DRS updated");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void Wrapper::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Wrapper]: timerMain() spinning");

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  if (!running_) {

    if (!sh_target_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[Formation]: waiting target coordinates");
      return;
    }

    for (int i = 0; i < _n_uavs_; i++) {
      if (!sh_pos_cmd_[i].hasMsg()) {
        ROS_WARN_THROTTLE(1.0, "[Formation]: waiting for uav%d position cmd", i);
        return;
      }
    }

    ROS_INFO_ONCE("[Formation]: got all position cmds");

    for (int i = 0; i < _n_uavs_; i++) {
      if (!sh_control_manager_diag_[i].hasMsg()) {
        ROS_WARN_THROTTLE(1.0, "[Formation]: waiting for uav%d control manager diagnostics", i);
        return;
      }
    }

    ROS_INFO_ONCE("[Formation]: got all control manager diags");

    for (int i = 0; i < _n_uavs_; i++) {
      auto msg = sh_control_manager_diag_[i].getMsg();
      if (msg->active_tracker != "MpcTracker") {
        ROS_WARN_THROTTLE(1.0, "[Formation]: waiting for uav%d to be flying normally", i);
        return;
      }
    }

    ROS_INFO_ONCE("[Wrapper]: got all data");

    {
      std::scoped_lock lock(mutex_running_since_);

      running_since_ = ros::Time::now();
      running_       = true;
    }
  }

  // | ------------------ extract uav positions ----------------- |

  for (int i = 0; i < _n_uavs_; i++) {
    auto pos_cmd      = sh_pos_cmd_[i].getMsg();
    abs_positions_[i] = Eigen::Vector3d(pos_cmd->position.x, pos_cmd->position.y, pos_cmd->position.z);
  }

  // | ------------ fabricate the range measurements ------------ |

  auto target = sh_target_.getMsg();

  std::normal_distribution<double> norm_robot_xy(0, params.robot_xy_deviation);

  Eigen::Vector3d target_position =
      Eigen::Vector3d(target->position.x + norm_robot_xy(random_engine_), target->position.y + norm_robot_xy(random_engine_), target->position.z);

  Eigen::VectorXd distances = Eigen::VectorXd::Zero(_n_uavs_);

  for (int i = 0; i < _n_uavs_; i++) {

    std::normal_distribution<double> norm_dist_abs(0, params.abs_deviation);
    std::normal_distribution<double> norm_dist_rel(noise_dist_means_[i], params.rel_deviation);

    distances[i] = dist(target_position, abs_positions_[i]) * norm_dist_rel(random_engine_) + norm_dist_abs(random_engine_);
  }

  // | ------------ publish rviz marker of the target ----------- |

  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "common_origin";
    marker.header.stamp    = ros::Time::now();

    marker.pose.position.x = target->position.x;
    marker.pose.position.y = target->position.y;
    marker.pose.position.z = target->position.z;

    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.id = 0;

    ph_true_target_.publish(marker);
  }

  // | ------------ check if formation is stationary ------------ |

  if ((ros::Time::now() - is_static_time_).toSec() >= 1.0) {

    formation_state_.is_static = true;

    for (int i = 0; i < _n_uavs_; i++) {
      auto msg = sh_control_manager_diag_[i].getMsg();
      if (msg->tracker_status.have_goal) {
        formation_state_.is_static = false;
        break;
      }
    }

  } else {

    formation_state_.is_static = false;
  }

  // | --------------- prepare the formation state -------------- |

  // set the followers positions

  for (int i = 0; i < _n_uavs_; i++) {
    formation_state_.followers[i] = abs_positions_[i] - formation_state_.virtual_leader;
  }

  // | ------------- prepare the virtual measurement ------------ |

  task_02_formation::Ranging_t ranging;
  ranging.distances = distances;

  // | ---------------- clean the student markers --------------- |

  student_markers_.markers.clear();
  marker_id_ = 0;

  // clear the previous markers
  {
    visualization_msgs::Marker del;

    del.header.frame_id  = "common_origin";
    del.ns               = "";
    del.id               = 0;
    del.scale.x          = 1.0;
    del.scale.y          = 1.0;
    del.scale.z          = 1.0;
    del.color.a          = 1.0;
    del.type             = visualization_msgs::Marker::CUBE;
    del.pose.position.x  = 10000;
    del.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    del.action           = visualization_msgs::Marker::DELETEALL;

    student_markers_.markers.push_back(del);

    ph_student_markers_.publish(student_markers_);

    student_markers_.markers.clear();
  }

  // | -------------- update the student's library -------------- |

  formation_->update(formation_state_, ranging, ros::Time::now().toSec(), action_handlers_);

  // | --------------- publish the student markers -------------- |

  ph_student_markers_.publish(student_markers_);

  // | -------------------- distance to robot ------------------- |

  {
    Eigen::Vector3d target_position = Eigen::Vector3d(target->position.x, target->position.y, target->position.z);

    double distance_to_robot = (target_position - formation_state_.virtual_leader).norm();

    mrs_lib::set_mutexed(mutex_distance_to_robot_, distance_to_robot, distance_to_robot_);
  }
}

//}

/* timerRviz() //{ */

void Wrapper::timerRviz([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Wrapper]: timerRviz() spinning");

  visualization_msgs::MarkerArray array;

  for (int i = 0; i < _x_columns_; i++) {

    for (int j = 0; j < _y_columns_; j++) {

      visualization_msgs::Marker marker;
      marker.header.frame_id = "common_origin";
      marker.header.stamp    = ros::Time::now();
      marker.ns              = "obstacles";
      marker.id              = i * _x_columns_ + j;
      marker.type            = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action          = visualization_msgs::Marker::ADD;
      marker.mesh_resource   = "file://" + _obstacle_file_;

      marker.pose.position.x = i * _column_spacing_ - ((_x_columns_) / 2.0) * _column_spacing_ + 0.5 * _column_spacing_;
      marker.pose.position.y = j * _column_spacing_ - ((_y_columns_) / 2.0) * _column_spacing_ + 0.5 * _column_spacing_;
      marker.pose.position.z = 0;

      marker.pose.orientation = mrs_lib::AttitudeConverter(1.57, 0, 0);

      marker.scale.x  = 0.001;
      marker.scale.y  = 0.001;
      marker.scale.z  = 0.001;
      marker.color.b  = 0.0f;
      marker.color.g  = 0.0f;
      marker.color.r  = 1.0;
      marker.color.a  = 0.2;
      marker.lifetime = ros::Duration();

      array.markers.push_back(marker);
    }
  }

  ph_rviz_marker_.publish(array);
}

//}

/* timerDiagnostics() //{ */

void Wrapper::timerDiagnostics([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Wrapper]: timerDiagnostics() spinning");

  auto distance_to_robot = mrs_lib::get_mutexed(mutex_distance_to_robot_, distance_to_robot_);
  auto running           = mrs_lib::get_mutexed(mutex_running_since_, running_);
  auto running_since     = mrs_lib::get_mutexed(mutex_running_since_, running_since_);

  task_02_wrapper::Diagnostics diagnostics;

  diagnostics.active            = running;
  diagnostics.failure           = false;
  diagnostics.total_duration    = running ? (ros::Time::now() - running_since).toSec() : 0;
  diagnostics.distance_to_robot = distance_to_robot;

  const Eigen::Vector3i vl_cell   = (formation_state_.virtual_leader / 10.0).array().round().cast<int>();
  diagnostics.virtual_leader_cell = {vl_cell.x(), vl_cell.y()};

  if (running_) {
    for (int i = 0; i < _n_uavs_; i++) {
      auto msg = sh_control_manager_diag_[i].getMsg();
      if (!msg->flying_normally) {
        diagnostics.failure = true;
      }
    }
  }

  ph_diagnostics_.publish(diagnostics);
}

//}

// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------

/* reshapeFormation() //{ */

bool Wrapper::reshapeFormation(const std::vector<std::vector<Eigen::Vector3d>> &paths) {

  ROS_INFO("[Wrapper]: reshapeFormation() called");

  if (!formation_state_.is_static) {

    ROS_WARN("[Wrapper]: reshapeFormation(): formation is not static");
    return false;
  }

  formation_state_.is_static = false;

  // | -------------- check if the paths is filled -------------- |

  if (int(paths.size()) < _n_uavs_) {
    ROS_WARN("[Wrapper]: reshapeFormation(): the supplied paths vector contains wrong number of paths (%d, supposed to be %d)", int(paths.size()), _n_uavs_);
    return false;
  }

  // | ---- check if each path contains at least one waypoint --- |

  for (int i = 0; i < _n_uavs_; i++) {
    if (paths[i].size() == 0) {
      ROS_WARN("[Wrapper]: reshapeFormation(): the path for the UAV #%d is empty", i);
      return false;
    }
  }

  // | ---------- check if the paths don't lead too far --------- |

  for (int i = 0; i < _n_uavs_; i++) {

    // for each waypoint
    for (size_t j = 0; j < paths[i].size(); j++) {

      const double dist = sqrt(pow(paths[i][j][0], 2) + pow(paths[i][j][1], 2) + pow(paths[i][j][2], 2));

      if (dist > 9.0) {
        ROS_WARN("[Wrapper]: paths contain points that are too far from initial leader's position (are you planning relative to leader's position?)");
        return false;
      }
    }
  }

  ros::Time des_time = ros::Time::now() + ros::Duration(1.0);

  for (int i = 0; i < _n_uavs_; i++) {

    int uav_id = i;

    mrs_msgs::PathSrv srv;
    srv.request.path.fly_now       = true;
    srv.request.path.relax_heading = true;
    srv.request.path.use_heading   = false;
    srv.request.path.header.stamp  = des_time;

    // for each waypoint
    for (size_t j = 0; j < paths[i].size(); j++) {

      mrs_msgs::Reference point;
      point.position.x = paths[i][j][0] + formation_state_.virtual_leader[0];
      point.position.y = paths[i][j][1] + formation_state_.virtual_leader[1];
      point.position.z = paths[i][j][2] + formation_state_.virtual_leader[2];

      srv.request.path.points.push_back(point);
    }

    sch_path_[uav_id].callAsync(srv);
  }

  is_static_time_ = ros::Time::now();

  return true;
}

//}

/* setLeaderPosition() //{ */

bool Wrapper::setLeaderPosition(const Eigen::Vector3d &position) {

  ROS_INFO("[Wrapper]: setLeaderPosition() called");

  if (!formation_state_.is_static) {

    ROS_WARN("[Wrapper]: setLeaderPosition(): formation is not static");
    return false;
  }

  formation_state_.is_static = false;

  ros::Time des_time = ros::Time::now() + ros::Duration(1.0);

  // for each uav
  for (int i = 0; i < _n_uavs_; i++) {

    mrs_msgs::PathSrv srv;
    srv.request.path.fly_now       = true;
    srv.request.path.relax_heading = true;
    srv.request.path.use_heading   = false;
    srv.request.path.header.stamp  = des_time;

    Eigen::Vector3d offset = position - formation_state_.virtual_leader;

    mrs_msgs::Reference point;
    point.position.x = abs_positions_[i][0] + offset[0];
    point.position.y = abs_positions_[i][1] + offset[1];
    point.position.z = abs_positions_[i][2] + offset[2];

    srv.request.path.points.push_back(point);

    sch_path_[i].callAsync(srv);
  }

  formation_state_.virtual_leader = position;

  is_static_time_ = ros::Time::now();

  return true;
}

//}

/* dist() //{ */

double Wrapper::dist(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {

  return (a - b).norm();
}

//}

/* randd() //{ */

double Wrapper::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* visualizeCube() //{ */

void Wrapper::visualizeCube(const task_02_formation::Position_t &position, const task_02_formation::Color_t &color, const double &size) {

  if (!std::isfinite(position.x)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"position.x\"!!!");
    return;
  }

  if (!std::isfinite(position.y)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"position.y\"!!!");
    return;
  }

  if (!std::isfinite(position.z)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"position.z\"!!!");
    return;
  }

  if (!std::isfinite(size)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"size\"!!!");
    return;
  }

  if (!std::isfinite(color.alpha)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"color.alpha\"!!!");
    return;
  }

  if (!std::isfinite(color.red)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"color.red\"!!!");
    return;
  }

  if (!std::isfinite(color.green)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"color.green\"!!!");
    return;
  }

  if (!std::isfinite(color.blue)) {
    ROS_ERROR("visualizeCube(): NaN detected in variable \"color.blue\"!!!");
    return;
  }

  visualization_msgs::Marker marker;

  marker.header.frame_id = "common_origin";
  marker.header.stamp    = ros::Time::now();

  marker.pose.position.x = position.x;
  marker.pose.position.y = position.y;
  marker.pose.position.z = position.z;

  marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  marker.color.a = color.alpha;
  marker.color.r = color.red;
  marker.color.g = color.green;
  marker.color.b = color.blue;

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.type = visualization_msgs::Marker::CUBE;

  marker.id = marker_id_++;

  student_markers_.markers.push_back(marker);
}

//}

}  // namespace task_02_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_02_wrapper::Wrapper, nodelet::Nodelet)
