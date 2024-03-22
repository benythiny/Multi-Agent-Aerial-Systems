/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/publisher_handler.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <student_headers/formation.h>

#include <random>

#include <stdio.h>

//}

namespace task_02_evaluation
{

/* class ReshapingDebug //{ */

class ReshapingDebug : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  double _requirements_min_uav_dist_;

  std::vector<std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>>> _problems_;

  // | ------------------ student's controller ------------------ |

  std::unique_ptr<task_02_formation::Formation> formation_;

  // | --------------------- action handlers -------------------- |

  task_02_formation::ActionHandlers_t action_handlers_;

  void visualizePose(const double x, const double y, const double z, const double heading);
  void plotValue(const std::string name, const double value);

  // | ------------------------- problem ------------------------ |

  std::vector<Eigen::Vector3d> _from_;
  std::vector<Eigen::Vector3d> _to_;
  int                          _n_uavs_;

  // | ------------------------ solution ------------------------ |

  std::vector<std::vector<Eigen::Vector3d>>                paths_;
  std::vector<std::vector<Eigen::Vector3d>>                trajectories_;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collisions_;

  // | ------------------------- message ------------------------ |

  visualization_msgs::MarkerArray msg_paths_out_;
  visualization_msgs::MarkerArray msg_cubes_out_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_marker_paths_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_marker_cubes_;
  int                                                        published_ = 0;

  // | --------------------- student markers -------------------- |

  visualization_msgs::MarkerArray student_markers_;
  int                             marker_id_ = 0;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | ---------------------- random engine --------------------- |

  bool reshapeFormation(const std::vector<std::vector<Eigen::Vector3d>>& paths);
  bool setLeaderPosition(const Eigen::Vector3d& position);
  void visualizeCube(const task_02_formation::Position_t& position, const task_02_formation::Color_t& color, const double& size);

  bool isVectorFinite(const Eigen::VectorXd& vector, const std::string name);
  bool isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name);

  void runtimeError();

  std::vector<Eigen::Vector3d> samplePath(const std::vector<Eigen::Vector3d>& path, const double& sample_distance);

  bool trajectoriesSafe(const std::vector<Eigen::Vector3d>& traj1, const std::vector<Eigen::Vector3d>& traj2, const double& threshold);

  bool validatePaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<Eigen::Vector3d>& starts,
                     const std::vector<Eigen::Vector3d>& ends);
};

//}

/* onInit() //{ */

void ReshapingDebug::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().nsec));

  mrs_lib::ParamLoader param_loader(nh_, false, "ReshapingDebug");

  Eigen::MatrixXd from = param_loader.loadMatrixDynamic2("initial_states", -1, 3);
  Eigen::MatrixXd to   = param_loader.loadMatrixDynamic2("final_states", -1, 3);

  if (from.rows() != to.rows()) {
    ROS_ERROR("[ParamServer]: the number of 3D points needs to be the same for 'from' and 'to'");
    ros::shutdown();
  }

  _n_uavs_ = from.rows();

  param_loader.loadParam("requirements/min_uav_distance", _requirements_min_uav_dist_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ParamServer]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  ph_marker_paths_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "marker_paths_out", 1, true);
  ph_marker_cubes_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "marker_cubes_out", 1, true);

  // | ------------------------- timers ------------------------- |

  // copy the problem
  for (int i = 0; i < from.rows(); i++) {
    _from_.push_back(from.row(i));
    _to_.push_back(to.row(i));
  }

  action_handlers_.reshapeFormation  = std::bind(&ReshapingDebug::reshapeFormation, this, std::placeholders::_1);
  action_handlers_.setLeaderPosition = std::bind(&ReshapingDebug::setLeaderPosition, this, std::placeholders::_1);
  action_handlers_.visualizeCube     = std::bind(&ReshapingDebug::visualizeCube, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  formation_ = std::make_unique<task_02_formation::Formation>();

  formation_->setActionHandlers(action_handlers_);

  formation_->init();

  paths_ = formation_->getPathsReshapeFormation(_from_, _to_);

  for (size_t i = 0; i < paths_.size(); i++) {
    ROS_INFO("[ReshapingDebug]: path #%d is %d waypoints long", int(i), int(paths_[i].size()));
  }

  if (validatePaths(paths_, _from_, _to_)) {

    ROS_INFO("[ReshapingDebug]: the paths are valid");

    for (int j = 0; j < _n_uavs_; j++) {
      trajectories_.push_back(samplePath(paths_[j], 0.1));
    }

    for (int a = 0; a < _n_uavs_; a++) {
      for (int b = a + 1; b < _n_uavs_; b++) {
        if (!trajectoriesSafe(trajectories_[a], trajectories_[b], _requirements_min_uav_dist_)) {
          ROS_ERROR("[ReshapingDebug]: the check failed for path #%d and #%d", a + 1, b + 1);
        }
      }
    }

  } else {
    ROS_ERROR("[ReshapingDebug]: the paths failed the validation check");
  }

  // | ------------------ construct the message ----------------- |

  int id = 0;

  // add starts
  for (size_t i = 0; i < _from_.size(); i++) {

    // cube
    {
      visualization_msgs::Marker waypoint;
      waypoint.header.frame_id = "world_frame";
      waypoint.id              = id++;
      waypoint.type            = visualization_msgs::Marker::CUBE;
      waypoint.ns              = "markers";

      waypoint.scale.x = 0.3;
      waypoint.scale.y = 0.3;
      waypoint.scale.z = 0.3;

      waypoint.pose.position.x  = _from_[i][0];
      waypoint.pose.position.y  = _from_[i][1];
      waypoint.pose.position.z  = _from_[i][2];
      waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      waypoint.color.r = 1.0;
      waypoint.color.g = 0.0;
      waypoint.color.b = 0.0;
      waypoint.color.a = 1.0;

      msg_paths_out_.markers.push_back(waypoint);
    }

    // text
    {
      visualization_msgs::Marker waypoint;
      waypoint.header.frame_id = "world_frame";
      waypoint.id              = id++;
      waypoint.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
      waypoint.ns              = "markers";

      waypoint.scale.x = 1.0;
      waypoint.scale.y = 1.0;
      waypoint.scale.z = 1.0;

      waypoint.pose.position.x  = _from_[i][0];
      waypoint.pose.position.y  = _from_[i][1];
      waypoint.pose.position.z  = _from_[i][2] + 1;
      waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      waypoint.color.r = 0.0;
      waypoint.color.g = 0.0;
      waypoint.color.b = 0.0;
      waypoint.color.a = 1.0;

      std::stringstream ss;
      ss << "start " << i;
      waypoint.text = ss.str();

      msg_paths_out_.markers.push_back(waypoint);
    }
  }

  // add ends
  for (size_t i = 0; i < _from_.size(); i++) {

    {
      visualization_msgs::Marker waypoint;
      waypoint.header.frame_id = "world_frame";
      waypoint.id              = id++;
      waypoint.type            = visualization_msgs::Marker::CUBE;
      waypoint.ns              = "markers";

      waypoint.scale.x = 0.3;
      waypoint.scale.y = 0.3;
      waypoint.scale.z = 0.3;

      waypoint.pose.position.x  = _to_[i][0];
      waypoint.pose.position.y  = _to_[i][1];
      waypoint.pose.position.z  = _to_[i][2];
      waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      waypoint.color.r = 0.0;
      waypoint.color.g = 0.0;
      waypoint.color.b = 1.0;
      waypoint.color.a = 1.0;

      msg_paths_out_.markers.push_back(waypoint);
    }

    // text
    {
      visualization_msgs::Marker waypoint;
      waypoint.header.frame_id = "world_frame";
      waypoint.id              = id++;
      waypoint.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
      waypoint.ns              = "markers";

      waypoint.scale.x = 1.0;
      waypoint.scale.y = 1.0;
      waypoint.scale.z = 1.0;

      waypoint.pose.position.x  = _to_[i][0];
      waypoint.pose.position.y  = _to_[i][1];
      waypoint.pose.position.z  = _to_[i][2] + 1;
      waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      waypoint.color.r = 0.0;
      waypoint.color.g = 0.0;
      waypoint.color.b = 0.0;
      waypoint.color.a = 1.0;

      std::stringstream ss;
      ss << "goal " << i;
      waypoint.text = ss.str();

      msg_paths_out_.markers.push_back(waypoint);
    }
  }

  // add path waypoints
  for (size_t i = 0; i < paths_.size(); i++) {

    for (size_t j = 0; j < paths_[i].size(); j++) {

      visualization_msgs::Marker waypoint;
      waypoint.header.frame_id = "world_frame";
      waypoint.id              = id++;
      waypoint.type            = visualization_msgs::Marker::CUBE;
      waypoint.ns              = "markers";

      waypoint.scale.x = 0.1;
      waypoint.scale.y = 0.1;
      waypoint.scale.z = 0.1;

      waypoint.pose.position.x  = paths_[i][j][0];
      waypoint.pose.position.y  = paths_[i][j][1];
      waypoint.pose.position.z  = paths_[i][j][2];
      waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      waypoint.color.r = 0.0;
      waypoint.color.g = 1.0;
      waypoint.color.b = 0.0;
      waypoint.color.a = 1.0;

      msg_paths_out_.markers.push_back(waypoint);
    }

    if (paths_[i].size() > 0) {

      visualization_msgs::Marker waypoint;

      waypoint.header.frame_id = "world_frame";
      waypoint.id              = id++;
      waypoint.type            = visualization_msgs::Marker::LINE_STRIP;
      waypoint.ns              = "markers";

      waypoint.scale.x = 0.1;
      waypoint.scale.y = 0.1;
      waypoint.scale.z = 0.1;

      waypoint.color.r = 0.0;
      waypoint.color.g = 1.0;
      waypoint.color.b = 0.0;
      waypoint.color.a = 1.0;

      waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      for (size_t j = 0; j < paths_[i].size(); j++) {

        geometry_msgs::Point point;

        point.x = paths_[i][j][0];
        point.y = paths_[i][j][1];
        point.z = paths_[i][j][2];

        waypoint.points.push_back(point);
      }

      msg_paths_out_.markers.push_back(waypoint);
    }
  }

  if (collisions_.size() > 0) {

    visualization_msgs::Marker waypoint;

    waypoint.header.frame_id = "world_frame";
    waypoint.id              = id++;
    waypoint.type            = visualization_msgs::Marker::LINE_LIST;
    waypoint.ns              = "markers";

    waypoint.scale.x = 0.2;
    waypoint.scale.y = 0.2;
    waypoint.scale.z = 0.2;

    waypoint.color.r = 1.0;
    waypoint.color.g = 0.0;
    waypoint.color.b = 0.0;
    waypoint.color.a = 1.0;

    waypoint.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    for (size_t i = 0; i < collisions_.size(); i++) {

      geometry_msgs::Point from;
      geometry_msgs::Point to;

      from.x = collisions_[i].first[0];
      from.y = collisions_[i].first[1];
      from.z = collisions_[i].first[2];

      to.x = collisions_[i].second[0];
      to.y = collisions_[i].second[1];
      to.z = collisions_[i].second[2];

      waypoint.points.push_back(from);
      waypoint.points.push_back(to);
    }

    msg_paths_out_.markers.push_back(waypoint);
  }

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(1.0), &ReshapingDebug::timerMain, this);
}

//}

// | ---------------- action handlers callbacks --------------- |

/* reshapeFormation() //{ */

bool ReshapingDebug::reshapeFormation([[maybe_unused]] const std::vector<std::vector<Eigen::Vector3d>>& paths) {
  return false;
}

//}

/* setLeaderPosition() //{ */

bool ReshapingDebug::setLeaderPosition([[maybe_unused]] const Eigen::Vector3d& position) {
  return false;
}

//}

/* visualizeCube() //{ */

void ReshapingDebug::visualizeCube([[maybe_unused]] const task_02_formation::Position_t& position, [[maybe_unused]] const task_02_formation::Color_t& color,
                                   [[maybe_unused]] const double& size) {

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

  marker.header.frame_id = "world_frame";
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

  msg_cubes_out_.markers.push_back(marker);
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void ReshapingDebug::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  ROS_INFO_ONCE("[ReshapingDebug]: timerMain() spinning");

  if (ph_marker_paths_.getNumSubscribers() > 0) {

    ROS_INFO("[ReshapingDebug]: got subscribers, publishing");

    visualization_msgs::MarkerArray msg_empty;

    {
      visualization_msgs::Marker del;

      del.header.frame_id  = "world_frame";
      del.ns               = "markers";
      del.id               = 0;
      del.scale.x          = 1.0;
      del.scale.y          = 1.0;
      del.scale.z          = 1.0;
      del.color.a          = 1.0;
      del.type             = visualization_msgs::Marker::CUBE;
      del.pose.position.x  = 10000;
      del.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
      del.action           = visualization_msgs::Marker::DELETEALL;

      msg_empty.markers.push_back(del);
    }

    {
      visualization_msgs::Marker del;

      del.header.frame_id = "world_frame";
      del.ns              = "markers";
      del.id              = 1;
      del.scale.x         = 1.0;
      del.scale.y         = 1.0;
      del.scale.z         = 1.0;
      del.color.a         = 1.0;
      del.type            = visualization_msgs::Marker::LINE_LIST;
      del.action          = visualization_msgs::Marker::DELETEALL;

      del.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      msg_empty.markers.push_back(del);
    }

    {

      visualization_msgs::Marker del;

      del.id = 2;

      del.header.frame_id = "world_frame";
      del.ns              = "markers";
      del.scale.x         = 1.0;
      del.scale.y         = 1.0;
      del.scale.z         = 1.0;
      del.color.a         = 1.0;
      del.type            = visualization_msgs::Marker::LINE_STRIP;
      del.action          = visualization_msgs::Marker::DELETEALL;

      del.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

      msg_empty.markers.push_back(del);
    }

    ph_marker_paths_.publish(msg_empty);
    ph_marker_cubes_.publish(msg_empty);

    ros::Duration(0.5).sleep();

    ph_marker_paths_.publish(msg_paths_out_);
    ph_marker_cubes_.publish(msg_cubes_out_);

    ros::shutdown();
  }
}

//}

// | ------------------------ routines ------------------------ |

/* isVectorFinite() //{ */

bool ReshapingDebug::isVectorFinite(const Eigen::VectorXd& vector, const std::string name) {

  for (int i = 0; i < vector.size(); i++) {
    if (!std::isfinite(vector[i])) {
      ROS_ERROR("[ReshapingDebug]: NaN detected in \"%s[%d]\"!!!", name.c_str(), i);
      return false;
    }
  }

  return true;
}

//}

/* isMatrixFinite() //{ */

bool ReshapingDebug::isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name) {

  for (int i = 0; i < matrix.cols(); i++) {
    for (int j = 0; j < matrix.rows(); j++) {
      if (!std::isfinite(matrix(i, j))) {
        ROS_ERROR("[ReshapingDebug]: NaN detected in \"%s[%d, %d]\"!!!", name.c_str(), i, j);
        return false;
      }
    }
  }

  return true;
}

//}

/* samplePath() //{ */

std::vector<Eigen::Vector3d> ReshapingDebug::samplePath(const std::vector<Eigen::Vector3d>& path, const double& sample_distance = 0.1) {

  std::vector<Eigen::Vector3d> trajectory;

  for (size_t i = 0; i < path.size() - 1; i++) {

    Eigen::Vector3d v1 = path[i];
    Eigen::Vector3d v2 = path[i + 1];

    Eigen::Vector3d direction = v2 - v1;
    double          distance  = direction.norm();

    int n_samples;

    if (distance <= sample_distance) {
      n_samples = 1;
    } else {
      n_samples = floor(distance / sample_distance);
    }

    Eigen::Vector3d vec = direction / ((double)n_samples);

    for (int j = 0; j <= n_samples; j++) {

      Eigen::Vector3d point = v1 + ((double)j) * vec;

      trajectory.push_back(point);
    }
  }

  return trajectory;
}

//}

/* validatePaths() //{ */

bool ReshapingDebug::validatePaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<Eigen::Vector3d>& start_in,
                                   const std::vector<Eigen::Vector3d>& ends_in) {

  std::vector<Eigen::Vector3d> ends = ends_in;

  bool success = true;

  if (paths.size() != start_in.size()) {
    ROS_ERROR("[ReshapingDebug]: the resulting vector of paths has a wrong dimension (%d != %d)", int(paths.size()), int(start_in.size()));
    return false;
  }

  // for each path
  for (size_t i = 0; i < start_in.size(); i++) {

    if (paths[i].size() <= 0) {
      ROS_ERROR("[ReshapingDebug]: the #%dth path is empty", int(i));
      success = false;
      break;
    }

    // check if the start point matches
    if ((paths[i][0] - start_in[i]).norm() > 1e-2) {
      success = false;
      ROS_ERROR("[ReshapingDebug]: the path's first point does not match the required initial condition");
      break;
    }

    // check if the end point is on the list
    int idx = -1;
    for (size_t j = 0; j < ends.size(); j++) {
      if ((paths[i][paths[i].size() - 1] - ends[j]).norm() <= 1e-2) {
        idx = j;
        break;
      }
    }

    if (idx >= 0) {
      ends.erase(ends.begin() + idx);
    } else {
      ROS_ERROR("[ReshapingDebug]: the path's end point is not within the list of final positions");
      success = false;
      break;
    }
  }

  return success;
}

//}

/* trajectoriesSafe() //{ */

bool ReshapingDebug::trajectoriesSafe(const std::vector<Eigen::Vector3d>& traj1, const std::vector<Eigen::Vector3d>& traj2, const double& threshold) {

  bool safe = true;

  for (size_t i = 0; i < traj1.size(); i++) {

    Eigen::Vector3d p1 = traj1[i];

    for (size_t j = 0; j < traj2.size(); j++) {

      Eigen::Vector3d p2 = traj2[j];

      // check if we are not closed to any beginning or end of the trajectory
      if ((p1 - p2).norm() < threshold) {
        collisions_.push_back({p1, p2});
        safe = false;
      }
    }
  }

  return safe;
}

//}

}  // namespace task_02_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_02_evaluation::ReshapingDebug, nodelet::Nodelet)
