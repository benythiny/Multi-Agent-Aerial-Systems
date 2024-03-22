#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/mutex.h>

#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/batch_visualizer.h>

#include <visualization_msgs/MarkerArray.h>

//}

namespace task_03_wrapper
{

/* class Plotter //{ */

class Plotter : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  // params

  double _column_spacing_;
  int    _x_columns_;
  int    _y_columns_;

  std::string _obstacle_file_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<geometry_msgs::Pose> sh_target_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_rviz_marker_;
  mrs_lib::PublisherHandler<visualization_msgs::Marker>      ph_true_target_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | -------------------- timer environment ------------------- |

  ros::Timer timer_env_;
  void       timerEnv(const ros::TimerEvent &event);

  // | -------------------- batch vizualizer -------------------- |

  mrs_lib::BatchVisualizer bv_true_target_;
  std::mutex               mutex_bv_;
};

//}

/* onInit() //{ */

void Plotter::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "Plotter");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Plotter]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("main_rate", _main_timer_rate_);

  param_loader.loadParam("rviz/spacing", _column_spacing_);
  param_loader.loadParam("rviz/x_columns", _x_columns_);
  param_loader.loadParam("rviz/y_columns", _y_columns_);

  param_loader.loadParam("obstacle_file", _obstacle_file_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Plotter]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  ph_rviz_marker_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "rviz_markers_out", 1, true);
  ph_true_target_ = mrs_lib::PublisherHandler<visualization_msgs::Marker>(nh_, "true_target_out");

  // | -------------------- Batch vizualizer -------------------- |

  bv_true_target_ = mrs_lib::BatchVisualizer(nh_, "true_target_path_out", "common_origin");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Plotter";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_target_ = mrs_lib::SubscribeHandler<geometry_msgs::Pose>(shopts, "target_pose_in");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &Plotter::timerMain, this);
  timer_env_  = nh_.createTimer(ros::Rate(1.0), &Plotter::timerEnv, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Plotter]: initialized, version %s", VERSION);
}

//}

// | ------------------------- timers ------------------------- |

/* timerEnv() //{ */

void Plotter::timerEnv([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Plotter]: timerEnv() spinning");

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

  timer_env_.stop();
}

//}

/* timerMain() //{ */

void Plotter::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Plotter]: timerMain() spinning");

  if (!sh_target_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[Plotter]: waiting for target");
    return;
  }

  auto target = sh_target_.getMsg();

  // | ------------ publish rviz marker of the target ----------- |

  Eigen::Vector3d target_position;
  target_position << target->position.x, target->position.y, target->position.z;

  {
    std::scoped_lock lock(mutex_bv_);

    /* bv_true_target_.clearVisuals(); */
    /* bv_true_target_.clearBuffers(); */

    Eigen::Quaterniond        orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    Eigen::Vector3d           scale(1.0, 1.0, 1.0);
    mrs_lib::geometry::Cuboid cub(target_position, scale, orientation);

    bv_true_target_.addCuboid(cub, 1.0, 0.0, 0.0, 1.0, true);  // draw colored faces

    bv_true_target_.publish();
  }

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

    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.id = 0;

    ph_true_target_.publish(marker);
  }
}

//}

}  // namespace task_03_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_wrapper::Plotter, nodelet::Nodelet)
