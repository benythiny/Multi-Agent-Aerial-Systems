#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_msgs/SpawnerDiagnostics.h>
#include <mrs_msgs/String.h>

#include <random>

//}

namespace task_03_wrapper
{

/* class Spawner //{ */

class Spawner : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  int         _n_uavs_;
  std::string _uav_type_;
  std::string _args_;

  Eigen::MatrixXd _spawn_points_;

  // | --------------------- service client --------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::String> sch_spawn_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::SpawnerDiagnostics> sh_spawner_diag_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  // | ------------------------- methods ------------------------ |

  double randd(const double from, const double to);
};

//}

/* onInit() //{ */

void Spawner::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "Spawner");

  param_loader.loadParam("n_uavs", _n_uavs_);
  param_loader.loadParam("uav_type", _uav_type_);
  param_loader.loadParam("args", _args_);

  _spawn_points_ = param_loader.loadMatrixDynamic2("spawn_points", -1, 2);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Spawner]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Spawner";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_spawner_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::SpawnerDiagnostics>(shopts, "spawner_diag_in");

  // | --------------------- service clients -------------------- |

  sch_spawn_ = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "spawn_out");

  // | --------------------------- rng -------------------------- |

  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  /* srand(time(NULL)); */

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(10.0), &Spawner::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Spawner]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void Spawner::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Spawner]: timerMain() spinning");

  if (!sh_spawner_diag_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[Spawner]: waiting for spawner diagnostics");
    return;
  }

  ROS_INFO_ONCE("[Spawner]: got spawner diagnostics");

  std::vector<int> position_ids;

  for (int i = 0; i < _spawn_points_.rows(); i++) {
    position_ids.push_back(i);
  }

  unsigned                   seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine e(seed);

  std::shuffle(position_ids.begin(), position_ids.end(), e);

  for (int i = 0; i < _n_uavs_; i++) {

    int pos_idx = position_ids[i];

    std::stringstream ss;
    ss << i + 1 << " " << _uav_type_ << " " << _args_ << " --pos " << _spawn_points_(pos_idx, 0) << " " << _spawn_points_(pos_idx, 1) << " 0.5 0";
    ROS_INFO("[Spawner]: spawning: '%s'", ss.str().c_str());

    mrs_msgs::String cmd;

    cmd.request.value = ss.str();

    sch_spawn_.call(cmd);

    if (!cmd.response.success) {
      ROS_ERROR("[Spawner]: failed: '%s'", cmd.response.message.c_str());
    }
  }

  ROS_INFO("[Spawner]: stopping mainTimer()");

  timer_main_.stop();
}

//}

/* randd() //{ */

double Spawner::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

}  // namespace task_03_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_wrapper::Spawner, nodelet::Nodelet)
