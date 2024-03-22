#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo/msgs/contacts.pb.h>

#include <task_03_wrapper/Diagnostics.h>

#include <std_srvs/SetBool.h>

//}

typedef const boost::shared_ptr<const gazebo::msgs::Contacts> ContactsPtr_t;

namespace task_03_wrapper
{

/* class CollisionChecker //{ */

class CollisionChecker : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ~CollisionChecker();

  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;
  std::atomic<bool> is_enabled_     = false;

  // | ------------------------- params ------------------------- |

  double                   _collision_duration_;
  std::string              _collisions_topic_;
  std::vector<std::string> _uavs_;

  // | ------------------------- gazebo ------------------------- |

  gazebo::transport::NodePtr gazebo_node_;

  // | ----------------------- subscribers ---------------------- |

  void                             callbackCollisions(ContactsPtr_t &msg);
  gazebo::transport::SubscriberPtr collisions_sub_;

  std::vector<mrs_lib::SubscribeHandler<task_03_wrapper::Diagnostics>> sh_diag_;

  // | --------------------- service clients -------------------- |

  std::vector<mrs_lib::ServiceClientHandler<std_srvs::SetBool>> sch_arm_;

  // | ----------------------- collisions ----------------------- |

  ros::Time  last_collision_time_;
  std::mutex mutex_last_collision_time_;

  ros::Time  in_collision_since_;
  std::mutex mutex_in_collision_since_;

  std::atomic<bool> in_collision_ = false;

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
};

//}

/* onInit() //{ */

void CollisionChecker::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  last_collision_time_ = ros::Time(0);

  int    argc = 0;
  char **argv = NULL;
  gazebo::client::setup(argc, argv);

  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  mrs_lib::ParamLoader param_loader(nh_, "CollisionChecker");

  param_loader.loadParam("collisions_topic", _collisions_topic_);
  param_loader.loadParam("collision_duration", _collision_duration_);
  param_loader.loadParam("uavs", _uavs_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[CollisionChecker]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  ROS_INFO("[CollisionChecker]: subscribing to gazebo topic: '%s'", _collisions_topic_.c_str());
  collisions_sub_ = gazebo_node_->Subscribe(_collisions_topic_, &CollisionChecker::callbackCollisions, this, 1);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "CollisionChecker";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  for (size_t i = 0; i < _uavs_.size(); i++) {
    sh_diag_.push_back(mrs_lib::SubscribeHandler<task_03_wrapper::Diagnostics>(shopts, "/" + _uavs_[i] + "/task_03_wrapper/diagnostics"));
  }

  // | --------------------- service clients -------------------- |

  for (size_t i = 0; i < _uavs_.size(); i++) {
    sch_arm_.push_back(mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "/" + _uavs_[i] + "/control_manager/arm"));
  }

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(100.0), &CollisionChecker::timerMain, this);

  is_initialized_ = true;

  ROS_INFO("[CollisionChecker]: initialized");
}

//}

/* ~CollisionChecker() //{ */

CollisionChecker::~CollisionChecker() {

  collisions_sub_->Unsubscribe();
};

//}

// | ------------------------ callbacks ----------------------- |

/* callbackCollisions() //{ */

void CollisionChecker::callbackCollisions(ContactsPtr_t &msg) {

  if (!is_initialized_) {
    return;
  }

  if (!is_enabled_) {
    return;
  }

  ROS_INFO_ONCE("[CollisionChecker]: getting collisions");

  if (msg->contact_size() > 0) {

    bool uav_in_collision_ = false;

    for (int i = 0; i < msg->contact_size(); i++) {
      if (msg->contact(i).collision1().substr(0, 3) == "uav") {
        uav_in_collision_ = true;
      }
    }

    if (!uav_in_collision_) {
      return;
    }

    ROS_INFO_THROTTLE(0.1, "[CollisionChecker]: collision detected");

    if (in_collision_) {

      if ((ros::Time::now() - in_collision_since_).toSec() >= _collision_duration_) {
        ROS_ERROR_THROTTLE(1.0, "[CollisionChecker]: in collision for too long");

        for (int i = 0; i < msg->contact_size(); i++) {
          for (size_t j = 0; j < _uavs_.size(); j++) {
            if (msg->contact(i).collision1().substr(0, 4) == _uavs_[j]) {

              std_srvs::SetBool srv;
              srv.request.data = false;

              sch_arm_[j].callAsync(srv);

              ROS_WARN_THROTTLE(1.0, "[CollisionChecker]: disarming %s", _uavs_[j].c_str());

              ros::shutdown();
            }
          }
        }
      }

    } else {

      in_collision_ = true;

      {
        std::scoped_lock lock(mutex_in_collision_since_);
        in_collision_since_ = ros::Time::now();
      }
    }

    {
      std::scoped_lock lock(mutex_last_collision_time_);
      last_collision_time_ = ros::Time::now();
    }
  }
}

//}

/* timerMain() //{ */

void CollisionChecker::timerMain(const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[CollisionChecker]: timerMain() spinning");

  if (!is_enabled_) {

    for (size_t i = 0; i < _uavs_.size(); i++) {
      if (!sh_diag_[i].hasMsg()) {
        ROS_INFO_ONCE("[CollisionChecker]: waiting for diagnostics'");
        return;
      }
    }

    for (size_t i = 0; i < _uavs_.size(); i++) {
      bool running = sh_diag_[i].getMsg()->active;
      if (!running) {
        ROS_INFO_ONCE("[CollisionChecker]: waiting for the uavs to be active");
        return;
      }
    }

    is_enabled_ = true;
  }

  auto last_collision_time = mrs_lib::get_mutexed(mutex_last_collision_time_, last_collision_time_);

  if (in_collision_) {
    if ((ros::Time::now() - last_collision_time).toSec() >= (_collision_duration_ + 0.1)) {
      in_collision_ = false;
    }
  }
}

//}

}  // namespace task_03_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_wrapper::CollisionChecker, nodelet::Nodelet)
