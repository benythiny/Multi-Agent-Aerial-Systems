#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/String.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/SpawnerDiagnostics.h>

#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/CameraInfo.h>

#include <topic_tools/shape_shifter.h>

//}

namespace task_03_wrapper
{

/* class Topic //{ */

class Topic {
private:
  std::string topic_name_;
  ros::Time   last_time_;

public:
  Topic(std::string topic_name) : topic_name_(topic_name) {
    last_time_ = ros::Time(0);
  }

  void updateTime(void) {
    last_time_ = ros::Time::now();
  }

  ros::Time getTime(void) {
    return last_time_;
  }

  std::string getTopicName(void) {
    return topic_name_;
  }
};

//}

/* class AutomaticStart //{ */

// state machine
typedef enum
{

  STATE_INIT,
  STATE_SPAWNING,
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_FINISHED

} LandingStates_t;

const char* state_names[5] = {

    "INIT", "SPAWNING", "IDLING", "TAKING OFF", "FINISHED"};

class AutomaticStart : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  std::string _uav_name_;
  bool        _simulation_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_offboard_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_validate_reference_;
  ros::ServiceClient service_client_spawn_;

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber subscriber_mavros_state_;
  ros::Subscriber subscriber_control_manager_diagnostics_;
  ros::Subscriber subscriber_spawner_diagnostics_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_can_takeoff_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
  double     _main_timer_rate_;

  // | ---------------------- mavros state ---------------------- |

  void              callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  std::atomic<bool> got_mavros_state_ = false;
  std::mutex        mutex_mavros_state_;

  // | ---------------------- mavros state ---------------------- |

  void                         callbackSpawnerDiagnostics(const mrs_msgs::SpawnerDiagnosticsConstPtr& msg);
  bool                         got_spawner_diagnostics = false;
  mrs_msgs::SpawnerDiagnostics spawner_diagnostics_;
  std::mutex                   mutex_spawner_diagnostics_;

  // | ----------------- arm and offboard check ----------------- |

  ros::Time armed_time_;
  bool      armed_ = false;

  ros::Time offboard_time_;
  bool      offboard_ = false;

  // | --------------- control manager diagnostics -------------- |

  void                                callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;

  // | ------------------------ routines ------------------------ |

  bool takeoff();

  bool validateReference();

  bool setMotors(const bool value);
  bool disarm();
  bool arm();
  bool setOffboard();

  // | ---------------------- other params ---------------------- |

  double _safety_timeout_;

  // | ---------------------- state machine --------------------- |

  uint current_state = STATE_INIT;
  void changeState(LandingStates_t new_state);

  // | ---------------- generic topic subscribers --------------- |

  bool                     _topic_check_enabled_ = false;
  double                   _topic_check_timeout_;
  std::vector<std::string> _topic_check_topic_names_;

  std::vector<Topic>           topic_check_topics_;
  std::vector<ros::Subscriber> generic_subscriber_vec_;

  // generic callback, for any topic, to monitor its rate
  void genericCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name, const int id);
};

//}

/* onInit() //{ */

void AutomaticStart::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed_      = false;
  armed_time_ = ros::Time(0);

  offboard_      = false;
  offboard_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStart");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[AutomaticStart]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("safety_timeout", _safety_timeout_);
  param_loader.loadParam("main_timer_rate", _main_timer_rate_);

  param_loader.loadParam("topic_check/enabled", _topic_check_enabled_);
  param_loader.loadParam("topic_check/timeout", _topic_check_timeout_);
  param_loader.loadParam("topic_check/topics", _topic_check_topic_names_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[AutomaticStart]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &AutomaticStart::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &AutomaticStart::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_spawner_diagnostics_ =
      nh_.subscribe("spawner_diagnostics_in", 1, &AutomaticStart::callbackSpawnerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_can_takeoff_ = nh_.advertise<std_msgs::Bool>("can_takeoff_out", 1);

  // | --------------------- service clients -------------------- |

  service_client_takeoff_  = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_motors_   = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_      = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");
  service_client_offboard_ = nh_.serviceClient<mavros_msgs::SetMode>("offboard_out");

  service_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_reference_out");

  // | ------------------ setup generic topics ------------------ |

  if (_topic_check_enabled_) {

    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;

    for (int i = 0; i < int(_topic_check_topic_names_.size()); i++) {

      std::string topic_name = _topic_check_topic_names_[i];

      if (topic_name.at(0) != '/') {
        topic_name = "/" + _uav_name_ + "/" + topic_name;
      }

      Topic tmp_topic(topic_name);
      topic_check_topics_.push_back(tmp_topic);

      int id = i;  // id to identify which topic called the generic callback

      callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { genericCallback(msg, topic_name, id); };
      ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 1, callback);

      generic_subscriber_vec_.push_back(tmp_subscriber);
    }
  }

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &AutomaticStart::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* genericCallback() //{ */

void AutomaticStart::genericCallback([[maybe_unused]] const topic_tools::ShapeShifter::ConstPtr& msg, [[maybe_unused]] const std::string& topic_name,
                                     const int id) {
  topic_check_topics_[id].updateTime();
}

//}

/* callbackMavrosState() //{ */

void AutomaticStart::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting mavros state");

  std::scoped_lock lock(mutex_mavros_state_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed > 0) {

      armed_      = true;
      armed_time_ = ros::Time::now();
    }

    // if we were armed_ previously
  } else if (armed_ == true) {

    // and we are not really now
    if (msg->armed == 0) {

      armed_ = false;
    }
  }

  // check offboard_ state
  if (offboard_ == false) {

    // if offboard_ state changed to true, please "start the clock"
    if (msg->mode == "OFFBOARD") {

      offboard_      = true;
      offboard_time_ = ros::Time::now();
    }

    // if we were in offboard_ previously
  } else if (offboard_ == true) {

    // and we are not really now
    if (msg->mode != "OFFBOARD") {

      offboard_ = false;
    }
  }

  got_mavros_state_ = true;
}

//}

/* callbackControlManagerDiagnostics() //{ */

void AutomaticStart::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting control manager diagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;

    got_control_manager_diagnostics_ = true;
  }
}

//}

/* callbackSpawnerDiagnostics() //{ */

void AutomaticStart::callbackSpawnerDiagnostics(const mrs_msgs::SpawnerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting spawner diagnostics");

  {
    std::scoped_lock lock(mutex_spawner_diagnostics_);

    spawner_diagnostics_ = *msg;

    got_spawner_diagnostics = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void AutomaticStart::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_control_manager_diagnostics_ || !got_mavros_state_) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStart]: waiting for data: ControManager=%s, Mavros=%s", got_control_manager_diagnostics_ ? "true" : "FALSE",
                      got_mavros_state_ ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  bool   motors           = control_manager_diagnostics.motors;
  double time_from_arming = (ros::Time::now() - armed_time).toSec();
  bool   position_valid   = validateReference();

  bool got_topics = true;

  std::stringstream missing_topics;

  if (_topic_check_enabled_) {

    for (int i = 0; i < int(topic_check_topics_.size()); i++) {
      if ((ros::Time::now() - topic_check_topics_[i].getTime()).toSec() > _topic_check_timeout_) {
        missing_topics << std::endl << "\t" << topic_check_topics_[i].getTopicName();
        got_topics = false;
      }
    }
  }

  std_msgs::Bool can_takeoff_msg;
  can_takeoff_msg.data = false;

  if (got_topics && position_valid && current_state == STATE_IDLE) {
    can_takeoff_msg.data = true;
  }

  try {
    publisher_can_takeoff_.publish(can_takeoff_msg);
  }
  catch (...) {
    ROS_ERROR("exception caught, could not publish");
  }

  if (!got_topics) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[AutomaticStart]: missing data on topics: " << missing_topics.str());
  }

  switch (current_state) {

    case STATE_INIT: {

      if (position_valid && got_topics) {

        std::scoped_lock lock(mutex_spawner_diagnostics_);

        bool res = arm();

        if (!res) {
          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not arm");
          return;
        }

        changeState(STATE_IDLE);
      }

      break;
    }

    case STATE_IDLE: {

      if (armed && !motors) {

        if (position_valid && got_topics) {

          bool res = setMotors(true);

          if (!res) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set motors ON");
          }

          ros::Duration(0.1).sleep();

          res = setOffboard();

          if (!res) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set offboard ON");
          }
        }

        if (time_from_arming > 1.5) {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set motors ON for 1.5 secs, disarming");
          disarm();
        }
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && motors) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          changeState(STATE_TAKEOFF);

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "starting in %.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case STATE_TAKEOFF: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      // if takeoff finished
      if (control_manager_diagnostics.active_tracker != "NullTracker" && control_manager_diagnostics.active_tracker != "LandoffTracker" &&
          !control_manager_diagnostics.tracker_status.have_goal) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: takeoff finished");

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_FINISHED: {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: we are done here");

      ros::shutdown();

      break;
    }
  }

}  // namespace automatic_start

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStart::changeState(LandingStates_t new_state) {

  ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TAKEOFF: {

      bool res = takeoff();

      if (!res) {
        return;
      }

      break;
    }

    default: {
      break;
    }
  }

  current_state = new_state;
}

//}

/* takeoff() //{ */

bool AutomaticStart::takeoff() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: taking off");

  std_srvs::Trigger srv;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: taking off failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for taking off failed");
  }

  return false;
}

//}

/* validateReference() //{ */

bool AutomaticStart::validateReference() {

  mrs_msgs::ValidateReference srv_out;

  srv_out.request.reference.header.frame_id = "fcu";

  bool res = service_client_validate_reference_.call(srv_out);

  if (res) {

    if (srv_out.response.success) {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: current position is valid");
      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current position is not valid (safety area, bumper)!");
      return false;
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current position could not be validated");
    return false;
  }
}

//}

/* motors() //{ */

bool AutomaticStart::setMotors(const bool value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: setting motors %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = service_client_motors_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: setting motors failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for setting motors failed");
  }

  return false;
}

//}

/* disarm() //{ */

bool AutomaticStart::disarm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot disarm, missing mavros state!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  if (offboard) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot disarm, already in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: disarming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 0;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: disarming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for disarming failed");
  }

  return false;
}

//}

/* arm() //{ */

bool AutomaticStart::arm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot arm, missing mavros state!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: arming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 1;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: arming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for arming failed");
  }

  return false;
}

//}

/* offboard() //{ */

bool AutomaticStart::setOffboard() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: switching to offboard");

  mavros_msgs::SetMode srv;
  srv.request.base_mode   = 0;
  srv.request.custom_mode = "offboard";

  bool res = service_client_offboard_.call(srv);

  if (!res) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for oddboard failed");
  }

  return false;
}

//}

}  // namespace task_03_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_wrapper::AutomaticStart, nodelet::Nodelet)
