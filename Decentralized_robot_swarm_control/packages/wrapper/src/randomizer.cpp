#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <student_headers/swarm.h>

#include <random>

#include <task_03_wrapper/CorruptMeasurements.h>
#include <task_03_wrapper/Diagnostics.h>

//}

/* defines //{ */

#define TAU 2 * M_PI

//}

namespace task_03_wrapper
{

/* class Randomizer //{ */

class Randomizer : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  int    _n_uavs_;
  double _main_timer_rate_;
  double _change_period_from_;
  double _change_period_to_;

  ros::Time _time_last_change_;
  double    _secs_to_next_change_;

  bool formation_static = false;
  bool first_iteration  = true;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<task_03_wrapper::CorruptMeasurements> ph_corrupt_;

  task_03_wrapper::CorruptMeasurements cm_msg;

  // | ----------------------- subscribers ---------------------- |

  std::vector<mrs_lib::SubscribeHandler<task_03_wrapper::Diagnostics>> sh_diagnostics_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  // | --------------------------- rng -------------------------- |

  std::default_random_engine random_engine_;
  double                     randd(const double from, const double to);
  double                     randdnorm(const double mean, const double stdev);
  int                        randi(const int from, const int to);
};

//}

/* onInit() //{ */

void Randomizer::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "Randomizer");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Randomizer]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("n_uavs", _n_uavs_);
  param_loader.loadParam("randomizer/timer_rate", _main_timer_rate_);
  param_loader.loadParam("randomizer/change_period/from", _change_period_from_);
  param_loader.loadParam("randomizer/change_period/to", _change_period_to_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Randomizer]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  ph_corrupt_ = mrs_lib::PublisherHandler<task_03_wrapper::CorruptMeasurements>(nh_, "corrupt_measurements_out", 1);

  // | --------------------------- rng -------------------------- |

  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  srand(time(NULL));

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Randomizer";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  for (int i = 0; i < _n_uavs_; i++) {

    int uav_id = i + 1;

    std::stringstream ss;
    ss << "/uav" << uav_id << "/task_03_wrapper/diagnostics";
    sh_diagnostics_.push_back(mrs_lib::SubscribeHandler<task_03_wrapper::Diagnostics>(shopts, ss.str()));
  }

  // | ------------------------- timers ------------------------- |

  _time_last_change_    = ros::Time::now();
  _secs_to_next_change_ = randd(_change_period_from_, _change_period_to_);
  cm_msg.uav_id         = randi(0, _n_uavs_);
  cm_msg.distribution   = task_03_wrapper::CorruptMeasurements::NONE;

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &Randomizer::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Randomizer]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void Randomizer::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  bool formation_is_now_static = true;

  for (auto &sh : sh_diagnostics_) {

    if (!sh.hasMsg()) {
      ROS_INFO_THROTTLE(1.0, "[Randomizer]: waiting for diagnostics from each UAV");
      return;
    }

    formation_is_now_static = formation_is_now_static && !sh.getMsg()->moving;
  }

  ROS_INFO_ONCE("[Randomizer]: timerMain() spinning");

  bool randomize_msg = false;

  if (first_iteration) {

    ROS_INFO("[Randomizer]: First iteration -> randomizing.");

    randomize_msg   = true;
    first_iteration = false;

  } else {

    // If the formation is moving
    if (!formation_is_now_static) {

      formation_static = false;

      // If the formation is now static and was not static in the previous step
    } else if (!formation_static) {

      ROS_INFO("[Randomizer]: Formation has stopped -> randomizing.");

      randomize_msg    = true;
      formation_static = true;
    }
  }

  const ros::Time now = ros::Time::now();
  if (randomize_msg || (now - _time_last_change_).toSec() > _secs_to_next_change_) {
    _time_last_change_    = now;
    _secs_to_next_change_ = randd(_change_period_from_, _change_period_to_);

    if (!randomize_msg) {
      ROS_INFO("[Randomizer]: Have not randomized for quite a long period -> randomizing.");
      randomize_msg = true;
    }
  }

  if (randomize_msg) {

    // Randomize UAV and error distribution
    cm_msg.uav_id       = randi(0, _n_uavs_);
    cm_msg.distribution = randi(0, task_03_wrapper::CorruptMeasurements::COUNT);

    switch (cm_msg.distribution) {
      case task_03_wrapper::CorruptMeasurements::NONE: {
        break;
      }
      case task_03_wrapper::CorruptMeasurements::STATIC: {
        const double offset = randd(-M_PI, M_PI);
        cm_msg.parameters   = {offset};
        break;
      }
      case task_03_wrapper::CorruptMeasurements::UNIFORM: {
        const double from = randd(-M_PI, M_PI);
        const double to   = randd(-M_PI, M_PI);
        if (from < to) {
          cm_msg.parameters = {from, to};
        } else {
          cm_msg.parameters = {to, from};
        }
        break;
      }
      case task_03_wrapper::CorruptMeasurements::NORMAL: {
        const double mean  = randd(-M_PI, M_PI);
        const double stdev = randd(0, M_PI);
        cm_msg.parameters  = {mean, stdev};
        break;
      }
      default: {
        break;
      }
    }
  }

  ph_corrupt_.publish(cm_msg);
}

//}

/* randd() //{ */

double Randomizer::randd(const double from, const double to) {

  const double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int Randomizer::randi(const int from, const int to) {

  const double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* randdnorm() //{ */

double Randomizer::randdnorm(const double mean, const double stdev) {

  std::normal_distribution<double> distribution(mean, stdev);
  return distribution(random_engine_);
}

//}


// --------------------------------------------------------------
// |                           methods                          |
// --------------------------------------------------------------
}  // namespace task_03_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_wrapper::Randomizer, nodelet::Nodelet)
