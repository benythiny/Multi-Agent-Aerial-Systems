#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <task_03_wrapper/Diagnostics.h>

#include <stdio.h>
#include <numeric>

//}

namespace task_03_evaluation
{

/* class SimulationTest //{ */

class SimulationTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  int _n_uavs_;

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE *      _output_file_;

  // | ----------------------- evaluation ----------------------- |

  bool      all_under_distance_ = false;
  ros::Time all_under_distance_since_;

  // | ----------------------- parameters ----------------------- |

  double _requirement_distance_to_robot_;
  double _requirement_max_mutual_distance_;
  double _requirement_time_under_distance_;
  double _requirement_time_limit_;

  int _scoring_points_;
  /* double _bonus_points_per_time_; */
  /* double _bonus_points_points_; */

  double max_mutual_distance_ = 0;

  // | -------------------- evaluation timer -------------------- |

  ros::Timer timer_eval_;
  void       timerEval(const ros::TimerEvent &event);
  double     _eval_timer_rate_;


  // | ----------------------- subscribers ---------------------- |

  std::vector<mrs_lib::SubscribeHandler<task_03_wrapper::Diagnostics>> sh_diagnostics_;
};

//}

/* onInit() //{ */

void SimulationTest::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "SimulationTest");

  param_loader.loadParam("version", _version_);

  param_loader.loadParam("file", _file_path_);

  if (_file_path_ != "") {
    _output_to_file_ = true;
  }

  if (_version_ != VERSION) {

    ROS_ERROR("[SimulationTest]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("n_uavs", _n_uavs_);
  param_loader.loadParam("eval_timer_rate", _eval_timer_rate_);

  param_loader.loadParam("requirements/distance_to_robot", _requirement_distance_to_robot_);
  param_loader.loadParam("requirements/max_mutual_distance", _requirement_max_mutual_distance_);
  param_loader.loadParam("requirements/time_under_distance", _requirement_time_under_distance_);
  param_loader.loadParam("requirements/time_limit", _requirement_time_limit_);

  param_loader.loadParam("scoring/points", _scoring_points_);

  if (!param_loader.loadedSuccessfully()) {

    ROS_ERROR("[SimulationTest]: failed to load params");
    ros::shutdown();
  }

  /* param_loader.loadParam("bonus_points/per_time", _bonus_points_per_time_); */
  /* param_loader.loadParam("bonus_points/points", _bonus_points_points_); */

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "SimulationTest";
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

  timer_eval_ = nh_.createTimer(ros::Rate(_eval_timer_rate_), &SimulationTest::timerEval, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[SimulationTest]: initialized, version %s", VERSION);
}

//}

// | ------------------------ callbacks ----------------------- |

/* timerEval() //{ */

void SimulationTest::timerEval([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[SimulationTest]: timerEval() running");

  // | -------------------- retrieve all msgs ------------------- |
  std::vector<task_03_wrapper::Diagnostics::ConstPtr> msgs;

  for (auto &sh : sh_diagnostics_) {

    if (!sh.hasMsg()) {
      ROS_INFO_THROTTLE(1.0, "[SimulationTest]: waiting for diagnostics from each UAV");
      return;
    }

    msgs.push_back(sh.getMsg());
  }

  double min_duration = msgs.at(0)->total_duration;
  double max_distance = msgs.at(0)->distance_to_robot;

  // | ------------------- check for failures ------------------- |
  for (const auto &msg : msgs) {

    if (msg->failure) {

      ROS_ERROR("[SimulationTest]: detected failure in the swarm, maybe one of the UAV crashed...");

      if (_output_to_file_) {

        _output_file_ = fopen(_file_path_.c_str(), "w+");
        fprintf(_output_file_, "0 %d %.2f %.2f Failure", 0, min_duration, max_mutual_distance_);
        fclose(_output_file_);
      }

      ros::shutdown();
    }

    if (msg->total_duration < min_duration) {
      min_duration = msg->total_duration;
    }
    if (msg->distance_to_robot > max_distance) {
      max_distance = msg->distance_to_robot;
    }
  }


  // | ---------- check if duration have been exceeded ---------- |
  if (min_duration > _requirement_time_limit_) {

    ROS_ERROR("[SimulationTest]: time limit exceeded");

    if (_output_to_file_) {
      _output_file_ = fopen(_file_path_.c_str(), "w+");
      fprintf(_output_file_, "0 %d %.2f %.2f Timeout", 0, min_duration, max_mutual_distance_);
      fclose(_output_file_);
    }

    ros::shutdown();
  }

  // | ------------- check if all robots are active ------------- |
  const bool all_active = std::all_of(msgs.begin(), msgs.end(), [](const task_03_wrapper::Diagnostics::ConstPtr msg) { return msg->active; });

  if (all_active) {

    const bool all_under_distance = std::all_of(msgs.begin(), msgs.end(), [this](const task_03_wrapper::Diagnostics::ConstPtr msg) {
      return msg->distance_to_robot < _requirement_distance_to_robot_;
    });

    for (unsigned int i = 0; i < msgs.size(); i++) {
      for (unsigned int j = i + 1; j < msgs.size(); j++) {
        if (i != j) {

          const double dx      = msgs[i]->position.x - msgs[j]->position.x;
          const double dy      = msgs[i]->position.y - msgs[j]->position.y;
          const double dz      = msgs[i]->position.z - msgs[j]->position.z;
          max_mutual_distance_ = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
      }
    }

    if (max_mutual_distance_ > _requirement_max_mutual_distance_) {

      ROS_ERROR("[SimulationTest]: max mutual distance between UAVs have been exceeded (%.2f > %.2f)", max_mutual_distance_, _requirement_max_mutual_distance_);

      if (_output_to_file_) {
        _output_file_ = fopen(_file_path_.c_str(), "w+");
        fprintf(_output_file_, "0 %d %.2f %.2f MutualDistanceExceeded", 0, min_duration, max_mutual_distance_);
        fclose(_output_file_);
      }

      ros::shutdown();
    }

    ROS_INFO_THROTTLE(1.0, "[SimulationTest]: simulation is running, time since start = %.2f s", min_duration);

    if (all_under_distance) {

      if (!all_under_distance_) {

        ROS_INFO("[SimulationTest]: the swarm reached the robot!");
        all_under_distance_since_ = ros::Time::now();
        all_under_distance_       = true;

      } else {

        const double time_under_distance = (ros::Time::now() - all_under_distance_since_).toSec();

        if (time_under_distance < _requirement_time_under_distance_) {

          ROS_INFO_THROTTLE(1.0, "[SimulationTest]: the swarm is close to the robot for %.2f seconds already!", time_under_distance);

        } else {

          const int score = _scoring_points_;
          /* const double score = floor((_requirement_time_limit_ - min_duration) / _bonus_points_per_time_) * _bonus_points_points_; */

          ROS_INFO("[SimulationTest]: PASS (%d points), the swarm has been close to the robot for more than %.2f seconds now", score, time_under_distance);
          /* ROS_INFO_THROTTLE(1.0, "[SimulationTest]: PASS (+ %.2f points), the swarm has been close to the robot for more than %.2f seconds now", score, */
          /*                   time_under_distance); */

          if (_output_to_file_) {
            _output_file_ = fopen(_file_path_.c_str(), "w+");
            fprintf(_output_file_, "1 %d %.2f %.2f Finished", score, min_duration, max_mutual_distance_);
            fclose(_output_file_);
          }

          ros::shutdown();
        }
      }
    } else {

      if (all_under_distance_) {

        all_under_distance_ = false;

        ROS_WARN_THROTTLE(1.0, "[SimulationTest]: the swarm got too far from the robot, restarting the timer");

      } else {

        ROS_INFO_THROTTLE(1.0, "[SimulationTest]: simulation is running, time since start = %.2f s, max swarm-UGV distance = %.2f m", min_duration,
                          max_distance);
      }
    }
  } else {

    ROS_WARN_THROTTLE(1.0, "[SimulationTest]: waiting for messages from all the UAV wrappers");
  }
}

//}

}  // namespace task_03_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_evaluation::SimulationTest, nodelet::Nodelet)
