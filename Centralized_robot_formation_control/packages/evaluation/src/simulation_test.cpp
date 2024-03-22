#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <task_02_wrapper/Diagnostics.h>

#include <stdio.h>

//}

namespace task_02_evaluation
{

/* class SimulationTest //{ */

class SimulationTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE*       _output_file_;

  // | ----------------------- evaluation ----------------------- |

  bool      under_distance_ = false;
  ros::Time udner_ditance_since_;

  // | ----------------------- parameters ----------------------- |

  double _requirement_distance_to_robot_;
  double _requirement_time_under_distance_;
  double _requirement_time_limit_;

  std::string _variant_;

  double _scoring_easy_;
  double _scoring_difficult_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<task_02_wrapper::Diagnostics> sh_diagnostics_;

  void callbackDiagnostics(mrs_lib::SubscribeHandler<task_02_wrapper::Diagnostics>& wrp);
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

  param_loader.loadParam("requirements/distance_to_robot", _requirement_distance_to_robot_);
  param_loader.loadParam("requirements/time_under_distance", _requirement_time_under_distance_);
  param_loader.loadParam("requirements/time_limit", _requirement_time_limit_);

  param_loader.loadParam("variant", _variant_);

  param_loader.loadParam("scoring/easy", _scoring_easy_);
  param_loader.loadParam("scoring/difficult", _scoring_difficult_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "SimulationTest";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_diagnostics_ = mrs_lib::SubscribeHandler<task_02_wrapper::Diagnostics>(shopts, "diagnostics_in", &SimulationTest::callbackDiagnostics, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[SimulationTest]: initialized, version %s", VERSION);
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackDiagnostics() //{ */

void SimulationTest::callbackDiagnostics(mrs_lib::SubscribeHandler<task_02_wrapper::Diagnostics>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[SimulationTest]: getting diagnostics");

  auto diag = wrp.getMsg();

  if (diag->failure) {

    ROS_ERROR("[SimulationTest]: detected failure of the formation, maybe one of the UAV crashed...");

    if (_output_to_file_) {

      _output_file_ = fopen(_file_path_.c_str(), "w+");
      fprintf(_output_file_, "0 %.2f %.2f Failure", 0.0, diag->total_duration);
      fclose(_output_file_);
    }

    ros::shutdown();
  }

  if (diag->total_duration > _requirement_time_limit_) {

    ROS_ERROR("[SimulationTest]: time limit exceeded");

    if (_output_to_file_) {
      _output_file_ = fopen(_file_path_.c_str(), "w+");
      fprintf(_output_file_, "0 %.2f %.2f Timeout", 0.0, diag->total_duration);
      fclose(_output_file_);
    }

    ros::shutdown();
  }

  if (diag->active) {

    bool under_distance = diag->distance_to_robot < _requirement_distance_to_robot_ ? true : false;

    ROS_INFO_THROTTLE(1.0, "[SimulationTest]: simulation is running, time since start = %.2f s", diag->total_duration);

    if (under_distance) {

      if (!under_distance_) {

        ROS_INFO("[SimulationTest]: the formation reached the robot!");
        udner_ditance_since_ = ros::Time::now();
        under_distance_      = true;

      } else {

        double time_under_distance = (ros::Time::now() - udner_ditance_since_).toSec();

        if (time_under_distance < _requirement_time_under_distance_) {
          ROS_INFO_THROTTLE(1.0, "[SimulationTest]: the formation is close to the robot for %.2f seconds already!", time_under_distance);
        } else {

          double score = _variant_ == "easy" ? _scoring_easy_ : _scoring_difficult_;

          ROS_INFO_THROTTLE(1.0, "[SimulationTest]: PASS (+ %.2f points), the formation has been close to the robot for more than %.2f seconds now", score,
                            time_under_distance);

          if (_output_to_file_) {
            _output_file_ = fopen(_file_path_.c_str(), "w+");
            fprintf(_output_file_, "1 %.2f %.2f Finished", score, diag->total_duration);
            fclose(_output_file_);
          }

          ros::shutdown();
        }
      }
    } else {

      if (under_distance_) {

        under_distance_ = false;

        ROS_WARN_THROTTLE(1.0, "[SimulationTest]: the formation got too far from the robot, restarting the timer");

      } else {

        ROS_INFO_THROTTLE(1.0, "[SimulationTest]: simulation is running, time since start = %.2f s, formation-robot distance = %.2f m", diag->total_duration,
                          diag->distance_to_robot);
      }
    }
  }
}

//}

}  // namespace task_02_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_02_evaluation::SimulationTest, nodelet::Nodelet)
