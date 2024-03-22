
/* includes //{ */

#include <ros/ros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/mutex.h>

#include <algorithm>
#include <random>
#include <sstream>

#include <task_03_boids/agent.h>

#include <task_03_wrapper/UserParams.h>

#include <visualization_msgs/MarkerArray.h>

//}

using AgentPtr = task_03_boids::AgentPtr;

namespace task_03_wrapper
{

/*//{ WrapperBoidsResult_t */
struct WrapperBoidsResult_t
{
  bool        success = false;
  std::string message = "";

  bool finished_in_time     = true;
  bool sequence_correct     = true;
  bool enough_survivors     = true;
  bool implementation_error = false;

  int    count_agents = 0;
  int    count_alive  = 0;
  int    count_dead   = 0;
  double ratio_alive  = 0.0;
  double ratio_dead   = 1.0;

  double time       = 0.0;
  int    iterations = 0;
};
/*//}*/

/* class WrapperBoids //{ */

class WrapperBoids {

private:
  /*//{ Beacon_t */
  typedef struct
  {
    int             id;
    Eigen::Vector3d position;
    Color_t         color;
    Color_t         color_pointer;
    Distribution    distribution;

    /*//{ toStr() */
    std::string toStr() const {
      std::ostringstream oss;
      oss << "[" + std::to_string(id) + "]:";
      /* oss << " value=" << std::to_string(directionToInt(value)) << ","; */
      oss << " xyz=(";
      oss << std::fixed << std::setprecision(1) << position.x() << ", ";
      oss << std::fixed << std::setprecision(1) << position.y() << ", ";
      oss << std::fixed << std::setprecision(1) << position.z() << "),";
      oss << " color=(";
      oss << color.red << ", ";
      oss << color.green << ", ";
      oss << color.blue << ")";
      oss << " color_pointer=(";
      oss << color_pointer.red << ", ";
      oss << color_pointer.green << ", ";
      oss << color_pointer.blue << ")";
      return oss.str();
    }
    /*//}*/

  } Beacon_t;
  /*//}*/

  /*//{ Options_t */
  typedef struct
  {
    // Static counts
    int classes_N;
    int agents_N;

    // Spawning agents
    Distribution spawn_distribution;
    double       spawn_prior_max_class_prob;
    double       spawn_lim_x_min;
    double       spawn_lim_x_max;
    double       spawn_lim_y_min;
    double       spawn_lim_y_max;

    // Agents properties
    double colorblind_ratio;

    // Neighborhood
    double neighborhood_radius;

    // Beacons
    double beacons_radius;
    double beacons_spawn_square_size;
    double beacons_emission_error;

    // Killing agents
    bool   killing_enabled;
    double kill_proximity_radius;
    double kill_proximity_prob;
    double kill_predator_dist_from_beacons;
    double kill_predator_prob;
    double kill_knn_radius;
    double kill_knn_prob;
    int    kill_knn_min_neighbors;

    // Constraints
    double velocity_min;
    double velocity_max;
    double heading_max_change;

    // Visualization
    std::string vis_frame;
    bool        vis_edges_show;
    /* bool        vis_show_texts; */
    double vis_agents_pos_size;
    double vis_agents_vel_size;
    double vis_agents_dead_size;
    double vis_edges_lw;
    double vis_predator_zone_lw;
    double vis_beacons_size_inner;
    double vis_beacons_alpha;
    double vis_arrow_shaft_diameter;
    double vis_arrow_head_diameter;

  } Options_t;
  /*//}*/

public:
  WrapperBoids(const ros::NodeHandle &nh, const bool verbose);
  WrapperBoidsResult_t runTask(const std::string &task_variant);

  // | ------------------------ VARIABLES ----------------------- |
private:
  ros::NodeHandle _nh_;
  std::string     _version_;
  bool            _initialized_ = false;
  bool            _verbose_;

  std::unique_ptr<mrs_lib::ParamLoader> _param_loader_;

  // Options
  Options_t _options_;

  // Timer
  ros::Timer _timer_iterate_;
  double     _timer_iterate_rate_;
  std::mutex _mutex_timer_running_;
  bool       _timer_running_ = false;

  // Random
  std::default_random_engine random_engine_;

  // Variables required for running one instance of the task
  int                   _iteration_       = 0;
  bool                  _killing_enabled_ = false;
  std::vector<Beacon_t> _beacons_;
  std::vector<AgentPtr> _agents_;
  std::vector<AgentPtr> _dead_agents_;

  // Predator zone
  std::tuple<double, double> _predator_zone_min_;
  std::tuple<double, double> _predator_zone_max_;

  // Beacons
  const Eigen::Vector3d _target_default_ = Eigen::Vector3d::Zero();
  Beacon_t              _target_beacon_;
  std::vector<Beacon_t> _expected_beacon_sequence_;
  std::vector<Beacon_t> _beacon_sequence_;

  // Task evaluation
  WrapperBoidsResult_t _task_result_;
  std::string          _task_variant_loaded_ = "";
  std::string          _task_fail_forced_msg_;
  std::mutex           _mutex_task_finished_;
  ros::Time            _task_init_time_;
  bool                 _task_initialized_ = false;
  bool                 _task_finished_    = false;
  bool                 _task_failed_      = false;
  bool                 _task_fail_forced_ = false;
  bool                 _stop_after_finish_;
  /* double               _task_timeout_; */
  double _dead_ratio_ = 0.0;
  double _max_dead_ratio_;
  int    _task_iterations_max_;

  // | ----------------------- subscribers ---------------------- |
  mrs_lib::SubscribeHandler<task_03_wrapper::UserParams> sh_params_;

  // | ----------------------- publishers ----------------------- |
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> _ph_vis_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> _ph_vis_students_;

  // | ---------------------- visualization --------------------- |
  visualization_msgs::Marker           _marker_predator_zone_;
  visualization_msgs::Marker           _marker_beacons_inner_;
  visualization_msgs::Marker           _marker_beacons_outer_;
  visualization_msgs::MarkerArray::Ptr _msg_vis_;
  visualization_msgs::MarkerArray::Ptr _msg_vis_students_;

  // | ------------------------- COLORS ------------------------- |
  const std::vector<Color_t> COLORS = {Color_t{1.0, 0.0, 0.0, 1.0}, Color_t{0.0, 0.6, 0.0, 1.0}, Color_t{0.0, 0.0, 1.0, 1.0}, Color_t{1.0, 0.5, 0.0, 1.0}};
  std_msgs::ColorRGBA        COLOR_BLACK;
  std_msgs::ColorRGBA        COLOR_GRAY;

  // | ------------------------- METHODS ------------------------ |
private:
  // | ---------------------- loading data ---------------------- |
  void resetVariables();
  bool loadCommonParams(std::string &err_msg);
  bool loadVariantSpecificParams(const std::string &task_variant, std::string &err_msg);
  bool loadOptions();

  // | --------------------- initialization --------------------- |
  void setupBeacons();
  void spawnAgents();

  // | ---------------------- visualization --------------------- |
  std::tuple<visualization_msgs::Marker, visualization_msgs::Marker, visualization_msgs::Marker, visualization_msgs::Marker> getVisAgents(
      const ros::Duration &lifetime, const ros::Time &stamp);
  void setupVisualization();
  void setupVisAgents(const ros::Duration &lifetime, const ros::Time &stamp);
  void setupVisEnvironment();
  void publishVisualization();

  // | ----------------------- main timer ----------------------- |
  void timerIterate(const ros::TimerEvent &event);

  // | ---------------- methods for task handling --------------- |
  void            killAgentsAndSetupNeighbors();
  Eigen::Vector3d selectTarget();
  void            updateAgentStates(const UserParams_t &user_params, const Eigen::Vector3d &target);
  void            moveAgents();
  bool            validateTask();

  // | ------------------------- methods ------------------------ |

  std::vector<int> findNeighborsIndices(const int agent_idx, const double radius, const std::vector<int> &blacklist = std::vector<int>());
  bool             binarySelect(const double p_true);
  int              nonUniformSelect(const Distribution &distribution);
  double           randd(const double from, const double to);
  double           randdnorm(const double from, const double to);
};

//}

}  // namespace task_03_wrapper
