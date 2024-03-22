#define VERSION "1.0.0"

#include <task_03_wrapper/wrapper_boids.h>

namespace task_03_wrapper
{

/* WrapperBoids() //{ */
WrapperBoids::WrapperBoids(const ros::NodeHandle &nh, const bool verbose) {

  _verbose_      = verbose;
  _nh_           = nh;
  _param_loader_ = std::make_unique<mrs_lib::ParamLoader>(_nh_, "WrapperBoids");

  COLOR_BLACK.r = 0.0;
  COLOR_BLACK.g = 0.0;
  COLOR_BLACK.b = 0.0;
  COLOR_BLACK.a = 1.0;
  COLOR_GRAY.r  = 0.75;
  COLOR_GRAY.g  = 0.75;
  COLOR_GRAY.b  = 0.75;
  COLOR_GRAY.a  = 1.0;

  if (!loadCommonParams(_task_result_.message)) {
    ros::shutdown();
    return;
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = _nh_;
  shopts.node_name          = "WrapperBoids";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_params_ = mrs_lib::SubscribeHandler<task_03_wrapper::UserParams>(shopts, "params_in");

  // | ----------------------- publishers ----------------------- |

  _ph_vis_          = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(_nh_, "visualization_out");
  _ph_vis_students_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(_nh_, "visualization_custom_out");

  _timer_iterate_ = _nh_.createTimer(ros::Rate(_timer_iterate_rate_), &WrapperBoids::timerIterate, this, false, false);

  _initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[WrapperBoids]: initialized, version %s", VERSION);
}

//}

// | --------------------- PUBLIC METHODS --------------------- |

/* runTask() //{ */
WrapperBoidsResult_t WrapperBoids::runTask(const std::string &task_variant) {

  if (!_initialized_) {
    _task_result_.success = false;
    _task_result_.message = "WrapperBoids is uninitialized.";
    return _task_result_;
  }

  // Reset the previous task
  resetVariables();

  // Load parameters specific for the given task
  _task_result_.success = loadVariantSpecificParams(task_variant, _task_result_.message);
  if (!_task_result_.success) {
    return _task_result_;
  }

  // | ----------------- Setup world and agents ----------------- |
  setupBeacons();
  spawnAgents();

  // | -------------- First-iteration visualization ------------- |
  setupVisEnvironment();
  publishVisualization();

  // Start new timer
  _timer_iterate_.start();

  // Wait till the task finishes
  while (ros::ok() && !mrs_lib::get_mutexed(_mutex_task_finished_, _task_finished_)) {
    ros::Duration(0.1).sleep();
  }

  return _task_result_;
}
//}

// | --------------------- PRIVATE METHODS -------------------- |

/* loadCommonParams() //{ */
bool WrapperBoids::loadCommonParams(std::string &err_msg) {

  _param_loader_->loadParam("version", _version_);

  if (_version_ != VERSION) {
    std::stringstream ss;
    ss << "[WrapperBoids]: the version of the binary (" << VERSION << ") does not match the config file (" << _version_ << "), please build me";
    err_msg = ss.str();
    ros::shutdown();
    return false;
  }

  _param_loader_->loadParam("rate", _timer_iterate_rate_);
  _param_loader_->loadParam("stop_after_finish", _stop_after_finish_);
  _param_loader_->loadParam("validation/max_dead_ratio", _max_dead_ratio_);
  _param_loader_->loadParam("validation/max_iterations", _task_iterations_max_);
  /* _task_iterations_max_ = _task_timeout_ * _timer_iterate_rate_; */

  const bool loading_succ = loadOptions();

  if (!_param_loader_->loadedSuccessfully()) {

    err_msg = "[WrapperBoids]: Could not load all parameters!";
    ROS_ERROR_COND(_verbose_, "%s", err_msg.c_str());
    ros::shutdown();
    return false;

  } else if (!loading_succ) {

    err_msg = "[WrapperBoids]: Some of the loaded parameters were incorrectly set!";
    ROS_ERROR_COND(_verbose_, "%s", err_msg.c_str());
    ros::shutdown();
    return false;
  }

  return true;
}
//}

/* loadVariantSpecificParams() //{ */
bool WrapperBoids::loadVariantSpecificParams(const std::string &task_variant, std::string &err_msg) {

  if (task_variant.empty() || task_variant == "testing" || task_variant == _task_variant_loaded_) {

    return true;

  } else if (task_variant != "easy" && task_variant != "medium" && task_variant != "difficult") {

    std::stringstream ss;
    ss << "[WrapperBoids]: Unknown task variant:" << task_variant << ". Allowed variants: easy, medium, difficult.";
    err_msg = ss.str();
    ros::shutdown();
    return false;
  }


  // Agents
  _param_loader_->loadParam("tasks/" + task_variant + "/agents/count", _options_.agents_N);

  if (_options_.agents_N <= 0) {
    err_msg = "[WrapperBoids]: Incorrect number of agents set in task-specific config file.";
    ros::shutdown();
    return false;
  }

  // Predator
  _param_loader_->loadParam("tasks/" + task_variant + "/agents/kill/predator/distance_from_beacons", _options_.kill_predator_dist_from_beacons);
  const double p_x_max = _options_.beacons_spawn_square_size + _options_.kill_predator_dist_from_beacons;
  const double p_x_min = -p_x_max;
  const double p_y_max = _options_.beacons_spawn_square_size + _options_.kill_predator_dist_from_beacons;
  const double p_y_min = -p_y_max;
  _predator_zone_min_  = std::make_tuple(p_x_min, p_y_min);
  _predator_zone_max_  = std::make_tuple(p_x_max, p_y_max);

  // Beacons
  _param_loader_->loadParam("tasks/" + task_variant + "/beacons/spawn_distance", _options_.beacons_spawn_square_size);
  if (_options_.beacons_spawn_square_size <= 0.0) {
    err_msg = "[WrapperBoids]: Beacons spawn square size is less than 0 in task-specific config file.";
    ros::shutdown();
    return false;
  }

  if (!_param_loader_->loadedSuccessfully()) {

    err_msg = "[WrapperBoids]: Could not load all parameters!";
    ROS_ERROR_COND(_verbose_, "%s", err_msg.c_str());
    ros::shutdown();
    return false;
  }

  _task_variant_loaded_ = task_variant;
  return true;
}
//}

/* resetVariables() //{ */
void WrapperBoids::resetVariables() {

  // Stop possibly running timer from the previous run
  _timer_iterate_.stop();
  while (ros::ok() && mrs_lib::get_mutexed(_mutex_timer_running_, _timer_running_)) {
    ros::Duration(0.01).sleep();
  }

  _task_result_ = WrapperBoidsResult_t();

  _iteration_ = 0;

  _beacons_.clear();
  _agents_.clear();
  _dead_agents_.clear();

  _killing_enabled_ = false;

  // Beacons
  _expected_beacon_sequence_.clear();
  _beacon_sequence_.clear();

  // Task evaluation
  _task_initialized_ = false;
  mrs_lib::set_mutexed(_mutex_task_finished_, false, _task_finished_);
  _task_failed_          = false;
  _task_fail_forced_     = false;
  _dead_ratio_           = 0.0;
  _task_fail_forced_msg_ = "";

  // | --------------------------- rng -------------------------- |
  random_engine_ = std::default_random_engine(ros::Time::now().nsec);

  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  srand(time(NULL));
}
//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerIterate() //{ */

void WrapperBoids::timerIterate([[maybe_unused]] const ros::TimerEvent &event) {

  if (!sh_params_.hasMsg()) {
    if (_verbose_) {
      ROS_WARN_THROTTLE(1.0, "[WrapperBoids]: waiting for params");
    }
    return;
  }

  mrs_lib::set_mutexed(_mutex_timer_running_, true, _timer_running_);

  if (!_task_initialized_) {
    ROS_INFO_COND(_verbose_, "[WrapperBoids]: timerIterate() spinning");
    _task_init_time_   = ros::Time::now();
    _task_initialized_ = true;
  }

  // | ----------------------- user params ---------------------- |
  UserParams_t user_params;
  auto         params = sh_params_.getMsg();
  user_params.param1  = params->param1;
  user_params.param2  = params->param2;
  user_params.param3  = params->param3;
  user_params.param4  = params->param4;
  user_params.param5  = params->param5;
  user_params.param6  = params->param6;
  user_params.param7  = params->param7;
  user_params.param8  = params->param8;
  user_params.param9  = params->param9;

  killAgentsAndSetupNeighbors();
  const auto target = selectTarget();
  updateAgentStates(user_params, target);
  moveAgents();

  // Clear visualization msg and fill it with obstacles
  publishVisualization();

  const bool print = validateTask();
  if (print) {

    const double dur = (ros::Time::now() - _task_init_time_).toSec();

    const double alive_ratio = double(_agents_.size()) / double(_options_.agents_N);
    const double dead_ratio  = double(_dead_agents_.size()) / double(_options_.agents_N);

    /* ROS_INFO("[WrapperBoids] TASK FINISHED"); */
    /* ROS_INFO("   - time:             %.1f s", dur); */
    /* ROS_INFO("   - surviving agents: %ld/%d (%d %s)", _agents_.size(), _options_.agents_N, int(100.0 * alive_ratio), "%"); */
    /* ROS_INFO("   - dead agents:      %ld/%d (%d %s)", _dead_agents_.size(), _options_.agents_N, int(100.0 * dead_ratio), "%"); */

    _task_result_.time = dur;

    _task_result_.count_agents = _options_.agents_N;
    _task_result_.count_alive  = _agents_.size();
    _task_result_.count_dead   = _dead_agents_.size();

    _task_result_.ratio_alive = alive_ratio;
    _task_result_.ratio_dead  = dead_ratio;

    _task_result_.iterations = _iteration_;

    if (_stop_after_finish_) {
      _timer_iterate_.stop();
      mrs_lib::set_mutexed(_mutex_timer_running_, false, _timer_running_);
      return;
    }
  }

  _iteration_++;

  mrs_lib::set_mutexed(_mutex_timer_running_, false, _timer_running_);
}

//}

// | ---------------------- methods: task --------------------- |

/*//{ killAgentsAndSetupNeighbors() */
void WrapperBoids::killAgentsAndSetupNeighbors() {

  const std::vector<double> rads        = {_options_.neighborhood_radius, _options_.kill_proximity_radius, _options_.kill_knn_radius};
  auto                      rads_max_it = std::max_element(rads.begin(), rads.end());
  const int                 rad_max_idx = std::distance(rads.begin(), rads_max_it);
  const double              rad_max_val = *rads_max_it;

  const auto &[p_x_min, p_y_min] = _predator_zone_min_;
  const auto &[p_x_max, p_y_max] = _predator_zone_max_;

  const bool killing_enabled = _killing_enabled_ && _options_.killing_enabled;

  std::vector<int> killed_indices;
  int              killed_color_sensitive_agents = 0;

  /* std::vector<int>      mapping; */
  std::vector<int>      mapping(_agents_.size(), -1);
  std::vector<AgentPtr> survivors;

  survivors.reserve(_agents_.size());

  // Decide if keep agent alive or kill it
  for (int i = 0; i < int(_agents_.size()); i++) {

    const auto &agent  = _agents_[i];
    const auto &a_pos  = agent->position;
    bool        killed = false;

    std::vector<int> neighbors_all;
    std::vector<int> neighbors_alive;

    // In predator zone?
    if (killing_enabled && (a_pos.x() < p_x_min || a_pos.x() > p_x_max || a_pos.y() < p_y_min || a_pos.y() > p_y_max)) {
      killed = binarySelect(_options_.kill_predator_prob);
      ROS_WARN_COND(_verbose_ && killed, "[WrapperBoids]: Killing agent %d (in predator zone)", agent->id);
    }

    if (!killed) {

      // Find all neighbors
      neighbors_all = findNeighborsIndices(i, rad_max_val);

      // Find all non-killed so far
      neighbors_alive = neighbors_all;
      neighbors_alive.erase(std::remove_if(std::begin(neighbors_alive), std::end(neighbors_alive),
                                           [&](const int x) { return std::binary_search(std::begin(killed_indices), std::end(killed_indices), x); }),
                            std::end(neighbors_alive));
    }

    // Have another agent too close?
    if (killing_enabled && !killed) {

      const bool skip_norm_comparison = rad_max_idx == 1;

      for (const int n : neighbors_alive) {

        // Kill if close and random selection decides so
        if ((skip_norm_comparison || (a_pos - _agents_[n]->position).norm() < _options_.kill_proximity_radius) && binarySelect(_options_.kill_proximity_prob)) {
          killed = true;
          ROS_WARN_COND(_verbose_, "[WrapperBoids]: Killing agent %d (proximity to %d | %.2f m < %.2f m)", agent->id, _agents_[n]->id,
                        (a_pos - _agents_[n]->position).norm(), _options_.kill_proximity_radius);
          break;
        }
      }
    }

    // Is there enough neighbors?
    if (killing_enabled && !killed) {

      int knn_count;

      // Count neighbors
      if (rad_max_idx == 2) {

        knn_count = neighbors_all.size();

      } else {

        knn_count = 0;
        for (const int n : neighbors_all) {

          if ((a_pos - _agents_[n]->position).norm() < _options_.kill_knn_radius) {
            knn_count++;

            if (knn_count >= _options_.kill_knn_min_neighbors) {
              break;
            }
          }
        }
      }
      // Kill if not enough agents in given neighborhood and random selection decides so
      killed = knn_count < _options_.kill_knn_min_neighbors && binarySelect(_options_.kill_knn_prob);

      ROS_WARN_COND(_verbose_ && killed, "[WrapperBoids]: Killing agent: %d (not enough neighbors | %d < %d)", agent->id, knn_count,
                    _options_.kill_knn_min_neighbors);
    }

    //
    if (killed) {

      /* ROS_ERROR("[%ld] Adding to dead agents.", _iteration_); */

      if (!agent->colorblind) {
        killed_color_sensitive_agents++;
      }
      killed_indices.push_back(i);
      _dead_agents_.push_back(agent);

    } else {

      // Store neighbors
      if (rad_max_idx == 0) {
        agent->neighbors = neighbors_alive;
      } else {
        agent->neighbors.clear();
        for (const int n : neighbors_alive) {
          if ((a_pos - _agents_[n]->position).norm() < _options_.neighborhood_radius) {
            agent->neighbors.push_back(n);
          }
        }
      }

      mapping[i] = survivors.size();
      survivors.push_back(agent);
    }
  }

  /* ROS_ERROR("mapping i->m[i]"); */
  /* for (unsigned int i = 0; i < mapping.size(); i++) { */
  /*   ROS_ERROR(" %d->%d", i, mapping[i]); */
  /* } */

  /* for (unsigned int i = 0; i < survivors.size(); i++) { */
  /*   ROS_ERROR("neighbors of %i:", i); */

  /*   const auto &survivor = survivors[i]; */
  /*   for (unsigned int j = 0; j < survivor->neighbors.size(); j++) { */
  /*     ROS_ERROR(" %d->%d", survivor->neighbors[j], mapping[survivor->neighbors[j]]); */
  /*   } */
  /* } */


  for (unsigned int i = 0; i < survivors.size(); i++) {

    const auto &survivor = survivors[i];

    // Reindex neighbors
    for (auto &n : survivor->neighbors) {
      n = mapping[n];
    }
    survivor->neighbors.erase(std::remove_if(std::begin(survivor->neighbors), std::end(survivor->neighbors), [&](const int n) { return n == -1; }),
                              std::end(survivor->neighbors));

    // Keep certain amount of color-sensitive agents
    if (killed_color_sensitive_agents > 0 && survivor->colorblind) {
      survivor->colorblind = false;
      killed_color_sensitive_agents--;
    }
  }

  _agents_ = survivors;
}
/*//}*/

/*//{ selectTarget() */
Eigen::Vector3d WrapperBoids::selectTarget() {

  const bool beacon_selected = !_beacon_sequence_.empty();

  // Check if consensus has been reached to any value
  const Direction_t common_value      = _agents_[0]->getDirection();
  bool              consensus_reached = true;
  for (unsigned int i = 1; i < _agents_.size(); i++) {
    const auto &a_direction = _agents_[i]->getDirection();

    if (a_direction != common_value) {
      consensus_reached = false;
      break;
    }
  }

  if (!consensus_reached) {

    // If not and no beacon is being directed to
    if (!beacon_selected) {
      return _target_default_;
    }

    // If not and a beacon is being directed to
    return _target_beacon_.position;
  }

  // Select next target beacon (there always is one to match the condition)
  const int common_color_idx = directionToInt(common_value);
  Color_t   common_color     = COLORS[common_color_idx];
  for (const auto &beacon : _beacons_) {
    if (beacon.color == common_color) {
      _target_beacon_ = beacon;
      break;
    }
  }

  /* ROS_INFO("[WrapperBoids]: Consensus reached to value: %d", common_color_idx); */

  // Already had some beacon selected
  if (beacon_selected) {

    if (common_color != _beacon_sequence_.back().color) {
      ROS_INFO_COND(_verbose_, "[WrapperBoids]: Setting next target beacon to: %s", _target_beacon_.toStr().c_str());
      _beacon_sequence_.push_back(_target_beacon_);
    }

  } else {

    _expected_beacon_sequence_ = {_target_beacon_};

    Color_t color_ptr = _target_beacon_.color_pointer;
    for (unsigned int i = 0; i < _beacons_.size(); i++) {

      auto           it          = std::find_if(_beacons_.begin(), _beacons_.end(), [&color_ptr](const Beacon_t &b) { return b.color == color_ptr; });
      const Beacon_t next_beacon = *it;
      color_ptr                  = next_beacon.color_pointer;

      _expected_beacon_sequence_.push_back(next_beacon);
    }

    ROS_INFO_COND(_verbose_, "[WrapperBoids]: Setting first target beacon to: %s", _target_beacon_.toStr().c_str());

    _beacon_sequence_ = {_target_beacon_};
    _killing_enabled_ = true;
  }

  return _target_beacon_.position;
}
/*//}*/

/*//{ updateAgentStates() */
void WrapperBoids::updateAgentStates(const UserParams_t &user_params, const Eigen::Vector3d &target) {

  // Containers for storing the results
  std::vector<Eigen::Vector3d> actions;
  std::vector<Distribution>    distributions;
  actions.resize(_agents_.size());
  distributions.resize(_agents_.size());

  for (unsigned int i = 0; i < _agents_.size(); i++) {
    const auto &agent     = _agents_.at(i);
    const auto &a_pos     = agent->position;
    const auto &neighbors = agent->neighbors;

    const Eigen::Vector3d target_in_agent_frame = saturateVector(target - a_pos, 1.0);

    // Setup neighbors
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Distribution>> neighbors_states;
    neighbors_states.resize(neighbors.size());

    for (unsigned int i = 0; i < neighbors.size(); i++) {
      const auto &neighbor           = _agents_[neighbors[i]];
      const auto &pos_in_agent_frame = neighbor->position - a_pos;
      const auto &vel_global         = neighbor->velocity;
      const auto &distribution       = neighbor->distribution;

      neighbors_states[i] = std::make_tuple(pos_in_agent_frame, vel_global, distribution);
    }

    bool         nearby_beacon = false;
    Distribution beacon_distribution(_beacons_[0].distribution.dim());
    /* ROS_ERROR("Agent [%d] %s colorblind", agent->id, agent->colorblind ? "is" : "is NOT"); */
    if (!agent->colorblind) {

      for (const auto &beacon : _beacons_) {

        if ((beacon.position - a_pos).norm() < _options_.beacons_radius) {

          nearby_beacon = true;

          /* ROS_ERROR("Agent is nearby beacon: %s", beacon.toStr().c_str()); */

          // Randomly set distribution to different class
          if (binarySelect(_options_.beacons_emission_error)) {

            Distribution error_distribution(beacon.distribution.dim(), beacon.distribution.argmax(), 0.0);
            const int    class_select = nonUniformSelect(error_distribution);
            beacon_distribution       = Distribution(beacon.distribution.dim(), class_select, 1.0);

            /* ROS_ERROR("  - emitting incorrect distribution: %s", beacon_distribution.toStr().c_str()); */

          } else {

            beacon_distribution = beacon.distribution;
            /* ROS_ERROR("  - emitting correct distribution: %s", beacon_distribution.toStr().c_str()); */
          }

          break;
        }
      }
    }

    // Call student's code
    Eigen::Vector3d action;
    Distribution    distribution;
    std::tie(action, distribution) = agent->iterate(neighbors_states, nearby_beacon, beacon_distribution, target_in_agent_frame, user_params);

    if (!distribution.valid()) {

      _task_fail_forced_     = true;
      _task_fail_forced_msg_ = "Function updateAgentState() has returned distribution which sums to 0.";

      return;

    } else {

      // | ---------------- Setup color distribution ---------------- |
      distribution.normalize();
      distributions[i] = distribution;
    }

    if (!vectorIsFinite(action)) {

      _task_fail_forced_     = true;
      _task_fail_forced_msg_ = "Function updateAgentState() has returned nan/inf value in its vector output " + vectorToStr(action) + ".";

      return;

    } else {

      // | ---------------------- Setup action ---------------------- |

      // Set z-axis component to 0
      action[2] = 0.0;

      // Clamp the action magnitude from min to max velocity
      clampVector(action, _options_.velocity_min, _options_.velocity_max);

      // Get signed angle between agent->neighbor velocity vectors
      double signed_angle = signedAngleBetweenTwoVectors(agent->velocity, action);
      if (signed_angle > _options_.heading_max_change) {
        signed_angle = _options_.heading_max_change;
      } else if (signed_angle < -_options_.heading_max_change) {
        signed_angle = -_options_.heading_max_change;
      }

      // Get polar coordinates of agent's velocity
      const auto &[vel_polar_r, vel_polar_az] = vectorToPolarCoordinates(agent->velocity);

      // Limit change in direction by the saturated angle + keep minimal magnitude of the velocity
      action     = vectorFromPolarCoordinates(std::max(_options_.velocity_min, action.norm()), vel_polar_az + signed_angle);
      actions[i] = action;
    }

    // Setup color distribution
    {
      distribution.normalize();
      distributions[i] = distribution;
    }
  }

  // Set the results for all
  for (unsigned int i = 0; i < _agents_.size(); i++) {
    _agents_[i]->velocity     = actions[i];
    _agents_[i]->distribution = distributions[i];
  }
}
/*//}*/

/*//{ moveAgents() */
void WrapperBoids::moveAgents() {
  for (const auto &agent : _agents_) {
    agent->position += agent->velocity;
  }
}
/*//}*/

/*//{ validateTask() */
bool WrapperBoids::validateTask() {
  if (mrs_lib::get_mutexed(_mutex_task_finished_, _task_finished_)) {
    return false;
  }

  if (_task_fail_forced_) {

    _task_result_.success              = false;
    _task_result_.message              = _task_fail_forced_msg_;
    _task_result_.implementation_error = true;

    ROS_ERROR_COND(_verbose_, "[WrapperBoids] TASK FAILED: %s", _task_fail_forced_msg_.c_str());
    _task_failed_ = true;

    mrs_lib::set_mutexed(_mutex_task_finished_, true, _task_finished_);

    return true;
  }

  if (_iteration_ > _task_iterations_max_) {
    /* if ((ros::Time::now() - _task_init_time_).toSec() > _task_timeout_) { */
    /* ROS_ERROR("[WrapperBoids] TASK FAILED: timeout of %.1f seconds have been exceeded.", _task_timeout_); */
    ROS_ERROR_COND(_verbose_, "[WrapperBoids] TASK FAILED: number of iterations have been exceeded (%d iterations > limit of %d).", _iteration_,
                   _task_iterations_max_);

    _task_result_.success          = false;
    _task_result_.message          = "Number of iterations exceeded.";
    _task_result_.finished_in_time = false;

    _task_failed_ = true;
    mrs_lib::set_mutexed(_mutex_task_finished_, true, _task_finished_);

    return true;
  }

  // Validate aliveness
  {
    const double dead_ratio = double(_dead_agents_.size()) / double(_options_.agents_N);

    if (dead_ratio > _dead_ratio_) {

      const double alive_ratio = double(_agents_.size()) / double(_options_.agents_N);

      ROS_WARN_COND(_verbose_, "[WrapperBoids]: Agents statistics -> alive: %ld/%d (%d %s), dead: %ld/%d (%d %s)", _agents_.size(), _options_.agents_N,
                    int(100 * alive_ratio), "%", _dead_agents_.size(), _options_.agents_N, int(100 * dead_ratio), "%");

      if (dead_ratio > _max_dead_ratio_) {
        ROS_ERROR_COND(_verbose_, "[WrapperBoids] TASK FAILED: Too many dead agents: %ld/%d (%d %s > %d %s)", _dead_agents_.size(), _options_.agents_N,
                       int(100 * dead_ratio), "%", int(100 * _max_dead_ratio_), "%");

        _task_result_.success          = false;
        _task_result_.message          = "Too many dead agents.";
        _task_result_.enough_survivors = false;

        _task_failed_ = true;
        mrs_lib::set_mutexed(_mutex_task_finished_, true, _task_finished_);

        return true;
      }
    }
    _dead_ratio_ = dead_ratio;
  }

  // Validate beacon sequence
  {
    // Init phase
    if (_expected_beacon_sequence_.empty()) {
      return false;
    }

    const int E = _expected_beacon_sequence_.size();
    const int S = _beacon_sequence_.size();

    for (int i = 0; i < S; i++) {
      if (_beacon_sequence_[i].id != _expected_beacon_sequence_[i].id) {

        ROS_ERROR_COND(_verbose_, "[WrapperBoids] TASK FAILED: Robots visited beacons in incorrect order.");
        ROS_ERROR_COND(_verbose_, "    VISITED ORDER:");
        for (const auto &beacon : _beacon_sequence_) {
          ROS_ERROR_COND(_verbose_, "      %s", beacon.toStr().c_str());
        }
        ROS_ERROR_COND(_verbose_, "    EXPECTED ORDER:");
        for (const auto &beacon : _expected_beacon_sequence_) {
          ROS_ERROR_COND(_verbose_, "      %s", beacon.toStr().c_str());
        }

        _task_result_.success          = false;
        _task_result_.message          = "Incorrect order.";
        _task_result_.sequence_correct = false;

        _task_failed_ = true;
        mrs_lib::set_mutexed(_mutex_task_finished_, true, _task_finished_);
        return true;
      }
    }

    if (S == E) {
      ROS_INFO_COND(_verbose_, "[WrapperBoids] TASK SUCCESS: Robots visited beacons in correct order and enough of them has survived.");

      _task_result_.success = true;

      _task_failed_ = false;
      mrs_lib::set_mutexed(_mutex_task_finished_, true, _task_finished_);
      return true;
    }
  }
  return false;
}
/*//}*/

// | -------------------- methods: helpers -------------------- |

/* findNeighborsIndices() //{ */
std::vector<int> WrapperBoids::findNeighborsIndices(const int agent_idx, const double radius, [[maybe_unused]] const std::vector<int> &blacklist) {
  // TODO: KDTree?

  // Get position of origin agent
  const auto &agent_pos = _agents_.at(agent_idx)->position;

  std::vector<int> indices;
  indices.reserve(_agents_.size() - 1);

  for (int i = 0; i < int(_agents_.size()); i++) {
    // Skip the origin agent and blacklisted indices
    /* if (i == agent_idx || std::find(blacklist.begin(), blacklist.end(), i) != blacklist.end()) { */
    if (i == agent_idx) {
      continue;
    }

    // Keep if within radius
    if ((agent_pos - _agents_.at(i)->position).norm() < radius) {
      indices.push_back(i);
    }
  }

  /* indices.erase( */
  /*     std::remove_if(std::begin(indices), std::end(indices), [&](auto x) { return std::binary_search(std::begin(blacklist), std::end(blacklist), x); }), */
  /*     std::end(indices)); */

  return indices;
}
//}

/* binarySelect() //{ */
bool WrapperBoids::binarySelect(const double p_true) {

  // Get uniform guess
  const double r = randd(0.0, 1.0);

  return r < p_true;
}
//}

/* nonUniformSelect() //{ */
int WrapperBoids::nonUniformSelect(const Distribution &distribution) {
  const int D = distribution.dim();

  // Get uniform guess
  const double r = randd(0.0, 1.0);

  // Select correct bin (bin width is given by the distribution)
  double p_acc = 0.0;
  for (int i = 0; i < D; i++) {
    const double p_i = distribution.get(i);
    if (r >= p_acc && r < p_acc + p_i) {
      return i;
    }
    p_acc += p_i;
  }

  // r == 1.0
  return D - 1;
}
//}

/* randd() //{ */

double WrapperBoids::randd(const double from, const double to) {

  const double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randdnorm() //{ */

double WrapperBoids::randdnorm(const double mean, const double stdev) {

  std::normal_distribution<double> distribution(mean, stdev);
  return distribution(random_engine_);
}

//}

// | ----------------- methods: visualization ----------------- |

/* publishVisualization() //{ */

void WrapperBoids::publishVisualization() {
  if (_ph_vis_.getNumSubscribers() > 0 || _ph_vis_students_.getNumSubscribers() > 0) {
    setupVisualization();
    _ph_vis_.publish(_msg_vis_);
    _ph_vis_students_.publish(_msg_vis_students_);
  }
}

//}

/* setupVisualization() //{ */
void WrapperBoids::setupVisualization() {
  _msg_vis_          = boost::make_shared<visualization_msgs::MarkerArray>();
  _msg_vis_students_ = boost::make_shared<visualization_msgs::MarkerArray>();

  const auto          stamp    = ros::Time::now();
  const ros::Duration lifetime = ros::Duration(2.0 / _timer_iterate_rate_);

  _marker_predator_zone_.header.stamp = stamp;
  _marker_beacons_inner_.header.stamp = stamp;
  _marker_beacons_outer_.header.stamp = stamp;
  _msg_vis_->markers                  = {_marker_predator_zone_, _marker_beacons_inner_, _marker_beacons_outer_};

  setupVisAgents(lifetime, stamp);
}
//}

/* setupVisAgents() //{ */

void WrapperBoids::setupVisAgents(const ros::Duration &lifetime, const ros::Time &stamp) {

  const auto &[m_pos, m_vel, m_dead, m_edges] = getVisAgents(lifetime, stamp);
  _msg_vis_->markers.push_back(m_pos);
  _msg_vis_->markers.push_back(m_vel);

  if (!m_dead.points.empty()) {
    _msg_vis_->markers.push_back(m_dead);
  }
  if (!m_edges.points.empty()) {
    _msg_vis_->markers.push_back(m_edges);
  }

  // Student's arrows
  for (const auto &agent : _agents_) {
    const auto &arrows = agent->getArrows();

    for (const auto &arrow : arrows) {
      visualization_msgs::Marker m_arrow;
      m_arrow.header.frame_id = _options_.vis_frame;
      m_arrow.header.stamp    = stamp;
      m_arrow.type            = visualization_msgs::Marker::ARROW;
      m_arrow.ns              = "agents/" + std::to_string(agent->id) + "/" + arrow.name;

      m_arrow.lifetime           = lifetime;
      m_arrow.pose.orientation.w = 1.0;

      m_arrow.scale.x = _options_.vis_arrow_shaft_diameter;
      m_arrow.scale.y = _options_.vis_arrow_head_diameter;

      m_arrow.color.r = arrow.color.red;
      m_arrow.color.g = arrow.color.green;
      m_arrow.color.b = arrow.color.blue;
      m_arrow.color.a = arrow.color.alpha;

      m_arrow.points.resize(2);
      m_arrow.points[0].x = agent->position.x();
      m_arrow.points[0].y = agent->position.y();
      m_arrow.points[0].z = agent->position.z();
      m_arrow.points[1].x = agent->position.x() + arrow.endpoint.x();
      m_arrow.points[1].y = agent->position.y() + arrow.endpoint.y();
      m_arrow.points[1].z = agent->position.z() + arrow.endpoint.z();

      _msg_vis_students_->markers.push_back(m_arrow);
    }
  }
}

//}

/*//{ getVisAgents() */
std::tuple<visualization_msgs::Marker, visualization_msgs::Marker, visualization_msgs::Marker, visualization_msgs::Marker> WrapperBoids::getVisAgents(
    const ros::Duration &lifetime, const ros::Time &stamp) {

  visualization_msgs::Marker m_pos;
  m_pos.header.frame_id = _options_.vis_frame;
  m_pos.header.stamp    = stamp;
  m_pos.type            = visualization_msgs::Marker::SPHERE_LIST;
  m_pos.ns              = "agents/positions";

  visualization_msgs::Marker m_vel;
  m_vel.header = m_pos.header;
  m_vel.type   = visualization_msgs::Marker::LINE_LIST;
  m_vel.ns     = "agents/velocities";

  visualization_msgs::Marker m_dead;
  m_dead.header = m_pos.header;
  m_dead.type   = visualization_msgs::Marker::CUBE_LIST;
  m_dead.ns     = "agents/dead";

  visualization_msgs::Marker m_edges;
  m_edges.header = m_pos.header;
  m_edges.type   = visualization_msgs::Marker::LINE_LIST;
  m_edges.ns     = "agents/edges";

  // Scales
  m_pos.scale.x = 2.0 * _options_.vis_agents_pos_size;
  m_pos.scale.y = m_pos.scale.x;
  m_pos.scale.z = m_pos.scale.x;

  m_vel.scale.x = _options_.vis_agents_vel_size;

  m_dead.scale.x = _options_.vis_agents_dead_size;
  m_dead.scale.y = m_dead.scale.x;
  m_dead.scale.z = m_dead.scale.x;

  m_edges.scale.x = _options_.vis_edges_lw;

  // Lifetimes
  m_pos.lifetime   = lifetime;
  m_vel.lifetime   = lifetime;
  m_dead.lifetime  = lifetime;
  m_edges.lifetime = lifetime;

  // Poses
  m_pos.pose.orientation.w   = 1.0;
  m_vel.pose.orientation.w   = 1.0;
  m_dead.pose.orientation.w  = 1.0;
  m_edges.pose.orientation.w = 1.0;

  // Resizes
  m_pos.points.resize(_agents_.size());
  m_pos.colors.resize(_agents_.size());

  m_vel.points.resize(2 * _agents_.size());
  m_vel.colors.resize(2 * _agents_.size());

  std::vector<std::pair<int, int>> edges;

  /* ROS_ERROR(" agents count: %ld", _agents_.size()); */
  for (unsigned int i = 0; i < _agents_.size(); i++) {

    /* ROS_ERROR("  processing agent: %d", i); */

    const auto &agent       = _agents_.at(i);
    const auto &a_direction = agent->getDirection();

    // Setup positions
    m_pos.points[i].x = agent->position.x();
    m_pos.points[i].y = agent->position.y();
    m_pos.points[i].z = agent->position.z();

    const auto a_color = COLORS[directionToInt(a_direction)];
    m_pos.colors[i].r  = a_color.red;
    m_pos.colors[i].g  = a_color.green;
    m_pos.colors[i].b  = a_color.blue;
    m_pos.colors[i].a  = a_color.alpha;

    // Setup velocities
    const int i_from     = 2 * i;
    const int i_to       = (2 * i) + 1;
    m_vel.points[i_from] = m_pos.points[i];

    const auto vel_endpoint = agent->position + agent->velocity;
    m_vel.points[i_to].x    = vel_endpoint.x();
    m_vel.points[i_to].y    = vel_endpoint.y();
    m_vel.points[i_to].z    = vel_endpoint.z();

    m_vel.colors[i_from] = m_pos.colors[i];
    m_vel.colors[i_to]   = m_pos.colors[i];

    // Setup edges
    if (_options_.vis_edges_show) {

      /* ROS_ERROR("   neighbors count: %ld", agent->neighbors.size()); */
      for (const int n : agent->neighbors) {

        /* ROS_ERROR("    neighbor: %d", n); */

        const std::pair<int, int> opposite_edge = std::make_pair(n, i);

        // Find iterator to opposite edge
        auto it = std::find_if(edges.begin(), edges.end(),
                               [&opposite_edge](const std::pair<int, int> &x) { return x.first == opposite_edge.second && x.second == opposite_edge.first; });

        // Add only if the opposite does not exist yet
        if (it == edges.end()) {
          edges.push_back(std::make_pair(i, n));

          const auto &neighbor = _agents_.at(n);

          geometry_msgs::Point p_from;
          geometry_msgs::Point p_to;

          p_from.x = agent->position.x();
          p_from.y = agent->position.y();
          p_from.z = agent->position.z() - 0.05;

          p_to.x = neighbor->position.x();
          p_to.y = neighbor->position.y();
          p_to.z = neighbor->position.z() - 0.05;

          const std_msgs::ColorRGBA color = (a_direction == neighbor->getDirection()) ? m_pos.colors[i] : COLOR_GRAY;

          m_edges.points.push_back(p_from);
          m_edges.points.push_back(p_to);
          m_edges.colors.push_back(color);
          m_edges.colors.push_back(color);
        }
      }
      /* ROS_ERROR("   neighbors iterated"); */
    }
  }

  if (!_dead_agents_.empty()) {

    m_dead.points.resize(_dead_agents_.size());

    m_dead.color = COLOR_BLACK;

    for (unsigned int i = 0; i < _dead_agents_.size(); i++) {
      const auto &agent = _dead_agents_.at(i);

      // Setup positions
      m_dead.points[i].x = agent->position.x();
      m_dead.points[i].y = agent->position.y();
      m_dead.points[i].z = agent->position.z() - (_options_.vis_agents_dead_size / 2.0) - 0.05;
    }
  }

  return {m_pos, m_vel, m_dead, m_edges};
}
/*//}*/

/*//{ setupVisEnvironment() */
void WrapperBoids::setupVisEnvironment() {

  // Marker definitions
  _marker_predator_zone_.header.frame_id = _options_.vis_frame;
  _marker_predator_zone_.type            = visualization_msgs::Marker::LINE_STRIP;
  _marker_predator_zone_.ns              = "predator/zone";

  _marker_beacons_inner_.header = _marker_predator_zone_.header;
  _marker_beacons_inner_.type   = visualization_msgs::Marker::SPHERE_LIST;
  _marker_beacons_inner_.ns     = "beacons/color";

  _marker_beacons_outer_.header = _marker_predator_zone_.header;
  _marker_beacons_outer_.type   = visualization_msgs::Marker::SPHERE_LIST;
  _marker_beacons_outer_.ns     = "beacons/pointer";

  // Scales
  _marker_predator_zone_.scale.x = _options_.vis_predator_zone_lw;

  _marker_beacons_inner_.scale.x = 2.0 * _options_.vis_beacons_size_inner;
  _marker_beacons_inner_.scale.y = _marker_beacons_inner_.scale.x;
  _marker_beacons_inner_.scale.z = _marker_beacons_inner_.scale.x;

  _marker_beacons_outer_.scale.x = 2.0 * _options_.beacons_radius;
  _marker_beacons_outer_.scale.y = _marker_beacons_outer_.scale.x;
  _marker_beacons_outer_.scale.z = _marker_beacons_outer_.scale.x;

  // Poses
  _marker_predator_zone_.pose.orientation.w = 1.0;
  _marker_beacons_inner_.pose.orientation.w = 1.0;
  _marker_beacons_outer_.pose.orientation.w = 1.0;

  // Default colors
  _marker_predator_zone_.color = COLOR_BLACK;

  // Resizes
  _marker_predator_zone_.points.resize(5);
  _marker_beacons_inner_.points.resize(_beacons_.size());
  _marker_beacons_inner_.colors.resize(_beacons_.size());
  _marker_beacons_outer_.points.resize(_beacons_.size());
  _marker_beacons_outer_.colors.resize(_beacons_.size());

  // Fill: border
  const double p_x_max               = _options_.beacons_spawn_square_size + _options_.kill_predator_dist_from_beacons;
  const double p_x_min               = -p_x_max;
  const double p_y_max               = _options_.beacons_spawn_square_size + _options_.kill_predator_dist_from_beacons;
  const double p_y_min               = -p_y_max;
  _marker_predator_zone_.points[0].x = p_x_min;
  _marker_predator_zone_.points[0].y = p_y_min;
  _marker_predator_zone_.points[1].x = p_x_max;
  _marker_predator_zone_.points[1].y = p_y_min;
  _marker_predator_zone_.points[2].x = p_x_max;
  _marker_predator_zone_.points[2].y = p_y_max;
  _marker_predator_zone_.points[3].x = p_x_min;
  _marker_predator_zone_.points[3].y = p_y_max;
  _marker_predator_zone_.points[4].x = p_x_min;
  _marker_predator_zone_.points[4].y = p_y_min;

  // Fill: beacons
  for (unsigned int i = 0; i < _beacons_.size(); i++) {
    const auto &beacon = _beacons_.at(i);

    // Setup positions
    _marker_beacons_inner_.points[i].x = beacon.position.x();
    _marker_beacons_inner_.points[i].y = beacon.position.y();
    _marker_beacons_inner_.points[i].z = beacon.position.z();

    const auto color_i                 = beacon.color;
    _marker_beacons_inner_.colors[i].r = color_i.red;
    _marker_beacons_inner_.colors[i].g = color_i.green;
    _marker_beacons_inner_.colors[i].b = color_i.blue;
    _marker_beacons_inner_.colors[i].a = 1.0;

    _marker_beacons_outer_.points[i] = _marker_beacons_inner_.points[i];
    _marker_beacons_outer_.points[i].z -= 0.05;  // Move a bit down

    const auto color_o                 = beacon.color_pointer;
    _marker_beacons_outer_.colors[i].r = color_o.red;
    _marker_beacons_outer_.colors[i].g = color_o.green;
    _marker_beacons_outer_.colors[i].b = color_o.blue;
    _marker_beacons_outer_.colors[i].a = _options_.vis_beacons_alpha;
  }
}
/*//}*/

// | ----------------- methods: initialization ---------------- |

/* loadOptions() //{ */
bool WrapperBoids::loadOptions() {

  _param_loader_->loadParam("classes", _options_.classes_N);
  if (_options_.classes_N <= 0) {
    return false;
  }

  std::vector<double> distribution;

  // Agents
  _param_loader_->loadParam("agents/count", _options_.agents_N);
  _param_loader_->loadParam("agents/distribution/spawn", distribution);
  _param_loader_->loadParam("agents/distribution/prior/max_class_prob", _options_.spawn_prior_max_class_prob);
  _param_loader_->loadParam("agents/spawn/limits/x/min", _options_.spawn_lim_x_min);
  _param_loader_->loadParam("agents/spawn/limits/x/max", _options_.spawn_lim_x_max);
  _param_loader_->loadParam("agents/spawn/limits/y/min", _options_.spawn_lim_y_min);
  _param_loader_->loadParam("agents/spawn/limits/y/max", _options_.spawn_lim_y_max);
  _param_loader_->loadParam("agents/colorblind_ratio", _options_.colorblind_ratio);
  _param_loader_->loadParam("agents/neighborhood/radius", _options_.neighborhood_radius);

  _options_.spawn_distribution = Distribution(distribution);

  if (_options_.agents_N <= 0 || int(_options_.spawn_distribution.dim()) != _options_.classes_N || _options_.spawn_lim_x_min > _options_.spawn_lim_x_max ||
      _options_.spawn_lim_y_min > _options_.spawn_lim_y_max || _options_.neighborhood_radius <= 0.0) {
    return false;
  }

  // Beacons
  _param_loader_->loadParam("beacons/radius", _options_.beacons_radius);
  _param_loader_->loadParam("beacons/spawn_distance", _options_.beacons_spawn_square_size);
  _param_loader_->loadParam("beacons/emission_error", _options_.beacons_emission_error);
  if (_options_.beacons_spawn_square_size <= 0.0) {
    return false;
  }

  // Killing agents
  _param_loader_->loadParam("agents/kill/enable", _options_.killing_enabled);
  _param_loader_->loadParam("agents/kill/proximity/radius", _options_.kill_proximity_radius);
  _param_loader_->loadParam("agents/kill/proximity/probability", _options_.kill_proximity_prob);
  _param_loader_->loadParam("agents/kill/predator/distance_from_beacons", _options_.kill_predator_dist_from_beacons);
  _param_loader_->loadParam("agents/kill/predator/probability", _options_.kill_predator_prob);
  _param_loader_->loadParam("agents/kill/knn/radius", _options_.kill_knn_radius);
  _param_loader_->loadParam("agents/kill/knn/min_neighbors", _options_.kill_knn_min_neighbors);
  _param_loader_->loadParam("agents/kill/knn/probability", _options_.kill_knn_prob);

  // Constraints
  _param_loader_->loadParam("constraints/velocity/min", _options_.velocity_min);
  _param_loader_->loadParam("constraints/velocity/max", _options_.velocity_max);
  _param_loader_->loadParam("constraints/heading/max_change", _options_.heading_max_change);

  if (_options_.velocity_min > _options_.velocity_max || _options_.heading_max_change <= 0.0) {
    return false;
  }

  // Visualization
  _param_loader_->loadParam("visualization/frame", _options_.vis_frame);
  _param_loader_->loadParam("visualization/edges/show", _options_.vis_edges_show);
  _param_loader_->loadParam("visualization/edges/lw", _options_.vis_edges_lw);
  /* _param_loader_->loadParam("visualization/texts", _options_.vis_show_texts); */
  _param_loader_->loadParam("visualization/agents/size/pos", _options_.vis_agents_pos_size);
  _param_loader_->loadParam("visualization/agents/size/vel", _options_.vis_agents_vel_size);
  _param_loader_->loadParam("visualization/agents/size/dead", _options_.vis_agents_dead_size);
  _param_loader_->loadParam("visualization/environment/predator_zone/lw", _options_.vis_predator_zone_lw);
  _param_loader_->loadParam("visualization/environment/beacons/inner/size", _options_.vis_beacons_size_inner);
  _param_loader_->loadParam("visualization/environment/beacons/alpha", _options_.vis_beacons_alpha);
  _param_loader_->loadParam("visualization/arrows/shaft_diameter", _options_.vis_arrow_shaft_diameter);
  _param_loader_->loadParam("visualization/arrows/head_diameter", _options_.vis_arrow_head_diameter);

  const double p_x_max = _options_.beacons_spawn_square_size + _options_.kill_predator_dist_from_beacons;
  const double p_x_min = -p_x_max;
  const double p_y_max = _options_.beacons_spawn_square_size + _options_.kill_predator_dist_from_beacons;
  const double p_y_min = -p_y_max;
  _predator_zone_min_  = std::make_tuple(p_x_min, p_y_min);
  _predator_zone_max_  = std::make_tuple(p_x_max, p_y_max);

  return true;
}
//}

/* setupBeacons() //{ */
void WrapperBoids::setupBeacons() {

  std::vector<Eigen::Vector3d> b_positions = {Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(-1, 1, 0), Eigen::Vector3d(1, -1, 0), Eigen::Vector3d(-1, -1, 0)};
  for (auto &vec : b_positions) {
    vec *= _options_.beacons_spawn_square_size;
  }
  std::shuffle(std::begin(b_positions), std::end(b_positions), random_engine_);

  std::vector<Color_t> b_colors = COLORS;
  std::shuffle(std::begin(b_colors), std::end(b_colors), random_engine_);

  const int D = b_colors.size();
  ROS_INFO_COND(_verbose_, "[WrapperBoids]: Initialized beacons:");
  for (int i = 0; i < D; i++) {

    Beacon_t beacon;
    beacon.id            = i;
    beacon.position      = b_positions[i];
    beacon.color         = b_colors[i];
    beacon.color_pointer = b_colors[(i + 1) % D];

    auto      color_it  = std::find(COLORS.begin(), COLORS.end(), beacon.color_pointer);
    const int color_idx = std::distance(COLORS.begin(), color_it);

    beacon.distribution = Distribution(D, color_idx, 1.0);

    _beacons_.push_back(beacon);

    ROS_INFO_COND(_verbose_, "  %s", beacon.toStr().c_str());
  }
}
//}

/* spawnAgents() //{ */
void WrapperBoids::spawnAgents() {
  const int N = _options_.agents_N;
  const int C = _options_.classes_N;

  _agents_ = std::vector<AgentPtr>(N);

  const int colorblind_N = _options_.colorblind_ratio * N;

  /* std::cout << "Spawning agents:" << std::endl; */
  for (int i = 0; i < N; i++) {

    const Eigen::Vector3d position =
        Eigen::Vector3d(randd(_options_.spawn_lim_x_min, _options_.spawn_lim_x_max), randd(_options_.spawn_lim_y_min, _options_.spawn_lim_y_max), 0.0);

    Eigen::Vector3d velocity = Eigen::Vector3d(randd(-1.0, 1.0), randd(-1.0, 1.0), 0.0);
    velocity                 = randd(_options_.velocity_min, _options_.velocity_max) * velocity.normalized();

    const int          class_select = nonUniformSelect(_options_.spawn_distribution);
    const Distribution distribution = Distribution(C, class_select, _options_.spawn_prior_max_class_prob);

    /* std::cout << "  i: " << i << std::endl; */
    /* std::cout << "  spawn distribution: " << _options_.spawn_distribution << std::endl; */
    /* std::cout << "  class: " << class_select << std::endl; */
    /* std::cout << "  distribution: " << distribution << std::endl; */

    const bool colorblind = i < colorblind_N;

    _agents_[i] = std::make_shared<task_03_boids::Agent>(i, position, velocity, distribution, colorblind, _options_.neighborhood_radius);
  }
}
//}

}  // namespace task_03_wrapper
