#ifndef AGENT_H
#define AGENT_H

#include <task_03_common/direction.h>
#include <task_03_common/distribution.h>
#include <task_03_common/utilities.h>
#include <task_03_common/user_params.h>
#include <task_03_boids/action_handlers.h>
#include <task_03_boids/boids.h>

#include <memory>
#include <map>
#include <mutex>

namespace task_03_boids
{

// |  LIBRARY INTERFACE: DO NOT MODIFY, YOUR CHANGES WILL BE OVERWRITTEN  |

class Agent {

private:
  /* struct Arrow_t //{ */

  typedef struct
  {
    std::string     name;
    Eigen::Vector3d endpoint;
    Color_t         color;
  } Arrow_t;

  //}

public:
  Agent(const int id, const Eigen::Vector3d &position, const Eigen::Vector3d &velocity, const Distribution &distribution, const bool colorblind,
        [[maybe_unused]] const double visibility_radius) {

    this->id           = id;
    this->position     = position;
    this->velocity     = velocity;
    this->distribution = distribution;
    this->colorblind   = colorblind;

    // Bind action handlers method
    this->action_handlers.visualizeArrow = std::bind(&Agent::storeArrow, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // Setup Boids object
    _boids_ = std::make_unique<Boids>();
  }

  Direction_t getDirection() {
    return intToDirection(this->distribution.argmax());
  }

  void setDistribution(const Distribution &distribution) {
    this->distribution = distribution;
  }

  bool operator==(const Agent &other) {
    return this->id == other.id;
  }

  /* storeArrow() //{ */

  void storeArrow(const std::string &name, const Eigen::Vector3d &endpoint, const Color_t &color) {

    Arrow_t arrow;

    arrow.name     = name;
    arrow.endpoint = endpoint;
    arrow.color    = color;

    {
      std::scoped_lock lock(_mutex_arrows_);
      _arrows_map_[name] = arrow;
    }
  }

  //}

  /* iterate() //{ */

  std::tuple<Eigen::Vector3d, Distribution> iterate(const std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Distribution>> &neighbors_states,
                                                    const bool nearby_beacon, const Distribution &beacon_distribution, const Eigen::Vector3d &target,
                                                    const UserParams_t &user_params) {
    {
      std::scoped_lock lock(_mutex_arrows_);
      _arrows_map_.clear();
    }

    AgentState_t state;

    state.velocity            = this->velocity;
    state.neighbors_states    = neighbors_states;
    state.target              = target;
    state.distribution        = this->distribution;
    state.nearby_beacon       = nearby_beacon;
    state.beacon_distribution = beacon_distribution;

    return _boids_->updateAgentState(state, user_params, this->action_handlers);
  }

  //}

  /* getArrows() //{ */

  std::vector<Arrow_t> getArrows() {

    std::vector<Arrow_t> arrows;
    {
      std::scoped_lock lock(_mutex_arrows_);
      arrows.reserve(_arrows_map_.size());

      for (auto it = _arrows_map_.begin(); it != _arrows_map_.end(); ++it) {
        arrows.push_back(it->second);
      }
    }

    return arrows;
  }

  //}

public:
  int             id;
  bool            colorblind;
  Eigen::Vector3d position;  // global
  Eigen::Vector3d velocity;  // global
  Distribution    distribution;

  // Indices in a central vector storing all the agents
  std::vector<int> neighbors;

private:
  std::unique_ptr<Boids> _boids_;

  task_03_boids::ActionHandlers_t action_handlers;
  std::mutex                      _mutex_arrows_;
  std::map<std::string, Arrow_t>  _arrows_map_;
};

typedef std::shared_ptr<Agent> AgentPtr;

}  // namespace task_03_boids

#endif  // AGENT_H
