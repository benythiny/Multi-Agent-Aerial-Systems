#include <task_03_boids/boids.h>

namespace task_03_boids
{

/**
 * @brief Non-linear weighting of forces.
 *
 * The function is to be non-increasing, non-negative, and grows to infinity as the distance is approaching the lower bound (the safety distance). Below the
 * lower bound (including), the function is to be undefined. Over the visibility range, the function shall return 0.
 *
 * @param distance to an agent/obstacle
 * @param visibility visibility range of the UAVs
 * @param safety distance: min distance to other UAVs or obstacles
 * @param desired distance: desired distance to other UAVs or obstacles (does not need to be used)
 *
 * @return
 *   bool:   True if function is defined for the given distance, False otherwise
 *   double: Weight for an agent/obstacle at given distance, if function is defined for the given distance.
 */
 double weightingFunctionBoids(const double distance, const double visibility, const double safety_distance) {

  bool valid_value = false;

  // hyperbola
  double h = visibility / (distance - safety_distance);

  if (distance <= visibility){
    //valid_value = true;
  } else {
    h = 0;
  }

  // Below the lower bound (including) the function is undefined.
  if (distance <= safety_distance){
    h = 1000000;
  }
  return h;
}


/**
 * @brief Calculate a next-iteration action of one agent given relative information of its neighbors and the direction towards a target.
 *        This method is supposed to be filled in by the student.
 *
 * @param AgentState_t Current state of the agent as defined in agent_state.h.
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization
 *  - visualizeArrow() will publish the given arrow in the agent frame within the visualization
 *
 * @return
 *    1) XYZ vector in frame of the agent to be set as velocity command. Beware that i) the vector z-axis component will be set to 0, ii) the vector magnitude
 * will be clamped into <v_min, v_max> limits and iii) azimuth of the vector's XY-projection will be saturated such that the azimuth between the agent's current
 * velocity and the vector does not exceed a maximal change.
 *
 *       Example 1: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 1, 1) will be clamped and saturated to
 * 0.2*(cos(d), sin(d), 0).
 *
 *       Example 2: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, -0.05, 1) will
 * be clamped and saturated to 0.1*(cos(-d), sin(-d), 0).
 *
 *    2) Probability distribution of colors to be set to the agent for next iteration. Beware that output distribution.dim() has to equal input
 * state.distribution.dim().
 */
std::tuple<Eigen::Vector3d, Distribution> Boids::updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                  const ActionHandlers_t &action_handlers) {

  Eigen::Vector3d target = state.target;
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  Eigen::Vector3d cohesion = Eigen::Vector3d::Zero();
  Eigen::Vector3d alignment = Eigen::Vector3d::Zero();
  Eigen::Vector3d separation = Eigen::Vector3d::Zero();

  // Access my own prob. distribution of colors
  Distribution my_distribution = state.distribution;
  int          dim             = my_distribution.dim();
  double small_prob = 0.0001;
  double learning_rate_for_neighbors = 1;

  // check that probs are not set to 0
  for (int color_id=0; color_id< dim; color_id++){
    if (my_distribution.get(color_id) < 0.001 ) {
      my_distribution.set(color_id, small_prob);
    }
    if (my_distribution.get(color_id) == 1) {
      my_distribution.set(color_id, 1 - 3 * small_prob);
    }
  }
  
  // Detect my color
  int my_color_id = my_distribution.argmax();

  // Am I nearby a beacon?
  Distribution beacon_distribution;
  if (state.nearby_beacon) {
    beacon_distribution = state.beacon_distribution;
  }

  // collective color distribution of the neighbouring boids
  double neighbor_colors[4] = {0.0, 0.0, 0.0, 0.0};
  Distribution neigbours_distrib = Distribution(4);
  neigbours_distrib = my_distribution;
  

  // Iterate over the states of the visible neighbors
  for (const auto &n_state : state.neighbors_states) {

    /* navigation part */
    auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

    cohesion += n_pos_rel;
    alignment += n_vel_global;

    double dist_to_neightbor = n_pos_rel.norm();
    double sep_wights = weightingFunctionBoids(dist_to_neightbor, user_params.param6, user_params.param5);
    separation -= n_pos_rel.normalized() * sep_wights;

    /* Color consensus part */

    for (int color_i=0; color_i<4; color_i++){

      // add up new neighbor color
      double value = n_distribution.get(color_i);

      if (n_distribution.argmax() == color_i){
        value = value * learning_rate_for_neighbors;
      }

      neigbours_distrib.add(color_i, value);
    }

    // check if the size of my prob. distribution matches the size of the neighbour's distribution
    if (dim != n_distribution.dim()) {
      std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
    }
  }

  /* navigation part */
  if (state.neighbors_states.size()){
    cohesion /= state.neighbors_states.size();
    separation /= state.neighbors_states.size();
    alignment /= (state.neighbors_states.size() + 1);
  }
  
  action = user_params.param1 * cohesion + user_params.param2 * separation + user_params.param3 * alignment + user_params.param4 * target;

  // Visualize the arrow in RViz
  action_handlers.visualizeArrow("action", action, Color_t{0.5, 0.5, 0.5, 1.0});

  /* Color consensus part */

  
  // check that neighbors' probs are not set to 0
  for (int color_id=0; color_id< dim; color_id++){
    if (neigbours_distrib.get(color_id) < 0.001) {
      neigbours_distrib.set(color_id, small_prob);
    }
    if (neigbours_distrib.get(color_id) == 1) {
      neigbours_distrib.set(color_id, 1 - 3 * small_prob);
    }
  }

  // normalise neighbors distribution
  neigbours_distrib.normalize();

  // check for correctness
  if (!neigbours_distrib.valid()){
    printf("ERROR IN ASSIGNING PROBABILITY\n");
  }

  // MAIN UPDATE my distribution color: p(my) * p (neighbors)
  for (int color_id=0; color_id<dim; color_id++){
    double value = my_distribution.get(color_id) * neigbours_distrib.get(color_id);
    my_distribution.set(color_id, value);
  }

  // normalise my distribution
  my_distribution.normalize();

  // Am I nearby a beacon?
  if (state.nearby_beacon) {
    my_distribution = state.beacon_distribution;
  } else {
    my_distribution = neigbours_distrib;
  }

  // check that probs are not set to 0
  for (int color_id=0; color_id< dim; color_id++){
    if (my_distribution.get(color_id) < 0.001) {
      my_distribution.set(color_id, small_prob);
    }
    if (my_distribution.get(color_id) == 1) {
      my_distribution.set(color_id, 1 - 3 * small_prob);
    }
  }

  
  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
