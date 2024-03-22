#include <student_headers/swarm.h>
#include <cmath>

namespace task_03_swarm
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The swarm controller (single unit) initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param visibility radius of the agent
 */
void Swarm::init(const double visibility_radius) {

  _visibility_radius_ = visibility_radius;
}

//}


Eigen::Vector3d Swarm::calcCohesion(const Perception_t &perception, const UserParams_t &user_params){
  Eigen::Vector3d result = Eigen::Vector3d(0,0,0);

  for (auto &uav : perception.neighbors){
    result += uav.position;
  }
  result *= 1.0 / perception.neighbors.size();
  printf("Cohesion: %f, %f, %f \n", result[0], result[1], result[2]);
  return result;
}

Eigen::Vector3d Swarm::calcAttraction(const Perception_t &perception, const UserParams_t &user_params){
  // return Eigen::Vector3d(cos(user_params.param5), sin(user_params.param5),0);
  //  the closest gate:
  // unsigned int closest_gate_idx = selectGateClosest(perception.obstacles);
  // auto closest_gate     = perception.obstacles.gates[closest_gate_idx];

  // auto gates_mean = closest_gate[0] + closest_gate[1];


  // return (perception.target_vector + gates_mean) / 2;
  //return perception.target_vector;
  printf("Attraction: %f, %f, %f \n", base_direction_vector[0], base_direction_vector[1], base_direction_vector[2]);
  return base_direction_vector;
}

Eigen::Vector3d Swarm::calcAvoidance(const Perception_t &perception, const UserParams_t &user_params, 
                                     const bool &closeToGate){


  double distance = perception.obstacles.closest.norm();
  double my_weight = 20;

  double safe_dist = user_params.param5;
  auto [defined, weight] = weightingFunction(distance, _visibility_radius_, safe_dist, 0.0);
  
  if (defined){
    my_weight = weight;
  } else {
    my_weight = 10e6;
  }

  Eigen::Vector3d result = - my_weight * perception.obstacles.closest.normalized();
  printf("Avoidance: %f, %f, %f \n", result[0], result[1], result[2]);
  return result;

}

Eigen::Vector3d Swarm::calcSeparation(const Perception_t &perception, const UserParams_t &user_params){

  Eigen::Vector3d result = Eigen::Vector3d(0,0,0);

  for (auto &uav : perception.neighbors) {
    double distance = uav.position.norm();

    auto [defined, weight] = weightingFunction(distance, _visibility_radius_, SAFETY_DISTANCE_UAVS, 0.0);

    double my_weight;

    if (defined){
      my_weight = weight;
    } else {
      my_weight = 10e6;
    }

    result += my_weight * uav.position.normalized();
  }

  result *= -1.0 / perception.neighbors.size();
  printf("Separation: %f, %f, %f \n", result[0], result[1], result[2]);
  return result;
}

/* updateAction() //{ */
/*
Eigen::Vector3d Swarm::updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers) {
  Eigen::Vector3d cohesion    = user_params.param1 * calcCohesion(perception, user_params);
  Eigen::Vector3d separation  = user_params.param2 * calcSeparation(perception, user_params);
  Eigen::Vector3d avoidance   = user_params.param3 * calcAvoidance(perception, user_params);
  Eigen::Vector3d attraction  = user_params.param4 * calcAttraction(perception, user_params);
return cohesion + separation + avoidance + attraction;
*/
/**
 * @brief This method calculates a next-iteration action of one UAV given relative information of its neighbors and obstacles; and the direction towards the
 *        moving target. This method is supposed to be filled in by the student.
 *
 * @param perception Current perceptual information of this UAV. Defined in perception.h. It contains:
 *  - current time
 *  - target vector: 3D vector towards the moving robot in the UAV body frame
 *  - neighbors defined by:
 *    - their position in the UAV body frame
 *    - the variables shared through the communication network
 *  - obstacles consisting of:
 *    - 3D vector from the body frame to the closest obstacle in the environment
 *    - 4 gates (pairs of 2 gate edges) in the UAV body frame
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization and data sharing among the UAVs:
 *  - shareVariables(int, int, double) will share the three basic-type variables among the UAVs
 *  - visualizeArrow() will publish the given arrow in the UAV body frame within the visualization
 *  - visualizeArrowFrom() will publish the given arrow at position given in the UAV body frame within the visualization
 *  - visualizeCube() will publish a cube in the UAV body frame within the visualization
 *
 * @return Next-iteration action for this UAV given as a 3D vector. Zero vector is expected if no action should be performed. Beware that i) the vector z-axis
 * component will be set to 0, ii) the vector vector magnitude will be clamped into <0, v_max> limits, and iii) the vector will be passed to a velocity
 * controller of the UAV.
 *
 *       Example 1: v_max=0.75 -> vector (1, 0, 1) will be saturated to 0.75*(1, 0, 0).
 */
Eigen::Vector3d Swarm::updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers) {

  // Setup output control signal
  Eigen::Vector3d vec_action = Eigen::Vector3d::Zero();

  // Access the perception struct
  double          current_time   = perception.time;
  Eigen::Vector3d vec_navigation = perception.target_vector;
  Eigen::Vector3d vec_separation = Eigen::Vector3d::Zero();

  // final returned vector with velocity direction
  Eigen::Vector3d final_vector = Eigen::Vector3d::Zero();
  

  // Variables initialization
  bool compute_action = false;

  // transform the direction ID into 3D vector 
  base_direction_vector = directionToDirectionVector(_navigation_direction_, perception);

  // STATE MACHINE BEGINNING
  switch (_state_) {

      /* case INIT_STATE //{ */

    case INIT_STATE: {

      std::cout << "Current state: " << stateToString(INIT_STATE) << std::endl;

      // std::cout << "Changing to state: " << stateToString(AGREEING_ON_DIRECTION) << std::endl;
      _state_          = AGREEING_ON_DIRECTION;
      idling_time_init = current_time;

      _navigation_direction_ = targetToDirection(perception.target_vector);
      action_handlers.shareVariables(INIT_STATE, directionToInt(_navigation_direction_), my_position_in_swarm);

      break;
    }

      //}

      /* case AGREEING_ON_DIRECTION() //{ */

    case AGREEING_ON_DIRECTION: {

      std::cout << "Current state: " << stateToString(AGREEING_ON_DIRECTION) << std::endl;
      bool direction_agreed = false;

      // init array for my_dir + neighbors_dir
      std::vector<int> all_directions = {};

      // get my direction 
      Direction_t my_navigation_direction_ = targetToDirection(perception.target_vector);
      all_directions.push_back({directionToInt(my_navigation_direction_)});

      // go through all neighbors to get their directions
      for (const auto &n : perception.neighbors) {
        SharedVariables_t n_shared_vars = n.shared_variables;
        
        int n_uav_state = n_shared_vars.int1;
        int n_direction = n_shared_vars.int2;
        
        all_directions.push_back(n_direction);
      }

      if (!direction_agreed){
        auto             counts                                    = countIntegers(all_directions);
        [[maybe_unused]] const auto &[majority_idx, majority_freq] = getMajority(counts);

        if ((majority_freq >= 2) && (intToDirection(majority_idx)!=NONE)){
          printf("Majority found with index: %d, majority freq: %d\n", majority_idx, majority_freq);
          direction_agreed = true;
          _navigation_direction_ = intToDirection(majority_idx);
        }
      }

      if (direction_agreed) {
        if (count_dir_agreed > 10){
          _state_ = DIR_AGREED;
          // set the gate, which I need to pass in the desired direction
          gateIDinDesiredDirection = selectGateInDirection(_navigation_direction_, perception.obstacles);
          std::cout << "Selected direction: " << directionToString(_navigation_direction_) << std::endl;

          // helper vars for the st2 state
          st2_wasNearDesiredGate = false;
          st2_passedDesiredGate = false;
          st2_newGateCounter = 0;
        }
        count_dir_agreed++;
      } else {
        _navigation_direction_ = my_navigation_direction_;
        std::cout << "Not selected, but my direction: " << directionToString(my_navigation_direction_) << std::endl;
      }
      action_handlers.shareVariables(AGREEING_ON_DIRECTION, directionToInt(_navigation_direction_), my_position_in_swarm);
      base_direction_vector = {0.0, 0.0, 0.0};
      final_vector = calcBoidsAction(perception, user_params, false);

      break;
    }
    case DIR_AGREED: {

      bool iCanMove = false;

      if (my_position_in_swarm > 1.5){
        // if someone has already position #1
        if ( (my_position_in_swarm == 2) && (
          ( (perception.neighbors[0].shared_variables.dbl > 0.9) && (perception.neighbors[0].shared_variables.dbl < 1.1) && (intToState(perception.neighbors[0].shared_variables.int1 )==GATE_PASSED)) ||
          ( (perception.neighbors[1].shared_variables.dbl > 0.9) && (perception.neighbors[1].shared_variables.dbl < 1.1) && (intToState(perception.neighbors[1].shared_variables.int1 )==GATE_PASSED)) )) 
        {
          iCanMove = true;
        }
          // if someone has already position #2
        if ( (my_position_in_swarm == 3) && (
          ( (perception.neighbors[0].shared_variables.dbl > 1.9) && (perception.neighbors[0].shared_variables.dbl < 2.1) && (intToState(perception.neighbors[0].shared_variables.int1 )==GATE_PASSED)) ||
          ( (perception.neighbors[1].shared_variables.dbl > 1.9) && (perception.neighbors[1].shared_variables.dbl < 2.1) && (intToState(perception.neighbors[1].shared_variables.int1 )==GATE_PASSED)) )) 
        {
          iCanMove = true;
        } 
      } else {
        iCanMove = true;
      } 

      if (!iCanMove){
        // // set the target vector to the center of the cell
        final_vector = moveToCellCenterWithBoids(perception, user_params, action_handlers);
        action_handlers.visualizeArrow("finalVector", final_vector, Color_t{0.0, 1.0, 0.0, 0.5});
        action_handlers.shareVariables(DIR_AGREED, directionToInt(_navigation_direction_), my_position_in_swarm);
        std::cout << "I cannot move, but direction agreed. "<< directionToString(_navigation_direction_) << std::endl;
        break;
      }

      // reset the counter for the previous state
      count_dir_agreed = 0;

      // transform the direction ID into 3D vector 
      base_direction_vector = directionToDirectionVector(_navigation_direction_, perception);

      // find closest gate ID
      int closesgateIdx = selectGateClosest(perception.obstacles);

      // if I was close to the right gate
      if (closesgateIdx == gateIDinDesiredDirection){

        auto gates = perception.obstacles.gates;
        Eigen::Vector3d G1 = gates[gateIDinDesiredDirection].first;
        Eigen::Vector3d G2 = gates[gateIDinDesiredDirection].second;

        if ((G1.norm() <= 1.1) || (G2.norm() <= 1.1) ){
          printf("Distance to closest gates: %f, %f \n", G1.norm(), G2.norm());
          st2_wasNearDesiredGate = true;
          st2_newGateCounter = 0;
        }
      } else {
        // if I passed the right gate
        if (st2_wasNearDesiredGate == true){
          if (st2_newGateCounter > 10){
            st2_passedDesiredGate = true;
            printf("Passed the desired gate! \n");
            _state_ = GATE_PASSED;
          }
          st2_newGateCounter++;
        }
      }

      // check if I am really close to the gate (like almost inside) IN DIRECTION WHERE I WANT TO GO
      bool closeToGate = closestObstacleIsGate(perception, gateIDinDesiredDirection);

      // check if I am in between this gate
      bool betweenGate = false;
      if (closeToGate){
        printf("I am close to gate\n");
        betweenGate = amIBetweenDesiredGate(perception);
      }

      if ((my_position_in_swarm > 0.5) && passingTheGate(perception)){
        final_vector = 10 * base_direction_vector;
        if (betweenGate){
          final_vector = 10 * directionToBaseDirectionVector(_navigation_direction_);
        }

      } else {

        if (betweenGate){
          printf("I am between desired gate!\n");
          final_vector = 10 * directionToBaseDirectionVector(_navigation_direction_);

        } else {
          // velocity vector for the next action calculated with Boids rules
          final_vector = calcBoidsAction(perception, user_params, closeToGate);
          
        }
      }
      action_handlers.visualizeArrow("finalVector", final_vector, Color_t{0.0, 1.0, 0.0, 0.5});
      action_handlers.shareVariables(DIR_AGREED, directionToInt(_navigation_direction_), my_position_in_swarm);
      std::cout << "Direction agreed. "<< directionToString(_navigation_direction_) << std::endl;
      break;
    }

    case GATE_PASSED: {
      st2_passedDesiredGate = false;
      st2_wasNearDesiredGate = false;

      // if I dont have a position yet
      if (my_position_in_swarm < 0.5){
        // if someone has already position #1
        if ( ( (perception.neighbors[0].shared_variables.dbl > 0.9) && (perception.neighbors[0].shared_variables.dbl < 1.1)) ||
             ( (perception.neighbors[1].shared_variables.dbl > 0.9) && (perception.neighbors[1].shared_variables.dbl < 1.1))) {
              
          // if someone has already position #2
          if ( ( (perception.neighbors[0].shared_variables.dbl > 1.9) && (perception.neighbors[0].shared_variables.dbl < 2.1)) ||
               ( (perception.neighbors[1].shared_variables.dbl > 1.9) && (perception.neighbors[1].shared_variables.dbl < 2.1))) {
                my_position_in_swarm = 3.0;
          } else {
            my_position_in_swarm = 2.0;
          }
        } else {
          my_position_in_swarm = 1.0;
        }
      } 

      // tell the others you are finished and your new desired location
      Direction_t my_navigation_direction_ = targetToDirection(perception.target_vector);
      action_handlers.shareVariables(GATE_PASSED, directionToInt(my_navigation_direction_), my_position_in_swarm);

      // check if others passed too
      std::vector<int> all_states = {};
      all_states.push_back(GATE_PASSED); // add my state
      for (const auto &n : perception.neighbors) { 
              
        int n_uav_state = n.shared_variables.int1;
        if (intToState(n_uav_state) == AGREEING_ON_DIRECTION){
          _state_ = AGREEING_ON_DIRECTION;
          st3_waitingCounter = 0;
          printf("Someone already changed the state. \n");
          break;
        }
        all_states.push_back(n_uav_state);
      }

      auto             counts                                    = countIntegers(all_states);
      [[maybe_unused]] const auto &[majority_idx, majority_freq] = getMajority(counts);

      if ((majority_freq > 2) && (intToState(majority_idx) == GATE_PASSED)){
        printf("Everyone passed the gate with index: %d, majority freq: %d\n", majority_idx, majority_freq);
        if (st3_waitingCounter > 3){
          _state_ = AGREEING_ON_DIRECTION;
          st3_waitingCounter = 0;
        }
        st3_waitingCounter++;
      }

      // // set the target vector to the center of the cell
      final_vector = moveToCellCenterWithBoids(perception, user_params, action_handlers);

      std::cout << "Gate passed. Waiting for the others. My position: " << my_position_in_swarm << std::endl;
      break;
    }

      //}
  }

  // STATE MACHINE END

  if (compute_action) {

    // | --------------- Separate from other agents --------------- |

    for (const auto &n : perception.neighbors) {
      Eigen::Vector3d n_pos  = n.position;
      double          n_dist = n_pos.norm();

      // You may want to use the weighting function you should have prepared first
      bool   weight_defined;
      double weight;
      std::tie(weight_defined, weight) = weightingFunction(n_dist, _visibility_radius_, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS);

      if (weight_defined) {
        // probably use the weight
      } else {
        // possibly use some backup weight
      }
    }

    // | ----------------- Separate from obstacles ---------------- |

    auto gates = perception.obstacles.gates;

    // You may access the gates relative to your body frame
    Eigen::Vector3d G1_p = gates[0].first;
    Eigen::Vector3d G1_n = gates[0].second;

    // Or you may iterate over them
    for (const auto &G : gates) {
      const auto G_p = G.first;
    }

    //  the closest gate:
    unsigned int closest_gate_idx = selectGateClosest(perception.obstacles);
    auto         closest_gate     = perception.obstacles.gates[closest_gate_idx];

    //  the gate for the direction:
    unsigned int gate_in_direction_idx = selectGateInDirection(UP, perception.obstacles);
    auto         gate_in_direction     = perception.obstacles.gates[gate_in_direction_idx];


    // | ------------------- sum the subvectors ------------------- |
    vec_action = vec_navigation + vec_separation;
    printVector3d(vec_action, "Action:");

    // | ------------------------ visualize ----------------------- |
    action_handlers.visualizeArrow("separation", vec_separation, Color_t{1.0, 0.0, 0.0, 0.5});
    action_handlers.visualizeArrow("navigation", vec_navigation, Color_t{0.0, 0.0, 1.0, 0.5});
  }

  action_handlers.visualizeArrow("target", perception.target_vector, Color_t{1.0, 1.0, 1.0, 0.5});
  action_handlers.visualizeArrow("action", vec_action, Color_t{0.0, 0.0, 0.0, 1.0});

  
  for (int i=0; i<2; i++){
    if (abs(final_vector[i]) > 1000){
      final_vector[i] = final_vector[i] / 1000;
    }
  }
  printf("Current vector: %f, %f, %f \n", final_vector[0], final_vector[1], final_vector[2]);

  return final_vector;
}

//}


Eigen::Vector3d Swarm::moveToCellCenterWithBoids(const Perception_t &perception, const UserParams_t &user_params,  const ActionHandlers_t &action_handlers){

  // set the target vector to the center of the cell
  auto gates = perception.obstacles.gates;
  Eigen::Vector3d centerLeftSide = (gates[2].first + gates[2].second)/2;
  Eigen::Vector3d centerRightSide = (gates[0].first + gates[0].second)/2;
  Eigen::Vector3d centerOfCell = (centerLeftSide + centerRightSide)/2;

  base_direction_vector = centerOfCell / 5;
  action_handlers.visualizeArrow("centerOfCell", centerOfCell, Color_t{1.0, 0.0, 0.0, 0.5});

  // velocity vector for the next action calculated with Boids rules
  Eigen::Vector3d final_vector = calcBoidsAction(perception, user_params, false);

  return final_vector;

}


bool Swarm::passingTheGate(const Perception_t &perception){
  bool iAmBetween = false;

  auto gates = perception.obstacles.gates;
  Eigen::Vector3d g1 = gates[gateIDinDesiredDirection].first;
  Eigen::Vector3d g2 = gates[gateIDinDesiredDirection].second;

  if ( (g1.norm() < 2) && (g2.norm() < 2) ) {
    iAmBetween = true;
  }
  return iAmBetween;
}


bool Swarm::amIBetweenDesiredGate(const Perception_t &perception){
  bool iAmBetween = false;

  auto gates = perception.obstacles.gates;
  Eigen::Vector3d g1 = gates[gateIDinDesiredDirection].first;
  Eigen::Vector3d g2 = gates[gateIDinDesiredDirection].second;

  if (abs(g1.norm() - g2.norm()) < 0.4){
    iAmBetween = true;
  }
  return iAmBetween;
}

Eigen::Vector3d Swarm::calcBoidsAction(const Perception_t &perception, const UserParams_t &user_params, const bool &closeToGate){

  Eigen::Vector3d cohesion    = user_params.param1 * calcCohesion(perception, user_params);
  Eigen::Vector3d separation  = user_params.param2 * calcSeparation(perception, user_params);
  Eigen::Vector3d avoidance   = user_params.param3 * calcAvoidance(perception, user_params, closeToGate);
  Eigen::Vector3d attraction  = user_params.param4 * calcAttraction(perception, user_params);

  Eigen::Vector3d final_vector = cohesion + separation + avoidance + attraction;
    
  return final_vector;
}


bool Swarm::closestObstacleIsGate(const Perception_t &perception, const int gateId){
  auto gates = perception.obstacles.gates;
  bool closeToGate = false;

  //  the closest gate:
  unsigned int closest_gate_idx = selectGateClosest(perception.obstacles);
  auto         closest_gate     = perception.obstacles.gates[closest_gate_idx];

  if (gateId == closest_gate_idx){
    // the gates relative to your body frame
    Eigen::Vector3d G_p = closest_gate.first;
    Eigen::Vector3d G_n = closest_gate.second;

    Eigen::Vector3d closestObstacle = perception.obstacles.closest;

    double THR = 0.1;

    if ( ((abs(closestObstacle[0] - G_p[0]) < THR ) || (abs(closestObstacle[0] - G_p[0]) < THR ))
      && ((abs(closestObstacle[1] - G_p[1]) < THR ) || (abs(closestObstacle[1] - G_p[1]) < THR )) )
    {
      // we are close to this gate, turn off avoidance or minimize it
      closeToGate = true;
    }
  }
  return closeToGate;
}

Eigen::Vector3d Swarm::directionToBaseDirectionVector(const Direction_t &direction){
  Eigen::Vector3d dir_vector = {0.0, 0.0, 0.0};
  switch (direction) {

    case UP: {
      dir_vector = {0.0, 1.0, 0.0};
      break;
    }

    case DOWN: {
      dir_vector = {0.0, -1.0, 0.0};


      break;
    }
    case LEFT: {
      dir_vector = {-1.0, 0.0, 0.0};


      break;
    }
    case RIGHT: {
      dir_vector = {1.0, 0.0, 0.0};


      break;
    }
    case NONE: {
      std::cout << "[ERROR] selectGateInDirection() given direction=NONE. Can't determine the gate, returning G1." << std::endl;

      break;
    } 
  }
  return dir_vector;
}

Eigen::Vector3d Swarm::directionToDirectionVector(const Direction_t &direction, const Perception_t &perception) {
  Eigen::Vector3d dir_vector = {0.0, 0.0, 0.0};
  int gate = selectGateInDirection(direction, perception.obstacles);

  auto gates = perception.obstacles.gates;
  Eigen::Vector3d g1 = gates[gate].first;
  Eigen::Vector3d g2 = gates[gate].second;

  dir_vector = (g1 + g2) / 2;

  return dir_vector;
}



/* weightingFunction() //{ */

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
std::tuple<bool, double> Swarm::weightingFunction(const double distance, const double visibility, const double safety_distance,
                                                  [[maybe_unused]] const double desired_distance) {

  bool valid_value = false;

  // hyperbola
  double h = visibility / (distance - safety_distance);

  if (distance <= visibility){
    valid_value = true;
  } else {
    // Over the visibility range, the function shall return 0.
    valid_value = true;
    h = 0;
  }

  // Below the lower bound (including) the function is undefined.
  if (distance <= safety_distance){
    valid_value = false;
  }

  return {valid_value, h};
}

//}

// | -- Helper methods  -- |

/* targetToDirection() //{ */

Direction_t Swarm::targetToDirection(const Eigen::Vector3d &target_vector) {

  
  double x = target_vector.x();
  double y = target_vector.y();

  double angleRadians = atan2(y, x);

  if ((angleRadians >= -M_PI/4 && angleRadians <= M_PI/4)) {
        return RIGHT;

    } else if (angleRadians >= M_PI/4 && angleRadians <= 3*M_PI/4) {
        return UP;

    } else if ((angleRadians >= 3*M_PI/4 ) ||
               ( angleRadians <= -3*M_PI/4)) {
        return LEFT;

    } else if (angleRadians >= -3*M_PI/4 && angleRadians <= -M_PI/4) {
        return DOWN;

    } else {
        std::cout << "Invalid direction\n";
    }

  return UP;
}

//}

/* robotsInIdenticalStates() //{ */

bool Swarm::robotsInIdenticalStates(const Perception_t &perception) {

  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning false if there are any neighbors, true otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return false;
  }

  return true;
}

//}

/* anyRobotInState() //{ */

bool Swarm::anyRobotInState(const Perception_t &perception, const State_t &state) {

  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning true if there are any neighbors, false otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return true;
  }

  return false;
}

//}

// | ------------ Helper methods for data handling ------------ |

/* selectGateInDirection() //{ */

/**
 * @brief Finds the index of the gate in the given direction.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[dir_idx])
 */
unsigned int Swarm::selectGateInDirection(const Direction_t &direction, const Obstacles_t &obstacles) {

  switch (direction) {

    case UP: {
      return 1;

      break;
    }

    case DOWN: {
      return 3;

      break;
    }

    case LEFT: {
      return 2;

      break;
    }

    case RIGHT: {
      return 0;

      break;
    }

    case NONE: {
      std::cout << "[ERROR] selectGateInDirection() given direction=NONE. Can't determine the gate, returning G1." << std::endl;

      break;
    }
  }

  return 0;
}

//}

/* selectGateClosest() //{ */

/**
 * @brief Finds the index of the gate closest to the agent.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[min_idx])
 */
unsigned int Swarm::selectGateClosest(const Obstacles_t &obstacles) {

  unsigned int min_idx  = 0;
  double       min_dist = obstacles.gates[0].first.norm();

  for (unsigned int i = 0; i < obstacles.gates.size(); i++) {

    const auto   G      = obstacles.gates[i];
    const double G_dist = (G.first.norm() < G.second.norm()) ? G.first.norm() : G.second.norm();

    if (G_dist < min_dist) {
      min_idx  = i;
      min_dist = G_dist;
    }
  }

  return min_idx;
}

//}

/* computeMutualDistances() //{ */

/**
 * @brief Computes the vector of mutual distances between agents
 *
 * @return vector of all mutual distances (unordered) between all agents (incl. me)
 */
std::vector<double> Swarm::computeMutualDistances(const std::vector<Neighbor_t> &neighbors) {

  // All known positions (incl. mine)
  std::vector<Eigen::Vector3d> positions = {Eigen::Vector3d::Zero()};
  for (const auto &n : neighbors) {
    positions.push_back(n.position);
  }

  // Compute all mutual distances
  std::vector<double> distances;
  for (unsigned int i = 0; i < positions.size(); i++) {
    for (unsigned int j = i + 1; j < positions.size(); j++) {
      distances.push_back((positions[j] - positions[i]).norm());
    }
  }

  return distances;
}

//}

/* integersAreUnique() //{ */

/**
 * @brief Check if integers in a vector are unique
 *
 * @return true if all the integers are unique
 */
bool Swarm::integersAreUnique(const std::vector<int> &integers) {

  const auto count_map = countIntegers(integers);

  return count_map.size() == integers.size();
}

//}

/* countIntegers() //{ */

/* Computes frequency of integers in the given array
 *
 * @return map of counts for each key in the initial list
 * */
std::map<int, int> Swarm::countIntegers(const std::vector<int> &integers) {

  std::map<int, int> count_map;

  for (const int i : integers) {
    if (count_map.find(i) == count_map.end()) {
      count_map[i] = 0;
    }
    count_map[i]++;
  }

  return count_map;
}

//}

/* getMajority() //{ */

/* Return the key and value of the first maximal element in the given idx->count map.
 *
 * @return key, count
 * */
std::tuple<int, int> Swarm::getMajority(const std::map<int, int> &integer_counts) {

  if (integer_counts.empty()) {
    return {-1, -1};
  }

  int max_idx = 0;
  int max_val = 0;

  for (auto it = integer_counts.begin(); it != integer_counts.end(); ++it) {
    if (it->second > max_val) {
      max_idx = it->first;
      max_val = it->second;
    }
  }

  return {max_idx, max_val};
}

std::tuple<int, int> Swarm::getMajority(const std::vector<int> &integers) {
  return getMajority(countIntegers(integers));
}

//}

// | --------- Helper methods for data-type conversion -------- |

/* stateToInt() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
int Swarm::stateToInt(const State_t &state) {
  return static_cast<int>(state);
}

//}

/* intToState() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
State_t Swarm::intToState(const int value) {
  return static_cast<State_t>(value);
}

//}

// | --------------- Helper methods for printing -------------- |

/* stateToString() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
std::string Swarm::stateToString(const State_t &state) {

  switch (state) {

    case INIT_STATE: {
      return "INIT_STATE";

      break;
    }

    case AGREEING_ON_DIRECTION: {
      return "AGREEING_ON_DIRECTION";

      break;
    }
    
    case DIR_AGREED: {
      return "DIR_AGREED";
      break;
    }

    case GATE_PASSED: {
      return "GATE_PASSED";
      break;
    }

    default: {
      break;
    }
  }

  return "UNKNOWN";
}

//}

}  // namespace task_03_swarm
