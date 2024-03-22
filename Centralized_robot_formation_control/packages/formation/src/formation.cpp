#include <student_headers/formation.h>

namespace task_02_formation
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 */
void Formation::init() {

}

//}

/* getPathsReshapeFormation() //{ */

/**
 * @brief This method calculates paths for each UAV from an initial state towards a desired final state.
 * This method is supposed to be filled in by the student.
 *
 * @param initial_states A vector of 3D initial positions for each UAV.
 * @param final_states A vector of 3D final positions of each UAV.
 *
 * @return A vector of paths, each path being a vector of 3D positions for each UAV. The initial and final states are supposed
 * to be the part of the path for each UAV. The expected result for point I, as the starting point for a UAV and point F as the final
 * point for a UAV, can be, e.g.:
 *   I -> F
 *   I -> B -> F
 *   I -> B -> C -> F
 * The following paths are considered invalid:
 *   I
 *   F
 *   D -> D
 *   I -> D
 *   F -> I
 */
std::vector<std::vector<Eigen::Vector3d>> Formation::getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                              const std::vector<Eigen::Vector3d> &final_states) {

  // use the visualizeCube() method
  // * very useful for showing the obstacle set for the Astar
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)

  const double resolution = 0.6; // #TODO: maybe change the resolution
  astar::Astar astar(resolution);

  std::set<astar::Cell> obstacles;

  action_handlers_.visualizeCube(Position_t{0, 0, 0}, Color_t{0.0, 0.0, 1.0, 0.1}, 1.0);

  // how many UAVs do we have
  int n_uavs = initial_states.size();

  // initialize the vector of paths
  std::vector<std::vector<Eigen::Vector3d>> paths;

  // for each UAV insert their initial and final positions
  double supersampling_factor = 2.5;
  int box_size = ceil(1.0*supersampling_factor);

  // for each UAV
  for (int i = 0; i < n_uavs; i++) {
    std::set<astar::Cell> obstacles_local;
    for (std::set<astar::Cell>::iterator it = obstacles.begin(); it != obstacles.end(); it++){
      //astar::Position pos = astar.fromGrid(*it);
      obstacles_local.insert(*it);
    }
    // for each UAV insert their initial and final positions except for the current uav
    for (int uav = 0; uav < n_uavs; uav++) {
      if ((uav != i) & (initial_states[uav] != final_states[i] ) & (initial_states[i] != final_states[uav] )){
      
        for (int idx_x = -box_size; idx_x <= box_size; idx_x++){
          for (int idx_z = -box_size; idx_z <= box_size; idx_z++){
            for (int idx_y = -box_size; idx_y <= box_size; idx_y++){

              double x = initial_states[uav][0] + idx_x/supersampling_factor;
              double y = initial_states[uav][1] + idx_y/supersampling_factor;
              double z = initial_states[uav][2] + idx_z/supersampling_factor;

              obstacles_local.insert(astar.toGrid(x, y, z));

              x = final_states[uav][0] + idx_x/supersampling_factor;
              y = final_states[uav][1] + idx_y/supersampling_factor;
              z = final_states[uav][2] + idx_z/supersampling_factor;

              obstacles_local.insert(astar.toGrid(x, y, z));
              
            }
          }
        }
      }
    }

    // prepare the path
    std::vector<Eigen::Vector3d> path;


    astar::Position start(initial_states[i][0], initial_states[i][1], initial_states[i][2]);
    astar::Position goal(final_states[i][0], final_states[i][1], final_states[i][2]);
    std::optional<std::list<astar::Position>> result = astar.plan(start, goal, obstacles_local);

    
    path.push_back(initial_states[i]);

    if (result){

        for (astar::Position pos : result.value()) {

          path.push_back(Eigen::Vector3d(pos.x(), pos.y(), pos.z()));          
          
          for (int idx_x = -box_size; idx_x <= box_size; idx_x++){
            for (int idx_z = -box_size; idx_z <= box_size; idx_z++){
              for (int idx_y = -box_size; idx_y <= box_size; idx_y++){
                
                double x = pos.x() + idx_x/supersampling_factor;
                double y = pos.y() + idx_y/supersampling_factor;
                double z = pos.z() + idx_z/supersampling_factor;

                obstacles.insert(astar.toGrid(x, y, z));
              }
            }
          }
          
        }      
          
    }
    path.push_back(final_states[i]);

    paths.push_back(path);
  }

  return paths;
}


//}

/* multilateration() //{ */

/**
 * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
 *
 * @param uav_states Vector of 3D positions of each UAV.
 * @param distances Vector of the measured distances from each UAV to the source of signal.
 *
 * @return the estimated 3D position of the source of radiation.
 */

Eigen::Vector3d Formation::multilateration(const std::vector<Eigen::Vector3d> &positions, const Eigen::VectorXd &distances) {

  const int N = int(positions.size());

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N, 3);
  Eigen::MatrixXd g = Eigen::VectorXd::Zero(N);

  // the solution... initialized as (0, 0, 0)^T, is it a good initialization?
  //Eigen::Vector3d s = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d s = prev_good_s;
  int rand_i = rand() % N;
  //Eigen::Vector3d s = positions[rand_i];
  //printf("initial positions for optimization: x: %f, y: %f, z: %f\n");

  const int max_iterations = 200;
  double lambda = 0.01; // Initial damping factor

  for (int n_iterations = 0; n_iterations < max_iterations; n_iterations++) {

    for (int j = 0; j < N; j++) {

      J.row(j) = (s - positions[j]) / (s - positions[j]).norm();
    }

    // distance from xk to the sphere with radius distances[i] and center positions[i]
    for (int i = 0; i < N; i++) {
      g(i) = (s - positions[i]).norm() - distances[i];
    }

    // do the Gauss-Newton iteration
    s = s - (J.transpose() * J).inverse() * J.transpose() * g;
    s[2] = 1;
  }


  if (std::isfinite(s[0])){
    //printf("Found s: %f,   %f,   %f\n", s[0], s[1], s[2]);
  }

  if ((s[0] > 90) || (s[0] < -90) || (s[1] > 90) || (s[1] < -90)){
    
    s[0] = 0.0/0.0;
    s[1] = 0.0/0.0;
  } else if (std::isfinite(s[0]) && std::isfinite(s[1])) {
    //prev_good_s = s;
  }
  
  return s;
}

//}


/* update() //{ */

/**
 * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz, and,
 * therefore, it should not be blocking.
 *
 * @param formation_state The current state of the formation. The state contains:
 * - absolute position of the virtual leader
 * - positions of the follower UAVs relative the virtual leader
 * - flag stating whether the formation is moving or whether it is stationary
 * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
 * @param time_stamp Current time in seconds.
 * @param action_handlers This structure provides users with functions to control the formation:
 *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
 *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
 * Moreover, the action_handlers structure provides additional methods for data visualization.
 */
void Formation::update(const FormationState_t &formation_state, const Ranging_t &ranging, [[maybe_unused]] const double &time_stamp,
                       ActionHandlers_t &action_handlers) {

  // how many UAVs are there in the formation?
  const int n_uavs = int(formation_state.followers.size());

  // | ------------- calculate the target's position ------------ |

  // calculate the abolsute positions of the formation members
  std::vector<Eigen::Vector3d> abs_positions;

  for (int i = 0; i < n_uavs; i++) {
    abs_positions.push_back(formation_state.followers[i] + formation_state.virtual_leader);
  }

  Eigen::Vector3d target_position = multilateration(abs_positions, ranging.distances);
  

  // | --------------- Publishing CUBE Rviz marker -------------- |
  // * you can use this function repeatedly with different names to visualize other stuff
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  //action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);
  if (std::isfinite(target_position[0]) & std::isfinite(target_position[1]) & std::isfinite(target_position[2])) {
   action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);
   //printf("Target position was detected at coords: %f, %f\n", target_position[0], target_position[1]);
  } else {
    //printf("NaNs detected in target vis\n");
  }

    if (std::isfinite(target_position[0]) & std::isfinite(target_position[1]) & std::isfinite(target_position[2])) {
        if (mean_iters == 0){
            estim_s_start[0] = 0;
            estim_s_start[1] = 0;
            estim_s_start[2] = 0;
        }

        if ((target_position[0] < 90) && (target_position[0] > -90) && (target_position[1] < 90) && (target_position[1] > -90)){
            estim_s_start[0] += target_position[0];
            estim_s_start[1] += target_position[1];
            estim_s_start[2] += target_position[2];
            mean_iters ++;
            
        }
    }
    if (mean_iters > 10){
        estim_s_start[0] = estim_s_start[0] / 11;
        estim_s_start[1] = estim_s_start[1] / 11;
        estim_s_start[2] = 1;
        target_position = estim_s_start;
        printf("Target = estim pose! %f,   %f,   %f\n", estim_s_start[0],estim_s_start[1], estim_s_start[2] );
        printf("-----");
        mean_iters = 0;
        prev_good_s = estim_s_start;
    }


  // do nothing while the formation is in motion
  if (!formation_state.is_static) {
    return;
  }
  
  double current_x = formation_state.virtual_leader[0];
  double current_y = formation_state.virtual_leader[1];
  double current_z = formation_state.virtual_leader[2];
  if (std::isfinite(current_x) && std::isfinite(current_y) && std::isfinite(current_z)) {
   action_handlers.visualizeCube(Position_t{current_x, current_y, current_z}, Color_t{0.0, 1.0, 1.0, 1.0}, 1.0);
  }
 
  switch (user_defined_variable_) {

    case 0: {

    if (std::isfinite(current_x) && std::isfinite(current_y)) {
        
        double x_diff = current_x - prev_good_s[0] ;
        double y_diff = current_y - prev_good_s[1] ;

        if (abs(x_diff) > abs(y_diff)) {
            current_cell_x = current_cell_x - floor(x_diff/10);
            user_defined_variable_ = 1;
        } else {
            current_cell_y = current_cell_y - floor(y_diff/10);
            user_defined_variable_ = 2;
        }

        printf("Moving to cell x:  %d, ____________ y: %d, \n", current_cell_x, current_cell_y);
        
    }

      break;

    }

    // in the fist state, reorganize the formation into a column
    case 1: {
      

      std::vector<Eigen::Vector3d> formation_line;
      
      double current_x = formation_state.virtual_leader[0];
      double current_y = formation_state.virtual_leader[1];
      
      for (int uav_i=1; uav_i<=n_uavs; uav_i++){
        double x = 1.5 * (uav_i - ceil((n_uavs+1)/2)) ;
        double y = 0.0 ;
        formation_line.push_back(Eigen::Vector3d(x, y, 5.0));
        printf("Formating formation: x: %f, y: %f\n", x, y);
      }
      // plan paths to reshape the formation
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      }

      user_defined_variable_ = 3;

      break;
    }

    case 2: {
       
      double current_x = formation_state.virtual_leader[0];
      double current_y = formation_state.virtual_leader[1];
     
      std::vector<Eigen::Vector3d> formation_line;
      for (int uav_i=1; uav_i<=n_uavs; uav_i++){
        double x = 0.0 ;
        double y = 1.5 * (uav_i - ceil((n_uavs+1)/2)) ;
        formation_line.push_back(Eigen::Vector3d( x, y, 5.0));
        printf("Formating formation: x: %f, y: %f\n", x, y);
      }
      // plan paths to reshape the formation
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        //user_defined_variable_ = 0;
        return;
      }

      user_defined_variable_ = 3;

      break;
    }

    case 3:{
      printf("Previous s: %f,  %f,  %f\n", prev_good_s[0], prev_good_s[1], prev_good_s[2]);
      // tell the virtual leader to move to the next "cylinder"
      if (current_cell_x > 9){
        current_cell_x = 9;
      }
      if (current_cell_x < -9){
        current_cell_x = -9;
      }
      if (current_cell_y > 9){
        current_cell_y = 9;
      }
      if (current_cell_y < -9){
        current_cell_y = -9;
      }
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(10.0 * current_cell_x, 10.0 * current_cell_y,  2));
      user_defined_variable_ = 0;
      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      }
      break;
    }

    case 4:{
      std::vector<Eigen::Vector3d> formation_line;
      for (int uav_i=1; uav_i<=n_uavs; uav_i++){
        
        formation_line.push_back(Eigen::Vector3d( 0, 0, 1.0 + 2 * uav_i));
      }
      // plan paths to reshape the formation
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      } else {

        user_defined_variable_ = 0;
        beginning = false;
      }

      break;
    }

    
    case 5: {
        if (std::isfinite(target_position[0]) & std::isfinite(target_position[1]) & std::isfinite(target_position[2])) {
            if ((target_position[0] > 90) || (target_position[0] < -90) || (target_position[1] > 90) || (target_position[1] < -90)){
                estim_s_start[0] += target_position[0];
                estim_s_start[1] += target_position[1];
                estim_s_start[2] += target_position[2];
                mean_iters ++;
            }
        }
        if (mean_iters > 10){
            estim_s_start[0] = estim_s_start[0] / 11;
            estim_s_start[1] = estim_s_start[1] / 11;
            estim_s_start[2] = estim_s_start[2] / 11;
            user_defined_variable_ = 0;
        }
    }

    default: {

      
      break;
    }
  }
}

//}

}  // namespace task_02_formation
