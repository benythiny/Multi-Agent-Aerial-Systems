#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <student_headers/formation.h>

#include <random>

#include <stdio.h>

//}

namespace task_02_evaluation
{

/* class ReshapingTest //{ */

class ReshapingTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  // | ----------------------- parameters ----------------------- |

  int    _test_n_uavs_;
  int    _test_n_problems_;
  double _problem_min_dist_points_;
  bool   _problem_generate_;

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE*       output_file_;

  std::string _problems_file_path_;
  FILE*       problems_file_;

  double _scoring_base_score_pass_;
  double _scoring_score_per_uav_;

  double _requirements_min_uav_dist_;
  double _requirements_time_limit_;

  std::vector<std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>>> _problems_;

  // | ------------------ student's controller ------------------ |

  std::unique_ptr<task_02_formation::Formation> formation_;

  // | --------------------- action handlers -------------------- |

  task_02_formation::ActionHandlers_t action_handlers_;

  void visualizePose(const double x, const double y, const double z, const double heading);
  void plotValue(const std::string name, const double value);

  // | ----------------- random number generator ---------------- |

  std::default_random_engine random_engine_;

  // | ---------------------- random engine --------------------- |

  double randd(const double from, const double to);
  int    randi(const int from, const int to);

  bool reshapeFormation(const std::vector<std::vector<Eigen::Vector3d>>& paths);
  bool setLeaderPosition(const Eigen::Vector3d& position);
  void visualizeCube(const task_02_formation::Position_t& position, const task_02_formation::Color_t& color, const double& size);

  bool isVectorFinite(const Eigen::VectorXd& vector, const std::string name);
  bool isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name);

  void runtimeError();
  void saveResult(const bool& pass, const double& score, const int& n_uavs);

  std::vector<Eigen::Vector3d> samplePath(const std::vector<Eigen::Vector3d>& path, const double& sample_distance);

  bool trajectoriesSafe(const std::vector<Eigen::Vector3d>& traj1, const std::vector<Eigen::Vector3d>& traj2, const double& threshold);

  std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> generateProblem(const int& n_uavs, const double& min_dist_points);

  std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> loadProblem(const int& uav_id, const int& problem_id);

  std::string printPath(const std::vector<Eigen::Vector3d>& path1, const std::string& name);

  std::string printProblem(const std::vector<Eigen::Vector3d>& from, const std::vector<Eigen::Vector3d>& to);

  bool validatePaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<Eigen::Vector3d>& starts,
                     const std::vector<Eigen::Vector3d>& ends);
};

//}

/* onInit() //{ */

void ReshapingTest::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().nsec));

  mrs_lib::ParamLoader param_loader(nh_, false, "ReshapingTest");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[ReshapingTest]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("file", _file_path_);

  if (_file_path_ != "") {
    _output_to_file_ = true;
  }

  param_loader.loadParam("requirements/min_uav_distance", _requirements_min_uav_dist_);
  param_loader.loadParam("requirements/time_limit", _requirements_time_limit_);
  param_loader.loadParam("test/n_uavs", _test_n_uavs_);
  param_loader.loadParam("test/n_problems", _test_n_problems_);
  param_loader.loadParam("test/problem/min_dist_points", _problem_min_dist_points_);
  param_loader.loadParam("test/problem/generate", _problem_generate_);
  param_loader.loadParam("problems_file_path", _problems_file_path_);
  param_loader.loadParam("scoring/base_score_pass", _scoring_base_score_pass_);
  param_loader.loadParam("scoring/score_per_uav", _scoring_score_per_uav_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ParamServer]: Could not load all parameters!");
    ros::shutdown();
  }

  ros::WallTime start_time = ros::WallTime::now();
  if (_problem_generate_) {
    problems_file_ = fopen(_problems_file_path_.c_str(), "w+");
  } else {

    int n_problems;
    int n_uavs;

    param_loader.loadParam("problems/n_problems", n_problems);
    param_loader.loadParam("problems/n_uavs", n_uavs);

    for (int i = 2; i <= n_uavs; i++) {

      std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> problems;

      for (int j = 0; j < n_problems; j++) {

        Eigen::MatrixXd from_matrix = Eigen::MatrixXd::Zero(i, 3);
        Eigen::MatrixXd to_matrix   = Eigen::MatrixXd::Zero(i, 3);

        std::stringstream ss_param_name;
        ss_param_name << "problems/" << i << "_uavs/problem_" << j + 1;

        param_loader.loadMatrixStatic(ss_param_name.str() + "/from", from_matrix, i, 3);
        param_loader.loadMatrixStatic(ss_param_name.str() + "/to", to_matrix, i, 3);

        std::vector<Eigen::Vector3d> from_vec;
        std::vector<Eigen::Vector3d> to_vec;

        for (int row = 0; row < i; row++) {
          from_vec.push_back(from_matrix.row(row));
          to_vec.push_back(to_matrix.row(row));
        }

        problems.push_back({from_vec, to_vec});
      }

      _problems_.push_back(problems);
    }
  }

  action_handlers_.reshapeFormation  = std::bind(&ReshapingTest::reshapeFormation, this, std::placeholders::_1);
  action_handlers_.setLeaderPosition = std::bind(&ReshapingTest::setLeaderPosition, this, std::placeholders::_1);
  action_handlers_.visualizeCube     = std::bind(&ReshapingTest::visualizeCube, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  formation_ = std::make_unique<task_02_formation::Formation>();

  formation_->setActionHandlers(action_handlers_);

  formation_->init();

  if (_problem_generate_) {
    fprintf(problems_file_, "problems:\n\n");
    fprintf(problems_file_, "  n_problems: %d\n", _test_n_problems_);
    fprintf(problems_file_, "  n_uavs: %d\n", _test_n_uavs_);
  }

  int worked_for = 0;

  // for each formation cardinality
  for (int n_uavs = 2; n_uavs <= _test_n_uavs_; n_uavs++) {

    if (_problem_generate_) {
      fprintf(problems_file_, "  %d_uavs:\n\n", n_uavs);
    }

    bool failed                    = 0;
    bool computation_time_exceeded = 0;

    // for each problem
    for (int i = 0; i < _test_n_problems_; i++) {

      ROS_INFO("[ReshapingTest]: testing for %d UAVs, problem %d/%d", n_uavs, i + 1, _test_n_problems_);

      std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> problem;

      if (_problem_generate_) {

        problem = generateProblem(n_uavs, _problem_min_dist_points_);

        fprintf(problems_file_, "    problem_%d:\n", i + 1);

        fprintf(problems_file_, "      from: [\n");
        for (size_t j = 0; j < problem.first.size(); j++) {
          fprintf(problems_file_, "        %.2f, %.2f, %.2f,\n", problem.first[j][0], problem.first[j][1], problem.first[j][2]);
        }
        fprintf(problems_file_, "      ]\n");

        fprintf(problems_file_, "      to: [\n");
        for (size_t j = 0; j < problem.first.size(); j++) {
          fprintf(problems_file_, "        %.2f, %.2f, %.2f,\n", problem.second[j][0], problem.second[j][1], problem.second[j][2]);
        }
        fprintf(problems_file_, "      ]\n");

        fprintf(problems_file_, "\n\n");

      } else {
        problem = loadProblem(n_uavs, i);
      }

      std::vector<std::vector<Eigen::Vector3d>> paths = formation_->getPathsReshapeFormation(problem.first, problem.second);

      if (!validatePaths(paths, problem.first, problem.second)) {
        failed = true;
        ROS_ERROR("[ReshapingTest]: the path failed the validation check");
        ROS_ERROR_STREAM("[ReshapingTest]: failed on the problem: " << printProblem(problem.first, problem.second));
        break;
      }

      if ((ros::WallTime::now() - start_time).toSec() > _requirements_time_limit_) {
        computation_time_exceeded = true;
        break;
      }

      std::vector<std::vector<Eigen::Vector3d>> trajectories;

      for (int j = 0; j < n_uavs; j++) {
        trajectories.push_back(samplePath(paths[j], 0.1));
      }

      for (int a = 0; a < n_uavs; a++) {
        for (int b = a + 1; b < n_uavs; b++) {
          if (!trajectoriesSafe(trajectories[a], trajectories[b], _requirements_min_uav_dist_)) {
            failed = true;
            ROS_ERROR("[ReshapingTest]: the check failed for path #%d and #%d", a + 1, b + 1);
            ROS_ERROR_STREAM("[ReshapingTest]: failed on the problem: " << printProblem(problem.first, problem.second));
            break;
          }
        }
        if (failed) {
          break;
        }
      }

      if (failed) {
        break;
      }
    }

    if (failed) {
      ROS_ERROR("[ReshapingTest]: test failed for #%d UAVs", n_uavs);
      break;
    }

    if (computation_time_exceeded) {
      ROS_ERROR("[ReshapingTest]: maximum time of computation (%.2f s) exceeded during test for #%d UAVs", _requirements_time_limit_, n_uavs);
      break;
    }

    worked_for = n_uavs;
  }

  double score = _scoring_base_score_pass_ + (worked_for - 5) * _scoring_score_per_uav_;

  saveResult(worked_for >= 5, score, worked_for);

  if (_problem_generate_) {
    fclose(problems_file_);
  }

  /* ROS_INFO("[ReshapingTest]: "); */
  ROS_INFO("[ReshapingTest]: finished");

  if (worked_for >= 5) {
    ROS_INFO("[ReshapingTest]: Pass, the algorithm worked for %d UAVs, bonus score = %.2f, time of computation = %.2f s.", worked_for, score,
             (ros::WallTime::now() - start_time).toSec());
  } else {
    ROS_WARN("[ReshapingTest]: Fail, the algorithm worked for %d UAVs, time of computation = %.2f s.", worked_for, (ros::WallTime::now() - start_time).toSec());
  }

  ros::shutdown();
}

//}

// | ---------------- action handlers callbacks --------------- |

/* reshapeFormation() //{ */

bool ReshapingTest::reshapeFormation([[maybe_unused]] const std::vector<std::vector<Eigen::Vector3d>>& paths) {
  return false;
}

//}

/* setLeaderPosition() //{ */

bool ReshapingTest::setLeaderPosition([[maybe_unused]] const Eigen::Vector3d& position) {
  return false;
}

//}

/* visualizeCube() //{ */

void ReshapingTest::visualizeCube([[maybe_unused]] const task_02_formation::Position_t& position, [[maybe_unused]] const task_02_formation::Color_t& color,
                                  [[maybe_unused]] const double& size) {
}

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double ReshapingTest::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int ReshapingTest::randi(const int from, const int to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* isVectorFinite() //{ */

bool ReshapingTest::isVectorFinite(const Eigen::VectorXd& vector, const std::string name) {

  for (int i = 0; i < vector.size(); i++) {
    if (!std::isfinite(vector[i])) {
      ROS_ERROR("[ReshapingTest]: NaN detected in \"%s[%d]\"!!!", name.c_str(), i);
      return false;
    }
  }

  return true;
}

//}

/* isMatrixFinite() //{ */

bool ReshapingTest::isMatrixFinite(const Eigen::MatrixXd& matrix, const std::string name) {

  for (int i = 0; i < matrix.cols(); i++) {
    for (int j = 0; j < matrix.rows(); j++) {
      if (!std::isfinite(matrix(i, j))) {
        ROS_ERROR("[ReshapingTest]: NaN detected in \"%s[%d, %d]\"!!!", name.c_str(), i, j);
        return false;
      }
    }
  }

  return true;
}

//}

/* runtimeError() //{ */

void ReshapingTest::runtimeError() {

  if (_output_to_file_) {

    output_file_ = fopen(_file_path_.c_str(), "w+");

    fprintf(output_file_, "0 0 0 RuntimeError");
    fclose(output_file_);
  }

  ros::shutdown();
}

//}

/* saveResult() //{ */

void ReshapingTest::saveResult(const bool& pass, const double& score, const int& n_uavs) {

  if (_output_to_file_) {
    output_file_ = fopen(_file_path_.c_str(), "w+");
    fprintf(output_file_, "%d %.2f %d RuntimeOk", pass, score, n_uavs);
    fclose(output_file_);
  }
}

//}

/* samplePath() //{ */

std::vector<Eigen::Vector3d> ReshapingTest::samplePath(const std::vector<Eigen::Vector3d>& path, const double& sample_distance = 0.1) {

  std::vector<Eigen::Vector3d> trajectory;

  for (size_t i = 0; i < path.size() - 1; i++) {

    Eigen::Vector3d v1 = path[i];
    Eigen::Vector3d v2 = path[i + 1];

    Eigen::Vector3d direction = v2 - v1;
    double          distance  = direction.norm();

    int n_samples;

    if (distance <= sample_distance) {
      n_samples = 1;
    } else {
      n_samples = floor(distance / sample_distance);
    }

    Eigen::Vector3d vec = direction / ((double)n_samples);

    for (int j = 0; j <= n_samples; j++) {

      Eigen::Vector3d point = v1 + ((double)j) * vec;

      trajectory.push_back(point);
    }
  }

  return trajectory;
}

//}

/* validatePaths() //{ */

bool ReshapingTest::validatePaths(const std::vector<std::vector<Eigen::Vector3d>>& paths, const std::vector<Eigen::Vector3d>& start_in,
                                  const std::vector<Eigen::Vector3d>& ends_in) {

  std::vector<Eigen::Vector3d> ends = ends_in;

  bool success = true;

  if (paths.size() != start_in.size()) {
    ROS_ERROR("[ReshapingDebug]: the resulting vector of paths has a wrong dimension (%d != %d)", int(paths.size()), int(start_in.size()));
    return false;
  }

  // for each path
  for (size_t i = 0; i < start_in.size(); i++) {

    if (paths[i].size() <= 0) {
      ROS_ERROR("[ReshapingDebug]: the #%dth path is empty", int(i));
      success = false;
      break;
    }

    // check if the start point matches
    if ((paths[i][0] - start_in[i]).norm() > 1e-2) {
      success = false;
      ROS_ERROR("[ReshapingDebug]: the path's first point does not match the required initial condition");
      break;
    }

    // check if the end point is on the list
    int idx = -1;
    for (size_t j = 0; j < ends.size(); j++) {
      if ((paths[i][paths[i].size() - 1] - ends[j]).norm() <= 1e-2) {
        idx = j;
        break;
      }
    }

    if (idx >= 0) {
      ends.erase(ends.begin() + idx);
    } else {
      ROS_ERROR("[ReshapingDebug]: the path's end point is not within the list of final positions");
      success = false;
      break;
    }
  }

  return success;
}

//}

/* trajectoriesSafe() //{ */

bool ReshapingTest::trajectoriesSafe(const std::vector<Eigen::Vector3d>& traj1, const std::vector<Eigen::Vector3d>& traj2, const double& threshold) {

  for (size_t i = 0; i < traj1.size(); i++) {

    Eigen::Vector3d p1 = traj1[i];

    for (size_t j = 0; j < traj2.size(); j++) {

      Eigen::Vector3d p2 = traj2[j];

      // check if we are not closed to any beginning or end of the trajectory
      if ((p1 - p2).norm() < threshold) {
        return false;
      }
    }
  }

  return true;
}

//}

/* generateProblem() //{ */

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> ReshapingTest::generateProblem(const int& n_uavs, const double& min_dist_points) {

  std::vector<Eigen::Vector3d> starts;
  std::vector<Eigen::Vector3d> ends;

  // generate starts
  for (int i = 0; i < n_uavs; i++) {

    Eigen::Vector3d point;

    while (true) {

      point[0] = randd(-15, 15);
      point[1] = randd(-15, 15);
      point[2] = randd(0, 10);

      bool point_safe = true;

      for (size_t j = 0; j < starts.size(); j++) {
        if ((starts[j] - point).norm() <= min_dist_points) {
          point_safe = false;
          break;
        }
      }

      if (point_safe) {
        break;
      }
    }

    starts.push_back(point);
  }

  // generate ends
  for (int i = 0; i < n_uavs; i++) {

    Eigen::Vector3d point;

    while (true) {

      point[0] = randd(-15, 15);
      point[1] = randd(-15, 15);
      point[2] = randd(0, 15);

      bool point_safe = true;

      for (size_t j = 0; j < starts.size(); j++) {
        if ((starts[j] - point).norm() <= min_dist_points) {
          point_safe = false;
          break;
        }
      }

      for (size_t j = 0; j < ends.size(); j++) {
        if ((ends[j] - point).norm() <= min_dist_points) {
          point_safe = false;
          break;
        }
      }

      if (point_safe) {
        break;
      }
    }

    ends.push_back(point);
  }

  return {starts, ends};
}

//}

/* loadProblem() //{ */

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> ReshapingTest::loadProblem(const int& uav_id, const int& problem_id) {

  return {_problems_[uav_id - 2][problem_id].first, _problems_[uav_id - 2][problem_id].second};
}

//}

/* printPath() //{ */

std::string ReshapingTest::printPath(const std::vector<Eigen::Vector3d>& path1, const std::string& name) {

  std::stringstream ss;

  ss << name << " = [" << std::endl;

  for (Eigen::Vector3d point : path1) {
    ss << "[" << point[0] << ", " << point[1] << ", " << point[2] << "]," << std::endl;
  }

  ss << "]" << std::endl;

  return ss.str();
}

//}

/* printProblem() //{ */

std::string ReshapingTest::printProblem(const std::vector<Eigen::Vector3d>& from, const std::vector<Eigen::Vector3d>& to) {

  std::stringstream ss;

  ss << std::endl;
  ss << "initial_states: [" << std::endl;
  ss << "  # x, y, z" << std::endl;

  for (Eigen::Vector3d point : from) {
    ss << "  " << point[0] << ", " << point[1] << ", " << point[2] << "," << std::endl;
  }

  ss << "]" << std::endl;

  ss << "final_states: [" << std::endl;
  ss << "  # x, y, z" << std::endl;

  for (Eigen::Vector3d point : to) {
    ss << "  " << point[0] << ", " << point[1] << ", " << point[2] << "," << std::endl;
  }

  ss << "]" << std::endl;

  return ss.str();
}

//}

}  // namespace task_02_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_02_evaluation::ReshapingTest, nodelet::Nodelet)
