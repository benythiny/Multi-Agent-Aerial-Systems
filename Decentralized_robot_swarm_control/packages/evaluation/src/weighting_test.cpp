#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <student_headers/swarm.h>

#include <random>

#include <stdio.h>

//}

namespace task_03_evaluation
{

typedef struct
{

  std::vector<double> distances;
  double              visibility_range;
  /* double              max_weight; */
  double safety_distance;
  double desired_distance;

} Problem_t;

/* class WeightingTest //{ */

class WeightingTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  std::string     _version_;

  // | ----------------------- parameters ----------------------- |

  bool _problem_generate_;
  int  _test_n_parametrizations_;
  int  _test_n_distances_;

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE*       output_file_;

  std::string _problems_file_path_;
  FILE*       problems_file_;

  int _scoring_base_score_pass_;

  double _requirements_time_limit_;

  std::vector<Problem_t> _problems_;

  // | ----------------- random number generator ---------------- |

  std::default_random_engine random_engine_;

  // | ---------------------- random engine --------------------- |

  double randd(const double from, const double to);
  int    randi(const int from, const int to);

  bool validateWeights(const Problem_t& problem, const std::vector<std::tuple<bool, double>>& weights);

  bool isDoubleFinite(const double value);

  void saveResult(const bool pass, const double score, const int n_parametrizations);

  Problem_t generateProblem(const int n);
  Problem_t loadProblem(const int problem_id);

  std::string printProblem(const Problem_t& problem);
};

//}

/* onInit() //{ */

void WeightingTest::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().nsec));

  mrs_lib::ParamLoader param_loader(nh_, false, "WeightingTest");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[WeightingTest]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("file", _file_path_);

  if (_file_path_ != "") {
    _output_to_file_ = true;
  }

  param_loader.loadParam("test/n_parametrizations", _test_n_parametrizations_);
  param_loader.loadParam("test/n_distances", _test_n_distances_);

  param_loader.loadParam("test/problem/generate", _problem_generate_);

  param_loader.loadParam("problems_file_path", _problems_file_path_);
  param_loader.loadParam("scoring/base_score_pass", _scoring_base_score_pass_);

  param_loader.loadParam("requirements/time_limit", _requirements_time_limit_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ParamServer]: Could not load all parameters!");
    ros::shutdown();
    exit(-1);
  }

  ros::WallTime start_time = ros::WallTime::now();
  if (_problem_generate_) {

    problems_file_ = fopen(_problems_file_path_.c_str(), "w+");

  } else {

    int n_parametrizations;
    int n_distances;

    param_loader.loadParam("problems/n_parametrizations", n_parametrizations);
    param_loader.loadParam("problems/n_distances", n_distances);

    for (int i = 0; i < n_parametrizations; i++) {

      Problem_t problem;

      std::stringstream ss_param_name;
      ss_param_name << "problems/problem_" << i + 1;

      param_loader.loadParam(ss_param_name.str() + "/visibility_range", problem.visibility_range);
      /* param_loader.loadParam(ss_param_name.str() + "/max_weight", problem.max_weight); */
      param_loader.loadParam(ss_param_name.str() + "/safety_distance", problem.safety_distance);
      param_loader.loadParam(ss_param_name.str() + "/desired_distance", problem.desired_distance);
      param_loader.loadParam(ss_param_name.str() + "/distances", problem.distances);

      _problems_.push_back(problem);
    }
  }

  /* swarm->init(problem.visibility_range); */

  if (_problem_generate_) {
    fprintf(problems_file_, "problems:\n\n");
    fprintf(problems_file_, "  n_parametrizations: %d\n", _test_n_parametrizations_);
    fprintf(problems_file_, "  n_distances: %d\n\n", _test_n_distances_);
  }

  int  worked_for                = 0;
  bool failed                    = false;
  bool computation_time_exceeded = false;

  for (int i = 0; i < _test_n_parametrizations_; i++) {

    std::unique_ptr<task_03_swarm::Swarm> swarm = std::make_unique<task_03_swarm::Swarm>();
    Problem_t                             problem;

    if (_problem_generate_) {

      ROS_INFO("[WeightingTest]: generating problem %d/%d", i + 1, _test_n_parametrizations_);

      problem = generateProblem(_test_n_distances_);

      fprintf(problems_file_, "  problem_%d:\n", i + 1);

      fprintf(problems_file_, "    visibility_range: %.5f\n", problem.visibility_range);
      /* fprintf(problems_file_, "    max_weight: %.2f\n", problem.max_weight); */
      fprintf(problems_file_, "    safety_distance: %.5f\n", problem.safety_distance);
      fprintf(problems_file_, "    desired_distance: %.2f\n", problem.desired_distance);

      fprintf(problems_file_, "    distances: [\n");
      fprintf(problems_file_, "      ");
      for (size_t j = 0; j < problem.distances.size(); j++) {
        fprintf(problems_file_, "%.5f", problem.distances[j]);
        if (j != problem.distances.size() - 1) {
          fprintf(problems_file_, ", ");
        }
      }
      fprintf(problems_file_, "\n    ]\n\n");

    } else {

      /* ROS_INFO("[WeightingTest]: loading problem %d/%d", i + 1, _test_n_parametrizations_); */
      problem = loadProblem(i);
    }

    ROS_INFO("[WeightingTest]: testing problem %d/%d", i + 1, _test_n_parametrizations_);

    std::vector<std::tuple<bool, double>> weights;
    weights.reserve(problem.distances.size());

    for (const double dst : problem.distances) {
      weights.push_back(swarm->weightingFunction(dst, problem.visibility_range, problem.safety_distance, problem.desired_distance));
    }

    if (!validateWeights(problem, weights)) {

      /* for (int i = 0; i < problem.distances.size(); i++) { */
      /*   ROS_ERROR("dist: %.2f, weight: %0.2f", problem.distances[i], */
      /*             swarm->weightingFunction(problem.distances[i], problem.visibility_range, problem.safety_distance, problem.desired_distance)); */
      /* } */

      failed = true;
      ROS_ERROR("[WeightingTest]: the weightingFunction() failed the validation check");
      ROS_ERROR_STREAM("[WeightingTest]: failed on the problem: " << printProblem(problem));
      break;
    }

    if ((ros::WallTime::now() - start_time).toSec() > _requirements_time_limit_) {
      computation_time_exceeded = true;
      ROS_ERROR("[WeightingTest]: maximum time of computation (%.2f s) exceeded", _requirements_time_limit_);
      break;
    }

    worked_for++;
  }

  const bool pass  = !failed && !computation_time_exceeded;
  const int  score = pass ? _scoring_base_score_pass_ : 0;

  saveResult(pass, score, _test_n_parametrizations_);

  if (_problem_generate_) {
    fclose(problems_file_);
  }

  /* ROS_INFO("[WeightingTest]: "); */
  ROS_INFO("[WeightingTest]: finished");
  ROS_INFO("[WeightingTest]: Algorithm worked for %d parametrizations, score = %d/%d, time of computation = %.2f s.", worked_for, score,
           _scoring_base_score_pass_, (ros::WallTime::now() - start_time).toSec());

  ros::shutdown();
}

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double WeightingTest::randd(const double from, const double to) {

  const double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int WeightingTest::randi(const int from, const int to) {

  const double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/*//{ validateWeights() */
bool WeightingTest::validateWeights(const Problem_t& problem, const std::vector<std::tuple<bool, double>>& weights) {

  if (problem.distances.size() != weights.size()) {
    ROS_ERROR("[WeightingTest]: the validateWeights() has non-matching-size inputs");
    return false;
  }

  unsigned int max_idx = 0;
  bool         crossed_lower_bound = false;

  for (unsigned int i = 0; i < weights.size(); i++) {

    const double d             = problem.distances[i];
    const auto& [w_defined, w] = weights[i];

    // Should be undefined under the lower bound
    if (w_defined) {

      if (d <= problem.safety_distance) {
        /* ROS_ERROR("[WeightingTest]: failed because of: d <= problem.safety_distance (w_defined == true)"); */
        /* ROS_ERROR("   d = %.10f", d); */
        /* ROS_ERROR("   safety distance = %.10f", problem.safety_distance); */
        return false;
      }

      // Should be defined over the lower bound
    } else {

      if (d > problem.safety_distance) {
        /* ROS_ERROR("[WeightingTest]: failed because of: d > problem.safety_distance (w_defined == false)"); */
        /* ROS_ERROR("   d = %.10f", d); */
        /* ROS_ERROR("   safety distance = %.10f", problem.safety_distance); */
        return false;
      }

      continue;
    }

    // Should not be NaN
    if (!isDoubleFinite(w)) {
      /* ROS_ERROR("[WeightingTest]: failed because w is not finite"); */
      /* ROS_ERROR("   w = %.10f", w); */
      return false;
    }

    // Should retrieve 0 over visibility range
    if (d > problem.visibility_range && std::fabs(w) > 1e-4) {
      /* ROS_ERROR("[WeightingTest]: failed because d > visibility and fabs(w) > 1e-4"); */
      /* ROS_ERROR("   d = %.10f", d); */
      /* ROS_ERROR("   w = %.10f", w); */
      return false;
    }

    // Should be non-negative
    if (w < 0.0) {
      /* ROS_ERROR("[WeightingTest]: failed because w < 0.0"); */
      /* ROS_ERROR("   w = %.10f", w); */
      return false;
    }

    // Should be non-increasing
    if (i > 0) {

      const auto& [w_defined_prev, w_prev] = weights[i - 1];

      if (w_defined_prev && w > w_prev) {
        /* ROS_ERROR("[WeightingTest]: failed because w > w_prev"); */
        /* ROS_ERROR("   w = %.10f", w); */
        /* ROS_ERROR("   w_prev = %.10f", w_prev); */
        return false;
      }
    }

    // The first element after the safety distance should be maximum (checked by non-increasing check)
    if (!crossed_lower_bound && w > problem.safety_distance) {
      crossed_lower_bound = true;
      max_idx             = i;
    }
  }

  // Validate if the max value "limits to infinity" as it is approaching the safety distance
  if (crossed_lower_bound) {
    const auto& [max_w_defined, max_w] = weights[max_idx];

    // A bit sketchy evaluation, but serves the purpose
    if (max_w < 10.0 * problem.safety_distance) {
      /* ROS_ERROR("[WeightingTest]: failed because max_w < 10*lower bound"); */
      /* ROS_ERROR("   max_w = %.10f", max_w); */
      /* ROS_ERROR("   safety distance = %.10f", problem.safety_distance); */
      return false;
    }

  } else {
    ROS_ERROR("[WeightingTest]: Something is wrong with the input data, distances have not crossed the lower bound.");
    return false;
  }

  return true;
}
/*//}*/

/* isDoubleFinite() //{ */
bool WeightingTest::isDoubleFinite(const double value) {
  return std::isfinite(value);
}
//}

/* saveResult() //{ */

void WeightingTest::saveResult(const bool pass, const double score, const int n_parametrizations) {

  if (_output_to_file_) {
    output_file_ = fopen(_file_path_.c_str(), "w+");
    fprintf(output_file_, "%d %.2f %d RuntimeOk", pass, score, n_parametrizations);
    fclose(output_file_);
  }
}

//}

/* generateProblem() //{ */

Problem_t WeightingTest::generateProblem(const int n) {

  Problem_t problem;

  problem.visibility_range = randd(5.0, 20.0);
  /* problem.max_weight       = randd(10.0, 100.0); */
  problem.safety_distance  = randd(0.5, 2.5);
  problem.desired_distance = randd(3.0, 4.0);

  problem.distances.reserve(n);

  const int N_to_lower_bound = int(0.05 * double(n));
  double    d_init           = -0.5;
  double    step             = (problem.safety_distance - d_init) / double(N_to_lower_bound - 1);

  // Add from d_init to safety_distance (incl.) (5% of elements)
  for (int i = 0; i < N_to_lower_bound; i++) {
    problem.distances.push_back(d_init + double(i) * step);
  }

  // Add safety_distance + some small values (5 elements)
  int N_at_lower_bound = problem.distances.size();
  problem.distances.push_back(problem.safety_distance + 1e-5);
  problem.distances.push_back(problem.safety_distance + 1e-4);
  problem.distances.push_back(problem.safety_distance + 1e-3);
  problem.distances.push_back(problem.safety_distance + 1e-2);
  problem.distances.push_back(problem.safety_distance + 1e-1);
  N_at_lower_bound = int(problem.distances.size()) - N_at_lower_bound;

  /* Add the rest up to n elements */
  const int I = n - (N_to_lower_bound + N_at_lower_bound);
  d_init      = problem.distances.back() + 0.1;
  step        = (problem.visibility_range + 0.5 - d_init) / double(I);
  for (int i = 0; i < I; i++) {
    problem.distances.push_back(d_init + double(i) * step);
  }

  return problem;
}

//}

/* loadProblem() //{ */

Problem_t WeightingTest::loadProblem(const int problem_id) {

  return _problems_[problem_id];
}

//}

/* printProblem() //{ */

std::string WeightingTest::printProblem(const Problem_t& problem) {

  std::stringstream ss;

  ss << std::endl;
  ss << "safety_distance: " << problem.safety_distance << std::endl;
  /* ss << "max_weight: " << problem.max_weight << std::endl; */
  ss << "visibility_range: " << problem.visibility_range << std::endl;
  ss << "desired_distance: " << problem.desired_distance << std::endl;

  return ss.str();
}

//}

}  // namespace task_03_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_evaluation::WeightingTest, nodelet::Nodelet)

