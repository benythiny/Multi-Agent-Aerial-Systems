#define VERSION "1.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <task_03_wrapper/wrapper_boids.h>

#include <numeric>
#include <stdio.h>

//}

namespace task_03_evaluation
{

/* class BoidsTest //{ */

class BoidsTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle _nh_;
  std::string     _version_;

  // | ----------------------- parameters ----------------------- |

  std::string _file_path_;
  bool        _output_to_file_ = false;
  FILE*       output_file_;

  std::tuple<int, int, std::vector<std::tuple<double, int>>> loadVariantParameters(mrs_lib::ParamLoader& param_loader, const std::string& task_variant);

  void   saveResult(const bool pass, const int score);
  double computeMean(const std::vector<double>& vec, const unsigned int begin = 0);
};

//}

/* onInit() //{ */

void BoidsTest::onInit() {

  _nh_ = nodelet::Nodelet::getPrivateNodeHandle();  // Not MT on purpose
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(_nh_, false, "EvaluationBoids");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {
    NODELET_ERROR("[EvaluationBoids]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
    exit(-1);
  }

  param_loader.loadParam("file", _file_path_);
  if (_file_path_ != "") {
    _output_to_file_ = true;
  }

  std::vector<std::tuple<std::string, int, int, std::vector<std::tuple<double, int>>>> tasks;

  bool   wrapper_verbose                = true;
  double task_eval_allowed_failure_rate = 0.0;

  task_eval_allowed_failure_rate = param_loader.loadParamReusable("statistical_evaluation/allowed_failure_rate", 0.0);
  if (task_eval_allowed_failure_rate > 1.0 || task_eval_allowed_failure_rate < 0.0) {
    NODELET_ERROR("[EvaluationBoids]: Loaded parameter for statistical evaluation are incorrectly set!");
    ros::shutdown();
    exit(-1);
  }

  const std::string force_variant = param_loader.loadParamReusable("variant", std::string(""));
  const bool        evaluation    = param_loader.loadParamReusable("evaluation", false);
  if (evaluation) {

    for (const std::string& v : {"easy", "medium", "difficult"}) {
      const auto& [task_variant_eval_repeat, base, bonus_table] = loadVariantParameters(param_loader, v);
      tasks.push_back({v, task_variant_eval_repeat, base, bonus_table});
    }

    wrapper_verbose = false;
  }

  // If NOT evaluation and a variant is given.
  else if (force_variant != "") {

    if (force_variant == "easy" || force_variant == "medium" || force_variant == "difficult") {

      const auto& [task_variant_eval_repeat, base, bonus_table] = loadVariantParameters(param_loader, force_variant);

      // Just one run
      tasks = {{force_variant, 1, base, bonus_table}};

    } else {

      NODELET_ERROR("[EvaluationBoids]: Given variant %s is undefined. Pass one of: easy, medium, difficult.", force_variant.c_str());
      ros::shutdown();
      exit(-1);
    }

  } else {

    // If NOT evaluation and NO specific variant is given
    tasks = {{"testing", 1, 0, {}}};
  }

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[EvaluationBoids]: Could not load all parameters!");
    ros::shutdown();
    exit(-1);
  }

  const auto start_time = ros::WallTime::now();

  bool passed      = true;
  int  score_total = 0;

  task_03_wrapper::WrapperBoids wrapper = task_03_wrapper::WrapperBoids(_nh_, wrapper_verbose);

  for (const auto& task : tasks) {

    const auto& [task_variant, task_eval_repeat, task_base_score, task_bonus_score_table] = task;

    const int max_failed_runs = int(task_eval_allowed_failure_rate * double(task_eval_repeat));

    int t_count_succ = 0;
    int t_count_fail = 0;
    int t_count_skip = 0;

    int t_count_fail_timeout              = 0;
    int t_count_fail_seq_incorrect        = 0;
    int t_count_fail_not_enough_survivors = 0;
    int t_count_fail_impl_err             = 0;

    /*//{ Run this variant N times */
    std::vector<double> alive_ratios(task_eval_repeat, 0.0);
    for (int i = 0; i < task_eval_repeat; i++) {

      NODELET_INFO("[EvaluationBoids]: Running task variant %s: run %d/%d", task_variant.c_str(), i + 1, task_eval_repeat);

      const task_03_wrapper::WrapperBoidsResult_t res = wrapper.runTask(task_variant);

      if (res.success) {
        t_count_succ++;
        alive_ratios.at(i) = res.ratio_alive;

        NODELET_INFO("[EvaluationBoids]: Run %d/%d: SUCCESS (alive ratio: %.2f, iterations: %d, time: %.2f s)", i + 1, task_eval_repeat, res.ratio_alive,
                     res.iterations, res.time);

      } else {
        t_count_fail++;

        if (!res.finished_in_time) {
          t_count_fail_timeout++;
        }
        if (!res.sequence_correct) {
          t_count_fail_seq_incorrect++;
        }
        if (!res.enough_survivors) {
          t_count_fail_not_enough_survivors++;
        }
        if (res.implementation_error) {
          t_count_fail_impl_err++;
        }

        NODELET_ERROR("[EvaluationBoids]: Run %d/%d: FAILED with msg: %s", i + 1, task_eval_repeat, res.message.c_str());

        if (t_count_fail > max_failed_runs) {
          NODELET_ERROR("[EvaluationBoids]: Too many runs have failed %d/%d, Skipping the rest.", t_count_fail, task_eval_repeat);
          t_count_skip = task_eval_repeat - t_count_succ - t_count_fail;
          break;
        }
      }
    }
    /*//}*/

    // | -------------------- EVALUATE VARIANT -------------------- |
    const bool t_success = (double(t_count_fail) / double(task_eval_repeat)) <= task_eval_allowed_failure_rate;

    std::sort(alive_ratios.begin(), alive_ratios.end());  // Sort in non-descending order

    if (t_success) {

      const double aliveness_mean = computeMean(alive_ratios, max_failed_runs);

      // COMPUTE BONUS SCORE
      int task_bonus_score = 0;
      for (const auto& tup : task_bonus_score_table) {
        const auto& [aliveness_limit, bonus_points] = tup;
        /* NODELET_INFO("[EvaluationBoids]: TMP: mean: %.2f, alivenes limit: %.2f, bonus points: %d", aliveness_mean, aliveness_limit, bonus_points); */
        if (aliveness_mean >= aliveness_limit) {
          /* NODELET_INFO("[EvaluationBoids]: TMP: mean: %.2f >= alivenes limit: %.2f", aliveness_mean, aliveness_limit); */
          task_bonus_score = bonus_points;
          break;
        }
      }

      score_total += task_base_score + task_bonus_score;

      NODELET_INFO("[EvaluationBoids]: Variant %s PASSED:", task_variant.c_str());
      NODELET_INFO("             score:            %d (base: %d, bonus: %d)", task_base_score + task_bonus_score, task_base_score, task_bonus_score);
      NODELET_INFO("             successfull runs: %d/%d", t_count_succ, task_eval_repeat);
      NODELET_INFO("             failed runs:      %d/%d", t_count_fail, task_eval_repeat);
      NODELET_INFO("                  timeout:              %d/%d", t_count_fail_timeout, t_count_fail);
      NODELET_INFO("                  incorrect sequence:   %d/%d", t_count_fail_seq_incorrect, t_count_fail);
      NODELET_INFO("                  not enough survivors: %d/%d", t_count_fail_not_enough_survivors, t_count_fail);
      NODELET_INFO("                  implementation error: %d/%d", t_count_fail_impl_err, t_count_fail);
      NODELET_INFO("             average ratio of alive agents in the best %d%s of runs: %.2f", int(100.0 * (1.0 - task_eval_allowed_failure_rate)), "%",
                   aliveness_mean);

    } else {

      const double aliveness_mean = computeMean(alive_ratios, task_eval_repeat - t_count_succ);

      const int run_count = t_count_succ + t_count_fail;
      NODELET_ERROR("[EvaluationBoids]: Variant %s FAILED:", task_variant.c_str());
      NODELET_ERROR("            successfull runs: %d/%d", t_count_succ, run_count);
      NODELET_ERROR("            failed runs:      %d/%d", t_count_fail, run_count);
      NODELET_ERROR("                 timeout:              %d/%d", t_count_fail_timeout, t_count_fail);
      NODELET_ERROR("                 incorrect sequence:   %d/%d", t_count_fail_seq_incorrect, t_count_fail);
      NODELET_ERROR("                 not enough survivors: %d/%d", t_count_fail_not_enough_survivors, t_count_fail);
      NODELET_ERROR("                 implementation error: %d/%d", t_count_fail_impl_err, t_count_fail);
      NODELET_ERROR("            skipped runs:      %d/%d", t_count_skip, task_eval_repeat);
      NODELET_ERROR_COND(t_count_succ > 0, "            average ratio of alive agents in successfull runs: %.2f", aliveness_mean);

      if (task_variant == "easy") {
        passed      = false;
        score_total = 0;
        NODELET_ERROR("[EvaluationBoids]: Passing %s variant is compulsory. 0 points received.", task_variant.c_str());

        break;
      }
    }
  }

  NODELET_INFO("[EvaluationBoids] ##########################");
  NODELET_INFO("[EvaluationBoids] ######### RESULTS ########");
  NODELET_INFO("[EvaluationBoids] ##########################");
  NODELET_INFO("[EvaluationBoids]: %s: %d points", passed ? "PASS" : "FAILED", score_total);
  NODELET_INFO("[EvaluationBoids]: Finished in %.2f s", (ros::WallTime::now() - start_time).toSec());

  saveResult(passed, score_total);

  ros::shutdown();
}

//}

// | ------------------------ routines ------------------------ |

/* loadVariantParameters() //{ */

std::tuple<int, int, std::vector<std::tuple<double, int>>> BoidsTest::loadVariantParameters(mrs_lib::ParamLoader& param_loader,
                                                                                            const std::string&    task_variant) {

  // | -------------------------- Load  ------------------------- |
  const int task_eval_repeat = param_loader.loadParamReusable("statistical_evaluation/repeat/" + task_variant, 1);
  if (task_eval_repeat <= 0) {
    NODELET_ERROR("[EvaluationBoids]: Loaded parameter for statistical evaluation are incorrectly set!");
    ros::shutdown();
    exit(-1);
  }

  // | -------------------- Load base points -------------------- |
  const int base = param_loader.loadParamReusable("tasks/" + task_variant + "/scoring/base", -1);

  if (base < 0) {
    NODELET_ERROR("[EvaluationBoids]: Base points for variant %s are negative!", task_variant.c_str());
    ros::shutdown();
    exit(-1);
  }

  // | -------------------- Load bonus points ------------------- |
  std::vector<double> bonus_aliveness;
  std::vector<int>    bonus_points;
  param_loader.loadParam("tasks/" + task_variant + "/scoring/bonus/aliveness", bonus_aliveness);
  param_loader.loadParam("tasks/" + task_variant + "/scoring/bonus/points", bonus_points);

  if (bonus_aliveness.size() != bonus_points.size()) {
    NODELET_ERROR("[EvaluationBoids]: Bonus points for variant %s are specified incorrectly!", task_variant.c_str());
    ros::shutdown();
    exit(-1);
  }

  std::vector<std::tuple<double, int>> bonus_table(bonus_points.size());
  for (unsigned int i = 0; i < bonus_points.size(); i++) {
    bonus_table.at(i) = {bonus_aliveness[i], bonus_points[i]};
  }

  return {task_eval_repeat, base, bonus_table};
}

//}

/* saveResult() //{ */

void BoidsTest::saveResult(const bool pass, const int score) {

  if (_output_to_file_) {
    output_file_ = fopen(_file_path_.c_str(), "w+");
    fprintf(output_file_, "%d %d %s", pass, score, "RuntimeOk");
    fclose(output_file_);
  }
}

//}

/*//{ computeMean() */
double BoidsTest::computeMean(const std::vector<double>& vec, const unsigned int begin) {

  if (begin >= vec.size()) {
    return 0.0;
  }

  const double sum  = std::accumulate(vec.begin() + begin, vec.end(), 0.0);
  const double mean = sum / double(vec.size() - begin);

  return mean;
}
/*//}*/

}  // namespace task_03_evaluation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_03_evaluation::BoidsTest, nodelet::Nodelet)

