/* include //{ */

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>

// STD includes
#include <random>

// MRS lib includes
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/geometry/conversions_eigen.h>

#include <Eigen/Dense>

//}

/* class TargetMover //{ */

class TargetMover {
public:
  /* TargetMover() //{ */

  TargetMover(mrs_lib::ParamLoader& pl) {

    pl.loadParam("model_name", m_name);

    const Eigen::MatrixXd start_pos = pl.loadMatrixDynamic2("start_positions", -1, 3);

    m_pose.position = mrs_lib::geometry::fromEigen(start_pos.row(randi(0, int(start_pos.rows()) - 1)));

    pl.loadParam("speed", m_speed);
    pl.loadParam("goal_threshold", m_close_threshold);
    pl.loadParam("cell_step", m_cell_step);
    pl.loadParam("num_cells", m_num_cells);

    if (!pl.loadedSuccessfully()) {
      ROS_ERROR("[%s]: failed to load all parameters", ros::this_node::getName().c_str());
      ros::shutdown();
    }

    m_goal = new_goal(m_pose.position);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "]: Initial goal is: " << m_goal);
  }

  //}

  /* getState() //{ */

  gazebo_msgs::ModelState get_state() {
    gazebo_msgs::ModelState ret;
    ret.model_name = m_name;
    ret.pose       = m_pose;
    ret.twist      = m_twist;
    return ret;
  }

  //}

  /* updateState() //{ */

  void update_state(const ros::Duration& dt) {
    update_pose(dt);
    update_twist(dt);
    if (close(m_pose.position, m_goal)) {
      m_goal = new_goal(m_goal);
      ROS_INFO_STREAM("[" << ros::this_node::getName() << "]: New goal is: " << m_goal);
    }
  }

  //}

private:
  /* update_pose //{ */

  void update_pose(const ros::Duration& dt) {

    const Eigen::Vector3d    cur_pos = mrs_lib::geometry::toEigen(m_pose.position);
    const Eigen::Vector3d    vel     = mrs_lib::geometry::toEigen(m_twist.linear);
    const Eigen::Quaterniond ori     = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), vel);

    m_pose.position    = mrs_lib::geometry::fromEigen(cur_pos + dt.toSec() * vel);
    m_pose.orientation = mrs_lib::geometry::fromEigen(ori);
  }

  //}

  /* update_twist() //{ */

  void update_twist([[maybe_unused]] const ros::Duration& dt) {

    const Eigen::Vector3d diff = mrs_lib::geometry::toEigen(m_goal) - mrs_lib::geometry::toEigen(m_pose.position);
    const Eigen::Vector3d vel  = m_speed * diff.normalized();

    m_twist.linear = mrs_lib::geometry::fromEigenVec(vel);
  }

  //}

  /* close() //{ */

  bool close(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {

    const Eigen::Vector3d ae = mrs_lib::geometry::toEigen(a);
    const Eigen::Vector3d be = mrs_lib::geometry::toEigen(b);

    return (ae - be).norm() < m_close_threshold * m_close_threshold;
  }

  //}

  /* new_goal() //{ */

  geometry_msgs::Point new_goal(const geometry_msgs::Point& from) {

    const Eigen::Vector3d cur_coords = mrs_lib::geometry::toEigen(from);
    const Eigen::Vector3i cur_cell   = (cur_coords / m_cell_step).array().round().cast<int>();

    std::vector<Eigen::Vector3i> valid_steps;
    if (cur_cell.x() + 1 < m_num_cells / 2)
      valid_steps.emplace_back(1, 0, 0);
    if (cur_cell.y() + 1 < m_num_cells / 2)
      valid_steps.emplace_back(0, 1, 0);
    if (cur_cell.x() - 1 > -m_num_cells / 2)
      valid_steps.emplace_back(-1, 0, 0);
    if (cur_cell.y() - 1 > -m_num_cells / 2)
      valid_steps.emplace_back(0, -1, 0);

    // Do not visit the (0, 0) cell -> prevent success if the students' solution gets stuck in the initial cell
    valid_steps.erase(std::remove_if(valid_steps.begin(), valid_steps.end(),
                                     [&cur_cell](const Eigen::Vector3i& step) { return step.x() + cur_cell.x() == 0 && step.y() + cur_cell.y() == 0; }),
                      valid_steps.end());

    if (valid_steps.empty()) {
      ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: No valid steps from cell [" << cur_cell.transpose() << "]!");
      return from;
    }

    std::uniform_int_distribution<size_t> m_rnd{0, valid_steps.size() - 1};
    const Eigen::Vector3i                 rnd_step   = valid_steps.at(m_rnd(m_gen));
    const Eigen::Vector3i                 new_cell   = cur_cell + rnd_step;
    const Eigen::Vector3d                 new_coords = m_cell_step * new_cell.cast<double>();
    return mrs_lib::geometry::fromEigen(new_coords);
  }

  //}

  /* randi() //{ */

  int randi(const int from, const int to) {

    double zero_to_one = double((float)rand()) / double(RAND_MAX);

    return int(double(to - from) * zero_to_one + from);
  }

  //}

private:
  std::string          m_name;
  geometry_msgs::Pose  m_pose;
  geometry_msgs::Twist m_twist;

  double               m_close_threshold;
  double               m_speed;
  double               m_cell_step;
  int                  m_num_cells;
  geometry_msgs::Point m_goal;

private:
  std::random_device m_rd{};
  std::mt19937       m_gen{m_rd()};
};

//}

/* main() //{ */

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "target_mover");

  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().nsec));

  // | --------------------- initialization --------------------- |

  mrs_lib::ParamLoader pl(nh, "target_mover");

  TargetMover tm(pl);

  if (!pl.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all necessary parameters!", ros::this_node::getName().c_str());
    ros::shutdown();
    return 1;
  }

  auto pub_model_state = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
  auto pub_target_pose = nh.advertise<geometry_msgs::Pose>("/target_pose", 100);

  ros::Rate r(100);
  ros::Time prev_t = ros::Time::now();

  while (ros::ok()) {
    const ros::Time     now = ros::Time::now();
    const ros::Duration dt  = now - prev_t;
    tm.update_state(dt);
    const auto cur_state = tm.get_state();
    pub_model_state.publish(cur_state);
    pub_target_pose.publish(cur_state.pose);
    prev_t = now;
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
}

//}
