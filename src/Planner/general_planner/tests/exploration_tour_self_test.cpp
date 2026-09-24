#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace fast_planner {
struct ExplorationTourTestAccess {
  static void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
  }

  static void run(bool lkh, const std::string &directory) {
    FastExplorationManager manager;
    manager.ep_ = std::make_shared<ExplorationParam>();
    manager.ed_ = std::make_shared<ExplorationData>();
    manager.planner_manager_ = std::make_shared<FastPlannerManager>();
    manager.planner_manager_->gcopter_config_ = std::make_unique<GcopterConfig>();
    manager.coverage_guidance_ = std::make_shared<CoverageGuidanceManager>();
    auto &config = *manager.ep_;
    config.use_lkh_ = lkh;
    config.tsp_dir_ = directory;
    config.coverage_motion_.enabled = false;
    config.goal_lock_enable_ = false;
    config.goal_keep_cost_ratio_ = 1.35;
    config.goal_switch_min_improvement_ = 2;
    config.goal_switch_min_interval_ = 1.2;
    config.goal_switch_high_speed_multiplier_ = 1.8;
    config.goal_lock_match_radius_ = 1;
    config.composite_candidate_cost_enable_ = true;
    config.candidate_travel_weight_ = 1;
    config.candidate_information_gain_weight_ = 100;
    config.candidate_gain_saturation_ = 30;
    config.candidate_return_horizon_ = 1;
    config.candidate_return_cost_cap_ = 12;
    std::vector<TopoNode::Ptr> candidates;
    std::vector<EdgeSafetyCost> edges(2);
    for (int i = 0; i < 2; ++i) {
      auto node = std::make_shared<TopoNode>();
      node->center_ = Eigen::Vector3f(1 + 4 * i, 0, 1);
      node->frontier_cluster_id_ = 10 + i;
      node->yaw_ = 0;
      candidates.push_back(node);
      edges[i].time_cost = edges[i].total_cost = i + 1;
    }
    candidates[0]->frontier_information_gain_ = 1000000;
    Eigen::MatrixXd matrix(3, 3);
    matrix << 0, 1, 2, 0, 0, 100, 0, 1, 0;
    std::vector<int> tour;
    auto choose = [&](const Eigen::MatrixXd &costs) {
      return manager.selectExplorationTour(candidates, edges, costs,
          Eigen::Vector3d::Zero(), false, false, 0, tour);
    };
    // A high-gain near goal is a terrible first hop for the full open route.
    for (const std::string mode : {"off", "shadow", "tour", "soft", "full"}) {
      manager.coverage_guidance_->config_.mode = mode;
      manager.ed_->has_goal_lock_ = false;
      const int expected = mode == "soft" || mode == "full" ? 0 : 1;
      require(choose(matrix) == expected, "guidance mode did not own the expected selector");
      require(tour.size() == 3 && tour[0] == 0 && tour[1] == expected + 1 &&
                  tour[2] == 2 - expected, "selected tour dropped or duplicated a goal");
    }
    manager.coverage_guidance_->config_.mode = "tour";
    config.goal_lock_enable_ = true;
    manager.ed_->has_goal_lock_ = true;
    manager.ed_->locked_goal_cluster_id_ = 10;
    manager.ed_->locked_goal_ = candidates[0]->center_;
    manager.ed_->locked_goal_time_ = ros::Time::now() - ros::Duration(10);
    require(choose(matrix) == 1, "one-hop hysteresis vetoed a much better complete tour");
    matrix(0, 2) = 1.1;
    matrix(1, 2) = 1;
    manager.ed_->locked_goal_time_ = ros::Time::now() - ros::Duration(10);
    require(choose(matrix) == 1 && tour[1] == 2,
            "small route improvement caused an unnecessary goal switch");
    config.goal_lock_enable_ = false;
    matrix(0, 2) = 2;

    // Exercise real ownership penalties, not an injected scoring stub.
    manager.swarm_coordinator_ = std::make_shared<SwarmExplorationCoordinator>();
    auto &swarm = *manager.swarm_coordinator_;
    swarm.enabled_ = true;
    swarm.team_size_ = 2;
    swarm.have_self_state_ = true;
    const double now = ros::Time::now().toSec();
    swarm.robots_[1].last_seen = now;
    swarm.robots_[1].position = candidates[0]->center_.cast<double>();
    for (int i = 0; i < 2; ++i) {
      auto key = swarm.taskKey(candidates[i]->center_.cast<double>());
      auto &task = swarm.tasks_[key];
      task.key = key;
      task.last_update = task.last_publish = now;
      if (i == 0) {
        task.state = SwarmExplorationCoordinator::TaskState::EXPLORING;
        task.owner_robot = 1;
        task.lease_until = now + 15;
      }
    }
    config.composite_candidate_cost_enable_ = false;
    require(choose(matrix) == 1, "tour refactor discarded a peer's live task lease");
    manager.swarm_coordinator_.reset();

    auto one = std::vector<TopoNode::Ptr>{candidates.front()};
    Eigen::MatrixXd single = Eigen::MatrixXd::Zero(2, 2);
    single(0, 1) = 1;
    require(manager.selectExplorationTour(one, {edges.front()}, single,
        Eigen::Vector3d::Zero(), false, false, 0, tour) == 0 && tour.size() == 2,
        "single-goal exploration failed");
    if (lkh) {
      std::ofstream parameters(directory + "/single.par");
      parameters << "PROBLEM_FILE = " << directory << "/single.tsp\n"
                 << "OUTPUT_TOUR_FILE = " << directory << "/missing/single.txt\n"
                 << "MOVE_TYPE = 2\nGAIN23 = NO\nRUNS = 1\nTRACE_LEVEL = 0\n";
      parameters.close();
      require(choose(matrix) == 0 && tour.size() == 3,
              "unwritable LKH output did not use the fallback tour");
    }
    matrix(0, 1) = std::numeric_limits<double>::quiet_NaN();
    require(choose(matrix) == -1 && tour.empty(), "malformed cost matrix was committed");
  }
};
}  // namespace fast_planner

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_tour_self_test", ros::init_options::AnonymousName);
  ros::Time::init();
  const auto directory = std::filesystem::temp_directory_path() /
      ("exploration_tour_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  int result = 0;
  try {
    std::ofstream parameters(directory / "single.par");
    parameters << "PROBLEM_FILE = " << (directory / "single.tsp").string()
               << "\nOUTPUT_TOUR_FILE = " << (directory / "single.txt").string()
               << "\nMOVE_TYPE = 2\nGAIN23 = NO\nRUNS = 1\nTRACE_LEVEL = 0\n";
    parameters.close();
    fast_planner::ExplorationTourTestAccess::run(false, directory.string());
    fast_planner::ExplorationTourTestAccess::run(true, directory.string());
    std::cout << "exploration tour PASS: modes, full-route hysteresis, peer leases, LKH and fallback\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  std::filesystem::remove_all(directory);
  return result;
}
