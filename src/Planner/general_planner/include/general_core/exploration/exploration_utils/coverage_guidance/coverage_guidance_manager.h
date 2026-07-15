#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/MarkerArray.h>

#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_types.h>

namespace fast_planner {

class CoverageGuidanceManager {
public:
  using Ptr = std::shared_ptr<CoverageGuidanceManager>;

  CoverageGuidanceManager();
  ~CoverageGuidanceManager();

  void initialize(ros::NodeHandle &nh, const CoverageMapSpec &map_spec);
  bool enabled() const;
  bool affectsPlanning() const;
  bool fullMode() const;
  const std::string &modeName() const;
  const CoverageMapSpec &mapSpec() const;
  bool samplingDue() const;

  void submit(CoverageMapDelta delta,
              std::vector<CoverageFrontier> frontiers,
              const Eigen::Vector3d &robot_position);
  CoveragePlan::Ptr latestUsablePlan() const;
  std::unordered_set<int> preferredClusterIds() const;
  double clusterPenalty(int cluster_id,
                        const Eigen::Vector3d &position) const;
  bool blocksFinish() const;
  void publishVisualization() const;

  static bool runDeterministicSelfTest(std::string *error = nullptr);

private:
  struct Config {
    std::string mode{"off"};
    double update_period{1.0};
    double stale_timeout{3.0};
    double fine_cell_size{4.8};
    double macro_cell_size{14.4};
    double near_expand_radius{36.0};
    double unknown_edge_penalty{1.8};
    double soft_weight{3.0};
    double full_rank_weight{12.0};
    double rank_penalty_cap{0.0};
    double unknown_first_penalty{1000.0};
    int min_unknown_voxels{8};
    int max_cp_nodes{160};
    int max_preferred_clusters{16};
    bool finish_guard_enable{true};
    bool visualization_enable{true};
  };

  struct WorkItem {
    CoverageMapDelta delta;
    std::vector<CoverageFrontier> frontiers;
    Eigen::Vector3d robot_position{Eigen::Vector3d::Zero()};
    std::uint64_t frontier_version{0};
  };

  void workerLoop();
  CoveragePlan buildPlan(const WorkItem &work);
  bool voxelIsValid(const Eigen::Vector3d &position) const;
  static double wallNow();

  Config config_;
  CoverageMapSpec map_spec_;
  std::vector<CoverageVoxelState> persistent_map_;
  ros::Publisher marker_pub_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<WorkItem> pending_work_;
  CoveragePlan::Ptr latest_plan_;
  std::thread worker_;
  bool stop_{false};
  mutable double last_submit_wall_{-std::numeric_limits<double>::infinity()};
  std::uint64_t frontier_version_{0};
};

}  // namespace fast_planner
