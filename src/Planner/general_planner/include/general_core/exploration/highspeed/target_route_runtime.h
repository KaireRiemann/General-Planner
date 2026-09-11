#pragma once

#include <Eigen/Core>
#include <map_manager/incremental_topology_graph.hpp>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace fast_planner {

// LOCAL_GOAL is certified entirely by current observations and works without
// any global graph. Only KNOWN_* count as persistent-topology reuse.
enum class TargetRouteSource { LOCAL_EXPLORATION, KNOWN_GOAL, KNOWN_ANCHOR, LOCAL_GOAL };
enum class TargetRouteEvidence { UNKNOWN, FREE, OCCUPIED, EXCLUDED };

struct TargetRouteConfig {
  std::string mode{"prefer_known"};  // legacy | shadow | prefer_known
  double query_interval{1.0};
  double query_budget_ms{12.0};
  int max_nodes{16000};
  int max_expansions{4000};
  int max_map_checks{16000};
  double sample_step{0.10};
  double prefix_length{12.0};
  double min_prefix_length{0.75};
  double stop_margin{0.40};
  double projection_radius{1.50};
  double progress_timeout{12.0};
  double cooldown{5.0};
  double anchor_switch_margin{1.0};
};

// Read-only callbacks belong to the serial world/map-owner queue. The graph
// itself is immutable. Never run these live local-map callbacks on a worker.
struct TargetRouteMapView {
  std::function<TargetRouteEvidence(const Eigen::Vector3d &)> global;
  std::function<bool(const Eigen::Vector3d &)> local_free;
};

// Shared commit gate, exposed for endpoint/sub-step/backup regression tests.
bool targetRouteTrajectoryKnownFree(double duration, double dt, double step,
    const std::function<Eigen::Vector3d(double)> &position,
    const std::function<bool(const Eigen::Vector3d &)> &local_free);

struct TargetRoute {
  TargetRouteSource source{TargetRouteSource::LOCAL_EXPLORATION};
  std::uint64_t id{0}, task{0}, world{0}, topology_revision{0};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> points;
  // One stable graph identity per point; 0 denotes an odometry/goal connector.
  std::vector<std::uint64_t> nodes;
  std::vector<double> arc;
  double progress{0.0};
  double last_progress_time{0.0};
  double progress_checkpoint{0.0};
  bool valid() const { return points.size() >= 2 && arc.size() == points.size(); }
};

struct TargetRouteExecutionContext {
  TargetRouteSource source{TargetRouteSource::LOCAL_EXPLORATION};
  std::uint64_t route_id{0}, task{0}, world{0};
  bool stop_at_boundary{true};
  bool enabled() const { return source != TargetRouteSource::LOCAL_EXPLORATION; }
};

struct TargetRoutePrefix {
  std::vector<Eigen::Vector3f> path;
  TargetRouteExecutionContext context;
  double remaining{0.0};
  std::string reason;
  bool ready() const { return context.enabled() && path.size() >= 2; }
};

// World topology is guidance only. This task-local object owns the route,
// actual-odometry progress and stable-node/edge cooldowns, never the world map.
class TargetRouteRuntime {
public:
  using Graph = general_planner::IncrementalTopologyGraph;
  void configure(TargetRouteConfig config);
  void reset();
  TargetRoutePrefix prepare(const Graph::SearchSnapshotPtr &snapshot,
                           std::uint64_t world, const Eigen::Vector3d &position,
                           const Eigen::Vector3d &goal, double now,
                           const TargetRouteMapView &map,
                           double local_objective = std::numeric_limits<double>::infinity());
  void fail(double now, const std::string &reason);
  const TargetRoute &route() const { return route_; }
  const std::string &status() const { return status_; }
  const TargetRouteConfig &config() const { return config_; }
  std::size_t cooldownSize() const { return cooldowns_.size(); }
  double lastQueryMs() const { return last_query_ms_; }
  int lastMapChecks() const { return last_map_checks_; }
  int lastExpansions() const { return last_expansions_; }
  const std::string &queryStage() const { return query_stage_; }

private:
  struct Cooldown { std::uint64_t a{0}, b{0}; double until{0.0}; };
  TargetRoute query(const Graph::SearchSnapshotPtr &snapshot,
                    const Eigen::Vector3d &position, const Eigen::Vector3d &goal,
                    double now, const TargetRouteMapView &map);
  TargetRoutePrefix prefix(const Eigen::Vector3d &position, double now,
                           const TargetRouteMapView &map);
  bool cooling(std::uint64_t a, std::uint64_t b, double now) const;
  void cool(std::uint64_t a, std::uint64_t b, double now);
  TargetRouteConfig config_;
  TargetRoute route_;
  std::vector<Cooldown> cooldowns_;
  std::uint64_t task_{0}, world_{0}, next_id_{0};
  Eigen::Vector3d goal_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d last_odom_{Eigen::Vector3d::Zero()};
  bool have_odom_{false};
  double last_query_{-std::numeric_limits<double>::infinity()};
  double last_sample_time_{0.0};
  double local_goal_retry_after_{0.0};
  double last_query_ms_{0.0};
  int last_map_checks_{0}, last_expansions_{0};
  std::string query_stage_;
  std::string status_{"RESET"};
};

}  // namespace fast_planner
