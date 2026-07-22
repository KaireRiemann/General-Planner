#pragma once

#include <Eigen/Eigen>
#include <data_structure/base/trajectory.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <traj_utils/PolyTraj.h>

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fast_planner {

// Stable spatial identity shared by all robots.  Frontier cluster ids remain
// local and are deliberately not part of the wire protocol.
struct SwarmTaskKey {
  int ix{0};
  int iy{0};
  int iz{0};
  int floor{0};

  bool operator==(const SwarmTaskKey &other) const {
    return ix == other.ix && iy == other.iy && iz == other.iz &&
           floor == other.floor;
  }
};

struct SwarmTaskKeyHash {
  std::size_t operator()(const SwarmTaskKey &key) const;
};

struct SwarmCandidate {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double information_gain{0.0};
  double travel_cost{0.0};
  bool coverage{false};
};

// A decentralized coordination plane inspired by GVP-MREP's sparse MR-DTG
// and RACER's explicit task ownership.  It is a strict no-op when disabled;
// the existing single-UAV exploration pipeline never depends on this class.
class SwarmExplorationCoordinator {
public:
  using Ptr = std::shared_ptr<SwarmExplorationCoordinator>;

  SwarmExplorationCoordinator() = default;
  ~SwarmExplorationCoordinator() = default;

  void init(ros::NodeHandle &nh);
  bool enabled() const { return enabled_; }
  int robotId() const { return robot_id_; }

  void updateRobotState(const Eigen::Vector3d &position,
                        const Eigen::Vector3d &velocity);

  // Returns one additive cost per candidate.  A peer's live lease receives a
  // hard-sized penalty; graph-Voronoi ownership and assist mode receive softer
  // penalties.  No candidate is removed from the single-UAV planner.
  std::vector<double> candidatePenalties(
      const std::vector<SwarmCandidate> &candidates);
  void claimTask(const SwarmCandidate &candidate);
  void completeTaskAt(const Eigen::Vector3d &position);
  void releaseCurrentTask(const std::string &reason);

  void setLocalConverged(bool converged,
                         bool frontier_audit_ready = false,
                         bool coverage_plateau = false,
                         bool targets_exhausted = false);
  bool teamFinishReady();
  bool teamFinishCandidate() const;
  bool hasLocallyActionableTask() const;
  std::string statusString() const;

  // Commit-time hard inter-UAV trajectory gate.  Lower robot id is the stable
  // tie-break priority.  A higher-priority trajectory received later raises a
  // yield request which the exploration FSM handles with its existing safe
  // stop/replan path.
  bool validateAndCommitTrajectory(const traj_utils::PolyTraj &trajectory,
                                   int *conflicting_robot = nullptr);
  bool consumeYieldRequest(int *priority_robot = nullptr);

private:
  enum class TaskState : int {
    AVAILABLE = 0,
    CLAIMED = 1,
    EXPLORING = 2,
    COMPLETED = 3,
    EXHAUSTED = 4,
    UNREACHABLE = 5
  };

  struct TaskRecord {
    SwarmTaskKey key;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    TaskState state{TaskState::AVAILABLE};
    int source_robot{-1};
    int owner_robot{-1};
    bool coverage{false};
    double information_gain{0.0};
    double estimated_cost{std::numeric_limits<double>::infinity()};
    double lease_until{0.0};
    std::uint32_t revision{0};
    double last_update{0.0};
    double last_publish{0.0};
  };

  struct RobotRecord {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    std::uint64_t anchor_id{0};
    bool local_converged{false};
    bool finish_candidate{false};
    std::uint64_t task_revision{0};
    std::uint32_t active_claims{0};
    double last_seen{0.0};
  };

  struct GraphNode {
    std::uint64_t id{0};
    int source_robot{-1};
    int floor{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    std::unordered_map<std::uint64_t, double> edges;
  };

  struct RemoteTrajectory {
    int robot_id{-1};
    int priority{0};
    double start_time{0.0};
    double valid_until{0.0};
    double horizontal_clearance{1.2};
    double vertical_clearance{0.8};
    geometry_utils::Trajectory trajectory;
  };

  static constexpr double kWireVersion = 1.0;

  SwarmTaskKey taskKey(const Eigen::Vector3d &position) const;
  std::uint64_t makeNodeId(std::uint64_t local_sequence) const;
  bool peerAliveLocked(int robot_id, double now) const;
  int taskOwnerLocked(const Eigen::Vector3d &position, double now) const;
  std::uint64_t nearestNodeLocked(const Eigen::Vector3d &position) const;
  double graphDistanceLocked(std::uint64_t source,
                             std::uint64_t target) const;
  bool livePeerLeaseLocked(const SwarmTaskKey &key, double now,
                           int *owner = nullptr) const;
  bool remoteClaimWinsLocked(const TaskRecord &remote,
                             const TaskRecord &local) const;
  void addHistoryAnchorLocked(const Eigen::Vector3d &position,
                              std::vector<double> *wire);
  std::vector<double> encodeGraphNodeLocked(const GraphNode &node,
                                             bool full_sync) const;
  void publishTaskLocked(TaskRecord &task);
  void pruneLocked(double now);
  std::uint32_t activeTaskCountLocked(double now) const;
  bool allMembersConvergedLocked(double now) const;
  bool trajectoryConflictLocked(const RemoteTrajectory &first,
                                const RemoteTrajectory &second) const;
  bool decodeTrajectory(const std_msgs::Float64MultiArray &msg,
                        RemoteTrajectory &out) const;
  std_msgs::Float64MultiArray encodeTrajectory(
      const traj_utils::PolyTraj &trajectory) const;

  void stateTimerCallback(const ros::TimerEvent &);
  void maintenanceTimerCallback(const ros::TimerEvent &);
  void robotStateCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  void taskCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  void graphCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  void finishCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  void trajectoryCallback(const std_msgs::Float64MultiArrayConstPtr &msg);

  bool enabled_{false};
  int robot_id_{0};
  int team_size_{1};
  std::uint32_t mission_epoch_{1};
  Eigen::Vector3d task_origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d task_size_{Eigen::Vector3d(3.6, 3.6, 1.8)};
  double floor_height_{3.0};
  double peer_timeout_{1.5};
  double task_evidence_timeout_{5.0};
  double task_evidence_publish_interval_{1.0};
  double task_lease_duration_{15.0};
  double ownership_penalty_{2000.0};
  double peer_lease_penalty_{1.0e6};
  double assist_penalty_{4.0};
  double continuity_bonus_{8.0};
  double load_balance_weight_{3.0};
  double anchor_spacing_{2.5};
  double graph_handshake_radius_{3.5};
  int graph_handshake_max_{4};
  int max_graph_nodes_{2500};
  double finish_quiet_period_{3.0};
  double finish_hold_duration_{3.0};
  double trajectory_horizon_{6.0};
  double trajectory_sample_dt_{0.08};
  double horizontal_clearance_{1.2};
  double vertical_clearance_{0.8};

  mutable std::mutex mutex_;
  Eigen::Vector3d self_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d self_velocity_{Eigen::Vector3d::Zero()};
  bool have_self_state_{false};
  bool local_converged_{false};
  bool frontier_audit_ready_{false};
  bool coverage_plateau_{false};
  bool targets_exhausted_{false};
  bool finish_candidate_{false};
  double finish_candidate_since_{0.0};
  double last_registry_mutation_{0.0};
  std::uint64_t task_revision_{0};
  std::uint64_t local_graph_sequence_{0};
  std::uint64_t current_anchor_id_{0};
  std::uint64_t previous_anchor_id_{0};
  Eigen::Vector3d last_anchor_position_{Eigen::Vector3d::Zero()};
  bool have_anchor_{false};
  bool have_current_task_{false};
  SwarmTaskKey current_task_key_;
  std::unordered_set<SwarmTaskKey, SwarmTaskKeyHash> local_candidate_keys_;
  std::unordered_map<SwarmTaskKey, TaskRecord, SwarmTaskKeyHash> tasks_;
  std::unordered_map<int, RobotRecord> robots_;
  std::unordered_map<std::uint64_t, GraphNode> graph_;
  std::vector<std::uint64_t> local_graph_nodes_;
  std::size_t full_sync_cursor_{0};
  std::unordered_map<int, RemoteTrajectory> remote_trajectories_;
  RemoteTrajectory local_trajectory_;
  bool have_local_trajectory_{false};
  bool yield_required_{false};
  int yield_priority_robot_{-1};

  ros::Publisher robot_state_pub_;
  ros::Publisher task_pub_;
  ros::Publisher graph_pub_;
  ros::Publisher finish_pub_;
  ros::Publisher trajectory_pub_;
  ros::Subscriber robot_state_sub_;
  ros::Subscriber task_sub_;
  ros::Subscriber graph_sub_;
  ros::Subscriber finish_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Timer state_timer_;
  ros::Timer maintenance_timer_;
};

}  // namespace fast_planner
