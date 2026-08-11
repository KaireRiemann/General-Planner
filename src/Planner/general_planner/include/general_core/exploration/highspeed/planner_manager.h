#pragma once

#include <Eigen/Eigen>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <data_structure/base/trajectory.h>
#include <general_core/exploration/exploration_utils/lidar_map/lidar_map.h>
#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_types.h>
#include <nav_msgs/Odometry.h>
#include <general_core/exploration/exploration_utils/path_searching/bubble_astar.h>
#include <general_core/exploration/exploration_utils/pointcloud_topo/graph.h>
#include <general_core/exploration/exploration_utils/pointcloud_topo/graph_visualizer.hpp>
#include <general_core/exploration/exploration_utils/pointcloud_topo/parallel_bubble_astar.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <traj_utils/PolyTraj.h>

#include <general_core/exploration/highspeed/visualizer.hpp>

namespace geometry_utils
{
class Trajectory;
}

namespace general_planner
{
class CorridorGenerator;
class MapManager;
}

namespace ros_interface
{
class RosInterface;
}

namespace rog_map
{
class ROGMapROS;
}

namespace traj_opt
{
class ExplorationTrajOpt;
class BackupTrajOpt;
class YawTrajOpt;
}

namespace fast_planner
{
struct GeneralCommitStore;

struct GcopterConfig
{
  std::string mapTopic;
  std::string targetTopic;
  double dilateRadiusSoft{0.45};
  double dilateRadiusHard{0.35};
  double timeoutRRT{0.01};
  double maxVelMag{6.0};
  double maxAccMag{8.0};
  double maxBdrMag{10.0};
  double maxTiltAngle{0.6};
  double minThrust{6.0};
  double maxThrust{20.0};
  double vehicleMass{1.0};
  double gravAcc{9.81};
  double horizDrag{0.0};
  double vertDrag{0.0};
  double parasDrag{0.0};
  double speedEps{1.0e-3};
  double weightT{1.0};
  double WeightSafeT{1.0};
  double energyWeight{1.0};
  std::vector<double> chiVec;
  double smoothingEps{1.0e-2};
  int integralIntervs{16};
  double relCostTol{1.0e-5};
  double corridor_size{5.0};
  double yaw_max_vel{2.0};
  double yaw_rho_vis{1.0};
  double yaw_time_fwd{1.0};
  bool dynamicVelocityEnable{true};
  bool rogMapEnable{true};
  bool generalCorridorEnable{true};
  bool corridorUseRogOccPoints{false};
  bool rogKnownFreeFallbackToLio{true};
  std::string rogMapConfigPath;
  double corridorLineMaxLength{3.0};
  double corridorMinOverlapThreshold{0.15};
  double corridorRobotRadius{0.20};
  double corridorMaxStartShift{0.45};
  int corridorBoxSearchSkipNum{2};
  int corridorIrisIterNum{2};
  double corridorVirtualGroundHeight{-0.10};
  double corridorVirtualCeilHeight{5.50};
  double minSegmentVel{2.5};
  // MinSegmentVel is the nominal cruise floor.  Optimization retries must be
  // allowed below it when a short/sharp path violates acceleration limits.
  double trajectoryRetryMinVel{1.0};
  double openSegmentVel{6.0};
  double dynamicVelocityMinClearance{0.35};
  double dynamicVelocityOpenClearance{2.5};
  double dynamicVelocityClearanceMargin{0.50};
  bool turnVelocityEnable{true};
  double turnLateralAcceleration{5.0};
  double turnSoftAngle{0.35};
  double turnHardAngle{1.05};
  double turnSoftVelocity{7.0};
  double turnHardVelocity{3.5};
  double reorientationHeadingAngle{1.75};
  bool nonstopTerminalVelocityEnable{false};
  double nonstopTerminalVelocityRatio{0.65};
  double nonstopTerminalMinPathLength{8.0};
  double nonstopTerminalMaxTurnAngle{0.65};
  double nonstopTerminalMaxYawDelta{1.0};
  bool backupTrajEnable{true};
  double backupStartRatio{0.55};
  double backupMinStartTime{0.65};
  double backupMaxStartTime{2.2};
  double backupSampleDt{0.08};
  double backupSearchMargin{4.0};
  int backupPieceNum{2};
  double backupMaxVel{6.0};
  double backupMaxAcc{8.0};
  double replanCommitDelay{0.60};
  double commitMinDuration{0.45};
  double commitMaxDuration{2.2};
  double commitSampleDt{0.05};
  double commitKnownFreeSafeDistance{0.50};
  double safetyClearanceTolerance{0.0};
  double commitBackupTimeBuffer{0.15};
  double knownFreeShortLength{4.0};
  double knownFreeMediumLength{10.0};
  double knownFreeLongLength{18.0};
  double velocityShortKnownFree{4.0};
  double velocityMediumKnownFree{8.0};
  double velocityLongKnownFree{6.0};
  double safetyMapQueryStep{0.20};
  bool safetyMapUnknownAsOccupiedForCommit{true};
  bool safetyMapUnknownAsOccupiedForBackup{true};
  bool safetyMapUnknownAllowedForExplore{true};
  double brakeAccel{8.0};
  double plannerLatency{0.12};
  double controlLatency{0.08};
  double safetyBrakeMargin{0.8};
  double curvatureMinRadius{0.8};
  bool velocityLogEnable{true};
  double highSpeedModeThreshold{5.0};
  double highSpeedModeExitThreshold{3.0};
  double viewScoreGainWeight{1.0};
  double viewScoreProgressWeight{0.10};
  double viewScoreVelocityAlignWeight{2.0};
  double viewScoreKnownFreeWeight{0.18};
  double viewScoreClearanceWeight{0.50};
  double viewScoreYawWeight{0.25};
  double viewScoreTurnWeight{1.20};
  double viewScoreBackupPenalty{8.0};
  double viewScoreKnownFreeMaxLen{25.0};
  bool viewScoreHardGateEnable{true};
  double viewScoreHardGateMinKnownFreeRatio{1.0};
  double viewScoreHardGateMaxTurnAngle{1.40};
  double viewScoreHardGateMaxYawDelta{1.60};
  double viewScoreHardGateMinClearance{0.50};
  int viewScoreTopCandidateNum{1};
  double edgeTurnPenaltyWeight{1.0};
  double edgeKnownFreePenaltyWeight{0.45};
  double edgeBackupPenaltyWeight{15.0};
  double edgeYawPenaltyWeight{0.20};
  bool corridorCruiseEnable{true};
  double corridorCruiseKnownFreeLength{18.0};
  double corridorCruiseMinAlignment{0.70};
  double corridorCruiseForwardWeight{0.35};
  double corridorCruiseLateralPenalty{12.0};
  double corridorCruiseMaxBacktrackDistance{2.0};
  double corridorCruiseMinProgress{4.0};
  double corridorCruiseMaxGoalDistance{45.0};

  void init(const ros::NodeHandle &nh_priv);
};

enum class MapVoxelState
{
  OCCUPIED = 0,
  KNOWN_FREE = 1,
  UNKNOWN = 2,
  OUT_OF_MAP = 3
};

struct RaycastSafetyInfo
{
  bool all_known_free{false};
  bool blocked_by_occupied{false};
  bool blocked_by_unknown{false};
  double length{0.0};
  double known_free_length{0.0};
  double min_clearance{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d first_blocked_pos{Eigen::Vector3d::Zero()};
  MapVoxelState first_blocked_state{MapVoxelState::UNKNOWN};
};

struct SegmentSafetyInfo
{
  double path_length{0.0};
  double known_free_length{0.0};
  double min_clearance{std::numeric_limits<double>::infinity()};
  double turn_angle{0.0};
  double max_local_turn{0.0};
  double min_turn_radius{std::numeric_limits<double>::infinity()};
  double initial_heading_delta{0.0};
  double yaw_delta{0.0};
  double current_speed{0.0};
  bool backup_feasible{false};
};

struct SegmentVelocityLimit
{
  double final_limit{0.0};
  double open{0.0};
  double known_free{0.0};
  double brake{0.0};
  double clearance{0.0};
  double curvature{0.0};
  double yaw{0.0};
  double backup{0.0};
  std::string reason{"open"};
};

struct EdgeSafetyCost
{
  double total_cost{0.0};
  double time_cost{0.0};
  double turn_penalty{0.0};
  double known_free_penalty{0.0};
  double backup_penalty{0.0};
  double yaw_penalty{0.0};
  double path_length{0.0};
  double known_free_length{0.0};
  double min_clearance{std::numeric_limits<double>::infinity()};
  double turn_angle{0.0};
  double initial_heading_delta{0.0};
  bool backup_feasible{false};
};

template <int Order>
class Trajectory
{
public:
  void clear()
  {
    points_.clear();
    duration_ = 0.0;
  }

  void setPolyline(const std::vector<Eigen::Vector3f> &path, double duration)
  {
    geometry_traj_.clear();
    points_.clear();
    points_.reserve(path.size());
    for (const auto &p : path)
    {
      points_.push_back(p.cast<double>());
    }
    duration_ = std::max(0.0, duration);
  }

  void setGeometryTrajectory(const geometry_utils::Trajectory &traj)
  {
    geometry_traj_ = traj;
    points_.clear();
    duration_ = geometry_traj_.getTotalDuration();
  }

  const geometry_utils::Trajectory &geometryTrajectory() const
  {
    return geometry_traj_;
  }

  int getPieceNum() const
  {
    if (!geometry_traj_.empty())
    {
      return geometry_traj_.getPieceNum();
    }
    return points_.size() >= 2U ? static_cast<int>(points_.size()) - 1 : 0;
  }

  double getTotalDuration() const
  {
    if (!geometry_traj_.empty())
    {
      return geometry_traj_.getTotalDuration();
    }
    return duration_;
  }

  Eigen::Vector3d getPos(double t) const
  {
    if (!geometry_traj_.empty())
    {
      return geometry_traj_.getPos(t);
    }
    if (points_.empty())
    {
      return Eigen::Vector3d::Zero();
    }
    if (points_.size() == 1U || duration_ <= 1.0e-6)
    {
      return points_.front();
    }
    const double alpha_total = std::clamp(t / duration_, 0.0, 1.0);
    const double scaled = alpha_total * static_cast<double>(points_.size() - 1U);
    const int idx = std::min<int>(static_cast<int>(std::floor(scaled)),
                                  static_cast<int>(points_.size()) - 2);
    const double alpha = scaled - static_cast<double>(idx);
    return (1.0 - alpha) * points_[idx] + alpha * points_[idx + 1];
  }

  Eigen::Vector3d getVel(double t) const
  {
    if (!geometry_traj_.empty())
    {
      return geometry_traj_.getVel(t);
    }
    (void)t;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getAcc(double t) const
  {
    if (!geometry_traj_.empty())
    {
      return geometry_traj_.getAcc(t);
    }
    (void)t;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getJer(double t) const
  {
    if (!geometry_traj_.empty())
    {
      return geometry_traj_.getJer(t);
    }
    (void)t;
    return Eigen::Vector3d::Zero();
  }

  double getMaxVelRate() const
  {
    return geometry_traj_.empty() ? 0.0 : geometry_traj_.getMaxVelRate();
  }

  double getMaxAccRate() const
  {
    return geometry_traj_.empty() ? 0.0 : geometry_traj_.getMaxAccRate();
  }

private:
  geometry_utils::Trajectory geometry_traj_;
  std::vector<Eigen::Vector3d> points_;
  double duration_{0.0};
};

struct LocalTrajData
{
  int traj_id_{0};
  double duration_{0.0};
  ros::Time start_time_;
  Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d curr_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d curr_vel_{Eigen::Vector3d::Zero()};
  double curr_yaw_{0.0};
  double end_yaw_{0.0};
  Trajectory<7> minco_traj_;
  Trajectory<5> minco_yaw_traj_;
  Trajectory<7> exp_traj_;
  Trajectory<5> exp_yaw_traj_;
  Trajectory<7> backup_traj_;
  Trajectory<5> backup_yaw_traj_;
  bool backup_available_{false};
  double backup_start_t_{std::numeric_limits<double>::infinity()};
};

class FastPlannerManager
{
public:
  using Ptr = std::shared_ptr<FastPlannerManager>;

  FastPlannerManager();
  ~FastPlannerManager();

  void printTimeCost(double time_threshold, double time_cost, std::string print_info);
  bool planExploreTraj(const std::vector<Eigen::Vector3f> &path,
                       bool is_static,
                       bool clearance_recovery = false,
                       bool rolling_horizon = false);
  bool planControlledStopTrajectory();
  bool flyToSafeRegion(bool is_static, bool force_relocation = false);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time);
  void polyYawTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time);
  void initPlanModules(ros::NodeHandle &nh,
                       ParallelBubbleAstar::Ptr &parallel_path_finder,
                       TopoGraph::Ptr &graph,
                       const std::shared_ptr<general_planner::MapManager>
                           &shared_map_manager = nullptr);

  bool checkTrajCollision(double &collision_time);
  bool checkTrajVelocity();
  bool hasCommittedTrajectory() const;
  bool hasCommittedBackup() const;
  bool hasCommittedStopTrajectory() const;
  double timeToCommittedBackup() const;
  double committedTrajectoryRemainingTime() const;
  bool isOnCommittedBackup() const;
  bool getCommittedReplanHeadState(Eigen::Vector3d &pos,
                                   Eigen::Vector3d &vel,
                                   double &yaw,
                                   double *traj_time = nullptr,
                                   double *switch_delay = nullptr);
  bool updateRogMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                    const nav_msgs::Odometry::ConstPtr &odom_msg);
  /** M2: GlobalMapRuntime has already fused this frame into the shared map. */
  void notifyGlobalMapUpdated(std::uint64_t map_revision = 0);
  bool sampleCoverageMap(const CoverageMapSpec &spec,
                         CoverageMapDelta &delta) const;
  bool isSafetyMapReady() const;
  MapVoxelState querySafetyState(const Eigen::Vector3d &pos) const;
  const char *safetyStateName(MapVoxelState state) const;
  double safetyDistanceToOcc(const Eigen::Vector3d &pos) const;
  RaycastSafetyInfo raycastSafety(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &end,
                                  bool unknown_as_occupied,
                                  double safe_distance,
                                  double step) const;
  double forwardKnownFreeLength(const Eigen::Vector3d &start,
                                const Eigen::Vector3d &direction,
                                double max_len,
                                double safe_distance,
                                double step) const;
  bool checkTrajectoryKnownFree(const Trajectory<7> &traj,
                                double safe_distance,
                                double step,
                                bool unknown_as_occupied) const;
  double estimatePathKnownFreeLength(const std::vector<Eigen::Vector3d> &path,
                                     double safe_distance,
                                     double step) const;
  double estimatePathMinClearance(const std::vector<Eigen::Vector3d> &path,
                                  double step) const;
  double estimatePathTurnAngle(const std::vector<Eigen::Vector3d> &path) const;
  SegmentSafetyInfo evaluatePathSegmentSafety(const std::vector<Eigen::Vector3d> &path,
                                              double yaw1,
                                              double yaw2) const;
  SegmentVelocityLimit computeSegmentVelocityLimit(const SegmentSafetyInfo &info) const;
  EdgeSafetyCost estimateHighSpeedEdgeCost(const std::vector<Eigen::Vector3f> &path,
                                           const Eigen::Vector3d &start_vel,
                                           double yaw1,
                                           double yaw2) const;
  void printSafetyMapSummary() const;

  bool YawTrajOpt(double &start_yaw, double &end_yaw, bool is_static, bool use_shorten_path);
  bool YawTrajwithoutOpt(double &start_yaw, double &end_yaw, bool is_static, bool use_shorten_path);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void posCallback(const nav_msgs::OdometryConstPtr &msg);
  bool YawInterpolationwithoutOpt(double &start,
                                  double &end,
                                  std::vector<double> &new_yaw,
                                  std::vector<double> &new_dur,
                                  double &comp_t);
  void YawLookforward(const Trajectory<5> &pos_traj,
                      double &start,
                      double &end,
                      std::vector<double> &new_yaw,
                      std::vector<double> &new_dur,
                      double &comp_t);
  void YawLookforwardwithoutOpt(double &start,
                                double &end,
                                std::vector<double> &new_yaw,
                                std::vector<double> &new_dur,
                                double &comp_t,
                                bool use_short_path);
  void angleLimite(double &angle);
  void calculateTimelb(const std::vector<Eigen::Vector3d> &path2next_goal,
                       const double &current_yaw,
                       const double &goal_yaw,
                       double &time_lb);

  double start_yaw{0.0};
  double end_yaw{0.0};
  double is_static_yaw{false};
  ros::Subscriber goal_sub;
  ros::Subscriber pos_sub;
  ros::Publisher yaw_state_pub;

  LocalTrajData local_data_;
  double max_traj_len_{0.0};
  LIOInterface::Ptr lidar_map_interface_;
  std::unique_ptr<GcopterConfig> gcopter_config_;
  std::unique_ptr<Visualizer> gcopter_viz_;
  BubbleAstar::Ptr bubble_path_finder_;
  ParallelBubbleAstar::Ptr parallel_path_finder_;
  TopoGraph::Ptr topo_graph_;
  GraphVisualizer::Ptr graph_visualizer_;
  FastSearcher::Ptr fast_searcher_;
  bool use_mid360{false};
  double max_ray_length{0.0};
  double fov_up{0.0};
  double fov_down{0.0};
  double lidar_pitch{0.0};

private:
  std::shared_ptr<ros_interface::RosInterface> ros_ptr_;
  std::unique_ptr<GeneralCommitStore> commit_store_;
  std::shared_ptr<traj_opt::ExplorationTrajOpt> exploration_traj_opt_;
  std::shared_ptr<traj_opt::BackupTrajOpt> backup_traj_opt_;
  std::shared_ptr<traj_opt::YawTrajOpt> yaw_traj_opt_;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_;
  std::shared_ptr<general_planner::MapManager> map_manager_;
  std::shared_ptr<general_planner::CorridorGenerator> corridor_generator_;
  std::shared_ptr<geometry_utils::Trajectory> committed_pos_traj_;
  std::shared_ptr<geometry_utils::Trajectory> committed_yaw_traj_;
  std::shared_ptr<geometry_utils::Trajectory> latest_exp_pos_traj_;
  std::shared_ptr<geometry_utils::Trajectory> latest_exp_yaw_traj_;
  std::vector<Eigen::Vector3f> last_frontend_path_;
  bool rog_map_updated_{false};
  bool committed_stop_active_{false};
};
} // namespace fast_planner
