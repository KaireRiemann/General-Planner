#include "person_tracker/ObservationDebug.h"
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tracking_detector/BoundingBoxes.h>
#include <std_srvs/Trigger.h>

#include "person_tracker/TrackedObjectArray.h"
#include "person_tracker/TargetStatus.h"
#include "person_tracker/PredictionStatus.h"
#include "person_tracker/FusionStatus.h"
#include "person_tracker/GroundEstimate.h"
#include "fusion/fusion_state_machine.hpp"
#include "lidar/ground_height_estimator.hpp"
#include "yolo/yolo_projection.hpp"

#include "lidar/DBSCAN_kdtree.h"
#include "third_party/Hungarian.h"
#include "third_party/ikd_Tree.h"
#include "third_party/ikd_Tree_impl.h"
#include "tracking/merge_support.hpp"
#include "tracking/target_ekf.hpp"

namespace person_tracker
{

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct Parameters
{
  std::string input_cloud_topic{"/cloud_registered"};
  std::string odom_topic{"/Odometry"};
  std::string world_frame{"camera_init"};
  bool cloud_is_registered{true};
  double max_odom_age{0.5};

  Eigen::Vector3d local_update_range{5.0, 5.0, 2.5};
  double min_range{0.3};
  double max_range{8.0};

  double map_resolution{0.10};
  double map_hit_probability{0.85};
  double map_miss_probability{0.30};
  double map_min_probability{0.12};
  double map_max_probability{0.98};
  double map_occupied_probability{0.80};
  double map_inflation_xy{0.10};
  double map_inflation_z_up{0.10};
  double map_inflation_z_down{0.10};
  bool map_current_frame_only{true};

  bool remove_ground{true};
  double ground_distance_threshold{0.12};
  double ground_eps_angle_deg{15.0};
  int ground_min_inliers{80};

  GroundHeightParameters ground_height;
  double ground_height_minimum_confidence{0.25};
  double ground_height_smoothing{0.35};
  double ground_height_max_step{0.25};

  int recent_voxel_frames{5};
  bool cluster_in_xy{true};
  int dbscan_core_points{3};
  double cluster_tolerance{0.30};
  int min_cluster_size{5};
  int max_cluster_size{2500};
  bool interaction_refinement_enabled{true};
  double interaction_refinement_tolerance{0.20};
  int interaction_refinement_core_points{2};
  int interaction_refinement_min_cluster_size{4};
  double interaction_refinement_min_coverage{0.45};

  bool dynamic_only{false};
  double dynamic_distance_threshold{0.06};
  double dynamic_variance_threshold{0.8};
  double dynamic_max_distance{2.0};
  double track_keep_distance{0.8};
  double track_keep_min_speed{0.1};
  int background_history_frames{25};
  int background_confirm_frames{30};

  bool human_filter_enabled{true};
  double human_min_height{0.4};
  double human_max_height{2.3};
  double human_min_width{0.15};
  double human_max_width{1.2};
  double human_measurement_tolerance{0.45};

  bool manual_selection_enabled{true};
  std::string clicked_point_topic{"/clicked_point"};
  std::string target_pose_topic{"/target_select"};
  double selection_max_click_distance{0.60};
  double selection_association_max_distance{0.60};
  double selection_max_size_change{0.60};
  int selection_max_missed_frames{15};
  bool selection_roi_fallback_enabled{true};
  double selection_roi_radius_xy{0.35};
  double selection_roi_half_height{1.00};
  int selection_roi_min_points{2};
  int selection_background_static_min_age_frames{20};
  double selection_tracking_background_distance{0.06};
  bool selection_reacquisition_enabled{true};
  double selection_reacquisition_max_distance{1.20};
  double selection_reacquisition_immediate_distance{0.45};
  int selection_reacquisition_confirm_frames{2};
  double selection_reacquisition_confirmation_radius{0.45};
  double selection_reacquisition_max_position_correction{0.35};
  double selection_reacquisition_velocity_blend{0.10};
  double selection_reacquisition_direction_weight{0.35};
  double selection_reacquisition_max_z_difference{1.00};
  double selection_reacquisition_preferred_center_clearance{0.30};
  double selection_reacquisition_clearance_weight{4.0};
  double selection_max_speed{3.5};
  double selection_measurement_velocity_blend{0.70};
  double selection_velocity_min_displacement{0.12};
  double selection_velocity_max_dt{0.50};
  bool selection_stop_hypothesis_enabled{false};
  double selection_stop_min_speed{0.40};
  double selection_stop_max_last_distance{0.35};
  double selection_stop_hypothesis_penalty{0.10};
  double selection_stop_velocity_damping{0.10};
  bool selection_prediction_gate_only{true};
  bool selection_fixed_body_size_enabled{true};
  Eigen::Vector3d selection_fixed_body_size{0.60, 0.60, 1.70};
  bool selection_adaptive_size_enabled{true};
  double selection_size_smoothing{0.12};
  double selection_size_max_relative_step{0.25};
  double selection_size_compatibility_min_ratio{0.45};
  double selection_size_compatibility_max_ratio{1.80};

  // Crossing/occlusion handling for manual single-target publication. Other
  // people are maintained as private auxiliary tracks and never enter /tracks.
  bool crossing_enabled{true};
  double crossing_shadow_spawn_radius{3.0};
  double crossing_shadow_association_distance{1.0};
  int crossing_shadow_max_missed_frames{20};
  int crossing_max_shadow_tracks{4};
  bool crossing_shadow_dynamic_only{true};
  int crossing_shadow_min_confirmed_hits{3};
  double crossing_shadow_min_speed{0.10};
  double crossing_interaction_distance{1.30};
  double crossing_seed_support_radius{0.45};
  int crossing_seed_min_points{3};
  double crossing_seed_assignment_margin{0.03};
  double crossing_merged_min_extent_xy{0.60};
  double crossing_min_track_separation{0.25};
  int crossing_merge_confirm_frames{2};
  int crossing_occlusion_max_frames{25};
  double crossing_split_min_separation{0.35};
  bool crossing_split_update_enabled{true};
  double crossing_split_update_max_correction{0.30};
  double crossing_split_velocity_blend{0.15};
  double crossing_recovery_gate{1.50};
  int crossing_hypothesis_confirm_frames{4};
  int crossing_hypothesis_max_frames{8};
  double crossing_hypothesis_score_margin{0.25};
  double crossing_hypothesis_timeout_min_margin{0.15};
  double crossing_position_continuity_weight{0.35};
  double crossing_signature_size_weight{0.35};
  double crossing_signature_points_weight{0.08};

  // V1 is merge-only. NORMAL tracking remains byte-for-byte on the v0 path.
  bool merge_v1_enabled{true};
  bool merge_v1_debug_enabled{true};
  int merge_v1_max_templates{8};
  int merge_v1_min_templates{1};
  int merge_v1_min_template_points{5};
  double merge_v1_min_template_quality{0.55};
  double merge_v1_max_template_size_ratio{1.8};
  double merge_v1_min_template_interval{0.20};
  int merge_v1_resume_delay_frames{2};
  double merge_v1_split_identity_support_weight{0.25};
  SupportMatcherParameters merge_v1_matcher;

  double association_max_distance{1.0};
  int min_confirmed_hits{2};
  int max_missed_frames{5};
  double acceleration_noise{2.0};
  bool body_center_enabled{false};
  double measurement_noise{0.15};
  double acceleration_smoothing{0.45};
  double turn_rate_smoothing{0.45};
  double maximum_acceleration{8.0};
  double maximum_turn_rate{4.0};
  bool publish_unconfirmed{true};
};

struct Detection
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double body_fit_radius{0.0};
  Eigen::Vector2d body_center_offset{Eigen::Vector2d::Zero()};
  pcl::PointIndices indices;
  double mean_background_distance{0.0};
  double normalized_distance_variance{0.0};
  bool dynamic{false};
  bool indices_from_current_cloud{false};
};

struct DetectionStats
{
  std::size_t shape_rejected{0};
  std::size_t dynamic_rejected{0};
};

struct VoxelIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex & index) const;
};

struct OccupancyCell
{
  double log_odds{0.0};
  std::uint64_t last_hit_frame{0};
};

struct BackgroundCandidate
{
  PointType point;
  int consecutive_hits{0};
  std::uint64_t last_seen_frame{0};
};

struct AuxiliaryTrack
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::shared_ptr<TargetEkf> filter;
  Eigen::Vector3d observed_size{Eigen::Vector3d::Zero()};
  double observed_point_count{0.0};
  TrackSupportMemory support_memory;
  int last_candidate_index{-1};
  double last_candidate_stamp{-1.0};
};

struct IdentityHypothesis
{
  std::shared_ptr<TargetEkf> target_filter;
  std::shared_ptr<TargetEkf> distractor_filter;
  double cumulative_score{0.0};
  int frames{0};
  int last_target_candidate{-1};
  int last_distractor_candidate{-1};
  Detection last_target_detection;
};

class MappingRos
{
public:
  MappingRos();
  ~MappingRos();

private:
  void declareAndLoadParameters();
  void configurePredictionModel();
  void validateParameters();

  void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
  void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr msg);
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg);

  void loadYoloParameters();
  void boxesCallback(const tracking_detector::BoundingBoxes::ConstPtr & msg);
  void cameraOdomCallback(const nav_msgs::Odometry::ConstPtr & msg);
  bool appendOdometry(const nav_msgs::Odometry::ConstPtr & msg,
    std::deque<nav_msgs::Odometry::ConstPtr> & history);
  bool poseAt(const std::deque<nav_msgs::Odometry::ConstPtr> & history,
    const ros::Time & stamp, Eigen::Isometry3d & pose) const;
  struct CameraVisibility
  {
    bool projection_valid{false};
    bool in_fov{false};
    double u{0.0};
    double v{0.0};
    double depth{0.0};
  };
  CameraVisibility cameraVisibility(
    const Eigen::Vector3d & world_position, const ros::Time & stamp) const;
  Eigen::Vector3d predictedTargetAt(
    const TargetEkf & tracker, const ros::Time & stamp) const;
  void observeFusion(
    SemanticEvidence evidence, const ros::Time & stamp, const std::string & reason,
    const std::optional<Eigen::Vector2d> & candidate = std::nullopt,
    double semantic_distance = -1.0);
  void expireFusionEvidence(const ros::Time & stamp);
  void publishFusionStatus(const ros::Time & stamp);
  void clearYoloDebugClouds(const ros::Time & stamp, bool clear_candidate);
  void cacheScan(const PointCloudPtr & cloud, const std_msgs::Header & header);
  void tryInitializeFromYolo(const ros::Time & current_stamp);
  void armYoloReacquisition(const std::string & reason);
  void cancelYoloReacquisition();
  void removeSeedFromBackground(const PointCloudPtr & cloud, const std::vector<int> & indices);
  int deleteBackgroundVoxels(const PointVector & points);
  bool resetTarget(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & response);
  void publishTargetState(const std_msgs::Header & header);
  void publishTargetStatus(const ros::Time & stamp);
  void publishTargetPath(const std_msgs::Header & header, bool valid);
  void targetWatchdog(const ros::TimerEvent &);
  void expireTarget(const ros::Time & stamp);

  PointCloudPtr preprocessCloud(const sensor_msgs::PointCloud2 & msg) const;
  PointCloudPtr voxelizeCloud(const PointCloudPtr & cloud) const;
  PointCloudPtr removeGroundPlane(const PointCloudPtr & cloud) const;
  void updateVoxelMap(const PointCloudPtr & cloud);
  VoxelIndex positionToVoxel(const Eigen::Vector3d & position) const;
  Eigen::Vector3d voxelToPosition(const VoxelIndex & index) const;
  bool voxelInLocalMap(const VoxelIndex & index) const;
  PointCloudPtr extractOccupiedCloud() const;
  PointCloudPtr extractForegroundCloud(const PointCloudPtr & cloud) const;
  double targetPointWeight(const PointType & point) const;
  double backgroundClearance(const Eigen::Vector3d & position) const;
  void anchorTargetHeight(
    Eigen::Vector3d & measurement, const PointCloudPtr & ground_cloud,
    const ros::Time & stamp, double reference_center_z, bool initialize);
  void publishGroundEstimate(
    const GroundHeightEstimate & estimate, double target_z,
    const PointCloudPtr & cloud, const ros::Time & stamp);
  void resetGroundHeight();
  PointCloudPtr buildInflatedCloud(const PointCloudPtr & occupied) const;
  std::vector<pcl::PointIndices> clusterCloud(const PointCloudPtr & cloud);
  std::vector<pcl::PointIndices> refineInteractionClusters(
    const PointCloudPtr & cloud, const std::vector<pcl::PointIndices> & coarse_clusters) const;
  std::vector<Detection> buildDetections(
    const PointCloudPtr & cloud, const std::vector<pcl::PointIndices> & clusters,
    DetectionStats * stats, bool apply_detection_filters) const;

  bool looksLikePerson(const Eigen::Vector3d & size) const;
  bool compatibleTargetSize(
    const Eigen::Vector3d & measured, const Eigen::Vector3d & reference) const;
  bool isNearMovingTrack(const Eigen::Vector3d & position) const;
  void predictTrackers(double stamp_sec);
  void updateTrackers(std::vector<Detection> & detections, double stamp_sec);
  std::vector<Detection> updateManualTarget(
    const std::vector<Detection> & candidates, const PointCloudPtr & candidate_cloud,
    const PointCloudPtr & current_cloud, double stamp_sec);
  bool buildRoiDetection(
    const PointCloudPtr & cloud, const Eigen::Vector3d & center,
    double radius_xy, double half_height, Detection & detection) const;
  void rememberBodyRadius(const Detection & detection, int target_id, double stamp_sec);
  void estimateBodyCenter(const PointCloudPtr & cloud, Detection & detection) const;
  bool buildDetectionFromIndices(
    const PointCloudPtr & cloud, const std::vector<int> & indices,
    Detection & detection) const;
  double seedDistanceSquared(
    const PointType & point, const TargetEkf & tracker) const;
  bool detectionSupportsTrack(
    const Detection & detection, const PointCloudPtr & cloud,
    const TargetEkf & tracker) const;
  bool splitMergedDetection(
    const Detection & merged, const PointCloudPtr & cloud,
    const TargetEkf & target, const TargetEkf & distractor,
    Detection & target_part, Detection & distractor_part) const;
  MergeAssignment splitMergedDetectionV1(
    const Detection & merged, const PointCloudPtr & cloud,
    const TargetEkf & target, const TargetEkf & distractor,
    const TrackSupportMemory & target_memory,
    const TrackSupportMemory & distractor_memory) const;
  double crossingAssociationCost(
    const TargetEkf & tracker, const Detection & detection,
    const Eigen::Vector3d & signature_size, double signature_points) const;
  bool auxiliaryTrackReliable(const AuxiliaryTrack & auxiliary) const;
  int closestAuxiliaryTrack() const;
  int findMergedCandidate(
    const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
    int auxiliary_index) const;
  bool findRecoveryPair(
    const std::vector<Detection> & candidates, const TargetEkf & target,
    const TargetEkf & distractor, int & first, int & second) const;
  bool handleCrossingState(
    const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
    double stamp_sec, std::vector<Detection> & selected_detection);
  void startIdentityHypotheses(
    const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
    int first, int second,
    int auxiliary_index, double stamp_sec);
  void advanceIdentityHypotheses(
    const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
    double stamp_sec,
    std::vector<Detection> & selected_detection);
  void updateAuxiliaryTracks(
    const std::vector<Detection> & candidates,
    const std::unordered_set<std::size_t> & excluded_candidates, double stamp_sec);
  void clearCrossingState();
  void updateObservedSignature(
    Eigen::Vector3d & size_signature, double & point_signature,
    const Detection & detection) const;
  SensorObservation sensorObservation(const Eigen::Vector3d & center) const;
  bool supportMemoryReady(const TrackSupportMemory & memory) const;
  double identityAssociationCost(
    const TargetEkf & tracker, const Detection & detection,
    const Eigen::Vector3d & signature_size, double signature_points,
    const PointCloudPtr & cloud, const TrackSupportMemory & memory) const;
  void updateCleanSupportMemories(
    const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
    double stamp_sec);
  void setSupportMemoriesFrozen(bool frozen);
  void resetMergeDebug();
  void fillMergeDebug(
    const Detection & merged, const PointCloudPtr & cloud,
    const MergeAssignment & assignment, const TargetEkf & target,
    const TargetEkf & distractor, const TrackSupportMemory & target_memory,
    const TrackSupportMemory & distractor_memory);
  void publishMergeDebug(const std_msgs::Header & header);
  bool insideTargetProtection(const Eigen::Vector3d & position) const;
  void removeTargetFromTransientBackground();
  void updateBackground(const PointCloudPtr & cloud);
  void resetTrackingState();

  void publishPointCloud(
    const PointCloud & cloud, const std_msgs::Header & header,
    const ros::Publisher & publisher) const;
  void publishClusterCloud(
    const PointCloudPtr & cloud, const std::vector<pcl::PointIndices> & clusters,
    const std_msgs::Header & header) const;
  void publishTracks(const std_msgs::Header & header);
  void publishMarkers(const std_msgs::Header & header) const;

  template<typename T>
  T readParameter(const std::string & name, const T & fallback) const
  {
    T value;
    nh_.param<T>(name, value, fallback);
    return value;
  }

  ros::NodeHandle nh_;
  Parameters params_;
  bool yolo_enabled_{true};
  bool yolo_armed_{true};
  bool target_ever_selected_{false};
  std::string target_label_{"car"};
  std::string boxes_topic_{"/yoloe/plot"};
  std::string camera_odom_topic_;
  ProjectionParameters projection_;
  Eigen::Isometry3d body_from_camera_{Eigen::Isometry3d::Identity()};
  double yolo_min_probability_{0.4};
  double yolo_max_age_{0.8};
  double yolo_cloud_slop_{0.12};
  double history_duration_{2.0};
  double yolo_confirmation_distance_{0.5};
  int yolo_confirmation_frames_{2};
  bool yolo_reacquisition_enabled_{true};
  double yolo_reacquisition_max_distance_{2.5};
  double yolo_reacquisition_max_z_difference_{0.8};
  bool yolo_continuous_validation_enabled_{true};
  double yolo_continuous_validation_max_age_{0.30};
  double yolo_continuous_validation_distance_{0.60};
  int yolo_confirmation_hits_{0};
  Eigen::Vector3d yolo_candidate_position_{Eigen::Vector3d::Zero()};
  ros::Time yolo_candidate_stamp_, yolo_processed_stamp_;
  bool yolo_reacquisition_active_{false};
  bool yolo_reacquisition_reference_valid_{false};
  int yolo_reacquisition_target_id_{-1};
  Eigen::Vector3d yolo_reacquisition_reference_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d yolo_reacquisition_size_{Eigen::Vector3d::Zero()};
  bool yolo_semantic_observation_valid_{false};
  Eigen::Vector3d yolo_semantic_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d yolo_semantic_size_{Eigen::Vector3d::Zero()};
  ros::Time yolo_semantic_stamp_;
  bool fusion_enabled_{true};
  bool lidar_evidence_policy_{false};
  tracking_detector::BoundingBoxes::ConstPtr evidence_boxes_;
  double fusion_camera_margin_pixels_{8.0};
  double fusion_consistent_distance_{0.45};
  double fusion_conflict_distance_{0.70};
  double fusion_reselection_max_distance_{2.5};
  FusionStateMachine fusion_state_machine_;
  bool fusion_camera_projection_valid_{false};
  bool fusion_predicted_in_fov_{false};
  bool fusion_yolo_target_present_{false};
  bool fusion_yolo_component_valid_{false};
  bool fusion_candidate_eligible_{false};
  double fusion_semantic_distance_{-1.0};
  ros::Time fusion_evidence_stamp_;
  std::string fusion_reason_{"waiting for target"};
  double max_cloud_age_{0.5};
  double max_prediction_publish_age_{2.0};
  double identity_timeout_{6.0};
  int last_target_id_{-1};
  ros::Time output_last_observation_stamp_;
  std::deque<nav_msgs::Odometry::ConstPtr> odom_history_, camera_odom_history_;
  tracking_detector::BoundingBoxes::ConstPtr pending_boxes_;
  ros::WallTime pending_boxes_received_;
  struct CachedScan {
    std_msgs::Header header;
    PointCloudPtr cloud;
    ros::WallTime received;
  };
  std::deque<CachedScan> scan_history_;
  ros::Subscriber boxes_sub_, camera_odom_sub_;
  double path_horizon_{1.0}, path_step_{0.25};
  ros::Publisher target_path_pub_, target_valid_pub_, observation_age_pub_;
  ros::Publisher prediction_status_pub_;
  ros::Publisher observation_debug_pub_;
  ros::Publisher yolo_candidate_pub_, yolo_seed_pub_, target_odom_pub_, target_status_pub_;
  ros::Publisher fusion_status_pub_;
  ros::ServiceServer reset_target_service_;
  ros::Timer target_watchdog_;
  Eigen::Vector3d odom_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond odom_orientation_{Eigen::Quaterniond::Identity()};
  ros::Time odom_stamp_;
  bool has_odom_{false};
  double last_cloud_stamp_sec_{-1.0};
  int body_radius_target_id_{-1};
  double body_radius_{0.0};
  double body_radius_stamp_{-1.0};
  Eigen::Vector2d body_center_offset_{Eigen::Vector2d::Zero()};

  double hit_log_odds_{0.0};
  double miss_log_odds_{0.0};
  double min_log_odds_{0.0};
  double max_log_odds_{0.0};
  double occupied_log_odds_{0.0};
  double unknown_log_odds_{0.0};
  std::uint64_t cloud_frame_index_{0};
  std::unordered_map<VoxelIndex, OccupancyCell, VoxelIndexHash> occupancy_map_;
  DBSCANKdtreeCluster<PointType> dbscan_;
  std::shared_ptr<KD_TREE<PointType>> background_tree_;
  std::deque<PointVector> background_chunks_;
  std::unordered_map<VoxelIndex, BackgroundCandidate, VoxelIndexHash> background_candidates_;
  std::unordered_map<VoxelIndex, std::uint64_t, VoxelIndexHash> background_last_seen_;
  std::unordered_map<VoxelIndex, std::uint64_t, VoxelIndexHash> background_first_seen_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> persistent_background_indices_;

  GroundHeightEstimator ground_height_estimator_;
  bool has_filtered_ground_height_{false};
  double filtered_ground_height_{0.0};
  GroundHeightEstimate latest_ground_estimate_;

  std::vector<std::shared_ptr<TargetEkf>> trackers_;
  std::vector<AuxiliaryTrack> auxiliary_tracks_;
  std::vector<IdentityHypothesis> identity_hypotheses_;
  GeometricSupportMatcher merge_v1_matcher_;
  TrackSupportMemory target_support_memory_;
  int support_memory_resume_countdown_{0};
  int next_track_id_{0};
  bool pending_target_click_{false};
  bool pending_click_xy_only_{false};
  bool manual_target_selected_{false};
  Eigen::Vector3d clicked_position_{Eigen::Vector3d::Zero()};
  bool has_last_target_measurement_{false};
  Eigen::Vector3d last_target_measurement_position_{Eigen::Vector3d::Zero()};
  double last_target_measurement_stamp_sec_{-1.0};
  Eigen::Vector3d pending_reacquisition_position_{Eigen::Vector3d::Zero()};
  double pending_reacquisition_stamp_sec_{-1.0};
  int pending_reacquisition_frames_{0};
  Eigen::Vector3d target_observed_size_{Eigen::Vector3d::Zero()};
  double target_observed_point_count_{0.0};
  bool crossing_occlusion_active_{false};
  int crossing_occlusion_frames_{0};
  int crossing_auxiliary_index_{-1};
  int crossing_pending_merge_frames_{0};
  int crossing_pending_auxiliary_index_{-1};

  bool merge_debug_active_{false};
  bool merge_debug_was_published_{false};
  Eigen::Vector3d merge_debug_target_prediction_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d merge_debug_distractor_prediction_{Eigen::Vector3d::Zero()};
  TrackVisibility merge_debug_target_visibility_{TrackVisibility::OCCLUDED};
  TrackVisibility merge_debug_distractor_visibility_{TrackVisibility::OCCLUDED};
  PointCloud merge_debug_target_support_;
  PointCloud merge_debug_distractor_support_;
  PointCloud merge_debug_merged_;
  PointCloud merge_debug_target_assigned_;
  PointCloud merge_debug_distractor_assigned_;
  PointCloud merge_debug_unknown_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber target_pose_sub_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher voxel_map_pub_;
  ros::Publisher inflated_voxel_map_pub_;
  ros::Publisher detection_voxel_pub_;
  ros::Publisher foreground_cloud_pub_;
  ros::Publisher dynamic_cloud_pub_;
  ros::Publisher cluster_cloud_pub_;
  ros::Publisher merge_target_support_pub_;
  ros::Publisher merge_distractor_support_pub_;
  ros::Publisher merge_cluster_pub_;
  ros::Publisher merge_target_assigned_pub_;
  ros::Publisher merge_distractor_assigned_pub_;
  ros::Publisher merge_unknown_pub_;
  ros::Publisher tracks_pub_;
  ros::Publisher markers_pub_;
  ros::Publisher ground_support_pub_;
  ros::Publisher ground_estimate_pub_;
};

}  // namespace person_tracker
