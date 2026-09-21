#pragma once

#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace person_tracker
{

enum class MergePointLabel
{
  TARGET,
  DISTRACTOR,
  UNKNOWN
};

enum class TrackVisibility
{
  VISIBLE,
  PARTIAL_OCCLUDED,
  OCCLUDED
};

struct SensorObservation
{
  double range{0.0};
  double bearing{0.0};
  bool valid{false};
};

// A clean observation is stored in the person's local coordinates. It is a
// geometric support hypothesis, never a synthetic lidar measurement.
struct TrackSupportTemplate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double stamp{0.0};
  Eigen::Vector3d original_center{Eigen::Vector3d::Zero()};
  pcl::PointCloud<pcl::PointXYZ> local_points;
  double sensor_range{0.0};
  double sensor_bearing{0.0};
  double quality{0.0};
  int original_point_count{0};
};

struct TrackSupportMemory
{
  std::deque<TrackSupportTemplate> templates;
  bool frozen{false};
  double last_update_stamp{-std::numeric_limits<double>::infinity()};
};

struct SupportMatcherParameters
{
  int selected_templates{3};
  double support_sigma{0.15};
  double range_sigma_scale{0.005};
  double max_support_distance{0.35};
  double support_weight{1.0};
  double motion_weight{0.25};
  double motion_sigma{0.45};
  double unknown_threshold{0.18};
  double ambiguity_margin{0.08};
  double view_range_scale{2.0};
  double view_bearing_scale{0.8};

  bool spatial_enabled{true};
  int spatial_knn{8};
  double spatial_neighbor_sigma{0.22};
  double spatial_smooth_weight{0.30};
  int spatial_iterations{2};
  double spatial_unknown_cost{0.55};

  int visible_min_points{5};
  int partial_min_points{3};
  double visible_min_mean_support{0.30};
  double partial_min_mean_support{0.20};
  double high_support_threshold{0.55};
  int visible_min_high_confidence_points{2};
  double visible_expected_ratio{0.30};
  double partial_expected_ratio{0.12};
};

struct TrackSupportEvaluation
{
  int assigned_count{0};
  int high_confidence_count{0};
  double mean_support{0.0};
  double expected_point_ratio{0.0};
  double confidence{0.0};
  TrackVisibility visibility{TrackVisibility::OCCLUDED};
};

struct MergePointEvidence
{
  int cloud_index{-1};
  double target_support{0.0};
  double distractor_support{0.0};
  double target_score{0.0};
  double distractor_score{0.0};
  MergePointLabel label{MergePointLabel::UNKNOWN};
};

struct MergeAssignment
{
  std::vector<int> target_indices;
  std::vector<int> distractor_indices;
  std::vector<int> unknown_indices;
  std::vector<MergePointEvidence> evidence;
  TrackSupportEvaluation target_evaluation;
  TrackSupportEvaluation distractor_evaluation;
  int target_selected_templates{0};
  int distractor_selected_templates{0};
};

class GeometricSupportMatcher
{
public:
  explicit GeometricSupportMatcher(SupportMatcherParameters parameters = {});

  void setParameters(const SupportMatcherParameters & parameters);

  TrackSupportTemplate makeTemplate(
    const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices,
    const Eigen::Vector3d & center, double stamp,
    const SensorObservation & observation, double quality) const;

  void appendTemplate(
    TrackSupportMemory & memory, TrackSupportTemplate support_template,
    std::size_t maximum_templates) const;

  MergeAssignment assignMergedPoints(
    const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices,
    const Eigen::Vector3d & target_center, const Eigen::Vector3d & distractor_center,
    const TrackSupportMemory & target_memory,
    const TrackSupportMemory & distractor_memory,
    const SensorObservation & target_observation,
    const SensorObservation & distractor_observation) const;

  double clusterSupportScore(
    const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices,
    const Eigen::Vector3d & cluster_center, const TrackSupportMemory & memory,
    const SensorObservation & observation) const;

  pcl::PointCloud<pcl::PointXYZ> predictedSupportCloud(
    const TrackSupportMemory & memory, const Eigen::Vector3d & predicted_center,
    const SensorObservation & observation) const;

private:
  std::vector<const TrackSupportTemplate *> selectTemplates(
    const TrackSupportMemory & memory, const SensorObservation & observation) const;

  double pointSupport(
    const Eigen::Vector3d & point, const Eigen::Vector3d & predicted_center,
    const std::vector<const TrackSupportTemplate *> & templates,
    double current_range) const;

  TrackSupportEvaluation evaluateTrack(
    const std::vector<MergePointEvidence> & evidence, MergePointLabel label,
    bool target, const std::vector<const TrackSupportTemplate *> & templates) const;

  void regularizeLabels(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    std::vector<MergePointEvidence> & evidence) const;

  SupportMatcherParameters parameters_;
};

const char * visibilityName(TrackVisibility visibility);

}  // namespace person_tracker
