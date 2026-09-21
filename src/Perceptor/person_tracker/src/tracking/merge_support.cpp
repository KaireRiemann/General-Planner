#include "tracking/merge_support.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace person_tracker
{

namespace
{

double wrappedAngleDistance(double left, double right)
{
  return std::abs(std::atan2(std::sin(left - right), std::cos(left - right)));
}

Eigen::Vector3d pointPosition(const pcl::PointXYZ & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

}  // namespace

GeometricSupportMatcher::GeometricSupportMatcher(SupportMatcherParameters parameters)
: parameters_(std::move(parameters))
{
}

void GeometricSupportMatcher::setParameters(const SupportMatcherParameters & parameters)
{
  parameters_ = parameters;
}

TrackSupportTemplate GeometricSupportMatcher::makeTemplate(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices,
  const Eigen::Vector3d & center, double stamp,
  const SensorObservation & observation, double quality) const
{
  TrackSupportTemplate support_template;
  support_template.stamp = stamp;
  support_template.original_center = center;
  support_template.sensor_range = observation.range;
  support_template.sensor_bearing = observation.bearing;
  support_template.quality = std::clamp(quality, 0.0, 1.0);
  support_template.local_points.reserve(indices.size());
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud.size()) {
      continue;
    }
    const Eigen::Vector3d local = pointPosition(cloud.points[static_cast<std::size_t>(index)]) - center;
    support_template.local_points.push_back(pcl::PointXYZ(
      static_cast<float>(local.x()), static_cast<float>(local.y()),
      static_cast<float>(local.z())));
  }
  support_template.local_points.width = support_template.local_points.size();
  support_template.local_points.height = 1;
  support_template.local_points.is_dense = true;
  support_template.original_point_count =
    static_cast<int>(support_template.local_points.size());
  return support_template;
}

void GeometricSupportMatcher::appendTemplate(
  TrackSupportMemory & memory, TrackSupportTemplate support_template,
  std::size_t maximum_templates) const
{
  if (memory.frozen || support_template.local_points.empty() || maximum_templates == 0U) {
    return;
  }
  memory.last_update_stamp = support_template.stamp;
  memory.templates.push_back(std::move(support_template));
  while (memory.templates.size() > maximum_templates) {
    memory.templates.pop_front();
  }
}

std::vector<const TrackSupportTemplate *> GeometricSupportMatcher::selectTemplates(
  const TrackSupportMemory & memory, const SensorObservation & observation) const
{
  struct RankedTemplate
  {
    double cost{0.0};
    const TrackSupportTemplate * support_template{nullptr};
  };
  std::vector<RankedTemplate> ranked;
  ranked.reserve(memory.templates.size());
  for (std::size_t index = 0; index < memory.templates.size(); ++index) {
    const auto & support_template = memory.templates[index];
    double cost = 0.35 * (1.0 - std::clamp(support_template.quality, 0.0, 1.0));
    if (observation.valid) {
      cost += std::abs(observation.range - support_template.sensor_range) /
        std::max(0.1, parameters_.view_range_scale);
      cost += wrappedAngleDistance(observation.bearing, support_template.sensor_bearing) /
        std::max(0.1, parameters_.view_bearing_scale);
    } else {
      cost += 0.02 * static_cast<double>(memory.templates.size() - 1U - index);
    }
    ranked.push_back({cost, &support_template});
  }
  std::sort(
    ranked.begin(), ranked.end(),
    [](const RankedTemplate & left, const RankedTemplate & right) {
      return left.cost < right.cost;
    });
  const std::size_t selected_count = std::min(
    ranked.size(), static_cast<std::size_t>(std::max(1, parameters_.selected_templates)));
  std::vector<const TrackSupportTemplate *> selected;
  selected.reserve(selected_count);
  for (std::size_t index = 0; index < selected_count; ++index) {
    selected.push_back(ranked[index].support_template);
  }
  return selected;
}

double GeometricSupportMatcher::pointSupport(
  const Eigen::Vector3d & point, const Eigen::Vector3d & predicted_center,
  const std::vector<const TrackSupportTemplate *> & templates,
  double current_range) const
{
  if (templates.empty()) {
    return 0.0;
  }
  const Eigen::Vector3d local = point - predicted_center;
  double best_squared = std::numeric_limits<double>::infinity();
  for (const auto * support_template : templates) {
    for (const auto & support_point : support_template->local_points.points) {
      const Eigen::Vector3d difference = local - pointPosition(support_point);
      best_squared = std::min(best_squared, difference.squaredNorm());
    }
  }
  const double maximum_distance = std::max(0.01, parameters_.max_support_distance);
  if (best_squared > maximum_distance * maximum_distance) {
    return 0.0;
  }
  const double sigma = std::max(
    0.01, parameters_.support_sigma +
    parameters_.range_sigma_scale * std::max(0.0, current_range));
  return std::exp(-0.5 * best_squared / (sigma * sigma));
}

MergeAssignment GeometricSupportMatcher::assignMergedPoints(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices,
  const Eigen::Vector3d & target_center, const Eigen::Vector3d & distractor_center,
  const TrackSupportMemory & target_memory,
  const TrackSupportMemory & distractor_memory,
  const SensorObservation & target_observation,
  const SensorObservation & distractor_observation) const
{
  MergeAssignment result;
  const auto target_templates = selectTemplates(target_memory, target_observation);
  const auto distractor_templates = selectTemplates(distractor_memory, distractor_observation);
  result.target_selected_templates = static_cast<int>(target_templates.size());
  result.distractor_selected_templates = static_cast<int>(distractor_templates.size());
  result.evidence.reserve(indices.size());

  const double total_weight = std::max(
    1e-6, parameters_.support_weight + parameters_.motion_weight);
  const double motion_sigma = std::max(0.05, parameters_.motion_sigma);
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud.size()) {
      continue;
    }
    const Eigen::Vector3d point = pointPosition(cloud.points[static_cast<std::size_t>(index)]);
    MergePointEvidence point_evidence;
    point_evidence.cloud_index = index;
    point_evidence.target_support = pointSupport(
      point, target_center, target_templates, target_observation.range);
    point_evidence.distractor_support = pointSupport(
      point, distractor_center, distractor_templates, distractor_observation.range);
    const double target_motion_squared =
      (point.head<2>() - target_center.head<2>()).squaredNorm();
    const double distractor_motion_squared =
      (point.head<2>() - distractor_center.head<2>()).squaredNorm();
    const double target_motion = std::exp(
      -0.5 * target_motion_squared / (motion_sigma * motion_sigma));
    const double distractor_motion = std::exp(
      -0.5 * distractor_motion_squared / (motion_sigma * motion_sigma));
    point_evidence.target_score =
      (parameters_.support_weight * point_evidence.target_support +
      parameters_.motion_weight * target_motion) / total_weight;
    point_evidence.distractor_score =
      (parameters_.support_weight * point_evidence.distractor_support +
      parameters_.motion_weight * distractor_motion) / total_weight;

    if (
      std::max(point_evidence.target_support, point_evidence.distractor_support) <
      parameters_.unknown_threshold ||
      std::abs(point_evidence.target_score - point_evidence.distractor_score) <
      parameters_.ambiguity_margin)
    {
      point_evidence.label = MergePointLabel::UNKNOWN;
    } else {
      point_evidence.label = point_evidence.target_score >= point_evidence.distractor_score ?
        MergePointLabel::TARGET : MergePointLabel::DISTRACTOR;
    }
    result.evidence.push_back(point_evidence);
  }

  regularizeLabels(cloud, result.evidence);
  for (const auto & point_evidence : result.evidence) {
    if (point_evidence.label == MergePointLabel::TARGET) {
      result.target_indices.push_back(point_evidence.cloud_index);
    } else if (point_evidence.label == MergePointLabel::DISTRACTOR) {
      result.distractor_indices.push_back(point_evidence.cloud_index);
    } else {
      result.unknown_indices.push_back(point_evidence.cloud_index);
    }
  }
  result.target_evaluation = evaluateTrack(
    result.evidence, MergePointLabel::TARGET, true, target_templates);
  result.distractor_evaluation = evaluateTrack(
    result.evidence, MergePointLabel::DISTRACTOR, false, distractor_templates);
  return result;
}

void GeometricSupportMatcher::regularizeLabels(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  std::vector<MergePointEvidence> & evidence) const
{
  if (
    !parameters_.spatial_enabled || evidence.size() < 3U ||
    parameters_.spatial_iterations <= 0 || parameters_.spatial_smooth_weight <= 0.0)
  {
    return;
  }

  const std::size_t neighbor_count = std::min(
    evidence.size() - 1U, static_cast<std::size_t>(std::max(1, parameters_.spatial_knn)));
  std::vector<std::vector<std::pair<std::size_t, double>>> neighbors(evidence.size());
  const double neighbor_sigma = std::max(0.02, parameters_.spatial_neighbor_sigma);
  for (std::size_t i = 0; i < evidence.size(); ++i) {
    const Eigen::Vector3d point = pointPosition(
      cloud.points[static_cast<std::size_t>(evidence[i].cloud_index)]);
    std::vector<std::pair<double, std::size_t>> distances;
    distances.reserve(evidence.size() - 1U);
    for (std::size_t j = 0; j < evidence.size(); ++j) {
      if (i == j) {
        continue;
      }
      const Eigen::Vector3d neighbor = pointPosition(
        cloud.points[static_cast<std::size_t>(evidence[j].cloud_index)]);
      distances.emplace_back((point - neighbor).squaredNorm(), j);
    }
    std::partial_sort(
      distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(neighbor_count),
      distances.end(),
      [](const auto & left, const auto & right) {return left.first < right.first;});
    neighbors[i].reserve(neighbor_count);
    for (std::size_t k = 0; k < neighbor_count; ++k) {
      const double weight = std::exp(
        -0.5 * distances[k].first / (neighbor_sigma * neighbor_sigma));
      neighbors[i].emplace_back(distances[k].second, weight);
    }
  }

  for (int iteration = 0; iteration < parameters_.spatial_iterations; ++iteration) {
    const auto previous = evidence;
    for (std::size_t i = 0; i < evidence.size(); ++i) {
      if (
        std::max(previous[i].target_support, previous[i].distractor_support) <
        parameters_.unknown_threshold)
      {
        evidence[i].label = MergePointLabel::UNKNOWN;
        continue;
      }
      const double unary_target = 1.0 - previous[i].target_score;
      const double unary_distractor = 1.0 - previous[i].distractor_score;
      double mismatch_target = 0.0;
      double mismatch_distractor = 0.0;
      double mismatch_unknown = 0.0;
      double weight_sum = 0.0;
      for (const auto & neighbor : neighbors[i]) {
        const MergePointLabel label = previous[neighbor.first].label;
        mismatch_target += neighbor.second * (label != MergePointLabel::TARGET);
        mismatch_distractor += neighbor.second * (label != MergePointLabel::DISTRACTOR);
        mismatch_unknown += neighbor.second * (label != MergePointLabel::UNKNOWN);
        weight_sum += neighbor.second;
      }
      weight_sum = std::max(1e-6, weight_sum);
      const double target_energy = unary_target + parameters_.spatial_smooth_weight *
        mismatch_target / weight_sum;
      const double distractor_energy = unary_distractor + parameters_.spatial_smooth_weight *
        mismatch_distractor / weight_sum;
      const double unknown_energy = parameters_.spatial_unknown_cost +
        parameters_.spatial_smooth_weight * mismatch_unknown / weight_sum;

      if (unknown_energy <= std::min(target_energy, distractor_energy)) {
        evidence[i].label = MergePointLabel::UNKNOWN;
      } else if (target_energy <= distractor_energy) {
        evidence[i].label = MergePointLabel::TARGET;
      } else {
        evidence[i].label = MergePointLabel::DISTRACTOR;
      }
    }
  }
}

TrackSupportEvaluation GeometricSupportMatcher::evaluateTrack(
  const std::vector<MergePointEvidence> & evidence, MergePointLabel label,
  bool target, const std::vector<const TrackSupportTemplate *> & templates) const
{
  TrackSupportEvaluation evaluation;
  double support_sum = 0.0;
  for (const auto & point_evidence : evidence) {
    if (point_evidence.label != label) {
      continue;
    }
    const double support = target ?
      point_evidence.target_support : point_evidence.distractor_support;
    support_sum += support;
    ++evaluation.assigned_count;
    if (support >= parameters_.high_support_threshold) {
      ++evaluation.high_confidence_count;
    }
  }
  evaluation.mean_support = evaluation.assigned_count > 0 ?
    support_sum / static_cast<double>(evaluation.assigned_count) : 0.0;
  double expected_points = 0.0;
  for (const auto * support_template : templates) {
    expected_points += static_cast<double>(support_template->original_point_count);
  }
  expected_points = templates.empty() ? 0.0 : expected_points / templates.size();
  evaluation.expected_point_ratio = expected_points > 0.5 ?
    static_cast<double>(evaluation.assigned_count) / expected_points : 0.0;

  const bool visible =
    evaluation.assigned_count >= parameters_.visible_min_points &&
    evaluation.mean_support >= parameters_.visible_min_mean_support &&
    evaluation.high_confidence_count >= parameters_.visible_min_high_confidence_points &&
    evaluation.expected_point_ratio >= parameters_.visible_expected_ratio;
  const bool partial =
    evaluation.assigned_count >= parameters_.partial_min_points &&
    evaluation.mean_support >= parameters_.partial_min_mean_support &&
    evaluation.high_confidence_count >= 1 &&
    evaluation.expected_point_ratio >= parameters_.partial_expected_ratio;
  evaluation.visibility = visible ? TrackVisibility::VISIBLE :
    (partial ? TrackVisibility::PARTIAL_OCCLUDED : TrackVisibility::OCCLUDED);

  const double mean_component = std::clamp(
    evaluation.mean_support /
    std::max(0.05, parameters_.visible_min_mean_support), 0.0, 1.0);
  const double ratio_component = std::clamp(
    evaluation.expected_point_ratio /
    std::max(0.05, parameters_.visible_expected_ratio), 0.0, 1.0);
  const double count_component = std::clamp(
    static_cast<double>(evaluation.high_confidence_count) /
    std::max(1, parameters_.visible_min_high_confidence_points), 0.0, 1.0);
  evaluation.confidence =
    0.45 * mean_component + 0.30 * ratio_component + 0.25 * count_component;
  return evaluation;
}

double GeometricSupportMatcher::clusterSupportScore(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> & indices,
  const Eigen::Vector3d & cluster_center, const TrackSupportMemory & memory,
  const SensorObservation & observation) const
{
  const auto templates = selectTemplates(memory, observation);
  if (templates.empty()) {
    return 0.0;
  }
  double support_sum = 0.0;
  int valid_points = 0;
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud.size()) {
      continue;
    }
    support_sum += pointSupport(
      pointPosition(cloud.points[static_cast<std::size_t>(index)]),
      cluster_center, templates, observation.range);
    ++valid_points;
  }
  return valid_points > 0 ? support_sum / valid_points : 0.0;
}

pcl::PointCloud<pcl::PointXYZ> GeometricSupportMatcher::predictedSupportCloud(
  const TrackSupportMemory & memory, const Eigen::Vector3d & predicted_center,
  const SensorObservation & observation) const
{
  pcl::PointCloud<pcl::PointXYZ> result;
  const auto templates = selectTemplates(memory, observation);
  std::size_t point_count = 0U;
  for (const auto * support_template : templates) {
    point_count += support_template->local_points.size();
  }
  result.reserve(point_count);
  for (const auto * support_template : templates) {
    for (const auto & local_point : support_template->local_points.points) {
      const Eigen::Vector3d world = pointPosition(local_point) + predicted_center;
      result.push_back(pcl::PointXYZ(
        static_cast<float>(world.x()), static_cast<float>(world.y()),
        static_cast<float>(world.z())));
    }
  }
  result.width = result.size();
  result.height = 1;
  result.is_dense = true;
  return result;
}

const char * visibilityName(TrackVisibility visibility)
{
  switch (visibility) {
    case TrackVisibility::VISIBLE:
      return "VISIBLE";
    case TrackVisibility::PARTIAL_OCCLUDED:
      return "PARTIAL";
    case TrackVisibility::OCCLUDED:
      return "OCCLUDED";
  }
  return "UNKNOWN";
}

}  // namespace person_tracker
