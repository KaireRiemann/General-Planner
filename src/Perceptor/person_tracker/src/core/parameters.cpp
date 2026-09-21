#include "core/mapping_manager.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace person_tracker
{

void MappingRos::declareAndLoadParameters()
{
  params_.body_center_enabled = readParameter<bool>("observation/body_center_enabled", false);
  params_.input_cloud_topic = readParameter<std::string>(
    "input_cloud_topic", params_.input_cloud_topic);
  params_.odom_topic = readParameter<std::string>("odom_topic", params_.odom_topic);
  params_.world_frame = readParameter<std::string>("world_frame", params_.world_frame);
  params_.cloud_is_registered = readParameter<bool>(
    "cloud_is_registered", params_.cloud_is_registered);
  params_.max_odom_age = readParameter<double>("max_odom_age", params_.max_odom_age);

  const auto local_range = readParameter<std::vector<double>>(
    "grid_map/local_update_range", {5.0, 5.0, 2.5});
  if (local_range.size() == 3U) {
    params_.local_update_range = Eigen::Vector3d(local_range[0], local_range[1], local_range[2]);
  } else {
    ROS_WARN( "local_update_range must have exactly three values; using defaults");
  }
  params_.min_range = readParameter<double>("min_range", params_.min_range);
  params_.max_range = readParameter<double>("max_range", params_.max_range);
  params_.map_resolution = readParameter<double>(
    "grid_map/resolution", params_.map_resolution);
  params_.map_hit_probability = readParameter<double>(
    "grid_map/p_hit", params_.map_hit_probability);
  params_.map_miss_probability = readParameter<double>(
    "grid_map/p_miss", params_.map_miss_probability);
  params_.map_min_probability = readParameter<double>(
    "grid_map/p_min", params_.map_min_probability);
  params_.map_max_probability = readParameter<double>(
    "grid_map/p_max", params_.map_max_probability);
  params_.map_occupied_probability = readParameter<double>(
    "grid_map/p_occ", params_.map_occupied_probability);
  params_.map_inflation_xy = readParameter<double>(
    "grid_map/inflation_xy", params_.map_inflation_xy);
  params_.map_inflation_z_up = readParameter<double>(
    "grid_map/inflation_z_up", params_.map_inflation_z_up);
  params_.map_inflation_z_down = readParameter<double>(
    "grid_map/inflation_z_down", params_.map_inflation_z_down);
  params_.map_current_frame_only = readParameter<bool>(
    "grid_map/current_frame_only", params_.map_current_frame_only);

  params_.remove_ground = readParameter<bool>("ground/remove", params_.remove_ground);
  params_.ground_distance_threshold = readParameter<double>(
    "ground/distance_threshold", params_.ground_distance_threshold);
  params_.ground_eps_angle_deg = readParameter<double>(
    "ground/eps_angle_deg", params_.ground_eps_angle_deg);
  params_.ground_min_inliers = readParameter<int>(
    "ground/min_inliers", params_.ground_min_inliers);

  params_.ground_height.enabled = readParameter<bool>(
    "ground_height/enabled", params_.ground_height.enabled);
  params_.ground_height.person_center_height = readParameter<double>(
    "ground_height/person_center_height", params_.ground_height.person_center_height);
  params_.ground_height.search_radius = readParameter<double>(
    "ground_height/search_radius", params_.ground_height.search_radius);
  params_.ground_height.normal_radius = readParameter<double>(
    "ground_height/normal_radius", params_.ground_height.normal_radius);
  params_.ground_height.vertical_search_range = readParameter<double>(
    "ground_height/vertical_search_range", params_.ground_height.vertical_search_range);
  params_.ground_height.radial_sigma = readParameter<double>(
    "ground_height/radial_sigma", params_.ground_height.radial_sigma);
  params_.ground_height.height_sigma = readParameter<double>(
    "ground_height/height_sigma", params_.ground_height.height_sigma);
  params_.ground_height.minimum_normal_z = readParameter<double>(
    "ground_height/minimum_normal_z", params_.ground_height.minimum_normal_z);
  params_.ground_height.maximum_curvature = readParameter<double>(
    "ground_height/maximum_curvature", params_.ground_height.maximum_curvature);
  params_.ground_height.minimum_neighbors = readParameter<int>(
    "ground_height/minimum_neighbors", params_.ground_height.minimum_neighbors);
  params_.ground_height.minimum_support_points = readParameter<int>(
    "ground_height/minimum_support_points", params_.ground_height.minimum_support_points);
  params_.ground_height_minimum_confidence = readParameter<double>(
    "ground_height/minimum_confidence", params_.ground_height_minimum_confidence);
  params_.ground_height_smoothing = readParameter<double>(
    "ground_height/smoothing", params_.ground_height_smoothing);
  params_.ground_height_max_step = readParameter<double>(
    "ground_height/max_step", params_.ground_height_max_step);

  params_.recent_voxel_frames = readParameter<int>(
    "detection/recent_voxel_frames", params_.recent_voxel_frames);
  params_.cluster_in_xy = readParameter<bool>(
    "dbscan/cluster_in_xy", params_.cluster_in_xy);
  params_.dbscan_core_points = readParameter<int>(
    "dbscan/core_points", params_.dbscan_core_points);
  params_.cluster_tolerance = readParameter<double>(
    "dbscan/tolerance", params_.cluster_tolerance);
  params_.min_cluster_size = readParameter<int>(
    "dbscan/min_cluster_size", params_.min_cluster_size);
  params_.max_cluster_size = readParameter<int>(
    "dbscan/max_cluster_size", params_.max_cluster_size);
  params_.interaction_refinement_enabled = readParameter<bool>(
    "dbscan/interaction_refinement/enabled", params_.interaction_refinement_enabled);
  params_.interaction_refinement_tolerance = readParameter<double>(
    "dbscan/interaction_refinement/tolerance", params_.interaction_refinement_tolerance);
  params_.interaction_refinement_core_points = readParameter<int>(
    "dbscan/interaction_refinement/core_points", params_.interaction_refinement_core_points);
  params_.interaction_refinement_min_cluster_size = readParameter<int>(
    "dbscan/interaction_refinement/min_cluster_size",
    params_.interaction_refinement_min_cluster_size);
  params_.interaction_refinement_min_coverage = readParameter<double>(
    "dbscan/interaction_refinement/min_coverage",
    params_.interaction_refinement_min_coverage);

  params_.dynamic_only = readParameter<bool>("detection/dynamic_only", params_.dynamic_only);
  params_.dynamic_distance_threshold = readParameter<double>(
    "detection/distance_threshold", params_.dynamic_distance_threshold);
  params_.dynamic_variance_threshold = readParameter<double>(
    "detection/variance_threshold", params_.dynamic_variance_threshold);
  params_.dynamic_max_distance = readParameter<double>(
    "detection/max_background_distance", params_.dynamic_max_distance);
  params_.track_keep_distance = readParameter<double>(
    "detection/track_keep_distance", params_.track_keep_distance);
  params_.track_keep_min_speed = readParameter<double>(
    "detection/track_keep_min_speed", params_.track_keep_min_speed);
  params_.background_history_frames = readParameter<int>(
    "detection/background_history_frames", params_.background_history_frames);
  params_.background_confirm_frames = readParameter<int>(
    "detection/background_confirm_frames", params_.background_confirm_frames);

  params_.human_filter_enabled = readParameter<bool>(
    "human_filter/enabled", params_.human_filter_enabled);
  params_.human_min_height = readParameter<double>(
    "human_filter/min_height", params_.human_min_height);
  params_.human_max_height = readParameter<double>(
    "human_filter/max_height", params_.human_max_height);
  params_.human_min_width = readParameter<double>(
    "human_filter/min_width", params_.human_min_width);
  params_.human_max_width = readParameter<double>(
    "human_filter/max_width", params_.human_max_width);
  params_.human_measurement_tolerance = readParameter<double>(
    "human_filter/measurement_tolerance", params_.human_measurement_tolerance);

  params_.manual_selection_enabled = readParameter<bool>(
    "selection/enabled", params_.manual_selection_enabled);
  params_.clicked_point_topic = readParameter<std::string>(
    "selection/clicked_point_topic", params_.clicked_point_topic);
  params_.target_pose_topic = readParameter<std::string>(
    "selection/target_pose_topic", params_.target_pose_topic);
  params_.selection_max_click_distance = readParameter<double>(
    "selection/max_click_distance", params_.selection_max_click_distance);
  params_.selection_association_max_distance = readParameter<double>(
    "selection/association_max_distance", params_.selection_association_max_distance);
  params_.selection_max_size_change = readParameter<double>(
    "selection/max_size_change", params_.selection_max_size_change);
  params_.selection_max_missed_frames = readParameter<int>(
    "selection/max_missed_frames", params_.selection_max_missed_frames);
  params_.selection_roi_fallback_enabled = readParameter<bool>(
    "selection/roi_fallback_enabled", params_.selection_roi_fallback_enabled);
  params_.selection_roi_radius_xy = readParameter<double>(
    "selection/roi_radius_xy", params_.selection_roi_radius_xy);
  params_.selection_roi_half_height = readParameter<double>(
    "selection/roi_half_height", params_.selection_roi_half_height);
  params_.selection_roi_min_points = readParameter<int>(
    "selection/roi_min_points", params_.selection_roi_min_points);
  params_.selection_background_static_min_age_frames = readParameter<int>(
    "selection/background_static_min_age_frames",
    params_.selection_background_static_min_age_frames);
  params_.selection_tracking_background_distance = readParameter<double>(
    "selection/tracking_background_distance",
    params_.selection_tracking_background_distance);
  params_.selection_reacquisition_enabled = readParameter<bool>(
    "selection/reacquisition_enabled", params_.selection_reacquisition_enabled);
  params_.selection_reacquisition_max_distance = readParameter<double>(
    "selection/reacquisition_max_distance", params_.selection_reacquisition_max_distance);
  params_.selection_reacquisition_immediate_distance = readParameter<double>(
    "selection/reacquisition_immediate_distance",
    params_.selection_reacquisition_immediate_distance);
  params_.selection_reacquisition_confirm_frames = readParameter<int>(
    "selection/reacquisition_confirm_frames",
    params_.selection_reacquisition_confirm_frames);
  params_.selection_reacquisition_confirmation_radius = readParameter<double>(
    "selection/reacquisition_confirmation_radius",
    params_.selection_reacquisition_confirmation_radius);
  params_.selection_reacquisition_max_position_correction = readParameter<double>(
    "selection/reacquisition_max_position_correction",
    params_.selection_reacquisition_max_position_correction);
  params_.selection_reacquisition_velocity_blend = readParameter<double>(
    "selection/reacquisition_velocity_blend",
    params_.selection_reacquisition_velocity_blend);
  params_.selection_reacquisition_direction_weight = readParameter<double>(
    "selection/reacquisition_direction_weight",
    params_.selection_reacquisition_direction_weight);
  params_.selection_reacquisition_max_z_difference = readParameter<double>(
    "selection/reacquisition_max_z_difference",
    params_.selection_reacquisition_max_z_difference);
  params_.selection_reacquisition_preferred_center_clearance = readParameter<double>(
    "selection/reacquisition_preferred_center_clearance",
    params_.selection_reacquisition_preferred_center_clearance);
  params_.selection_reacquisition_clearance_weight = readParameter<double>(
    "selection/reacquisition_clearance_weight",
    params_.selection_reacquisition_clearance_weight);
  params_.selection_max_speed = readParameter<double>(
    "selection/max_speed", params_.selection_max_speed);
  params_.selection_measurement_velocity_blend = readParameter<double>(
    "selection/measurement_velocity_blend", params_.selection_measurement_velocity_blend);
  params_.selection_velocity_min_displacement = readParameter<double>(
    "selection/velocity_min_displacement", params_.selection_velocity_min_displacement);
  params_.selection_velocity_max_dt = readParameter<double>(
    "selection/velocity_max_dt", params_.selection_velocity_max_dt);
  params_.selection_stop_hypothesis_enabled = readParameter<bool>(
    "selection/stop_hypothesis_enabled", params_.selection_stop_hypothesis_enabled);
  params_.selection_stop_min_speed = readParameter<double>(
    "selection/stop_min_speed", params_.selection_stop_min_speed);
  params_.selection_stop_max_last_distance = readParameter<double>(
    "selection/stop_max_last_distance", params_.selection_stop_max_last_distance);
  params_.selection_stop_hypothesis_penalty = readParameter<double>(
    "selection/stop_hypothesis_penalty", params_.selection_stop_hypothesis_penalty);
  params_.selection_stop_velocity_damping = readParameter<double>(
    "selection/stop_velocity_damping", params_.selection_stop_velocity_damping);
  params_.selection_prediction_gate_only = readParameter<bool>(
    "selection/prediction_gate_only", params_.selection_prediction_gate_only);
  params_.selection_fixed_body_size_enabled = readParameter<bool>(
    "selection/fixed_body_size_enabled", params_.selection_fixed_body_size_enabled);
  const auto fixed_body_size = readParameter<std::vector<double>>(
    "selection/fixed_body_size", {0.60, 0.60, 1.70});
  if (fixed_body_size.size() == 3U) {
    params_.selection_fixed_body_size = Eigen::Vector3d(
      fixed_body_size[0], fixed_body_size[1], fixed_body_size[2]);
  } else {
    ROS_WARN( "selection.fixed_body_size needs exactly three values");
  }
  params_.selection_adaptive_size_enabled = readParameter<bool>(
    "selection/adaptive_size_enabled", params_.selection_adaptive_size_enabled);
  params_.selection_size_smoothing = readParameter<double>(
    "selection/size_smoothing", params_.selection_size_smoothing);
  params_.selection_size_max_relative_step = readParameter<double>(
    "selection/size_max_relative_step", params_.selection_size_max_relative_step);
  params_.selection_size_compatibility_min_ratio = readParameter<double>(
    "selection/size_compatibility_min_ratio",
    params_.selection_size_compatibility_min_ratio);
  params_.selection_size_compatibility_max_ratio = readParameter<double>(
    "selection/size_compatibility_max_ratio",
    params_.selection_size_compatibility_max_ratio);
  params_.crossing_enabled = readParameter<bool>(
    "crossing/enabled", params_.crossing_enabled);
  params_.crossing_shadow_spawn_radius = readParameter<double>(
    "crossing/shadow_spawn_radius", params_.crossing_shadow_spawn_radius);
  params_.crossing_shadow_association_distance = readParameter<double>(
    "crossing/shadow_association_distance", params_.crossing_shadow_association_distance);
  params_.crossing_shadow_max_missed_frames = readParameter<int>(
    "crossing/shadow_max_missed_frames", params_.crossing_shadow_max_missed_frames);
  params_.crossing_max_shadow_tracks = readParameter<int>(
    "crossing/max_shadow_tracks", params_.crossing_max_shadow_tracks);
  params_.crossing_shadow_dynamic_only = readParameter<bool>(
    "crossing/shadow_dynamic_only", params_.crossing_shadow_dynamic_only);
  params_.crossing_shadow_min_confirmed_hits = readParameter<int>(
    "crossing/shadow_min_confirmed_hits", params_.crossing_shadow_min_confirmed_hits);
  params_.crossing_shadow_min_speed = readParameter<double>(
    "crossing/shadow_min_speed", params_.crossing_shadow_min_speed);
  params_.crossing_interaction_distance = readParameter<double>(
    "crossing/interaction_distance", params_.crossing_interaction_distance);
  params_.crossing_seed_support_radius = readParameter<double>(
    "crossing/seed_support_radius", params_.crossing_seed_support_radius);
  params_.crossing_seed_min_points = readParameter<int>(
    "crossing/seed_min_points", params_.crossing_seed_min_points);
  params_.crossing_seed_assignment_margin = readParameter<double>(
    "crossing/seed_assignment_margin", params_.crossing_seed_assignment_margin);
  params_.crossing_merged_min_extent_xy = readParameter<double>(
    "crossing/merged_min_extent_xy", params_.crossing_merged_min_extent_xy);
  params_.crossing_min_track_separation = readParameter<double>(
    "crossing/min_track_separation", params_.crossing_min_track_separation);
  params_.crossing_merge_confirm_frames = readParameter<int>(
    "crossing/merge_confirm_frames", params_.crossing_merge_confirm_frames);
  params_.crossing_occlusion_max_frames = readParameter<int>(
    "crossing/occlusion_max_frames", params_.crossing_occlusion_max_frames);
  params_.crossing_split_min_separation = readParameter<double>(
    "crossing/split_min_separation", params_.crossing_split_min_separation);
  params_.crossing_split_update_enabled = readParameter<bool>(
    "crossing/split_update_enabled", params_.crossing_split_update_enabled);
  params_.crossing_split_update_max_correction = readParameter<double>(
    "crossing/split_update_max_correction", params_.crossing_split_update_max_correction);
  params_.crossing_split_velocity_blend = readParameter<double>(
    "crossing/split_velocity_blend", params_.crossing_split_velocity_blend);
  params_.crossing_recovery_gate = readParameter<double>(
    "crossing/recovery_gate", params_.crossing_recovery_gate);
  params_.crossing_hypothesis_confirm_frames = readParameter<int>(
    "crossing/hypothesis_confirm_frames", params_.crossing_hypothesis_confirm_frames);
  params_.crossing_hypothesis_max_frames = readParameter<int>(
    "crossing/hypothesis_max_frames", params_.crossing_hypothesis_max_frames);
  params_.crossing_hypothesis_score_margin = readParameter<double>(
    "crossing/hypothesis_score_margin", params_.crossing_hypothesis_score_margin);
  params_.crossing_hypothesis_timeout_min_margin = readParameter<double>(
    "crossing/hypothesis_timeout_min_margin",
    params_.crossing_hypothesis_timeout_min_margin);
  params_.crossing_position_continuity_weight = readParameter<double>(
    "crossing/position_continuity_weight",
    params_.crossing_position_continuity_weight);
  params_.crossing_signature_size_weight = readParameter<double>(
    "crossing/signature_size_weight", params_.crossing_signature_size_weight);
  params_.crossing_signature_points_weight = readParameter<double>(
    "crossing/signature_points_weight", params_.crossing_signature_points_weight);

  params_.merge_v1_enabled = readParameter<bool>(
    "merge_v1/enable", params_.merge_v1_enabled);
  params_.merge_v1_debug_enabled = readParameter<bool>(
    "merge_v1/debug/enable", params_.merge_v1_debug_enabled);
  params_.merge_v1_max_templates = readParameter<int>(
    "merge_v1/support_memory/max_templates", params_.merge_v1_max_templates);
  params_.merge_v1_min_templates = readParameter<int>(
    "merge_v1/support_memory/min_templates", params_.merge_v1_min_templates);
  params_.merge_v1_min_template_points = readParameter<int>(
    "merge_v1/support_memory/min_points", params_.merge_v1_min_template_points);
  params_.merge_v1_min_template_quality = readParameter<double>(
    "merge_v1/support_memory/min_quality", params_.merge_v1_min_template_quality);
  params_.merge_v1_max_template_size_ratio = readParameter<double>(
    "merge_v1/support_memory/max_size_ratio", params_.merge_v1_max_template_size_ratio);
  params_.merge_v1_min_template_interval = readParameter<double>(
    "merge_v1/support_memory/min_template_interval",
    params_.merge_v1_min_template_interval);
  params_.merge_v1_resume_delay_frames = readParameter<int>(
    "merge_v1/support_memory/resume_delay_frames", params_.merge_v1_resume_delay_frames);
  params_.merge_v1_matcher.selected_templates = readParameter<int>(
    "merge_v1/support_memory/selected_templates",
    params_.merge_v1_matcher.selected_templates);

  params_.merge_v1_matcher.support_sigma = readParameter<double>(
    "merge_v1/matching/support_sigma", params_.merge_v1_matcher.support_sigma);
  params_.merge_v1_matcher.range_sigma_scale = readParameter<double>(
    "merge_v1/matching/range_sigma_scale", params_.merge_v1_matcher.range_sigma_scale);
  params_.merge_v1_matcher.max_support_distance = readParameter<double>(
    "merge_v1/matching/max_support_distance",
    params_.merge_v1_matcher.max_support_distance);
  params_.merge_v1_matcher.support_weight = readParameter<double>(
    "merge_v1/matching/support_weight", params_.merge_v1_matcher.support_weight);
  params_.merge_v1_matcher.motion_weight = readParameter<double>(
    "merge_v1/matching/motion_weight", params_.merge_v1_matcher.motion_weight);
  params_.merge_v1_matcher.motion_sigma = readParameter<double>(
    "merge_v1/matching/motion_sigma", params_.merge_v1_matcher.motion_sigma);
  params_.merge_v1_matcher.unknown_threshold = readParameter<double>(
    "merge_v1/matching/unknown_threshold", params_.merge_v1_matcher.unknown_threshold);
  params_.merge_v1_matcher.ambiguity_margin = readParameter<double>(
    "merge_v1/matching/ambiguity_margin", params_.merge_v1_matcher.ambiguity_margin);
  params_.merge_v1_matcher.view_range_scale = readParameter<double>(
    "merge_v1/matching/view_range_scale", params_.merge_v1_matcher.view_range_scale);
  params_.merge_v1_matcher.view_bearing_scale = readParameter<double>(
    "merge_v1/matching/view_bearing_scale", params_.merge_v1_matcher.view_bearing_scale);

  params_.merge_v1_matcher.spatial_enabled = readParameter<bool>(
    "merge_v1/spatial/enable", params_.merge_v1_matcher.spatial_enabled);
  params_.merge_v1_matcher.spatial_knn = readParameter<int>(
    "merge_v1/spatial/knn", params_.merge_v1_matcher.spatial_knn);
  params_.merge_v1_matcher.spatial_neighbor_sigma = readParameter<double>(
    "merge_v1/spatial/neighbor_sigma", params_.merge_v1_matcher.spatial_neighbor_sigma);
  params_.merge_v1_matcher.spatial_smooth_weight = readParameter<double>(
    "merge_v1/spatial/smooth_weight", params_.merge_v1_matcher.spatial_smooth_weight);
  params_.merge_v1_matcher.spatial_iterations = readParameter<int>(
    "merge_v1/spatial/iterations", params_.merge_v1_matcher.spatial_iterations);
  params_.merge_v1_matcher.spatial_unknown_cost = readParameter<double>(
    "merge_v1/spatial/unknown_cost", params_.merge_v1_matcher.spatial_unknown_cost);

  params_.merge_v1_matcher.visible_min_points = readParameter<int>(
    "merge_v1/visibility/min_visible_points",
    params_.merge_v1_matcher.visible_min_points);
  params_.merge_v1_matcher.partial_min_points = readParameter<int>(
    "merge_v1/visibility/min_partial_points",
    params_.merge_v1_matcher.partial_min_points);
  params_.merge_v1_matcher.visible_min_mean_support = readParameter<double>(
    "merge_v1/visibility/min_mean_support",
    params_.merge_v1_matcher.visible_min_mean_support);
  params_.merge_v1_matcher.partial_min_mean_support = readParameter<double>(
    "merge_v1/visibility/min_partial_mean_support",
    params_.merge_v1_matcher.partial_min_mean_support);
  params_.merge_v1_matcher.high_support_threshold = readParameter<double>(
    "merge_v1/visibility/high_support_threshold",
    params_.merge_v1_matcher.high_support_threshold);
  params_.merge_v1_matcher.visible_min_high_confidence_points = readParameter<int>(
    "merge_v1/visibility/min_high_confidence_points",
    params_.merge_v1_matcher.visible_min_high_confidence_points);
  params_.merge_v1_matcher.visible_expected_ratio = readParameter<double>(
    "merge_v1/visibility/visible_expected_ratio",
    params_.merge_v1_matcher.visible_expected_ratio);
  params_.merge_v1_matcher.partial_expected_ratio = readParameter<double>(
    "merge_v1/visibility/partial_expected_ratio",
    params_.merge_v1_matcher.partial_expected_ratio);
  params_.merge_v1_split_identity_support_weight = readParameter<double>(
    "merge_v1/split_identity_support_weight",
    params_.merge_v1_split_identity_support_weight);
  params_.association_max_distance = readParameter<double>(
    "tracking/association_max_distance", params_.association_max_distance);
  params_.min_confirmed_hits = readParameter<int>(
    "tracking/min_confirmed_hits", params_.min_confirmed_hits);
  params_.max_missed_frames = readParameter<int>(
    "tracking/max_missed_frames", params_.max_missed_frames);
  params_.acceleration_noise = readParameter<double>(
    "tracking/acceleration_noise", params_.acceleration_noise);
  params_.measurement_noise = readParameter<double>(
    "tracking/measurement_noise", params_.measurement_noise);
  params_.acceleration_smoothing = readParameter<double>(
    "tracking/acceleration_smoothing", params_.acceleration_smoothing);
  params_.turn_rate_smoothing = readParameter<double>(
    "tracking/turn_rate_smoothing", params_.turn_rate_smoothing);
  params_.maximum_acceleration = readParameter<double>(
    "tracking/maximum_acceleration", params_.maximum_acceleration);
  params_.maximum_turn_rate = readParameter<double>(
    "tracking/maximum_turn_rate", params_.maximum_turn_rate);
  params_.publish_unconfirmed = readParameter<bool>(
    "tracking/publish_unconfirmed", params_.publish_unconfirmed);
}

void MappingRos::validateParameters()
{
  params_.local_update_range = params_.local_update_range.cwiseMax(Eigen::Vector3d::Constant(0.1));
  params_.min_range = std::max(0.0, params_.min_range);
  params_.max_range = std::max(params_.min_range + 0.1, params_.max_range);
  params_.map_resolution = std::max(0.02, params_.map_resolution);
  auto valid_probability = [](double value) {return std::clamp(value, 0.001, 0.999);};
  params_.map_hit_probability = valid_probability(params_.map_hit_probability);
  params_.map_miss_probability = valid_probability(params_.map_miss_probability);
  params_.map_min_probability = valid_probability(params_.map_min_probability);
  params_.map_max_probability = valid_probability(params_.map_max_probability);
  params_.map_occupied_probability = valid_probability(params_.map_occupied_probability);
  if (!(params_.map_min_probability < params_.map_occupied_probability &&
    params_.map_occupied_probability < params_.map_max_probability))
  {
    ROS_WARN( "Invalid grid-map probabilities; using p_min=0.12 p_occ=0.80 p_max=0.98");
    params_.map_min_probability = 0.12;
    params_.map_occupied_probability = 0.80;
    params_.map_max_probability = 0.98;
  }
  params_.map_inflation_xy = std::max(0.0, params_.map_inflation_xy);
  params_.map_inflation_z_up = std::max(0.0, params_.map_inflation_z_up);
  params_.map_inflation_z_down = std::max(0.0, params_.map_inflation_z_down);
  params_.ground_height_minimum_confidence = std::clamp(
    params_.ground_height_minimum_confidence, 0.0, 1.0);
  params_.ground_height_smoothing = std::clamp(
    params_.ground_height_smoothing, 0.0, 1.0);
  params_.ground_height_max_step = std::max(
    params_.map_resolution, params_.ground_height_max_step);
  params_.cluster_tolerance = std::max(params_.map_resolution, params_.cluster_tolerance);
  params_.recent_voxel_frames = std::max(1, params_.recent_voxel_frames);
  params_.dbscan_core_points = std::max(1, params_.dbscan_core_points);
  params_.min_cluster_size = std::max(1, params_.min_cluster_size);
  params_.max_cluster_size = std::max(params_.min_cluster_size, params_.max_cluster_size);
  params_.interaction_refinement_tolerance = std::clamp(
    params_.interaction_refinement_tolerance, params_.map_resolution,
    params_.cluster_tolerance);
  params_.interaction_refinement_core_points = std::max(
    1, params_.interaction_refinement_core_points);
  params_.interaction_refinement_min_cluster_size = std::max(
    2, params_.interaction_refinement_min_cluster_size);
  params_.interaction_refinement_min_coverage = std::clamp(
    params_.interaction_refinement_min_coverage, 0.0, 1.0);
  params_.background_history_frames = std::max(1, params_.background_history_frames);
  params_.background_confirm_frames = std::max(1, params_.background_confirm_frames);
  params_.min_confirmed_hits = std::max(1, params_.min_confirmed_hits);
  params_.max_missed_frames = std::max(0, params_.max_missed_frames);
  params_.association_max_distance = std::max(0.05, params_.association_max_distance);
  params_.measurement_noise = std::max(0.001, params_.measurement_noise);
  params_.acceleration_noise = std::max(0.001, params_.acceleration_noise);
  params_.human_measurement_tolerance = std::max(0.0, params_.human_measurement_tolerance);
  params_.selection_max_click_distance = std::max(0.05, params_.selection_max_click_distance);
  params_.selection_association_max_distance = std::max(
    0.05, params_.selection_association_max_distance);
  params_.selection_max_size_change = std::max(0.0, params_.selection_max_size_change);
  params_.selection_max_missed_frames = std::max(0, params_.selection_max_missed_frames);
  params_.selection_roi_radius_xy = std::max(0.05, params_.selection_roi_radius_xy);
  params_.selection_roi_half_height = std::max(0.05, params_.selection_roi_half_height);
  params_.selection_roi_min_points = std::max(1, params_.selection_roi_min_points);
  params_.selection_background_static_min_age_frames = std::max(
    1, params_.selection_background_static_min_age_frames);
  params_.selection_tracking_background_distance = std::max(
    params_.dynamic_distance_threshold, params_.selection_tracking_background_distance);
  params_.selection_reacquisition_max_distance = std::max(
    params_.selection_roi_radius_xy, params_.selection_reacquisition_max_distance);
  params_.selection_reacquisition_immediate_distance = std::clamp(
    params_.selection_reacquisition_immediate_distance, params_.map_resolution,
    params_.selection_reacquisition_max_distance);
  params_.selection_reacquisition_confirm_frames = std::max(
    1, params_.selection_reacquisition_confirm_frames);
  params_.selection_reacquisition_confirmation_radius = std::max(
    params_.map_resolution, params_.selection_reacquisition_confirmation_radius);
  params_.selection_reacquisition_max_position_correction = std::clamp(
    params_.selection_reacquisition_max_position_correction, params_.map_resolution,
    params_.selection_reacquisition_max_distance);
  params_.selection_reacquisition_velocity_blend = std::clamp(
    params_.selection_reacquisition_velocity_blend, 0.0, 1.0);
  params_.selection_reacquisition_direction_weight = std::max(
    0.0, params_.selection_reacquisition_direction_weight);
  params_.selection_reacquisition_max_z_difference = std::max(
    params_.map_resolution, params_.selection_reacquisition_max_z_difference);
  params_.selection_reacquisition_preferred_center_clearance = std::max(
    0.0, params_.selection_reacquisition_preferred_center_clearance);
  params_.selection_reacquisition_clearance_weight = std::max(
    0.0, params_.selection_reacquisition_clearance_weight);
  params_.selection_max_speed = std::max(0.1, params_.selection_max_speed);
  params_.selection_measurement_velocity_blend = std::clamp(
    params_.selection_measurement_velocity_blend, 0.0, 1.0);
  params_.selection_velocity_min_displacement = std::max(
    0.0, params_.selection_velocity_min_displacement);
  params_.selection_velocity_max_dt = std::max(
    0.05, params_.selection_velocity_max_dt);
  params_.selection_stop_min_speed = std::max(0.0, params_.selection_stop_min_speed);
  params_.selection_stop_max_last_distance = std::max(
    params_.map_resolution, params_.selection_stop_max_last_distance);
  params_.selection_stop_hypothesis_penalty = std::max(
    0.0, params_.selection_stop_hypothesis_penalty);
  params_.selection_stop_velocity_damping = std::clamp(
    params_.selection_stop_velocity_damping, 0.0, 1.0);
  params_.selection_fixed_body_size = params_.selection_fixed_body_size.cwiseMax(
    Eigen::Vector3d::Constant(params_.map_resolution));
  params_.selection_size_smoothing = std::clamp(
    params_.selection_size_smoothing, 0.0, 1.0);
  params_.selection_size_max_relative_step = std::clamp(
    params_.selection_size_max_relative_step, 0.0, 2.0);
  params_.selection_size_compatibility_min_ratio = std::clamp(
    params_.selection_size_compatibility_min_ratio, 0.05, 1.0);
  params_.selection_size_compatibility_max_ratio = std::max(
    1.0, params_.selection_size_compatibility_max_ratio);
  params_.crossing_shadow_spawn_radius = std::max(
    params_.selection_roi_radius_xy, params_.crossing_shadow_spawn_radius);
  params_.crossing_shadow_association_distance = std::max(
    params_.map_resolution, params_.crossing_shadow_association_distance);
  params_.crossing_shadow_max_missed_frames = std::max(
    1, params_.crossing_shadow_max_missed_frames);
  params_.crossing_max_shadow_tracks = std::max(1, params_.crossing_max_shadow_tracks);
  params_.crossing_shadow_min_confirmed_hits = std::max(
    2, params_.crossing_shadow_min_confirmed_hits);
  params_.crossing_shadow_min_speed = std::max(0.0, params_.crossing_shadow_min_speed);
  params_.crossing_interaction_distance = std::max(
    params_.selection_roi_radius_xy, params_.crossing_interaction_distance);
  params_.crossing_seed_support_radius = std::max(
    params_.map_resolution, params_.crossing_seed_support_radius);
  params_.crossing_seed_min_points = std::max(1, params_.crossing_seed_min_points);
  params_.crossing_seed_assignment_margin = std::max(
    0.0, params_.crossing_seed_assignment_margin);
  params_.crossing_merged_min_extent_xy = std::max(
    params_.map_resolution, params_.crossing_merged_min_extent_xy);
  params_.crossing_min_track_separation = std::clamp(
    params_.crossing_min_track_separation, params_.map_resolution,
    params_.crossing_interaction_distance);
  params_.crossing_merge_confirm_frames = std::max(1, params_.crossing_merge_confirm_frames);
  params_.crossing_occlusion_max_frames = std::max(1, params_.crossing_occlusion_max_frames);
  params_.crossing_split_min_separation = std::max(
    params_.map_resolution, params_.crossing_split_min_separation);
  params_.crossing_split_update_max_correction = std::max(
    params_.map_resolution, params_.crossing_split_update_max_correction);
  params_.crossing_split_velocity_blend = std::clamp(
    params_.crossing_split_velocity_blend, 0.0, 1.0);
  params_.crossing_recovery_gate = std::max(
    params_.crossing_split_min_separation, params_.crossing_recovery_gate);
  params_.crossing_hypothesis_confirm_frames = std::max(
    2, params_.crossing_hypothesis_confirm_frames);
  params_.crossing_hypothesis_max_frames = std::max(
    params_.crossing_hypothesis_confirm_frames, params_.crossing_hypothesis_max_frames);
  params_.crossing_hypothesis_score_margin = std::max(
    0.0, params_.crossing_hypothesis_score_margin);
  params_.crossing_hypothesis_timeout_min_margin = std::clamp(
    params_.crossing_hypothesis_timeout_min_margin, 0.0,
    params_.crossing_hypothesis_score_margin);
  params_.crossing_position_continuity_weight = std::max(
    0.0, params_.crossing_position_continuity_weight);
  params_.crossing_signature_size_weight = std::max(
    0.0, params_.crossing_signature_size_weight);
  params_.crossing_signature_points_weight = std::max(
    0.0, params_.crossing_signature_points_weight);
  params_.merge_v1_max_templates = std::max(1, params_.merge_v1_max_templates);
  params_.merge_v1_min_templates = std::clamp(
    params_.merge_v1_min_templates, 1, params_.merge_v1_max_templates);
  params_.merge_v1_min_template_points = std::max(3, params_.merge_v1_min_template_points);
  params_.merge_v1_min_template_quality = std::clamp(
    params_.merge_v1_min_template_quality, 0.0, 1.0);
  params_.merge_v1_max_template_size_ratio = std::max(
    1.05, params_.merge_v1_max_template_size_ratio);
  params_.merge_v1_min_template_interval = std::max(
    0.0, params_.merge_v1_min_template_interval);
  params_.merge_v1_resume_delay_frames = std::max(0, params_.merge_v1_resume_delay_frames);
  params_.merge_v1_split_identity_support_weight = std::max(
    0.0, params_.merge_v1_split_identity_support_weight);
  auto & merge_matcher = params_.merge_v1_matcher;
  merge_matcher.selected_templates = std::clamp(
    merge_matcher.selected_templates, 1, params_.merge_v1_max_templates);
  merge_matcher.support_sigma = std::max(0.5 * params_.map_resolution, merge_matcher.support_sigma);
  merge_matcher.range_sigma_scale = std::max(0.0, merge_matcher.range_sigma_scale);
  merge_matcher.max_support_distance = std::max(
    params_.map_resolution, merge_matcher.max_support_distance);
  merge_matcher.support_weight = std::max(0.01, merge_matcher.support_weight);
  merge_matcher.motion_weight = std::max(0.0, merge_matcher.motion_weight);
  merge_matcher.motion_sigma = std::max(params_.map_resolution, merge_matcher.motion_sigma);
  merge_matcher.unknown_threshold = std::clamp(merge_matcher.unknown_threshold, 0.0, 1.0);
  merge_matcher.ambiguity_margin = std::clamp(merge_matcher.ambiguity_margin, 0.0, 1.0);
  merge_matcher.view_range_scale = std::max(0.1, merge_matcher.view_range_scale);
  merge_matcher.view_bearing_scale = std::max(0.1, merge_matcher.view_bearing_scale);
  merge_matcher.spatial_knn = std::max(1, merge_matcher.spatial_knn);
  merge_matcher.spatial_neighbor_sigma = std::max(
    params_.map_resolution, merge_matcher.spatial_neighbor_sigma);
  merge_matcher.spatial_smooth_weight = std::max(0.0, merge_matcher.spatial_smooth_weight);
  merge_matcher.spatial_iterations = std::max(0, merge_matcher.spatial_iterations);
  merge_matcher.spatial_unknown_cost = std::clamp(
    merge_matcher.spatial_unknown_cost, 0.0, 2.0);
  merge_matcher.visible_min_points = std::max(1, merge_matcher.visible_min_points);
  merge_matcher.partial_min_points = std::clamp(
    merge_matcher.partial_min_points, 1, merge_matcher.visible_min_points);
  merge_matcher.visible_min_mean_support = std::clamp(
    merge_matcher.visible_min_mean_support, 0.0, 1.0);
  merge_matcher.partial_min_mean_support = std::clamp(
    merge_matcher.partial_min_mean_support, 0.0, merge_matcher.visible_min_mean_support);
  merge_matcher.high_support_threshold = std::clamp(
    merge_matcher.high_support_threshold, 0.0, 1.0);
  merge_matcher.visible_min_high_confidence_points = std::max(
    1, merge_matcher.visible_min_high_confidence_points);
  merge_matcher.visible_expected_ratio = std::max(0.01, merge_matcher.visible_expected_ratio);
  merge_matcher.partial_expected_ratio = std::clamp(
    merge_matcher.partial_expected_ratio, 0.01, merge_matcher.visible_expected_ratio);
  params_.acceleration_smoothing = std::clamp(params_.acceleration_smoothing, 0.0, 1.0);
  params_.turn_rate_smoothing = std::clamp(params_.turn_rate_smoothing, 0.0, 1.0);
  params_.maximum_acceleration = std::max(0.1, params_.maximum_acceleration);
  params_.maximum_turn_rate = std::max(0.1, params_.maximum_turn_rate);
}

}  // namespace person_tracker
