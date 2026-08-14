/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-04-14 21:44:58
 * @LastEditTime: 2024-04-15 13:30:53
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h>
#include <general_planner/FrontierClusterArray.h>
#include <general_core/exploration/exploration_utils/frontier_manager/yaw_candidate_selector.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/MarkerArray.h>
size_t ByteArrayRaw::size = 0;

namespace {
constexpr int kVisibleFailBeforeSuspend = 3;
constexpr int kReachableFailBeforeSuspend = 3;
constexpr double kStableScoreAlpha = 0.65;

float normalizeYawDiff(float angle) {
  return viewpoint_yaw_selector::normalizeYaw(angle);
}

bool isTerminalFrontierState(const FrontierState state) {
  return state == FrontierState::VISITED || state == FrontierState::BLACKLISTED;
}

bool boxesOverlap(const ClusterInfo::Ptr &a, const ClusterInfo::Ptr &b) {
  return !(a->box_max_.x() < b->box_min_.x() ||
           a->box_max_.y() < b->box_min_.y() ||
           a->box_max_.z() < b->box_min_.z() ||
           a->box_min_.x() > b->box_max_.x() ||
           a->box_min_.y() > b->box_max_.y() ||
           a->box_min_.z() > b->box_max_.z());
}

void inheritClusterLifecycle(ClusterInfo::Ptr &cluster,
                             const vector<ClusterInfo::Ptr> &old_clusters,
                             const FrontierParam &param) {
  ClusterInfo::Ptr best_old;
  float best_dist = std::numeric_limits<float>::max();
  const float match_radius =
      std::max(param.cluster_match_radius_,
               std::max(param.cluster_min_radius_, param.cell_size_));
  for (const auto &old_cluster : old_clusters) {
    const float dist = (cluster->center_ - old_cluster->center_).norm();
    if ((dist <= match_radius || boxesOverlap(cluster, old_cluster)) &&
        dist < best_dist) {
      best_dist = dist;
      best_old = old_cluster;
    }
  }
  if (!best_old) {
    return;
  }

  cluster->observation_count_ =
      std::max(cluster->observation_count_, best_old->observation_count_ + 1);
  cluster->selected_count_ = best_old->selected_count_;
  cluster->visible_fail_count_ = best_old->visible_fail_count_;
  cluster->reachable_fail_count_ = best_old->reachable_fail_count_;
  cluster->last_selected_time_ = best_old->last_selected_time_;
  cluster->first_reachable_time_ = best_old->first_reachable_time_;
  cluster->last_goal_time_ = best_old->last_goal_time_;
  cluster->last_pass_time_ = best_old->last_pass_time_;
  cluster->goal_selected_count_ = best_old->goal_selected_count_;
  cluster->pass_count_ = best_old->pass_count_;
  cluster->pass_debt_ = best_old->pass_debt_;
  cluster->inside_pass_zone_ = best_old->inside_pass_zone_;
  cluster->pass_zone_min_distance_ = best_old->pass_zone_min_distance_;
  cluster->last_score_ = best_old->last_score_;
  cluster->stable_score_ = best_old->stable_score_;
  cluster->last_visible_gain_ = best_old->last_visible_gain_;
  cluster->stable_visible_gain_ = best_old->stable_visible_gain_;
  cluster->best_vp_ = best_old->best_vp_;
  cluster->best_vp_yaw_ = best_old->best_vp_yaw_;
  cluster->candidate_vps_ = best_old->candidate_vps_;
  cluster->candidate_yaws_ = best_old->candidate_yaws_;
  cluster->candidate_scores_ = best_old->candidate_scores_;
  if (isTerminalFrontierState(best_old->state_)) {
    cluster->state_ = best_old->state_;
    cluster->is_reachable_ = false;
    cluster->is_dormant_ = true;
  }
}

void markClusterRetry(ClusterInfo::Ptr &cluster, const bool visible_fail) {
  if (isTerminalFrontierState(cluster->state_)) {
    return;
  }
  if (visible_fail) {
    cluster->visible_fail_count_++;
  } else {
    cluster->reachable_fail_count_++;
  }
  const int fail_count = visible_fail ? cluster->visible_fail_count_
                                      : cluster->reachable_fail_count_;
  cluster->is_reachable_ = false;
  cluster->state_ = fail_count >= (visible_fail ? kVisibleFailBeforeSuspend
                                                : kReachableFailBeforeSuspend)
                        ? FrontierState::SUSPENDED
                        : FrontierState::PENDING_RETRY;
}

void markClusterActive(ClusterInfo::Ptr &cluster) {
  if (isTerminalFrontierState(cluster->state_)) {
    return;
  }
  cluster->is_reachable_ = true;
  cluster->visible_fail_count_ = 0;
  cluster->reachable_fail_count_ = 0;
  if (cluster->first_reachable_time_.isZero()) {
    cluster->first_reachable_time_ = ros::Time::now();
  }
  if (!cluster->is_dormant_) {
    cluster->state_ = FrontierState::ACTIVE;
  }
}
}  // namespace

void FrontierManager::setHighSpeedViewScoreContext(
    const HighSpeedViewScoreContext &ctx) {
  high_speed_view_ctx_ = ctx;
}

void FrontierManager::requestGlobalRecluster() {
  force_recluster_.clear();
  global_audit_pending_ = true;
  for (auto &cluster : cluster_list_) {
    if (cluster->state_ == FrontierState::BLACKLISTED) {
      continue;
    }
    force_recluster_.insert(cluster->id_);
    if (cluster->state_ == FrontierState::SUSPENDED) {
      cluster->state_ = FrontierState::PENDING_RETRY;
    }
    if (!cluster->is_dormant_) {
      cluster->is_reachable_ = true;
      cluster->needs_revalidation_ = true;
    }
  }
}

void FrontierManager::forceGlobalRefresh(
    vector<ClusterInfo::Ptr> &cluster_updated, vector<int> &cluster_removed) {
  requestGlobalRecluster();
  force_refresh_running_ = true;
  updateFrontierClusters(cluster_updated, cluster_removed);
  force_refresh_running_ = false;
}

bool FrontierManager::markClusterVisitedNear(const Eigen::Vector3f &goal,
                                             const float radius) {
  ClusterInfo::Ptr best_cluster;
  float best_distance = std::numeric_limits<float>::max();
  for (auto &cluster : cluster_list_) {
    if (cluster->state_ == FrontierState::VISITED ||
        cluster->state_ == FrontierState::BLACKLISTED) {
      continue;
    }
    const float best_vp_distance =
        cluster->selected_count_ > 0
            ? (cluster->best_vp_ - goal).norm()
            : std::numeric_limits<float>::max();
    const float center_distance = (cluster->center_ - goal).norm();
    const float distance = std::min(best_vp_distance, center_distance);
    if (distance < best_distance) {
      best_distance = distance;
      best_cluster = cluster;
    }
  }
  if (!best_cluster || best_distance > radius) {
    return false;
  }

  best_cluster->state_ = FrontierState::VISITED;
  best_cluster->is_reachable_ = false;
  best_cluster->is_dormant_ = true;
  best_cluster->last_selected_time_ = ros::Time::now();
  best_cluster->inside_pass_zone_ = false;
  best_cluster->pass_debt_ = 0.0;
  vector<ViewpointCluster>().swap(best_cluster->vp_clusters_);
  vector<Eigen::Vector3f>().swap(best_cluster->candidate_vps_);
  vector<float>().swap(best_cluster->candidate_yaws_);
  vector<double>().swap(best_cluster->candidate_scores_);
  ROS_INFO_STREAM("[frontier lifecycle] mark visited cluster="
                  << best_cluster->id_ << " distance=" << best_distance);
  refreshSemanticRevision();
  return true;
}

void FrontierManager::updateExplorationDebt(
    const Eigen::Vector3f &robot_pos, const int selected_cluster_id,
    const Eigen::Vector3f &selected_goal, const double selected_match_radius,
    const double pass_radius, const double pass_exit_margin,
    const double pass_cooldown, const double debt_increment,
    const double debt_max) {
  const ros::Time now = ros::Time::now();
  const double enter_radius = std::max(0.5, pass_radius);
  const double exit_radius = enter_radius + std::max(0.1, pass_exit_margin);
  const double cooldown = std::max(0.0, pass_cooldown);
  const double increment = std::max(0.0, debt_increment);
  const double max_debt = std::max(increment, debt_max);
  const double goal_match_radius = std::max(0.1, selected_match_radius);

  for (auto &cluster : cluster_list_) {
    if (!cluster || cluster->is_dormant_ || !cluster->is_reachable_ ||
        isTerminalFrontierState(cluster->state_)) {
      if (cluster) {
        cluster->inside_pass_zone_ = false;
        cluster->pass_zone_min_distance_ =
            std::numeric_limits<double>::infinity();
      }
      continue;
    }
    if (cluster->first_reachable_time_.isZero()) {
      cluster->first_reachable_time_ = now;
    }

    const Eigen::Vector3f reference =
        cluster->candidate_vps_.empty() ? cluster->center_
                                        : cluster->candidate_vps_.front();
    const double distance = (reference - robot_pos).norm();
    const bool selected =
        cluster->id_ == selected_cluster_id ||
        (selected_cluster_id >= 0 &&
         (reference - selected_goal).norm() <= goal_match_radius);
    if (selected) {
      cluster->inside_pass_zone_ = false;
      cluster->pass_zone_min_distance_ =
          std::numeric_limits<double>::infinity();
      continue;
    }

    if (distance <= enter_radius) {
      cluster->inside_pass_zone_ = true;
      cluster->pass_zone_min_distance_ =
          std::min(cluster->pass_zone_min_distance_, distance);
      continue;
    }

    if (cluster->inside_pass_zone_ && distance >= exit_radius) {
      const bool cooldown_elapsed =
          cluster->last_pass_time_.isZero() ||
          (now - cluster->last_pass_time_).toSec() >= cooldown;
      if (cooldown_elapsed &&
          cluster->pass_zone_min_distance_ <= enter_radius) {
        cluster->pass_count_++;
        cluster->pass_debt_ =
            std::min(max_debt, cluster->pass_debt_ + increment);
        cluster->last_pass_time_ = now;
        ROS_INFO_STREAM("[frontier debt] passed cluster=" << cluster->id_
                        << " count=" << cluster->pass_count_
                        << " debt=" << cluster->pass_debt_
                        << " closest=" << cluster->pass_zone_min_distance_);
      }
      cluster->inside_pass_zone_ = false;
      cluster->pass_zone_min_distance_ =
          std::numeric_limits<double>::infinity();
    }
  }
}

void FrontierManager::markClusterGoalSelected(
    const int cluster_id, const Eigen::Vector3f &goal,
    const double match_radius) {
  ClusterInfo::Ptr matched;
  double best_distance = std::numeric_limits<double>::infinity();
  for (auto &cluster : cluster_list_) {
    if (!cluster || isTerminalFrontierState(cluster->state_)) {
      continue;
    }
    const Eigen::Vector3f reference =
        cluster->candidate_vps_.empty() ? cluster->best_vp_
                                        : cluster->candidate_vps_.front();
    const double distance = (reference - goal).norm();
    if (cluster->id_ == cluster_id) {
      matched = cluster;
      best_distance = distance;
      break;
    }
    if (distance < best_distance) {
      best_distance = distance;
      matched = cluster;
    }
  }
  if (!matched || (matched->id_ != cluster_id &&
                   best_distance > std::max(0.1, match_radius))) {
    return;
  }
  const ros::Time now = ros::Time::now();
  matched->goal_selected_count_++;
  matched->last_goal_time_ = now;
  matched->first_reachable_time_ = now;
  matched->pass_debt_ = 0.0;
  matched->inside_pass_zone_ = false;
  matched->pass_zone_min_distance_ =
      std::numeric_limits<double>::infinity();
  ROS_INFO_STREAM("[frontier lifecycle] selected cluster=" << matched->id_
                  << " selected_count=" << matched->goal_selected_count_
                  << " match_distance=" << best_distance);
}

int FrontierManager::activeClusterCount() const {
  int count = 0;
  for (const auto &cluster : cluster_list_) {
    if (cluster->is_dormant_ ||
        cluster->state_ == FrontierState::VISITED ||
        cluster->state_ == FrontierState::BLACKLISTED) {
      continue;
    }
    count++;
  }
  return count;
}

int FrontierManager::reachableClusterCount() const {
  int count = 0;
  for (const auto &cluster : cluster_list_) {
    if (!cluster->is_dormant_ && cluster->is_reachable_ &&
        cluster->state_ != FrontierState::VISITED &&
        cluster->state_ != FrontierState::BLACKLISTED) {
      count++;
    }
  }
  return count;
}

bool FrontierManager::frontierAuditReady() const {
  return !global_audit_pending_ &&
         audited_frontier_revision_ == frontier_revision_;
}

uint64_t FrontierManager::computeSemanticSignature() const {
  struct Entry {
    int x;
    int y;
    int z;
    int state;
    int reachable;
    int dormant;
    int size_bucket;
    bool operator<(const Entry &other) const {
      return std::tie(x, y, z, state, reachable, dormant, size_bucket) <
             std::tie(other.x, other.y, other.z, other.state,
                      other.reachable, other.dormant, other.size_bucket);
    }
  };
  const double quantization = std::max(
      0.5, 2.0 * static_cast<double>(std::max(0.1f, frtp_.cell_size_)));
  std::vector<Entry> entries;
  entries.reserve(cluster_list_.size());
  for (const auto &cluster : cluster_list_) {
    if (!cluster || isTerminalFrontierState(cluster->state_)) {
      continue;
    }
    entries.push_back(
        {static_cast<int>(std::llround(cluster->center_.x() / quantization)),
         static_cast<int>(std::llround(cluster->center_.y() / quantization)),
         static_cast<int>(std::llround(cluster->center_.z() / quantization)),
         static_cast<int>(cluster->state_), cluster->is_reachable_ ? 1 : 0,
         cluster->is_dormant_ ? 1 : 0,
         static_cast<int>(cluster->cells_.size() / 8U)});
  }
  std::sort(entries.begin(), entries.end());
  uint64_t value = 1469598103934665603ULL;
  auto mix = [&](const uint64_t item) {
    value ^= item + 0x9e3779b97f4a7c15ULL + (value << 6) + (value >> 2);
    value *= 1099511628211ULL;
  };
  mix(entries.size());
  for (const Entry &entry : entries) {
    mix(static_cast<uint32_t>(entry.x));
    mix(static_cast<uint32_t>(entry.y));
    mix(static_cast<uint32_t>(entry.z));
    mix(static_cast<uint32_t>(entry.state));
    mix(static_cast<uint32_t>(entry.reachable));
    mix(static_cast<uint32_t>(entry.dormant));
    mix(static_cast<uint32_t>(entry.size_bucket));
  }
  return value;
}

void FrontierManager::refreshSemanticRevision() {
  const uint64_t signature = computeSemanticSignature();
  if (!frontier_signature_initialized_) {
    frontier_semantic_signature_ = signature;
    frontier_signature_initialized_ = true;
    return;
  }
  if (signature == frontier_semantic_signature_) {
    return;
  }
  frontier_semantic_signature_ = signature;
  ++frontier_revision_;
  audited_frontier_revision_ = std::numeric_limits<uint64_t>::max();
}

void FrontierManager::releaseDerivedViewpointCache() {
  // vp_clusters_ is a per-planning-cycle intermediate. The persistent
  // candidate_vps_/candidate_yaws_ output remains available for goal locking,
  // coverage association and lifecycle inheritance.
  for (auto &cluster : cluster_list_) {
    if (cluster && !cluster->vp_clusters_.empty()) {
      vector<ViewpointCluster>().swap(cluster->vp_clusters_);
    }
  }
}

void FrontierManager::finishGlobalAuditIfComplete() {
  if (!global_audit_pending_) {
    return;
  }
  for (const auto &cluster : cluster_list_) {
    if (!cluster || cluster->is_dormant_ ||
        isTerminalFrontierState(cluster->state_)) {
      continue;
    }
    if (cluster->needs_revalidation_) {
      return;
    }
  }
  audited_frontier_revision_ = frontier_revision_;
  global_audit_pending_ = false;
  ROS_INFO_STREAM("[frontier audit] complete revision=" << frontier_revision_
                  << " active=" << activeClusterCount()
                  << " reachable=" << reachableClusterCount());
}

void FrontierManager::init(ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
                           TopoGraph::Ptr graph) {
  nh_ = nh;
  graph_ = graph;
  lidar_map_interface_ = lio_interface;
  nh.getParam("FrontierManager/cell_size", frtp_.cell_size_);
  frtp_.inv_cell_size_ = 1 / frtp_.cell_size_;
  nh.getParam("FrontierManager/noise_cell_range", frtp_.noise_cell_range_);
  nh.getParam("FrontierManager/good_observation_direction_score",
              frtp_.good_observation_direction_score_);
  nh.getParam("FrontierManager/good_observation_trust_length",
              frtp_.good_observation_trust_length_);
  nh.getParam("FrontierManager/good_observation_force_trust_length",
              frtp_.good_observation_force_trust_length_);
  nh.getParam("FrontierManager/update_length", frtp_.update_length_);
  nh.getParam("FrontierManager/view_frt", frtp_.view_frt_);
  nh.getParam("FrontierManager/view_cluster", frtp_.view_cluster_);
  nh.param("FrontierManager/publish_frontier_topic",
           frtp_.publish_frontier_topic_, true);
  if (frtp_.publish_frontier_topic_) {
    frontier_clusters_pub_ =
        nh_.advertise<general_planner::FrontierClusterArray>(
            "frontier_clusters", 5);
  }

  nh.getParam("FrontierManager/cluster_min_radius", frtp_.cluster_min_radius_);
  nh.getParam("FrontierManager/cluster_min_size", frtp_.cluster_min_size_);
  nh.getParam("FrontierManager/cluster_max_size", frtp_.cluster_radius_);
  nh.getParam("FrontierManager/cluster_direction_radius",
              frtp_.cluster_direction_radius_);
  nh.getParam("FrontierManager/cluster_minmum_point_num",
              frtp_.cluster_minmum_point_num_);
  nh.param("FrontierManager/cluster_min_observation_frames",
           frtp_.cluster_min_observation_frames_, 2);
  nh.param("FrontierManager/cluster_max_fov_edge_ratio",
           frtp_.cluster_max_fov_edge_ratio_, 0.65f);
  nh.param("FrontierManager/cluster_max_gap_ratio",
           frtp_.cluster_max_gap_ratio_, 0.85f);
  nh.param("FrontierManager/cluster_match_radius",
           frtp_.cluster_match_radius_, frtp_.cluster_min_radius_ * 1.5f);
  nh.param("FrontierManager/boundary_ignore_cells",
           frtp_.boundary_ignore_cells_, 1);
  frtp_.boundary_ignore_cells_ = std::max(0, frtp_.boundary_ignore_cells_);

  // frt_cluster_ptr_.reset(new FrontierCluster);
  // frt_cluster_ptr_->init(nh);

  nh.param("ViewpointManager/sample_pillar_height_layer_num",
           vpp_.sample_pillar_height_layer_num_, 5);
  nh.param("ViewpointManager/sample_pillar_radius_layer_num",
           vpp_.sample_pillar_radius_layer_num_, 8);
  nh.param("ViewpointManager/sample_pillar_circle_sample_num",
           vpp_.sample_pillar_circle_sample_num_, 4);
  nh.param("ViewpointManager/sample_pillar_max_height",
           vpp_.sample_pillar_max_height_, 2.5f);
  nh.param("ViewpointManager/sample_pillar_min_height",
           vpp_.sample_pillar_min_height_, -2.0f);
  nh.param("ViewpointManager/sample_pillar_min_radius",
           vpp_.sample_pillar_min_radius_, 1.0f);
  nh.param("ViewpointManager/sample_pillar_max_radius",
           vpp_.sample_pillar_max_radius_, 4.0f);

  nh.param("ViewpointManager/consider_range", vpp_.consider_range_, 12);
  nh.param("ViewpointManager/global_recluster_size",
           vpp_.global_recluster_size_, 24);
  nh.param("ViewpointManager/local_tsp_size", vpp_.local_tsp_size_, 10);
  nh.param("ViewpointManager/view_direction_range",
           vpp_.view_direction_range_, 120.0f);
  nh.param("ViewpointManager/top_candidate_num", vpp_.top_candidate_num_, 1);
  vpp_.top_candidate_num_ = std::max(1, vpp_.top_candidate_num_);

  nh.getParam("lidar_perception/fov_viewpoint_up", vpp_.fov_up_);
  nh.getParam("lidar_perception/lidar_pitch", vpp_.lidar_pitch_);
  nh.getParam("lidar_perception/fov_viewpoint_down", vpp_.fov_down_);

  vpp_.view_direction_range_ = cos(vpp_.view_direction_range_ * M_PI / 180.0);
  vpp_.fov_up_ = vpp_.fov_up_ * M_PI / 180.0;
  vpp_.fov_down_ = vpp_.fov_down_ * M_PI / 180.0;
  frtp_.map_min_ =
      lidar_map_interface_->lp_->global_map_min_boundary_.cast<float>();
  frtp_.map_max_ =
      lidar_map_interface_->lp_->global_map_max_boundary_.cast<float>();
  frtp_.cell_max_cnt_ =
      ((frtp_.map_max_ - frtp_.map_min_).array() / frtp_.cell_size_)
          .cast<int>()
          .matrix() +
      Eigen::Vector3i::Ones();
  frtp_.bits_need_.x() = std::ceil(std::log2(frtp_.cell_max_cnt_.x()));
  frtp_.bits_need_.y() = std::ceil(std::log2(frtp_.cell_max_cnt_.y()));
  frtp_.bits_need_.z() = std::ceil(std::log2(frtp_.cell_max_cnt_.z()));
  frtp_.idx_byte_size_ =
      (frtp_.bits_need_.x() + frtp_.bits_need_.y() + frtp_.bits_need_.z() + 7) /
      8;
  const int total_index_bits = frtp_.bits_need_.sum();
  if (total_index_bits > 64 || frtp_.idx_byte_size_ > sizeof(uint64_t)) {
    ROS_FATAL_STREAM("Frontier cell index needs " << total_index_bits
                     << " bits, but the compact key supports at most 64");
    throw std::runtime_error("frontier map dimensions exceed 64-bit key");
  }

  float start_degree = 0;
  float degree_step = 2 * M_PI / vpp_.sample_pillar_circle_sample_num_;
  float start_degree_step =
      degree_step / (float)vpp_.sample_pillar_height_layer_num_;
  float height_step =
      (vpp_.sample_pillar_max_height_ - vpp_.sample_pillar_min_height_) /
      vpp_.sample_pillar_height_layer_num_;
  float radius_step =
      (vpp_.sample_pillar_max_radius_ - vpp_.sample_pillar_min_radius_) /
      vpp_.sample_pillar_radius_layer_num_;
  for (float height = vpp_.sample_pillar_min_height_;
       height <= vpp_.sample_pillar_max_height_ - 1e-3; height += height_step) {
    for (float radius = vpp_.sample_pillar_min_radius_;
         radius <= vpp_.sample_pillar_max_radius_ - 1e-3;
         radius += radius_step) {
      start_degree += start_degree_step;
      for (float degree = start_degree;
           degree <= start_degree + 2 * M_PI - 1e-6; degree += degree_step) {
        Eigen::Vector3f vp(radius * cos(degree), radius * sin(degree), height);
        origin_viewpoints_.push_back(vp);
      }
    }
  }
  frtd_ = FrontierData(frtp_.idx_byte_size_);
  frtd_.label_map_.max_load_factor(1.5);
  frtd_.frt_map_.max_load_factor(1.5);
  ROS_INFO_STREAM("[frontier storage] compact inline key enabled, bits="
                  << frtp_.bits_need_.transpose()
                  << " bytes=" << static_cast<int>(frtp_.idx_byte_size_));
}

void FrontierManager::pos2idx(const PointType &pt, Eigen::Vector3i &idx) {
  //
  idx = ((pt.getVector3fMap() - frtp_.map_min_) * frtp_.inv_cell_size_)
            .array()
            .floor()
            .cast<int>();
}

void FrontierManager::pos2idx(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  //
  idx = ((pt - frtp_.map_min_) * frtp_.inv_cell_size_)
            .array()
            .floor()
            .cast<int>();
}

void FrontierManager::idx2bytes(const Eigen::Vector3i &idx,
                                ByteArrayRaw &bytes) {
  // 无需resize，因为ByteArrayRaw在构造时已分配好固定大小
  uint64_t value = (static_cast<uint64_t>(idx.x())
                    << (frtp_.bits_need_.y() + frtp_.bits_need_.z())) |
                   (static_cast<uint64_t>(idx.y()) << frtp_.bits_need_.z()) |
                   static_cast<uint64_t>(idx.z());

  for (int i = 0; i < frtp_.idx_byte_size_; ++i) {
    bytes.data[i] = static_cast<uint8_t>(value >> (i * 8));
  }
}

void FrontierManager::bytes2pos(const ByteArrayRaw &bytes, PointType &pt) {
  Eigen::Vector3i idx;

  uint64_t value = 0;
  for (int i = frtp_.idx_byte_size_ - 1; i >= 0; --i) {
    value = (value << 8) | static_cast<uint64_t>(bytes.data[i]);
  }

  idx.z() = static_cast<int>(value & ((1 << frtp_.bits_need_.z()) - 1));
  value >>= frtp_.bits_need_.z();

  idx.y() = static_cast<int>(value & ((1 << frtp_.bits_need_.y()) - 1));
  value >>= frtp_.bits_need_.y();

  idx.x() = static_cast<int>(value);

  Eigen::Vector3f pt_v3f =
      (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ +
      frtp_.map_min_;
  pt.x = pt_v3f.x();
  pt.y = pt_v3f.y();
  pt.z = pt_v3f.z();
}

void FrontierManager::pos2bytes(const PointType &pt, ByteArrayRaw &bytes) {
  Eigen::Vector3i idx =
      ((pt.getVector3fMap() - frtp_.map_min_) * frtp_.inv_cell_size_)
          .array()
          .floor()
          .cast<int>();
  idx2bytes(idx, bytes);
}

CELL_STATE FrontierManager::get_state(const PointType &pt) {
  Eigen::Vector3i idx;
  pos2idx(pt, idx);
  return get_state(idx);
}

CELL_STATE FrontierManager::get_state(const Eigen::Vector3i &idx) {
  ByteArrayRaw bytes;
  idx2bytes(idx, bytes);
  if (frtd_.label_map_.find(bytes) == frtd_.label_map_.end())
    return UNKNOWN;
  else
    return (CELL_STATE)frtd_.label_map_[bytes];
}

bool FrontierManager::is_boundary_cell(const Eigen::Vector3i &idx) const {
  const int margin = frtp_.boundary_ignore_cells_;
  if (margin <= 0) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (idx(axis) < margin ||
        idx(axis) >= frtp_.cell_max_cnt_(axis) - margin) {
      return true;
    }
  }
  return false;
}

void FrontierManager::get_cells_2_update(
    const PointVector &points, vector<Eigen::Vector3i> &cells_2_update) {
  cells_2_update.clear();
  std::unordered_set<Eigen::Vector3i, Vector3i_Hash> cells_2_update_set;
  std::unordered_set<Eigen::Vector3i, Vector3i_Hash> updated;
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  for (auto &pt : points) {
    if (!lidar_map_interface_->IsInBox(pt))
      continue;
    if ((pt.getVector3fMap() -
         lidar_map_interface_->ld_->lidar_pose_.cast<float>())
            .norm() > frtp_.update_length_)
      continue;
    Eigen::Vector3i idx;
    pos2idx(pt, idx);
    if (updated.count(idx))
      continue;
    cells_2_update_set.insert(idx);
    updated.insert(idx);
    if (is_gap_point(pt) || is_fov_edge(pt)) {
      continue;
    }
    // 下面这块是为了去除噪声，也可以改成raycast
    for (int i = -frtp_.noise_cell_range_; i <= frtp_.noise_cell_range_; i++)
      for (int j = -frtp_.noise_cell_range_; j <= frtp_.noise_cell_range_; j++)
        for (int k = -frtp_.noise_cell_range_; k <= frtp_.noise_cell_range_;
             k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          Eigen::Vector3i cell = idx + Eigen::Vector3i(i, j, k);
          if (cells_2_update_set.count(cell))
            continue;
          ByteArrayRaw bytes;
          idx2bytes(cell, bytes);
          if (!frtd_.label_map_.count(bytes))
            continue;
          if (frtd_.label_map_[bytes] == DENSE)
            continue;
          cells_2_update_set.insert(cell);
        }
  }
  cells_2_update.insert(cells_2_update.end(), cells_2_update_set.begin(),
                        cells_2_update_set.end());
}

void FrontierManager::get_pts_in_cells(
    const vector<Eigen::Vector3i> &cells_2_update,
    vector<PointVector> &pts_inside) {
  ros::Time t1 = ros::Time::now();
  pts_inside.clear();
  pts_inside.resize(cells_2_update.size());
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto i = 0; i < cells_2_update.size(); i++) {
    if (get_state(cells_2_update[i]) == DENSE)
      continue;
    PointType pt;
    idx2pos(cells_2_update[i], pt);
    vector<float> t;
    lidar_map_interface_->KNN(pt, 15, pts_inside[i], t);
  }
  // ROS_INFO("get_pts_in_cells time cost: %f", (ros::Time::now() - t1).toSec()
  // * 1000.0);
}

bool FrontierManager::is_gap_point(const PointType &pt) {

  return frtd_.is_gap_[surface_pos2idx(pt)];
  // return false;
}

void FrontierManager::update_lidar_pt_gap(const vector<float> &depth) {

  frtd_.is_gap_ = vector<bool>(20000, false);
  auto viz_img = [&](cv::Mat img, string name) {
    cv::Mat image_8u, upsampled;
    cv::normalize(img, image_8u, 0, 255, cv::NORM_MINMAX);
    image_8u.convertTo(image_8u, CV_8UC1);
    cv::resize(image_8u, upsampled, cv::Size(400, 800));
    cv::imshow(name, upsampled);
    cv::waitKey(1);
  };
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  frtd_.direction_score_ = vector<float>(20000, 0.0f);
  static vector<Eigen::Vector2i> diff_lis{{-1, 0}, {0, -1}, {0, 1}, {1, 0}};
  static vector<Eigen::Vector2i> diff_lis2{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 200; j++) {
      float dis1 = depth[i * 200 + j];
      for (auto &diff : diff_lis2) {
        if (!(i + diff[0] >= 0 && i + diff[0] < 100 && j + diff[1] >= 0 &&
              j + diff[1] < 200))
          continue;
        float dis2 = depth[(i + diff[0]) * 200 + (j + diff[1])];
        if (dis1 < 0 || dis2 < 0) {
          frtd_.is_gap_[i * 200 + j] = true;
          break;
        }
        float score = dis1 / dis2;
        if (score > 1.0)
          score = 1 / score;
        // float score = dis2 / dis1;
        frtd_.direction_score_[i * 200 + j] =
            frtd_.direction_score_[i * 200 + j] <= 1e-6
                ? score
                : min(frtd_.direction_score_[i * 200 + j], score);
        if (frtd_.direction_score_[i * 200 + j] <
            frtp_.good_observation_direction_score_) {
          frtd_.is_gap_[i * 200 + j] = true;
          break;
          // if (fabs(dis1 - dis2) > 0.5) {
          //   is_gap_[i * 200 + j] = true;
          //   break;
        }
      }
    }
  }
  vector<bool> is_gap_tmp = frtd_.is_gap_;
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 200; j++) {
      if (is_gap_tmp[i * 200 + j]) {
        for (auto &diff : diff_lis2) {
          if (!(i + diff[0] >= 0 && i + diff[0] < 100 && j + diff[1] >= 0 &&
                j + diff[1] < 200)) {
            continue;
          }
          frtd_.is_gap_[(i + diff[0]) * 200 + (j + diff[1])] = true;
        }
      }
    }
  }
  ros::Time t4 = ros::Time::now();

  cv::Mat img_gap = cv::Mat::zeros(100, 200, CV_8UC1);
  cv::Mat img_depth = cv::Mat::zeros(100, 200, CV_32FC1);
  for (int i = 0; i < 100; i++)
    for (int j = 0; j < 200; j++) {
      img_gap.at<uchar>(i, j) = frtd_.is_gap_[i * 200 + j] ? 255 : 0;
      img_depth.at<float>(i, j) = depth[i * 200 + j];
    }
}

void FrontierManager::cluster_frts(const PointVector &frt_new,
                                   vector<ClusterInfo::Ptr> &new_clusters,
                                   vector<int> &cluster_removed) {
  cluster_removed.clear();
  PointVector frts2cluster;
  // static int frt_cluser_id = 0;
  // cout << "SF_list size: " << frt_cluster_ptr_->SF_list.size() << endl;
  vector<Eigen::Vector3f> frts_norm;
  vector<ClusterInfo::Ptr> old_clusters;
  if (cluster_list_.size() != 0) {
    cluster_list_.remove_if([this, &frts2cluster, &frts_norm,
                             &cluster_removed,
                             &old_clusters](ClusterInfo::Ptr &cluster) {
      // if (!cluster->is_reachable_)
      //   return false;
      const bool should_update =
          force_recluster_.count(cluster->id_) ||
          (force_recluster_.empty() &&
           has_overlap(cluster->box_max_, cluster->box_min_));
      if (!should_update) {
        return false;
      }
      if (cluster->state_ == FrontierState::BLACKLISTED) {
        old_clusters.push_back(cluster);
        cluster_removed.push_back(cluster->id_);
        return true;
      }
      {
        old_clusters.push_back(cluster);
        for (auto &pt : cluster->cells_) {
          if (get_state(pt) == FRONTIER_DIS || get_state(pt) == FRONTIER_DIR) {
            ByteArrayRaw bytes;
            pos2bytes(pt, bytes);
            Eigen::Vector3f norm = frtd_.frt_map_[bytes];
            if (!norm.allFinite()) {
              ROS_WARN_THROTTLE(
                  1.0, "[frontier cluster] discard old cell with invalid normal");
              frtd_.label_map_[bytes] = DENSE;
              frtd_.frt_map_.erase(bytes);
              continue;
            }
            frts2cluster.push_back(pt);
            frts_norm.push_back(norm);
          }
        }
        cluster_removed.push_back(cluster->id_);
        return true;
      }
      return false;
    });
  }

  for (auto &pt : frt_new) {
    ByteArrayRaw bytes;
    pos2bytes(pt, bytes);
    Eigen::Vector3f norm = frtd_.frt_map_[bytes];
    if (!norm.allFinite()) {
      ROS_WARN_THROTTLE(
          1.0, "[frontier cluster] discard new cell with invalid normal");
      frtd_.label_map_[bytes] = DENSE;
      frtd_.frt_map_.erase(bytes);
      continue;
    }
    frts2cluster.push_back(pt);
    frts_norm.push_back(norm);
  }

  // 重新聚类:
  if (frts2cluster.size() < frtp_.cluster_minmum_point_num_) {
    force_recluster_.clear();
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr frt_pc(
      new pcl::PointCloud<pcl::PointXYZ>);
  frt_pc->points = frts2cluster;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(frt_pc);
  auto getNbrs = [&](int norm_idx, int idx, vector<int> &nbr_idxs) -> int {
    nbr_idxs.clear();
    std::vector<int> indices;
    std::vector<float> squared_distances;
    if (kdtree.radiusSearch(idx, frtp_.cluster_min_radius_, indices,
                            squared_distances) < 3)
      return 0;
    Eigen::Vector3f norm = frts_norm[norm_idx];
    for (int nbr_idx : indices) {
      Eigen::Vector3f nbr_norm = frts_norm[nbr_idx];
      if (norm.dot(nbr_norm) > (frtp_.cluster_direction_radius_)) {
        nbr_idxs.push_back(nbr_idx);
      }
    }
    return nbr_idxs.size();
  };
  std::vector<int> labels;
  labels.resize(frt_pc->points.size(), -1); // 初始化标签，-1 表示未访问
  int cluster_id = 0;                       // 聚类ID
  for (size_t i = 0; i < frts2cluster.size(); i++) {
    if (labels[i] != -1)
      continue;
    std::vector<int> indices;
    if (getNbrs(i, i, indices) < 1)
      continue;
    cluster_id++;           // 分配新的聚类ID
    labels[i] = cluster_id; // 标记当前点
    std::list<size_t> queue;
    queue.push_back(i);
    Eigen::Vector3f aabb_max =
        Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                        std::numeric_limits<float>::lowest(),
                        std::numeric_limits<float>::lowest());
    Eigen::Vector3f aabb_min = Eigen::Vector3f(
        std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    while (!queue.empty()) {
      size_t current = queue.front();
      queue.pop_front();
      // 执行基于半径的搜索找到当前点的邻居
      if (getNbrs(i, current, indices) >= 1) {
        Eigen::Vector3f norm = frts_norm[current];
        for (size_t j = 0; j < indices.size(); ++j) {
          int neighbor_index = indices[j];
          if (labels[neighbor_index] == -1) {
            // 如果邻居未被访问，则将其添加到聚类中
            labels[neighbor_index] = cluster_id;
            queue.push_back(neighbor_index);
          } else if (labels[neighbor_index] == 0) {
            // 如果邻居是噪声点，则将其重新标记为当前聚类的一部分
            labels[neighbor_index] = cluster_id;
          }
          aabb_min =
              aabb_min.cwiseMin(frts2cluster[neighbor_index].getVector3fMap());
          aabb_max =
              aabb_max.cwiseMax(frts2cluster[neighbor_index].getVector3fMap());
          if ((aabb_max - aabb_min).maxCoeff() > frtp_.cluster_radius_)
            break;
        }
      } else {
        // 如果邻域内的点数不足以形成一个聚类，则将其标记为噪声
        labels[i] = 0;
      }
    }
  }
  for (int i = 1; i <= cluster_id; i++) {
    PointVector frt_cluster_pt;
    vector<Eigen::Vector3f> frt_cluster_norm;
    for (int j = 0; j < labels.size(); j++) {
      if (labels[j] == i) {
        frt_cluster_pt.push_back(frts2cluster[j]);
        frt_cluster_norm.push_back(frts_norm[j]);
      }
    }
    // if (frt_cluster_pt.size() < frtp_.cluster_minmum_point_num_)
    //   continue;
    ClusterInfo::Ptr cluster = make_shared<ClusterInfo>();
    compute_cluster_info(frt_cluster_pt, frt_cluster_norm, cluster);
    inheritClusterLifecycle(cluster, old_clusters, frtp_);
    cluster_list_.push_back(cluster);
    new_clusters.push_back(cluster);
  }
  // 将噪音删掉
  for (int i = 0; i < labels.size(); i++) {
    if (labels[i] == 0) {
      ByteArrayRaw bytes;
      pos2bytes(frts2cluster[i], bytes);
      frtd_.label_map_[bytes] = DENSE;
      frtd_.frt_map_.erase(bytes);
    }
  }
  force_recluster_.clear();
}

void FrontierManager::idx2pos(const Eigen::Vector3i &idx, PointType &pt) {
  Eigen::Vector3f pt_v3f =
      (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ +
      frtp_.map_min_;
  pt.x = pt_v3f.x();
  pt.y = pt_v3f.y();
  pt.z = pt_v3f.z();
}

void FrontierManager::computeNormal(const PointVector &local_pts,
                                    Eigen::Vector3f &normal) {
  if (local_pts.size() < 3) {
    ROS_WARN_THROTTLE(1.0, "computeNormal input size < 3");
    normal = Eigen::Vector3f::UnitZ();
    return;
  }
  Eigen::Vector3f center(0.0, 0.0, 0.0);
  for (int i = 0; i < local_pts.size(); i++) {
    center += local_pts[i].getVector3fMap();
  }
  center /= local_pts.size();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (auto &pt : local_pts) {
    Eigen::Vector3f div = pt.getVector3fMap() - center;
    covariance += div * div.transpose();
  }
  covariance /= local_pts.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
  normal = saes.eigenvectors().col(0);
  normal.normalize();
}

void FrontierManager::computeNormalCell(const PointVector &local_pts,
                                        Eigen::Vector3f &normal,
                                        Eigen::Vector3f &center) {
  if (local_pts.size() < 3) {
    ROS_WARN_THROTTLE(1.0, "computeNormalCell input size < 3");
    normal = Eigen::Vector3f::UnitZ();
    center = Eigen::Vector3f::Zero();
    return;
  }
  center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < local_pts.size(); i++) {
    center += local_pts[i].getVector3fMap();
  }
  center /= local_pts.size();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (auto &pt : local_pts) {
    Eigen::Vector3f div = pt.getVector3fMap() - center;
    covariance += div * div.transpose();
  }

  covariance /= local_pts.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
  normal = saes.eigenvectors().col(0);
  normal.normalize();
  Eigen::Vector3f dir =
      center - lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  if (dir.dot(normal) < 0) {
    return;
  } else {
    normal = -normal;
  }

  Eigen::Vector3f move_pt_ =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>() - center;
  double dotProduct = move_pt_.dot(normal);
  double normVec1 = move_pt_.norm();
  double normVec2 = normal.norm();
  double cosAngle = dotProduct / (normVec1 * normVec2);
  double angle = std::acos(cosAngle);
  angle = angle * 180.0 / M_PI;
  if (angle > 180)
    angle = 360 - angle;
  if (angle > 90)
    normal = -normal;
}

void FrontierManager::update_lidar_fov_edge(const vector<float> &depth) {
  auto viz_img = [&](cv::Mat img, string name) {
    cv::Mat image_8u, upsampled;
    cv::normalize(img, image_8u, 0, 255, cv::NORM_MINMAX);
    image_8u.convertTo(image_8u, CV_8UC1);
    cv::resize(image_8u, upsampled, cv::Size(400, 800));
    cv::imshow(name, upsampled);
    cv::waitKey(1);
  };
  frtd_.is_fov_edge_ = vector<bool>(20000, false);

  for (int j = 0; j < 200; j++) {
    for (int i = 0; i < 100; i++) {
      if (depth[i * 200 + j] <= 0.1)
        frtd_.is_fov_edge_[i * 200 + j] = true;
      else {
        frtd_.is_fov_edge_[i * 200 + j] = true;
        break;
      }
    }
    for (int i = 99; i >= 0; i--) {
      if (depth[i * 200 + j] <= 0.1)
        frtd_.is_fov_edge_[i * 200 + j] = true;
      else {
        frtd_.is_fov_edge_[i * 200 + j] = true;
        break;
      }
    }
  }
  cv::Mat img_origin(100, 200, CV_8UC1);
  for (int j = 0; j < 200; j++) {
    for (int i = 0; i < 100; i++) {
      img_origin.at<uchar>(i, j) = frtd_.is_fov_edge_[i * 200 + j];
    }
  }
  // viz_img(img_origin, "is_fov_edge");
}

bool FrontierManager::is_fov_edge(const PointType &pt) {
  return frtd_.is_fov_edge_[surface_pos2idx(pt)];
}

void FrontierManager::updateFrontierClusters(
    vector<ClusterInfo::Ptr> &cluster_updated, vector<int> &cluster_removed) {
  PointVector frt_new;
  //26领域检测是否有dense邻居或者sparse邻居
  auto has_dense_nbr = [&](const Eigen::Vector3i &idx) -> bool {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          if (get_state(idx + Eigen::Vector3i(i, j, k)) == DENSE) {
            return true;
          }
        }
      }
    }
    return false;
  };
  auto has_sparse_nbr = [&](const Eigen::Vector3i &idx) -> bool {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          if (get_state(idx + Eigen::Vector3i(i, j, k)) == SPARSE ||
              get_state(idx + Eigen::Vector3i(i, j, k)) == FRONTIER_DIR ||
              get_state(idx + Eigen::Vector3i(i, j, k)) == FRONTIER_DIS) {
            return true;
          }
        }
      }
    }
    return false;
  };

  ros::Time t1 = ros::Time::now();
  vector<Eigen::Vector3i> cells_2_update;
  update_lidar_pos();
  // Step1: 更新视角
  ros::Time t2 = ros::Time::now();
  static vector<PointVector> pts_vec;
  static int idx = 0;
  if (pts_vec.size() < 5) {
    pts_vec.push_back(lidar_map_interface_->ld_->lidar_cloud_.points);
    idx++;
  } else {
    idx = idx % 5;
    pts_vec[idx] = lidar_map_interface_->ld_->lidar_cloud_.points;
    idx++;
  }
  //将点云投影到深度图上，更新gap点和fov边缘点
  vector<float> depth = vector<float>(20000, -0.1);
  project_pts_2_depth_image(lidar_map_interface_->ld_->lidar_cloud_.points, depth);
  update_lidar_fov_edge(depth); // handle 雷达保护罩/旋翼/近点之类的东西
  for (int i = 0; i < pts_vec.size(); i++) {
    if (i == idx - 1)
      continue;
    project_pts_2_depth_image(pts_vec[i], depth);
  }
  update_lidar_pt_gap(depth);
  // 把gap-point可视化出来
  get_cells_2_update(lidar_map_interface_->ld_->lidar_cloud_.points,
                     cells_2_update);
  for (auto &cell : cells_2_update) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    frtd_.frt_map_.erase(bytes);
  }
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();

  // These clouds are diagnostic-only.  Building and serializing them for every
  // lidar callback used to consume a measurable part of the single ROS callback
  // queue even when frontier visualization was disabled.
  if (frtp_.view_frt_) {
    PointVector bad_observation, good_observation;
    bad_observation.reserve(cells_2_update.size());
    good_observation.reserve(cells_2_update.size());
    for (auto &cell : cells_2_update) {
      PointType pt;
      idx2pos(cell, pt);
      if (is_gap_point(pt) || is_fov_edge(pt) ||
          (pt.getVector3fMap() - lidar_position).norm() >
              frtp_.good_observation_trust_length_) {
        bad_observation.push_back(pt);
      } else {
        good_observation.push_back(pt);
      }
    }
    viz_point(bad_observation, "bad_obs");
    viz_point(good_observation, "good_obs");
  }
  // cout << "update and vizgap: " << (ros::Time::now() - t2).toSec() * 1000
  // <<"----------------------------------------"<< endl;
  unordered_set<Eigen::Vector3i, Vector3i_Hash> old_frt_cells;
  old_frt_cells.reserve(cells_2_update.size());
  for (auto &cell : cells_2_update) {
    if (get_state(cell) == FRONTIER_DIS || get_state(cell) == FRONTIER_DIR)
      old_frt_cells.insert(cell);
  }
  // cout << "get_cells_2_update: " << (ros::Time::now() - t1).toSec() * 1000 <<
  // "----------------------------------------"<< endl;

  /*
  距离合适 && 视角合理->good
  距离过长 || 视角过大->bad
  距离特别短 ->good
  */

  ros::Time t3 = ros::Time::now();

  // Step2: 分类，距离特别短的->good, 距离合适 && 视角合理->good
  vector<Eigen::Vector3i> cells_2_box_search;
  cells_2_box_search.reserve(cells_2_update.size());
  unordered_set<Eigen::Vector3i, Vector3i_Hash> bad_dis_set, bad_dir_set;
  for (int i = 0; i < cells_2_update.size(); i++) {
    ByteArrayRaw bytes;
    idx2bytes(cells_2_update[i], bytes);
    if (get_state(cells_2_update[i]) == DENSE) {
      continue;
    }
    PointType pt;
    idx2pos(cells_2_update[i], pt);
    float view_distance = (pt.getVector3fMap() - lidar_position).norm();
    if (view_distance < frtp_.good_observation_force_trust_length_ &&
        !is_fov_edge(pt)) {
      // if (view_distance < frtp_.good_observation_force_trust_length_) {
      frtd_.label_map_[bytes] = DENSE;
      continue;
    }
    bool bad_dir = is_gap_point(pt) || is_fov_edge(pt);
    bool bad_dis = view_distance > frtp_.good_observation_trust_length_;
    if (!bad_dir && !bad_dis) {
      frtd_.label_map_[bytes] = DENSE;
      continue;
    } else if (bad_dis) {
      bad_dis_set.insert(cells_2_update[i]);
    } else {
      bad_dir_set.insert(cells_2_update[i]);
    }
    cells_2_box_search.push_back(cells_2_update[i]);
  }
  // Step3: 分类，距离过长 || 视角过大->bad ,
  // 但要计算法向量判断一下是否是噪声点，如果是噪声点也设成good
  vector<PointVector> pts_inside;
  // cout << "prepare && split dense node: " << (ros::Time::now() - t3).toSec()
  // * 1000 << endl;
  ros::Time t4_ = ros::Time::now();
  get_pts_in_cells(cells_2_box_search, pts_inside);
  // cout << "get_pts_in_cells " << (ros::Time::now() - t4_).toSec() * 1000 <<
  // endl;
  ros::Time t4 = ros::Time::now();

  // 全都设置成sparse先
  unordered_set<Eigen::Vector3i, Vector3i_Hash> old_frt_set;
  for (auto &cell : cells_2_box_search) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    if (get_state(cell) == FRONTIER_DIR || get_state(cell) == FRONTIER_DIS) {
      old_frt_set.insert(cell);
    }
    frtd_.label_map_[bytes] = SPARSE;
    frtd_.frt_map_[bytes] = Eigen::Vector3f::Zero();
  }

  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < cells_2_box_search.size(); i++) {
    auto local_pts = pts_inside[i];
    if (local_pts.size() < 3)
      continue;
    else {
      ByteArrayRaw bytes;
      idx2bytes(cells_2_box_search[i], bytes);
      Eigen::Vector3f norm;
      Eigen::Vector3f t;
      // 使用view_directon(类似ray-cast)去噪
      computeNormalCell(local_pts, norm, t);
      Eigen::Vector3i norm_nbr_1, norm_nbr_2, norm_nbr_3;
      PointType pt;
      idx2pos(cells_2_box_search[i], pt);
      // norm = (lidar_position - pt.getVector3fMap()).normalized();
      pos2idx(pt.getVector3fMap() + norm * frtp_.cell_size_, norm_nbr_1);
      pos2idx(pt.getVector3fMap() - norm * frtp_.cell_size_, norm_nbr_2);
      pos2idx(pt.getVector3fMap() - 2 * norm * frtp_.cell_size_, norm_nbr_3);
      if (get_state(norm_nbr_1) == DENSE || get_state(norm_nbr_2) == DENSE ||
          get_state(norm_nbr_3) == DENSE) {
        frtd_.label_map_[bytes] = DENSE; // 说明这个点是噪声点
        // frt_map_[cells_2_box_search[i]] = norm;

        continue;
      } else {
        frtd_.frt_map_[bytes] = norm;
      }
    }
  }
  frt_new.clear();
  frt_new.reserve(cells_2_box_search.size());
  for (auto &cell : cells_2_box_search) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    if (is_boundary_cell(cell)) {
      // The outermost exploration-box layer is a configured task boundary,
      // not an observable surface objective.  Keeping it as FRONTIER_DIR made
      // vertical FOV edge returns at the box ceiling persist indefinitely.
      frtd_.label_map_[bytes] = SPARSE;
      frtd_.frt_map_.erase(bytes);
      continue;
    }
    if (get_state(cell) == SPARSE && has_dense_nbr(cell)) {
      if (bad_dis_set.count(cell)) {
        frtd_.label_map_[bytes] = FRONTIER_DIS;
      } else if (bad_dir_set.count(cell)) {
        frtd_.label_map_[bytes] = FRONTIER_DIR;
      } else {
        // The cell changed concurrently between classification passes. It has
        // no trustworthy frontier cause, so keep it non-executable and let the
        // next incremental update reconsider it.
        ROS_WARN_THROTTLE(
            1.0, "[frontier label] sparse/dense transition lost its cause");
        frtd_.label_map_[bytes] = SPARSE;
        frtd_.frt_map_.erase(bytes);
        continue;
      }
      if (old_frt_set.count(cell) == 0) {
        PointType pt;
        idx2pos(cell, pt);
        frt_new.push_back(pt);
      }
    } else {
      frtd_.frt_map_.erase(bytes);
    }
  }

  // cout << "set normal " << (ros::Time::now() - t4).toSec() * 1000 << endl;
  ros::Time t5 = ros::Time::now();
  PointVector updated_frt_pts;
  for (auto &cell : old_frt_cells) {
    if (get_state(cell) != FRONTIER_DIR && get_state(cell) != FRONTIER_DIS) {
      // updated_frt_pts.emplace_back
      PointType pt;
      idx2pos(cell, pt);
      updated_frt_pts.emplace_back(pt);
    }
  }
  PointVector updated_pts;
  updated_pts.insert(updated_pts.end(), updated_frt_pts.begin(),
                     updated_frt_pts.end());
  updated_pts.insert(updated_pts.end(), frt_new.begin(), frt_new.end());
  update_updating_aabb(updated_pts);
  cluster_frts(frt_new, cluster_updated, cluster_removed);
  // Reconstructed cluster IDs change on nearly every sensor callback.  FINISH
  // auditing cares about semantic frontier changes, not those transient IDs.
  // A forced refresh temporarily marks clusters reachable and is finalized
  // after viewpoint revalidation in generateTSPViewpoints().
  if (!force_refresh_running_) {
    refreshSemanticRevision();
  }
}

int FrontierManager::surface_pos2idx(const PointType &pt) {
  Eigen::Vector3f pt_lidar_frame = transform_world2lidar * pt.getVector3fMap();
  Eigen::Vector2i surface_idx;
  Sphere_PosToIndex(Eigen::Vector3f::Zero(), pt_lidar_frame,
                                      surface_idx);
  return surface_idx.x() * 200 + surface_idx.y();
}

void FrontierManager::update_lidar_pos() {
  transform_world2lidar = Eigen::Isometry3f::Identity();
  transform_world2lidar.translate(
      lidar_map_interface_->ld_->lidar_pose_.cast<float>());
  transform_world2lidar.rotate(lidar_map_interface_->ld_->lidar_q_.cast<float>());
  transform_world2lidar = transform_world2lidar.inverse();
}

void FrontierManager::project_pts_2_depth_image(PointVector &pts_vec,
                                                vector<float> &depth_img) {
  PointVector pts_lidar_frame;
  // depth_img = vector<float>(20000, -0.1);
  auto project_pt = [&](PointType &pt) {
    Eigen::Vector3f pt_lidar_frame =
        transform_world2lidar * pt.getVector3fMap();
    float dis = pt_lidar_frame.norm();
    if (dis > frtp_.update_length_)
      return;
    Eigen::Vector2i surface_idx;
    Sphere_PosToIndex(Eigen::Vector3f::Zero(), pt_lidar_frame,
                                        surface_idx);
    if (depth_img[surface_idx.x() * 200 + surface_idx.y()] < 0 ||
        depth_img[surface_idx.x() * 200 + surface_idx.y()] > dis) {
      depth_img[surface_idx.x() * 200 + surface_idx.y()] = dis;
    }
  };
  for (auto &pt : pts_vec)
    project_pt(pt);
}

void FrontierManager::update_updating_aabb(const PointVector &new_frt_pts) {
  frtd_.updating_aabb_min = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max());
  frtd_.updating_aabb_max =
      Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
  for (auto &p : new_frt_pts) {
    frtd_.updating_aabb_min =
        frtd_.updating_aabb_min.cwiseMin(p.getVector3fMap());
    frtd_.updating_aabb_max =
        frtd_.updating_aabb_max.cwiseMax(p.getVector3fMap());
  }
  frtd_.updating_aabb_min -= Eigen::Vector3f::Ones() * 0.1;
  frtd_.updating_aabb_max += Eigen::Vector3f::Ones() * 0.1;
}

void FrontierManager::compute_cluster_info(
    const PointVector &frt_pts, const vector<Eigen::Vector3f> &frt_norms,
    ClusterInfo::Ptr cluster) {
  static int id = 0;
  cluster->center_.setZero();
  cluster->normal_.setZero();
  cluster->cells_.resize(frt_pts.size());
  cluster->norms_.resize(frt_pts.size());
  cluster->box_max_ = Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest());
  cluster->box_min_ = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max());
  int fov_edge_count = 0;
  int gap_count = 0;
  for (int i = 0; i < frt_pts.size(); i++) {
    Eigen::Vector3f pt = frt_pts[i].getVector3fMap();
    Eigen::Vector3f norm = frt_norms[i];
    cluster->center_ += pt;
    cluster->normal_ += norm;
    cluster->box_max_ = cluster->box_max_.cwiseMax(pt);
    cluster->box_min_ = cluster->box_min_.cwiseMin(pt);
    cluster->cells_[i] = PointType(pt.x(), pt.y(), pt.z());
    cluster->norms_[i] = norm;
    if (!frtd_.is_fov_edge_.empty() && is_fov_edge(cluster->cells_[i])) {
      ++fov_edge_count;
    }
    if (!frtd_.is_gap_.empty() && is_gap_point(cluster->cells_[i])) {
      ++gap_count;
    }
  }
  cluster->box_max_ += Eigen::Vector3f::Ones() * 0.1;
  cluster->box_min_ -= Eigen::Vector3f::Ones() * 0.1;
  cluster->center_ /= (float)frt_pts.size();
  cluster->normal_.normalize();
  cluster->id_ = id++;
  cluster->is_dormant_ = false;
  // cluster->is_reachable_ = false;
  cluster->is_reachable_ = true;
  if ((cluster->box_max_ - cluster->box_min_).maxCoeff() <
      frtp_.cluster_min_size_)
    cluster->is_dormant_ = true;
  if (cluster->cells_.size() < frtp_.cluster_min_size_)
    cluster->is_dormant_ = true;
  cluster->is_new_cluster_ = true;
  cluster->observation_count_ = 1;
  cluster->state_ =
      cluster->is_dormant_ ? FrontierState::SUSPENDED : FrontierState::ACTIVE;
  cluster->visible_fail_count_ = 0;
  cluster->reachable_fail_count_ = 0;
  cluster->selected_count_ = 0;
  cluster->last_seen_time_ = ros::Time::now();
  cluster->last_selected_time_ = ros::Time(0);
  cluster->first_reachable_time_ =
      cluster->is_dormant_ ? ros::Time(0) : ros::Time::now();
  cluster->last_goal_time_ = ros::Time(0);
  cluster->last_pass_time_ = ros::Time(0);
  cluster->goal_selected_count_ = 0;
  cluster->pass_count_ = 0;
  cluster->pass_debt_ = 0.0;
  cluster->inside_pass_zone_ = false;
  cluster->pass_zone_min_distance_ =
      std::numeric_limits<double>::infinity();
  cluster->last_score_ = 0.0;
  cluster->stable_score_ = 0.0;
  cluster->last_visible_gain_ = 0.0;
  cluster->stable_visible_gain_ = 0.0;
  cluster->fov_edge_ratio_ =
      frt_pts.empty() ? 0.0
                      : static_cast<double>(fov_edge_count) / frt_pts.size();
  cluster->gap_ratio_ =
      frt_pts.empty() ? 0.0 : static_cast<double>(gap_count) / frt_pts.size();
  cluster->needs_revalidation_ = global_audit_pending_;
  cluster->best_vp_ = Eigen::Vector3f::Zero();
  cluster->best_vp_yaw_ = 0.0f;
  cluster->candidate_vps_.clear();
  cluster->candidate_yaws_.clear();
  cluster->candidate_scores_.clear();
}

bool FrontierManager::has_overlap(const Eigen::Vector3f &box_max_,
                                  const Eigen::Vector3f &box_min_) {
  if (frtd_.updating_aabb_max.x() < box_min_.x() ||
      frtd_.updating_aabb_max.y() < box_min_.y() ||
      frtd_.updating_aabb_max.z() < box_min_.z() ||
      frtd_.updating_aabb_min.x() > box_max_.x() ||
      frtd_.updating_aabb_min.y() > box_max_.y() ||
      frtd_.updating_aabb_min.z() > box_max_.z()) {
    return false;
  }
  return true;
}

void FrontierManager::updateHalfSpaces(vector<ClusterInfo::Ptr> &clusters) {
  auto getNbrs = [&](Eigen::Vector3i &idx, vector<Eigen::Vector3i> &nbrs) {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          nbrs.emplace_back(idx[0] + i, idx[1] + j, idx[2] + k);
        }
      }
    }
  };
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &cluster : clusters) {
    unordered_set<Eigen::Vector3i, Vector3i_Hash> dense, sparse;
    for (auto &cell : cluster->cells_) {
      Eigen::Vector3i idx;
      pos2idx(cell, idx);
      vector<Eigen::Vector3i> nbrs;
      getNbrs(idx, nbrs);
      for (auto &nbr : nbrs) {
        if (get_state(nbr) == DENSE) {
          dense.insert(nbr);
        } else if (get_state(nbr) == UNKNOWN) {
          continue;
        } else {
          sparse.insert(nbr);
        }
      }
    }
    Eigen::Vector3f sparse_center = Eigen::Vector3f::Zero();
    for (auto &cell : sparse) {
      PointType pt;
      idx2pos(cell, pt);
      sparse_center += pt.getVector3fMap();
    }
    sparse_center /= sparse.size();
    Eigen::Vector3f dense_center = Eigen::Vector3f::Zero();
    for (auto &cell : dense) {
      PointType pt;
      idx2pos(cell, pt);
      sparse_center += pt.getVector3fMap();
    }
    dense_center /= dense.size();
    Eigen::Vector3f dir = sparse_center - dense_center;
    dir.z() = 0;
    dir.normalize();
    cluster->view_halfspace_ =
        Eigen::Vector4f(dir.x(), dir.y(), dir.z(), -cluster->center_.dot(dir));
  }
}

inline bool FrontierManager::isInBox(const PointType &pt) {
  return lidar_map_interface_->IsInBox(pt);
}

inline bool FrontierManager::isInBox(const Eigen::Vector3f &pt) {
  return lidar_map_interface_->IsInBox(pt);
}

void FrontierManager::selectBestViewpoint(ClusterInfo::Ptr &cluster) {
  if (cluster->vp_clusters_.empty()) {
    markClusterRetry(cluster, false);
    return;
  }
  PointVector vps;
  for (auto &vp_cluster : cluster->vp_clusters_) {
    vps.insert(vps.end(), vp_cluster.vps_.begin(), vp_cluster.vps_.end());
  }
  if (vps.empty()) {
    markClusterRetry(cluster, false);
    return;
  }
  vector<double> score(vps.size(), 0.0);
  vector<int> visible_gain(vps.size(), 0);
  vector<float> yaw(vps.size(), 0);
  vector<bool> hard_rejected(vps.size(), false);
  vector<double> known_free_len_arr(vps.size(), 0.0);
  vector<double> clearance_arr(vps.size(),
                               std::numeric_limits<double>::quiet_NaN());
  vector<double> turn_angle_arr(vps.size(), 0.0);
  vector<double> yaw_change_arr(vps.size(), 0.0);
  vector<bool> known_free_rejected(vps.size(), false);
  vector<bool> clearance_rejected(vps.size(), false);
  vector<bool> turn_rejected(vps.size(), false);
  vector<bool> yaw_rejected(vps.size(), false);
  vector<PointVector> occ_free_frts; // raycast成功，但没有考虑视角
  occ_free_frts.resize(vps.size(), PointVector());
  const HighSpeedViewScoreContext ctx = high_speed_view_ctx_;
  const bool use_high_speed_score =
      ctx.enabled && ctx.forward_known_free && ctx.clearance;
  Eigen::Vector3f heading_dir(std::cos(ctx.curr_yaw), std::sin(ctx.curr_yaw), 0.0f);
  if (ctx.curr_vel.norm() > 0.5f) {
    heading_dir = ctx.curr_vel.normalized();
  }
  if (heading_dir.norm() < 1.0e-3f) {
    heading_dir = Eigen::Vector3f::UnitX();
  }
  const double heading_known_free =
      use_high_speed_score
          ? ctx.forward_known_free(ctx.curr_pos.cast<double>(),
                                   heading_dir.cast<double>(),
                                   ctx.known_free_max_len, ctx.min_clearance,
                                   ctx.query_step)
          : 0.0;
  const bool corridor_cruise_mode =
      use_high_speed_score && ctx.corridor_cruise_enable &&
      heading_known_free >= ctx.corridor_known_free_len;
  const double current_speed = ctx.curr_vel.norm();
  const bool hard_gate_active =
      use_high_speed_score && ctx.hard_gate_enable &&
      (current_speed >= ctx.high_speed_threshold || corridor_cruise_mode);
  RayCaster ray_caster;
  ray_caster.setParams(double(frtp_.cell_size_), frtp_.map_min_.cast<double>());
  for (int i = 0; i < vps.size(); i++) {
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    for (int j = 0; j < cluster->cells_.size(); j++) {
      Eigen::Vector3f frt = cluster->cells_[j].getVector3fMap();
      Eigen::Vector3f dir = (frt - vp).cast<float>();
      float distance = dir.norm();
      dir.normalize();
      if (distance > frtp_.good_observation_trust_length_)
        continue;
      CELL_STATE state = get_state(cluster->cells_[j]);
      if (state == FRONTIER_DIR &&
          distance > frtp_.good_observation_force_trust_length_)
        continue;
      Eigen::Vector3f norm = cluster->norms_[j];
      float sin_theta = dir.dot(norm);
      float cos_thera = sqrt(1 - sin_theta * sin_theta);
      float delta = M_PI / 100.0;
      float score = sin_theta / (sin_theta + delta * cos_thera);
      if (score < frtp_.good_observation_direction_score_)
        continue;
      ray_caster.input(frt.cast<double>(), vp.cast<double>());
      bool visib = true;
      Eigen::Vector3i idx;
      while (ray_caster.nextId(idx)) {
        // 必须在box里
        CELL_STATE state = get_state(idx);
        PointType pt;
        idx2pos(idx, pt);
        if (!lidar_map_interface_->IsInBox(pt) || state == DENSE ||
            state == SPARSE) {
          visib = false;
          break;
        }
      }
      if (visib) {
        occ_free_frts[i].push_back(cluster->cells_[j]);
      }
    }
  }
  for (int i = 0; i < vps.size(); i++) {
    if (occ_free_frts[i].size() < 3) {
      continue;
    }
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    const float reference_yaw = normalizeYawDiff(ctx.curr_yaw);
    const vector<float> yaw_candidates =
        viewpoint_yaw_selector::buildCandidates(reference_yaw);
    vector<int> yaw_score(yaw_candidates.size(), 0);
    for (int yaw_idx = 0;
         yaw_idx < static_cast<int>(yaw_candidates.size()); ++yaw_idx) {
      const float candidate_yaw = yaw_candidates[yaw_idx];
      Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
      transform.rotate(Eigen::AngleAxisf(-vpp_.lidar_pitch_ * M_PI / 180.0,
                                         Eigen::Vector3f::UnitY()));
      transform.rotate(
          Eigen::AngleAxisf(-candidate_yaw, Eigen::Vector3f::UnitZ()));
      for (auto &pt : occ_free_frts[i]) {
        Eigen::Vector3f pt2see = transform * (pt.getVector3fMap() - vp);
        float pitch = atan2(pt2see.z(), sqrt(pt2see.x() * pt2see.x() +
                                             pt2see.y() * pt2see.y()));
        if (pitch > vpp_.fov_up_ || pitch < vpp_.fov_down_)
          continue;
        yaw_score[yaw_idx]++;
      }
    }

    const float yaw_limit =
        hard_gate_active
            ? static_cast<float>(ctx.hard_gate_max_yaw_delta)
            : std::numeric_limits<float>::infinity();
    auto yaw_selection = viewpoint_yaw_selector::select(
        yaw_score, yaw_candidates, reference_yaw, yaw_limit);
    if (!yaw_selection.valid() && hard_gate_active) {
      // Preserve an unconstrained positive-gain result so that the hard-gate
      // diagnostics below report yaw as the actual rejection reason.
      yaw_selection = viewpoint_yaw_selector::select(
          yaw_score, yaw_candidates, reference_yaw);
    }
    if (!yaw_selection.valid()) {
      continue;
    }
    visible_gain[i] = yaw_selection.visible_gain;
    score[i] = visible_gain[i];
    // Eigen::Vector4f hs = cluster->view_halfspace_;
    // Eigen::Vector4f vp_h(vp.x(), vp.y(), vp.z(), 1.0);
    // if (vp_h.dot(hs) < -0.1) {
    //   score[i] *= (1.5 - vp_h.dot(hs));
    // }
    yaw[i] = yaw_candidates[yaw_selection.index];

    if (use_high_speed_score && visible_gain[i] > 0) {
      Eigen::Vector3f to_vp = vp - ctx.curr_pos;
      const double dist = to_vp.norm();
      if (dist > 1.0e-3) {
        Eigen::Vector3f dir = to_vp.normalized();
        const double dot =
            std::clamp<double>(dir.dot(heading_dir), -1.0, 1.0);
        const double forward_progress =
            std::max(0.0, dot) * std::min(dist, ctx.known_free_max_len);
        const double velocity_alignment =
            0.5 * (dot + 1.0);
        const double known_free_len = ctx.forward_known_free(
            ctx.curr_pos.cast<double>(), to_vp.cast<double>(),
            std::min(ctx.known_free_max_len, dist), ctx.min_clearance,
            ctx.query_step);
        double clearance = ctx.clearance(vp.cast<double>());
        if (!std::isfinite(clearance)) {
          clearance = 0.0;
        }
        const double yaw_change =
            std::fabs(normalizeYawDiff(yaw[i] - ctx.curr_yaw));
        const double turn_angle =
            current_speed > 0.5 ? std::acos(dot) : 0.0;
        const bool high_speed_mode =
            current_speed >= ctx.high_speed_threshold || corridor_cruise_mode;
        const double align_weight =
            ctx.velocity_align_weight * (high_speed_mode ? 1.6 : 1.0);
        const double known_weight =
            ctx.known_free_weight * (high_speed_mode ? 1.5 : 1.0);
        const double turn_weight =
            ctx.turn_weight * (high_speed_mode ? 1.6 : 1.0);
        const bool backup_infeasible =
            known_free_len < std::min(ctx.backup_required_len, dist) ||
            clearance < ctx.min_clearance;
        known_free_len_arr[i] = known_free_len;
        clearance_arr[i] = clearance;
        turn_angle_arr[i] = turn_angle;
        yaw_change_arr[i] = yaw_change;
        if (hard_gate_active) {
          const double required_known =
              std::min({dist, ctx.backup_required_len, ctx.known_free_max_len}) *
              std::max(0.0, ctx.hard_gate_min_known_free_ratio);
          const double required_clearance =
              std::max(ctx.min_clearance, ctx.hard_gate_min_clearance);
          known_free_rejected[i] =
              known_free_len + 1.0e-3 < required_known;
          clearance_rejected[i] = clearance < required_clearance;
          turn_rejected[i] = turn_angle > ctx.hard_gate_max_turn_angle;
          yaw_rejected[i] = yaw_change > ctx.hard_gate_max_yaw_delta;
          hard_rejected[i] = known_free_rejected[i] ||
                             clearance_rejected[i] || turn_rejected[i] ||
                             yaw_rejected[i];
        }
        score[i] = ctx.gain_weight * visible_gain[i] +
                   ctx.progress_weight * forward_progress +
                   align_weight * velocity_alignment +
                   known_weight *
                       std::min(known_free_len, ctx.known_free_max_len) +
                   ctx.clearance_weight * std::min(clearance, 5.0) -
                   ctx.yaw_weight * yaw_change -
                   turn_weight * turn_angle -
                   (backup_infeasible ? ctx.backup_penalty : 0.0);
        if (corridor_cruise_mode) {
          const double alignment_bonus =
              ctx.corridor_forward_weight * std::max(0.0, dot) *
              std::min({dist, known_free_len, ctx.known_free_max_len});
          const double lateral_penalty =
              dot < ctx.corridor_min_alignment
                  ? ctx.corridor_lateral_penalty *
                        (ctx.corridor_min_alignment - dot)
                  : 0.0;
          const double backward_penalty =
              dot < 0.0 ? ctx.corridor_lateral_penalty * (-dot) : 0.0;
          score[i] += alignment_bonus - lateral_penalty - backward_penalty;
        }
      }
    }
  }

  vector<int> candidate_indices;
  candidate_indices.reserve(vps.size());
  int hard_rejected_count = 0;
  int known_free_rejected_count = 0;
  int clearance_rejected_count = 0;
  int turn_rejected_count = 0;
  int yaw_rejected_count = 0;
  for (int i = 0; i < static_cast<int>(vps.size()); ++i) {
    if (visible_gain[i] == 0) {
      continue;
    }
    if (hard_gate_active && hard_rejected[i]) {
      ++hard_rejected_count;
      known_free_rejected_count += known_free_rejected[i] ? 1 : 0;
      clearance_rejected_count += clearance_rejected[i] ? 1 : 0;
      turn_rejected_count += turn_rejected[i] ? 1 : 0;
      yaw_rejected_count += yaw_rejected[i] ? 1 : 0;
      continue;
    }
    candidate_indices.push_back(i);
  }

  if (candidate_indices.empty()) {
    cluster->last_score_ = 0.0;
    cluster->stable_score_ *= kStableScoreAlpha;
    cluster->candidate_vps_.clear();
    cluster->candidate_yaws_.clear();
    cluster->candidate_scores_.clear();
    markClusterRetry(cluster, true);
    if (ctx.log && hard_gate_active && hard_rejected_count > 0) {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[view gate] reject cluster="
                   << cluster->id_ << " visible_rejected="
                   << hard_rejected_count << " speed=" << current_speed
                   << " obs=" << cluster->observation_count_
                   << " reasons{known_free=" << known_free_rejected_count
                   << ",clearance=" << clearance_rejected_count
                   << ",turn=" << turn_rejected_count
                   << ",yaw=" << yaw_rejected_count << "}"
                   << " fov_edge_ratio=" << cluster->fov_edge_ratio_
                   << " gap_ratio=" << cluster->gap_ratio_);
    }
  } else {
    std::stable_sort(candidate_indices.begin(), candidate_indices.end(),
                     [&](const int a, const int b) {
                       return score[a] > score[b];
                     });
    const int best_vp_idx = candidate_indices.front();
    markClusterActive(cluster);
    cluster->best_vp_yaw_ = yaw[best_vp_idx];
    cluster->best_vp_ = vps[best_vp_idx].getVector3fMap();
    if (((cluster->best_vp_ - graph_->odom_node_->center_).norm() < 1e-2) &&
        (fabs(normalizeYawDiff(cluster->best_vp_yaw_ -
                              graph_->odom_node_->yaw_)) < 1e-2)) {
      cluster->candidate_vps_.clear();
      cluster->candidate_yaws_.clear();
      cluster->candidate_scores_.clear();
      markClusterRetry(cluster, true);
      return;
    }
    cluster->state_ = FrontierState::SELECTED;
    cluster->selected_count_++;
    cluster->last_selected_time_ = ros::Time::now();
    cluster->last_score_ = score[best_vp_idx];
    cluster->last_visible_gain_ = visible_gain[best_vp_idx];
    cluster->stable_score_ =
        cluster->stable_score_ <= 1.0e-6
            ? cluster->last_score_
            : kStableScoreAlpha * cluster->stable_score_ +
                  (1.0 - kStableScoreAlpha) * cluster->last_score_;
    cluster->stable_visible_gain_ =
        cluster->stable_visible_gain_ <= 1.0e-6
            ? cluster->last_visible_gain_
            : kStableScoreAlpha * cluster->stable_visible_gain_ +
                  (1.0 - kStableScoreAlpha) * cluster->last_visible_gain_;
    auto viewpointClusterDistance = [&](const int vp_idx) {
      int tmp_idx = vp_idx;
      for (auto &vpc : cluster->vp_clusters_) {
        if (tmp_idx < static_cast<int>(vpc.vps_.size())) {
          return vpc.distance_;
        }
        tmp_idx -= static_cast<int>(vpc.vps_.size());
      }
      return cluster->distance_;
    };
    cluster->distance_ = viewpointClusterDistance(best_vp_idx);
    cluster->candidate_vps_.clear();
    cluster->candidate_yaws_.clear();
    cluster->candidate_scores_.clear();
    const int top_k =
        std::min(static_cast<int>(candidate_indices.size()),
                 std::max(1, std::min(vpp_.top_candidate_num_,
                                      std::max(1, ctx.top_viewpoint_num))));
    for (int k = 0; k < top_k; ++k) {
      const int idx = candidate_indices[k];
      cluster->candidate_vps_.push_back(vps[idx].getVector3fMap());
      cluster->candidate_yaws_.push_back(yaw[idx]);
      cluster->candidate_scores_.push_back(score[idx]);
    }
    if (ctx.log && use_high_speed_score) {
      ROS_INFO_STREAM_THROTTLE(
          0.5,
          "[view score] cluster=" << cluster->id_
                                  << " best_score=" << score[best_vp_idx]
                                  << " visible_gain="
                                  << visible_gain[best_vp_idx]
                                  << " speed=" << current_speed
                                  << " corridor_cruise="
                                  << corridor_cruise_mode
                                  << " hard_gate=" << hard_gate_active
                                  << " rejected=" << hard_rejected_count
                                  << " top_k=" << top_k
                                  << " known_free="
                                  << known_free_len_arr[best_vp_idx]
                                  << " clearance="
                                  << clearance_arr[best_vp_idx]
                                  << " turn="
                                  << turn_angle_arr[best_vp_idx]
                                  << " selected_yaw=" << yaw[best_vp_idx]
                                  << " current_yaw=" << ctx.curr_yaw
                                  << " yaw_delta="
                                  << yaw_change_arr[best_vp_idx]
                                  << " heading_known_free="
                                  << heading_known_free
                                  << " vp=(" << cluster->best_vp_.x()
                                  << ", " << cluster->best_vp_.y()
                                  << ", " << cluster->best_vp_.z() << ")");
    }
  }
}

void FrontierManager::initClusterViewpoints(ClusterInfo::Ptr &cluster) {
  cluster->vp_clusters_.clear();
  PointVector vps_init;
  vps_init.reserve(origin_viewpoints_.size());
  for (auto &ovp : origin_viewpoints_) {
    Eigen::Vector3f vp = ovp + cluster->center_;
    if (lidar_map_interface_->getDisToOcc(vp) < 0.9)
      continue;
    if (!isInBox(vp))
      continue;
    Eigen::Vector3i idx;
    graph_->getIndex(vp, idx);
    if (graph_->getRegionNode(idx) == nullptr)
      continue;
    vps_init.emplace_back(vp.x(), vp.y(), vp.z());
  }
  if (vps_init.empty()) {
    markClusterRetry(cluster, false);
    return;
  }
  markClusterActive(cluster);
  pcl::PointCloud<PointType>::Ptr vp_cloud(new pcl::PointCloud<PointType>);
  vp_cloud->points = vps_init;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(vp_cloud);
  vector<float> radius_vec;
  radius_vec.resize(vps_init.size(), 0.0);
  for (int i = 0; i < vps_init.size(); i++) {
    radius_vec[i] = lidar_map_interface_->getDisToOcc(vps_init[i]);
  }
  // DB-SCAN 基于连通性将初始viewpoint聚成几类
  std::vector<int> labels;
  labels.resize(vps_init.size(), -1); // 初始化标签，-1 表示未访问
  auto getNbrs = [&](int idx, vector<int> &nbr_idx) -> int {
    vector<float> sqr_distances;
    PointType p = vps_init[idx];
    vector<int> nbrs_tmp;
    kdtree.radiusSearch(p, radius_vec[idx], nbrs_tmp, sqr_distances);
    nbr_idx.clear();
    for (int i = 0; i < nbrs_tmp.size(); i++) {
      if (labels[nbrs_tmp[i]] == -1)
        nbr_idx.push_back(nbrs_tmp[i]);
    }
    return nbr_idx.size();
  };

  int cluster_id = 0; // 聚类ID
  for (int i = 0; i < vps_init.size(); i++) {
    if (labels[i] != -1)
      continue;
    vector<int> nbr_idx;
    if (getNbrs(i, nbr_idx) == 0)
      continue;
    cluster_id++;
    labels[i] = cluster_id;
    std::list<size_t> queue;
    queue.push_back(i);
    while (!queue.empty()) {
      size_t current = queue.front();
      queue.pop_front();
      if (getNbrs(current, nbr_idx) == 0)
        continue;
      for (int j = 0; j < nbr_idx.size(); j++) {
        auto nbr = nbr_idx[j];
        if (labels[nbr] == -1) {
          labels[nbr] = cluster_id;
          queue.push_back(nbr);
        }
      }
    }
  }
  for (int i = 0; i < cluster_id; i++) {
    ViewpointCluster vp_cluster;
    vp_cluster.vps_.clear();
    vector<float> cls_radius_vec;
    vector<int> cls_idx_vec;
    for (int j = 0; j < labels.size(); j++) {
      if (labels[j] != i + 1)
        continue;
      vp_cluster.vps_.push_back(vps_init[j]);
      cls_radius_vec.push_back(radius_vec[j]);
    }
    for (int j = 0; j < cls_radius_vec.size(); j++) {
      cls_idx_vec.push_back(j);
    }
    sort(cls_idx_vec.begin(), cls_idx_vec.end(),
         [&](int a, int b) { return cls_radius_vec[a] > cls_radius_vec[b]; });
    vp_cluster.center_ = vp_cluster.vps_[cls_idx_vec[0]].getVector3fMap();
    for (int j = 0; j < cls_radius_vec.size(); j++) {
      Eigen::Vector3f pt(vp_cluster.vps_[cls_idx_vec[j]].x,
                         vp_cluster.vps_[cls_idx_vec[j]].y,
                         vp_cluster.vps_[cls_idx_vec[j]].z);
      Eigen::Vector3i idx;
      graph_->getIndex(pt, idx);
      auto region = graph_->getRegionNode(idx);
      if (region && !region->topo_nodes_.empty()) {
        vp_cluster.center_ = vp_cluster.vps_[cls_idx_vec[j]].getVector3fMap();
        break;
      }
    }
    cluster->vp_clusters_.push_back(vp_cluster);
  }
  std::sort(cluster->vp_clusters_.begin(), cluster->vp_clusters_.end(),
            [](const ViewpointCluster &a, const ViewpointCluster &b) {
              return a.vps_.size() > b.vps_.size();
            });
  // cluster->vp_clusters_.resize(min(16, int(cluster->vp_clusters_.size())));
}

void FrontierManager::removeUnreachableViewpoints(
    vector<ClusterInfo::Ptr> &clusters) {
  if (graph_->odom_node_->neighbors_.empty())
    return;
  // 建立一张映射表，可以通过topo-node映射到要删除的vp_cluster
  vector<int> nodeidx2clusteridx;
  vector<int> nodeidx2vpclusteridx;
  vector<TopoNode::Ptr> nodes2insert;
  for (int i = 0; i < clusters.size(); i++) {
    for (int j = 0; j < clusters[i]->vp_clusters_.size(); j++) {
      nodeidx2clusteridx.push_back(i);
      nodeidx2vpclusteridx.push_back(j);
      TopoNode::Ptr vp_node = make_shared<TopoNode>();
      vp_node->center_ = clusters[i]->vp_clusters_[j].center_;
      nodes2insert.push_back(vp_node);
    }
  }

  ros::Time t1 = ros::Time::now();
  graph_->insertNodes(nodes2insert, true); // only_raycast=true 可以显著加速
  ros::Time t2 = ros::Time::now();
  vector<bool> vp_cluster_kept;
  vp_cluster_kept.resize(nodes2insert.size(), true);
  // 可以并行
  for (int i = 0; i < nodes2insert.size(); i++) {
    if (nodes2insert[i]->neighbors_.empty()) {
      vp_cluster_kept[i] = false;
      continue;
    }
    vector<TopoNode::Ptr> topo_path;
    auto closest_node = graph_->odom_node_;
    float closest_dis =
        (closest_node->center_ - nodes2insert[i]->center_).squaredNorm();
    for (auto &hodom : graph_->history_odom_nodes_) {
      if ((hodom->center_ - nodes2insert[i]->center_).squaredNorm() <
          closest_dis) {
        closest_dis = (hodom->center_ - nodes2insert[i]->center_).squaredNorm();
        closest_node = hodom;
      }
    }
    if (!graph_->graphSearch(closest_node, nodes2insert[i], topo_path, 3e-4)) {
      vp_cluster_kept[i] = false;
    } else {
      clusters[nodeidx2clusteridx[i]]
          ->vp_clusters_[nodeidx2vpclusteridx[i]]
          .distance_ = graph_->getPathLength(topo_path);
    }
  }
  graph_->removeNodes(nodes2insert);
  vector<unordered_set<int>> kept_vp_cluster;
  kept_vp_cluster.resize(clusters.size(), unordered_set<int>());
  for (int i = 0; i < vp_cluster_kept.size(); i++) {
    if (!vp_cluster_kept[i])
      continue;
    kept_vp_cluster[nodeidx2clusteridx[i]].insert(nodeidx2vpclusteridx[i]);
  }
  for (int i = 0; i < clusters.size(); i++) {
    vector<ViewpointCluster> tmp;
    tmp.swap(clusters[i]->vp_clusters_);
    for (int j = 0; j < tmp.size(); j++) {
      if (kept_vp_cluster[i].find(j) == kept_vp_cluster[i].end())
        continue;
      clusters[i]->vp_clusters_.push_back(tmp[j]);
    }
    if (clusters[i]->vp_clusters_.empty()) {
      clusters[i]->vp_clusters_.swap(tmp);
      clusters[i]->candidate_vps_.clear();
      clusters[i]->candidate_yaws_.clear();
      clusters[i]->candidate_scores_.clear();
      markClusterRetry(clusters[i], false);
    } else {
      markClusterActive(clusters[i]);
      sort(clusters[i]->vp_clusters_.begin(), clusters[i]->vp_clusters_.end(),
           [](const ViewpointCluster &a, const ViewpointCluster &b) {
             return a.distance_ < b.distance_;
           });
      float min_distance = clusters[i]->vp_clusters_[0].distance_;
      vector<ViewpointCluster> tmp2;
      for (int j = 0; j < min(8, int(clusters[i]->vp_clusters_.size())); j++) {
        if (clusters[i]->vp_clusters_[j].distance_ <= min_distance * 1.35 ||
            clusters[i]->vp_clusters_[j].distance_ <=
                min_distance + vpp_.sample_pillar_max_radius_)
          tmp2.push_back(clusters[i]->vp_clusters_[j]);
      }
      clusters[i]->vp_clusters_.swap(tmp2);
    }
  }
}

void FrontierManager::printMemoryCost() {
  const size_t label_map_size = frtd_.label_map_.size();
  const size_t frt_map_size = frtd_.frt_map_.size();
  size_t cluster_cells = 0;
  size_t candidate_count = 0;
  size_t cached_viewpoints = 0;
  size_t terminal_clusters = 0;
  for (const auto &cluster : cluster_list_) {
    if (!cluster) {
      continue;
    }
    cluster_cells += cluster->cells_.capacity();
    candidate_count += cluster->candidate_vps_.capacity();
    for (const auto &vp_cluster : cluster->vp_clusters_) {
      cached_viewpoints += vp_cluster.vps_.capacity();
    }
    if (isTerminalFrontierState(cluster->state_)) {
      ++terminal_clusters;
    }
  }
  const size_t map_bytes =
      label_map_size * (sizeof(ByteArrayRaw) + sizeof(uint8_t)) +
      frt_map_size * (sizeof(ByteArrayRaw) + sizeof(Eigen::Vector3f)) +
      (frtd_.label_map_.bucket_count() + frtd_.frt_map_.bucket_count()) *
          sizeof(void *);
  const size_t cluster_payload_bytes =
      cluster_cells * (sizeof(PointType) + sizeof(Eigen::Vector3f)) +
      candidate_count *
          (sizeof(Eigen::Vector3f) + sizeof(float) + sizeof(double)) +
      cached_viewpoints * sizeof(PointType);
  ROS_INFO_STREAM("[frontier storage] labels=" << label_map_size
                  << "/" << frtd_.label_map_.bucket_count()
                  << " normals=" << frt_map_size
                  << "/" << frtd_.frt_map_.bucket_count()
                  << " clusters=" << cluster_list_.size()
                  << " terminal=" << terminal_clusters
                  << " cluster_cells(cap)=" << cluster_cells
                  << " candidates(cap)=" << candidate_count
                  << " derived_viewpoints(cap)=" << cached_viewpoints
                  << " estimated_kb="
                  << (map_bytes + cluster_payload_bytes) / 1024.0
                  << " revision=" << frontier_revision_
                  << " audit_ready=" << frontierAuditReady());
  static ros::Publisher mem_pub =
      nh_.advertise<std_msgs::Float32>("/mem_cost", 1);
  static ros::Publisher mem_pub_2 =
      nh_.advertise<std_msgs::Float32>("/mem_cost_2", 1);
  static ros::Publisher mem_pub_3 =
      nh_.advertise<std_msgs::Float32>("/mem_cost_3", 1);
  std_msgs::Float32 msg, msg_2, msg_3;
  msg.data = static_cast<float>(map_bytes) / 1024.0f;
  mem_pub.publish(msg);
  msg_2.data = static_cast<float>(cluster_payload_bytes) / 1024.0f;
  msg_3.data =
      static_cast<float>(map_bytes + cluster_payload_bytes) / 1024.0f;
  mem_pub_2.publish(msg_2);
  mem_pub_3.publish(msg_3);
}
inline void
FrontierManager::Sphere_PosToIndex(const Eigen::Vector3f &lidar_center,
                                   const Eigen::Vector3f &pos,
                                   Eigen::Vector2i &id) {
  // double dis = sqrt(pow((pos(0)-lidar_center(0)),2) +
  // pow((pos(1)-lidar_center(1)),2) + pow((pos(2)-lidar_center(2)),2));
  double dis = (pos - lidar_center).norm(); // pos是被按回去的，卡了最大范围。
  double phi_x = atan2((pos(1) - lidar_center(1)),
                       (pos(0) - lidar_center(0))); // 水平面，以x为极轴的转角
  if (phi_x < 0)
    phi_x = 2 * M_PI + phi_x;                              // 范围是0-2pi
  double theta_z = acos((pos(2) - lidar_center(2)) / dis); // 以z为极轴的转角
  if (theta_z < 0)
    theta_z = 2 * M_PI + theta_z;
  double sphere_r = 1 / M_PI;
  Eigen::Vector2d Arc_l;
  Arc_l(0) = sphere_r * theta_z;
  Arc_l(1) = sphere_r * phi_x;
  for (int i = 0; i < 2; ++i) {
    id(i) = floor(Arc_l(i) * 100);
  }
}
