#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_guidance_manager.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>

#include <visualization_msgs/Marker.h>

namespace fast_planner {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Zone {
  CoverageVoxelState state{CoverageVoxelState::UNKNOWN};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3i cell{Eigen::Vector3i::Zero()};
  int voxel_count{0};
  std::vector<std::pair<int, double>> edges;
};

struct UnknownGroup {
  Eigen::Vector3d weighted_center{Eigen::Vector3d::Zero()};
  std::vector<int> zones;
  int voxel_count{0};
};

bool pointInBox(const Eigen::Vector3d &point, const CoverageBox &box) {
  return (point.array() >= box.min.array()).all() &&
         (point.array() <= box.max.array()).all();
}

std::string groupKey(const Eigen::Vector3d &position,
                     const CoverageMapSpec &spec, double size,
                     bool near) {
  const Eigen::Vector3i index =
      ((position - spec.min) / std::max(1.0e-3, size))
          .array()
          .floor()
          .cast<int>();
  return std::string(near ? "n:" : "m:") + std::to_string(index.x()) +
         ":" + std::to_string(index.y()) + ":" +
         std::to_string(index.z());
}

std::uint64_t stableGroupId(const std::string &key) {
  // FNV-1a is deterministic across processes, unlike std::hash whose result is
  // not part of the C++ stability contract.
  std::uint64_t value = 1469598103934665603ULL;
  for (const unsigned char byte : key) {
    value ^= static_cast<std::uint64_t>(byte);
    value *= 1099511628211ULL;
  }
  return value == 0 ? 1 : value;
}

std::vector<double> shortestDistances(const std::vector<Zone> &zones,
                                      int source) {
  std::vector<double> distance(zones.size(), kInfinity);
  if (source < 0 || source >= static_cast<int>(zones.size())) {
    return distance;
  }
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  distance[source] = 0.0;
  queue.emplace(0.0, source);
  while (!queue.empty()) {
    const double current_distance = queue.top().first;
    const int current = queue.top().second;
    queue.pop();
    if (current_distance > distance[current] + 1.0e-9) {
      continue;
    }
    for (const auto &edge : zones[current].edges) {
      const double candidate = current_distance + edge.second;
      if (candidate + 1.0e-9 < distance[edge.first]) {
        distance[edge.first] = candidate;
        queue.emplace(candidate, edge.first);
      }
    }
  }
  return distance;
}

}  // namespace

CoverageGuidanceManager::CoverageGuidanceManager() = default;

CoverageGuidanceManager::~CoverageGuidanceManager() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

double CoverageGuidanceManager::wallNow() {
  return ros::WallTime::now().toSec();
}

void CoverageGuidanceManager::initialize(ros::NodeHandle &nh,
                                         const CoverageMapSpec &map_spec) {
  map_spec_ = map_spec;
  nh.param("coverage_guidance/mode", config_.mode, config_.mode);
  std::transform(config_.mode.begin(), config_.mode.end(), config_.mode.begin(),
                 [](unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  if (config_.mode != "off" && config_.mode != "shadow" &&
      config_.mode != "soft" && config_.mode != "full") {
    ROS_WARN_STREAM("[coverage guidance] invalid mode='" << config_.mode
                    << "', disable guidance");
    config_.mode = "off";
  }
  nh.param("coverage_guidance/update_period", config_.update_period,
           config_.update_period);
  nh.param("coverage_guidance/stale_timeout", config_.stale_timeout,
           config_.stale_timeout);
  nh.param("coverage_guidance/fine_cell_size", config_.fine_cell_size,
           config_.fine_cell_size);
  nh.param("coverage_guidance/macro_cell_size", config_.macro_cell_size,
           config_.macro_cell_size);
  nh.param("coverage_guidance/near_expand_radius",
           config_.near_expand_radius, config_.near_expand_radius);
  nh.param("coverage_guidance/unknown_edge_penalty",
           config_.unknown_edge_penalty, config_.unknown_edge_penalty);
  nh.param("coverage_guidance/soft_weight", config_.soft_weight,
           config_.soft_weight);
  nh.param("coverage_guidance/full_rank_weight", config_.full_rank_weight,
           config_.full_rank_weight);
  nh.param("coverage_guidance/rank_penalty_cap", config_.rank_penalty_cap,
           config_.rank_penalty_cap);
  nh.param("coverage_guidance/unknown_first_penalty",
           config_.unknown_first_penalty, config_.unknown_first_penalty);
  nh.param("coverage_guidance/min_unknown_voxels",
           config_.min_unknown_voxels, config_.min_unknown_voxels);
  nh.param("coverage_guidance/max_cp_nodes", config_.max_cp_nodes,
           config_.max_cp_nodes);
  nh.param("coverage_guidance/max_preferred_clusters",
           config_.max_preferred_clusters, config_.max_preferred_clusters);
  nh.param("coverage_guidance/finish_guard_enable",
           config_.finish_guard_enable, config_.finish_guard_enable);
  nh.param("coverage_guidance/visualization_enable",
           config_.visualization_enable, config_.visualization_enable);

  config_.update_period = std::max(0.2, config_.update_period);
  config_.stale_timeout =
      std::max(config_.update_period + 0.2, config_.stale_timeout);
  config_.fine_cell_size =
      std::max(map_spec_.resolution, config_.fine_cell_size);
  config_.macro_cell_size =
      std::max(config_.fine_cell_size, config_.macro_cell_size);
  config_.min_unknown_voxels = std::max(1, config_.min_unknown_voxels);
  config_.max_cp_nodes = std::max(8, config_.max_cp_nodes);
  config_.max_preferred_clusters =
      std::max(1, config_.max_preferred_clusters);
  config_.rank_penalty_cap = std::max(0.0, config_.rank_penalty_cap);

  if (!map_spec_.valid()) {
    ROS_ERROR("[coverage guidance] invalid persistent map specification; disable");
    config_.mode = "off";
  }
  if (!enabled()) {
    ROS_INFO("[coverage guidance] mode=off (legacy frontier behavior)");
    return;
  }

  const int voxel_count = map_spec_.dims.prod();
  persistent_map_.assign(voxel_count, CoverageVoxelState::UNKNOWN);
  for (int linear = 0; linear < voxel_count; ++linear) {
    const int x = linear % map_spec_.dims.x();
    const int yz = linear / map_spec_.dims.x();
    const int y = yz % map_spec_.dims.y();
    const int z = yz / map_spec_.dims.y();
    if (!voxelIsValid(
            map_spec_.indexToPosition(Eigen::Vector3i(x, y, z)))) {
      persistent_map_[linear] = CoverageVoxelState::INVALID;
    }
  }
  if (config_.visualization_enable) {
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "coverage_guidance/route", 1, true);
  }
  worker_ = std::thread(&CoverageGuidanceManager::workerLoop, this);
  ROS_INFO_STREAM("[coverage guidance] mode=" << config_.mode
                  << " persistent_voxels=" << voxel_count
                  << " resolution=" << map_spec_.resolution
                  << " fine_cell=" << config_.fine_cell_size
                  << " macro_cell=" << config_.macro_cell_size
                  << " update_period=" << config_.update_period << "s");
}

bool CoverageGuidanceManager::enabled() const {
  return config_.mode != "off";
}

bool CoverageGuidanceManager::affectsPlanning() const {
  return config_.mode == "soft" || config_.mode == "full";
}

bool CoverageGuidanceManager::fullMode() const {
  return config_.mode == "full";
}

bool CoverageGuidanceManager::finishGuardEnabled() const {
  return fullMode() && config_.finish_guard_enable;
}

const std::string &CoverageGuidanceManager::modeName() const {
  return config_.mode;
}

const CoverageMapSpec &CoverageGuidanceManager::mapSpec() const {
  return map_spec_;
}

bool CoverageGuidanceManager::samplingDue() const {
  if (!enabled()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return wallNow() - last_submit_wall_ >= config_.update_period;
}

void CoverageGuidanceManager::submit(
    CoverageMapDelta delta, std::vector<CoverageFrontier> frontiers,
    const Eigen::Vector3d &robot_position) {
  if (!enabled() || !robot_position.allFinite()) {
    return;
  }
  WorkItem work;
  work.delta = std::move(delta);
  work.frontiers = std::move(frontiers);
  work.robot_position = robot_position;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    work.frontier_version = ++frontier_version_;
    last_submit_wall_ = wallNow();
    // Bound latency and memory. Dropping an old item is safe because map
    // deltas are merged into the newest queued item before it is discarded.
    if (pending_work_.size() >= 2) {
      for (WorkItem &queued : pending_work_) {
        work.delta.samples.insert(work.delta.samples.end(),
                                  queued.delta.samples.begin(),
                                  queued.delta.samples.end());
        work.delta.version =
            std::max(work.delta.version, queued.delta.version);
      }
      pending_work_.clear();
      pending_work_.emplace_back(std::move(work));
    } else {
      pending_work_.emplace_back(std::move(work));
    }
  }
  condition_.notify_one();
}

CoveragePlan::Ptr CoverageGuidanceManager::latestUsablePlan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_plan_ || !latest_plan_->valid ||
      wallNow() - latest_plan_->wall_stamp_sec > config_.stale_timeout) {
    return nullptr;
  }
  return latest_plan_;
}

std::unordered_set<int> CoverageGuidanceManager::preferredClusterIds() const {
  const CoveragePlan::Ptr plan = latestUsablePlan();
  if (!affectsPlanning() || !plan) {
    return {};
  }
  return plan->preferred_cluster_ids;
}

double CoverageGuidanceManager::clusterPenalty(
    int cluster_id, const Eigen::Vector3d &position) const {
  const CoveragePlan::Ptr plan = latestUsablePlan();
  if (!affectsPlanning() || !plan) {
    return 0.0;
  }
  const auto found = plan->cluster_priority.find(cluster_id);
  double priority = 1.25;
  if (found != plan->cluster_priority.end()) {
    priority = std::clamp(found->second, 0.0, 1.25);
  } else if (position.allFinite()) {
    int active_ordinal = 0;
    for (const CoverageTarget &target : plan->ordered_targets) {
      if (target.type != CoverageTargetType::ACTIVE_FREE) {
        continue;
      }
      priority = std::min(
          priority,
          std::min(1.25, static_cast<double>(active_ordinal) /
                             std::max(1, plan->active_free_count) +
                             0.15 * (position - target.position).norm() /
                                 std::max(0.5, config_.fine_cell_size)));
      ++active_ordinal;
    }
  }
  const double rank_penalty =
      (fullMode() ? config_.full_rank_weight : config_.soft_weight) *
      priority;
  // Coverage is a long-horizon tie breaker, not permission to ignore a much
  // cheaper executable side-room target.  Bound the extra delay it can assign
  // to any candidate so route rank cannot force a long return through already
  // observed space.
  return config_.rank_penalty_cap > 0.0
             ? std::min(config_.rank_penalty_cap, rank_penalty)
             : rank_penalty;
}

bool CoverageGuidanceManager::blocksFinish() const {
  if (!fullMode() || !config_.finish_guard_enable) {
    return false;
  }
  const CoveragePlan::Ptr plan = latestUsablePlan();
  return plan && plan->reachable_unknown_count > 0;
}

std::vector<CoverageTarget>
CoverageGuidanceManager::unknownApproachTargets(
    const Eigen::Vector3d &robot_position,
    int max_targets,
    double min_travel_distance) const {
  std::vector<CoverageTarget> result;
  const CoveragePlan::Ptr plan = latestUsablePlan();
  if (!fullMode() || !plan || !robot_position.allFinite()) {
    return result;
  }
  max_targets = std::max(1, max_targets);
  min_travel_distance = std::max(0.0, min_travel_distance);
  for (const CoverageTarget &target : plan->ordered_targets) {
    if (target.type != CoverageTargetType::REACHABLE_UNKNOWN ||
        !target.has_approach || !target.approach_position.allFinite() ||
        (target.approach_position - robot_position).norm() <
            min_travel_distance) {
      continue;
    }
    bool duplicate = false;
    for (const CoverageTarget &kept : result) {
      if ((kept.approach_position - target.approach_position).norm() <
          0.5 * config_.fine_cell_size) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      result.emplace_back(target);
      if (static_cast<int>(result.size()) >= max_targets) {
        break;
      }
    }
  }
  return result;
}

bool CoverageGuidanceManager::voxelIsValid(
    const Eigen::Vector3d &position) const {
  for (const CoverageBox &box : map_spec_.dead_boxes) {
    if (pointInBox(position, box)) {
      return false;
    }
  }
  if (map_spec_.valid_boxes.empty()) {
    return true;
  }
  for (const CoverageBox &box : map_spec_.valid_boxes) {
    if (pointInBox(position, box)) {
      return true;
    }
  }
  return false;
}

void CoverageGuidanceManager::workerLoop() {
  while (true) {
    WorkItem work;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&]() { return stop_ || !pending_work_.empty(); });
      if (stop_) {
        return;
      }
      work = std::move(pending_work_.back());
      pending_work_.clear();
    }

    const double begin = wallNow();
    CoveragePlan plan = buildPlan(work);
    plan.compute_ms = 1000.0 * (wallNow() - begin);
    plan.wall_stamp_sec = wallNow();
    const auto immutable = std::make_shared<const CoveragePlan>(std::move(plan));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_plan_ = immutable;
    }
    ROS_INFO_STREAM_THROTTLE(
        2.0, "[coverage guidance] plan valid=" << immutable->valid
              << " map_v=" << immutable->map_version
              << " frontier_v=" << immutable->frontier_version
              << " zones(f/u)=" << immutable->free_zone_count << "/"
              << immutable->unknown_zone_count
              << " observed=" << immutable->observed_voxel_count << "/"
              << immutable->valid_voxel_count
              << " coverage=" << std::fixed << std::setprecision(4)
              << immutable->coverage_ratio
              << " voxels(f/o/u)=" << immutable->known_free_voxel_count
              << "/" << immutable->occupied_voxel_count << "/"
              << immutable->unsafe_free_voxel_count
              << " active=" << immutable->active_free_count
              << " reachable_unknown=" << immutable->reachable_unknown_count
              << " route_nodes=" << immutable->ordered_targets.size()
              << " preferred=" << immutable->preferred_cluster_ids.size()
              << " cost=" << std::fixed << std::setprecision(1)
              << immutable->compute_ms << "ms");
  }
}

CoveragePlan CoverageGuidanceManager::buildPlan(const WorkItem &work) {
  CoveragePlan plan;
  plan.map_version = work.delta.version;
  plan.frontier_version = work.frontier_version;
  if (!map_spec_.valid() || persistent_map_.empty()) {
    return plan;
  }

  for (const CoverageSample &sample : work.delta.samples) {
    if (sample.linear_index < 0 ||
        sample.linear_index >= static_cast<int>(persistent_map_.size()) ||
        persistent_map_[sample.linear_index] == CoverageVoxelState::INVALID ||
        sample.state == CoverageVoxelState::UNKNOWN ||
        sample.state == CoverageVoxelState::INVALID) {
      continue;
    }
    // Unknown observations never erase persistent evidence. Positive free or
    // occupied evidence may update an older state when an area is revisited.
    persistent_map_[sample.linear_index] = sample.state;
  }

  const Eigen::Vector3i dims = map_spec_.dims;
  const int voxel_count = static_cast<int>(persistent_map_.size());
  for (const CoverageVoxelState state : persistent_map_) {
    switch (state) {
      case CoverageVoxelState::KNOWN_FREE:
        ++plan.valid_voxel_count;
        ++plan.observed_voxel_count;
        ++plan.known_free_voxel_count;
        break;
      case CoverageVoxelState::OCCUPIED:
        ++plan.valid_voxel_count;
        ++plan.observed_voxel_count;
        ++plan.occupied_voxel_count;
        break;
      case CoverageVoxelState::UNSAFE_FREE:
        ++plan.valid_voxel_count;
        ++plan.observed_voxel_count;
        ++plan.unsafe_free_voxel_count;
        break;
      case CoverageVoxelState::UNKNOWN:
        ++plan.valid_voxel_count;
        break;
      case CoverageVoxelState::INVALID:
        break;
    }
  }
  plan.coverage_ratio =
      plan.valid_voxel_count > 0
          ? static_cast<double>(plan.observed_voxel_count) /
                plan.valid_voxel_count
          : 0.0;
  const int fine_voxels = std::max(
      1, static_cast<int>(std::round(config_.fine_cell_size /
                                     map_spec_.resolution)));
  std::vector<int> zone_of(voxel_count, -1);
  std::vector<Zone> raw_zones;
  std::queue<int> flood_queue;
  const Eigen::Vector3i directions[6] = {
      Eigen::Vector3i(1, 0, 0),  Eigen::Vector3i(-1, 0, 0),
      Eigen::Vector3i(0, 1, 0),  Eigen::Vector3i(0, -1, 0),
      Eigen::Vector3i(0, 0, 1),  Eigen::Vector3i(0, 0, -1)};

  auto indexFromLinear = [&](int linear) {
    const int x = linear % dims.x();
    const int yz = linear / dims.x();
    return Eigen::Vector3i(x, yz % dims.y(), yz / dims.y());
  };
  auto traversable = [](CoverageVoxelState state) {
    return state == CoverageVoxelState::KNOWN_FREE ||
           state == CoverageVoxelState::UNKNOWN;
  };

  for (int seed = 0; seed < voxel_count; ++seed) {
    if (zone_of[seed] >= 0 || !traversable(persistent_map_[seed])) {
      continue;
    }
    Zone zone;
    zone.state = persistent_map_[seed];
    const Eigen::Vector3i seed_index = indexFromLinear(seed);
    zone.cell = seed_index / fine_voxels;
    const int zone_id = static_cast<int>(raw_zones.size());
    zone_of[seed] = zone_id;
    flood_queue.push(seed);
    while (!flood_queue.empty()) {
      const int current_linear = flood_queue.front();
      flood_queue.pop();
      const Eigen::Vector3i current = indexFromLinear(current_linear);
      zone.center += map_spec_.indexToPosition(current);
      ++zone.voxel_count;
      for (const Eigen::Vector3i &direction : directions) {
        const Eigen::Vector3i next = current + direction;
        if (!map_spec_.contains(next) ||
            (next / fine_voxels) != zone.cell) {
          continue;
        }
        const int next_linear = map_spec_.flatten(next);
        if (zone_of[next_linear] >= 0 ||
            persistent_map_[next_linear] != zone.state) {
          continue;
        }
        zone_of[next_linear] = zone_id;
        flood_queue.push(next_linear);
      }
    }
    zone.center /= std::max(1, zone.voxel_count);
    raw_zones.emplace_back(std::move(zone));
  }

  std::vector<int> remap(raw_zones.size(), -1);
  std::vector<Zone> zones;
  zones.reserve(raw_zones.size());
  for (int i = 0; i < static_cast<int>(raw_zones.size()); ++i) {
    if (raw_zones[i].state == CoverageVoxelState::UNKNOWN &&
        raw_zones[i].voxel_count < config_.min_unknown_voxels) {
      continue;
    }
    remap[i] = static_cast<int>(zones.size());
    zones.emplace_back(std::move(raw_zones[i]));
  }
  for (int &zone_id : zone_of) {
    zone_id = zone_id >= 0 ? remap[zone_id] : -1;
  }
  for (const Zone &zone : zones) {
    if (zone.state == CoverageVoxelState::KNOWN_FREE) {
      ++plan.free_zone_count;
    } else {
      ++plan.unknown_zone_count;
    }
  }

  std::vector<std::unordered_map<int, double>> edge_maps(zones.size());
  const Eigen::Vector3i positive_directions[3] = {
      Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(0, 1, 0),
      Eigen::Vector3i(0, 0, 1)};
  for (int linear = 0; linear < voxel_count; ++linear) {
    const int first = zone_of[linear];
    if (first < 0) {
      continue;
    }
    const Eigen::Vector3i index = indexFromLinear(linear);
    for (const Eigen::Vector3i &direction : positive_directions) {
      const Eigen::Vector3i next_index = index + direction;
      if (!map_spec_.contains(next_index)) {
        continue;
      }
      const int second = zone_of[map_spec_.flatten(next_index)];
      if (second < 0 || second == first) {
        continue;
      }
      double weight = std::max(map_spec_.resolution,
                               (zones[first].center - zones[second].center)
                                   .norm());
      if (zones[first].state == CoverageVoxelState::UNKNOWN ||
          zones[second].state == CoverageVoxelState::UNKNOWN) {
        weight *= std::max(1.0, config_.unknown_edge_penalty);
      }
      auto addEdge = [&](int from, int to) {
        const auto found = edge_maps[from].find(to);
        if (found == edge_maps[from].end() || weight < found->second) {
          edge_maps[from][to] = weight;
        }
      };
      addEdge(first, second);
      addEdge(second, first);
    }
  }
  for (int i = 0; i < static_cast<int>(zones.size()); ++i) {
    zones[i].edges.reserve(edge_maps[i].size());
    for (const auto &edge : edge_maps[i]) {
      zones[i].edges.emplace_back(edge.first, edge.second);
    }
  }

  int start_zone = -1;
  const Eigen::Vector3i robot_index =
      map_spec_.positionToIndex(work.robot_position);
  if (map_spec_.contains(robot_index)) {
    const int candidate = zone_of[map_spec_.flatten(robot_index)];
    if (candidate >= 0 &&
        zones[candidate].state == CoverageVoxelState::KNOWN_FREE) {
      start_zone = candidate;
    }
  }
  if (start_zone < 0) {
    double nearest = kInfinity;
    for (int i = 0; i < static_cast<int>(zones.size()); ++i) {
      if (zones[i].state != CoverageVoxelState::KNOWN_FREE) {
        continue;
      }
      const double distance = (zones[i].center - work.robot_position).norm();
      if (distance < nearest) {
        nearest = distance;
        start_zone = i;
      }
    }
  }
  if (start_zone < 0) {
    return plan;
  }
  const std::vector<double> start_distance =
      shortestDistances(zones, start_zone);

  std::unordered_map<int, std::vector<CoverageFrontier>> zone_frontiers;
  std::unordered_map<int, int> frontier_zone;
  for (const CoverageFrontier &frontier : work.frontiers) {
    if (frontier.cluster_id < 0 || !frontier.position.allFinite()) {
      continue;
    }
    int zone_id = -1;
    const Eigen::Vector3i index =
        map_spec_.positionToIndex(frontier.position);
    if (map_spec_.contains(index)) {
      const int candidate = zone_of[map_spec_.flatten(index)];
      if (candidate >= 0 &&
          zones[candidate].state == CoverageVoxelState::KNOWN_FREE &&
          std::isfinite(start_distance[candidate])) {
        zone_id = candidate;
      }
    }
    if (zone_id < 0) {
      double nearest = 1.75 * config_.fine_cell_size;
      for (int i = 0; i < static_cast<int>(zones.size()); ++i) {
        if (zones[i].state != CoverageVoxelState::KNOWN_FREE ||
            !std::isfinite(start_distance[i])) {
          continue;
        }
        const double distance = (zones[i].center - frontier.position).norm();
        if (distance < nearest) {
          nearest = distance;
          zone_id = i;
        }
      }
    }
    if (zone_id >= 0) {
      zone_frontiers[zone_id].push_back(frontier);
      frontier_zone[frontier.cluster_id] = zone_id;
    }
  }

  std::vector<CoverageTarget> targets;
  CoverageTarget current_target;
  current_target.type = CoverageTargetType::CURRENT;
  current_target.position = work.robot_position;
  current_target.zone_id = start_zone;
  current_target.voxel_count = 1;
  current_target.route_rank = 0;
  targets.emplace_back(std::move(current_target));
  std::unordered_map<int, int> active_target_by_zone;
  for (const auto &entry : zone_frontiers) {
    CoverageTarget target;
    target.type = CoverageTargetType::ACTIVE_FREE;
    target.stable_id =
        stableGroupId("active:" + std::to_string(entry.first));
    target.zone_id = entry.first;
    target.position = Eigen::Vector3d::Zero();
    for (const CoverageFrontier &frontier : entry.second) {
      target.position += frontier.position;
    }
    target.position /= std::max<std::size_t>(1, entry.second.size());
    target.voxel_count = zones[entry.first].voxel_count;
    active_target_by_zone[entry.first] = static_cast<int>(targets.size());
    targets.emplace_back(target);
  }
  plan.active_free_count = static_cast<int>(active_target_by_zone.size());

  auto collectUnknownGroups = [&](double macro_size) {
    std::unordered_map<std::string, UnknownGroup> groups;
    for (int zone_id = 0; zone_id < static_cast<int>(zones.size()); ++zone_id) {
      if (zones[zone_id].state != CoverageVoxelState::UNKNOWN ||
          !std::isfinite(start_distance[zone_id])) {
        continue;
      }
      const bool near = (zones[zone_id].center - work.robot_position).norm() <=
                        config_.near_expand_radius;
      const std::string key = groupKey(
          zones[zone_id].center, map_spec_,
          near ? config_.fine_cell_size : macro_size, near);
      UnknownGroup &group = groups[key];
      group.weighted_center +=
          zones[zone_id].center * zones[zone_id].voxel_count;
      group.voxel_count += zones[zone_id].voxel_count;
      group.zones.emplace_back(zone_id);
    }
    return groups;
  };

  double macro_size = config_.macro_cell_size;
  auto unknown_groups = collectUnknownGroups(macro_size);
  for (int attempt = 0;
       targets.size() + unknown_groups.size() >
               static_cast<std::size_t>(config_.max_cp_nodes) &&
           attempt < 4;
       ++attempt) {
    macro_size *= 1.5;
    unknown_groups = collectUnknownGroups(macro_size);
  }

  std::vector<CoverageTarget> unknown_targets;
  unknown_targets.reserve(unknown_groups.size());
  for (auto &entry : unknown_groups) {
    UnknownGroup &group = entry.second;
    if (group.voxel_count < config_.min_unknown_voxels) {
      continue;
    }
    const Eigen::Vector3d center =
        group.weighted_center / std::max(1, group.voxel_count);
    int representative = group.zones.front();
    for (int zone_id : group.zones) {
      if ((zones[zone_id].center - center).squaredNorm() <
          (zones[representative].center - center).squaredNorm()) {
        representative = zone_id;
      }
    }
    CoverageTarget target;
    target.type = CoverageTargetType::REACHABLE_UNKNOWN;
    target.stable_id = stableGroupId(entry.first);
    target.position = center;
    target.zone_id = representative;
    target.voxel_count = group.voxel_count;

    // Select the cheapest known-free zone touching this unknown component.
    // The unknown center remains the observation direction/visualization
    // target; only this adjacent free point is eligible for navigation.
    std::vector<std::pair<double, int>> free_candidates;
    for (const int unknown_zone : group.zones) {
      for (const auto &edge : zones[unknown_zone].edges) {
        const int neighbor = edge.first;
        if (neighbor < 0 || neighbor >= static_cast<int>(zones.size()) ||
            zones[neighbor].state != CoverageVoxelState::KNOWN_FREE ||
            !std::isfinite(start_distance[neighbor])) {
          continue;
        }
        const double score =
            start_distance[neighbor] +
            0.1 * (zones[neighbor].center - center).norm();
        free_candidates.emplace_back(score, neighbor);
      }
    }
    std::stable_sort(free_candidates.begin(), free_candidates.end());
    std::unordered_set<int> used_free_zones;
    for (const auto &candidate : free_candidates) {
      if (!used_free_zones.insert(candidate.second).second) {
        continue;
      }
      const Eigen::Vector3d approach = zones[candidate.second].center;
      bool spatial_duplicate = false;
      for (const Eigen::Vector3d &kept : target.approach_candidates) {
        if ((kept - approach).norm() < 0.5 * config_.fine_cell_size) {
          spatial_duplicate = true;
          break;
        }
      }
      if (!spatial_duplicate) {
        target.approach_candidates.emplace_back(approach);
      }
      if (target.approach_candidates.size() >= 4U) {
        break;
      }
    }
    if (!target.approach_candidates.empty()) {
      target.approach_position = target.approach_candidates.front();
      target.has_approach = true;
    }
    unknown_targets.emplace_back(target);
  }
  const int remaining_capacity =
      std::max(0, config_.max_cp_nodes - static_cast<int>(targets.size()));
  if (static_cast<int>(unknown_targets.size()) > remaining_capacity) {
    std::vector<CoverageTarget> near_targets;
    std::vector<CoverageTarget> far_targets;
    for (const CoverageTarget &target : unknown_targets) {
      ((target.position - work.robot_position).norm() <=
               config_.near_expand_radius
           ? near_targets
           : far_targets)
          .push_back(target);
    }
    auto rankTargets = [&](std::vector<CoverageTarget> &values) {
      std::stable_sort(values.begin(), values.end(),
                       [&](const CoverageTarget &first,
                           const CoverageTarget &second) {
                         const double first_score =
                             first.voxel_count /
                             (1.0 +
                              (first.position - work.robot_position).norm());
                         const double second_score =
                             second.voxel_count /
                             (1.0 +
                              (second.position - work.robot_position).norm());
                         return first_score > second_score;
                       });
    };
    rankTargets(near_targets);
    rankTargets(far_targets);

    // Keep the fine near field responsive while reserving a third of the
    // bounded CP for far macro cells. Without this quota a large open sensor
    // horizon can fill all nodes with nearby cells and silently reduce the
    // supposedly long-horizon planner back to a local frontier heuristic.
    const int far_quota = std::min(
        static_cast<int>(far_targets.size()), remaining_capacity / 3);
    int near_quota = std::min(static_cast<int>(near_targets.size()),
                              remaining_capacity - far_quota);
    int selected_far = far_quota;
    int unused = remaining_capacity - near_quota - selected_far;
    if (unused > 0) {
      const int extra_far =
          std::min(unused,
                   static_cast<int>(far_targets.size()) - selected_far);
      selected_far += extra_far;
      unused -= extra_far;
    }
    if (unused > 0) {
      near_quota +=
          std::min(unused,
                   static_cast<int>(near_targets.size()) - near_quota);
    }
    unknown_targets.clear();
    unknown_targets.insert(unknown_targets.end(), near_targets.begin(),
                           near_targets.begin() + near_quota);
    unknown_targets.insert(unknown_targets.end(), far_targets.begin(),
                           far_targets.begin() + selected_far);
  }
  plan.reachable_unknown_count =
      static_cast<int>(unknown_targets.size());
  targets.insert(targets.end(), unknown_targets.begin(), unknown_targets.end());

  const int target_count = static_cast<int>(targets.size());
  std::vector<std::vector<double>> target_cost(
      target_count, std::vector<double>(target_count, kInfinity));
  for (int i = 0; i < target_count; ++i) {
    const std::vector<double> distance =
        shortestDistances(zones, targets[i].zone_id);
    target_cost[i][i] = 0.0;
    for (int j = 0; j < target_count; ++j) {
      const double graph_distance = distance[targets[j].zone_id];
      target_cost[i][j] =
          std::isfinite(graph_distance)
              ? graph_distance
              : 500.0 + 5.0 *
                            (targets[i].position - targets[j].position).norm();
    }
  }
  if (plan.active_free_count > 0) {
    for (int j = 1; j < target_count; ++j) {
      if (targets[j].type == CoverageTargetType::REACHABLE_UNKNOWN) {
        target_cost[0][j] += config_.unknown_first_penalty;
      }
    }
  }

  std::vector<int> route;
  route.reserve(target_count);
  std::vector<bool> used(target_count, false);
  route.push_back(0);
  used[0] = true;
  while (static_cast<int>(route.size()) < target_count) {
    const int current = route.back();
    int best = -1;
    for (int next = 1; next < target_count; ++next) {
      if (!used[next] &&
          (best < 0 || target_cost[current][next] <
                           target_cost[current][best])) {
        best = next;
      }
    }
    if (best < 0) {
      break;
    }
    used[best] = true;
    route.push_back(best);
  }
  // A bounded deterministic 2-opt pass improves the long-horizon open route
  // without introducing another global-state LKH instance.
  for (int pass = 0; pass < 6; ++pass) {
    bool improved = false;
    for (int i = 1; i + 2 < static_cast<int>(route.size()); ++i) {
      for (int j = i + 1; j + 1 < static_cast<int>(route.size()); ++j) {
        const double before = target_cost[route[i - 1]][route[i]] +
                              target_cost[route[j]][route[j + 1]];
        const double after = target_cost[route[i - 1]][route[j]] +
                             target_cost[route[i]][route[j + 1]];
        if (after + 1.0e-6 < before) {
          std::reverse(route.begin() + i, route.begin() + j + 1);
          improved = true;
        }
      }
    }
    if (!improved) {
      break;
    }
  }

  // Normalize frontier priorities by their order among executable active
  // regions, not by the absolute route index.  The route also contains many
  // macro unknown targets; using that mixed index with active_free_count as
  // the denominator inflated a nominal [0, 1] coverage bias above 10 and made
  // it dominate travel time by hundreds of seconds.
  std::unordered_map<int, int> active_rank_by_zone;
  int active_ordinal = 0;
  for (int rank = 0; rank < static_cast<int>(route.size()); ++rank) {
    CoverageTarget target = targets[route[rank]];
    target.route_rank = rank;
    plan.ordered_targets.emplace_back(target);
    if (target.type == CoverageTargetType::ACTIVE_FREE) {
      active_rank_by_zone[target.zone_id] = active_ordinal++;
    }
  }

  int first_active_zone = -1;
  for (const CoverageTarget &target : plan.ordered_targets) {
    if (target.type == CoverageTargetType::ACTIVE_FREE) {
      first_active_zone = target.zone_id;
      break;
    }
  }
  std::vector<std::pair<double, int>> ranked_clusters;
  ranked_clusters.reserve(frontier_zone.size());
  for (const CoverageFrontier &frontier : work.frontiers) {
    const auto zone_found = frontier_zone.find(frontier.cluster_id);
    if (zone_found == frontier_zone.end()) {
      continue;
    }
    const int zone_id = zone_found->second;
    const int route_rank = active_rank_by_zone.count(zone_id)
                               ? active_rank_by_zone[zone_id]
                               : target_count;
    const double local_detour =
        (frontier.position - zones[zone_id].center).norm() /
        std::max(0.5, config_.fine_cell_size);
    const double gain_reward =
        0.08 * frontier.information_gain /
        (30.0 + std::max(0.0, frontier.information_gain));
    const double age_reward = 0.05 * std::min(1.0, frontier.wait_age / 20.0);
    const double debt_reward = 0.07 * std::min(1.0, frontier.pass_debt / 4.0);
    const double normalized_priority = std::clamp(
        static_cast<double>(route_rank) /
                std::max(1, plan.active_free_count) +
            0.15 * std::min(1.0, local_detour) - gain_reward - age_reward -
            debt_reward,
        0.0, 1.25);
    plan.cluster_priority[frontier.cluster_id] = normalized_priority;
    ranked_clusters.emplace_back(normalized_priority, frontier.cluster_id);
    if (zone_id == start_zone || zone_id == first_active_zone) {
      plan.preferred_cluster_ids.insert(frontier.cluster_id);
    }
  }
  std::stable_sort(ranked_clusters.begin(), ranked_clusters.end());
  for (const auto &ranked : ranked_clusters) {
    if (static_cast<int>(plan.preferred_cluster_ids.size()) >=
        config_.max_preferred_clusters) {
      break;
    }
    plan.preferred_cluster_ids.insert(ranked.second);
  }
  if (static_cast<int>(plan.preferred_cluster_ids.size()) >
      config_.max_preferred_clusters) {
    std::unordered_set<int> capped;
    for (const auto &ranked : ranked_clusters) {
      if (plan.preferred_cluster_ids.count(ranked.second)) {
        capped.insert(ranked.second);
      }
      if (static_cast<int>(capped.size()) >=
          config_.max_preferred_clusters) {
        break;
      }
    }
    plan.preferred_cluster_ids.swap(capped);
  }
  plan.valid = !plan.ordered_targets.empty();
  return plan;
}

void CoverageGuidanceManager::publishVisualization() const {
  if (!config_.visualization_enable || marker_pub_.getTopic().empty()) {
    return;
  }
  const CoveragePlan::Ptr plan = latestUsablePlan();
  if (!plan) {
    return;
  }
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.emplace_back(clear);

  visualization_msgs::Marker line;
  line.header.frame_id = "world";
  line.header.stamp = ros::Time::now();
  line.ns = "coverage_route";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.16;
  line.color.r = 0.15;
  line.color.g = 0.75;
  line.color.b = 1.0;
  line.color.a = 0.85;

  visualization_msgs::Marker free_points = line;
  free_points.ns = "coverage_active_free";
  free_points.id = 1;
  free_points.type = visualization_msgs::Marker::SPHERE_LIST;
  free_points.scale.x = free_points.scale.y = free_points.scale.z = 0.55;
  free_points.color.r = 0.1;
  free_points.color.g = 1.0;
  free_points.color.b = 0.25;
  visualization_msgs::Marker unknown_points = free_points;
  unknown_points.ns = "coverage_reachable_unknown";
  unknown_points.id = 2;
  unknown_points.color.r = 0.2;
  unknown_points.color.g = 0.45;
  unknown_points.color.b = 1.0;

  for (const CoverageTarget &target : plan->ordered_targets) {
    geometry_msgs::Point point;
    point.x = target.position.x();
    point.y = target.position.y();
    point.z = target.position.z();
    line.points.emplace_back(point);
    if (target.type == CoverageTargetType::ACTIVE_FREE) {
      free_points.points.emplace_back(point);
    } else if (target.type == CoverageTargetType::REACHABLE_UNKNOWN) {
      unknown_points.points.emplace_back(point);
    }
  }
  array.markers.emplace_back(line);
  array.markers.emplace_back(free_points);
  array.markers.emplace_back(unknown_points);
  marker_pub_.publish(array);
}

bool CoverageGuidanceManager::runDeterministicSelfTest(std::string *error) {
  CoverageGuidanceManager manager;
  manager.config_.mode = "full";
  manager.config_.fine_cell_size = 4.0;
  manager.config_.macro_cell_size = 8.0;
  manager.config_.near_expand_radius = 8.0;
  manager.config_.min_unknown_voxels = 1;
  manager.map_spec_.min = Eigen::Vector3d::Zero();
  manager.map_spec_.max = Eigen::Vector3d(12.0, 8.0, 1.0);
  manager.map_spec_.resolution = 1.0;
  manager.map_spec_.dims = Eigen::Vector3i(12, 8, 1);
  manager.persistent_map_.assign(manager.map_spec_.dims.prod(),
                                 CoverageVoxelState::UNKNOWN);
  WorkItem work;
  work.delta.version = 1;
  work.frontier_version = 1;
  work.robot_position = Eigen::Vector3d(0.5, 3.5, 0.5);
  for (int x = 0; x <= 6; ++x) {
    for (int y = 2; y <= 5; ++y) {
      const int linear = manager.map_spec_.flatten(Eigen::Vector3i(x, y, 0));
      work.delta.samples.push_back(
          {linear, CoverageVoxelState::KNOWN_FREE});
    }
  }
  CoverageFrontier frontier;
  frontier.cluster_id = 7;
  frontier.position = Eigen::Vector3d(5.5, 3.5, 0.5);
  frontier.information_gain = 20.0;
  work.frontiers.push_back(frontier);
  const CoveragePlan plan = manager.buildPlan(work);
  if (!plan.valid || plan.active_free_count != 1 ||
      plan.reachable_unknown_count <= 0 ||
      !plan.preferred_cluster_ids.count(7) ||
      !plan.cluster_priority.count(7)) {
    if (error) {
      std::ostringstream stream;
      stream << "valid=" << plan.valid << " active=" << plan.active_free_count
             << " unknown=" << plan.reachable_unknown_count
             << " preferred=" << plan.preferred_cluster_ids.size();
      *error = stream.str();
    }
    return false;
  }
  std::unordered_map<std::uint64_t, std::size_t> first_unknown_targets;
  bool found_actionable_unknown = false;
  for (const CoverageTarget &target : plan.ordered_targets) {
    if (target.type != CoverageTargetType::REACHABLE_UNKNOWN) {
      continue;
    }
    if (target.stable_id == 0 ||
        target.has_approach != !target.approach_candidates.empty()) {
      if (error) {
        *error = "reachable unknown has invalid identity/approach state";
      }
      return false;
    }
    found_actionable_unknown =
        found_actionable_unknown || target.has_approach;
    first_unknown_targets[target.stable_id] =
        target.approach_candidates.size();
  }
  if (!found_actionable_unknown) {
    if (error) {
      *error = "test map produced no executable unknown observation target";
    }
    return false;
  }
  WorkItem repeated_work = work;
  repeated_work.delta.version = 2;
  repeated_work.frontier_version = 2;
  const CoveragePlan repeated_plan = manager.buildPlan(repeated_work);
  for (const CoverageTarget &target : repeated_plan.ordered_targets) {
    if (target.type == CoverageTargetType::REACHABLE_UNKNOWN &&
        !first_unknown_targets.count(target.stable_id)) {
      if (error) {
        *error = "coverage target identity changed without a map change";
      }
      return false;
    }
  }

  // A complete occupied wall must disconnect unknown space behind it. This
  // guards against accidentally replacing graph reachability with Euclidean
  // distance in a later optimization.
  CoverageGuidanceManager blocked_manager;
  blocked_manager.config_ = manager.config_;
  blocked_manager.map_spec_ = manager.map_spec_;
  blocked_manager.persistent_map_.assign(
      blocked_manager.map_spec_.dims.prod(), CoverageVoxelState::UNKNOWN);
  WorkItem blocked_work;
  blocked_work.delta.version = 2;
  blocked_work.frontier_version = 2;
  blocked_work.robot_position = Eigen::Vector3d(0.5, 3.5, 0.5);
  for (int x = 0; x <= 4; ++x) {
    for (int y = 0; y < blocked_manager.map_spec_.dims.y(); ++y) {
      const int linear = blocked_manager.map_spec_.flatten(
          Eigen::Vector3i(x, y, 0));
      blocked_work.delta.samples.push_back(
          {linear, x == 4 ? CoverageVoxelState::OCCUPIED
                          : CoverageVoxelState::KNOWN_FREE});
    }
  }
  frontier.position = Eigen::Vector3d(2.5, 3.5, 0.5);
  blocked_work.frontiers.push_back(frontier);
  const CoveragePlan blocked_plan = blocked_manager.buildPlan(blocked_work);
  if (!blocked_plan.valid || blocked_plan.active_free_count != 1 ||
      blocked_plan.reachable_unknown_count != 0) {
    if (error) {
      std::ostringstream stream;
      stream << "disconnected case valid=" << blocked_plan.valid
             << " active=" << blocked_plan.active_free_count
             << " reachable_unknown="
             << blocked_plan.reachable_unknown_count;
      *error = stream.str();
    }
    return false;
  }
  return true;
}

}  // namespace fast_planner
