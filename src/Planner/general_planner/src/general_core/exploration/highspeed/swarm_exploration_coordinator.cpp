#include <general_core/exploration/highspeed/swarm_exploration_coordinator.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <queue>
#include <sstream>
#include <utility>

namespace fast_planner {
namespace {
double nowSec() { return ros::Time::now().toSec(); }

Eigen::Vector3d fromXYZ(const std::vector<double> &data, std::size_t offset) {
  return Eigen::Vector3d(data[offset], data[offset + 1], data[offset + 2]);
}
}  // namespace

std::size_t SwarmTaskKeyHash::operator()(const SwarmTaskKey &key) const {
  std::size_t seed = 0;
  const auto combine = [&seed](int value) {
    seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
  };
  combine(key.ix);
  combine(key.iy);
  combine(key.iz);
  combine(key.floor);
  return seed;
}

void SwarmExplorationCoordinator::init(ros::NodeHandle &nh) {
  nh.param("swarm_exploration/enabled", enabled_, false);
  if (!enabled_) {
    ROS_INFO("[swarm exploration] disabled; single-UAV path is unchanged");
    return;
  }

  nh.param("swarm_exploration/robot_id", robot_id_, 0);
  nh.param("swarm_exploration/team_size", team_size_, 2);
  int epoch = 1;
  nh.param("swarm_exploration/mission_epoch", epoch, 1);
  mission_epoch_ = static_cast<std::uint32_t>(std::max(1, epoch));
  std::vector<double> values;
  if (nh.getParam("swarm_exploration/task_origin", values) &&
      values.size() >= 3U) {
    task_origin_ = Eigen::Vector3d(values[0], values[1], values[2]);
  }
  values.clear();
  if (nh.getParam("swarm_exploration/task_size", values) &&
      values.size() >= 3U) {
    task_size_ = Eigen::Vector3d(std::max(0.5, values[0]),
                                std::max(0.5, values[1]),
                                std::max(0.5, values[2]));
  }
  nh.param("swarm_exploration/floor_height", floor_height_, 3.0);
  nh.param("swarm_exploration/peer_timeout", peer_timeout_, 1.5);
  nh.param("swarm_exploration/task_evidence_timeout",
           task_evidence_timeout_, 5.0);
  nh.param("swarm_exploration/task_evidence_publish_interval",
           task_evidence_publish_interval_, 1.0);
  nh.param("swarm_exploration/task_lease_duration", task_lease_duration_,
           15.0);
  nh.param("swarm_exploration/ownership_penalty", ownership_penalty_,
           2000.0);
  nh.param("swarm_exploration/peer_lease_penalty", peer_lease_penalty_,
           1.0e6);
  nh.param("swarm_exploration/assist_penalty", assist_penalty_, 4.0);
  nh.param("swarm_exploration/continuity_bonus", continuity_bonus_, 8.0);
  nh.param("swarm_exploration/load_balance_weight", load_balance_weight_,
           3.0);
  nh.param("swarm_exploration/anchor_spacing", anchor_spacing_, 2.5);
  nh.param("swarm_exploration/graph_handshake_radius",
           graph_handshake_radius_, 3.5);
  nh.param("swarm_exploration/graph_handshake_max", graph_handshake_max_, 4);
  nh.param("swarm_exploration/max_graph_nodes", max_graph_nodes_, 2500);
  nh.param("swarm_exploration/finish_quiet_period", finish_quiet_period_,
           3.0);
  nh.param("swarm_exploration/finish_hold_duration", finish_hold_duration_,
           3.0);
  nh.param("swarm_exploration/trajectory_horizon", trajectory_horizon_, 6.0);
  nh.param("swarm_exploration/trajectory_sample_dt", trajectory_sample_dt_,
           0.08);
  nh.param("swarm_exploration/horizontal_clearance",
           horizontal_clearance_, 1.2);
  nh.param("swarm_exploration/vertical_clearance", vertical_clearance_, 0.8);

  team_size_ = std::max(1, team_size_);
  graph_handshake_max_ = std::max(0, graph_handshake_max_);
  max_graph_nodes_ = std::max(100, max_graph_nodes_);
  last_registry_mutation_ = nowSec();

  std::string state_topic = "/swarm/exploration/robot_state";
  std::string task_topic = "/swarm/exploration/task";
  std::string graph_topic = "/swarm/exploration/graph_delta";
  std::string finish_topic = "/swarm/exploration/finish_vote";
  std::string trajectory_topic = "/swarm/exploration/trajectory";
  nh.param("swarm_exploration/topics/robot_state", state_topic, state_topic);
  nh.param("swarm_exploration/topics/task", task_topic, task_topic);
  nh.param("swarm_exploration/topics/graph_delta", graph_topic, graph_topic);
  nh.param("swarm_exploration/topics/finish_vote", finish_topic, finish_topic);
  nh.param("swarm_exploration/topics/trajectory", trajectory_topic,
           trajectory_topic);

  robot_state_pub_ = nh.advertise<std_msgs::Float64MultiArray>(state_topic, 50);
  task_pub_ = nh.advertise<std_msgs::Float64MultiArray>(task_topic, 200);
  graph_pub_ = nh.advertise<std_msgs::Float64MultiArray>(graph_topic, 200);
  finish_pub_ = nh.advertise<std_msgs::Float64MultiArray>(finish_topic, 50);
  trajectory_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>(trajectory_topic, 50);
  robot_state_sub_ = nh.subscribe(state_topic, 100,
      &SwarmExplorationCoordinator::robotStateCallback, this,
      ros::TransportHints().tcpNoDelay());
  task_sub_ = nh.subscribe(task_topic, 500,
      &SwarmExplorationCoordinator::taskCallback, this,
      ros::TransportHints().tcpNoDelay());
  graph_sub_ = nh.subscribe(graph_topic, 500,
      &SwarmExplorationCoordinator::graphCallback, this,
      ros::TransportHints().tcpNoDelay());
  finish_sub_ = nh.subscribe(finish_topic, 100,
      &SwarmExplorationCoordinator::finishCallback, this,
      ros::TransportHints().tcpNoDelay());
  trajectory_sub_ = nh.subscribe(trajectory_topic, 100,
      &SwarmExplorationCoordinator::trajectoryCallback, this,
      ros::TransportHints().tcpNoDelay());
  state_timer_ = nh.createTimer(ros::Duration(0.1),
      &SwarmExplorationCoordinator::stateTimerCallback, this);
  maintenance_timer_ = nh.createTimer(ros::Duration(0.5),
      &SwarmExplorationCoordinator::maintenanceTimerCallback, this);

  ROS_INFO_STREAM("[swarm exploration] enabled robot=" << robot_id_
                  << "/" << team_size_ << " epoch=" << mission_epoch_
                  << " task_size=" << task_size_.transpose()
                  << " anchor_spacing=" << anchor_spacing_);
}

SwarmTaskKey SwarmExplorationCoordinator::taskKey(
    const Eigen::Vector3d &position) const {
  const Eigen::Array3d relative =
      (position - task_origin_).array() / task_size_.array();
  SwarmTaskKey key;
  key.ix = static_cast<int>(std::floor(relative.x()));
  key.iy = static_cast<int>(std::floor(relative.y()));
  key.iz = static_cast<int>(std::floor(relative.z()));
  key.floor = static_cast<int>(std::floor(
      (position.z() - task_origin_.z()) / std::max(0.5, floor_height_)));
  return key;
}

std::uint64_t SwarmExplorationCoordinator::makeNodeId(
    std::uint64_t local_sequence) const {
  return (static_cast<std::uint64_t>(std::max(0, robot_id_) + 1) *
          1000000000ULL) + local_sequence;
}

bool SwarmExplorationCoordinator::peerAliveLocked(int robot_id,
                                                   double now) const {
  if (robot_id == robot_id_) {
    return have_self_state_;
  }
  const auto it = robots_.find(robot_id);
  return it != robots_.end() && now - it->second.last_seen <= peer_timeout_;
}

std::uint64_t SwarmExplorationCoordinator::nearestNodeLocked(
    const Eigen::Vector3d &position) const {
  std::uint64_t best_id = 0;
  double best = std::numeric_limits<double>::infinity();
  for (const auto &entry : graph_) {
    const double dist = (entry.second.position - position).squaredNorm();
    if (dist < best) {
      best = dist;
      best_id = entry.first;
    }
  }
  return best_id;
}

double SwarmExplorationCoordinator::graphDistanceLocked(
    std::uint64_t source, std::uint64_t target) const {
  if (source == 0 || target == 0 || graph_.find(source) == graph_.end() ||
      graph_.find(target) == graph_.end()) {
    return std::numeric_limits<double>::infinity();
  }
  if (source == target) {
    return 0.0;
  }
  using QueueItem = std::pair<double, std::uint64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>,
                      std::greater<QueueItem>> queue;
  std::unordered_map<std::uint64_t, double> distance;
  distance[source] = 0.0;
  queue.push({0.0, source});
  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    const auto known = distance.find(current.second);
    if (known == distance.end() || current.first > known->second + 1.0e-9) {
      continue;
    }
    if (current.second == target) {
      return current.first;
    }
    const auto node_it = graph_.find(current.second);
    if (node_it == graph_.end()) {
      continue;
    }
    for (const auto &edge : node_it->second.edges) {
      if (graph_.find(edge.first) == graph_.end()) {
        continue;
      }
      const double candidate = current.first + std::max(0.01, edge.second);
      const auto old = distance.find(edge.first);
      if (old == distance.end() || candidate < old->second) {
        distance[edge.first] = candidate;
        queue.push({candidate, edge.first});
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

int SwarmExplorationCoordinator::taskOwnerLocked(
    const Eigen::Vector3d &position, double now) const {
  const std::uint64_t task_anchor = nearestNodeLocked(position);
  int best_robot = robot_id_;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int id = 0; id < team_size_; ++id) {
    if (!peerAliveLocked(id, now)) {
      continue;
    }
    Eigen::Vector3d robot_position = self_position_;
    std::uint64_t robot_anchor = current_anchor_id_;
    if (id != robot_id_) {
      const auto peer = robots_.find(id);
      if (peer == robots_.end()) {
        continue;
      }
      robot_position = peer->second.position;
      robot_anchor = peer->second.anchor_id;
    }
    double cost = graphDistanceLocked(robot_anchor, task_anchor);
    if (!std::isfinite(cost)) {
      cost = (robot_position - position).norm();
    } else if (task_anchor != 0) {
      cost += (graph_.at(task_anchor).position - position).norm();
    }
    const auto state = robots_.find(id);
    if (state != robots_.end()) {
      // Lightweight RACER-style workload compensation.  The sparse graph
      // remains the primary distance metric; active claims break Voronoi ties
      // without introducing a continuous pairwise ACVRP solve.
      cost += load_balance_weight_ *
              static_cast<double>(state->second.active_claims);
    }
    if (cost < best_cost - 1.0e-6 ||
        (std::fabs(cost - best_cost) <= 1.0e-6 && id < best_robot)) {
      best_cost = cost;
      best_robot = id;
    }
  }
  return best_robot;
}

bool SwarmExplorationCoordinator::livePeerLeaseLocked(
    const SwarmTaskKey &key, double now, int *owner) const {
  const auto it = tasks_.find(key);
  if (it == tasks_.end()) {
    return false;
  }
  const TaskRecord &task = it->second;
  const bool active =
      task.owner_robot >= 0 && task.owner_robot != robot_id_ &&
      task.lease_until > now &&
      (task.state == TaskState::CLAIMED ||
       task.state == TaskState::EXPLORING);
  if (active && owner != nullptr) {
    *owner = task.owner_robot;
  }
  return active;
}

bool SwarmExplorationCoordinator::remoteClaimWinsLocked(
    const TaskRecord &remote, const TaskRecord &local) const {
  if (remote.estimated_cost < local.estimated_cost - 0.1) {
    return true;
  }
  if (local.estimated_cost < remote.estimated_cost - 0.1) {
    return false;
  }
  return remote.owner_robot < local.owner_robot;
}

std::vector<double> SwarmExplorationCoordinator::encodeGraphNodeLocked(
    const GraphNode &node, bool full_sync) const {
  std::vector<double> wire = {
      kWireVersion, static_cast<double>(mission_epoch_),
      static_cast<double>(robot_id_), static_cast<double>(local_graph_sequence_),
      static_cast<double>(node.id), static_cast<double>(node.floor),
      node.position.x(), node.position.y(), node.position.z(),
      full_sync ? 1.0 : 0.0, static_cast<double>(node.edges.size())};
  wire.reserve(wire.size() + 2U * node.edges.size());
  for (const auto &edge : node.edges) {
    wire.push_back(static_cast<double>(edge.first));
    wire.push_back(edge.second);
  }
  return wire;
}

void SwarmExplorationCoordinator::addHistoryAnchorLocked(
    const Eigen::Vector3d &position, std::vector<double> *wire) {
  if (static_cast<int>(graph_.size()) >= max_graph_nodes_) {
    ROS_WARN_THROTTLE(2.0, "[swarm graph] node cap reached: %d",
                      max_graph_nodes_);
    return;
  }
  GraphNode node;
  node.id = makeNodeId(++local_graph_sequence_);
  node.source_robot = robot_id_;
  node.position = position;
  node.floor = taskKey(position).floor;
  if (previous_anchor_id_ != 0 && graph_.find(previous_anchor_id_) != graph_.end()) {
    const double cost =
        std::max(0.01, (graph_[previous_anchor_id_].position - position).norm());
    node.edges[previous_anchor_id_] = cost;
  }

  std::vector<std::pair<double, std::uint64_t>> handshake;
  for (const auto &entry : graph_) {
    if (entry.second.source_robot == robot_id_) {
      continue;
    }
    const double distance = (entry.second.position - position).norm();
    if (distance <= graph_handshake_radius_) {
      handshake.push_back({distance, entry.first});
    }
  }
  std::sort(handshake.begin(), handshake.end());
  for (int i = 0; i < std::min(graph_handshake_max_,
                               static_cast<int>(handshake.size())); ++i) {
    node.edges[handshake[i].second] = std::max(0.01, handshake[i].first);
  }

  graph_[node.id] = node;
  for (const auto &edge : node.edges) {
    auto neighbor = graph_.find(edge.first);
    if (neighbor != graph_.end()) {
      neighbor->second.edges[node.id] = edge.second;
    }
  }
  previous_anchor_id_ = node.id;
  current_anchor_id_ = node.id;
  last_anchor_position_ = position;
  have_anchor_ = true;
  local_graph_nodes_.push_back(node.id);
  last_registry_mutation_ = nowSec();
  if (wire != nullptr) {
    *wire = encodeGraphNodeLocked(node, false);
  }
}

void SwarmExplorationCoordinator::updateRobotState(
    const Eigen::Vector3d &position, const Eigen::Vector3d &velocity) {
  if (!enabled_ || !position.allFinite() || !velocity.allFinite()) {
    return;
  }
  std::vector<double> graph_wire;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    self_position_ = position;
    self_velocity_ = velocity;
    have_self_state_ = true;
    if (!have_anchor_ ||
        (position - last_anchor_position_).norm() >= anchor_spacing_) {
      addHistoryAnchorLocked(position, &graph_wire);
    }
    RobotRecord &self = robots_[robot_id_];
    self.position = position;
    self.velocity = velocity;
    self.anchor_id = current_anchor_id_;
    self.local_converged = local_converged_;
    self.finish_candidate = finish_candidate_;
    self.task_revision = task_revision_;
    self.active_claims = have_current_task_ ? 1U : 0U;
    self.last_seen = nowSec();
  }
  if (!graph_wire.empty()) {
    std_msgs::Float64MultiArray msg;
    msg.data = std::move(graph_wire);
    graph_pub_.publish(msg);
  }
}

void SwarmExplorationCoordinator::publishTaskLocked(TaskRecord &task) {
  std_msgs::Float64MultiArray msg;
  msg.data = {
      kWireVersion, static_cast<double>(mission_epoch_),
      static_cast<double>(robot_id_), static_cast<double>(task.owner_robot),
      static_cast<double>(task.key.ix), static_cast<double>(task.key.iy),
      static_cast<double>(task.key.iz), static_cast<double>(task.key.floor),
      static_cast<double>(static_cast<int>(task.state)),
      task.coverage ? 1.0 : 0.0, task.position.x(), task.position.y(),
      task.position.z(), task.information_gain, task.estimated_cost,
      task.lease_until, static_cast<double>(task.revision)};
  task.last_publish = nowSec();
  task_pub_.publish(msg);
}

std::vector<double> SwarmExplorationCoordinator::candidatePenalties(
    const std::vector<SwarmCandidate> &candidates) {
  std::vector<double> penalties(candidates.size(), 0.0);
  if (!enabled_ || candidates.empty()) {
    return penalties;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = nowSec();
  pruneLocked(now);
  local_candidate_keys_.clear();
  std::vector<int> owners(candidates.size(), robot_id_);
  bool have_self_owned = false;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const SwarmTaskKey key = taskKey(candidates[i].position);
    local_candidate_keys_.insert(key);
    auto inserted = tasks_.emplace(key, TaskRecord());
    TaskRecord &task = inserted.first->second;
    const bool was_new = inserted.second;
    if (was_new) {
      task.key = key;
      task.source_robot = robot_id_;
      task.owner_robot = -1;
      task.state = TaskState::AVAILABLE;
      task.revision = 1;
      last_registry_mutation_ = now;
      ++task_revision_;
    } else if ((task.state == TaskState::COMPLETED ||
                task.state == TaskState::EXHAUSTED ||
                task.state == TaskState::UNREACHABLE) &&
               candidates[i].information_gain > 0.0) {
      task.state = TaskState::AVAILABLE;
      task.owner_robot = -1;
      task.lease_until = 0.0;
      ++task.revision;
      ++task_revision_;
      last_registry_mutation_ = now;
    }
    task.position = candidates[i].position;
    task.coverage = candidates[i].coverage;
    task.information_gain = candidates[i].information_gain;
    task.last_update = now;
    if (task.state == TaskState::AVAILABLE &&
        now - task.last_publish >= task_evidence_publish_interval_) {
      publishTaskLocked(task);
    }
    owners[i] = taskOwnerLocked(candidates[i].position, now);
    have_self_owned = have_self_owned || owners[i] == robot_id_;
  }

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const SwarmTaskKey key = taskKey(candidates[i].position);
    int lease_owner = -1;
    if (livePeerLeaseLocked(key, now, &lease_owner)) {
      penalties[i] += peer_lease_penalty_;
      continue;
    }
    if (have_current_task_ && key == current_task_key_) {
      penalties[i] -= continuity_bonus_;
    }
    if (owners[i] == robot_id_) {
      continue;
    }
    const auto peer = robots_.find(owners[i]);
    const bool owner_converged =
        peer != robots_.end() && peer->second.local_converged;
    penalties[i] += (have_self_owned && !owner_converged)
                        ? ownership_penalty_
                        : assist_penalty_;
  }
  return penalties;
}

void SwarmExplorationCoordinator::claimTask(
    const SwarmCandidate &candidate) {
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = nowSec();
  const SwarmTaskKey key = taskKey(candidate.position);
  auto inserted = tasks_.emplace(key, TaskRecord());
  TaskRecord &task = inserted.first->second;
  task.key = key;
  task.position = candidate.position;
  task.source_robot = robot_id_;
  task.owner_robot = robot_id_;
  task.coverage = candidate.coverage;
  task.information_gain = candidate.information_gain;
  task.estimated_cost = candidate.travel_cost;
  task.state = TaskState::EXPLORING;
  task.lease_until = now + task_lease_duration_;
  task.last_update = now;
  ++task.revision;
  ++task_revision_;
  have_current_task_ = true;
  current_task_key_ = key;
  local_converged_ = false;
  finish_candidate_ = false;
  finish_candidate_since_ = 0.0;
  last_registry_mutation_ = now;
  publishTaskLocked(task);
}

void SwarmExplorationCoordinator::completeTaskAt(
    const Eigen::Vector3d &position) {
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  SwarmTaskKey key = taskKey(position);
  if (have_current_task_) {
    key = current_task_key_;
  }
  auto it = tasks_.find(key);
  if (it != tasks_.end()) {
    TaskRecord &task = it->second;
    task.state = TaskState::COMPLETED;
    task.owner_robot = robot_id_;
    task.lease_until = 0.0;
    task.last_update = nowSec();
    ++task.revision;
    ++task_revision_;
    last_registry_mutation_ = task.last_update;
    publishTaskLocked(task);
  }
  have_current_task_ = false;
}

void SwarmExplorationCoordinator::releaseCurrentTask(
    const std::string &reason) {
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_current_task_) {
    return;
  }
  auto it = tasks_.find(current_task_key_);
  if (it != tasks_.end()) {
    TaskRecord &task = it->second;
    task.state = TaskState::AVAILABLE;
    task.owner_robot = -1;
    task.lease_until = 0.0;
    task.last_update = nowSec();
    ++task.revision;
    ++task_revision_;
    last_registry_mutation_ = task.last_update;
    publishTaskLocked(task);
  }
  have_current_task_ = false;
  ROS_WARN_STREAM("[swarm task] release current task: " << reason);
}

void SwarmExplorationCoordinator::setLocalConverged(
    bool converged, bool frontier_audit_ready, bool coverage_plateau,
    bool targets_exhausted) {
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (local_converged_ != converged) {
    last_registry_mutation_ = nowSec();
  }
  local_converged_ = converged;
  frontier_audit_ready_ = converged && frontier_audit_ready;
  coverage_plateau_ = converged && coverage_plateau;
  targets_exhausted_ = converged && targets_exhausted;
  if (!converged) {
    finish_candidate_ = false;
    finish_candidate_since_ = 0.0;
  } else {
    // The local finish gate has proved that its prior execution shortlist is
    // empty.  A stale candidate identity must not immediately wake it again.
    local_candidate_keys_.clear();
  }
  RobotRecord &self = robots_[robot_id_];
  self.local_converged = local_converged_;
  self.last_seen = nowSec();
}

std::uint32_t SwarmExplorationCoordinator::activeTaskCountLocked(
    double now) const {
  std::uint32_t count = 0;
  for (const auto &entry : tasks_) {
    const TaskRecord &task = entry.second;
    if (task.state == TaskState::AVAILABLE &&
        now - task.last_update <= task_evidence_timeout_) {
      ++count;
    } else if ((task.state == TaskState::CLAIMED ||
                task.state == TaskState::EXPLORING) &&
               task.lease_until > now) {
      ++count;
    }
  }
  return count;
}

bool SwarmExplorationCoordinator::allMembersConvergedLocked(double now) const {
  for (int id = 0; id < team_size_; ++id) {
    const auto peer = robots_.find(id);
    if (peer == robots_.end() || now - peer->second.last_seen > peer_timeout_ ||
        !peer->second.local_converged) {
      return false;
    }
  }
  return true;
}

bool SwarmExplorationCoordinator::teamFinishReady() {
  if (!enabled_) {
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = nowSec();
  pruneLocked(now);
  const bool ready = local_converged_ && frontier_audit_ready_ &&
                     coverage_plateau_ && targets_exhausted_ &&
                     allMembersConvergedLocked(now) &&
                     activeTaskCountLocked(now) == 0U &&
                     now - last_registry_mutation_ >= finish_quiet_period_;
  if (!ready) {
    finish_candidate_ = false;
    finish_candidate_since_ = 0.0;
    return false;
  }
  if (!finish_candidate_) {
    finish_candidate_ = true;
    finish_candidate_since_ = now;
    return false;
  }
  return now - finish_candidate_since_ >= finish_hold_duration_;
}

bool SwarmExplorationCoordinator::teamFinishCandidate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return finish_candidate_;
}

bool SwarmExplorationCoordinator::hasLocallyActionableTask() const {
  if (!enabled_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = nowSec();
  for (const SwarmTaskKey &key : local_candidate_keys_) {
    int owner = -1;
    if (!livePeerLeaseLocked(key, now, &owner)) {
      return true;
    }
  }
  return false;
}

std::string SwarmExplorationCoordinator::statusString() const {
  if (!enabled_) {
    return "disabled";
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = nowSec();
  std::ostringstream stream;
  stream << "robot=" << robot_id_ << "/" << team_size_
         << " peers=";
  int peers = 0;
  for (const auto &entry : robots_) {
    if (now - entry.second.last_seen <= peer_timeout_) {
      ++peers;
    }
  }
  stream << peers << " tasks=" << activeTaskCountLocked(now)
         << " graph=" << graph_.size()
         << " local_converged=" << local_converged_
         << " finish_candidate=" << finish_candidate_;
  return stream.str();
}

void SwarmExplorationCoordinator::pruneLocked(double now) {
  bool changed = false;
  for (auto it = tasks_.begin(); it != tasks_.end();) {
    const bool stale_available =
        it->second.state == TaskState::AVAILABLE &&
        now - it->second.last_update > task_evidence_timeout_;
    const bool stale_terminal =
        (it->second.state == TaskState::COMPLETED ||
         it->second.state == TaskState::EXHAUSTED ||
         it->second.state == TaskState::UNREACHABLE) &&
        now - it->second.last_update > 10.0 * task_evidence_timeout_;
    if (stale_available || stale_terminal) {
      it = tasks_.erase(it);
      changed = true;
      continue;
    }
    if ((it->second.state == TaskState::CLAIMED ||
         it->second.state == TaskState::EXPLORING) &&
        it->second.lease_until <= now) {
      it->second.state = TaskState::AVAILABLE;
      it->second.owner_robot = -1;
      it->second.last_update = now;
      ++it->second.revision;
      changed = true;
    }
    ++it;
  }
  for (auto it = remote_trajectories_.begin();
       it != remote_trajectories_.end();) {
    if (it->second.valid_until < now - peer_timeout_) {
      it = remote_trajectories_.erase(it);
    } else {
      ++it;
    }
  }
  if (changed) {
    ++task_revision_;
    last_registry_mutation_ = now;
  }
}

void SwarmExplorationCoordinator::stateTimerCallback(const ros::TimerEvent &) {
  if (!enabled_) {
    return;
  }
  std_msgs::Float64MultiArray state;
  std_msgs::Float64MultiArray finish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = nowSec();
    RobotRecord &self = robots_[robot_id_];
    self.position = self_position_;
    self.velocity = self_velocity_;
    self.anchor_id = current_anchor_id_;
    self.local_converged = local_converged_;
    self.finish_candidate = finish_candidate_;
    self.task_revision = task_revision_;
    self.active_claims = have_current_task_ ? 1U : 0U;
    self.last_seen = now;
    state.data = {
        kWireVersion, static_cast<double>(mission_epoch_),
        static_cast<double>(robot_id_), static_cast<double>(team_size_),
        self_position_.x(), self_position_.y(), self_position_.z(),
        self_velocity_.x(), self_velocity_.y(), self_velocity_.z(),
        static_cast<double>(current_anchor_id_), local_converged_ ? 1.0 : 0.0,
        finish_candidate_ ? 1.0 : 0.0, static_cast<double>(task_revision_),
        have_current_task_ ? 1.0 : 0.0};
    finish.data = {
        kWireVersion, static_cast<double>(mission_epoch_),
        static_cast<double>(robot_id_), local_converged_ ? 1.0 : 0.0,
        frontier_audit_ready_ ? 1.0 : 0.0,
        coverage_plateau_ ? 1.0 : 0.0,
        targets_exhausted_ ? 1.0 : 0.0,
        static_cast<double>(task_revision_),
        static_cast<double>(activeTaskCountLocked(now))};
  }
  robot_state_pub_.publish(state);
  finish_pub_.publish(finish);
}

void SwarmExplorationCoordinator::maintenanceTimerCallback(
    const ros::TimerEvent &) {
  if (!enabled_) {
    return;
  }
  std::vector<double> graph_wire;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = nowSec();
    pruneLocked(now);
    if (have_current_task_) {
      auto current = tasks_.find(current_task_key_);
      if (current != tasks_.end() &&
          current->second.lease_until - now < 0.5 * task_lease_duration_) {
        current->second.lease_until = now + task_lease_duration_;
        current->second.last_update = now;
        ++current->second.revision;
        ++task_revision_;
        publishTaskLocked(current->second);
      }
    }
    if (!local_graph_nodes_.empty()) {
      if (full_sync_cursor_ >= local_graph_nodes_.size()) {
        full_sync_cursor_ = 0;
      }
      const auto node = graph_.find(local_graph_nodes_[full_sync_cursor_++]);
      if (node != graph_.end()) {
        graph_wire = encodeGraphNodeLocked(node->second, true);
      }
    }
  }
  if (!graph_wire.empty()) {
    std_msgs::Float64MultiArray msg;
    msg.data = std::move(graph_wire);
    graph_pub_.publish(msg);
  }
}

void SwarmExplorationCoordinator::robotStateCallback(
    const std_msgs::Float64MultiArrayConstPtr &msg) {
  if (!enabled_ || !msg || msg->data.size() < 15U ||
      msg->data[0] != kWireVersion ||
      static_cast<std::uint32_t>(msg->data[1]) != mission_epoch_) {
    return;
  }
  const int id = static_cast<int>(msg->data[2]);
  if (id == robot_id_ || id < 0 || id >= team_size_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  RobotRecord &peer = robots_[id];
  peer.position = fromXYZ(msg->data, 4);
  peer.velocity = fromXYZ(msg->data, 7);
  peer.anchor_id = static_cast<std::uint64_t>(msg->data[10]);
  peer.local_converged = msg->data[11] > 0.5;
  peer.finish_candidate = msg->data[12] > 0.5;
  peer.task_revision = static_cast<std::uint64_t>(msg->data[13]);
  peer.active_claims = static_cast<std::uint32_t>(msg->data[14]);
  peer.last_seen = nowSec();
}

void SwarmExplorationCoordinator::taskCallback(
    const std_msgs::Float64MultiArrayConstPtr &msg) {
  if (!enabled_ || !msg || msg->data.size() < 17U ||
      msg->data[0] != kWireVersion ||
      static_cast<std::uint32_t>(msg->data[1]) != mission_epoch_) {
    return;
  }
  const int source = static_cast<int>(msg->data[2]);
  if (source == robot_id_) {
    return;
  }
  TaskRecord remote;
  remote.source_robot = source;
  remote.owner_robot = static_cast<int>(msg->data[3]);
  remote.key.ix = static_cast<int>(msg->data[4]);
  remote.key.iy = static_cast<int>(msg->data[5]);
  remote.key.iz = static_cast<int>(msg->data[6]);
  remote.key.floor = static_cast<int>(msg->data[7]);
  remote.state = static_cast<TaskState>(static_cast<int>(msg->data[8]));
  remote.coverage = msg->data[9] > 0.5;
  remote.position = fromXYZ(msg->data, 10);
  remote.information_gain = msg->data[13];
  remote.estimated_cost = msg->data[14];
  remote.lease_until = msg->data[15];
  remote.revision = static_cast<std::uint32_t>(msg->data[16]);
  remote.last_update = nowSec();

  std::lock_guard<std::mutex> lock(mutex_);
  auto local_it = tasks_.find(remote.key);
  bool accept = local_it == tasks_.end();
  if (!accept) {
    const TaskRecord &local = local_it->second;
    const bool remote_terminal =
        remote.state == TaskState::COMPLETED ||
        remote.state == TaskState::EXHAUSTED;
    const bool remote_claim =
        remote.state == TaskState::CLAIMED ||
        remote.state == TaskState::EXPLORING;
    const bool local_claim =
        local.state == TaskState::CLAIMED ||
        local.state == TaskState::EXPLORING;
    if (remote_terminal) {
      accept = true;
    } else if (remote_claim && local_claim) {
      accept = remoteClaimWinsLocked(remote, local);
    } else if (remote_claim) {
      accept = true;
    } else if (!local_claim && remote.revision >= local.revision) {
      accept = true;
    }
  }
  if (!accept) {
    return;
  }
  if (have_current_task_ && remote.key == current_task_key_ &&
      remote.owner_robot != robot_id_ &&
      (remote.state == TaskState::CLAIMED ||
       remote.state == TaskState::EXPLORING)) {
    have_current_task_ = false;
  }
  tasks_[remote.key] = remote;
  ++task_revision_;
  last_registry_mutation_ = remote.last_update;
}

void SwarmExplorationCoordinator::graphCallback(
    const std_msgs::Float64MultiArrayConstPtr &msg) {
  if (!enabled_ || !msg || msg->data.size() < 11U ||
      msg->data[0] != kWireVersion ||
      static_cast<std::uint32_t>(msg->data[1]) != mission_epoch_) {
    return;
  }
  const int source = static_cast<int>(msg->data[2]);
  if (source == robot_id_) {
    return;
  }
  const std::size_t edge_count =
      static_cast<std::size_t>(std::max(0.0, msg->data[10]));
  if (msg->data.size() < 11U + 2U * edge_count) {
    return;
  }
  GraphNode node;
  node.source_robot = source;
  node.id = static_cast<std::uint64_t>(msg->data[4]);
  node.floor = static_cast<int>(msg->data[5]);
  node.position = fromXYZ(msg->data, 6);
  for (std::size_t i = 0; i < edge_count; ++i) {
    const std::uint64_t neighbor =
        static_cast<std::uint64_t>(msg->data[11U + 2U * i]);
    const double cost = msg->data[12U + 2U * i];
    node.edges[neighbor] = std::max(0.01, cost);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (graph_.find(node.id) == graph_.end() &&
      static_cast<int>(graph_.size()) >= max_graph_nodes_) {
    return;
  }
  GraphNode &stored = graph_[node.id];
  stored.id = node.id;
  stored.source_robot = node.source_robot;
  stored.floor = node.floor;
  stored.position = node.position;
  for (const auto &edge : node.edges) {
    stored.edges[edge.first] = edge.second;
    auto neighbor = graph_.find(edge.first);
    if (neighbor != graph_.end()) {
      neighbor->second.edges[node.id] = edge.second;
    }
  }
  // A handshake can be discovered after both robots have already created
  // their first anchors.  Derive the cross-robot edge deterministically on
  // every replica when the remote node arrives; otherwise two stationary
  // robots that start together would keep disjoint graph components forever.
  std::vector<std::pair<double, std::uint64_t>> local_handshakes;
  for (const auto &entry : graph_) {
    if (entry.first == node.id || entry.second.source_robot != robot_id_) {
      continue;
    }
    const double distance = (entry.second.position - node.position).norm();
    if (distance <= graph_handshake_radius_) {
      local_handshakes.push_back({distance, entry.first});
    }
  }
  std::sort(local_handshakes.begin(), local_handshakes.end());
  for (int i = 0; i < std::min(graph_handshake_max_,
                               static_cast<int>(local_handshakes.size())); ++i) {
    const double cost = std::max(0.01, local_handshakes[i].first);
    stored.edges[local_handshakes[i].second] = cost;
    graph_[local_handshakes[i].second].edges[node.id] = cost;
  }
}

void SwarmExplorationCoordinator::finishCallback(
    const std_msgs::Float64MultiArrayConstPtr &msg) {
  if (!enabled_ || !msg || msg->data.size() < 9U ||
      msg->data[0] != kWireVersion ||
      static_cast<std::uint32_t>(msg->data[1]) != mission_epoch_) {
    return;
  }
  const int id = static_cast<int>(msg->data[2]);
  if (id == robot_id_ || id < 0 || id >= team_size_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  RobotRecord &peer = robots_[id];
  peer.local_converged = msg->data[3] > 0.5 && msg->data[4] > 0.5 &&
                         msg->data[5] > 0.5 && msg->data[6] > 0.5;
  peer.task_revision = static_cast<std::uint64_t>(msg->data[7]);
  peer.active_claims = static_cast<std::uint32_t>(msg->data[8]);
  peer.last_seen = nowSec();
}

bool SwarmExplorationCoordinator::decodeTrajectory(
    const std_msgs::Float64MultiArray &msg, RemoteTrajectory &out) const {
  if (msg.data.size() < 12U || msg.data[0] != kWireVersion ||
      static_cast<std::uint32_t>(msg.data[1]) != mission_epoch_) {
    return false;
  }
  out.robot_id = static_cast<int>(msg.data[2]);
  out.priority = static_cast<int>(msg.data[3]);
  out.horizontal_clearance = std::max(0.1, msg.data[4]);
  out.vertical_clearance = std::max(0.1, msg.data[5]);
  out.valid_until = msg.data[6];
  out.start_time = msg.data[7];
  const int order = static_cast<int>(msg.data[9]);
  const std::size_t duration_count =
      static_cast<std::size_t>(std::max(0.0, msg.data[10]));
  const std::size_t coef_count =
      static_cast<std::size_t>(std::max(0.0, msg.data[11]));
  const std::size_t expected = 12U + duration_count + 3U * coef_count;
  if (order < 1 || duration_count == 0U ||
      coef_count != duration_count * static_cast<std::size_t>(order + 1) ||
      msg.data.size() != expected) {
    return false;
  }
  out.trajectory.clear();
  const std::size_t duration_offset = 12U;
  const std::size_t x_offset = duration_offset + duration_count;
  const std::size_t y_offset = x_offset + coef_count;
  const std::size_t z_offset = y_offset + coef_count;
  for (std::size_t piece = 0; piece < duration_count; ++piece) {
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, order + 1);
    for (int j = 0; j <= order; ++j) {
      const std::size_t index = piece * static_cast<std::size_t>(order + 1) +
                                static_cast<std::size_t>(j);
      coefficients(0, j) = msg.data[x_offset + index];
      coefficients(1, j) = msg.data[y_offset + index];
      coefficients(2, j) = msg.data[z_offset + index];
    }
    out.trajectory.emplace_back(
        std::max(1.0e-4, msg.data[duration_offset + piece]), coefficients);
  }
  return !out.trajectory.empty();
}

std_msgs::Float64MultiArray SwarmExplorationCoordinator::encodeTrajectory(
    const traj_utils::PolyTraj &trajectory) const {
  std_msgs::Float64MultiArray msg;
  const std::size_t duration_count = trajectory.duration.size();
  const std::size_t coef_count = trajectory.coef_x.size();
  msg.data.reserve(12U + duration_count + 3U * coef_count);
  msg.data = {kWireVersion, static_cast<double>(mission_epoch_),
              static_cast<double>(robot_id_), static_cast<double>(robot_id_),
              horizontal_clearance_, vertical_clearance_,
              trajectory.start_time.toSec() +
                  std::min(trajectory_horizon_,
                           std::accumulate(trajectory.duration.begin(),
                                           trajectory.duration.end(), 0.0)),
              trajectory.start_time.toSec(),
              static_cast<double>(trajectory.traj_id),
              static_cast<double>(trajectory.order),
              static_cast<double>(duration_count),
              static_cast<double>(coef_count)};
  msg.data.insert(msg.data.end(), trajectory.duration.begin(),
                  trajectory.duration.end());
  msg.data.insert(msg.data.end(), trajectory.coef_x.begin(),
                  trajectory.coef_x.end());
  msg.data.insert(msg.data.end(), trajectory.coef_y.begin(),
                  trajectory.coef_y.end());
  msg.data.insert(msg.data.end(), trajectory.coef_z.begin(),
                  trajectory.coef_z.end());
  return msg;
}

bool SwarmExplorationCoordinator::trajectoryConflictLocked(
    const RemoteTrajectory &first, const RemoteTrajectory &second) const {
  if (first.trajectory.empty() || second.trajectory.empty()) {
    return false;
  }
  const double begin = std::max(first.start_time, second.start_time);
  const double end = std::min({first.valid_until, second.valid_until,
                               begin + trajectory_horizon_});
  if (end <= begin) {
    return false;
  }
  const double horizontal =
      std::max(first.horizontal_clearance, second.horizontal_clearance);
  const double vertical =
      std::max(first.vertical_clearance, second.vertical_clearance);
  for (double stamp = begin; stamp <= end;
       stamp += std::max(0.02, trajectory_sample_dt_)) {
    const double first_t = std::clamp(stamp - first.start_time, 0.0,
                                     first.trajectory.getTotalDuration());
    const double second_t = std::clamp(stamp - second.start_time, 0.0,
                                      second.trajectory.getTotalDuration());
    const Eigen::Vector3d difference =
        first.trajectory.getPos(first_t) - second.trajectory.getPos(second_t);
    const double ellipsoid =
        difference.head<2>().squaredNorm() / (horizontal * horizontal) +
        difference.z() * difference.z() / (vertical * vertical);
    if (ellipsoid < 1.0) {
      return true;
    }
  }
  return false;
}

bool SwarmExplorationCoordinator::validateAndCommitTrajectory(
    const traj_utils::PolyTraj &trajectory, int *conflicting_robot) {
  if (!enabled_) {
    return true;
  }
  const std_msgs::Float64MultiArray wire = encodeTrajectory(trajectory);
  RemoteTrajectory local;
  if (!decodeTrajectory(wire, local)) {
    ROS_ERROR("[swarm trajectory] refuse malformed local trajectory");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = nowSec();
    for (const auto &entry : remote_trajectories_) {
      const RemoteTrajectory &remote = entry.second;
      if (remote.valid_until < now ||
          !trajectoryConflictLocked(local, remote)) {
        continue;
      }
      // Lower numeric priority (and then lower robot id) keeps its command.
      if (remote.priority < local.priority ||
          (remote.priority == local.priority &&
           remote.robot_id < local.robot_id)) {
        if (conflicting_robot != nullptr) {
          *conflicting_robot = remote.robot_id;
        }
        return false;
      }
    }
    local_trajectory_ = local;
    have_local_trajectory_ = true;
  }
  trajectory_pub_.publish(wire);
  return true;
}

void SwarmExplorationCoordinator::trajectoryCallback(
    const std_msgs::Float64MultiArrayConstPtr &msg) {
  if (!enabled_ || !msg) {
    return;
  }
  RemoteTrajectory remote;
  if (!decodeTrajectory(*msg, remote) || remote.robot_id == robot_id_ ||
      remote.robot_id < 0 || remote.robot_id >= team_size_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  remote_trajectories_[remote.robot_id] = remote;
  if (have_local_trajectory_ &&
      trajectoryConflictLocked(local_trajectory_, remote) &&
      (remote.priority < local_trajectory_.priority ||
       (remote.priority == local_trajectory_.priority &&
        remote.robot_id < robot_id_))) {
    yield_required_ = true;
    yield_priority_robot_ = remote.robot_id;
  }
}

bool SwarmExplorationCoordinator::consumeYieldRequest(int *priority_robot) {
  if (!enabled_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!yield_required_) {
    return false;
  }
  if (priority_robot != nullptr) {
    *priority_robot = yield_priority_robot_;
  }
  yield_required_ = false;
  yield_priority_robot_ = -1;
  return true;
}

}  // namespace fast_planner
