#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/expl_data.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fast_planner {

// Compatibility scoring is deliberately separate from tour-led exploration.
// Only soft/full guidance and multi-UAV allocation use these node rewards.
vector<double> FastExplorationManager::scoreExplorationCandidates(
    const vector<TopoNode::Ptr> &viewpoint_reachable,
    const vector<double> &viewpoint_reachable_distance,
    const vector<EdgeSafetyCost> &viewpoint_reachable_edges,
    const Eigen::MatrixXd &mat, bool priority_floor_active,
    bool ascending_to_priority_floor, int first_priority_floor_rank) {
  const ros::Time coverage_now = ros::Time::now();
  std::vector<double> swarm_candidate_penalties(viewpoint_reachable.size(),
                                                 0.0);
  if (swarm_coordinator_ && swarm_coordinator_->enabled()) {
    std::vector<SwarmCandidate> swarm_candidates;
    swarm_candidates.reserve(viewpoint_reachable.size());
    for (std::size_t i = 0; i < viewpoint_reachable.size(); ++i) {
      SwarmCandidate candidate;
      candidate.position = viewpoint_reachable[i]->center_.cast<double>();
      candidate.information_gain =
          viewpoint_reachable[i]->is_coverage_target_
              ? static_cast<double>(
                    std::max(0, viewpoint_reachable[i]->coverage_voxel_count_))
              : viewpoint_reachable[i]->frontier_information_gain_;
      candidate.travel_cost = viewpoint_reachable_distance[i];
      candidate.coverage = viewpoint_reachable[i]->is_coverage_target_;
      swarm_candidates.emplace_back(candidate);
    }
    swarm_candidate_penalties =
        swarm_coordinator_->candidatePenalties(swarm_candidates);
  }

  const auto &motion = ep_->coverage_motion_;
  const bool region_hold = coverageMotionEnabled() &&
      !coverage_region_since_.isZero() &&
      (coverage_now - coverage_region_since_).toSec() < motion.region_duration &&
      std::any_of(viewpoint_reachable.begin(), viewpoint_reachable.end(),
          [&](const TopoNode::Ptr &candidate) {
            return !candidate->is_coverage_target_ &&
                coverage_motion::sameRegion(candidate->center_.cast<double>(),
                                            coverage_region_anchor_, motion);
          });
  struct CandidateCostBreakdown {
    double travel{0.0};
    double turn_brake{0.0};
    double future_return{0.0};
    double gain_norm{0.0};
    double wait_norm{0.0};
    double debt_norm{0.0};
    double coverage{0.0};
    double failed_goal{0.0};
    double swarm{0.0};
    double region{0.0};
    double total{0.0};
  };
  vector<CandidateCostBreakdown> candidate_terms(viewpoint_reachable.size());
  auto fillStaticTerms = [&](const int i) {
    CandidateCostBreakdown &terms = candidate_terms[i];
    const auto &edge = viewpoint_reachable_edges[i];
    const auto &viewpoint = viewpoint_reachable[i];
    terms.travel = edge.time_cost;
    if (coverageMotionEnabled() && std::isfinite(edge.moving_time_cost))
      terms.travel = edge.moving_time_cost;
    if (region_hold && !viewpoint->is_coverage_target_ &&
        !coverage_motion::sameRegion(viewpoint->center_.cast<double>(),
                                     coverage_region_anchor_, motion)) {
      const double age = std::max(0.0, (coverage_now - coverage_region_since_).toSec());
      terms.region = motion.region_switch_cost *
          (1.0 - age / std::max(0.01, motion.region_duration));
    }
    terms.turn_brake = edge.turn_penalty + edge.yaw_penalty +
                       edge.known_free_penalty + edge.backup_penalty;
    if (viewpoint->is_mission_goal_target_) {
      terms.gain_norm = 0.0;
      terms.wait_norm = 0.0;
      terms.debt_norm = 0.0;
      terms.coverage = 0.0;
    } else if (viewpoint->is_coverage_target_) {
      const double gain =
          static_cast<double>(std::max(0, viewpoint->coverage_voxel_count_));
      terms.gain_norm = gain / (60.0 + gain);
      terms.wait_norm = 0.0;
      terms.debt_norm = 0.0;
      const double rank_bonus =
          viewpoint->coverage_route_rank_ >= 0
              ? 1.0 / (1.0 + viewpoint->coverage_route_rank_)
              : 0.0;
      const double bounded_rank = std::min(
          40.0, static_cast<double>(
                    std::max(0, viewpoint->coverage_route_rank_)));
      terms.coverage = -coverage_executable_candidate_bonus_ - rank_bonus +
                       coverage_route_rank_weight_ * bounded_rank;
      if (priority_floor_active &&
          (!ascending_to_priority_floor ||
           (viewpoint->coverage_route_rank_ >=
                std::max(0, first_priority_floor_rank -
                                coverage_floor_transition_rank_window_) &&
            viewpoint->coverage_route_rank_ <= first_priority_floor_rank))) {
        terms.coverage -= coverage_executable_candidate_bonus_;
      }
    } else {
      terms.gain_norm =
          viewpoint->frontier_information_gain_ /
          (std::max(1.0e-3, ep_->candidate_gain_saturation_) +
           viewpoint->frontier_information_gain_);
      terms.wait_norm = std::clamp(
          viewpoint->frontier_wait_age_ /
              std::max(1.0, ep_->candidate_wait_saturation_),
          0.0, 1.0);
      terms.debt_norm = std::clamp(
          viewpoint->frontier_pass_debt_ /
              std::max(1.0, ep_->candidate_debt_saturation_),
          0.0, 1.0);
      terms.coverage = coverage_guidance_
                           ? coverage_guidance_->clusterPenalty(
                                 viewpoint->frontier_cluster_id_,
                                 viewpoint->center_.cast<double>())
                           : 0.0;
    }
    terms.failed_goal = failedGoalPenalty(viewpoint);
    terms.swarm = swarm_candidate_penalties[i];
  };
  auto finishCompositeCost = [&](const int i) {
    CandidateCostBreakdown &terms = candidate_terms[i];
    if (!ep_->composite_candidate_cost_enable_) {
      terms.total = viewpoint_reachable_distance[i] + terms.coverage +
                    terms.failed_goal + terms.swarm + terms.region;
      return;
    }
    terms.total =
        ep_->candidate_travel_weight_ * terms.travel +
        ep_->candidate_turn_brake_weight_ * terms.turn_brake +
        ep_->candidate_future_return_weight_ * terms.future_return -
        ep_->candidate_information_gain_weight_ * terms.gain_norm -
        ep_->candidate_wait_weight_ * terms.wait_norm -
        ep_->candidate_debt_weight_ * terms.debt_norm + terms.coverage +
        terms.failed_goal + terms.swarm + terms.region;
  };

  for (int i = 0; i < static_cast<int>(candidate_terms.size()); ++i) {
    fillStaticTerms(i);
  }

  // Estimate the cost of skipping a currently cheap/high-value frontier and
  // returning to it after visiting candidate i.  This is evaluated only for
  // next-goal selection; adding a fixed node reward to an all-node TSP would be
  // a constant and could not change visit order.
  const int return_horizon = std::clamp(
      ep_->candidate_return_horizon_, 1,
      std::max(1, static_cast<int>(viewpoint_reachable.size()) - 1));
  for (int i = 0; i < static_cast<int>(viewpoint_reachable.size()); ++i) {
    struct ReturnAlternative {
      double priority;
      double extra;
    };
    vector<ReturnAlternative> alternatives;
    alternatives.reserve(viewpoint_reachable.size() - 1);
    for (int k = 0; k < static_cast<int>(viewpoint_reachable.size()); ++k) {
      if (k == i) {
        continue;
      }
      const double extra = std::max(
          0.0, mat(i + 1, k + 1) - candidate_terms[k].travel);
      if (extra <= 1.0e-6) {
        continue;
      }
      const double priority = 1.0 + candidate_terms[k].gain_norm +
                              candidate_terms[k].wait_norm +
                              candidate_terms[k].debt_norm;
      alternatives.push_back({priority, extra});
    }
    std::stable_sort(
        alternatives.begin(), alternatives.end(),
        [](const ReturnAlternative &a, const ReturnAlternative &b) {
          return a.priority * a.extra > b.priority * b.extra;
        });
    double weighted_extra = 0.0;
    double weight_sum = 0.0;
    for (int k = 0;
         k < std::min(return_horizon, static_cast<int>(alternatives.size()));
         ++k) {
      weighted_extra += alternatives[k].priority * alternatives[k].extra;
      weight_sum += alternatives[k].priority;
    }
    candidate_terms[i].future_return =
        std::min(std::max(0.0, ep_->candidate_return_cost_cap_),
                 weight_sum > 1.0e-6 ? weighted_extra / weight_sum : 0.0);
    finishCompositeCost(i);
  }

  vector<double> costs;
  for (const auto &terms : candidate_terms) costs.push_back(terms.total);
  return costs;
}

int FastExplorationManager::selectExplorationTour(
    const vector<TopoNode::Ptr> &candidates,
    const vector<EdgeSafetyCost> &edges, const Eigen::MatrixXd &path_costs,
    const Eigen::Vector3d &velocity, bool priority_floor_active,
    bool ascending_to_priority_floor, int first_priority_floor_rank,
    vector<int> &indices) {
  indices.clear();
  const int count = static_cast<int>(candidates.size());
  if (count == 0 || static_cast<int>(edges.size()) != count ||
      path_costs.rows() != count + 1 || path_costs.cols() != count + 1 ||
      !path_costs.allFinite()) return -1;

  Eigen::MatrixXd open_costs = path_costs;
  open_costs.col(0).setZero();
  const bool biased = (coverage_guidance_ && coverage_guidance_->affectsPlanning()) ||
                      (swarm_coordinator_ && swarm_coordinator_->enabled());
  vector<double> costs(count);
  int proposed = 0;
  auto orderWithFirst = [](const vector<int> &order, int first) {
    vector<int> result{0, first};
    for (const int node : order)
      if (node != 0 && node != first) result.push_back(node);
    return result;
  };
  // Prescribe a first action by solving the remaining open tour from that
  // action. This avoids magic departure penalties and preserves every node.
  auto solveFromFirst = [&](int first) {
    vector<int> mapping{first};
    for (int node = 1; node <= count; ++node)
      if (node != first) mapping.push_back(node);
    Eigen::MatrixXd tail = Eigen::MatrixXd::Zero(count, count);
    for (int i = 0; i < count; ++i)
      for (int j = 1; j < count; ++j)
        tail(i, j) = open_costs(mapping[i], mapping[j]);
    vector<int> suffix;
    solveTour(tail, suffix);
    indices.assign(1, 0);
    for (int node : suffix) indices.push_back(mapping[node]);
  };

  if (biased) {
    vector<double> departure;
    for (int i = 0; i < count; ++i) departure.push_back(path_costs(0, i + 1));
    costs = scoreExplorationCandidates(candidates, departure, edges, path_costs,
        priority_floor_active, ascending_to_priority_floor, first_priority_floor_rank);
    const auto less = [&](int a, int b) {
      if (std::fabs(costs[a] - costs[b]) > 1e-6) return costs[a] < costs[b];
      for (int axis = 0; axis < 3; ++axis)
        if (candidates[a]->center_[axis] != candidates[b]->center_[axis])
          return candidates[a]->center_[axis] < candidates[b]->center_[axis];
      if (candidates[a]->is_coverage_target_ != candidates[b]->is_coverage_target_)
        return !candidates[a]->is_coverage_target_;
      return candidates[a]->frontier_cluster_id_ < candidates[b]->frontier_cluster_id_;
    };
    for (int i = 1; i < count; ++i) if (less(i, proposed)) proposed = i;
  } else {
    solveTour(open_costs, indices);
    if (static_cast<int>(indices.size()) != count + 1) return -1;
    proposed = indices[1] - 1;
    // Compare route costs when applying hysteresis. Comparing only the first
    // edge used to veto a better tour merely because its first hop was longer.
    for (int candidate = 0; candidate < count; ++candidate) {
      const auto order = orderWithFirst(indices, candidate + 1);
      double total = 0.0;
      for (std::size_t i = 1; i < order.size(); ++i)
        total += open_costs(order[i - 1], order[i]);
      costs[candidate] = total;
    }
  }
  const int chosen = selectStableGoalIndex(candidates, costs, proposed, velocity);
  if (biased || chosen != proposed) solveFromFirst(chosen + 1);
  if (static_cast<int>(indices.size()) != count + 1 || indices[1] != chosen + 1)
    return -1;
  ROS_INFO_STREAM_THROTTLE(0.5, "[exploration selection] policy="
      << (biased ? "candidate" : "tour") << " proposed_cluster="
      << candidates[proposed]->frontier_cluster_id_ << " chosen_cluster="
      << candidates[chosen]->frontier_cluster_id_ << " coverage_id="
      << candidates[chosen]->coverage_target_id_ << " objective=" << costs[chosen]
      << " retained=" << (chosen != proposed));
  return chosen;
}

int FastExplorationManager::selectStableGoalIndex(
    const vector<TopoNode::Ptr> &viewpoints,
    const vector<double> &distance_odom2vp, const int candidate_idx,
    const Eigen::Vector3d &vel) {
  if (!ep_->goal_lock_enable_ || viewpoints.empty() || candidate_idx < 0 ||
      candidate_idx >= static_cast<int>(viewpoints.size())) {
    return candidate_idx;
  }

  const ros::Time now = ros::Time::now();
  int locked_idx = -1;
  double locked_match_distance = std::numeric_limits<double>::max();
  if (ed_->has_goal_lock_) {
    if (ed_->locked_goal_is_mission_) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (viewpoints[i]->is_mission_goal_target_) {
          locked_idx = i;
          locked_match_distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          break;
        }
      }
    } else if (ed_->locked_goal_is_coverage_ &&
        ed_->locked_goal_coverage_id_ != 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (viewpoints[i]->is_coverage_target_ &&
            viewpoints[i]->coverage_target_id_ ==
                ed_->locked_goal_coverage_id_) {
          locked_idx = i;
          locked_match_distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          break;
        }
      }
    } else if (ed_->locked_goal_cluster_id_ >= 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (!viewpoints[i]->is_coverage_target_ &&
            viewpoints[i]->frontier_cluster_id_ ==
            ed_->locked_goal_cluster_id_) {
          const double distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          // A globally cached cluster may retain its numeric ID while its best
          // observation point moves to another doorway/side of a room.  Do not
          // force that spatially different target through the old goal lock.
          if (distance <= ep_->goal_lock_match_radius_) {
            locked_idx = i;
            locked_match_distance = distance;
            break;
          }
        }
      }
    }
    if (locked_idx < 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        const double distance =
            (viewpoints[i]->center_ - ed_->locked_goal_).norm();
        if (distance < locked_match_distance) {
          locked_match_distance = distance;
          locked_idx = i;
        }
      }
      if (locked_match_distance > ep_->goal_lock_match_radius_) {
        locked_idx = -1;
      }
    }
  }

  int chosen_idx = candidate_idx;
  const bool hold_coverage_action=coverageMotionEnabled() && has_active_coverage_goal_ &&
      locked_idx>=0 && viewpoints[locked_idx]->is_coverage_target_ &&
      viewpoints[locked_idx]->coverage_target_id_==active_coverage_target_.stable_id &&
      (viewpoints[locked_idx]->center_.cast<double>()-active_coverage_target_.approach_position).norm()<0.60 &&
      !active_coverage_goal_start_.isZero() &&
      (now-active_coverage_goal_start_).toSec()<coverage_recovery_timeout_;
  if (hold_coverage_action) {
    chosen_idx=locked_idx;
  } else if (locked_idx >= 0 && locked_idx != candidate_idx) {
    const double candidate_cost =
        candidate_idx < static_cast<int>(distance_odom2vp.size())
            ? distance_odom2vp[candidate_idx]
            : 0.0;
    const double locked_cost =
        locked_idx < static_cast<int>(distance_odom2vp.size())
            ? distance_odom2vp[locked_idx]
            : ed_->locked_goal_cost_;
    const double since_lock = (now - ed_->locked_goal_time_).toSec();
    // Goal switching becomes expensive before the exact high-speed threshold.
    // Waiting until v >= MaxVelMag made the high-speed multiplier practically
    // unreachable in normal flight (the logged odometry usually stays just
    // below the configured maximum).
    const bool high_speed =
        vel.norm() >=
        planner_manager_->gcopter_config_->highSpeedModeExitThreshold;
    const double high_speed_multiplier =
        high_speed ? std::max(1.0, ep_->goal_switch_high_speed_multiplier_) : 1.0;
    const double min_improvement =
        ep_->goal_switch_min_improvement_ * high_speed_multiplier;
    const double candidate_improvement = locked_cost - candidate_cost;
    const bool in_cooldown =
        since_lock < ep_->goal_switch_min_interval_ * high_speed_multiplier;
    const double relative_margin =
        std::max(0.0, ep_->goal_keep_cost_ratio_ - 1.0) *
        std::max(1.0, std::fabs(candidate_cost));
    const bool old_goal_cost_ok =
        locked_cost <= candidate_cost + relative_margin + min_improvement;

    if (in_cooldown || old_goal_cost_ok ||
        candidate_improvement < min_improvement) {
      chosen_idx = locked_idx;
      ROS_INFO_STREAM_THROTTLE(
          0.5,
          "[goal lock] keep goal idx=" << locked_idx
                                      << " candidate_idx=" << candidate_idx
                                      << " locked_cost=" << locked_cost
                                      << " candidate_cost=" << candidate_cost
                                      << " since=" << since_lock
                                      << " high_speed=" << high_speed);
    }
  }

  // Frontier IDs are regenerated during reclustering.  A nearby replacement
  // is the same logical goal even if its numeric ID changed.
  const bool chosen_is_mission =
      viewpoints[chosen_idx]->is_mission_goal_target_;
  const bool chosen_is_coverage =
      !chosen_is_mission && viewpoints[chosen_idx]->is_coverage_target_;
  const bool same_identity =
      ed_->has_goal_lock_ &&
      chosen_is_mission == ed_->locked_goal_is_mission_ &&
      chosen_is_coverage == ed_->locked_goal_is_coverage_ &&
      (chosen_is_mission
           ? true
           : chosen_is_coverage
           ? viewpoints[chosen_idx]->coverage_target_id_ ==
                 ed_->locked_goal_coverage_id_
           : viewpoints[chosen_idx]->frontier_cluster_id_ ==
                     ed_->locked_goal_cluster_id_ &&
                 (viewpoints[chosen_idx]->center_ - ed_->locked_goal_).norm() <=
                     ep_->goal_lock_match_radius_);
  const bool new_lock =
      !same_identity &&
      (!ed_->has_goal_lock_ ||
       (viewpoints[chosen_idx]->center_ - ed_->locked_goal_).norm() >
           ep_->goal_lock_match_radius_);
  ed_->has_goal_lock_ = true;
  ed_->locked_goal_is_mission_ = chosen_is_mission;
  ed_->locked_goal_is_coverage_ = chosen_is_coverage;
  ed_->locked_goal_cluster_id_ =
      (chosen_is_coverage || chosen_is_mission)
          ? -1
          : viewpoints[chosen_idx]->frontier_cluster_id_;
  ed_->locked_goal_coverage_id_ =
      chosen_is_coverage ? viewpoints[chosen_idx]->coverage_target_id_ : 0;
  ed_->locked_goal_ = viewpoints[chosen_idx]->center_;
  ed_->locked_goal_yaw_ = viewpoints[chosen_idx]->yaw_;
  ed_->locked_goal_cost_ =
      chosen_idx < static_cast<int>(distance_odom2vp.size())
          ? distance_odom2vp[chosen_idx]
          : 0.0;
  if (new_lock) {
    ed_->locked_goal_time_ = now;
    if (frontier_manager_ptr_ && !chosen_is_coverage && !chosen_is_mission) {
      frontier_manager_ptr_->markClusterGoalSelected(
          viewpoints[chosen_idx]->frontier_cluster_id_,
          viewpoints[chosen_idx]->center_, ep_->goal_lock_match_radius_);
    }
  }
  return chosen_idx;
}

void FastExplorationManager::solveTour(Eigen::MatrixXd &cost_mat,
                                       vector<int> &indices) {
  indices.clear();
  const int dimension = cost_mat.rows();
  if (dimension <= 0 || cost_mat.cols() != dimension)
    return;
  if (dimension == 1) {
    indices.emplace_back(0);
    return;
  }

  if (dimension >= 3 && ep_->use_lkh_ && solveLKH(cost_mat, indices)) {
    return;
  }

  if (dimension >= 3 && ep_->use_lkh_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[global tour] LKH failed; use deterministic ATSP fallback");
  }
  solveFallbackTour(cost_mat, indices);
}

bool FastExplorationManager::solveLKH(const Eigen::MatrixXd &cost_mat,
                                      vector<int> &indices) {
  indices.clear();
  const int dimension = cost_mat.rows();
  if (dimension < 3 || cost_mat.cols() != dimension || ep_->tsp_dir_.empty()) {
    return false;
  }

  const string problem_file = ep_->tsp_dir_ + "/single.tsp";
  const string parameter_file = ep_->tsp_dir_ + "/single.par";
  const string result_file = ep_->tsp_dir_ + "/single.txt";
  ofstream problem(problem_file, std::ios::out | std::ios::trunc);
  if (!problem.is_open()) {
    return false;
  }

  problem << "NAME : single\n"
          << "TYPE : ATSP\n"
          << "DIMENSION : " << dimension << "\n"
          << "EDGE_WEIGHT_TYPE : EXPLICIT\n"
          << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
          << "EDGE_WEIGHT_SECTION\n";
  constexpr double kScale = 100.0;
  constexpr int kMaxLkhCost = std::numeric_limits<int>::max() / 8;
  for (int row = 0; row < dimension; ++row) {
    for (int col = 0; col < dimension; ++col) {
      const double cost = cost_mat(row, col);
      if (!std::isfinite(cost)) {
        problem.close();
        return false;
      }
      const long long scaled = std::llround(cost * kScale);
      problem << std::clamp<long long>(scaled, 0, kMaxLkhCost) << " ";
    }
    problem << "\n";
  }
  problem << "EOF\n";
  problem.close();
  if (!problem) {
    return false;
  }

  // Do not accept a tour left by an earlier failed invocation.
  std::remove(result_file.c_str());
  if (solveTSPLKH(parameter_file.c_str()) != EXIT_SUCCESS) {
    return false;
  }

  ifstream result(result_file);
  if (!result.is_open()) {
    return false;
  }
  string line;
  bool in_tour_section = false;
  vector<int> raw_tour;
  raw_tour.reserve(dimension);
  while (std::getline(result, line)) {
    if (!in_tour_section) {
      if (line == "TOUR_SECTION") {
        in_tour_section = true;
      }
      continue;
    }
    std::istringstream line_stream(line);
    int id = 0;
    if (!(line_stream >> id)) {
      continue;
    }
    if (id == -1) {
      break;
    }
    raw_tour.emplace_back(id - 1);
  }
  if (static_cast<int>(raw_tour.size()) != dimension) {
    return false;
  }

  vector<bool> seen(dimension, false);
  for (const int node : raw_tour) {
    if (node < 0 || node >= dimension || seen[node]) {
      return false;
    }
    seen[node] = true;
  }
  const auto depot = std::find(raw_tour.begin(), raw_tour.end(), 0);
  if (depot == raw_tour.end()) {
    return false;
  }
  indices.insert(indices.end(), depot, raw_tour.end());
  indices.insert(indices.end(), raw_tour.begin(), depot);
  return static_cast<int>(indices.size()) == dimension && indices.front() == 0;
}

void FastExplorationManager::solveFallbackTour(
    const Eigen::MatrixXd &cost_mat, vector<int> &indices) const {
  indices.clear();
  const int dimension = cost_mat.rows();
  if (dimension <= 0 || cost_mat.cols() != dimension) {
    return;
  }
  if (dimension == 1) {
    indices.emplace_back(0);
    return;
  }

  constexpr int kExactMaxDimension = 16;
  const double inf = std::numeric_limits<double>::infinity();
  if (dimension <= kExactMaxDimension) {
    const int node_num = dimension - 1;
    const int state_num = 1 << node_num;
    vector<double> dp(static_cast<std::size_t>(state_num) * node_num, inf);
    vector<int> parent(static_cast<std::size_t>(state_num) * node_num, -1);
    const auto offset = [node_num](const int mask, const int node) {
      return static_cast<std::size_t>(mask) * node_num + node;
    };

    for (int node = 0; node < node_num; ++node) {
      dp[offset(1 << node, node)] = cost_mat(0, node + 1);
    }
    for (int mask = 1; mask < state_num; ++mask) {
      for (int node = 0; node < node_num; ++node) {
        if ((mask & (1 << node)) == 0)
          continue;
        const int previous_mask = mask ^ (1 << node);
        if (previous_mask == 0)
          continue;
        for (int previous = 0; previous < node_num; ++previous) {
          if ((previous_mask & (1 << previous)) == 0)
            continue;
          const double candidate =
              dp[offset(previous_mask, previous)] +
              cost_mat(previous + 1, node + 1);
          if (candidate < dp[offset(mask, node)]) {
            dp[offset(mask, node)] = candidate;
            parent[offset(mask, node)] = previous;
          }
        }
      }
    }

    const int full_mask = state_num - 1;
    int last_node = -1;
    double best_cost = inf;
    for (int node = 0; node < node_num; ++node) {
      const double cycle_cost =
          dp[offset(full_mask, node)] + cost_mat(node + 1, 0);
      if (cycle_cost < best_cost) {
        best_cost = cycle_cost;
        last_node = node;
      }
    }
    if (last_node >= 0 && std::isfinite(best_cost)) {
      vector<int> reversed;
      int mask = full_mask;
      while (last_node >= 0) {
        reversed.push_back(last_node + 1);
        const int previous = parent[offset(mask, last_node)];
        mask ^= 1 << last_node;
        last_node = previous;
      }
      indices.push_back(0);
      indices.insert(indices.end(), reversed.rbegin(), reversed.rend());
      return;
    }
  }

  // Directed cheapest insertion keeps the depot-closing edge in the objective.
  // For the current open-route matrix every frontier-to-depot edge is zero,
  // and any prescribed first action is the depot of a smaller suffix tour.
  vector<bool> visited(dimension, false);
  vector<int> route{0};
  visited[0] = true;
  while (static_cast<int>(route.size()) < dimension) {
    int best_node = -1;
    int best_insert_after = -1;
    double best_delta = inf;
    for (int node = 1; node < dimension; ++node) {
      if (visited[node]) {
        continue;
      }
      for (int pos = 0; pos < static_cast<int>(route.size()); ++pos) {
        const int from = route[pos];
        const int to = pos + 1 < static_cast<int>(route.size())
                           ? route[pos + 1]
                           : 0;
        const double delta = cost_mat(from, node) + cost_mat(node, to) -
                             cost_mat(from, to);
        if (std::isfinite(delta) && delta < best_delta) {
          best_delta = delta;
          best_node = node;
          best_insert_after = pos;
        }
      }
    }
    if (best_node < 0) {
      indices.clear();
      return;
    }
    route.insert(route.begin() + best_insert_after + 1, best_node);
    visited[best_node] = true;
  }

  const auto cycle_cost = [&cost_mat](const vector<int> &tour) {
    double cost = 0.0;
    for (int i = 0; i < static_cast<int>(tour.size()); ++i) {
      cost += cost_mat(tour[i],
                       i + 1 < static_cast<int>(tour.size()) ? tour[i + 1] : 0);
    }
    return cost;
  };
  double best_cost = cycle_cost(route);
  // A few deterministic relocate passes cheaply remove bad insertions while
  // retaining directed edge costs.
  for (int pass = 0; pass < 4; ++pass) {
    bool improved = false;
    for (int from = 1; from < dimension && !improved; ++from) {
      for (int insert_at = 1; insert_at < dimension && !improved;
           ++insert_at) {
        vector<int> candidate = route;
        const int node = candidate[from];
        candidate.erase(candidate.begin() + from);
        candidate.insert(candidate.begin() + insert_at, node);
        if (candidate == route) {
          continue;
        }
        const double candidate_cost = cycle_cost(candidate);
        if (candidate_cost + 1.0e-9 < best_cost) {
          route.swap(candidate);
          best_cost = candidate_cost;
          improved = true;
        }
      }
    }
    if (!improved) {
      break;
    }
  }
  indices.swap(route);
}

}  // namespace fast_planner
