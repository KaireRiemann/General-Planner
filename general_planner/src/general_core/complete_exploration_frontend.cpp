#include <general_core/complete_exploration_frontend.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>

namespace general_planner {
namespace {
constexpr double kInfCost = 1.0e9;

bool committedRet(const super_utils::RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::NEW_TRAJ ||
           ret == super_utils::FINISH;
}

double wrapYawDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

double pathLength(const super_utils::vec_E<super_utils::Vec3f> &path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

super_utils::vec_E<super_utils::Vec3f> truncatePath(
        const super_utils::vec_E<super_utils::Vec3f> &path,
        const double distance) {
    super_utils::vec_E<super_utils::Vec3f> truncated;
    if (path.empty()) {
        return truncated;
    }
    truncated.push_back(path.front());
    if (path.size() == 1 || distance <= 0.0) {
        return truncated;
    }
    double accumulated = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const super_utils::Vec3f from = path[i - 1];
        const super_utils::Vec3f to = path[i];
        const double segment = (to - from).norm();
        if (segment < 1.0e-6) {
            continue;
        }
        if (accumulated + segment >= distance) {
            const double ratio = (distance - accumulated) / segment;
            const super_utils::Vec3f cut = from + ratio * (to - from);
            if ((cut - truncated.back()).norm() > 1.0e-4) {
                truncated.push_back(cut);
            }
            return truncated;
        }
        if ((to - truncated.back()).norm() > 1.0e-4) {
            truncated.push_back(to);
        }
        accumulated += segment;
    }
    return truncated;
}

std::size_t mixHash(std::size_t seed, const int value) {
    const std::size_t h = std::hash<int>{}(value);
    return seed ^ (h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}
}  // namespace

CompleteExplorationFrontend::CompleteExplorationFrontend(const Config &cfg,
                                                         const MapManager::Ptr &map_manager,
                                                         const path_search::Astar::Ptr &astar)
        : cfg_(cfg),
          map_manager_(map_manager),
          astar_(astar),
          memory_(cfg.memory_cfg),
          frontier_db_(cfg.frontier_db_cfg),
          region_graph_(cfg.region_graph_cfg),
          coverage_planner_(cfg.coverage_cfg),
          viewpoint_planner_(cfg.viewpoint_cfg, map_manager, astar),
          tour_solver_(cfg.lkh_cfg) {
    cfg_.finish_confirm_count = std::max(1, cfg_.finish_confirm_count);
    cfg_.finish_confirm_time = std::max(0.0, cfg_.finish_confirm_time);
    cfg_.update_radius = std::max(1.0, cfg_.update_radius);
    cfg_.max_frontiers_per_cycle = std::max(1, cfg_.max_frontiers_per_cycle);
    cfg_.tour_refined_num = std::max(1, cfg_.tour_refined_num);
    cfg_.tour_refined_radius = std::max(0.5, cfg_.tour_refined_radius);
    cfg_.tour_replan_min_interval = std::max(0.0, cfg_.tour_replan_min_interval);
    cfg_.tour_radius_far = std::max(1.0, cfg_.tour_radius_far);
    cfg_.tour_radius_close = std::max(0.2, cfg_.tour_radius_close);
}

void CompleteExplorationFrontend::reset() {
    memory_.reset();
    frontier_db_.reset();
    region_graph_.reset();
    coverage_planner_.reset();
    exploration_finished_ = false;
    finish_confirm_counter_ = 0;
    finish_confirm_start_time_ = -1.0;
    last_new_frontier_time_ = 0.0;
    last_active_count_ = 0;
    latest_candidate_count_ = 0;
    latest_reachable_candidate_count_ = 0;
    latest_selected_frontier_id_ = -1;
    latest_selected_region_id_ = -1;
    latest_selected_score_ = 0.0;
    latest_selected_gain_ = 0.0;
    latest_selected_travel_cost_ = 0.0;
    latest_memory_known_free_ = 0;
    latest_memory_occupied_ = 0;
    latest_memory_unknown_ = 0;
    latest_tour_size_ = 0;
    latest_tour_used_lkh_ = false;
    latest_tour_cost_ = 0.0;
    latest_tour_reason_ = "reset";
    latest_reason_ = "reset";
    active_tour_.clear();
    last_tour_frontier_hash_ = 0;
    last_tour_update_time_ = -1.0;
    ++debug_sequence_;
    latest_debug_info_ = ExplorationDebugInfo{};
    latest_debug_info_.sequence = debug_sequence_;
}

double CompleteExplorationFrontend::now() const {
    static const auto start = std::chrono::steady_clock::now();
    const auto current = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(current - start).count();
}

bool CompleteExplorationFrontend::checkFinishCondition(const double stamp) {
    const bool no_blocking_frontier =
            frontier_db_.activeCount() == 0 &&
            frontier_db_.reachableCount() == 0 &&
            frontier_db_.dormantCount() == 0;
    if (!no_blocking_frontier) {
        finish_confirm_counter_ = 0;
        finish_confirm_start_time_ = -1.0;
        return false;
    }
    if (finish_confirm_counter_ == 0) {
        finish_confirm_start_time_ = stamp;
    }
    ++finish_confirm_counter_;
    const bool count_ok = finish_confirm_counter_ >= cfg_.finish_confirm_count;
    const bool time_ok = finish_confirm_start_time_ >= 0.0 &&
                         stamp - finish_confirm_start_time_ >= cfg_.finish_confirm_time;
    return count_ok && time_ok;
}

void CompleteExplorationFrontend::fillGoalFromViewpoint(const CompleteExplorationViewpoint &viewpoint,
                                                        const super_utils::Vec3f &robot_pos,
                                                        ExplorationGoal &goal) const {
    goal = ExplorationGoal{};
    goal.valid = true;
    goal.position = viewpoint.position;
    goal.yaw = viewpoint.yaw;
    goal.score = viewpoint.score;
    goal.information_gain = viewpoint.information_gain;
    goal.travel_cost = viewpoint.travel_cost;
    goal.yaw_cost = viewpoint.yaw_cost;
    goal.curvature_cost = viewpoint.curvature_cost;
    goal.distance_to_robot = (viewpoint.position - robot_pos).norm();
    goal.unknown_risk = viewpoint.unknown_risk;
    goal.lifecycle_score = -viewpoint.fail_penalty;
    goal.frontier_cluster_id = viewpoint.frontier_id;
    goal.reason = "complete exploration selected viewpoint";
    goal.viewpoint_case = "complete_coverage";
    goal.guide_path = viewpoint.guide_path;
}

std::size_t CompleteExplorationFrontend::candidateFrontierHash(
        const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier) const {
    std::vector<int> ids;
    ids.reserve(viewpoints_by_frontier.size());
    for (const auto &kv : viewpoints_by_frontier) {
        if (!kv.second.empty()) {
            ids.push_back(kv.first);
        }
    }
    std::sort(ids.begin(), ids.end());
    std::size_t seed = 0;
    for (const int id : ids) {
        seed = mixHash(seed, id);
    }
    seed = mixHash(seed, static_cast<int>(ids.size()));
    return seed;
}

void CompleteExplorationFrontend::pruneTour(
        const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier) {
    active_tour_.erase(
            std::remove_if(active_tour_.begin(),
                           active_tour_.end(),
                           [&](const int id) {
                               const auto it = viewpoints_by_frontier.find(id);
                               return it == viewpoints_by_frontier.end() || it->second.empty();
                           }),
            active_tour_.end());
}

bool CompleteExplorationFrontend::rebuildFuelStyleTour(
        const super_utils::StatePVAJ &robot_state,
        const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier,
        const double current_yaw,
        const double stamp) {
    active_tour_.clear();
    latest_tour_size_ = 0;
    latest_tour_used_lkh_ = false;
    latest_tour_cost_ = 0.0;
    latest_tour_reason_.clear();

    std::vector<int> frontier_ids;
    frontier_ids.reserve(viewpoints_by_frontier.size());
    for (const auto &kv : viewpoints_by_frontier) {
        if (!kv.second.empty()) {
            frontier_ids.push_back(kv.first);
        }
    }
    std::sort(frontier_ids.begin(), frontier_ids.end(), [&](const int lhs, const int rhs) {
        const auto &lv = viewpoints_by_frontier.at(lhs).front();
        const auto &rv = viewpoints_by_frontier.at(rhs).front();
        if (std::abs(lv.travel_cost - rv.travel_cost) > 1.0e-6) {
            return lv.travel_cost < rv.travel_cost;
        }
        return lhs < rhs;
    });
    if (frontier_ids.empty()) {
        latest_tour_reason_ = "no frontier with valid viewpoint";
        return false;
    }
    if (frontier_ids.size() == 1) {
        active_tour_ = frontier_ids;
        latest_tour_size_ = 1;
        latest_tour_reason_ = "single-frontier tour";
        last_tour_frontier_hash_ = candidateFrontierHash(viewpoints_by_frontier);
        last_tour_update_time_ = stamp;
        return true;
    }

    const super_utils::Vec3f robot_pos = robot_state.col(0);
    const int dim = static_cast<int>(frontier_ids.size()) + 1;
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(dim, dim);

    for (int i = 1; i < dim; ++i) {
        const auto &viewpoint = viewpoints_by_frontier.at(frontier_ids[static_cast<std::size_t>(i - 1)]).front();
        const double travel = std::isfinite(viewpoint.travel_cost)
                                      ? viewpoint.travel_cost
                                      : (viewpoint.position - robot_pos).norm();
        cost_matrix(0, i) = std::max(0.01, travel + 0.25 * std::abs(wrapYawDiff(viewpoint.yaw, current_yaw)));
        cost_matrix(i, 0) = 0.0;
    }

    for (int i = 1; i < dim; ++i) {
        const auto &from = viewpoints_by_frontier.at(frontier_ids[static_cast<std::size_t>(i - 1)]).front();
        for (int j = 1; j < dim; ++j) {
            if (i == j) {
                cost_matrix(i, j) = 0.0;
                continue;
            }
            const auto &to = viewpoints_by_frontier.at(frontier_ids[static_cast<std::size_t>(j - 1)]).front();
            const double dist = (to.position - from.position).norm();
            const double yaw_cost = std::abs(wrapYawDiff(to.yaw, from.yaw));
            const double gain_reward = 0.02 * std::max(0.0, to.information_gain);
            cost_matrix(i, j) = std::max(0.01, dist + 0.25 * yaw_cost - gain_reward);
        }
    }

    std::vector<int> raw_tour;
    std::string reason;
    if (!tour_solver_.solve(cost_matrix,
                            raw_tour,
                            latest_tour_cost_,
                            latest_tour_used_lkh_,
                            reason)) {
        latest_tour_reason_ = reason.empty() ? "ATSP tour failed" : reason;
        return false;
    }

    for (const int idx : raw_tour) {
        if (idx <= 0 || idx >= dim) {
            continue;
        }
        active_tour_.push_back(frontier_ids[static_cast<std::size_t>(idx - 1)]);
    }
    if (active_tour_.empty()) {
        latest_tour_reason_ = "ATSP returned empty frontier tour";
        return false;
    }

    latest_tour_size_ = static_cast<int>(active_tour_.size());
    latest_tour_reason_ = reason.empty() ? "ATSP tour solved" : reason;
    last_tour_frontier_hash_ = candidateFrontierHash(viewpoints_by_frontier);
    last_tour_update_time_ = stamp;
    return true;
}

bool CompleteExplorationFrontend::chooseRefinedViewpoint(
        const super_utils::StatePVAJ &robot_state,
        const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier,
        CompleteExplorationViewpoint &selected) const {
    selected = CompleteExplorationViewpoint{};
    if (active_tour_.empty()) {
        return false;
    }

    const super_utils::Vec3f robot_pos = robot_state.col(0);
    std::vector<int> refined_ids;
    refined_ids.reserve(static_cast<std::size_t>(cfg_.tour_refined_num));
    for (const int id : active_tour_) {
        const auto it = viewpoints_by_frontier.find(id);
        if (it == viewpoints_by_frontier.end() || it->second.empty()) {
            continue;
        }
        if (!refined_ids.empty() &&
            (it->second.front().position - robot_pos).norm() > cfg_.tour_refined_radius &&
            refined_ids.size() >= 2U) {
            break;
        }
        refined_ids.push_back(id);
        if (static_cast<int>(refined_ids.size()) >= cfg_.tour_refined_num) {
            break;
        }
    }
    if (refined_ids.empty()) {
        return false;
    }

    std::vector<const std::vector<CompleteExplorationViewpoint> *> layers;
    layers.reserve(refined_ids.size());
    for (const int id : refined_ids) {
        const auto it = viewpoints_by_frontier.find(id);
        if (it == viewpoints_by_frontier.end() || it->second.empty()) {
            return false;
        }
        layers.push_back(&it->second);
    }

    std::vector<double> prev_cost(layers.front()->size(), kInfCost);
    std::vector<std::vector<int>> parent;
    parent.reserve(layers.size());
    parent.emplace_back(layers.front()->size(), -1);
    for (std::size_t i = 0; i < layers.front()->size(); ++i) {
        const auto &view = (*layers.front())[i];
        const double travel = std::isfinite(view.travel_cost)
                                      ? view.travel_cost
                                      : (view.position - robot_pos).norm();
        prev_cost[i] = travel + 0.05 * view.score;
    }

    for (std::size_t layer = 1; layer < layers.size(); ++layer) {
        const auto &prev_layer = *layers[layer - 1];
        const auto &cur_layer = *layers[layer];
        std::vector<double> cur_cost(cur_layer.size(), kInfCost);
        std::vector<int> cur_parent(cur_layer.size(), -1);
        for (std::size_t j = 0; j < cur_layer.size(); ++j) {
            const auto &to = cur_layer[j];
            for (std::size_t i = 0; i < prev_layer.size(); ++i) {
                if (!std::isfinite(prev_cost[i])) {
                    continue;
                }
                const auto &from = prev_layer[i];
                const double transition =
                        (to.position - from.position).norm() +
                        0.25 * std::abs(wrapYawDiff(to.yaw, from.yaw));
                const double total = prev_cost[i] + transition + 0.05 * to.score;
                if (total < cur_cost[j]) {
                    cur_cost[j] = total;
                    cur_parent[j] = static_cast<int>(i);
                }
            }
        }
        parent.push_back(cur_parent);
        prev_cost.swap(cur_cost);
    }

    int best_idx = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < prev_cost.size(); ++i) {
        if (prev_cost[i] < best_cost) {
            best_cost = prev_cost[i];
            best_idx = static_cast<int>(i);
        }
    }
    if (!std::isfinite(best_cost)) {
        selected = layers.front()->front();
        return true;
    }

    int first_idx = best_idx;
    for (std::size_t layer = layers.size() - 1; layer > 0; --layer) {
        const int p = parent[layer][static_cast<std::size_t>(first_idx)];
        if (p < 0) {
            break;
        }
        first_idx = p;
    }
    selected = (*layers.front())[static_cast<std::size_t>(std::max(0, first_idx))];
    return true;
}

void CompleteExplorationFrontend::segmentFarGoal(const CompleteExplorationViewpoint &viewpoint,
                                                 const super_utils::Vec3f &robot_pos,
                                                 ExplorationGoal &goal) const {
    super_utils::vec_E<super_utils::Vec3f> guide_path = viewpoint.guide_path;
    if (guide_path.empty()) {
        guide_path.push_back(robot_pos);
        guide_path.push_back(viewpoint.position);
    }
    const double guide_length = pathLength(guide_path);
    if (guide_length > cfg_.tour_radius_far) {
        goal.position = viewpoint.position;
        goal.distance_to_robot = (viewpoint.position - robot_pos).norm();
        goal.travel_cost = guide_length;
        goal.guide_path = truncatePath(guide_path, cfg_.tour_radius_far);
        goal.reason = "selected by FUEL-style rolling ATSP tour viewpoint";
        goal.viewpoint_case = "fuel_rolling_viewpoint";
    } else {
        goal.travel_cost = guide_length;
        goal.guide_path = guide_path;
        goal.reason = guide_length < cfg_.tour_radius_close
                              ? "selected by FUEL-style close viewpoint"
                              : "selected by FUEL-style ATSP tour viewpoint";
        goal.viewpoint_case = guide_length < cfg_.tour_radius_close
                                      ? "fuel_close_viewpoint"
                                      : "fuel_tour_viewpoint";
    }
}

bool CompleteExplorationFrontend::selectFuelStyleGoal(
        const super_utils::StatePVAJ &robot_state,
        const double current_yaw,
        const std::vector<CompleteFrontierCluster> &frontiers,
        const std::vector<CompleteExplorationViewpoint> &candidates,
        const double stamp,
        ExplorationGoal &goal) {
    (void)frontiers;
    goal = ExplorationGoal{};
    std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> viewpoints_by_frontier;
    viewpoints_by_frontier.reserve(candidates.size());
    for (const auto &candidate : candidates) {
        if (!candidate.reachable || candidate.frontier_id <= 0) {
            continue;
        }
        viewpoints_by_frontier[candidate.frontier_id].push_back(candidate);
    }
    for (auto &kv : viewpoints_by_frontier) {
        std::sort(kv.second.begin(), kv.second.end(),
                  [](const CompleteExplorationViewpoint &lhs,
                     const CompleteExplorationViewpoint &rhs) {
                      if (std::abs(lhs.score - rhs.score) > 1.0e-9) return lhs.score < rhs.score;
                      if (std::abs(lhs.travel_cost - rhs.travel_cost) > 1.0e-9) {
                          return lhs.travel_cost < rhs.travel_cost;
                      }
                      return lhs.frontier_id < rhs.frontier_id;
                  });
    }

    pruneTour(viewpoints_by_frontier);
    const std::size_t current_hash = candidateFrontierHash(viewpoints_by_frontier);
    const bool first_target_valid = !active_tour_.empty() &&
                                    viewpoints_by_frontier.find(active_tour_.front()) !=
                                            viewpoints_by_frontier.end();
    const bool replan_interval_elapsed =
            last_tour_update_time_ < 0.0 ||
            stamp - last_tour_update_time_ >= cfg_.tour_replan_min_interval;

    const bool frontier_set_changed = current_hash != last_tour_frontier_hash_;
    if (!first_target_valid ||
        active_tour_.empty() ||
        (frontier_set_changed && replan_interval_elapsed)) {
        if (!rebuildFuelStyleTour(robot_state, viewpoints_by_frontier, current_yaw, stamp)) {
            goal.reason = latest_tour_reason_.empty()
                                  ? "failed to build FUEL-style frontier tour"
                                  : latest_tour_reason_;
            latest_reason_ = goal.reason;
            return false;
        }
    } else {
        latest_tour_size_ = static_cast<int>(active_tour_.size());
        latest_tour_reason_ = "reuse active frontier tour";
    }

    CompleteExplorationViewpoint selected_viewpoint;
    if (!chooseRefinedViewpoint(robot_state, viewpoints_by_frontier, selected_viewpoint)) {
        goal.reason = "failed to refine FUEL-style viewpoint";
        latest_reason_ = goal.reason;
        return false;
    }

    const super_utils::Vec3f robot_pos = robot_state.col(0);
    const double min_rolling_goal_distance =
            std::max(cfg_.tour_radius_close, 0.45 * cfg_.tour_radius_far);
    auto viewpoint_length = [&](const CompleteExplorationViewpoint &viewpoint) {
        const double guide_length = pathLength(viewpoint.guide_path);
        if (guide_length > 1.0e-6) {
            return guide_length;
        }
        if (std::isfinite(viewpoint.travel_cost) && viewpoint.travel_cost > 0.0) {
            return viewpoint.travel_cost;
        }
        return (viewpoint.position - robot_pos).norm();
    };
    if (active_tour_.size() > 1U &&
        viewpoint_length(selected_viewpoint) < min_rolling_goal_distance) {
        for (const int frontier_id : active_tour_) {
            const auto it = viewpoints_by_frontier.find(frontier_id);
            if (it == viewpoints_by_frontier.end()) {
                continue;
            }
            const auto progress_it = std::find_if(
                    it->second.begin(),
                    it->second.end(),
                    [&](const CompleteExplorationViewpoint &viewpoint) {
                        return viewpoint_length(viewpoint) >= min_rolling_goal_distance;
                    });
            if (progress_it != it->second.end()) {
                selected_viewpoint = *progress_it;
                break;
            }
        }
    }

    fillGoalFromViewpoint(selected_viewpoint, robot_pos, goal);
    segmentFarGoal(selected_viewpoint, robot_pos, goal);
    goal.score = selected_viewpoint.score + 0.02 * latest_tour_cost_;
    goal.frontier_cluster_id = selected_viewpoint.frontier_id;
    latest_selected_frontier_id_ = selected_viewpoint.frontier_id;
    latest_selected_region_id_ = selected_viewpoint.region_id;
    latest_selected_score_ = goal.score;
    latest_selected_gain_ = selected_viewpoint.information_gain;
    latest_selected_travel_cost_ = goal.travel_cost;
    latest_reason_ = goal.reason;
    frontier_db_.markSelected(selected_viewpoint.frontier_id, stamp);
    return true;
}

bool CompleteExplorationFrontend::planNextGoal(const super_utils::StatePVAJ &robot_state,
                                               const double current_yaw,
                                               ExplorationGoal &goal) {
    goal = ExplorationGoal{};
    exploration_finished_ = false;
    latest_selected_frontier_id_ = -1;
    latest_selected_region_id_ = -1;
    latest_candidate_count_ = 0;
    latest_reachable_candidate_count_ = 0;

    if (!cfg_.enable) {
        goal.reason = "complete exploration disabled";
        latest_reason_ = goal.reason;
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        goal.reason = "complete exploration map not ready";
        latest_reason_ = goal.reason;
        return false;
    }
    const super_utils::Vec3f robot_pos = robot_state.col(0);
    if (!robot_pos.allFinite() || !map_manager_->insideLocalMap(robot_pos)) {
        goal.reason = "complete exploration invalid robot state";
        latest_reason_ = goal.reason;
        return false;
    }

    const double stamp = now();
    ++debug_sequence_;
    latest_debug_info_ = ExplorationDebugInfo{};
    latest_debug_info_.sequence = debug_sequence_;
    latest_debug_info_.has_robot_state = true;
    latest_debug_info_.robot_position = robot_pos;
    latest_debug_info_.robot_yaw = current_yaw;
    memory_.updateFromMap(map_manager_, robot_pos, cfg_.update_radius, stamp);
    latest_memory_known_free_ = memory_.knownFreeCount();
    latest_memory_occupied_ = memory_.occupiedCount();
    latest_memory_unknown_ = memory_.explicitUnknownCount();
    super_utils::vec_E<CompleteFrontierCell> frontier_cells;
    memory_.getCandidateFrontierCells(frontier_cells);
    latest_debug_info_.frontier_voxels.reserve(frontier_cells.size());
    for (const auto &cell : frontier_cells) {
        FrontierVoxel voxel;
        voxel.position = cell.position;
        voxel.index = cell.index;
        latest_debug_info_.frontier_voxels.push_back(voxel);
    }
    frontier_db_.update(frontier_cells, memory_, stamp);
    frontier_db_.reviveUnreachableNear(robot_pos, std::max(2.0, cfg_.update_radius * 0.25), stamp);

    if (frontier_db_.activeCount() > last_active_count_) {
        last_new_frontier_time_ = stamp;
    }
    last_active_count_ = frontier_db_.activeCount();

    if (cfg_.use_region_graph) {
        region_graph_.update(memory_, frontier_db_, robot_pos, stamp);
    }
    std::vector<int> region_route;
    if (cfg_.use_coverage_guidance && cfg_.use_region_graph) {
        coverage_planner_.planRegionRoute(region_graph_,
                                          region_graph_.currentRegion(robot_pos),
                                          region_route);
    } else {
        coverage_planner_.reset();
    }

    std::vector<CompleteFrontierCluster> frontiers = frontier_db_.getActiveFrontiers();
    std::sort(frontiers.begin(), frontiers.end(),
              [&](const CompleteFrontierCluster &lhs, const CompleteFrontierCluster &rhs) {
                  const double lhs_key = lhs.estimated_gain - 0.15 * (lhs.center - robot_pos).norm();
                  const double rhs_key = rhs.estimated_gain - 0.15 * (rhs.center - robot_pos).norm();
                  if (std::abs(lhs_key - rhs_key) > 1.0e-9) return lhs_key > rhs_key;
                  return lhs.id < rhs.id;
              });
    if (static_cast<int>(frontiers.size()) > cfg_.max_frontiers_per_cycle) {
        frontiers.resize(static_cast<std::size_t>(cfg_.max_frontiers_per_cycle));
    }

    std::vector<CompleteExplorationViewpoint> candidates;
    candidates.reserve(static_cast<std::size_t>(cfg_.viewpoint_cfg.max_total_candidates));
    for (const auto &frontier : frontiers) {
        const std::size_t before = candidates.size();
        viewpoint_planner_.sampleAndScore(frontier,
                                          region_graph_,
                                          coverage_planner_,
                                          robot_state,
                                          current_yaw,
                                          candidates);
        if (candidates.size() == before) {
            frontier_db_.markDormant(frontier.id, stamp);
        }
        if (static_cast<int>(candidates.size()) >= cfg_.viewpoint_cfg.max_total_candidates) {
            break;
        }
    }
    latest_candidate_count_ = static_cast<int>(candidates.size());
    latest_reachable_candidate_count_ =
            static_cast<int>(std::count_if(candidates.begin(), candidates.end(),
                                           [](const CompleteExplorationViewpoint &candidate) {
                                               return candidate.reachable;
                                           }));
    latest_debug_info_.viewpoints.clear();
    latest_debug_info_.viewpoints.reserve(candidates.size());
    int debug_id = 0;
    for (const auto &candidate : candidates) {
        ExplorationViewpointDebug debug;
        debug.debug_id = debug_id++;
        debug.frontier_cluster_id = candidate.frontier_id;
        debug.position = candidate.position;
        debug.yaw = candidate.yaw;
        debug.accepted = true;
        debug.reachable = candidate.reachable;
        debug.selected = false;
        debug.viewpoint_case = "complete_candidate";
        debug.status = candidate.reachable ? "reachable" : "candidate";
        debug.score = candidate.score;
        debug.information_gain = candidate.information_gain;
        debug.travel_cost = candidate.travel_cost;
        debug.yaw_cost = candidate.yaw_cost;
        debug.curvature_cost = candidate.curvature_cost;
        debug.unknown_risk = candidate.unknown_risk;
        latest_debug_info_.viewpoints.push_back(debug);
    }

    if (candidates.empty()) {
        if (checkFinishCondition(stamp)) {
            exploration_finished_ = true;
            goal.reason = "complete exploration finished";
            latest_debug_info_.exploration_finished = true;
        } else {
            goal.reason = "no reachable complete exploration candidate";
        }
        latest_debug_info_.reason = goal.reason;
        latest_debug_info_.planning_success = false;
        latest_reason_ = goal.reason;
        return false;
    }

    if (cfg_.use_fuel_style_tour) {
        if (!selectFuelStyleGoal(robot_state, current_yaw, frontiers, candidates, stamp, goal)) {
            latest_debug_info_.reason = goal.reason;
            latest_debug_info_.planning_success = false;
            return false;
        }
        latest_debug_info_.planning_success = true;
        latest_debug_info_.reason = goal.reason;
        latest_debug_info_.selected_goal.valid = goal.valid;
        latest_debug_info_.selected_goal.position = goal.position;
        latest_debug_info_.selected_goal.yaw = goal.yaw;
        latest_debug_info_.selected_goal.frontier_cluster_id = goal.frontier_cluster_id;
        latest_debug_info_.selected_goal.viewpoint_case = goal.viewpoint_case;
        latest_debug_info_.selected_goal.score = goal.score;
        latest_debug_info_.selected_goal.information_gain = goal.information_gain;
        latest_debug_info_.selected_goal.travel_cost = goal.travel_cost;
        latest_debug_info_.selected_goal.guide_path = goal.guide_path;
        finish_confirm_counter_ = 0;
        finish_confirm_start_time_ = -1.0;
        return true;
    }

    const auto best_it = std::min_element(candidates.begin(), candidates.end(),
                                          [](const CompleteExplorationViewpoint &lhs,
                                             const CompleteExplorationViewpoint &rhs) {
                                              if (std::abs(lhs.score - rhs.score) > 1.0e-9) {
                                                  return lhs.score < rhs.score;
                                              }
                                              return lhs.frontier_id < rhs.frontier_id;
                                          });
    if (best_it == candidates.end()) {
        goal.reason = "complete exploration candidate selection failed";
        latest_debug_info_.reason = goal.reason;
        latest_debug_info_.planning_success = false;
        latest_reason_ = goal.reason;
        return false;
    }

    fillGoalFromViewpoint(*best_it, robot_pos, goal);
    frontier_db_.markSelected(best_it->frontier_id, stamp);
    if (best_it->region_id > 0) {
        region_graph_.markVisited(best_it->region_id);
    }
    finish_confirm_counter_ = 0;
    finish_confirm_start_time_ = -1.0;
    latest_selected_frontier_id_ = best_it->frontier_id;
    latest_selected_region_id_ = best_it->region_id;
    latest_selected_score_ = best_it->score;
    latest_selected_gain_ = best_it->information_gain;
    latest_selected_travel_cost_ = best_it->travel_cost;
    latest_reason_ = goal.reason;
    latest_debug_info_.planning_success = true;
    latest_debug_info_.reason = goal.reason;
    latest_debug_info_.selected_goal.valid = goal.valid;
    latest_debug_info_.selected_goal.position = goal.position;
    latest_debug_info_.selected_goal.yaw = goal.yaw;
    latest_debug_info_.selected_goal.frontier_cluster_id = goal.frontier_cluster_id;
    latest_debug_info_.selected_goal.viewpoint_case = goal.viewpoint_case;
    latest_debug_info_.selected_goal.score = goal.score;
    latest_debug_info_.selected_goal.information_gain = goal.information_gain;
    latest_debug_info_.selected_goal.travel_cost = goal.travel_cost;
    latest_debug_info_.selected_goal.guide_path = goal.guide_path;
    return true;
}

void CompleteExplorationFrontend::onGoalResult(const ExplorationGoal &goal,
                                               const super_utils::RET_CODE ret,
                                               const bool reached) {
    if (goal.frontier_cluster_id <= 0) {
        return;
    }
    const double stamp = now();
    if (reached) {
        frontier_db_.markCovered(goal.frontier_cluster_id, stamp);
        active_tour_.erase(std::remove(active_tour_.begin(),
                                       active_tour_.end(),
                                       goal.frontier_cluster_id),
                           active_tour_.end());
        return;
    }
    if (!committedRet(ret)) {
        frontier_db_.markFailed(goal.frontier_cluster_id,
                                ret >= 0 && ret < static_cast<int>(super_utils::RET_CODE_STR.size())
                                        ? super_utils::RET_CODE_STR[ret]
                                        : std::string("planning_failed"),
                                stamp);
        active_tour_.erase(std::remove(active_tour_.begin(),
                                       active_tour_.end(),
                                       goal.frontier_cluster_id),
                           active_tour_.end());
    }
}

bool CompleteExplorationFrontend::isExplorationFinished() const {
    return exploration_finished_;
}

const ExplorationDebugInfo &CompleteExplorationFrontend::latestDebugInfo() const {
    return latest_debug_info_;
}

std::string CompleteExplorationFrontend::latestStatusString() const {
    std::ostringstream ss;
    ss << "active_frontiers=" << frontier_db_.activeCount()
       << ", reachable_frontiers=" << frontier_db_.reachableCount()
       << ", covered_frontiers=" << frontier_db_.coveredCount()
       << ", unreachable_frontiers=" << frontier_db_.unreachableCount()
       << ", dormant_frontiers=" << frontier_db_.dormantCount()
       << ", blacklisted_frontiers=" << frontier_db_.blacklistedCount()
       << ", regions=" << region_graph_.regions().size()
       << ", candidate_count=" << latest_candidate_count_
       << ", reachable_candidate_count=" << latest_reachable_candidate_count_
       << ", selected_frontier_id=" << latest_selected_frontier_id_
       << ", selected_region_id=" << latest_selected_region_id_
       << ", selected_score=" << latest_selected_score_
       << ", selected_gain=" << latest_selected_gain_
       << ", selected_travel_cost=" << latest_selected_travel_cost_
       << ", memory_free=" << latest_memory_known_free_
       << ", memory_occupied=" << latest_memory_occupied_
       << ", memory_unknown=" << latest_memory_unknown_
       << ", tour_size=" << latest_tour_size_
       << ", tour_used_lkh=" << latest_tour_used_lkh_
       << ", tour_cost=" << latest_tour_cost_
       << ", tour_reason=" << latest_tour_reason_
       << ", finish_confirm_counter=" << finish_confirm_counter_
       << ", reason=" << latest_reason_;
    return ss.str();
}

}  // namespace general_planner
