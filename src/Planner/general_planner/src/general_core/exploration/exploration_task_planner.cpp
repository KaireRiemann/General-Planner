#include <general_core/exploration/exploration_task_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace general_planner {
namespace {

double clampPositiveFinite(const double value, const double fallback = 0.0)
{
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::max(0.0, value);
}

bool sameGoalRegion(const ExplorationGoal &lhs,
                    const ExplorationGoal &rhs,
                    const double radius)
{
    if (!lhs.position.allFinite() || !rhs.position.allFinite()) {
        return false;
    }
    return (lhs.position - rhs.position).norm() <= std::max(0.0, radius);
}

} // namespace

ExplorationTaskPlanner::ExplorationTaskPlanner(const Config &cfg)
        : cfg_(cfg)
{
}

void ExplorationTaskPlanner::reset()
{
}

bool ExplorationTaskPlanner::plan(const Request &request, Plan &plan_out) const
{
    plan_out = Plan{};
    if (!cfg_.exploration_task_planner_enable ||
        request.frontier_objects == nullptr ||
        request.frontier_objects->empty()) {
        plan_out.reason = "task_planner_disabled_or_empty_frontier_db";
        return false;
    }

    plan_out.raw_candidate_count =
            request.candidate_set != nullptr
                    ? static_cast<int>(request.candidate_set->candidates.size())
                    : 0;
    std::unordered_set<std::string> sectors;
    std::vector<FrontierObject> objects;
    objects.reserve(request.frontier_objects->size());
    for (const FrontierObject &input_object : *request.frontier_objects) {
        if (input_object.key.empty() || input_object.viewpoints.empty()) {
            continue;
        }
        FrontierObject object = input_object;
        if (!object.sector_key.empty()) {
            sectors.insert(object.sector_key);
        }
        std::sort(object.viewpoints.begin(),
                  object.viewpoints.end(),
                  [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                      if (lhs.score != rhs.score) {
                          return lhs.score < rhs.score;
                      }
                      return lhs.information_gain > rhs.information_gain;
                  });
        const int max_viewpoints =
                std::max(1, cfg_.exploration_task_planner_viewpoints_per_frontier);
        if (static_cast<int>(object.viewpoints.size()) > max_viewpoints) {
            object.viewpoints.resize(static_cast<size_t>(max_viewpoints));
        }
        objects.push_back(std::move(object));
    }

    if (objects.empty()) {
        plan_out.reason = "task_planner_no_active_frontier_objects";
        return false;
    }

    std::sort(objects.begin(),
              objects.end(),
              [this](const FrontierObject &lhs, const FrontierObject &rhs) {
                  const double lhs_score = objectScore(lhs);
                  const double rhs_score = objectScore(rhs);
                  if (lhs_score != rhs_score) {
                      return lhs_score < rhs_score;
                  }
                  return lhs.total_gain > rhs.total_gain;
              });

    const int max_objects =
            std::min({static_cast<int>(objects.size()),
                      std::max(1, cfg_.exploration_task_planner_max_frontier_objects),
                      std::max(1, cfg_.exploration_atsp_max_candidate_num)});
    objects.resize(static_cast<size_t>(max_objects));

    std::vector<int> ordered_object_indices;
    std::string solver_reason;
    solveObjectTour(request, objects, ordered_object_indices, solver_reason);
    if (ordered_object_indices.empty()) {
        ordered_object_indices.resize(objects.size());
        for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
            ordered_object_indices[static_cast<size_t>(i)] = i;
        }
        solver_reason = solver_reason.empty() ? "object_score_order"
                                              : solver_reason + "_object_score_order";
    }

    std::unordered_set<std::string> emitted_frontiers;
    ExplorationGoal previous_goal;
    bool has_previous_goal = false;
    for (const int object_index : ordered_object_indices) {
        if (object_index < 0 || object_index >= static_cast<int>(objects.size())) {
            continue;
        }
        const FrontierObject &object = objects[static_cast<size_t>(object_index)];
        if (object.viewpoints.empty()) {
            continue;
        }
        if (emitted_frontiers.find(object.key) != emitted_frontiers.end()) {
            continue;
        }
        ExplorationGoal goal =
                refineViewpointForObject(object,
                                         has_previous_goal ? &previous_goal : nullptr,
                                         request,
                                         static_cast<int>(plan_out.ordered_goals.size()));
        goal.identity.frontier_key =
                goal.identity.frontier_key.empty() ? object.key : goal.identity.frontier_key;
        goal.reason += " task_planner_frontier_db_object candidates=" +
                       std::to_string(object.candidate_count) +
                       " sector=" + object.sector_key;
        plan_out.ordered_goals.push_back(goal);
        previous_goal = goal;
        has_previous_goal = true;
        emitted_frontiers.insert(object.key);
    }

    plan_out.valid = !plan_out.ordered_goals.empty();
    plan_out.frontier_object_count = static_cast<int>(objects.size());
    plan_out.sector_count = static_cast<int>(sectors.size());
    plan_out.representative_count = static_cast<int>(plan_out.ordered_goals.size());
    std::ostringstream reason;
    reason << "task_planner:frontier_db:" << solver_reason
           << ",raw=" << plan_out.raw_candidate_count
           << ",objects=" << plan_out.frontier_object_count
           << ",sectors=" << plan_out.sector_count
           << ",tour=" << plan_out.representative_count;
    plan_out.reason = reason.str();
    return plan_out.valid;
}

double ExplorationTaskPlanner::objectScore(const FrontierObject &object) const
{
    const double saturation =
            std::max(1.0, cfg_.exploration_information_gain_saturation);
    const double gain_bonus =
            std::max(0.0, cfg_.exploration_task_planner_gain_bonus_weight) *
            std::min(object.total_gain, saturation);
    const double breadth_bonus =
            std::max(0.0, cfg_.exploration_task_planner_breadth_bonus_weight) *
            std::log1p(static_cast<double>(std::max(0, object.candidate_count)));
    const double coverage_intent_bonus =
            cfg_.exploration_coverage_intent_enable
                    ? std::max(0.0, cfg_.exploration_coverage_intent_weight) *
                              std::max(0.0, object.coverage_intent)
                    : 0.0;
    const double expansion_penalty =
            object.expansion_only
                    ? std::max(0.0, cfg_.exploration_task_planner_expansion_penalty)
                    : 0.0;
    return object.best_score - gain_bonus - breadth_bonus -
           coverage_intent_bonus + expansion_penalty;
}

ExplorationGoal ExplorationTaskPlanner::representativeGoal(
        const FrontierObject &object) const
{
    if (object.viewpoints.empty()) {
        return ExplorationGoal{};
    }
    ExplorationGoal goal = object.viewpoints.front();
    goal.history_score_delta += objectScore(object) - goal.score;
    goal.score = objectScore(object);
    goal.information_gain =
            std::max(goal.information_gain,
                     std::min(object.total_gain,
                              std::max(1.0, cfg_.exploration_information_gain_saturation)));
    return goal;
}

ExplorationGoal ExplorationTaskPlanner::refineViewpointForObject(
        const FrontierObject &object,
        const ExplorationGoal *previous_goal,
        const Request &request,
        const int tour_rank) const
{
    if (object.viewpoints.empty()) {
        return ExplorationGoal{};
    }
    const int local_refine_frontiers =
            std::max(0, cfg_.exploration_task_planner_local_refine_frontiers);
    const int max_options =
            tour_rank < local_refine_frontiers
                    ? std::max(1, cfg_.exploration_task_planner_viewpoints_per_frontier)
                    : 1;
    const int option_count =
            std::min(static_cast<int>(object.viewpoints.size()), max_options);

    int best_index = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < option_count; ++i) {
        const ExplorationGoal &candidate =
                object.viewpoints[static_cast<size_t>(i)];
        double transition_cost = 0.0;
        if (previous_goal != nullptr && previous_goal->valid && request.pairwise_cost) {
            transition_cost = request.pairwise_cost(*previous_goal, candidate);
        } else if (request.start_cost) {
            transition_cost = request.start_cost(candidate);
        } else if (request.robot_pos.allFinite() && candidate.position.allFinite()) {
            transition_cost = (candidate.position - request.robot_pos).norm();
        }
        const double total_cost =
                clampPositiveFinite(transition_cost) +
                (std::isfinite(candidate.score) ? candidate.score : 0.0);
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_index = i;
        }
    }

    ExplorationGoal goal = object.viewpoints[static_cast<size_t>(best_index)];
    goal.history_score_delta += objectScore(object) - goal.score;
    goal.score = objectScore(object);
    goal.information_gain =
            std::max(goal.information_gain,
                     std::min(object.total_gain,
                              std::max(1.0, cfg_.exploration_information_gain_saturation)));
    goal.reason += " local_viewpoint_refine options=" + std::to_string(option_count) +
                   ",choice=" + std::to_string(best_index) +
                   ",coverage_intent=" + std::to_string(object.coverage_intent);
    return goal;
}

void ExplorationTaskPlanner::solveObjectTour(
        const Request &request,
        const std::vector<FrontierObject> &objects,
        std::vector<int> &ordered_indices,
        std::string &reason) const
{
    ordered_indices.clear();
    reason.clear();
    if (objects.empty()) {
        reason = "empty_object_graph";
        return;
    }
    if (objects.size() == 1U) {
        ordered_indices.push_back(0);
        reason = "single_object";
        return;
    }

    if (!cfg_.exploration_use_atsp) {
        ordered_indices.resize(objects.size());
        for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
            ordered_indices[static_cast<size_t>(i)] = i;
        }
        reason = "atsp_disabled_object_score";
        return;
    }

    exploration::ATSPProblem problem;
    problem.depot_index = 0;
    problem.time_budget_ms = cfg_.exploration_atsp_time_budget_ms;
    problem.candidates.reserve(objects.size());
    problem.node_reward.reserve(objects.size());
    problem.directed_cost_matrix =
            Eigen::MatrixXd::Zero(static_cast<int>(objects.size()) + 1,
                                  static_cast<int>(objects.size()) + 1);

    const double reward_weight =
            std::max(0.0, cfg_.exploration_task_planner_gain_bonus_weight);
    const double reward_saturation =
            std::max(1.0, cfg_.exploration_information_gain_saturation);
    for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
        const FrontierObject &object = objects[static_cast<size_t>(i)];
        const ExplorationGoal goal = representativeGoal(object);
        problem.candidates.push_back(exploration::ATSPCandidate{i});
        problem.node_reward.push_back(
                reward_weight * std::min(object.total_gain, reward_saturation));
        const double start_cost =
                request.start_cost ? request.start_cost(goal)
                                   : (request.robot_pos.allFinite()
                                              ? (goal.position - request.robot_pos).norm()
                                              : clampPositiveFinite(goal.travel_cost));
        problem.directed_cost_matrix(0, i + 1) =
                clampPositiveFinite(start_cost) + std::max(0.0, objectScore(object));
        problem.directed_cost_matrix(i + 1, 0) = 0.0;
    }

    for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
        const ExplorationGoal from = representativeGoal(objects[static_cast<size_t>(i)]);
        for (int j = 0; j < static_cast<int>(objects.size()); ++j) {
            if (i == j) {
                problem.directed_cost_matrix(i + 1, j + 1) = 0.0;
                continue;
            }
            const ExplorationGoal to = representativeGoal(objects[static_cast<size_t>(j)]);
            const bool same_frontier =
                    !objects[static_cast<size_t>(i)].key.empty() &&
                    objects[static_cast<size_t>(i)].key ==
                            objects[static_cast<size_t>(j)].key;
            const bool same_region =
                    sameGoalRegion(from,
                                   to,
                                   std::max(0.75, cfg_.exploration_active_tour_match_radius));
            double cost =
                    request.pairwise_cost ? request.pairwise_cost(from, to)
                                          : (from.position - to.position).norm();
            if (same_frontier || same_region) {
                cost += 1000.0;
            }
            problem.directed_cost_matrix(i + 1, j + 1) =
                    clampPositiveFinite(cost, 1000.0);
        }
    }

    exploration::ATSPTourPlanner::Config atsp_cfg;
    atsp_cfg.enable = true;
    atsp_cfg.solver = cfg_.exploration_atsp_solver;
    atsp_cfg.work_dir = cfg_.exploration_atsp_work_dir;
    atsp_cfg.external_command = cfg_.exploration_atsp_external_command;
    atsp_cfg.cost_scale = cfg_.exploration_atsp_cost_scale;
    atsp_cfg.time_budget_ms = cfg_.exploration_atsp_time_budget_ms;
    atsp_cfg.max_candidate_num = cfg_.exploration_atsp_max_candidate_num;

    exploration::ATSPTourPlanner solver(atsp_cfg);
    const exploration::ATSPSolution solution = solver.solve(problem);
    std::vector<char> used(objects.size(), 0);
    for (const int object_index : solution.ordered_candidate_ids) {
        if (object_index < 0 ||
            object_index >= static_cast<int>(objects.size()) ||
            used[static_cast<size_t>(object_index)] != 0) {
            continue;
        }
        used[static_cast<size_t>(object_index)] = 1;
        ordered_indices.push_back(object_index);
    }

    for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
        if (used[static_cast<size_t>(i)] == 0) {
            ordered_indices.push_back(i);
        }
    }
    reason = "object_atsp:" + solution.solver_status +
             ",fallback=" + std::to_string(solution.fallback_used ? 1 : 0);
}

} // namespace general_planner
