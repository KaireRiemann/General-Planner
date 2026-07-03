#include <general_core/nhbp/far_goal_reasoner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace general_planner::nhbp {

FarGoalReasoner::FarGoalReasoner()
    : FarGoalReasoner(Config{})
{
}

FarGoalReasoner::FarGoalReasoner(Config config)
    : config_(config)
{
}

void FarGoalReasoner::configure(Config config)
{
    config_ = config;
}

FarGoalDecision FarGoalReasoner::selectSubgoal(
        const general_utils::Vec3f &robot_pos,
        const general_utils::Vec3f &far_goal,
        const double stamp,
        const SparseGlobalMap *sparse_map,
        const TopologicalMemory *topology) const
{
    FarGoalDecision decision;
    if (!config_.enable || !robot_pos.allFinite() || !far_goal.allFinite()) {
        decision.reason = "disabled_or_invalid_input";
        return decision;
    }

    const double goal_distance = (far_goal - robot_pos).norm();
    if (goal_distance <= std::max(0.0, config_.direct_goal_radius)) {
        decision.ready = true;
        decision.type = FarGoalDecisionType::DIRECT_GOAL;
        decision.local_goal = far_goal;
        decision.identity.intent_mode = "far_goal";
        decision.identity.goal_key = quantizedPositionKey(far_goal, 0.5, "far_direct");
        decision.identity.candidate_key = decision.identity.goal_key;
        decision.reason = "far_goal_inside_direct_radius";
        decision.score = goal_distance;
        return decision;
    }

    if (topology != nullptr) {
        TopoPath path;
        general_utils::Vec3f subgoal = general_utils::Vec3f::Zero();
        if (topology->searchPath(robot_pos, far_goal, stamp, path) &&
            topology->selectLocalSubgoalFromPath(path,
                                                 robot_pos,
                                                 std::max(0.1, config_.direct_goal_radius),
                                                 subgoal) &&
            subgoal.allFinite()) {
            decision.ready = true;
            decision.type = FarGoalDecisionType::TOPO_PATH_PREFIX;
            decision.local_goal = subgoal;
            decision.guide_path = path.positions;
            decision.identity.intent_mode = "far_goal";
            decision.identity.goal_key =
                    quantizedPositionKey(far_goal, 0.5, "far_topology_goal");
            decision.identity.candidate_key =
                    quantizedPositionKey(subgoal, 0.5, "topology_prefix");
            if (!path.node_ids.empty()) {
                decision.identity.topo_node_id = path.node_ids.back();
            }
            if (!path.edge_ids.empty()) {
                decision.identity.topo_edge_id = path.edge_ids.front();
            }
            decision.identity.guide_path_key = makeGuidePathKey(path.positions, 0.5);
            decision.reason = "topology_path_prefix";
            decision.score = (subgoal - robot_pos).norm() +
                             0.25 * (subgoal - far_goal).norm() +
                             0.1 * path.length;
            return decision;
        }
    }

    if (sparse_map != nullptr) {
        const auto frontiers =
                sparse_map->frontierRecords(robot_pos,
                                            std::max(0.0, config_.frontier_search_radius),
                                            stamp,
                                            std::max(1, config_.max_frontier_candidates));
        double best_score = std::numeric_limits<double>::infinity();
        for (const SparseCellRecord &frontier : frontiers) {
            const double travel_cost = (frontier.position - robot_pos).norm();
            const double goal_cost = (frontier.position - far_goal).norm();
            const double score = std::max(0.0, config_.travel_weight) * travel_cost +
                                 std::max(0.0, config_.goal_weight) * goal_cost -
                                 0.5 * std::clamp(frontier.confidence, 0.0, 1.0);
            if (score < best_score) {
                best_score = score;
                decision.ready = true;
                decision.type = FarGoalDecisionType::FRONTIER_TOWARD_GOAL;
                decision.local_goal = frontier.position;
                decision.identity.intent_mode = "far_goal";
                decision.identity.goal_key =
                        quantizedPositionKey(far_goal, 0.5, "far_goal");
                decision.identity.candidate_key =
                        quantizedPositionKey(frontier.position, 0.5, "sparse_frontier");
                decision.identity.frontier_key = decision.identity.candidate_key;
                decision.reason = "frontier_toward_far_goal";
                decision.score = score;
            }
        }
        if (decision.ready) {
            return decision;
        }
    }

    if (topology != nullptr) {
        TopoPath path;
        general_utils::Vec3f recovery = general_utils::Vec3f::Zero();
        const bool has_path = topology->findRecoveryPath(robot_pos, stamp, path);
        if (has_path && path.valid && !path.positions.empty()) {
            recovery = path.positions.back();
        }
        if ((has_path && path.valid) ||
            topology->findRecoveryPosition(robot_pos, stamp, recovery)) {
            decision.ready = true;
            decision.type = FarGoalDecisionType::TOPOLOGY_RECOVERY;
            decision.local_goal = recovery;
            decision.identity.intent_mode = "far_goal_recovery";
            decision.identity.recovery_intent = true;
            decision.identity.goal_key =
                    quantizedPositionKey(recovery, 0.5, "far_topology_recovery");
            decision.identity.candidate_key = decision.identity.goal_key;
            if (has_path && path.valid) {
                decision.guide_path = path.positions;
                decision.identity.guide_path_key = makeGuidePathKey(path.positions, 0.5);
            }
            decision.reason = has_path && path.valid
                                      ? "topology_recovery_path_fallback"
                                      : "topology_recovery_fallback";
            decision.score = (recovery - robot_pos).norm() +
                             0.5 * (recovery - far_goal).norm();
            return decision;
        }
    }

    decision.reason = "no_frontier_or_topology_subgoal";
    return decision;
}

const char *toString(const FarGoalDecisionType type)
{
    switch (type) {
        case FarGoalDecisionType::DIRECT_GOAL:
            return "DIRECT_GOAL";
        case FarGoalDecisionType::TOPO_PATH_PREFIX:
            return "TOPO_PATH_PREFIX";
        case FarGoalDecisionType::FRONTIER_TOWARD_GOAL:
            return "FRONTIER_TOWARD_GOAL";
        case FarGoalDecisionType::TOPOLOGY_RECOVERY:
            return "TOPOLOGY_RECOVERY";
        case FarGoalDecisionType::UNAVAILABLE:
            return "UNAVAILABLE";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
