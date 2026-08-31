#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <utils/header/type_utils.hpp>

namespace general_planner::state2state_task {

/**
 * Task-local consumer state for the persistent global topology graph.
 *
 * The graph remains owned and maintained by MapManager/GlobalMapRuntime.  This
 * object only retains the raw route returned by a global query and the user's
 * policy choice.  All non-atomic fields are accessed under GeneralPlanner's
 * state2state replan lock.
 */
struct GlobalTopologyRouteContext {
    bool valid{false};
    bool reaches_goal{false};
    std::uint64_t route_id{0};
    std::uint64_t task_epoch{0};
    std::uint64_t world_epoch{0};
    std::uint64_t map_revision_at_query{0};
    std::uint64_t topo_revision{0};
    general_utils::Vec3f goal{general_utils::Vec3f::Zero()};
    general_utils::vec_Vec3f raw_topology_route;
    std::vector<double> arc_length;
    double committed_route_s{0.0};
    double last_query_time{-std::numeric_limits<double>::infinity()};
    std::string last_result{"LOCAL_ONLY"};
};

struct State2StateTopologyRouteRuntime {
    // These two values are written by ROS callbacks without entering the
    // planner.  The frontend consumes the new generation at its next plan or
    // replan, which keeps callbacks free of planning and map work.
    std::atomic<bool> policy_enabled{false};
    std::atomic<std::uint64_t> policy_generation{0};
    std::atomic<std::uint64_t> task_epoch{0};
    std::atomic<std::uint64_t> task_generation{0};

    std::uint64_t consumed_policy_generation{0};
    std::uint64_t consumed_task_generation{0};
    GlobalTopologyRouteContext route;

    void setPolicy(const bool enabled) {
        const bool previous = policy_enabled.exchange(enabled,
                                                      std::memory_order_acq_rel);
        if (previous != enabled) {
            policy_generation.fetch_add(1, std::memory_order_acq_rel);
        }
    }

    void setTaskEpoch(const std::uint64_t epoch) {
        const std::uint64_t previous = task_epoch.exchange(
            epoch, std::memory_order_acq_rel);
        if (previous != epoch) {
            task_generation.fetch_add(1, std::memory_order_acq_rel);
        }
    }
};

// A global-topology selection is a routing policy, not merely a route hint:
// for non-local goals it may require the frontend to reject a route that
// cannot remain attached to the persistent topology graph.  Keeping this
// predicate independent makes the fallback boundary explicit and testable.
inline bool topologyRouteRequired(
        const bool query_capability_enabled,
        const bool topology_enabled,
        const State2StateTopologyRouteRuntime *runtime,
        const bool strict_route_enabled,
        const double goal_distance,
        const double min_query_distance) {
    return query_capability_enabled && topology_enabled &&
           strict_route_enabled && runtime != nullptr &&
           runtime->policy_enabled.load(std::memory_order_acquire) &&
           std::isfinite(goal_distance) &&
           goal_distance >= std::max(0.0, min_query_distance);
}

inline void resetGlobalTopologyRoute(GlobalTopologyRouteContext &route,
                                     const std::string &reason) {
    route.valid = false;
    route.reaches_goal = false;
    route.raw_topology_route.clear();
    route.arc_length.clear();
    route.committed_route_s = 0.0;
    route.last_result = reason;
}

inline void buildRouteArcLength(const general_utils::vec_Vec3f &path,
                                std::vector<double> &arc_length) {
    arc_length.assign(path.size(), 0.0);
    for (std::size_t i = 1; i < path.size(); ++i) {
        arc_length[i] = arc_length[i - 1] + (path[i] - path[i - 1]).norm();
    }
}

inline bool interpolateRouteAt(const general_utils::vec_Vec3f &path,
                               const std::vector<double> &arc_length,
                               const double s,
                               general_utils::Vec3f &point) {
    if (path.empty() || path.size() != arc_length.size() || !std::isfinite(s)) {
        return false;
    }
    if (s <= 0.0 || path.size() == 1) {
        point = path.front();
        return true;
    }
    if (s >= arc_length.back()) {
        point = path.back();
        return true;
    }
    const auto upper = std::upper_bound(arc_length.begin(), arc_length.end(), s);
    const std::size_t end = static_cast<std::size_t>(upper - arc_length.begin());
    if (end == 0 || end >= path.size()) {
        return false;
    }
    const std::size_t begin = end - 1;
    const double length = arc_length[end] - arc_length[begin];
    if (length <= 1.0e-9) {
        point = path[end];
        return true;
    }
    const double alpha = std::clamp((s - arc_length[begin]) / length, 0.0, 1.0);
    point = path[begin] + alpha * (path[end] - path[begin]);
    return point.allFinite();
}

inline bool projectRouteMonotonically(const general_utils::vec_Vec3f &path,
                                      const std::vector<double> &arc_length,
                                      const general_utils::Vec3f &point,
                                      const double min_s,
                                      double &projected_s,
                                      general_utils::Vec3f &projected_point) {
    if (path.size() < 2 || path.size() != arc_length.size() ||
        !point.allFinite()) {
        return false;
    }
    const double bounded_min_s = std::clamp(min_s, 0.0, arc_length.back());
    double best_sq = std::numeric_limits<double>::infinity();
    bool found = false;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const double segment_begin_s = arc_length[i - 1];
        const double segment_end_s = arc_length[i];
        if (segment_end_s + 1.0e-9 < bounded_min_s) {
            continue;
        }
        const general_utils::Vec3f delta = path[i] - path[i - 1];
        const double length_sq = delta.squaredNorm();
        if (length_sq <= 1.0e-12) {
            continue;
        }
        const double segment_length = std::sqrt(length_sq);
        const double min_alpha = std::clamp(
            (bounded_min_s - segment_begin_s) / segment_length, 0.0, 1.0);
        double alpha = (point - path[i - 1]).dot(delta) / length_sq;
        alpha = std::clamp(alpha, min_alpha, 1.0);
        const general_utils::Vec3f candidate = path[i - 1] + alpha * delta;
        const double sq = (candidate - point).squaredNorm();
        if (sq < best_sq) {
            best_sq = sq;
            projected_s = segment_begin_s + alpha * segment_length;
            projected_point = candidate;
            found = true;
        }
    }
    return found;
}

inline bool sliceRouteByArcLength(const general_utils::vec_Vec3f &path,
                                  const std::vector<double> &arc_length,
                                  const double begin_s,
                                  const double end_s,
                                  general_utils::vec_Vec3f &slice) {
    slice.clear();
    if (path.size() < 2 || path.size() != arc_length.size() ||
        !std::isfinite(begin_s) || !std::isfinite(end_s) || end_s < begin_s) {
        return false;
    }
    const double clamped_begin = std::clamp(begin_s, 0.0, arc_length.back());
    const double clamped_end = std::clamp(end_s, clamped_begin, arc_length.back());
    general_utils::Vec3f begin_point;
    general_utils::Vec3f end_point;
    if (!interpolateRouteAt(path, arc_length, clamped_begin, begin_point) ||
        !interpolateRouteAt(path, arc_length, clamped_end, end_point)) {
        return false;
    }
    slice.push_back(begin_point);
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        if (arc_length[i] > clamped_begin + 1.0e-9 &&
            arc_length[i] < clamped_end - 1.0e-9) {
            slice.push_back(path[i]);
        }
    }
    if ((slice.back() - end_point).norm() > 1.0e-6) {
        slice.push_back(end_point);
    }
    return slice.size() >= 2 || (end_point - begin_point).norm() <= 1.0e-6;
}

} // namespace general_planner::state2state_task
