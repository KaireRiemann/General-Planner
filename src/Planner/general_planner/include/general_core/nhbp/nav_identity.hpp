#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

#include <general_utils/type_utils.hpp>

namespace general_planner::nhbp {

struct NavIdentity {
    int candidate_id{-1};
    int frontier_id{-1};
    int topo_node_id{-1};
    int topo_edge_id{-1};
    int tour_rank{-1};
    std::string intent_mode{"exploration"};
    std::string candidate_key;
    std::string frontier_key;
    std::string goal_key;
    std::string branch_key;
    std::string guide_path_key;
    std::string tour_key;
    bool recovery_intent{false};

    bool empty() const
    {
        return candidate_id < 0 &&
               frontier_id < 0 &&
               topo_node_id < 0 &&
               topo_edge_id < 0 &&
               candidate_key.empty() &&
               frontier_key.empty() &&
               goal_key.empty() &&
               branch_key.empty() &&
               guide_path_key.empty() &&
               tour_key.empty();
    }

    std::string canonicalKey() const
    {
        if (!goal_key.empty()) {
            return goal_key;
        }
        if (!candidate_key.empty()) {
            return candidate_key;
        }
        if (!branch_key.empty()) {
            return branch_key;
        }
        if (candidate_id >= 0) {
            return std::string("candidate:") + std::to_string(candidate_id);
        }
        if (topo_node_id >= 0) {
            return std::string("topo_node:") + std::to_string(topo_node_id);
        }
        return {};
    }

    std::string frontierIdentityKey() const
    {
        if (!frontier_key.empty()) {
            return frontier_key;
        }
        if (frontier_id >= 0) {
            return std::string("frontier:") + std::to_string(frontier_id);
        }
        return {};
    }

    std::string guideIdentityKey() const
    {
        return guide_path_key;
    }

    std::string blacklistKey() const
    {
        const std::string key = canonicalKey();
        if (!key.empty()) {
            return key;
        }
        return frontierIdentityKey();
    }
};

inline std::string quantizedPositionKey(const general_utils::Vec3f &position,
                                        const double resolution,
                                        const std::string &prefix)
{
    if (!position.allFinite()) {
        return {};
    }
    const double bucket = std::max(1.0e-3, resolution);
    std::ostringstream oss;
    if (!prefix.empty()) {
        oss << prefix << ":";
    }
    oss << static_cast<int>(std::floor(position.x() / bucket)) << ":"
        << static_cast<int>(std::floor(position.y() / bucket)) << ":"
        << static_cast<int>(std::floor(position.z() / bucket));
    return oss.str();
}

inline std::uint64_t fnv1a64(const std::string &text)
{
    std::uint64_t hash = 1469598103934665603ULL;
    for (const char ch : text) {
        hash ^= static_cast<unsigned char>(ch);
        hash *= 1099511628211ULL;
    }
    return hash;
}

inline std::string makeGuidePathKey(const general_utils::vec_E<general_utils::Vec3f> &path,
                                    const double resolution)
{
    if (path.empty()) {
        return {};
    }
    std::ostringstream oss;
    const double bucket = std::max(1.0e-3, resolution);
    for (const general_utils::Vec3f &point : path) {
        if (!point.allFinite()) {
            continue;
        }
        oss << static_cast<int>(std::floor(point.x() / bucket)) << ":"
            << static_cast<int>(std::floor(point.y() / bucket)) << ":"
            << static_cast<int>(std::floor(point.z() / bucket)) << ";";
    }
    const std::string encoded = oss.str();
    if (encoded.empty()) {
        return {};
    }
    std::ostringstream key;
    key << "guide:" << std::hex << fnv1a64(encoded);
    return key.str();
}

inline bool sameGoalIntent(const NavIdentity &lhs, const NavIdentity &rhs)
{
    const std::string lhs_goal = lhs.canonicalKey();
    const std::string rhs_goal = rhs.canonicalKey();
    if (!lhs_goal.empty() && !rhs_goal.empty()) {
        return lhs_goal == rhs_goal;
    }
    const std::string lhs_frontier = lhs.frontierIdentityKey();
    const std::string rhs_frontier = rhs.frontierIdentityKey();
    return !lhs_frontier.empty() && lhs_frontier == rhs_frontier;
}

} // namespace general_planner::nhbp
