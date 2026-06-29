#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

#include <general_utils/type_utils.hpp>

namespace general_planner::nhbp {

enum class TopoEdgeStatus {
    UNKNOWN,
    VALID,
    SUSPECT,
    BLOCKED,
    STALE
};

struct TopoNode {
    int node_id{-1};
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    double created_time{0.0};
    double last_seen_time{0.0};
    int visit_count{0};
    int failure_count{0};
    double blacklist_until{0.0};
};

struct TopoEdge {
    int edge_id{-1};
    int from_node{-1};
    int to_node{-1};
    double length{0.0};
    TopoEdgeStatus status{TopoEdgeStatus::UNKNOWN};
    double last_validated_time{0.0};
    int failure_count{0};
    double blacklist_until{0.0};
};

class TopologicalMemory {
public:
    struct Config {
        bool enable{false};
        int max_nodes{256};
        int max_edges{512};
        double node_merge_radius{1.0};
        double node_blacklist_ttl{12.0};
        double edge_blacklist_ttl{12.0};
        double recovery_min_distance{1.0};
        double recovery_max_distance{8.0};
    };

    TopologicalMemory();
    explicit TopologicalMemory(Config config);

    void reset();

    int observePose(const general_utils::Vec3f &position, double stamp);
    void observeTransition(const general_utils::Vec3f &from,
                           const general_utils::Vec3f &to,
                           double stamp);
    void recordFailureNear(const general_utils::Vec3f &position, double stamp);

    bool findRecoveryPosition(const general_utils::Vec3f &robot_pos,
                              double stamp,
                              general_utils::Vec3f &position) const;

    int activeNodeCount(double stamp) const;
    int blockedNodeCount(double stamp) const;
    int edgeCount() const;

private:
    int nearestNode(const general_utils::Vec3f &position,
                    double max_distance,
                    double stamp,
                    bool allow_blocked) const;
    std::string edgeKey(int from_node, int to_node) const;
    bool edgeUsable(const TopoEdge &edge, double stamp) const;
    void prune(double stamp);

    Config config_;
    std::unordered_map<int, TopoNode> nodes_;
    std::unordered_map<std::string, TopoEdge> edges_;
    int next_node_id_{0};
    int next_edge_id_{0};
};

const char *toString(TopoEdgeStatus status);

} // namespace general_planner::nhbp
