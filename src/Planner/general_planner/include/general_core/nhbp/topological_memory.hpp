#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <general_utils/type_utils.hpp>

namespace general_planner::nhbp {

enum class TopoEdgeStatus {
    UNKNOWN,
    VALID,
    SUSPECT,
    BLOCKED,
    STALE
};

enum class TopoNodeType {
    POSE,
    BRANCH,
    FRONTIER,
    FAILURE,
    VERTICAL_CONNECTOR
};

struct TopoNode {
    int node_id{-1};
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    TopoNodeType node_type{TopoNodeType::POSE};
    int floor_id{0};
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

struct TopoPath {
    bool valid{false};
    std::vector<int> node_ids;
    std::vector<int> edge_ids;
    general_utils::vec_E<general_utils::Vec3f> positions;
    double length{0.0};
    std::string reason;
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

    int observePose(const general_utils::Vec3f &position,
                    double stamp,
                    TopoNodeType node_type = TopoNodeType::POSE,
                    int floor_id = 0);
    int observeVerticalConnector(const general_utils::Vec3f &position,
                                 double stamp,
                                 int floor_id);
    void observeTransition(const general_utils::Vec3f &from,
                           const general_utils::Vec3f &to,
                           double stamp);
    void recordFailureNear(const general_utils::Vec3f &position, double stamp);

    bool findRecoveryPosition(const general_utils::Vec3f &robot_pos,
                              double stamp,
                              general_utils::Vec3f &position,
                              const std::function<bool(const general_utils::Vec3f &)> &accept =
                                      std::function<bool(const general_utils::Vec3f &)>{}) const;
    bool findRecoveryPath(const general_utils::Vec3f &robot_pos,
                          double stamp,
                          TopoPath &path,
                          const std::function<bool(const general_utils::Vec3f &)> &accept =
                                  std::function<bool(const general_utils::Vec3f &)>{}) const;
    bool searchPath(const general_utils::Vec3f &start,
                    const general_utils::Vec3f &goal,
                    double stamp,
                    TopoPath &path) const;
    bool selectLocalSubgoalFromPath(const TopoPath &path,
                                    const general_utils::Vec3f &robot_pos,
                                    double local_radius,
                                    general_utils::Vec3f &subgoal) const;

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
const char *toString(TopoNodeType type);

} // namespace general_planner::nhbp
