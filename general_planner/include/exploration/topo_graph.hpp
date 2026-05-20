#pragma once

#include <unordered_map>
#include <unordered_set>

#include <path_search/astar.h>

#include "exploration/parallel_bubble_astar.hpp"
#include "exploration/viewpoint_manager.hpp"

namespace general_planner {
namespace exploration {

struct PairIntHash {
    std::size_t operator()(const std::pair<int, int> &p) const {
        const auto a = static_cast<std::uint64_t>(p.first);
        const auto b = static_cast<std::uint64_t>(p.second);
        return static_cast<std::size_t>((a << 32U) ^ (b + 0x9e3779b97f4a7c15ULL + (a << 6U) + (a >> 2U)));
    }
};

class TopoGraph {
public:
    using Ptr = std::shared_ptr<TopoGraph>;

    struct Config {
        double history_node_min_distance{1.0};
        double connect_radius{8.0};
        double local_edge_astar_timeout{0.05};
        double global_edge_max_length{14.0};
        int max_history_nodes{500};
        bool use_local_astar_for_edges{true};
        bool use_global_line_free_for_edges{true};
        double global_line_safe_distance{0.45};
        double global_line_step{0.25};
        bool use_parallel_bubble_astar_for_edges{false};
        double bubble_astar_resolution{0.5};
        double bubble_astar_safe_distance{0.45};
        int bubble_astar_max_nodes{8000};
    };

    TopoGraph(Config cfg,
              MapManager::Ptr map_manager,
              path_search::Astar::Ptr astar,
              std::shared_ptr<ObservationMap> observation_map);

    void updateOdomNode(const super_utils::Vec3f &pos, double yaw);
    void updateHistoryOdomNodes(const super_utils::Vec3f &pos, double yaw);
    void insertOrUpdateFrontierNodes(const std::vector<FrontierRecord> &frontiers);

    bool graphSearchToFrontier(int frontier_id, GlobalRoute &route) const;
    bool graphSearchBetweenFrontiers(int from_frontier_id,
                                     int to_frontier_id,
                                     double &cost) const;
    bool routeToPosition(const super_utils::Vec3f &position,
                         double &cost,
                         double timeout = 0.03) const;
    bool routeToPosition(const super_utils::Vec3f &position,
                         GlobalRoute &route,
                         double timeout = 0.03) const;

    bool getNode(int node_id, ExplorationTopoNode &node) const;
    void getGraph(std::vector<ExplorationTopoNode> &nodes,
                  std::vector<ExplorationTopoEdge> &edges) const;
    void markEdgeUnreachable(int from, int to);
    void removeInactiveFrontierNodes(const std::unordered_set<int> &active_frontier_ids);
    void reset();

    int nodeCount() const { return static_cast<int>(nodes_.size()); }
    int edgeCount() const;

private:
    bool tryBuildEdge(int from_id, int to_id, ExplorationTopoEdge &edge) const;
    bool tryBuildEdgeBetweenPositions(const super_utils::Vec3f &start,
                                      const super_utils::Vec3f &goal,
                                      super_utils::vec_E<super_utils::Vec3f> &path,
                                      double &cost) const;

    bool localAstarPath(const super_utils::Vec3f &start,
                        const super_utils::Vec3f &goal,
                        super_utils::vec_E<super_utils::Vec3f> &path,
                        double &cost) const;

    bool globalLineOrObservedFreePath(const super_utils::Vec3f &start,
                                      const super_utils::Vec3f &goal,
                                      super_utils::vec_E<super_utils::Vec3f> &path,
                                      double &cost) const;

    std::vector<int> dijkstra(int start_id, int goal_id, double &cost) const;
    void rebuildEdges();
    static std::pair<int, int> canonicalEdge(int a, int b);

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    std::shared_ptr<ObservationMap> observation_map_;
    std::unique_ptr<ParallelBubbleAstar> bubble_astar_;

    int next_node_id_{0};
    int odom_node_id_{-1};

    std::unordered_map<int, ExplorationTopoNode> nodes_;
    std::unordered_map<int, std::vector<ExplorationTopoEdge>> adjacency_;
    std::unordered_map<int, int> frontier_id_to_node_id_;
    std::vector<int> history_node_ids_;

    std::unordered_set<std::pair<int, int>, PairIntHash> unreachable_edges_;
};

}  // namespace exploration
}  // namespace general_planner
