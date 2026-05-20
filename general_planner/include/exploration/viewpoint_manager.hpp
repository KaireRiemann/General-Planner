#pragma once

#include "exploration/epic_map_adapter.hpp"
#include "exploration/frontier_database.hpp"

namespace general_planner {
namespace exploration {

class TopoGraph;

class ViewpointManager {
public:
    using Ptr = std::shared_ptr<ViewpointManager>;

    struct Config {
        double min_distance{1.4};
        double max_distance{4.0};
        int radius_samples{3};
        int yaw_samples{16};
        int height_samples{3};
        double height_step{0.6};
        double safe_distance{0.45};
        double sensor_range{7.0};
        double horizontal_fov_deg{90.0};
        double vertical_fov_deg{60.0};
        double normal_dot_min{0.25};
        int max_cells_per_gain_eval{260};
        double line_of_sight_step{0.20};
        double min_gain{3.0};
        bool use_local_map_safety{false};
        bool cluster_by_visibility_sphere{true};
        bool use_topo_reachability_filter{true};
        int max_viewpoint_clusters{8};
        double viewpoint_cluster_connectivity_scale{1.0};
        double topo_reachability_timeout{0.03};
        int epic_yaw_bins{8};
    };

    explicit ViewpointManager(Config cfg);

    bool generateBestViewpoints(const FrontierRecord &frontier,
                                const ObservationMap &observation_map,
                                const MapManager::Ptr &map_manager,
                                const EpicMapAdapter::Ptr &map_adapter,
                                const TopoGraph *topo_graph,
                                const super_utils::Vec3f &robot_pos,
                                double current_yaw,
                                double stamp,
                                std::vector<ExplorationViewpoint> &viewpoints,
                                ExplorationViewpoint &best_viewpoint) const;

private:
    bool viewpointSafe(const super_utils::Vec3f &position,
                       const ObservationMap &observation_map,
                       const MapManager::Ptr &map_manager,
                       const EpicMapAdapter::Ptr &map_adapter,
                       bool &local_safe,
                       double &surface_distance) const;

    bool topoReachable(const TopoGraph *topo_graph,
                       const super_utils::Vec3f &position,
                       double &topo_cost) const;

    double evaluateGain(const FrontierRecord &frontier,
                        const ExplorationViewpoint &candidate,
                        const ObservationMap &observation_map) const;

    double bestYawForViewpoint(const FrontierRecord &frontier,
                               const super_utils::Vec3f &position,
                               const ObservationMap &observation_map,
                               double &best_gain) const;

private:
    Config cfg_;
};

}  // namespace exploration
}  // namespace general_planner
