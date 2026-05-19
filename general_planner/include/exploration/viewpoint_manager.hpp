#pragma once

#include "exploration/frontier_database.hpp"

namespace general_planner {
namespace exploration {

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
    };

    explicit ViewpointManager(Config cfg);

    bool generateBestViewpoints(const FrontierRecord &frontier,
                                const ObservationMap &observation_map,
                                const MapManager::Ptr &map_manager,
                                const super_utils::Vec3f &robot_pos,
                                double current_yaw,
                                double stamp,
                                std::vector<ExplorationViewpoint> &viewpoints,
                                ExplorationViewpoint &best_viewpoint) const;

private:
    bool viewpointSafe(const super_utils::Vec3f &position,
                       const ObservationMap &observation_map,
                       const MapManager::Ptr &map_manager,
                       bool &local_safe,
                       double &surface_distance) const;

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
