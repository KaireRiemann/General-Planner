#pragma once

#include <memory>
#include <vector>

#include <general_core/exploration_coverage_planner.hpp>
#include <path_search/astar.h>

namespace general_planner {

class ExplorationViewpointPlanner {
public:
    using Ptr = std::shared_ptr<ExplorationViewpointPlanner>;

    struct Config {
        double min_distance{1.2};
        double max_distance{4.0};
        double min_robot_distance{0.7};
        double height_offset{0.0};
        double safe_distance{0.45};

        int yaw_sample_num{16};
        int radius_sample_num{3};
        int max_viewpoints_per_frontier{8};
        int max_total_candidates{256};

        bool require_line_free_to_frontier{false};
        bool unknown_as_occupied_for_motion{true};
        bool use_astar_cost{true};

        double min_information_gain{3.0};

        double weight_travel{1.0};
        double weight_yaw{0.5};
        double weight_curvature{0.8};
        double weight_info_gain{-2.0};
        double weight_unknown_risk{1.0};
        double weight_coverage_order{2.0};
        double weight_revisit{2.0};
        double weight_fail{1.5};
    };

    ExplorationViewpointPlanner(const Config &cfg,
                                const MapManager::Ptr &map_manager,
                                const path_search::Astar::Ptr &astar);

    bool sampleAndScore(const CompleteFrontierCluster &frontier,
                        const ExplorationRegionGraph &region_graph,
                        const CoveragePathPlanner &coverage_planner,
                        const super_utils::StatePVAJ &robot_state,
                        double current_yaw,
                        std::vector<CompleteExplorationViewpoint> &out) const;

private:
    bool viewpointSafe(const super_utils::Vec3f &pos) const;
    int countVisibleCells(const super_utils::Vec3f &viewpoint,
                          double yaw,
                          const CompleteFrontierCluster &frontier) const;
    double estimateTravelCost(const super_utils::Vec3f &robot_pos,
                              const super_utils::Vec3f &viewpoint,
                              super_utils::vec_E<super_utils::Vec3f> &guide_path) const;
    double estimateYawCost(double current_yaw, double candidate_yaw) const;
    double estimateCurvatureCost(const super_utils::StatePVAJ &robot_state,
                                 const super_utils::Vec3f &viewpoint) const;
    double estimateUnknownRisk(const super_utils::Vec3f &viewpoint) const;
    bool insideYawFov(const super_utils::Vec3f &viewpoint,
                      double yaw,
                      const super_utils::Vec3f &target) const;
    static double wrapAngleDiff(double lhs, double rhs);

    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

}  // namespace general_planner
