#pragma once

#include <memory>
#include <string>

#include <general_core/map_manager.hpp>
#include <path_search/astar.h>
#include <super_utils/type_utils.hpp>

namespace general_planner {

struct ExplorationGoal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    double score{0.0};
    double information_gain{0.0};
    double travel_cost{0.0};
    double yaw_cost{0.0};
    double curvature_cost{0.0};
    double distance_to_robot{0.0};

    std::string reason;
    super_utils::vec_E<super_utils::Vec3f> guide_path;
};

class ExplorationFrontend {
public:
    using Ptr = std::shared_ptr<ExplorationFrontend>;

    struct Config {
        bool enable{false};
        bool print_log{false};

        double map_resolution{0.2};
        double frontier_search_radius{12.0};
        double frontier_cluster_radius{0.8};
        int min_frontier_cluster_size{5};

        double viewpoint_min_distance{1.2};
        double viewpoint_max_distance{4.0};
        double viewpoint_height_offset{0.0};
        double viewpoint_safe_distance{0.45};

        int viewpoint_yaw_sample_num{16};
        int viewpoint_radius_sample_num{3};
        int max_candidate_num{128};

        double weight_travel{1.0};
        double weight_yaw{0.5};
        double weight_curvature{0.8};
        double weight_info_gain{-2.0};
        double weight_unknown_risk{1.0};

        double min_information_gain{3.0};
        double goal_switch_min_score_improvement{0.25};
        double goal_reached_distance{0.5};

        bool unknown_as_occupied_for_motion{true};
        bool require_line_free_to_frontier{false};
        bool use_astar_cost{true};
    };

    ExplorationFrontend(const Config &cfg,
                        const MapManager::Ptr &map_manager,
                        const path_search::Astar::Ptr &astar);

    bool planNextGoal(const super_utils::StatePVAJ &robot_state,
                      double current_yaw,
                      ExplorationGoal &goal);

    bool isExplorationFinished() const;

    void reset();

private:
    struct FrontierCell {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        super_utils::Vec3f position{super_utils::Vec3f::Zero()};
        super_utils::Vec3i index{super_utils::Vec3i::Zero()};
    };

    struct FrontierCluster {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        super_utils::vec_E<FrontierCell> cells;
        super_utils::Vec3f center{super_utils::Vec3f::Zero()};
        super_utils::Vec3f unknown_direction{super_utils::Vec3f::UnitX()};
        super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
        super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};
        int size{0};
    };

    struct FrontierSearchStats {
        int searched_cells{0};
        int known_free_cells{0};
        int unknown_cells{0};
        int occupied_cells{0};
        int frontier_cells{0};
    };

    bool collectFrontierCells(const super_utils::Vec3f &robot_pos,
                              super_utils::vec_E<FrontierCell> &frontier_cells,
                              FrontierSearchStats &stats) const;

    bool isFrontierCell(const FrontierCell &cell,
                        rog_map::GridType grid_type) const;

    bool mapObservationReady(const FrontierSearchStats &stats) const;

    void clusterFrontiers(const super_utils::vec_E<FrontierCell> &frontier_cells,
                          super_utils::vec_E<FrontierCluster> &clusters) const;

    void sampleViewpointsForCluster(const FrontierCluster &cluster,
                                    const super_utils::StatePVAJ &robot_state,
                                    double current_yaw,
                                    super_utils::vec_E<ExplorationGoal> &candidates) const;

    bool viewpointSafe(const super_utils::Vec3f &viewpoint) const;

    bool viewpointVisible(const super_utils::Vec3f &viewpoint,
                          const FrontierCluster &cluster) const;

    double estimateInformationGain(const super_utils::Vec3f &viewpoint,
                                   const FrontierCluster &cluster) const;

    double estimateTravelCost(const super_utils::Vec3f &robot_pos,
                              const super_utils::Vec3f &viewpoint,
                              super_utils::vec_E<super_utils::Vec3f> &guide_path) const;

    double estimateYawCost(double current_yaw,
                           double candidate_yaw) const;

    double estimateCurvatureCost(const super_utils::StatePVAJ &robot_state,
                                 const super_utils::Vec3f &viewpoint,
                                 const FrontierCluster &cluster) const;

    double estimateUnknownRisk(const super_utils::Vec3f &viewpoint) const;

    double scoreCandidate(const ExplorationGoal &candidate,
                          double unknown_risk) const;

    bool isUnknownLike(rog_map::GridType type) const;

    bool isFreeLike(rog_map::GridType type) const;

    static double wrapAngleDiff(double lhs, double rhs);

    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    bool exploration_finished_{false};
};

}  // namespace general_planner
