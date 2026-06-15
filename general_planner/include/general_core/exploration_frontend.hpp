#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <exploration_manager/global_exploration_planner.hpp>
#include <general_core/exploration_debug.hpp>
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
    double unknown_risk{0.0};
    double open_space_score{0.0};
    double velocity_alignment_score{0.0};
    double high_speed_score{0.0};
    double lifecycle_score{0.0};
    int frontier_cluster_id{-1};
    int viewpoint_debug_id{-1};

    std::string reason;
    std::string viewpoint_case;
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
        double frontier_lifecycle_match_distance{1.2};
        int frontier_lifecycle_min_observations{1};
        int frontier_lifecycle_max_missing_frames{3};

        double viewpoint_min_distance{1.2};
        double viewpoint_max_distance{4.0};
        double viewpoint_height_offset{0.0};
        double viewpoint_safe_distance{0.45};
        double viewpoint_side_step{0.8};

        int viewpoint_yaw_sample_num{16};
        int viewpoint_radius_sample_num{3};
        int viewpoint_top_view_num{15};
        double viewpoint_max_decay{0.8};
        int viewpoint_min_visible_cells{5};
        int max_candidate_num{128};

        double weight_travel{1.0};
        double weight_yaw{0.5};
        double weight_curvature{0.8};
        double weight_info_gain{-2.0};
        double weight_unknown_risk{1.0};
        double weight_high_speed{-1.0};
        double weight_lifecycle{-0.1};

        double min_information_gain{3.0};
        double goal_switch_min_score_improvement{0.25};
        double goal_reached_distance{0.5};
        double high_speed_open_space_radius{2.0};
        double high_speed_min_speed{0.8};

        bool unknown_as_occupied_for_motion{true};
        bool require_line_free_to_frontier{false};
        bool use_astar_cost{true};

        double global_grid_cell_size{5.0};
        int global_max_nodes{80};
        double global_hybrid_search_radius{15.0};
        double global_unknown_penalty_factor{2.0};
        int global_refined_num{7};
        double global_refined_radius{5.0};
        std::string lkh_binary;
        std::string lkh_work_dir{"/tmp/general_planner_lkh"};
    };

    ExplorationFrontend(const Config &cfg,
                        const MapManager::Ptr &map_manager,
                        const path_search::Astar::Ptr &astar);

    bool planNextGoal(const super_utils::StatePVAJ &robot_state,
                      double current_yaw,
                      ExplorationGoal &goal);

    bool isExplorationFinished() const;

    const ExplorationDebugInfo &latestDebugInfo() const;

    void reset();

private:
    bool mapObservationReady(const FrontierSearchStats &stats) const;

    bool planWithGlobalCoverage(const super_utils::StatePVAJ &robot_state,
                                double current_yaw,
                                const rog_map::vec_E<FrontierCluster> &clusters,
                                ExplorationGoal &goal,
                                std::string &failure_reason);

    void sampleViewpointsForCluster(const FrontierCluster &cluster,
                                    const super_utils::StatePVAJ &robot_state,
                                    double current_yaw,
                                    super_utils::vec_E<ExplorationGoal> &candidates);

    int appendViewpointDebug(const FrontierCluster &cluster,
                             const super_utils::Vec3f &viewpoint,
                             double yaw,
                             const std::string &viewpoint_case,
                             const std::string &status,
                             bool accepted,
                             const ExplorationGoal *candidate = nullptr);

    void markViewpointReachable(int debug_id,
                                double travel_cost,
                                double score);

    void markViewpointSelected(const ExplorationGoal &goal);

    void setDebugReason(const std::string &reason,
                        bool planning_success,
                        bool exploration_finished);

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

    double estimateOpenSpaceScore(const super_utils::Vec3f &viewpoint) const;

    double estimateVelocityAlignmentScore(const super_utils::StatePVAJ &robot_state,
                                          const super_utils::Vec3f &viewpoint) const;

    double estimateHighSpeedScore(const super_utils::StatePVAJ &robot_state,
                                  const super_utils::Vec3f &viewpoint) const;

    double estimateLifecycleScore(const FrontierCluster &cluster) const;

    double scoreCandidate(const ExplorationGoal &candidate,
                          double unknown_risk) const;

    bool isUnknownLike(rog_map::GridType type) const;

    static double wrapAngleDiff(double lhs, double rhs);

    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    FrontierClusterManager frontier_cluster_manager_;
    ViewpointSelector viewpoint_selector_;
    GlobalCoveragePlanner global_coverage_planner_;
    bool exploration_finished_{false};
    uint64_t debug_sequence_{0};
    ExplorationDebugInfo latest_debug_info_;
};

}  // namespace general_planner
