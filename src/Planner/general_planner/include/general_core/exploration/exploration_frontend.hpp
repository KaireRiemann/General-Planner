#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <general_core/exploration/atsp/atsp_tour_planner.hpp>
#include <general_core/map_manager.hpp>
#include <general_core/nhbp/nav_identity.hpp>
#include <path_search/astar.h>
#include <general_utils/type_utils.hpp>

namespace general_planner {

class ExplorationManager;

struct ExplorationGoal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    double yaw{0.0};

    double score{0.0};
    double information_gain{0.0};
    double travel_cost{0.0};
    double yaw_cost{0.0};
    double curvature_cost{0.0};
    double distance_to_robot{0.0};
    double frontier_area{0.0};
    double history_score_delta{0.0};
    int visible_frontier_cell_count{0};
    double visible_frontier_ratio{0.0};

    int checked_candidate_count{0};
    int astar_check_count{0};
    int reachable_candidate_count{0};
    int cluster_count{0};
    int raw_cluster_count{0};
    int frontier_cell_count{0};
    int raw_frontier_cell_count{0};

    int candidate_id{-1};
    int frontier_id{-1};
    std::string memory_key;
    nhbp::NavIdentity identity;

    bool frontier_center_valid{false};
    general_utils::Vec3f frontier_center{general_utils::Vec3f::Zero()};
    general_utils::Vec3f frontier_bbox_min{general_utils::Vec3f::Zero()};
    general_utils::Vec3f frontier_bbox_max{general_utils::Vec3f::Zero()};

    std::string reason;
    general_utils::vec_E<general_utils::Vec3f> guide_path;
};

struct ExplorationCandidateSet {
    bool valid{false};
    bool exploration_finished{false};
    ExplorationGoal suggested_goal;
    general_utils::vec_E<ExplorationGoal> candidates;

    int checked_candidate_count{0};
    int astar_check_count{0};
    int reachable_candidate_count{0};
    int cluster_count{0};
    int raw_cluster_count{0};
    int frontier_cell_count{0};
    int raw_frontier_cell_count{0};
    std::string source;
    std::string reason;
};

class ExplorationFrontend {
public:
    using Ptr = std::shared_ptr<ExplorationFrontend>;

    struct Config {
        bool enable{false};
        bool print_log{false};
        std::string frontier_source{"fallback_scan"};
        bool global_box_enable{false};
        general_utils::Vec3f global_box_min{-50.0, -50.0, 0.0};
        general_utils::Vec3f global_box_max{50.0, 50.0, 3.0};

        double map_resolution{0.2};
        double frontier_search_radius{12.0};
        double frontier_cluster_radius{0.8};
        double frontier_sample_resolution{0.8};
        double frontier_subcluster_size{4.0};
        int min_frontier_cluster_size{5};
        double min_frontier_area{0.12};
        double min_frontier_extent{0.25};
        int min_unknown_neighbor_count{6};
        int max_raw_frontier_points{4096};
        int max_frontier_cells{1536};
        int max_frontier_clusters{64};
        int max_subclusters_per_cluster{16};

        bool frontier_manager_enable{true};
        int frontier_manager_max_records{512};
        double frontier_manager_match_radius{2.0};
        double frontier_manager_stale_time{8.0};
        double frontier_manager_dormant_time{6.0};
        double frontier_manager_covered_unknown_radius{1.5};
        double frontier_manager_min_changed_fraction{0.2};
        double frontier_manager_selection_penalty{6.0};
        double frontier_manager_recent_selection_penalty{12.0};
        double frontier_manager_recent_selection_window{10.0};
        int frontier_manager_no_view_threshold{2};

        double viewpoint_min_distance{1.2};
        double viewpoint_max_distance{4.0};
        double viewpoint_height_offset{0.0};
        double viewpoint_safe_distance{0.45};

        int viewpoint_yaw_sample_num{16};
        int viewpoint_radius_sample_num{3};
        bool viewpoint_use_normal_sampling{true};
        double viewpoint_normal_angle{1.2};
        int viewpoint_z_sample_num{1};
        double viewpoint_z_min{0.0};
        double viewpoint_z_max{0.0};

        bool expansion_fallback_enable{true};
        int expansion_trigger_min_candidates{10};
        int expansion_trigger_max_clusters{1};
        int expansion_max_candidate_num{24};
        int expansion_yaw_sample_num{16};
        int expansion_radius_sample_num{3};
        double expansion_min_radius{2.0};
        double expansion_max_radius{7.0};
        int expansion_z_sample_num{3};
        double expansion_z_min{-0.6};
        double expansion_z_max{0.8};
        double expansion_min_information_gain{4.0};
        bool expansion_allow_unknown_viewpoint{false};
        bool expansion_memory_enable{true};
        int expansion_memory_max_records{128};
        double expansion_memory_ttl{45.0};
        double expansion_memory_match_radius{1.2};
        bool expansion_anti_revisit_enable{true};
        int expansion_retire_after_commits{1};
        double expansion_commit_block_radius{3.0};
        double expansion_commit_block_ttl{45.0};
        double expansion_synthetic_gain_ratio{0.25};

        int min_visible_frontier_cells{3};
        double min_visible_frontier_ratio{0.05};
        int max_candidate_num{128};
        int max_candidates_per_frontier_cluster{8};
        double candidate_separation_distance{1.5};
        int max_astar_checks{64};
        int min_direct_reachable_before_astar{8};
        int max_astar_checks_per_frontier{2};
        int max_reachable_candidate_num{48};
        int max_gain_rays{96};
        double gain_ray_length{5.0};
        double gain_ray_step{0.2};
        std::string yaw_policy{"velocity"};

        double weight_travel{1.0};
        double weight_yaw{0.5};
        double weight_curvature{0.8};
        double weight_info_gain{-2.0};
        double weight_unknown_risk{1.0};
        double weight_progress{-0.75};
        double weight_short_goal{4.0};
        double information_gain_saturation{250.0};

        double min_information_gain{3.0};
        double min_goal_distance{3.0};
        double preferred_goal_distance{8.0};
        double goal_switch_min_score_improvement{0.25};
        double goal_reached_distance{0.5};

        bool unknown_as_occupied_for_motion{true};
        bool require_line_free_to_frontier{false};
        bool use_astar_cost{true};
        double astar_time_out{0.05};
        double astar_total_time_budget_ms{40.0};
        double astar_failure_cache_ttl{4.0};

        bool use_atsp{false};
        exploration::ATSPTourPlanner::Config atsp;
    };

    ExplorationFrontend(const Config &cfg,
                        const MapManager::Ptr &map_manager,
                        const path_search::Astar::Ptr &astar);

    ~ExplorationFrontend();

    bool generateCandidates(const general_utils::StatePVAJ &robot_state,
                            double current_yaw,
                            double stamp,
                            ExplorationCandidateSet &out);

    bool planNextGoal(const general_utils::StatePVAJ &robot_state,
                      double current_yaw,
                      ExplorationGoal &goal,
                      double stamp = 0.0);

    bool getLastCandidateSet(ExplorationCandidateSet &candidate_set) const;

    bool isExplorationFinished() const;

    void reset();

    void setMissionManager(ExplorationManager *manager);

    void recordGoalCommitted(const ExplorationGoal &goal,
                             double stamp,
                             bool goal_switched);

    void recordGoalFailed(const ExplorationGoal &goal,
                          double stamp);

private:
    struct FrontierCell {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        general_utils::Vec3f position{general_utils::Vec3f::Zero()};
        general_utils::Vec3i index{general_utils::Vec3i::Zero()};
    };

    struct FrontierCluster {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        general_utils::vec_E<FrontierCell> raw_cells;
        general_utils::vec_E<FrontierCell> cells;
        general_utils::Vec3f center{general_utils::Vec3f::Zero()};
        general_utils::Vec3f unknown_direction{general_utils::Vec3f::UnitX()};
        general_utils::Vec3f normal{general_utils::Vec3f::UnitX()};
        general_utils::Vec3f free_direction{-1.0, 0.0, 0.0};
        general_utils::Vec3f bbox_min{general_utils::Vec3f::Zero()};
        general_utils::Vec3f bbox_max{general_utils::Vec3f::Zero()};
        general_utils::Vec3f extent{general_utils::Vec3f::Zero()};
        double area{0.0};
        int size{0};
        int unknown_neighbor_count{0};

        int object_id{-1};
        int object_seen_count{0};
        int object_selection_count{0};
        int object_failure_count{0};
        double object_score_delta{0.0};
        double object_last_seen_stamp{0.0};
    };

    struct FrontierSearchStats {
        std::string source{"fallback_scan"};
        int searched_cells{0};
        int known_free_cells{0};
        int unknown_cells{0};
        int occupied_cells{0};
        int frontier_cells{0};
        int raw_frontier_cells{0};
        bool fallback_used{false};
    };

    struct AstarFailureCacheEntry {
        std::chrono::steady_clock::time_point expires_at;
        int failure_count{0};
    };

    struct FrontierObjectStats {
        int observed{0};
        int active{0};
        int dormant{0};
        int covered{0};
        int stale{0};
        int records{0};
    };

    struct ExpansionViewpointRecord {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ExplorationGoal goal;
        double last_seen_stamp{0.0};
        int seen_count{0};
    };

    struct ExpansionVisitRecord {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        general_utils::Vec3f position{general_utils::Vec3f::Zero()};
        double last_commit_stamp{0.0};
        int commit_count{0};
    };

    class FrontierObjectManager;

    bool collectFrontierCells(const general_utils::Vec3f &robot_pos,
                              general_utils::vec_E<FrontierCell> &frontier_cells,
                              FrontierSearchStats &stats);

    bool collectFallbackFrontierCells(const general_utils::Vec3f &robot_pos,
                                      general_utils::vec_E<FrontierCell> &frontier_cells,
                                      FrontierSearchStats &stats) const;

    bool collectRogMapFrontierCells(const general_utils::Vec3f &robot_pos,
                                    general_utils::vec_E<FrontierCell> &frontier_cells,
                                    FrontierSearchStats &stats);

    bool collectRogMapFrontierClusters(const general_utils::Vec3f &robot_pos,
                                       general_utils::vec_E<FrontierCluster> &clusters,
                                       FrontierSearchStats &stats);

    bool updateIncrementalRogFrontiers(const general_utils::Vec3f &robot_pos,
                                       FrontierSearchStats &stats);

    bool frontierClusterChanged(const FrontierCluster &cluster) const;

    bool frontierClusterOverlapsBox(const FrontierCluster &cluster,
                                    const general_utils::Vec3f &box_min,
                                    const general_utils::Vec3f &box_max) const;

    void downsampleFrontierCluster(FrontierCluster &cluster) const;

    void appendCachedFrontierCells(const general_utils::Vec3f &robot_pos,
                                   general_utils::vec_E<FrontierCell> &frontier_cells,
                                   FrontierSearchStats &stats) const;

    void appendCachedFrontierClusters(const general_utils::Vec3f &robot_pos,
                                      general_utils::vec_E<FrontierCluster> &clusters,
                                      FrontierSearchStats &stats) const;

    bool isFrontierCell(const FrontierCell &cell,
                        rog_map::GridType grid_type) const;

    bool mapObservationReady(const FrontierSearchStats &stats) const;

    void clusterFrontiers(const general_utils::vec_E<FrontierCell> &frontier_cells,
                          general_utils::vec_E<FrontierCluster> &clusters) const;

    bool finalizeFrontierCluster(FrontierCluster &cluster) const;

    void splitLargeFrontierClusters(const general_utils::vec_E<FrontierCluster> &clusters,
                                    const general_utils::Vec3f &robot_pos,
                                    general_utils::vec_E<FrontierCluster> &split_clusters) const;

    double clusterPriorityScore(const FrontierCluster &cluster,
                                const general_utils::Vec3f &robot_pos) const;

    bool frontierClusterValid(const FrontierCluster &cluster) const;

    void sampleViewpointsForCluster(const FrontierCluster &cluster,
                                    const general_utils::StatePVAJ &robot_state,
                                    double current_yaw,
                                    general_utils::vec_E<ExplorationGoal> &candidates,
                                    int candidate_budget) const;

    void appendExpansionFallbackCandidates(
            const general_utils::StatePVAJ &robot_state,
            double current_yaw,
            double stamp,
            const general_utils::vec_E<FrontierCluster> &clusters,
            general_utils::vec_E<ExplorationGoal> &candidates,
            int &attempt_count,
            int &added_count);

    void appendRememberedExpansionCandidates(
            const general_utils::StatePVAJ &robot_state,
            double current_yaw,
            double stamp,
            const general_utils::vec_E<FrontierCluster> &clusters,
            general_utils::vec_E<ExplorationGoal> &candidates,
            int &added_count);

    bool makeExpansionCandidate(
            const general_utils::StatePVAJ &robot_state,
            double current_yaw,
            double stamp,
            const general_utils::Vec3f &viewpoint,
            const general_utils::vec_E<FrontierCluster> &clusters,
            ExplorationGoal &candidate) const;

    bool expansionViewpointSafe(const general_utils::Vec3f &viewpoint,
                                bool allow_unknown) const;

    bool candidateSeparatedFromPool(
            const ExplorationGoal &candidate,
            const general_utils::vec_E<ExplorationGoal> &pool,
            double separation) const;

    int nearestClusterIndex(const general_utils::Vec3f &viewpoint,
                            const general_utils::vec_E<FrontierCluster> &clusters) const;

    void rememberExpansionCandidate(const ExplorationGoal &candidate, double stamp);

    void pruneExpansionViewpointMemory(double stamp);

    bool expansionBlockedByVisitMemory(const general_utils::Vec3f &viewpoint,
                                       double stamp) const;

    void rememberExpansionVisit(const ExplorationGoal &goal, double stamp);

    void pruneExpansionVisitMemory(double stamp);

    void diversifyCandidates(general_utils::vec_E<ExplorationGoal> &candidates) const;

    bool viewpointSafe(const general_utils::Vec3f &viewpoint) const;

    bool viewpointVisible(const general_utils::Vec3f &viewpoint,
                          const FrontierCluster &cluster) const;

    int visibleFrontierThreshold(const FrontierCluster &cluster) const;

    int countVisibleFrontierCells(const general_utils::Vec3f &viewpoint,
                                  const FrontierCluster &cluster,
                                  int max_checks = 0) const;

    double estimateInformationGain(const general_utils::Vec3f &viewpoint,
                                   const FrontierCluster &cluster) const;

    double estimateLocalUnknownGain(const general_utils::Vec3f &viewpoint,
                                    int max_rays = 0) const;

    double estimateTravelCost(const general_utils::Vec3f &robot_pos,
                              const general_utils::Vec3f &viewpoint,
                              general_utils::vec_E<general_utils::Vec3f> &guide_path,
                              bool allow_astar = true,
                              bool *astar_used = nullptr) const;

    double estimateYawCost(double current_yaw,
                           double candidate_yaw) const;

    double resolveCandidateYaw(double current_yaw,
                               const general_utils::Vec3f &robot_pos,
                               const general_utils::Vec3f &viewpoint,
                               const FrontierCluster &cluster) const;

    double estimateCurvatureCost(const general_utils::StatePVAJ &robot_state,
                                 const general_utils::Vec3f &viewpoint,
                                 const FrontierCluster &cluster) const;

    double estimateUnknownRisk(const general_utils::Vec3f &viewpoint) const;

    double scoreCandidate(const ExplorationGoal &candidate,
                          double unknown_risk) const;

    double effectiveInformationGain(double information_gain) const;

    ExplorationGoal selectGoalWithAtsp(const general_utils::Vec3f &robot_pos,
                                       double current_yaw,
                                       const general_utils::vec_E<ExplorationGoal> &reachable_candidates) const;

    double pairwiseCandidateCost(const ExplorationGoal &from,
                                 const ExplorationGoal &to) const;

    bool isUnknownLike(rog_map::GridType type) const;

    bool isFreeLike(rog_map::GridType type) const;

    bool insideTaskRegion(const general_utils::Vec3f &position) const;

    bool clipTaskSearchBox(general_utils::Vec3f &box_min,
                           general_utils::Vec3f &box_max) const;

    void clampViewpointToTaskRegion(general_utils::Vec3f &viewpoint) const;

    static double wrapAngleDiff(double lhs, double rhs);

    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    ExplorationManager *mission_manager_{nullptr};
    bool exploration_finished_{false};
    std::unordered_map<std::string, AstarFailureCacheEntry> astar_failure_cache_;
    std::unique_ptr<FrontierObjectManager> frontier_object_manager_;
    general_utils::vec_E<FrontierCluster> cached_frontier_clusters_;
    bool frontier_cache_initialized_{false};
    general_utils::vec_E<ExpansionViewpointRecord> expansion_viewpoint_records_;
    general_utils::vec_E<ExpansionVisitRecord> expansion_visit_records_;
    ExplorationCandidateSet last_candidate_set_;
};

}  // namespace general_planner
