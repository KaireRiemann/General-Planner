#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <exploration_manager/global_exploration_planner.hpp>
#include <general_core/exploration_frontend.hpp>
#include <general_core/exploration_viewpoint_planner.hpp>

namespace general_planner {

class CompleteExplorationFrontend {
public:
    using Ptr = std::shared_ptr<CompleteExplorationFrontend>;

    struct Config {
        bool enable{false};
        bool print_log{true};

        double update_radius{18.0};
        double finish_confirm_time{2.0};
        int finish_confirm_count{5};

        bool use_rog_complete_frontier{false};
        bool use_memory_grid{true};
        bool use_region_graph{false};
        bool use_coverage_guidance{false};
        bool use_fuel_style_tour{true};

        int max_frontiers_per_cycle{64};
        int max_astar_checks_per_cycle{32};
        bool lazy_astar_enable{true};
        double travel_cost_cache_timeout{0.5};
        int tour_refined_num{5};
        double tour_refined_radius{5.0};
        double tour_replan_min_interval{1.0};
        double tour_radius_far{5.0};
        double tour_radius_close{1.5};

        ExplorationMemoryGrid::Config memory_cfg;
        FrontierDatabase::Config frontier_db_cfg;
        ExplorationRegionGraph::Config region_graph_cfg;
        CoveragePathPlanner::Config coverage_cfg;
        ExplorationViewpointPlanner::Config viewpoint_cfg;
        LkhAtspSolver::Config lkh_cfg;
    };

    CompleteExplorationFrontend(const Config &cfg,
                                const MapManager::Ptr &map_manager,
                                const path_search::Astar::Ptr &astar);

    void reset();

    bool planNextGoal(const super_utils::StatePVAJ &robot_state,
                      double current_yaw,
                      ExplorationGoal &goal);

    void onGoalResult(const ExplorationGoal &goal,
                      super_utils::RET_CODE ret,
                      bool reached);

    bool isExplorationFinished() const;

    std::string latestStatusString() const;

    const ExplorationDebugInfo &latestDebugInfo() const;

private:
    bool checkFinishCondition(double stamp);
    double now() const;
    void fillGoalFromViewpoint(const CompleteExplorationViewpoint &viewpoint,
                               const super_utils::Vec3f &robot_pos,
                               ExplorationGoal &goal) const;
    bool selectFuelStyleGoal(const super_utils::StatePVAJ &robot_state,
                             double current_yaw,
                             const std::vector<CompleteFrontierCluster> &frontiers,
                             const std::vector<CompleteExplorationViewpoint> &candidates,
                             double stamp,
                             ExplorationGoal &goal);
    bool rebuildFuelStyleTour(const super_utils::StatePVAJ &robot_state,
                              const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier,
                              double current_yaw,
                              double stamp);
    bool chooseRefinedViewpoint(const super_utils::StatePVAJ &robot_state,
                                const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier,
                                CompleteExplorationViewpoint &selected) const;
    void pruneTour(const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier);
    void segmentFarGoal(const CompleteExplorationViewpoint &viewpoint,
                        const super_utils::Vec3f &robot_pos,
                        ExplorationGoal &goal) const;
    std::size_t candidateFrontierHash(
            const std::unordered_map<int, std::vector<CompleteExplorationViewpoint>> &viewpoints_by_frontier) const;

    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;

    ExplorationMemoryGrid memory_;
    FrontierDatabase frontier_db_;
    ExplorationRegionGraph region_graph_;
    CoveragePathPlanner coverage_planner_;
    ExplorationViewpointPlanner viewpoint_planner_;
    LkhAtspSolver tour_solver_;

    bool exploration_finished_{false};
    int finish_confirm_counter_{0};
    double finish_confirm_start_time_{-1.0};
    double last_new_frontier_time_{0.0};
    int last_active_count_{0};
    int latest_candidate_count_{0};
    int latest_reachable_candidate_count_{0};
    int latest_selected_frontier_id_{-1};
    int latest_selected_region_id_{-1};
    double latest_selected_score_{0.0};
    double latest_selected_gain_{0.0};
    double latest_selected_travel_cost_{0.0};
    int latest_memory_known_free_{0};
    int latest_memory_occupied_{0};
    int latest_memory_unknown_{0};
    int latest_tour_size_{0};
    bool latest_tour_used_lkh_{false};
    double latest_tour_cost_{0.0};
    std::string latest_tour_reason_{"not_started"};
    std::string latest_reason_{"not_started"};

    std::vector<int> active_tour_;
    std::size_t last_tour_frontier_hash_{0};
    double last_tour_update_time_{-1.0};
    uint64_t debug_sequence_{0};
    ExplorationDebugInfo latest_debug_info_;
};

}  // namespace general_planner
