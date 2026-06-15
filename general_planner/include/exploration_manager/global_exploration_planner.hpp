#pragma once

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <map_manager/frontier_cluster_manager.hpp>
#include <path_search/astar.h>
#include <super_utils/type_utils.hpp>

namespace general_planner {

struct ExplorationViewpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int frontier_cluster_id{-1};
    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};
    int visible_frontier_cells{0};
    double unknown_gain{0.0};
    double score{0.0};
    std::string viewpoint_case{"coverage_sample"};
};

struct ExplorationCoverageNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};
    super_utils::Vec3i key{super_utils::Vec3i::Zero()};
    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};
    std::vector<int> cluster_ids;
    int frontier_cell_count{0};
};

struct ExplorationCoveragePlan {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool success{false};
    bool used_lkh{false};
    std::string reason;
    double tour_cost{0.0};
    std::vector<int> node_tour;
    std::vector<int> cluster_tour;
    super_utils::vec_E<super_utils::Vec3f> coverage_path;
    super_utils::vec_E<ExplorationViewpoint> local_viewpoint_sequence;
    super_utils::vec_E<super_utils::Vec3f> guide_path;
};

class LkhAtspSolver {
public:
    struct Config {
        std::string binary_path;
        std::string work_dir{"/tmp/general_planner_lkh"};
        std::string problem_name{"general_planner_exploration"};
        int scale{100};
        bool allow_fallback{true};
    };

    LkhAtspSolver();
    explicit LkhAtspSolver(Config cfg);

    bool solve(const Eigen::MatrixXd &cost_matrix,
               std::vector<int> &tour,
               double &total_cost,
               bool &used_lkh,
               std::string &reason) const;

private:
    bool solveWithExternalLkh(const Eigen::MatrixXd &cost_matrix,
                              const std::string &binary_path,
                              std::vector<int> &tour,
                              double &total_cost,
                              std::string &reason) const;

    bool solveFallback(const Eigen::MatrixXd &cost_matrix,
                       std::vector<int> &tour,
                       double &total_cost,
                       std::string &reason) const;

    Config cfg_;
};

class ExplorationCostEvaluator {
public:
    struct Config {
        double max_vel{3.0};
        double max_acc{3.0};
        double max_yaw_rate{2.0};
        double hybrid_search_radius{15.0};
        double unknown_penalty_factor{2.0};
        bool use_astar{true};
        bool unknown_as_occupied_for_motion{true};
    };

    ExplorationCostEvaluator(Config cfg,
                             MapManager::Ptr map_manager,
                             path_search::Astar::Ptr astar);

    double computeCost(const super_utils::Vec3f &from,
                       const super_utils::Vec3f &to,
                       double from_yaw,
                       double to_yaw,
                       const super_utils::Vec3f &from_vel,
                       bool allow_unknown_motion,
                       bool hard_fail,
                       super_utils::vec_E<super_utils::Vec3f> *path = nullptr) const;

    double estimatePathLength(const super_utils::vec_E<super_utils::Vec3f> &path) const;

private:
    double yawTime(double from_yaw, double to_yaw) const;

    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

class ViewpointSelector {
public:
    struct Config {
        double min_radius{1.5};
        double max_radius{3.0};
        int radius_sample_num{3};
        int yaw_sample_num{24};
        double height_offset{0.0};
        double safe_distance{0.45};
        double unknown_clearance{0.0};
        double occupied_clearance{0.5};
        int min_visible_cells{5};
        int top_view_num{15};
        double max_decay{0.8};
        double horizontal_fov_deg{90.0};
        double sensor_range{8.0};
        double map_resolution{0.2};
    };

    ViewpointSelector(Config cfg, MapManager::Ptr map_manager);

    void selectViewpoints(const FrontierCluster &cluster,
                          const super_utils::Vec3f &robot_pos,
                          super_utils::vec_E<ExplorationViewpoint> &viewpoints) const;

private:
    bool isViewpointSafe(const super_utils::Vec3f &pos) const;

    bool isNearUnknown(const super_utils::Vec3f &pos) const;

    bool isNearOccupied(const super_utils::Vec3f &pos) const;

    int countVisibleCells(const super_utils::Vec3f &viewpoint,
                          double yaw,
                          const FrontierCluster &cluster) const;

    double estimateUnknownGain(const super_utils::Vec3f &viewpoint,
                               double yaw) const;

    bool insideYawFov(const super_utils::Vec3f &viewpoint,
                      double yaw,
                      const super_utils::Vec3f &target) const;

    static double wrapYaw(double yaw);

    Config cfg_;
    MapManager::Ptr map_manager_;
};

class CoverageGridManager {
public:
    struct Config {
        double cell_size{5.0};
        int max_active_nodes{80};
    };

    CoverageGridManager();
    explicit CoverageGridManager(Config cfg);

    void reset();

    void update(const rog_map::vec_E<FrontierCluster> &clusters,
                const std::unordered_map<int, super_utils::vec_E<ExplorationViewpoint>> &viewpoints_by_cluster,
                const super_utils::Vec3f &robot_pos);

    const super_utils::vec_E<ExplorationCoverageNode> &activeNodes() const;

private:
    super_utils::Vec3i makeKey(const super_utils::Vec3f &pos) const;

    Config cfg_;
    super_utils::vec_E<ExplorationCoverageNode> active_nodes_;
};

class GlobalCoveragePlanner {
public:
    struct Config {
        CoverageGridManager::Config grid;
        ExplorationCostEvaluator::Config cost;
        LkhAtspSolver::Config lkh;
        int refined_num{7};
        double refined_radius{5.0};
        int max_tour_nodes{80};
    };

    GlobalCoveragePlanner(Config cfg,
                          MapManager::Ptr map_manager,
                          path_search::Astar::Ptr astar);

    void reset();

    bool plan(const super_utils::StatePVAJ &robot_state,
              double current_yaw,
              const rog_map::vec_E<FrontierCluster> &clusters,
              const std::unordered_map<int, super_utils::vec_E<ExplorationViewpoint>> &viewpoints_by_cluster,
              ExplorationCoveragePlan &plan);

    const super_utils::vec_E<ExplorationCoverageNode> &activeCoverageNodes() const;

private:
    bool buildCoverageTour(const super_utils::StatePVAJ &robot_state,
                           double current_yaw,
                           ExplorationCoveragePlan &plan);

    bool refineLocalViewpoints(const super_utils::StatePVAJ &robot_state,
                               double current_yaw,
                               const std::unordered_map<int, super_utils::vec_E<ExplorationViewpoint>> &viewpoints_by_cluster,
                               ExplorationCoveragePlan &plan);

    std::vector<int> orderedClustersFromTour(const ExplorationCoveragePlan &plan) const;

    double nodeEdgeCost(const super_utils::Vec3f &from,
                        const super_utils::Vec3f &to,
                        double from_yaw,
                        double to_yaw,
                        const super_utils::Vec3f &from_vel,
                        bool from_robot) const;

    Config cfg_;
    CoverageGridManager coverage_grid_;
    ExplorationCostEvaluator cost_evaluator_;
    LkhAtspSolver atsp_solver_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

}  // namespace general_planner
