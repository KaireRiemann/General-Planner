#pragma once

#include <memory>
#include <vector>

#include <map_manager/coverage_guide_planner.hpp>
#include <map_manager/exploration_types.hpp>
#include <map_manager/map_manager.hpp>
#include <path_search/astar.h>
#include <ros_interface/ros_interface.hpp>
#include <super_utils/type_utils.hpp>

namespace general_planner {

class ExplorationManager {
public:
    using Ptr = std::shared_ptr<ExplorationManager>;

    ExplorationManager(const ExplorationConfig &cfg,
                       const MapManager::Ptr &map_manager,
                       const path_search::Astar::Ptr &astar,
                       const ros_interface::RosInterface::Ptr &ros_ptr);

    bool planNextGoal(const super_utils::StatePVAJ &robot_state,
                      double current_yaw,
                      double committed_traj_remaining,
                      ExplorationGoal &goal);

    bool isExplorationFinished() const;

    void reset();

    bool getCurrentGoal(ExplorationGoal &goal) const;

private:
    bool updateFrontierClusters(std::vector<FrontierCluster> &clusters);

    bool updateLocalFrontierClusters(const super_utils::Vec3f &robot_pos,
                                     std::vector<FrontierCluster> &clusters) const;

    void splitLargeClusters(std::vector<FrontierCluster> &clusters) const;

    void filterClusters(std::vector<FrontierCluster> &clusters) const;

    void sampleViewpointsForClusters(const std::vector<FrontierCluster> &clusters,
                                     const super_utils::StatePVAJ &robot_state,
                                     double current_yaw,
                                     std::vector<ViewpointCandidate> &candidates);

    bool viewpointSafe(const super_utils::Vec3f &p) const;

    bool insideSensorFov(const super_utils::Vec3f &viewpoint,
                         double yaw,
                         const super_utils::Vec3f &target) const;

    bool lineOfSightFree(const super_utils::Vec3f &viewpoint,
                         const super_utils::Vec3f &target) const;

    double computeVisibilityGain(const ViewpointCandidate &candidate,
                                 const FrontierCluster &cluster) const;

    double computeTravelCheapCost(const super_utils::Vec3f &robot_pos,
                                  const super_utils::Vec3f &candidate_pos) const;

    double computeYawCost(double current_yaw, double candidate_yaw) const;

    double computeCurvatureCost(const super_utils::StatePVAJ &robot_state,
                                const super_utils::Vec3f &candidate_pos,
                                const super_utils::Vec3f &cluster_center) const;

    double computeSwitchingCost(const ViewpointCandidate &candidate) const;

    double computeCheapScore(ViewpointCandidate &candidate) const;

    bool runAstarForCandidate(const super_utils::Vec3f &start,
                              ViewpointCandidate &candidate);

    bool shouldKeepCurrentGoal(const ExplorationGoal &current,
                               const ExplorationGoal &candidate,
                               double committed_traj_remaining) const;

    bool currentGoalStillInformative(const ExplorationGoal &goal) const;

    bool currentGoalStillSafe(const ExplorationGoal &goal) const;

    void addFailedCandidateToBlacklist(const ViewpointCandidate &candidate);

    bool candidateInBlacklist(const super_utils::Vec3f &p) const;

    ExplorationGoal toGoal(const ViewpointCandidate &candidate) const;

    bool isUnknownLike(rog_map::GridType type) const;
    bool isFreeLike(rog_map::GridType type) const;
    bool localCellHasUnknownNeighbor(const super_utils::Vec3f &p) const;
    bool globalCellHasUnknownNeighbor(const super_utils::Vec3f &p) const;
    void purgeBlacklist();
    void logGoalSelected(const ExplorationGoal &goal,
                         int candidate_count,
                         int astar_checked,
                         int frontier_count,
                         int cluster_count) const;
    void logNoGoal(const std::string &reason,
                   int candidate_count,
                   int frontier_count,
                   int cluster_count) const;

private:
    ExplorationConfig cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    ros_interface::RosInterface::Ptr ros_ptr_;
    std::unique_ptr<CoverageGuidePlanner> coverage_guide_planner_;

    ExplorationGoal current_goal_;
    bool exploration_finished_{false};

    struct FailedCandidate {
        super_utils::Vec3f position{super_utils::Vec3f::Zero()};
        double stamp{0.0};
    };
    std::vector<FailedCandidate> failed_candidates_;
};

class ExplorationFrontend : public ExplorationManager {
public:
    using Config = ExplorationConfig;
    using Ptr = std::shared_ptr<ExplorationFrontend>;

    ExplorationFrontend(const Config &cfg,
                        const MapManager::Ptr &map_manager,
                        const path_search::Astar::Ptr &astar);
};

}  // namespace general_planner
