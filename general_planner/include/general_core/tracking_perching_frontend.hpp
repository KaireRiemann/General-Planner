#pragma once

#include <memory>

#include "path_search/astar.h"
#include "general_core/map_manager.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner
{

class TrackingFrontend
{
public:
    using Ptr = std::shared_ptr<TrackingFrontend>;

    struct Config
    {
        double tracking_distance{3.0};
        double distance_tolerance{0.8};
        double height_offset{0.8};
        double height_tolerance{0.6};
        double safe_distance{0.45};
        double visibility_safe_distance{0.25};
        double visibility_cone_ratio{0.12};
        double searching_horizon{8.0};
        double candidate_angle_step{0.3926990817};
        int candidate_radius_num{3};
        int visibility_samples{5};
        bool unknown_as_occupied{true};
        bool use_astar{true};
    };

    TrackingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const super_utils::StatePVAJ &head_pvaj,
                      const traj_opt::DynamicTargetStates &target_prediction,
                      traj_opt::TrackingProblem &problem) const;

private:
    bool isViewpointVisible(const super_utils::Vec3f &viewpoint,
                            const super_utils::Vec3f &target) const;
    bool isViewpointSafe(const super_utils::Vec3f &viewpoint) const;
    super_utils::Vec3f chooseVisibleViewpoint(const super_utils::Vec3f &seed,
                                              const traj_opt::DynamicTargetState &target) const;
    bool findOcclusionAwareSeed(const super_utils::Vec3f &last_viewpoint,
                                const super_utils::Vec3f &last_target,
                                const super_utils::Vec3f &target,
                                super_utils::Vec3f &seed) const;
    super_utils::Vec3f extendToTrackingDistance(const super_utils::Vec3f &seed,
                                                const super_utils::Vec3f &target,
                                                const super_utils::Vec3f &fallback) const;
    super_utils::Vec3f choosePropagatedViewpoint(const super_utils::Vec3f &last_viewpoint,
                                                 const traj_opt::DynamicTargetState &last_target,
                                                 const traj_opt::DynamicTargetState &target) const;
    bool appendPathSegment(const super_utils::Vec3f &start,
                           const super_utils::Vec3f &goal,
                           super_utils::vec_E<super_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

class PerchingFrontend
{
public:
    using Ptr = std::shared_ptr<PerchingFrontend>;

    struct Config
    {
        double robot_l{0.25};
        double v_plus{1.0};
        double pre_contact_distance{0.45};
        double terminal_relax_time{0.35};
        double safe_distance{0.45};
        double platform_radius{0.35};
        double robot_radius{0.25};
        double platform_clearance{0.05};
        double thrust_nominal{9.81};
        double thrust_range{2.0};
        double weight_nu{1.0e-2};
        double weight_tau_f{1.0e-3};
        double searching_horizon{8.0};
        bool use_astar{true};
        bool use_dynamics_terminal_accel{true};
    };

    PerchingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const super_utils::StatePVAJ &head_pvaj,
                      const traj_opt::PerchingSurfaceState &surface,
                      traj_opt::PerchingProblem &problem) const;

private:
    bool appendPathSegment(const super_utils::Vec3f &start,
                           const super_utils::Vec3f &goal,
                           super_utils::vec_E<super_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

} // namespace general_planner
