#pragma once

#include <memory>

#include <map_manager/map_manager.hpp>
#include "path_search/astar.h"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner
{

class TakeoffFrontend
{
public:
    using Ptr = std::shared_ptr<TakeoffFrontend>;

    struct Config
    {
        double robot_l{0.28};
        double robot_radius{0.25};
        double platform_radius{0.35};
        double platform_clearance{0.05};
        double platform_clearance_after_release{0.08};
        double release_contact_time{0.20};
        double escape_distance{1.0};
        double escape_height{0.8};
        double reference_speed{1.5};
        double min_duration{0.6};
        double max_duration{3.0};
        int piece_num{3};
        bool frontend_astar{true};
        double safe_distance{0.35};
        bool use_tangent_release_velocity{false};
        double thrust_nominal{9.81};
        double thrust_range{2.0};
        double gravity{9.81};
        double weight_eta{1.0};
        double weight_tau_f{1.0e-3};
        bool rotate_surface_with_yaw_rate{true};
    };

    TakeoffFrontend(Config cfg,
                    MapManager::Ptr map_manager,
                    path_search::Astar::Ptr astar);

    bool buildProblem(const traj_opt::PerchingSurfaceState &surface,
                      traj_opt::DynamicTakeoffProblem &problem) const;

private:
    bool pointSafe(const general_utils::Vec3f &p) const;
    bool appendSafeEscapeSegment(const general_utils::Vec3f &start,
                                 const general_utils::Vec3f &goal,
                                 general_utils::vec_E<general_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

} // namespace general_planner
