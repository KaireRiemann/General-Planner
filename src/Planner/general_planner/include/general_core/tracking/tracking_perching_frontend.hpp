#pragma once

#include <memory>
#include <string>
#include <vector>

#include "general_core/tracking/tracking_frontend.hpp"
#include "path_search/astar.h"
#include <map_manager/map_manager.hpp>
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner
{

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
        double min_duration{0.6};
        double max_duration{4.0};
        double reference_speed{2.0};
        double max_speed{5.0};
        double max_acc{4.0};
        double max_jerk{20.0};
        double max_omega{4.0};
        double relative_z_min{0.1};
        double relative_z_max{3.0};
        double weight_relative_height{1.0};
        double visual_min_distance{0.2};
        double visual_activation_distance{3.0};
        double visual_fx{1.0};
        double visual_fy{1.0};
        double gravity{9.81};
        double searching_horizon{8.0};
        int piece_num{0};
        double min_piece_duration{0.12};
        double min_total_duration{0.0};
        double max_total_duration{-1.0};
        double time_lower_bound_weight{0.0};
        double time_upper_bound_weight{0.0};
        double duration_seed_weight{0.0};
        double duration_margin{0.20};
        bool allow_long_standalone{false};
        double max_piece_duration{1.2};
        int min_piece_num{3};
        int max_piece_num{8};
        bool multi_point_guide_enable{true};
        int moving_guide_sample_num{4};
        double tau_f_seed_limit{1.30};
        bool reset_surface_time{true};
        bool use_astar{true};
        bool use_dynamics_terminal_accel{true};
        bool rotate_surface_with_yaw_rate{true};
    };

    PerchingFrontend(const Config &cfg,
                     const MapManager::Ptr &map_manager,
                     const path_search::Astar::Ptr &astar);

    bool buildProblem(const general_utils::StatePVAJ &head_pvaj,
                      const traj_opt::PerchingSurfaceState &surface,
                      traj_opt::PerchingProblem &problem) const;

private:
    bool appendPathSegment(const general_utils::Vec3f &start,
                           const general_utils::Vec3f &goal,
                           general_utils::vec_E<general_utils::Vec3f> &path) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
};

} // namespace general_planner
