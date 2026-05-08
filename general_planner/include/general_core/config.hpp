/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef GENERAL_PLANNER_CONFIG_HPP
#define GENERAL_PLANNER_CONFIG_HPP

#include <rog_map/rog_map_core/config.hpp>
#include <traj_opt/config.hpp>
#include <utils/header/yaml_loader.hpp>

namespace general_planner {
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    class Config {
    public:
        enum YawMode{
            YAW_TO_VEL = 1,
            YAW_TO_GOAL = 2
        };

        traj_opt::Config exp_traj_cfg, back_traj_cfg, esdf_traj_cfg, plain_traj_cfg;

        // Bool Params
        bool visualization_en{true};
        bool detailed_log_en{false};
        bool backup_traj_en;
        bool esdf_traj_en{false};
        bool plain_traj_en{false};
        bool use_fov_cut, print_log;
        bool goal_vel_en,goal_yaw_en;
        bool visual_process;
        bool frontend_in_known_free;

        double resolution;
        double planning_horizon;
        double receding_dis;
        double safe_corridor_line_max_length;
        // for fov cut
        double sensing_horizon;

        // Planning Params
        int obs_skip_num;
        double corridor_bound_dis, corridor_line_max_length;
        double replan_forward_dt;
        double sample_traj_dt;
        double robot_r;
        double esdf_safe_distance{0.3};
        int iris_iter_num;

        int mpc_horizon{};

        double yaw_dot_max;
        // Yaw mode: 1 heading to velocity, 2 heading to goal
        int yaw_mode = YAW_TO_VEL;

        double tracking_distance{2.2};
        double tracking_distance_tolerance{0.8};
        double tracking_height_offset{0.7};
        double tracking_height_tolerance{0.6};
        double tracking_safe_distance{0.35};
        double tracking_visibility_safe_distance{0.25};
        double tracking_visibility_cone_ratio{0.12};
        double tracking_visibility_angle_clearance{0.08726646259971647};
        double tracking_reacquire_distance{6.0};
        double tracking_min_commit_duration{0.8};
        double tracking_candidate_angle_step{0.3926990817};
        int tracking_candidate_radius_num{3};
        int tracking_visibility_samples{5};
        double tracking_weight_od_near{20.0};
        double tracking_weight_od_far{5.0};
        double tracking_weight_od_vertical{8.0};
        double tracking_weight_oa{5.0};
        double tracking_weight_fov{20.0};
        double tracking_weight_oe{1.0};
        double tracking_weight_relative_velocity{1.0};
        double tracking_weight_tangent_velocity{5.0};
        double tracking_weight_viewpoint_attractor{50.0};
        double tracking_weight_visible_region{3.0};
        double tracking_fov_horizontal_deg{90.0};
        double tracking_fov_vertical_deg{60.0};
        double tracking_fov_margin_deg{5.0};
        double tracking_fov_range{4.0};
        bool tracking_unknown_as_occupied{false};
        bool tracking_frontend_astar{true};
        bool tracking_use_visible_region{true};
        bool tracking_use_snap{false};

        double perching_robot_l{0.28};
        double perching_v_plus{0.8};
        double perching_pre_contact_distance{0.55};
        double perching_terminal_relax_time{0.35};
        double perching_safe_distance{0.0};
        double perching_platform_radius{0.35};
        double perching_robot_radius{0.25};
        double perching_platform_clearance{0.05};
        double perching_thrust_nominal{9.81};
        double perching_thrust_range{2.0};
        double perching_weight_nu{1.0e-2};
        double perching_weight_tau_f{1.0e-3};
        bool perching_frontend_astar{true};
        bool perching_use_dynamics_terminal_accel{true};

        bool swarm_enable{false};
        int swarm_drone_id{-1};
        double swarm_clearance{0.75};
        double swarm_des_clearance{0.75};
        double swarm_weight{0.0};
        double swarm_horizontal_scale{1.0};
        double swarm_vertical_scale{2.0};
        double swarm_activation_scale{1.5};
        double swarm_time_horizon{5.0};
        double swarm_stale_timeout{1.0};

        rog_map::vec_E<rog_map::Vec3i> seed_line_neighbour;


        Config() = default;
        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            exp_traj_cfg = traj_opt::Config(cfg_path, "exp_traj");
            back_traj_cfg = traj_opt::Config(cfg_path, "backup_traj");
            esdf_traj_cfg = traj_opt::Config(cfg_path, "esdf_traj");
            plain_traj_cfg = traj_opt::Config(cfg_path, "plain_traj");
            loader.LoadParam("general_planner/print_log", print_log, false);
            loader.LoadParam("general_planner/detailed_log_en", detailed_log_en, false);
            loader.LoadParam("general_planner/visualization_en", visualization_en, false);
            loader.LoadParam("general_planner/backup_traj_en", backup_traj_en, false);
            loader.LoadParam("general_planner/esdf_traj_en", esdf_traj_en, false);
            loader.LoadParam("general_planner/plain_traj_en", plain_traj_en, false);
            if (plain_traj_en) {
                if (plain_traj_cfg.penna_t < 0.0) plain_traj_cfg.penna_t = 40000.0;
                if (plain_traj_cfg.penna_pos < 0.0) plain_traj_cfg.penna_pos = 1.0e+7;
                if (plain_traj_cfg.penna_vel < 0.0) plain_traj_cfg.penna_vel = 5.0e+5;
                if (plain_traj_cfg.penna_acc < 0.0) plain_traj_cfg.penna_acc = 5.0e+5;
                if (plain_traj_cfg.penna_jerk < 0.0) plain_traj_cfg.penna_jerk = 5.0e+5;
                if (plain_traj_cfg.penna_attract < 0.0) plain_traj_cfg.penna_attract = 2.0e+3;
                if (plain_traj_cfg.penna_guide_vel < 0.0) plain_traj_cfg.penna_guide_vel = 1.0e+2;
                if (plain_traj_cfg.penna_omg < 0.0) plain_traj_cfg.penna_omg = 5.0e+5;
                if (plain_traj_cfg.penna_thr < 0.0) plain_traj_cfg.penna_thr = 1.0e+5;
            }
            loader.LoadParam("general_planner/esdf_safe_distance", esdf_safe_distance, 0.3);
            loader.LoadParam("general_planner/goal_vel_en", goal_vel_en, false);
            loader.LoadParam("general_planner/goal_yaw_en", goal_yaw_en, false);
            loader.LoadParam("general_planner/visual_process", visual_process, false);
            loader.LoadParam("general_planner/use_fov_cut", use_fov_cut, false);
            loader.LoadParam("general_planner/frontend_in_known_free", frontend_in_known_free, false);
            loader.LoadParam("general_planner/safe_corridor_line_max_length", safe_corridor_line_max_length, 3.0);
            loader.LoadParam("general_planner/sensing_horizon", sensing_horizon, 3.0);
            loader.LoadParam("general_planner/obs_skip_num", obs_skip_num, 1);
            loader.LoadParam("general_planner/replan_forward_dt", replan_forward_dt, 0.3);
            loader.LoadParam("general_planner/corridor_bound_dis", corridor_bound_dis, 3.0);
            loader.LoadParam("general_planner/corridor_line_max_length", corridor_line_max_length, 3.0);
            loader.LoadParam("general_planner/planning_horizon", planning_horizon, 10.0);
            loader.LoadParam("general_planner/receding_dis", receding_dis, 5.0);
            loader.LoadParam("general_planner/robot_r", robot_r, 0.3);
            loader.LoadParam("general_planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("general_planner/yaw_mode", yaw_mode, 1);
            loader.LoadParam("general_planner/mpc_horizon", mpc_horizon, 1);
            loader.LoadParam("general_planner/yaw_dot_max", yaw_dot_max, 3.14);
            loader.LoadParam("general_planner/tracking/distance", tracking_distance, 2.2);
            loader.LoadParam("general_planner/tracking/distance_tolerance", tracking_distance_tolerance, 0.8);
            loader.LoadParam("general_planner/tracking/height_offset", tracking_height_offset, 0.7);
            loader.LoadParam("general_planner/tracking/height_tolerance", tracking_height_tolerance, 0.6);
            loader.LoadParam("general_planner/tracking/safe_distance", tracking_safe_distance, 0.35);
            loader.LoadParam("general_planner/tracking/visibility_safe_distance", tracking_visibility_safe_distance, 0.25);
            loader.LoadParam("general_planner/tracking/visibility_cone_ratio", tracking_visibility_cone_ratio, 0.12);
            loader.LoadParam("general_planner/tracking/visibility_angle_clearance",
                             tracking_visibility_angle_clearance, 0.08726646259971647);
            loader.LoadParam("general_planner/tracking/reacquire_distance", tracking_reacquire_distance, 6.0);
            loader.LoadParam("general_planner/tracking/min_commit_duration", tracking_min_commit_duration, 0.8);
            loader.LoadParam("general_planner/tracking/candidate_angle_step", tracking_candidate_angle_step, 0.3926990817);
            loader.LoadParam("general_planner/tracking/candidate_radius_num", tracking_candidate_radius_num, 3);
            loader.LoadParam("general_planner/tracking/visibility_samples", tracking_visibility_samples, 5);
            loader.LoadParam("general_planner/tracking/weight_od_near", tracking_weight_od_near, 20.0);
            loader.LoadParam("general_planner/tracking/weight_od_far", tracking_weight_od_far, 5.0);
            loader.LoadParam("general_planner/tracking/weight_od_vertical", tracking_weight_od_vertical, 8.0);
            loader.LoadParam("general_planner/tracking/weight_oa", tracking_weight_oa, 5.0);
            loader.LoadParam("general_planner/tracking/weight_fov", tracking_weight_fov, 20.0);
            loader.LoadParam("general_planner/tracking/weight_oe", tracking_weight_oe, 1.0);
            loader.LoadParam("general_planner/tracking/weight_relative_velocity", tracking_weight_relative_velocity, 1.0);
            loader.LoadParam("general_planner/tracking/weight_tangent_velocity", tracking_weight_tangent_velocity, 5.0);
            loader.LoadParam("general_planner/tracking/weight_viewpoint_attractor", tracking_weight_viewpoint_attractor, 50.0);
            loader.LoadParam("general_planner/tracking/weight_visible_region", tracking_weight_visible_region, 3.0);
            loader.LoadParam("general_planner/tracking/fov_horizontal_deg", tracking_fov_horizontal_deg, 90.0);
            loader.LoadParam("general_planner/tracking/fov_vertical_deg", tracking_fov_vertical_deg, 60.0);
            loader.LoadParam("general_planner/tracking/fov_margin_deg", tracking_fov_margin_deg, 5.0);
            loader.LoadParam("general_planner/tracking/fov_range", tracking_fov_range, 4.0);
            loader.LoadParam("general_planner/tracking/unknown_as_occupied", tracking_unknown_as_occupied, false);
            loader.LoadParam("general_planner/tracking/frontend_astar", tracking_frontend_astar, true);
            loader.LoadParam("general_planner/tracking/use_visible_region", tracking_use_visible_region, true);
            loader.LoadParam("general_planner/tracking/use_snap", tracking_use_snap, false);
            loader.LoadParam("general_planner/perching/robot_l", perching_robot_l, 0.28);
            loader.LoadParam("general_planner/perching/v_plus", perching_v_plus, 0.8);
            loader.LoadParam("general_planner/perching/pre_contact_distance", perching_pre_contact_distance, 0.55);
            loader.LoadParam("general_planner/perching/terminal_relax_time", perching_terminal_relax_time, 0.35);
            loader.LoadParam("general_planner/perching/safe_distance", perching_safe_distance, 0.0);
            loader.LoadParam("general_planner/perching/platform_radius", perching_platform_radius, 0.35);
            loader.LoadParam("general_planner/perching/robot_radius", perching_robot_radius, 0.25);
            loader.LoadParam("general_planner/perching/platform_clearance", perching_platform_clearance, 0.05);
            loader.LoadParam("general_planner/perching/thrust_nominal", perching_thrust_nominal, 9.81);
            loader.LoadParam("general_planner/perching/thrust_range", perching_thrust_range, 2.0);
            loader.LoadParam("general_planner/perching/weight_nu", perching_weight_nu, 1.0e-2);
            loader.LoadParam("general_planner/perching/weight_tau_f", perching_weight_tau_f, 1.0e-3);
            loader.LoadParam("general_planner/perching/frontend_astar", perching_frontend_astar, true);
            loader.LoadParam("general_planner/perching/use_dynamics_terminal_accel",
                             perching_use_dynamics_terminal_accel, true);
            loader.LoadParam("general_planner/swarm/enable", swarm_enable, false);
            loader.LoadParam("general_planner/swarm/drone_id", swarm_drone_id, -1);
            loader.LoadParam("general_planner/swarm/clearance", swarm_clearance, 0.75);
            loader.LoadParam("general_planner/swarm/des_clearance", swarm_des_clearance, 0.75);
            loader.LoadParam("general_planner/swarm/weight", swarm_weight, 0.0);
            loader.LoadParam("general_planner/swarm/horizontal_scale", swarm_horizontal_scale, 1.0);
            loader.LoadParam("general_planner/swarm/vertical_scale", swarm_vertical_scale, 2.0);
            loader.LoadParam("general_planner/swarm/activation_scale", swarm_activation_scale, 1.5);
            loader.LoadParam("general_planner/swarm/time_horizon", swarm_time_horizon, 5.0);
            loader.LoadParam("general_planner/swarm/stale_timeout", swarm_stale_timeout, 1.0);

            loader.LoadParam("rog_map/resolution", resolution, 0.01, true);

            sample_traj_dt = resolution / exp_traj_cfg.max_vel;

            int step = ceil(robot_r / resolution);
            for (int x = -step; x <= step; x++) {
                for (int y = -step; y <= step; y++) {
                    for (int z = -step; z <= step; z++) {
                        if (x * x + y * y + z * z <= step * step) {
                            seed_line_neighbour.push_back({x, y, z});
                        }
                    }
                }
            }
            std::sort(seed_line_neighbour.begin(), seed_line_neighbour.end(),
                      [](const auto& a, const auto& b) {
                          return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
                      });
        }


    };
}

#endif
