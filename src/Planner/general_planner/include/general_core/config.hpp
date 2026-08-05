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

#include <string>

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
        double frontend_astar_time_out{0.1};
        bool over_wall_search_en{true};
        double over_wall_max_climb{3.0};
        double over_wall_height_step{0.3};
        double over_wall_min_blocked_span{0.8};
        double over_wall_min_progress_gain{0.5};
        double over_wall_forward_ratio{0.9};
        bool dynamic_obstacle_layer_enable{false};
        double dynamic_obstacle_layer_ttl{0.35};
        double dynamic_obstacle_layer_voxel_size{0.20};
        double dynamic_obstacle_layer_inflation_radius{0.35};
        double dynamic_obstacle_layer_line_check_step{0.05};
        int dynamic_obstacle_layer_max_points_per_frame{30000};
        general_utils::Vec3f dynamic_obstacle_layer_local_half_size{10.0, 10.0, 3.0};
        bool state2state_direct_line_frontend_enable{true};
        bool state2state_topology_enable{false};
        bool state2state_topology_query_enable{false};
        std::string state2state_topology_construction_mode{"bubble_topology"};
        bool state2state_topology_unknown_as_free{false};
        bool state2state_topology_planar_mode{false};
        double state2state_topology_navigation_altitude{-1.0e6};
        double state2state_topology_min_query_distance{3.0};
        int state2state_topology_update_budget{1};
        double state2state_topology_update_period{0.20};
        double state2state_topology_publish_period{0.50};
        double state2state_topology_region_size{4.0};
        double state2state_topology_sample_spacing{1.0};
        double state2state_topology_evidence_vertical_tolerance{0.50};
        double state2state_topology_min_clearance{0.45};
        double state2state_topology_max_clearance{2.5};
        double state2state_topology_connection_radius{6.0};
        double state2state_topology_dirty_padding{2.5};
        double state2state_topology_bubble_overlap_margin{0.10};
        int state2state_topology_max_nodes_per_region{4};
        int state2state_topology_max_bubbles_per_region{256};
        int state2state_topology_max_neighbors{8};
        bool state2state_altitude_guard_enable{false};
        double state2state_altitude_band{0.25};
        bool state2state_altitude_escape_enable{false};
        double state2state_altitude_escape_band{3.5};
        // Keep the guide independent from a potentially overshooting command
        // trajectory. The corridor backend uses the guide only as a soft
        // altitude-floor reference; it never rejects a feasible trajectory
        // merely for following a different timing profile.
        bool state2state_replan_use_command_guide{true};
        bool state2state_over_goal_guard_enable{true};
        double state2state_over_goal_tolerance{0.25};
        double state2state_near_goal_radius{2.0};
        bool state2state_accept_exp_without_backup_near_goal{true};
        bool unknown_goal_reveal_en{true};
        int iris_iter_num;
        std::string ellipsoid_optimizer{"classic"};
        bool ellipsoid_optimizer_fallback{false};

        int mpc_horizon{};

        double yaw_dot_max;
        // Yaw mode: 1 heading to velocity, 2 heading to goal
        int yaw_mode = YAW_TO_VEL;

        bool se3_aggressive_enable{true};
        int se3_piece_num{4};
        double se3_reference_speed{3.0};
        double se3_min_duration{0.5};
        double se3_max_duration{8.0};
        double se3_horiz_half_len{0.35};
        double se3_vert_half_len{0.12};
        double se3_safe_margin{0.05};
        double se3_max_vel{8.0};
        double se3_thrust_acc_min{4.0};
        double se3_thrust_acc_max{20.0};
        double se3_body_rate_max{5.0};
        double se3_yaw_rate_max{3.0};
        double se3_weight_time{10.0};
        double se3_weight_corridor{1.0e4};
        double se3_weight_vel{1.0e3};
        double se3_weight_thrust{1.0e3};
        double se3_weight_body_rate{1.0e3};
        bool se3_use_yaw{false};
        bool se3_yaw_heading_to_velocity{true};
        bool se3_use_corridor{true};
        bool se3_runtime_check_enable{true};
        bool se3_use_numeric_shape_gradient{true};

        double tracking_distance{2.2};
        double tracking_distance_tolerance{0.8};
        double tracking_distance_lower_tolerance{0.45};
        double tracking_distance_upper_tolerance{0.9};
        double tracking_height_offset{0.7};
        double tracking_height_tolerance{0.6};
        double tracking_safe_distance{0.35};
        double tracking_hard_safe_distance{0.22};
        bool tracking_narrow_passage_enable{true};
        double tracking_narrow_passage_clearance_threshold{0.38};
        double tracking_narrow_passage_soft_safe_distance_scale{0.65};
        double tracking_visibility_safe_distance{0.25};
        double tracking_visibility_cone_ratio{0.12};
        double tracking_visibility_angle_clearance{0.08726646259971647};
        bool tracking_adaptive_occlusion_enable{true};
        double tracking_adaptive_occlusion_activation_distance{0.25};
        double tracking_adaptive_occlusion_max_weight_scale{12.0};
        double tracking_adaptive_occlusion_recovery_oe_scale{8.0};
        double tracking_adaptive_occlusion_od_far_weight_scale{4.0};
        double tracking_adaptive_occlusion_distance_upper_scale{0.65};
        double tracking_adaptive_occlusion_min_horizontal_upper{0.85};
        bool tracking_adaptive_occlusion_postcheck_enable{true};
        double tracking_reacquire_distance{6.0};
        double tracking_min_commit_duration{0.8};
        double tracking_low_speed_velocity_threshold{0.25};
        double tracking_angular_hysteresis{0.35};
        bool tracking_runtime_manager_enable{true};
        bool tracking_anti_rollback_enable{true};
        double tracking_anti_rollback_horizon{1.0};
        double tracking_anti_rollback_dt{0.25};
        double tracking_anti_rollback_margin{0.35};
        double tracking_keep_old_horizon{1.0};
        double tracking_keep_old_safety_dt{0.05};
        bool tracking_keep_old_short_safety_grace_enable{true};
        double tracking_keep_old_short_safety_grace_horizon{0.25};
        double tracking_keep_old_min_remaining{0.45};
        double tracking_keep_old_min_speed{0.15};
        double tracking_keep_old_min_displacement{0.04};
        double tracking_keep_old_min_progress_ratio{0.10};
        double tracking_keep_old_max_tracking_error_scale{1.25};
        int tracking_max_consecutive_keep_old{2};
        bool tracking_no_motion_guard_enable{true};
        double tracking_no_motion_check_horizon{0.35};
        double tracking_no_motion_min_displacement{0.04};
        bool tracking_motion_3d_enable{true};
        double tracking_vertical_motion_threshold{0.12};
        double tracking_no_motion_min_displacement_z{0.04};
        double tracking_keep_old_min_progress_3d_ratio{0.08};
        double tracking_no_motion_target_speed_threshold{0.25};
        double tracking_commit_start_time_tolerance{0.05};
        bool tracking_detour_grace_enable{true};
        double tracking_detour_grace_horizon{1.2};
        double tracking_detour_max_tracking_error_scale{2.5};
        bool tracking_anti_rollback_eval_after_prefix{true};
        double tracking_candidate_angle_step{0.3926990817};
        int tracking_candidate_radius_num{3};
        int tracking_visibility_samples{5};
        bool tracking_recovery_enable{true};
        double tracking_recovery_horizon{1.5};
        double tracking_recovery_distance_tolerance_scale{1.8};
        double tracking_recovery_height_tolerance_scale{1.8};
        double tracking_recovery_time_scale{1.4};
        double tracking_recovery_reduce_visible_region_weight{0.3};
        double tracking_recovery_reduce_target_forward_weight{0.5};
        bool tracking_soft_recovery_enable{true};
        double tracking_soft_recovery_margin{0.2};
        double tracking_soft_recovery_hold_duration{0.8};
        bool tracking_reacquire_recovery_enable{true};
        double tracking_reacquire_recovery_horizon{0.8};
        bool tracking_reacquire_transit_enable{true};
        double tracking_reacquire_transit_horizon_scale{0.6};
        double tracking_reacquire_transit_max_horizon{2.5};
        double tracking_reacquire_visible_region_weight_scale{0.0};
        bool tracking_reacquire_fov_relax_enable{true};
        bool tracking_reacquire_fov_deferred_strict_enable{true};
        double tracking_reacquire_fov_entry_distance{6.0};
        double tracking_reacquire_min_progress_distance{0.5};
        double tracking_reacquire_min_progress_ratio{0.10};
        double tracking_reacquire_fov_range_grace{8.0};
        double tracking_reacquire_fov_angular_grace_deg{8.0};
        bool tracking_optimizer_commit_safety_precheck_enable{true};
        bool tracking_retry_without_corridor_enable{true};
        bool tracking_fallback_relax_enable{true};
        double tracking_fallback_distance_tolerance_scale{1.6};
        double tracking_fallback_height_tolerance_scale{1.5};
        int tracking_fallback_candidate_radius_extra{2};
        double tracking_fallback_candidate_angle_step_scale{0.5};
        double tracking_fallback_search_horizon_scale{1.3};
        bool tracking_frontend_elastic_enable{true};
        double tracking_frontend_elastic_distance_tolerance_scale{2.0};
        double tracking_frontend_elastic_height_tolerance_scale{2.0};
        bool tracking_frontend_partial_guide_enable{true};
        double tracking_frontend_partial_min_duration{0.45};
        int tracking_frontend_partial_min_samples{2};
        double tracking_weight_od_near{20.0};
        double tracking_weight_od_far{5.0};
        double tracking_weight_od_vertical{8.0};
        double tracking_weight_oa{5.0};
        double tracking_weight_oe{1.0};
        double tracking_weight_relative_velocity{1.0};
        double tracking_weight_tangent_velocity{5.0};
        double tracking_weight_viewpoint_attractor{50.0};
        double tracking_weight_visible_region{3.0};
        double tracking_weight_fov{20.0};
        double tracking_weight_target_forward{15.0};
        double tracking_static_distance_tolerance_scale{0.35};
        double tracking_static_height_tolerance_scale{0.5};
        double tracking_static_tangent_weight_scale{3.0};
        double tracking_static_tail_speed_epsilon{0.08};
        double tracking_fov_horizontal_deg{90.0};
        double tracking_fov_vertical_deg{60.0};
        double tracking_fov_range{4.0};
        double tracking_target_front_margin{0.15};
        bool tracking_fov_commit_check_enable{true};
        bool tracking_fov_check_strict{true};
        double tracking_fov_check_dt{0.03};
        bool tracking_fov_range_grace_enable{true};
        double tracking_fov_range_grace{0.9};
        double tracking_fov_keep_old_angular_grace_deg{5.0};
        double tracking_fov_keep_old_violation_ratio_grace{0.25};
        double tracking_fov_range_margin{0.05};
        double tracking_fov_front_margin{0.05};
        bool tracking_fov_check_first_commit{true};
        bool tracking_keep_old_requires_fov{true};
        bool tracking_frontend_fov_feasibility_enable{true};
        bool tracking_frontend_yaw_rate_feasibility_enable{true};
        double tracking_frontend_fov_range_margin{0.05};
        double tracking_frontend_yaw_rate_margin{0.10};
        bool tracking_frontend_obstacle_recovery_enable{true};
        int tracking_frontend_grid_neighbor_mode{26};
        bool tracking_frontend_over_wall_enable{true};
        double tracking_frontend_over_wall_max_climb{2.0};
        bool tracking_frontend_side_pass_enable{true};
        double tracking_frontend_side_pass_width{1.5};
        bool tracking_frontend_reacquire_relax_yaw_rate{true};
        double tracking_joint_sample_dt{0.05};
        bool tracking_dense_joint_sample_enable{true};
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
        double perching_min_duration{0.6};
        double perching_max_duration{4.0};
        double perching_reference_speed{2.0};
        double perching_relative_z_min{0.1};
        double perching_relative_z_max{3.0};
        double perching_weight_relative_height{1.0};
        double perching_visual_min_distance{0.2};
        double perching_visual_activation_distance{3.0};
        double perching_visual_fx{1.0};
        double perching_visual_fy{1.0};
        double perching_terminal_pos_tolerance{0.15};
        double perching_terminal_vel_tolerance{0.5};
        double perching_contact_time_margin{0.15};
        double perching_contact_distance_tolerance{0.12};
        int perching_max_replan_fail_keep{3};
        bool perching_rotate_surface_with_yaw_rate{true};
        bool perching_frontend_astar{true};
        bool perching_use_dynamics_terminal_accel{true};
        bool perching_contact_occupancy_allowance_enable{true};
        bool perching_contact_linefree_allowance_enable{true};
        double perching_contact_occupancy_normal_tolerance{0.20};
        double perching_contact_occupancy_tangent_margin{0.15};
        double perching_duration_margin{0.20};
        double perching_duration_tolerance{0.15};
        bool perching_allow_long_standalone{false};
        double perching_time_upper_bound_weight{5000.0};
        double perching_duration_seed_weight{50.0};
        double perching_max_piece_duration{1.2};
        int perching_min_piece_num{3};
        int perching_max_piece_num{8};
        bool perching_multi_point_guide_enable{true};
        int perching_moving_guide_sample_num{4};
        double perching_tau_f_seed_limit{1.30};
        bool perching_reset_surface_time{true};

        double takeoff_robot_l{0.28};
        double takeoff_robot_radius{0.25};
        double takeoff_platform_radius{0.35};
        double takeoff_platform_clearance{0.05};
        double takeoff_release_contact_time{0.20};
        double takeoff_escape_distance{1.0};
        double takeoff_escape_height{0.8};
        double takeoff_reference_speed{1.5};
        double takeoff_min_duration{0.6};
        double takeoff_max_duration{3.0};
        double takeoff_safe_distance{0.35};
        int takeoff_piece_num{3};
        bool takeoff_frontend_astar{true};
        bool takeoff_use_tangent_release_velocity{false};
        double takeoff_weight_eta{1.0};
        double takeoff_weight_tau_f{1.0e-3};
        double takeoff_platform_clearance_after_release{0.08};

        bool tracking_perching_enable{true};
        bool tracking_perching_auto_trigger_enable{false};
        bool tracking_perching_require_external_request{true};
        double tracking_perching_readiness_min_distance{0.5};
        double tracking_perching_readiness_max_distance{3.0};
        double tracking_perching_readiness_max_relative_speed{2.0};
        double tracking_perching_readiness_max_lateral_speed{1.5};
        double tracking_perching_readiness_max_required_duration{3.0};
        double tracking_perching_readiness_min_prediction_time{1.2};
        int tracking_perching_readiness_hold_cycles{3};
        double tracking_to_perching_handover_delay{0.0};
        double tracking_to_perching_prefix_ratio{0.4};
        double tracking_to_perching_reference_speed{2.0};
        double tracking_to_perching_max_seed_duration{3.0};
        bool tracking_to_perching_use_tracking_suffix{true};
        bool tracking_to_perching_stitch_prefix{true};
        bool perching_commit_only_after_candidate_check{true};
        bool perching_abort_to_tracking_enable{true};

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
        bool swarm_formation_enable{false};
        double swarm_formation_weight{0.0};
        int swarm_formation_num{0};
        std::vector<double> swarm_formation_offsets;
        std::vector<double> swarm_formation_start{0.0, 0.0, 0.0};
        std::vector<double> swarm_formation_end{1.0, 0.0, 0.0};
        double swarm_formation_time_horizon{5.0};
        double swarm_formation_stale_timeout{1.0};

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
            loader.LoadParam("general_planner/frontend_astar_time_out", frontend_astar_time_out, 0.1);
            loader.LoadParam("general_planner/over_wall_search_en", over_wall_search_en, true);
            loader.LoadParam("general_planner/over_wall_max_climb", over_wall_max_climb, 3.0);
            loader.LoadParam("general_planner/over_wall_height_step", over_wall_height_step, 0.3);
            loader.LoadParam("general_planner/over_wall_min_blocked_span", over_wall_min_blocked_span, 0.8);
            loader.LoadParam("general_planner/over_wall_min_progress_gain", over_wall_min_progress_gain, 0.5);
            loader.LoadParam("general_planner/over_wall_forward_ratio", over_wall_forward_ratio, 0.9);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/enable",
                             dynamic_obstacle_layer_enable, false);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/ttl",
                             dynamic_obstacle_layer_ttl, 0.35);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/voxel_size",
                             dynamic_obstacle_layer_voxel_size, 0.20);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/inflation_radius",
                             dynamic_obstacle_layer_inflation_radius, 0.35);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/line_check_step",
                             dynamic_obstacle_layer_line_check_step, 0.05);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/max_points_per_frame",
                             dynamic_obstacle_layer_max_points_per_frame, 30000);
            std::vector<double> dynamic_obstacle_local_half_size;
            loader.LoadParam("general_planner/dynamic_obstacle_layer/local_half_size",
                             dynamic_obstacle_local_half_size,
                             std::vector<double>{10.0, 10.0, 3.0});
            if (dynamic_obstacle_local_half_size.size() == 3) {
                dynamic_obstacle_layer_local_half_size =
                        general_utils::Vec3f(dynamic_obstacle_local_half_size[0],
                                           dynamic_obstacle_local_half_size[1],
                                           dynamic_obstacle_local_half_size[2]);
            }
            loader.LoadParam("general_planner/state2state/direct_line_frontend_enable",
                             state2state_direct_line_frontend_enable, true);
            loader.LoadParam("general_planner/state2state/topology/enable",
                             state2state_topology_enable, false);
            loader.LoadParam("general_planner/state2state/topology/query_enable",
                             state2state_topology_query_enable, false);
            loader.LoadParam("general_planner/state2state/topology/construction_mode",
                             state2state_topology_construction_mode,
                             std::string{"bubble_topology"});
            loader.LoadParam("general_planner/state2state/topology/unknown_as_free",
                             state2state_topology_unknown_as_free, false);
            loader.LoadParam("general_planner/state2state/topology/planar_mode",
                             state2state_topology_planar_mode, false);
            loader.LoadParam("general_planner/state2state/topology/navigation_altitude",
                             state2state_topology_navigation_altitude, -1.0e6);
            loader.LoadParam("general_planner/state2state/topology/min_query_distance",
                             state2state_topology_min_query_distance, 3.0);
            loader.LoadParam("general_planner/state2state/topology/update_budget",
                             state2state_topology_update_budget, 1);
            loader.LoadParam("general_planner/state2state/topology/update_period",
                             state2state_topology_update_period, 0.20);
            loader.LoadParam("general_planner/state2state/topology/publish_period",
                             state2state_topology_publish_period, 0.50);
            loader.LoadParam("general_planner/state2state/topology/region_size",
                             state2state_topology_region_size, 4.0);
            loader.LoadParam("general_planner/state2state/topology/sample_spacing",
                             state2state_topology_sample_spacing, 1.0);
            loader.LoadParam(
                "general_planner/state2state/topology/evidence_vertical_tolerance",
                state2state_topology_evidence_vertical_tolerance, 0.50);
            loader.LoadParam("general_planner/state2state/topology/min_clearance",
                             state2state_topology_min_clearance, 0.45);
            loader.LoadParam("general_planner/state2state/topology/max_clearance",
                             state2state_topology_max_clearance, 2.5);
            loader.LoadParam("general_planner/state2state/topology/connection_radius",
                             state2state_topology_connection_radius, 6.0);
            loader.LoadParam("general_planner/state2state/topology/dirty_padding",
                             state2state_topology_dirty_padding, 2.5);
            loader.LoadParam("general_planner/state2state/topology/bubble_overlap_margin",
                             state2state_topology_bubble_overlap_margin, 0.10);
            loader.LoadParam("general_planner/state2state/topology/max_nodes_per_region",
                             state2state_topology_max_nodes_per_region, 4);
            loader.LoadParam("general_planner/state2state/topology/max_bubbles_per_region",
                             state2state_topology_max_bubbles_per_region, 256);
            loader.LoadParam("general_planner/state2state/topology/max_neighbors",
                             state2state_topology_max_neighbors, 8);
            loader.LoadParam("general_planner/state2state/altitude_guard_enable",
                             state2state_altitude_guard_enable, false);
            loader.LoadParam("general_planner/state2state/altitude_band",
                             state2state_altitude_band, 0.25);
            loader.LoadParam("general_planner/state2state/altitude_escape_enable",
                             state2state_altitude_escape_enable, false);
            loader.LoadParam("general_planner/state2state/altitude_escape_band",
                             state2state_altitude_escape_band, 3.5);
            loader.LoadParam("general_planner/state2state/replan_use_command_guide",
                             state2state_replan_use_command_guide, true);
            loader.LoadParam("general_planner/state2state/over_goal_guard_enable",
                             state2state_over_goal_guard_enable, true);
            loader.LoadParam("general_planner/state2state/over_goal_tolerance",
                             state2state_over_goal_tolerance, 0.25);
            loader.LoadParam("general_planner/state2state/near_goal_radius",
                             state2state_near_goal_radius, 2.0);
            loader.LoadParam("general_planner/state2state/accept_exp_without_backup_near_goal",
                             state2state_accept_exp_without_backup_near_goal, true);
            loader.LoadParam("general_planner/unknown_goal_reveal_en", unknown_goal_reveal_en, true);
            loader.LoadParam("general_planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("general_planner/ellipsoid_optimizer", ellipsoid_optimizer, std::string("classic"));
            loader.LoadParam("general_planner/ellipsoid_optimizer_fallback", ellipsoid_optimizer_fallback, false);
            loader.LoadParam("general_planner/yaw_mode", yaw_mode, 1);
            loader.LoadParam("general_planner/mpc_horizon", mpc_horizon, 1);
            loader.LoadParam("general_planner/yaw_dot_max", yaw_dot_max, 3.14);
            loader.LoadParam("general_planner/se3_aggressive/enable", se3_aggressive_enable, true);
            loader.LoadParam("general_planner/se3_aggressive/piece_num", se3_piece_num, 4);
            loader.LoadParam("general_planner/se3_aggressive/reference_speed", se3_reference_speed, 3.0);
            loader.LoadParam("general_planner/se3_aggressive/min_duration", se3_min_duration, 0.5);
            loader.LoadParam("general_planner/se3_aggressive/max_duration", se3_max_duration, 8.0);
            loader.LoadParam("general_planner/se3_aggressive/horiz_half_len", se3_horiz_half_len, 0.35);
            loader.LoadParam("general_planner/se3_aggressive/vert_half_len", se3_vert_half_len, 0.12);
            loader.LoadParam("general_planner/se3_aggressive/safe_margin", se3_safe_margin, 0.05);
            loader.LoadParam("general_planner/se3_aggressive/max_vel", se3_max_vel, 8.0);
            loader.LoadParam("general_planner/se3_aggressive/thrust_acc_min", se3_thrust_acc_min, 4.0);
            loader.LoadParam("general_planner/se3_aggressive/thrust_acc_max", se3_thrust_acc_max, 20.0);
            loader.LoadParam("general_planner/se3_aggressive/body_rate_max", se3_body_rate_max, 5.0);
            loader.LoadParam("general_planner/se3_aggressive/yaw_rate_max", se3_yaw_rate_max, 3.0);
            loader.LoadParam("general_planner/se3_aggressive/weight_time", se3_weight_time, 10.0);
            loader.LoadParam("general_planner/se3_aggressive/weight_corridor", se3_weight_corridor, 1.0e4);
            loader.LoadParam("general_planner/se3_aggressive/weight_vel", se3_weight_vel, 1.0e3);
            loader.LoadParam("general_planner/se3_aggressive/weight_thrust", se3_weight_thrust, 1.0e3);
            loader.LoadParam("general_planner/se3_aggressive/weight_body_rate", se3_weight_body_rate, 1.0e3);
            loader.LoadParam("general_planner/se3_aggressive/use_yaw", se3_use_yaw, false);
            loader.LoadParam("general_planner/se3_aggressive/yaw_heading_to_velocity",
                             se3_yaw_heading_to_velocity, true);
            loader.LoadParam("general_planner/se3_aggressive/use_corridor", se3_use_corridor, true);
            loader.LoadParam("general_planner/se3_aggressive/runtime_check_enable",
                             se3_runtime_check_enable, true);
            loader.LoadParam("general_planner/se3_aggressive/use_numeric_shape_gradient",
                             se3_use_numeric_shape_gradient, true);
            loader.LoadParam("general_planner/tracking/distance", tracking_distance, 2.2);
            loader.LoadParam("general_planner/tracking/distance_tolerance", tracking_distance_tolerance, 0.8);
            loader.LoadParam("general_planner/tracking/distance_lower_tolerance",
                             tracking_distance_lower_tolerance, tracking_distance_tolerance);
            loader.LoadParam("general_planner/tracking/distance_upper_tolerance",
                             tracking_distance_upper_tolerance, tracking_distance_tolerance);
            loader.LoadParam("general_planner/tracking/height_offset", tracking_height_offset, 0.7);
            loader.LoadParam("general_planner/tracking/height_tolerance", tracking_height_tolerance, 0.6);
            loader.LoadParam("general_planner/tracking/safe_distance", tracking_safe_distance, 0.35);
            loader.LoadParam("general_planner/tracking/hard_safe_distance",
                             tracking_hard_safe_distance, 0.22);
            loader.LoadParam("general_planner/tracking/narrow_passage_enable",
                             tracking_narrow_passage_enable, true);
            loader.LoadParam("general_planner/tracking/narrow_passage_clearance_threshold",
                             tracking_narrow_passage_clearance_threshold, 0.38);
            loader.LoadParam("general_planner/tracking/narrow_passage_soft_safe_distance_scale",
                             tracking_narrow_passage_soft_safe_distance_scale, 0.65);
            loader.LoadParam("general_planner/tracking/visibility_safe_distance", tracking_visibility_safe_distance, 0.25);
            loader.LoadParam("general_planner/tracking/visibility_cone_ratio", tracking_visibility_cone_ratio, 0.12);
            loader.LoadParam("general_planner/tracking/visibility_angle_clearance",
                             tracking_visibility_angle_clearance, 0.08726646259971647);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_enable",
                             tracking_adaptive_occlusion_enable, true);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_activation_distance",
                             tracking_adaptive_occlusion_activation_distance, 0.25);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_max_weight_scale",
                             tracking_adaptive_occlusion_max_weight_scale, 12.0);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_recovery_oe_scale",
                             tracking_adaptive_occlusion_recovery_oe_scale, 8.0);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_od_far_weight_scale",
                             tracking_adaptive_occlusion_od_far_weight_scale, 4.0);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_distance_upper_scale",
                             tracking_adaptive_occlusion_distance_upper_scale, 0.65);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_min_horizontal_upper",
                             tracking_adaptive_occlusion_min_horizontal_upper, 0.85);
            loader.LoadParam("general_planner/tracking/adaptive_occlusion_postcheck_enable",
                             tracking_adaptive_occlusion_postcheck_enable, true);
            loader.LoadParam("general_planner/tracking/reacquire_distance", tracking_reacquire_distance, 6.0);
            loader.LoadParam("general_planner/tracking/min_commit_duration", tracking_min_commit_duration, 0.8);
            loader.LoadParam("general_planner/tracking/low_speed_velocity_threshold",
                             tracking_low_speed_velocity_threshold, 0.25);
            loader.LoadParam("general_planner/tracking/angular_hysteresis", tracking_angular_hysteresis, 0.35);
            loader.LoadParam("general_planner/tracking/runtime_manager_enable",
                             tracking_runtime_manager_enable, true);
            loader.LoadParam("general_planner/tracking/anti_rollback_enable",
                             tracking_anti_rollback_enable, true);
            loader.LoadParam("general_planner/tracking/anti_rollback_horizon",
                             tracking_anti_rollback_horizon, 1.0);
            loader.LoadParam("general_planner/tracking/anti_rollback_dt", tracking_anti_rollback_dt, 0.25);
            loader.LoadParam("general_planner/tracking/anti_rollback_margin",
                             tracking_anti_rollback_margin, 0.35);
            loader.LoadParam("general_planner/tracking/keep_old_horizon", tracking_keep_old_horizon, 1.0);
            loader.LoadParam("general_planner/tracking/keep_old_safety_dt", tracking_keep_old_safety_dt, 0.05);
            loader.LoadParam("general_planner/tracking/keep_old_short_safety_grace_enable",
                             tracking_keep_old_short_safety_grace_enable, true);
            loader.LoadParam("general_planner/tracking/keep_old_short_safety_grace_horizon",
                             tracking_keep_old_short_safety_grace_horizon, 0.25);
            loader.LoadParam("general_planner/tracking/keep_old_min_remaining",
                             tracking_keep_old_min_remaining, 0.45);
            loader.LoadParam("general_planner/tracking/keep_old_min_speed",
                             tracking_keep_old_min_speed, 0.15);
            loader.LoadParam("general_planner/tracking/keep_old_min_displacement",
                             tracking_keep_old_min_displacement, 0.04);
            loader.LoadParam("general_planner/tracking/keep_old_min_progress_ratio",
                             tracking_keep_old_min_progress_ratio, 0.10);
            loader.LoadParam("general_planner/tracking/keep_old_max_tracking_error_scale",
                             tracking_keep_old_max_tracking_error_scale, 1.25);
            loader.LoadParam("general_planner/tracking/max_consecutive_keep_old",
                             tracking_max_consecutive_keep_old, 2);
            loader.LoadParam("general_planner/tracking/no_motion_guard_enable",
                             tracking_no_motion_guard_enable, true);
            loader.LoadParam("general_planner/tracking/no_motion_check_horizon",
                             tracking_no_motion_check_horizon, 0.35);
            loader.LoadParam("general_planner/tracking/no_motion_min_displacement",
                             tracking_no_motion_min_displacement, 0.04);
            loader.LoadParam("general_planner/tracking/motion_3d_enable",
                             tracking_motion_3d_enable, true);
            loader.LoadParam("general_planner/tracking/vertical_motion_threshold",
                             tracking_vertical_motion_threshold, 0.12);
            loader.LoadParam("general_planner/tracking/no_motion_min_displacement_z",
                             tracking_no_motion_min_displacement_z, 0.04);
            loader.LoadParam("general_planner/tracking/keep_old_min_progress_3d_ratio",
                             tracking_keep_old_min_progress_3d_ratio, 0.08);
            loader.LoadParam("general_planner/tracking/no_motion_target_speed_threshold",
                             tracking_no_motion_target_speed_threshold, 0.25);
            loader.LoadParam("general_planner/tracking/commit_start_time_tolerance",
                             tracking_commit_start_time_tolerance, 0.05);
            loader.LoadParam("general_planner/tracking/detour_grace_enable",
                             tracking_detour_grace_enable, true);
            loader.LoadParam("general_planner/tracking/detour_grace_horizon",
                             tracking_detour_grace_horizon, 1.2);
            loader.LoadParam("general_planner/tracking/detour_max_tracking_error_scale",
                             tracking_detour_max_tracking_error_scale, 2.5);
            loader.LoadParam("general_planner/tracking/anti_rollback_eval_after_prefix",
                             tracking_anti_rollback_eval_after_prefix, true);
            loader.LoadParam("general_planner/tracking/candidate_angle_step", tracking_candidate_angle_step, 0.3926990817);
            loader.LoadParam("general_planner/tracking/candidate_radius_num", tracking_candidate_radius_num, 3);
            loader.LoadParam("general_planner/tracking/visibility_samples", tracking_visibility_samples, 5);
            loader.LoadParam("general_planner/tracking/recovery_enable",
                             tracking_recovery_enable, true);
            loader.LoadParam("general_planner/tracking/recovery_horizon",
                             tracking_recovery_horizon, 1.5);
            loader.LoadParam("general_planner/tracking/recovery_distance_tolerance_scale",
                             tracking_recovery_distance_tolerance_scale, 1.8);
            loader.LoadParam("general_planner/tracking/recovery_height_tolerance_scale",
                             tracking_recovery_height_tolerance_scale, 1.8);
            loader.LoadParam("general_planner/tracking/recovery_time_scale",
                             tracking_recovery_time_scale, 1.4);
            loader.LoadParam("general_planner/tracking/recovery_reduce_visible_region_weight",
                             tracking_recovery_reduce_visible_region_weight, 0.3);
            loader.LoadParam("general_planner/tracking/recovery_reduce_target_forward_weight",
                             tracking_recovery_reduce_target_forward_weight, 0.5);
            loader.LoadParam("general_planner/tracking/soft_recovery_enable",
                             tracking_soft_recovery_enable, true);
            loader.LoadParam("general_planner/tracking/soft_recovery_margin",
                             tracking_soft_recovery_margin, 0.2);
            loader.LoadParam("general_planner/tracking/soft_recovery_hold_duration",
                             tracking_soft_recovery_hold_duration, 0.8);
            loader.LoadParam("general_planner/tracking/reacquire_recovery_enable",
                             tracking_reacquire_recovery_enable, true);
            loader.LoadParam("general_planner/tracking/reacquire_recovery_horizon",
                             tracking_reacquire_recovery_horizon, 0.8);
            loader.LoadParam("general_planner/tracking/reacquire_transit_enable",
                             tracking_reacquire_transit_enable, true);
            loader.LoadParam("general_planner/tracking/reacquire_transit_horizon_scale",
                             tracking_reacquire_transit_horizon_scale, 0.6);
            loader.LoadParam("general_planner/tracking/reacquire_transit_max_horizon",
                             tracking_reacquire_transit_max_horizon, 2.5);
            loader.LoadParam("general_planner/tracking/reacquire_visible_region_weight_scale",
                             tracking_reacquire_visible_region_weight_scale, 0.0);
            loader.LoadParam("general_planner/tracking/reacquire_fov_relax_enable",
                             tracking_reacquire_fov_relax_enable, true);
            loader.LoadParam("general_planner/tracking/reacquire_fov_deferred_strict_enable",
                             tracking_reacquire_fov_deferred_strict_enable, true);
            loader.LoadParam("general_planner/tracking/reacquire_fov_entry_distance",
                             tracking_reacquire_fov_entry_distance, 6.0);
            loader.LoadParam("general_planner/tracking/reacquire_min_progress_distance",
                             tracking_reacquire_min_progress_distance, 0.5);
            loader.LoadParam("general_planner/tracking/reacquire_min_progress_ratio",
                             tracking_reacquire_min_progress_ratio, 0.10);
            loader.LoadParam("general_planner/tracking/reacquire_fov_range_grace",
                             tracking_reacquire_fov_range_grace, 8.0);
            loader.LoadParam("general_planner/tracking/reacquire_fov_angular_grace_deg",
                             tracking_reacquire_fov_angular_grace_deg, 8.0);
            loader.LoadParam("general_planner/tracking/optimizer_commit_safety_precheck_enable",
                             tracking_optimizer_commit_safety_precheck_enable, true);
            loader.LoadParam("general_planner/tracking/retry_without_corridor_enable",
                             tracking_retry_without_corridor_enable, true);
            loader.LoadParam("general_planner/tracking/fallback_relax_enable",
                             tracking_fallback_relax_enable, true);
            loader.LoadParam("general_planner/tracking/fallback_distance_tolerance_scale",
                             tracking_fallback_distance_tolerance_scale, 1.6);
            loader.LoadParam("general_planner/tracking/fallback_height_tolerance_scale",
                             tracking_fallback_height_tolerance_scale, 1.5);
            loader.LoadParam("general_planner/tracking/fallback_candidate_radius_extra",
                             tracking_fallback_candidate_radius_extra, 2);
            loader.LoadParam("general_planner/tracking/fallback_candidate_angle_step_scale",
                             tracking_fallback_candidate_angle_step_scale, 0.5);
            loader.LoadParam("general_planner/tracking/fallback_search_horizon_scale",
                             tracking_fallback_search_horizon_scale, 1.3);
            loader.LoadParam("general_planner/tracking/frontend_elastic_enable",
                             tracking_frontend_elastic_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_elastic_distance_tolerance_scale",
                             tracking_frontend_elastic_distance_tolerance_scale, 2.0);
            loader.LoadParam("general_planner/tracking/frontend_elastic_height_tolerance_scale",
                             tracking_frontend_elastic_height_tolerance_scale, 2.0);
            loader.LoadParam("general_planner/tracking/frontend_partial_guide_enable",
                             tracking_frontend_partial_guide_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_partial_min_duration",
                             tracking_frontend_partial_min_duration, 0.45);
            loader.LoadParam("general_planner/tracking/frontend_partial_min_samples",
                             tracking_frontend_partial_min_samples, 2);
            loader.LoadParam("general_planner/tracking/weight_od_near", tracking_weight_od_near, 20.0);
            loader.LoadParam("general_planner/tracking/weight_od_far", tracking_weight_od_far, 5.0);
            loader.LoadParam("general_planner/tracking/weight_od_vertical", tracking_weight_od_vertical, 8.0);
            loader.LoadParam("general_planner/tracking/weight_oa", tracking_weight_oa, 5.0);
            loader.LoadParam("general_planner/tracking/weight_oe", tracking_weight_oe, 1.0);
            loader.LoadParam("general_planner/tracking/weight_relative_velocity", tracking_weight_relative_velocity, 1.0);
            loader.LoadParam("general_planner/tracking/weight_tangent_velocity", tracking_weight_tangent_velocity, 5.0);
            loader.LoadParam("general_planner/tracking/weight_viewpoint_attractor", tracking_weight_viewpoint_attractor, 50.0);
            loader.LoadParam("general_planner/tracking/weight_visible_region", tracking_weight_visible_region, 3.0);
            loader.LoadParam("general_planner/tracking/weight_fov", tracking_weight_fov, 20.0);
            loader.LoadParam("general_planner/tracking/weight_target_forward", tracking_weight_target_forward, 15.0);
            loader.LoadParam("general_planner/tracking/static_distance_tolerance_scale",
                             tracking_static_distance_tolerance_scale, 0.35);
            loader.LoadParam("general_planner/tracking/static_height_tolerance_scale",
                             tracking_static_height_tolerance_scale, 0.5);
            loader.LoadParam("general_planner/tracking/static_tangent_weight_scale",
                             tracking_static_tangent_weight_scale, 3.0);
            loader.LoadParam("general_planner/tracking/static_tail_speed_epsilon",
                             tracking_static_tail_speed_epsilon, 0.08);
            loader.LoadParam("general_planner/tracking/fov_horizontal_deg", tracking_fov_horizontal_deg, 90.0);
            loader.LoadParam("general_planner/tracking/fov_vertical_deg", tracking_fov_vertical_deg, 60.0);
            loader.LoadParam("general_planner/tracking/fov_range", tracking_fov_range, 4.0);
            loader.LoadParam("general_planner/tracking/target_front_margin", tracking_target_front_margin, 0.15);
            loader.LoadParam("general_planner/tracking/fov_commit_check_enable",
                             tracking_fov_commit_check_enable, true);
            loader.LoadParam("general_planner/tracking/fov_check_strict",
                             tracking_fov_check_strict, true);
            loader.LoadParam("general_planner/tracking/fov_check_dt",
                             tracking_fov_check_dt, 0.03);
            loader.LoadParam("general_planner/tracking/fov_range_grace_enable",
                             tracking_fov_range_grace_enable, true);
            loader.LoadParam("general_planner/tracking/fov_range_grace",
                             tracking_fov_range_grace, 0.9);
            loader.LoadParam("general_planner/tracking/fov_keep_old_angular_grace_deg",
                             tracking_fov_keep_old_angular_grace_deg, 5.0);
            loader.LoadParam("general_planner/tracking/fov_keep_old_violation_ratio_grace",
                             tracking_fov_keep_old_violation_ratio_grace, 0.25);
            loader.LoadParam("general_planner/tracking/fov_range_margin",
                             tracking_fov_range_margin, 0.05);
            loader.LoadParam("general_planner/tracking/fov_front_margin",
                             tracking_fov_front_margin, 0.05);
            loader.LoadParam("general_planner/tracking/fov_check_first_commit",
                             tracking_fov_check_first_commit, true);
            loader.LoadParam("general_planner/tracking/keep_old_requires_fov",
                             tracking_keep_old_requires_fov, true);
            loader.LoadParam("general_planner/tracking/frontend_fov_feasibility_enable",
                             tracking_frontend_fov_feasibility_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_yaw_rate_feasibility_enable",
                             tracking_frontend_yaw_rate_feasibility_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_fov_range_margin",
                             tracking_frontend_fov_range_margin, 0.05);
            loader.LoadParam("general_planner/tracking/frontend_yaw_rate_margin",
                             tracking_frontend_yaw_rate_margin, 0.10);
            loader.LoadParam("general_planner/tracking/frontend_obstacle_recovery_enable",
                             tracking_frontend_obstacle_recovery_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_grid_neighbor_mode",
                             tracking_frontend_grid_neighbor_mode, 26);
            loader.LoadParam("general_planner/tracking/frontend_over_wall_enable",
                             tracking_frontend_over_wall_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_over_wall_max_climb",
                             tracking_frontend_over_wall_max_climb, 2.0);
            loader.LoadParam("general_planner/tracking/frontend_side_pass_enable",
                             tracking_frontend_side_pass_enable, true);
            loader.LoadParam("general_planner/tracking/frontend_side_pass_width",
                             tracking_frontend_side_pass_width, 1.5);
            loader.LoadParam("general_planner/tracking/frontend_reacquire_relax_yaw_rate",
                             tracking_frontend_reacquire_relax_yaw_rate, true);
            loader.LoadParam("general_planner/tracking/joint_sample_dt",
                             tracking_joint_sample_dt, 0.05);
            loader.LoadParam("general_planner/tracking/dense_joint_sample_enable",
                             tracking_dense_joint_sample_enable, true);
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
            loader.LoadParam("general_planner/perching/min_duration", perching_min_duration, 0.6);
            loader.LoadParam("general_planner/perching/max_duration", perching_max_duration, 4.0);
            loader.LoadParam("general_planner/perching/reference_speed", perching_reference_speed, 2.0);
            loader.LoadParam("general_planner/perching/relative_z_min", perching_relative_z_min, 0.1);
            loader.LoadParam("general_planner/perching/relative_z_max", perching_relative_z_max, 3.0);
            loader.LoadParam("general_planner/perching/weight_relative_height",
                             perching_weight_relative_height, 1.0);
            loader.LoadParam("general_planner/perching/visual_min_distance",
                             perching_visual_min_distance, 0.2);
            loader.LoadParam("general_planner/perching/visual_activation_distance",
                             perching_visual_activation_distance, 3.0);
            loader.LoadParam("general_planner/perching/visual_fx", perching_visual_fx, 1.0);
            loader.LoadParam("general_planner/perching/visual_fy", perching_visual_fy, 1.0);
            loader.LoadParam("general_planner/perching/terminal_pos_tolerance",
                             perching_terminal_pos_tolerance, 0.15);
            loader.LoadParam("general_planner/perching/terminal_vel_tolerance",
                             perching_terminal_vel_tolerance, 0.5);
            loader.LoadParam("general_planner/perching/contact_time_margin",
                             perching_contact_time_margin, 0.15);
            loader.LoadParam("general_planner/perching/contact_distance_tolerance",
                             perching_contact_distance_tolerance, 0.12);
            loader.LoadParam("general_planner/perching/max_replan_fail_keep",
                             perching_max_replan_fail_keep, 3);
            loader.LoadParam("general_planner/perching/rotate_surface_with_yaw_rate",
                             perching_rotate_surface_with_yaw_rate, true);
            loader.LoadParam("general_planner/perching/frontend_astar", perching_frontend_astar, true);
            loader.LoadParam("general_planner/perching/use_dynamics_terminal_accel",
                             perching_use_dynamics_terminal_accel, true);
            loader.LoadParam("general_planner/perching/contact_occupancy_allowance_enable",
                             perching_contact_occupancy_allowance_enable, true);
            loader.LoadParam("general_planner/perching/contact_linefree_allowance_enable",
                             perching_contact_linefree_allowance_enable, true);
            loader.LoadParam("general_planner/perching/contact_occupancy_normal_tolerance",
                             perching_contact_occupancy_normal_tolerance, 0.20);
            loader.LoadParam("general_planner/perching/contact_occupancy_tangent_margin",
                             perching_contact_occupancy_tangent_margin, 0.15);
            loader.LoadParam("general_planner/perching/duration_margin",
                             perching_duration_margin, 0.20);
            loader.LoadParam("general_planner/perching/duration_tolerance",
                             perching_duration_tolerance, 0.15);
            loader.LoadParam("general_planner/perching/allow_long_standalone",
                             perching_allow_long_standalone, false);
            loader.LoadParam("general_planner/perching/time_upper_bound_weight",
                             perching_time_upper_bound_weight, 5000.0);
            loader.LoadParam("general_planner/perching/duration_seed_weight",
                             perching_duration_seed_weight, 50.0);
            loader.LoadParam("general_planner/perching/max_piece_duration",
                             perching_max_piece_duration, 1.2);
            loader.LoadParam("general_planner/perching/min_piece_num",
                             perching_min_piece_num, 3);
            loader.LoadParam("general_planner/perching/max_piece_num",
                             perching_max_piece_num, 8);
            loader.LoadParam("general_planner/perching/multi_point_guide_enable",
                             perching_multi_point_guide_enable, true);
            loader.LoadParam("general_planner/perching/moving_guide_sample_num",
                             perching_moving_guide_sample_num, 4);
            loader.LoadParam("general_planner/perching/tau_f_seed_limit",
                             perching_tau_f_seed_limit, 1.30);
            loader.LoadParam("general_planner/perching/reset_surface_time",
                             perching_reset_surface_time, true);
            loader.LoadParam("general_planner/takeoff/robot_l",
                             takeoff_robot_l, 0.28);
            loader.LoadParam("general_planner/takeoff/robot_radius",
                             takeoff_robot_radius, 0.25);
            loader.LoadParam("general_planner/takeoff/platform_radius",
                             takeoff_platform_radius, 0.35);
            loader.LoadParam("general_planner/takeoff/platform_clearance",
                             takeoff_platform_clearance, 0.05);
            loader.LoadParam("general_planner/takeoff/release_contact_time",
                             takeoff_release_contact_time, 0.20);
            loader.LoadParam("general_planner/takeoff/escape_distance",
                             takeoff_escape_distance, 1.0);
            loader.LoadParam("general_planner/takeoff/escape_height",
                             takeoff_escape_height, 0.8);
            loader.LoadParam("general_planner/takeoff/reference_speed",
                             takeoff_reference_speed, 1.5);
            loader.LoadParam("general_planner/takeoff/min_duration",
                             takeoff_min_duration, 0.6);
            loader.LoadParam("general_planner/takeoff/max_duration",
                             takeoff_max_duration, 3.0);
            loader.LoadParam("general_planner/takeoff/safe_distance",
                             takeoff_safe_distance, 0.35);
            loader.LoadParam("general_planner/takeoff/piece_num",
                             takeoff_piece_num, 3);
            loader.LoadParam("general_planner/takeoff/frontend_astar",
                             takeoff_frontend_astar, true);
            loader.LoadParam("general_planner/takeoff/use_tangent_release_velocity",
                             takeoff_use_tangent_release_velocity, false);
            loader.LoadParam("general_planner/takeoff/weight_eta",
                             takeoff_weight_eta, 1.0);
            loader.LoadParam("general_planner/takeoff/weight_tau_f",
                             takeoff_weight_tau_f, 1.0e-3);
            loader.LoadParam("general_planner/takeoff/platform_clearance_after_release",
                             takeoff_platform_clearance_after_release, 0.08);
            loader.LoadParam("general_planner/tracking_perching/enable",
                             tracking_perching_enable, true);
            loader.LoadParam("general_planner/tracking_perching/auto_trigger_enable",
                             tracking_perching_auto_trigger_enable, false);
            loader.LoadParam("general_planner/tracking_perching/require_external_request",
                             tracking_perching_require_external_request, true);
            loader.LoadParam("general_planner/tracking_perching/readiness_min_distance",
                             tracking_perching_readiness_min_distance, 0.5);
            loader.LoadParam("general_planner/tracking_perching/readiness_max_distance",
                             tracking_perching_readiness_max_distance, 3.0);
            loader.LoadParam("general_planner/tracking_perching/readiness_max_relative_speed",
                             tracking_perching_readiness_max_relative_speed, 2.0);
            loader.LoadParam("general_planner/tracking_perching/readiness_max_lateral_speed",
                             tracking_perching_readiness_max_lateral_speed, 1.5);
            loader.LoadParam("general_planner/tracking_perching/readiness_max_required_duration",
                             tracking_perching_readiness_max_required_duration, 3.0);
            loader.LoadParam("general_planner/tracking_perching/readiness_min_prediction_time",
                             tracking_perching_readiness_min_prediction_time, 1.2);
            loader.LoadParam("general_planner/tracking_perching/readiness_hold_cycles",
                             tracking_perching_readiness_hold_cycles, 3);
            loader.LoadParam("general_planner/tracking_perching/handover_delay",
                             tracking_to_perching_handover_delay, 0.0);
            loader.LoadParam("general_planner/tracking_perching/prefix_ratio",
                             tracking_to_perching_prefix_ratio, 0.4);
            loader.LoadParam("general_planner/tracking_perching/reference_speed",
                             tracking_to_perching_reference_speed, 2.0);
            loader.LoadParam("general_planner/tracking_perching/max_seed_duration",
                             tracking_to_perching_max_seed_duration, 3.0);
            loader.LoadParam("general_planner/tracking_perching/use_tracking_suffix",
                             tracking_to_perching_use_tracking_suffix, true);
            loader.LoadParam("general_planner/tracking_perching/stitch_prefix",
                             tracking_to_perching_stitch_prefix, true);
            loader.LoadParam("general_planner/tracking_perching/commit_only_after_candidate_check",
                             perching_commit_only_after_candidate_check, true);
            loader.LoadParam("general_planner/tracking_perching/abort_to_tracking_enable",
                             perching_abort_to_tracking_enable, true);
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
            loader.LoadParam("general_planner/swarm/formation/enable", swarm_formation_enable, false);
            loader.LoadParam("general_planner/swarm/formation/weight", swarm_formation_weight, 0.0);
            loader.LoadParam("general_planner/swarm/formation/num", swarm_formation_num, 0);
            loader.LoadParam("general_planner/swarm/formation/offsets",
                             swarm_formation_offsets, std::vector<double>{});
            loader.LoadParam("general_planner/swarm/formation/start",
                             swarm_formation_start, std::vector<double>{0.0, 0.0, 0.0});
            loader.LoadParam("general_planner/swarm/formation/end",
                             swarm_formation_end, std::vector<double>{1.0, 0.0, 0.0});
            loader.LoadParam("general_planner/swarm/formation/time_horizon",
                             swarm_formation_time_horizon, 5.0);
            loader.LoadParam("general_planner/swarm/formation/stale_timeout",
                             swarm_formation_stale_timeout, 1.0);

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
