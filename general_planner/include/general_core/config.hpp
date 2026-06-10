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
#include <map_manager/map_backend.hpp>
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
        MapBackend astar_backend{MapBackend::ROG};
        MapBackend corridor_backend{MapBackend::ROG};
        double over_wall_max_climb{3.0};
        double over_wall_height_step{0.3};
        double over_wall_min_blocked_span{0.8};
        double over_wall_min_progress_gain{0.5};
        double over_wall_forward_ratio{0.9};
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

        bool exploration_enable{false};
        std::string exploration_frontend_type{"epic_original"};
        std::string exploration_cloud_frame{"WORLD"};
        bool exploration_use_epic_frontend{true};
        bool exploration_update_rog_map{false};
        bool exploration_backup_traj_enable{false};
        bool exploration_backend_fallback_enable{false};
        bool exploration_print_log{true};
        bool exploration_epic_lio_publish_map{true};
        double exploration_epic_lio_publish_map_period{0.5};
        double exploration_epic_lio_self_filter_radius{0.8};
        double exploration_lidar_pitch_deg{40.0};
        double exploration_lidar_fov_up_deg{52.0};
        double exploration_lidar_fov_down_deg{-7.0};
        double exploration_lidar_viewpoint_fov_up_deg{48.0};
        double exploration_lidar_viewpoint_fov_down_deg{-5.0};
        double exploration_lidar_max_ray_length{16.0};

        double exploration_observation_resolution{0.25};
        double exploration_observation_min_distance{0.2};
        double exploration_observation_well_distance{4.0};
        double exploration_observation_max_distance{12.0};
        double exploration_observation_good_force_trust_length{1.5};
        double exploration_observation_good_trust_length{4.0};
        double exploration_observation_good_direction_score{0.5};
        int exploration_observation_cloud_downsample_step{1};
        int exploration_observation_max_points_per_update{12000};
        double exploration_observation_frontier_cluster_radius{0.65};
        double exploration_observation_frontier_normal_similarity{0.35};
        int exploration_observation_min_frontier_cluster_size{8};
        double exploration_frontier_cluster_min_radius{1.8};
        double exploration_frontier_cluster_min_size{2.0};
        double exploration_frontier_cluster_max_size{8.0};
        double exploration_frontier_cluster_direction_radius{0.0};
        int exploration_frontier_cluster_minimum_point_num{10};
        std::vector<double> exploration_observation_bbox_min{-50.0, -50.0, -2.0};
        std::vector<double> exploration_observation_bbox_max{50.0, 50.0, 10.0};

        double exploration_frontier_association_distance{1.2};
        double exploration_frontier_bbox_overlap_min_ratio{0.08};
        int exploration_frontier_max_failed_count{3};
        int exploration_frontier_max_selected_count_without_gain{4};
        double exploration_frontier_blacklist_time{8.0};
        double exploration_frontier_covered_gain_threshold{4.0};
        double exploration_frontier_missing_timeout{2.0};
        double exploration_frontier_dormant_time{4.0};
        double exploration_frontier_visited_viewpoint_radius{1.5};
        double exploration_frontier_visited_viewpoint_penalty{600.0};
        int exploration_frontier_max_visited_viewpoints{400};

        double exploration_viewpoint_min_distance{1.4};
        double exploration_viewpoint_max_distance{4.0};
        int exploration_viewpoint_radius_samples{3};
        int exploration_viewpoint_yaw_samples{16};
        int exploration_viewpoint_height_samples{3};
        double exploration_viewpoint_height_step{0.6};
        double exploration_viewpoint_sample_pillar_min_height{-2.0};
        double exploration_viewpoint_sample_pillar_max_height{2.5};
        double exploration_viewpoint_sample_pillar_min_radius{1.0};
        double exploration_viewpoint_sample_pillar_max_radius{4.0};
        int exploration_viewpoint_sample_pillar_height_layer_num{5};
        int exploration_viewpoint_sample_pillar_radius_layer_num{8};
        int exploration_viewpoint_sample_pillar_circle_sample_num{4};
        int exploration_viewpoint_local_tsp_size{10};
        double exploration_viewpoint_safe_distance{0.45};
        double exploration_viewpoint_sensor_range{7.0};
        double exploration_viewpoint_horizontal_fov_deg{90.0};
        double exploration_viewpoint_vertical_fov_deg{60.0};
        double exploration_viewpoint_normal_dot_min{0.25};
        int exploration_viewpoint_max_cells_per_gain_eval{260};
        double exploration_viewpoint_line_of_sight_step{0.20};
        double exploration_viewpoint_min_gain{3.0};
        bool exploration_viewpoint_use_local_map_safety{false};
        bool exploration_viewpoint_cluster_by_visibility_sphere{true};
        bool exploration_viewpoint_use_topo_reachability_filter{true};
        int exploration_viewpoint_max_clusters{8};
        double exploration_viewpoint_cluster_connectivity_scale{1.0};
        double exploration_viewpoint_topo_reachability_timeout{0.03};
        int exploration_viewpoint_epic_yaw_bins{8};

        double exploration_topo_history_node_min_distance{1.0};
        double exploration_topo_connect_radius{8.0};
        double exploration_topo_local_edge_astar_timeout{0.05};
        double exploration_topo_global_edge_max_length{14.0};
        int exploration_topo_max_history_nodes{500};
        bool exploration_topo_use_local_astar_for_edges{true};
        bool exploration_topo_use_global_line_free_for_edges{true};
        double exploration_topo_global_line_safe_distance{0.45};
        double exploration_topo_global_line_step{0.25};
        bool exploration_topo_use_parallel_bubble_astar_for_edges{false};
        double exploration_topo_bubble_astar_resolution{0.5};
        double exploration_topo_bubble_astar_safe_distance{0.45};
        int exploration_topo_bubble_astar_max_nodes{8000};
        bool exploration_topo_use_epic_region_bubble_graph{true};
        double exploration_topo_region_size_xy{4.0};
        double exploration_topo_region_size_z{2.0};
        double exploration_topo_min_subregion_size_xy{0.5};
        double exploration_topo_min_subregion_size_z{0.5};
        double exploration_topo_bubble_min_radius{0.5};
        double exploration_topo_frontier_bubble_min_radius{0.5};
        double exploration_topo_cube_discrete_size{0.3};
        int exploration_topo_max_update_region_num{20};
        int exploration_topo_neighbor_mode{26};
        double exploration_topo_edge_search_padding_scale{1.0};
        double exploration_topo_edge_safe_distance{0.45};
        int exploration_topo_max_edges_per_node{10};
        int exploration_topo_max_region_edges_per_node{8};
        int exploration_topo_max_frontier_edges_per_node{4};
        int exploration_topo_max_history_edges_per_node{3};
        int exploration_topo_max_candidate_neighbors{24};

        bool exploration_global_guidance_enable{true};
        int exploration_global_guidance_max_frontiers_in_tour{16};
        double exploration_global_guidance_weight_path_cost{1.0};
        double exploration_global_guidance_weight_gain{1.0};
        double exploration_global_guidance_weight_revisit{0.5};
        bool exploration_global_guidance_use_two_opt{true};
        bool exploration_global_guidance_keep_current_target{true};
        bool exploration_global_guidance_use_lkh{true};
        bool exploration_global_guidance_lkh_fallback_to_two_opt{true};
        std::string exploration_global_guidance_tsp_dir{"/tmp/general_planner_tsp"};
        std::string exploration_global_guidance_tsp_problem_name{"general_planner_global"};
        std::string exploration_global_guidance_lkh_executable;
        int exploration_global_guidance_lkh_cost_scale{100};

        double exploration_route_waypoint_lookahead{3.0};
        double exploration_route_waypoint_min_distance{1.0};
        double exploration_route_waypoint_max_distance{5.0};
        double exploration_local_goal_safe_distance{0.35};
        double exploration_route_replan_distance_threshold{1.0};
        bool exploration_global_route_use_local_map_safety{false};
        double exploration_runtime_final_goal_radius{0.7};
        double exploration_runtime_route_progress_min{0.5};
        int exploration_runtime_max_local_segment_fail_count{3};
        bool exploration_runtime_keep_active_target{true};
        double exploration_runtime_switch_score_ratio{1.25};
        double exploration_runtime_global_update_dt{0.2};
        double exploration_runtime_replan_time_after_traj_start{0.5};
        double exploration_runtime_replan_time_before_traj_end{0.5};
        double exploration_runtime_safety_check_dt{0.15};
        double exploration_runtime_safety_check_horizon{2.0};
        double exploration_runtime_stop_traj_time{0.2};
        double exploration_runtime_collision_replan_time{0.5};
        double exploration_local_guide_lookahead{4.0};
        double exploration_local_guide_min_distance{1.0};
        double exploration_local_guide_planning_horizon{8.0};
        double exploration_local_guide_max_segment_length{1.0};
        double exploration_local_guide_safe_distance{0.45};
        double exploration_local_guide_start_safe_distance{0.20};
        double exploration_local_guide_line_step{0.20};
        bool exploration_local_guide_shortcut_enable{true};
        bool exploration_local_guide_astar_repair_enable{true};
        bool exploration_local_guide_unknown_as_occupied{false};
        MapBackend exploration_local_guide_backend{MapBackend::HYBRID};
        int exploration_stuck_repeated_goal_threshold{3};
        double exploration_stuck_repeated_goal_distance{0.5};
        double exploration_stuck_min_robot_displacement{0.5};
        double exploration_stuck_min_explored_volume_gain{1.0};

        bool global_exploration_map_enable{false};
        double global_exploration_map_resolution{0.20};
        double global_exploration_map_raycast_step{0.10};
        double global_exploration_map_max_range{8.0};
        double global_exploration_map_min_range{0.20};
        std::vector<double> global_exploration_map_bbox_min{-50.0, -50.0, -2.0};
        std::vector<double> global_exploration_map_bbox_max{50.0, 50.0, 10.0};
        int global_exploration_map_occupied_hit_threshold{2};
        int global_exploration_map_free_miss_threshold{1};
        double global_exploration_map_update_min_interval{0.2};
        int global_exploration_map_cloud_downsample_step{1};
        int global_exploration_map_max_points_per_update{8000};

        bool global_pointcloud_map_enable{false};
        double global_pointcloud_map_voxel_size{0.10};
        std::string global_pointcloud_map_save_path{"/tmp/explored_global_map.pcd"};
        bool global_pointcloud_map_crop_enable{false};
        std::vector<double> global_pointcloud_map_crop_min{-50.0, -50.0, -2.0};
        std::vector<double> global_pointcloud_map_crop_max{50.0, 50.0, 10.0};

        bool global_region_grid_enable{false};
        double global_region_grid_region_size_xy{4.0};
        double global_region_grid_region_size_z{2.0};
        int global_region_grid_min_frontier_count{10};
        double global_region_grid_explored_ratio_threshold{0.85};

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
        double tracking_keep_old_safety_dt{0.10};
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
            std::string astar_backend_name{"rog"};
            std::string corridor_backend_name{"rog"};
            loader.LoadParam("general_planner/astar_backend", astar_backend_name, std::string("rog"));
            loader.LoadParam("general_planner/corridor_backend", corridor_backend_name, std::string("rog"));
            loader.LoadParam("general_planner/exploration/astar_backend", astar_backend_name, astar_backend_name);
            loader.LoadParam("general_planner/exploration/corridor_backend", corridor_backend_name, corridor_backend_name);
            astar_backend = mapBackendFromString(astar_backend_name);
            corridor_backend = mapBackendFromString(corridor_backend_name);
            loader.LoadParam("general_planner/over_wall_search_en", over_wall_search_en, true);
            loader.LoadParam("general_planner/over_wall_max_climb", over_wall_max_climb, 3.0);
            loader.LoadParam("general_planner/over_wall_height_step", over_wall_height_step, 0.3);
            loader.LoadParam("general_planner/over_wall_min_blocked_span", over_wall_min_blocked_span, 0.8);
            loader.LoadParam("general_planner/over_wall_min_progress_gain", over_wall_min_progress_gain, 0.5);
            loader.LoadParam("general_planner/over_wall_forward_ratio", over_wall_forward_ratio, 0.9);
            loader.LoadParam("general_planner/unknown_goal_reveal_en", unknown_goal_reveal_en, true);
            loader.LoadParam("general_planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("general_planner/ellipsoid_optimizer", ellipsoid_optimizer, std::string("classic"));
            loader.LoadParam("general_planner/ellipsoid_optimizer_fallback", ellipsoid_optimizer_fallback, false);
            loader.LoadParam("general_planner/yaw_mode", yaw_mode, 1);
            loader.LoadParam("general_planner/mpc_horizon", mpc_horizon, 1);
            loader.LoadParam("general_planner/yaw_dot_max", yaw_dot_max, 3.14);
            loader.LoadParam("general_planner/se3_aggressive/enable",
                             se3_aggressive_enable, true);
            loader.LoadParam("general_planner/se3_aggressive/piece_num",
                             se3_piece_num, 4);
            loader.LoadParam("general_planner/se3_aggressive/reference_speed",
                             se3_reference_speed, 3.0);
            loader.LoadParam("general_planner/se3_aggressive/min_duration",
                             se3_min_duration, 0.5);
            loader.LoadParam("general_planner/se3_aggressive/max_duration",
                             se3_max_duration, 8.0);
            loader.LoadParam("general_planner/se3_aggressive/horiz_half_len",
                             se3_horiz_half_len, 0.35);
            loader.LoadParam("general_planner/se3_aggressive/vert_half_len",
                             se3_vert_half_len, 0.12);
            loader.LoadParam("general_planner/se3_aggressive/safe_margin",
                             se3_safe_margin, 0.05);
            loader.LoadParam("general_planner/se3_aggressive/max_vel",
                             se3_max_vel, 8.0);
            loader.LoadParam("general_planner/se3_aggressive/thrust_acc_min",
                             se3_thrust_acc_min, 4.0);
            loader.LoadParam("general_planner/se3_aggressive/thrust_acc_max",
                             se3_thrust_acc_max, 20.0);
            loader.LoadParam("general_planner/se3_aggressive/body_rate_max",
                             se3_body_rate_max, 5.0);
            loader.LoadParam("general_planner/se3_aggressive/yaw_rate_max",
                             se3_yaw_rate_max, 3.0);
            loader.LoadParam("general_planner/se3_aggressive/weight_time",
                             se3_weight_time, 10.0);
            loader.LoadParam("general_planner/se3_aggressive/weight_corridor",
                             se3_weight_corridor, 1.0e4);
            loader.LoadParam("general_planner/se3_aggressive/weight_vel",
                             se3_weight_vel, 1.0e3);
            loader.LoadParam("general_planner/se3_aggressive/weight_thrust",
                             se3_weight_thrust, 1.0e3);
            loader.LoadParam("general_planner/se3_aggressive/weight_body_rate",
                             se3_weight_body_rate, 1.0e3);
            loader.LoadParam("general_planner/se3_aggressive/use_yaw",
                             se3_use_yaw, false);
            loader.LoadParam("general_planner/se3_aggressive/yaw_heading_to_velocity",
                             se3_yaw_heading_to_velocity, true);
            loader.LoadParam("general_planner/se3_aggressive/use_corridor",
                             se3_use_corridor, true);
            loader.LoadParam("general_planner/se3_aggressive/runtime_check_enable",
                             se3_runtime_check_enable, true);
            loader.LoadParam("general_planner/se3_aggressive/use_numeric_shape_gradient",
                             se3_use_numeric_shape_gradient, true);
            loader.LoadParam("general_planner/exploration_enable", exploration_enable, false);
            loader.LoadParam("general_planner/exploration/enable", exploration_enable, exploration_enable);
            loader.LoadParam("general_planner/exploration/frontend_type",
                             exploration_frontend_type, std::string("epic_original"));
            loader.LoadParam("general_planner/exploration/cloud_frame",
                             exploration_cloud_frame, std::string("WORLD"));
            loader.LoadParam("general_planner/exploration/use_epic_frontend",
                             exploration_use_epic_frontend, true);
            loader.LoadParam("general_planner/exploration/update_rog_map",
                             exploration_update_rog_map, true);
            loader.LoadParam("general_planner/exploration/backup_traj_enable",
                             exploration_backup_traj_enable,
                             backup_traj_en && !exploration_use_epic_frontend);
            loader.LoadParam("general_planner/exploration/backend_fallback_enable",
                             exploration_backend_fallback_enable,
                             backup_traj_en && !exploration_use_epic_frontend);
            loader.LoadParam("general_planner/exploration/print_log",
                             exploration_print_log, true);
            loader.LoadParam("general_planner/exploration/epic_lio/publish_map",
                             exploration_epic_lio_publish_map, true);
            loader.LoadParam("general_planner/exploration/epic_lio/publish_map_period",
                             exploration_epic_lio_publish_map_period, 0.5);
            loader.LoadParam("general_planner/exploration/epic_lio/self_filter_radius",
                             exploration_epic_lio_self_filter_radius, 0.8);
            loader.LoadParam("general_planner/exploration/lidar_perception/lidar_pitch",
                             exploration_lidar_pitch_deg, 40.0);
            loader.LoadParam("general_planner/exploration/lidar_perception/fov_up",
                             exploration_lidar_fov_up_deg, 52.0);
            loader.LoadParam("general_planner/exploration/lidar_perception/fov_down",
                             exploration_lidar_fov_down_deg, -7.0);
            loader.LoadParam("general_planner/exploration/lidar_perception/fov_viewpoint_up",
                             exploration_lidar_viewpoint_fov_up_deg, 48.0);
            loader.LoadParam("general_planner/exploration/lidar_perception/fov_viewpoint_down",
                             exploration_lidar_viewpoint_fov_down_deg, -5.0);
            loader.LoadParam("general_planner/exploration/lidar_perception/max_ray_length",
                             exploration_lidar_max_ray_length, 16.0);
            loader.LoadParam("general_planner/exploration/observation_map/resolution",
                             exploration_observation_resolution, 0.25);
            loader.LoadParam("general_planner/exploration/observation_map/min_distance",
                             exploration_observation_min_distance, 0.2);
            loader.LoadParam("general_planner/exploration/observation_map/well_observed_distance",
                             exploration_observation_well_distance, 4.0);
            loader.LoadParam("general_planner/exploration/observation_map/max_distance",
                             exploration_observation_max_distance, 12.0);
            loader.LoadParam("general_planner/exploration/observation_map/good_force_trust_length",
                             exploration_observation_good_force_trust_length, 1.5);
            loader.LoadParam("general_planner/exploration/observation_map/good_trust_length",
                             exploration_observation_good_trust_length, 4.0);
            loader.LoadParam("general_planner/exploration/observation_map/good_direction_score",
                             exploration_observation_good_direction_score, 0.5);
            loader.LoadParam("general_planner/exploration/observation_map/cloud_downsample_step",
                             exploration_observation_cloud_downsample_step, 1);
            loader.LoadParam("general_planner/exploration/observation_map/max_points_per_update",
                             exploration_observation_max_points_per_update, 12000);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_cluster_radius",
                             exploration_observation_frontier_cluster_radius, 0.65);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_normal_similarity",
                             exploration_observation_frontier_normal_similarity, 0.35);
            loader.LoadParam("general_planner/exploration/observation_map/min_frontier_cluster_size",
                             exploration_observation_min_frontier_cluster_size, 8);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_cluster_min_radius",
                             exploration_frontier_cluster_min_radius, 1.8);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_cluster_min_size",
                             exploration_frontier_cluster_min_size, 2.0);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_cluster_max_size",
                             exploration_frontier_cluster_max_size, 8.0);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_cluster_direction_radius",
                             exploration_frontier_cluster_direction_radius, 0.0);
            loader.LoadParam("general_planner/exploration/observation_map/frontier_cluster_minimum_point_num",
                             exploration_frontier_cluster_minimum_point_num, 10);
            loader.LoadParam("general_planner/exploration/observation_map/bbox_min",
                             exploration_observation_bbox_min, std::vector<double>{-50.0, -50.0, -2.0});
            loader.LoadParam("general_planner/exploration/observation_map/bbox_max",
                             exploration_observation_bbox_max, std::vector<double>{50.0, 50.0, 10.0});

            loader.LoadParam("general_planner/exploration/frontier_database/association_distance",
                             exploration_frontier_association_distance, 1.2);
            loader.LoadParam("general_planner/exploration/frontier_database/bbox_overlap_min_ratio",
                             exploration_frontier_bbox_overlap_min_ratio, 0.08);
            loader.LoadParam("general_planner/exploration/frontier_database/max_failed_count",
                             exploration_frontier_max_failed_count, 3);
            loader.LoadParam("general_planner/exploration/frontier_database/max_selected_count_without_gain",
                             exploration_frontier_max_selected_count_without_gain, 4);
            loader.LoadParam("general_planner/exploration/frontier_database/blacklist_time",
                             exploration_frontier_blacklist_time, 8.0);
            loader.LoadParam("general_planner/exploration/frontier_database/covered_gain_threshold",
                             exploration_frontier_covered_gain_threshold, 4.0);
            loader.LoadParam("general_planner/exploration/frontier_database/missing_frontier_timeout",
                             exploration_frontier_missing_timeout, 2.0);
            loader.LoadParam("general_planner/exploration/frontier_database/dormant_time",
                             exploration_frontier_dormant_time, 4.0);
            loader.LoadParam("general_planner/exploration/frontier_database/visited_viewpoint_radius",
                             exploration_frontier_visited_viewpoint_radius, 1.5);
            loader.LoadParam("general_planner/exploration/frontier_database/visited_viewpoint_penalty",
                             exploration_frontier_visited_viewpoint_penalty, 600.0);
            loader.LoadParam("general_planner/exploration/frontier_database/max_visited_viewpoints",
                             exploration_frontier_max_visited_viewpoints, 400);

            loader.LoadParam("general_planner/exploration/viewpoint/min_distance",
                             exploration_viewpoint_min_distance, 1.4);
            loader.LoadParam("general_planner/exploration/viewpoint/max_distance",
                             exploration_viewpoint_max_distance, 4.0);
            loader.LoadParam("general_planner/exploration/viewpoint/radius_samples",
                             exploration_viewpoint_radius_samples, 3);
            loader.LoadParam("general_planner/exploration/viewpoint/yaw_samples",
                             exploration_viewpoint_yaw_samples, 16);
            loader.LoadParam("general_planner/exploration/viewpoint/height_samples",
                             exploration_viewpoint_height_samples, 3);
            loader.LoadParam("general_planner/exploration/viewpoint/height_step",
                             exploration_viewpoint_height_step, 0.6);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_min_height",
                             exploration_viewpoint_sample_pillar_min_height, -2.0);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_max_height",
                             exploration_viewpoint_sample_pillar_max_height, 2.5);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_min_radius",
                             exploration_viewpoint_sample_pillar_min_radius, 1.0);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_max_radius",
                             exploration_viewpoint_sample_pillar_max_radius, 4.0);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_height_layer_num",
                             exploration_viewpoint_sample_pillar_height_layer_num, 5);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_radius_layer_num",
                             exploration_viewpoint_sample_pillar_radius_layer_num, 8);
            loader.LoadParam("general_planner/exploration/viewpoint/sample_pillar_circle_sample_num",
                             exploration_viewpoint_sample_pillar_circle_sample_num, 4);
            loader.LoadParam("general_planner/exploration/viewpoint/local_tsp_size",
                             exploration_viewpoint_local_tsp_size, 10);
            loader.LoadParam("general_planner/exploration/viewpoint/safe_distance",
                             exploration_viewpoint_safe_distance, 0.45);
            loader.LoadParam("general_planner/exploration/viewpoint/sensor_range",
                             exploration_viewpoint_sensor_range, 7.0);
            loader.LoadParam("general_planner/exploration/viewpoint/horizontal_fov_deg",
                             exploration_viewpoint_horizontal_fov_deg, 90.0);
            loader.LoadParam("general_planner/exploration/viewpoint/vertical_fov_deg",
                             exploration_viewpoint_vertical_fov_deg, 60.0);
            loader.LoadParam("general_planner/exploration/viewpoint/normal_dot_min",
                             exploration_viewpoint_normal_dot_min, 0.25);
            loader.LoadParam("general_planner/exploration/viewpoint/max_cells_per_gain_eval",
                             exploration_viewpoint_max_cells_per_gain_eval, 260);
            loader.LoadParam("general_planner/exploration/viewpoint/line_of_sight_step",
                             exploration_viewpoint_line_of_sight_step, 0.20);
            loader.LoadParam("general_planner/exploration/viewpoint/min_gain",
                             exploration_viewpoint_min_gain, 3.0);
            loader.LoadParam("general_planner/exploration/viewpoint/use_local_map_safety",
                             exploration_viewpoint_use_local_map_safety, false);
            loader.LoadParam("general_planner/exploration/viewpoint/cluster_by_visibility_sphere",
                             exploration_viewpoint_cluster_by_visibility_sphere, true);
            loader.LoadParam("general_planner/exploration/viewpoint/use_topo_reachability_filter",
                             exploration_viewpoint_use_topo_reachability_filter, true);
            loader.LoadParam("general_planner/exploration/viewpoint/max_clusters",
                             exploration_viewpoint_max_clusters, 8);
            loader.LoadParam("general_planner/exploration/viewpoint/cluster_connectivity_scale",
                             exploration_viewpoint_cluster_connectivity_scale, 1.0);
            loader.LoadParam("general_planner/exploration/viewpoint/topo_reachability_timeout",
                             exploration_viewpoint_topo_reachability_timeout, 0.03);
            loader.LoadParam("general_planner/exploration/viewpoint/epic_yaw_bins",
                             exploration_viewpoint_epic_yaw_bins, 8);

            loader.LoadParam("general_planner/exploration/topo_graph/history_node_min_distance",
                             exploration_topo_history_node_min_distance, 1.0);
            loader.LoadParam("general_planner/exploration/topo_graph/connect_radius",
                             exploration_topo_connect_radius, 8.0);
            loader.LoadParam("general_planner/exploration/topo_graph/local_edge_astar_timeout",
                             exploration_topo_local_edge_astar_timeout, 0.05);
            loader.LoadParam("general_planner/exploration/topo_graph/global_edge_max_length",
                             exploration_topo_global_edge_max_length, 14.0);
            loader.LoadParam("general_planner/exploration/topo_graph/max_history_nodes",
                             exploration_topo_max_history_nodes, 500);
            loader.LoadParam("general_planner/exploration/topo_graph/use_local_astar_for_edges",
                             exploration_topo_use_local_astar_for_edges, true);
            loader.LoadParam("general_planner/exploration/topo_graph/use_global_line_free_for_edges",
                             exploration_topo_use_global_line_free_for_edges, true);
            loader.LoadParam("general_planner/exploration/topo_graph/global_line_safe_distance",
                             exploration_topo_global_line_safe_distance, 0.45);
            loader.LoadParam("general_planner/exploration/topo_graph/global_line_step",
                             exploration_topo_global_line_step, 0.25);
            loader.LoadParam("general_planner/exploration/topo_graph/use_parallel_bubble_astar_for_edges",
                             exploration_topo_use_parallel_bubble_astar_for_edges, false);
            loader.LoadParam("general_planner/exploration/topo_graph/bubble_astar_resolution",
                             exploration_topo_bubble_astar_resolution, 0.5);
            loader.LoadParam("general_planner/exploration/topo_graph/bubble_astar_safe_distance",
                             exploration_topo_bubble_astar_safe_distance, 0.45);
            loader.LoadParam("general_planner/exploration/topo_graph/bubble_astar_max_nodes",
                             exploration_topo_bubble_astar_max_nodes, 8000);
            loader.LoadParam("general_planner/exploration/topo/use_epic_region_bubble_graph",
                             exploration_topo_use_epic_region_bubble_graph, true);
            loader.LoadParam("general_planner/exploration/topo_graph/use_epic_region_bubble_graph",
                             exploration_topo_use_epic_region_bubble_graph,
                             exploration_topo_use_epic_region_bubble_graph);
            loader.LoadParam("general_planner/exploration/topo/region_size_xy",
                             exploration_topo_region_size_xy, 4.0);
            loader.LoadParam("general_planner/exploration/topo/region_size_z",
                             exploration_topo_region_size_z, 2.0);
            loader.LoadParam("general_planner/exploration/topo/min_subregion_size_xy",
                             exploration_topo_min_subregion_size_xy, 0.5);
            loader.LoadParam("general_planner/exploration/topo/min_subregion_size_z",
                             exploration_topo_min_subregion_size_z, 0.5);
            loader.LoadParam("general_planner/exploration/topo/bubble_min_radius",
                             exploration_topo_bubble_min_radius, 0.5);
            loader.LoadParam("general_planner/exploration/topo/frontier_bubble_min_radius",
                             exploration_topo_frontier_bubble_min_radius, 0.5);
            loader.LoadParam("general_planner/exploration/topo/cube_discrete_size",
                             exploration_topo_cube_discrete_size, 0.3);
            loader.LoadParam("general_planner/exploration/topo/max_update_region_num",
                             exploration_topo_max_update_region_num, 20);
            loader.LoadParam("general_planner/exploration/topo/neighbor_mode",
                             exploration_topo_neighbor_mode, 26);
            loader.LoadParam("general_planner/exploration/topo/edge_search_padding_scale",
                             exploration_topo_edge_search_padding_scale, 1.0);
            loader.LoadParam("general_planner/exploration/topo/edge_safe_distance",
                             exploration_topo_edge_safe_distance, 0.45);
            loader.LoadParam("general_planner/exploration/topo_graph/max_edges_per_node",
                             exploration_topo_max_edges_per_node, 10);
            loader.LoadParam("general_planner/exploration/topo_graph/max_region_edges_per_node",
                             exploration_topo_max_region_edges_per_node, 8);
            loader.LoadParam("general_planner/exploration/topo_graph/max_frontier_edges_per_node",
                             exploration_topo_max_frontier_edges_per_node, 4);
            loader.LoadParam("general_planner/exploration/topo_graph/max_history_edges_per_node",
                             exploration_topo_max_history_edges_per_node, 3);
            loader.LoadParam("general_planner/exploration/topo_graph/max_candidate_neighbors",
                             exploration_topo_max_candidate_neighbors, 24);

            loader.LoadParam("general_planner/exploration/global_guidance/enable",
                             exploration_global_guidance_enable, true);
            loader.LoadParam("general_planner/exploration/global_guidance/max_frontiers_in_tour",
                             exploration_global_guidance_max_frontiers_in_tour, 16);
            loader.LoadParam("general_planner/exploration/global_guidance/weight_path_cost",
                             exploration_global_guidance_weight_path_cost, 1.0);
            loader.LoadParam("general_planner/exploration/global_guidance/weight_gain",
                             exploration_global_guidance_weight_gain, 1.0);
            loader.LoadParam("general_planner/exploration/global_guidance/weight_revisit",
                             exploration_global_guidance_weight_revisit, 0.5);
            loader.LoadParam("general_planner/exploration/global_guidance/use_two_opt",
                             exploration_global_guidance_use_two_opt, true);
            loader.LoadParam("general_planner/exploration/global_guidance/keep_current_target",
                             exploration_global_guidance_keep_current_target, true);
            loader.LoadParam("general_planner/exploration/global_guidance/use_lkh",
                             exploration_global_guidance_use_lkh, true);
            loader.LoadParam("general_planner/exploration/global_guidance/lkh_fallback_to_two_opt",
                             exploration_global_guidance_lkh_fallback_to_two_opt, true);
            loader.LoadParam("general_planner/exploration/global_guidance/tsp_dir",
                             exploration_global_guidance_tsp_dir, std::string("/tmp/general_planner_tsp"));
            loader.LoadParam("general_planner/exploration/tsp_dir",
                             exploration_global_guidance_tsp_dir, exploration_global_guidance_tsp_dir);
            loader.LoadParam("general_planner/exploration/global_guidance/tsp_problem_name",
                             exploration_global_guidance_tsp_problem_name, std::string("general_planner_global"));
            loader.LoadParam("general_planner/exploration/global_guidance/lkh_executable",
                             exploration_global_guidance_lkh_executable, std::string(""));
            loader.LoadParam("general_planner/exploration/global_guidance/lkh_cost_scale",
                             exploration_global_guidance_lkh_cost_scale, 100);

            loader.LoadParam("general_planner/exploration/global_route/route_waypoint_lookahead",
                             exploration_route_waypoint_lookahead, 3.0);
            loader.LoadParam("general_planner/exploration/global_route/route_waypoint_min_distance",
                             exploration_route_waypoint_min_distance, 1.0);
            loader.LoadParam("general_planner/exploration/global_route/route_waypoint_max_distance",
                             exploration_route_waypoint_max_distance, 5.0);
            loader.LoadParam("general_planner/exploration/global_route/local_goal_safe_distance",
                             exploration_local_goal_safe_distance, 0.35);
            loader.LoadParam("general_planner/exploration/global_route/route_replan_distance_threshold",
                             exploration_route_replan_distance_threshold, 1.0);
            loader.LoadParam("general_planner/exploration/global_route/use_local_map_safety",
                             exploration_global_route_use_local_map_safety, false);
            loader.LoadParam("general_planner/exploration/runtime/final_goal_radius",
                             exploration_runtime_final_goal_radius, 0.7);
            loader.LoadParam("general_planner/exploration/runtime/route_progress_min",
                             exploration_runtime_route_progress_min, 0.5);
            loader.LoadParam("general_planner/exploration/runtime/max_local_segment_fail_count",
                             exploration_runtime_max_local_segment_fail_count, 3);
            loader.LoadParam("general_planner/exploration/runtime/keep_active_target",
                             exploration_runtime_keep_active_target, true);
            loader.LoadParam("general_planner/exploration/runtime/switch_score_ratio",
                             exploration_runtime_switch_score_ratio, 1.25);
            loader.LoadParam("general_planner/exploration/runtime/global_update_dt",
                             exploration_runtime_global_update_dt, 0.2);
            loader.LoadParam("general_planner/exploration/runtime/replan_time_after_traj_start",
                             exploration_runtime_replan_time_after_traj_start, 0.5);
            loader.LoadParam("general_planner/exploration/runtime/replan_time_before_traj_end",
                             exploration_runtime_replan_time_before_traj_end, 0.5);
            loader.LoadParam("general_planner/exploration/runtime/safety_check_dt",
                             exploration_runtime_safety_check_dt, 0.15);
            loader.LoadParam("general_planner/exploration/runtime/safety_check_horizon",
                             exploration_runtime_safety_check_horizon, 2.0);
            loader.LoadParam("general_planner/exploration/runtime/stop_traj_time",
                             exploration_runtime_stop_traj_time, 0.2);
            loader.LoadParam("general_planner/exploration/runtime/collision_replan_time",
                             exploration_runtime_collision_replan_time, 0.5);
            loader.LoadParam("general_planner/exploration/local_guide/local_goal_lookahead",
                             exploration_local_guide_lookahead, 4.0);
            loader.LoadParam("general_planner/exploration/runtime/local_goal_lookahead",
                             exploration_local_guide_lookahead, exploration_local_guide_lookahead);
            loader.LoadParam("general_planner/exploration/local_guide/local_goal_min_distance",
                             exploration_local_guide_min_distance, 1.0);
            loader.LoadParam("general_planner/exploration/runtime/local_goal_min_distance",
                             exploration_local_guide_min_distance, exploration_local_guide_min_distance);
            loader.LoadParam("general_planner/exploration/local_guide/planning_horizon",
                             exploration_local_guide_planning_horizon, 8.0);
            loader.LoadParam("general_planner/exploration/local_guide/max_segment_length",
                             exploration_local_guide_max_segment_length, 1.0);
            loader.LoadParam("general_planner/exploration/local_guide/safe_distance",
                             exploration_local_guide_safe_distance, 0.45);
            loader.LoadParam("general_planner/exploration/local_guide/start_safe_distance",
                             exploration_local_guide_start_safe_distance, 0.20);
            loader.LoadParam("general_planner/exploration/local_guide/line_step",
                             exploration_local_guide_line_step, 0.20);
            loader.LoadParam("general_planner/exploration/local_guide/shortcut_enable",
                             exploration_local_guide_shortcut_enable, true);
            loader.LoadParam("general_planner/exploration/local_guide/astar_repair_enable",
                             exploration_local_guide_astar_repair_enable, true);
            loader.LoadParam("general_planner/exploration/local_guide/unknown_as_occupied",
                             exploration_local_guide_unknown_as_occupied, false);
            {
                std::string backend_name = mapBackendToString(exploration_local_guide_backend);
                loader.LoadParam("general_planner/exploration/local_guide/backend",
                                 backend_name, backend_name);
                exploration_local_guide_backend = mapBackendFromString(backend_name);
            }
            loader.LoadParam("general_planner/exploration/stuck/repeated_goal_threshold",
                             exploration_stuck_repeated_goal_threshold, 3);
            loader.LoadParam("general_planner/exploration/stuck/repeated_goal_distance",
                             exploration_stuck_repeated_goal_distance, 0.5);
            loader.LoadParam("general_planner/exploration/stuck/min_robot_displacement",
                             exploration_stuck_min_robot_displacement, 0.5);
            loader.LoadParam("general_planner/exploration/stuck/min_explored_volume_gain",
                             exploration_stuck_min_explored_volume_gain, 1.0);
            loader.LoadParam("general_planner/global_exploration_map/enable",
                             global_exploration_map_enable, false);
            loader.LoadParam("general_planner/global_exploration_map/resolution",
                             global_exploration_map_resolution, 0.20);
            loader.LoadParam("general_planner/global_exploration_map/raycast_step",
                             global_exploration_map_raycast_step, 0.10);
            loader.LoadParam("general_planner/global_exploration_map/max_range",
                             global_exploration_map_max_range, 8.0);
            loader.LoadParam("general_planner/global_exploration_map/min_range",
                             global_exploration_map_min_range, 0.20);
            loader.LoadParam("general_planner/global_exploration_map/bbox_min",
                             global_exploration_map_bbox_min, std::vector<double>{-50.0, -50.0, -2.0});
            loader.LoadParam("general_planner/global_exploration_map/bbox_max",
                             global_exploration_map_bbox_max, std::vector<double>{50.0, 50.0, 10.0});
            loader.LoadParam("general_planner/global_exploration_map/occupied_hit_threshold",
                             global_exploration_map_occupied_hit_threshold, 2);
            loader.LoadParam("general_planner/global_exploration_map/free_miss_threshold",
                             global_exploration_map_free_miss_threshold, 1);
            loader.LoadParam("general_planner/global_exploration_map/update_min_interval",
                             global_exploration_map_update_min_interval, 0.2);
            loader.LoadParam("general_planner/global_exploration_map/cloud_downsample_step",
                             global_exploration_map_cloud_downsample_step, 1);
            loader.LoadParam("general_planner/global_exploration_map/max_points_per_update",
                             global_exploration_map_max_points_per_update, 8000);

            loader.LoadParam("general_planner/global_pointcloud_map/enable",
                             global_pointcloud_map_enable, false);
            loader.LoadParam("general_planner/global_pointcloud_map/voxel_size",
                             global_pointcloud_map_voxel_size, 0.10);
            loader.LoadParam("general_planner/global_pointcloud_map/save_path",
                             global_pointcloud_map_save_path, std::string("/tmp/explored_global_map.pcd"));
            loader.LoadParam("general_planner/global_pointcloud_map/crop_enable",
                             global_pointcloud_map_crop_enable, false);
            loader.LoadParam("general_planner/global_pointcloud_map/crop_min",
                             global_pointcloud_map_crop_min, std::vector<double>{-50.0, -50.0, -2.0});
            loader.LoadParam("general_planner/global_pointcloud_map/crop_max",
                             global_pointcloud_map_crop_max, std::vector<double>{50.0, 50.0, 10.0});

            loader.LoadParam("general_planner/global_region_grid/enable",
                             global_region_grid_enable, false);
            loader.LoadParam("general_planner/global_region_grid/region_size_xy",
                             global_region_grid_region_size_xy, 4.0);
            loader.LoadParam("general_planner/global_region_grid/region_size_z",
                             global_region_grid_region_size_z, 2.0);
            loader.LoadParam("general_planner/global_region_grid/min_frontier_count",
                             global_region_grid_min_frontier_count, 10);
            loader.LoadParam("general_planner/global_region_grid/explored_ratio_threshold",
                             global_region_grid_explored_ratio_threshold, 0.85);

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
            loader.LoadParam("general_planner/tracking/keep_old_safety_dt", tracking_keep_old_safety_dt, 0.10);
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
