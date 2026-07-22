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

#pragma once

#include <string>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/yaml_loader.hpp>
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+(name)))

namespace traj_opt {
    using std::string;
    using std::vector;

    enum PosConstrainType {
        WAYPOINT = 1,
        CORRIDOR = 2,
    };

    class Config {
    public:
        bool uniform_time_en{false};

        flatness::FlatnessMap quadrotot_flatness;

        bool print_optimizer_log{false};

        /// Param for flatness
        double mass, dh, dv, grav, cp, v_eps;

        // if save the optimization problem to log
        bool save_log_en{false};

        int pos_constraint_type{CORRIDOR};
        // Set to true for only min time.
        bool block_energy_cost{false};
        // Limit conditions.
        double max_vel{0}, max_acc{0}, max_jerk{0}, max_omg{0}, max_acc_thr{0}, min_acc_thr{0};
        // Penalty cost.
        double penna_scale{-1}, penna_vel{0}, penna_acc{0}, penna_jerk{0}, penna_omg{0}, penna_theta{0}, penna_thr{0};
        // penna_t; penna_pos only for corridor based method.
        double penna_t{0}, penna_pos{0}, penna_attract{0}, penna_guide_path{0}, penna_guide_vel{0};
        double penna_guide_z_tube{0}, guide_z_tube_radius{0};
        double guide_path_tube_radius{0.25};
        double guide_path_z_tube_radius{0.10};
        double guide_path_huber_delta{0.25};
        bool guide_path_time_gradient_en{false};
        // penna_ts only for backupTraj;
        double penna_ts{0};
        // for backup traj piece num
        int piece_num{0};

        double penna_margin{0.05};

        double smooth_eps{0};
        int integral_reso{0};
        // Used only by the ordinary state2state ExpTrajOpt corridor backend.
        bool convex_hull_en{false};
        int convex_hull_basis{0};
        int convex_hull_subdivision_depth{0};
        int convex_hull_cost_version{1};
        bool convex_hull_alm_en{false};
        bool convex_hull_alm_warm_start_en{true};
        double convex_hull_alm_warm_start_accuracy{1.0e-3};
        bool convex_hull_alm_active_set_en{false};
        double convex_hull_alm_active_set_margin{0.05};
        bool convex_hull_adaptive_en{true};
        bool convex_hull_refine_derivative_constraints{false};
        int convex_hull_alm_max_outer_iterations{4};
        double convex_hull_refine_margin{0.05};
        double convex_hull_derivative_refine_margin{0.05};
        double convex_hull_alm_position_scale{0.25};
        double convex_hull_alm_initial_penalty{3.0e5};
        double convex_hull_alm_penalty_growth{5.0};
        double convex_hull_alm_progress_ratio{0.5};
        double convex_hull_alm_constraint_tolerance{1.0e-2};
        bool convex_hull_alm_require_certification{true};
        double opt_accuracy{0};
        double init_profile_vel_ratio{0.65};
        double init_duration_scale{1.25};
        double terminal_vel_ratio{0.0};

        // Zeroth-order state2state backend.  These parameters are intentionally
        // independent from gradient penalty weights: constraints define ranks,
        // while MINCO is used only to recover a polynomial trajectory.
        bool igo_enable{false};
        bool igo_fallback_to_legacy{true};
        bool igo_antithetic{true};
        bool igo_unknown_as_occupied{true};
        // State2state corridors are normally generated after accounting for
        // robot_r.  In that case raw occupancy avoids applying the robot
        // radius twice.  Enable this only for point-robot corridors.
        bool igo_use_inflated_map{false};
        int igo_population{24};
        int igo_generations{5};
        int igo_threads{4};
        int igo_seed{7};
        int igo_hull_subdivision_depth{1};
        int igo_no_feasible_expand_generations{3};
        int igo_snapshot_max_voxels{1000000};
        double igo_elite_ratio{0.25};
        double igo_mean_learning_rate{0.8};
        double igo_covariance_learning_rate{0.2};
        double igo_initial_sigma{0.65};
        double igo_min_eigenvalue{1.0e-6};
        double igo_max_condition_number{1.0e6};
        double igo_covariance_expand_factor{1.6};
        double igo_time_min_scale{0.45};
        double igo_time_max_scale{2.75};
        double igo_spatial_margin{0.02};
        double igo_sample_dt{0.03};
        double igo_sfc_tolerance{1.0e-6};
        double igo_dynamic_tolerance{0.02};
        double igo_energy_weight{0.01};
        double igo_guide_weight{0.05};
        double igo_unknown_weight{0.02};
        double igo_replan_budget_ratio{0.45};
        double igo_plan_from_rest_budget{0.15};

        Config() = default;

        Config(const std::string & cfg_path, string ns) {
            yaml_loader::YamlLoader loader(cfg_path);
            if (ns.empty()) {
                ns = "/";
            }
            else {
                ns = "/" + ns + "/";
            }

            loader.LoadParam("traj_opt/switch/print_optimizer_log", print_optimizer_log, false);
            /// Load Param for Flatness
            loader.LoadParam("traj_opt/flatness/mass", mass, 1.0);
            loader.LoadParam("traj_opt/flatness/dh", dh, 0.7);
            loader.LoadParam("traj_opt/flatness/dv", dv, 0.8);
            loader.LoadParam("traj_opt/flatness/grav", grav, 1.0);
            loader.LoadParam("traj_opt/flatness/cp", cp, 0.01);
            loader.LoadParam("traj_opt/flatness/v_eps", v_eps, 0.0001);

            loader.LoadParam("traj_opt/switch/save_log_en", save_log_en, false);
            loader.LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);
            loader.LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);
            loader.LoadParam("traj_opt" + ns + "uniform_time_en", uniform_time_en, false);
            loader.LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);
            loader.LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);
            loader.LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
            loader.LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
            loader.LoadParam("traj_opt" + ns + "convex_hull_en", convex_hull_en, false);
            loader.LoadParam("traj_opt" + ns + "convex_hull_basis", convex_hull_basis, 0);
            loader.LoadParam("traj_opt" + ns + "convex_hull_subdivision_depth",
                             convex_hull_subdivision_depth, 0);
            loader.LoadParam("traj_opt" + ns + "convex_hull_cost_version",
                             convex_hull_cost_version, 1);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_en",
                             convex_hull_alm_en, false);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_warm_start_en",
                             convex_hull_alm_warm_start_en, true);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_warm_start_accuracy",
                             convex_hull_alm_warm_start_accuracy, 1.0e-3);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_active_set_en",
                             convex_hull_alm_active_set_en, false);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_active_set_margin",
                             convex_hull_alm_active_set_margin, 0.05);
            loader.LoadParam("traj_opt" + ns + "convex_hull_adaptive_en",
                             convex_hull_adaptive_en, true);
            loader.LoadParam("traj_opt" + ns + "convex_hull_refine_derivative_constraints",
                             convex_hull_refine_derivative_constraints, false);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_max_outer_iterations",
                             convex_hull_alm_max_outer_iterations, 4);
            loader.LoadParam("traj_opt" + ns + "convex_hull_refine_margin",
                             convex_hull_refine_margin, 0.05);
            loader.LoadParam("traj_opt" + ns + "convex_hull_derivative_refine_margin",
                             convex_hull_derivative_refine_margin, 0.05);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_position_scale",
                             convex_hull_alm_position_scale, 0.25);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_initial_penalty",
                             convex_hull_alm_initial_penalty, 3.0e5);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_penalty_growth",
                             convex_hull_alm_penalty_growth, 5.0);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_progress_ratio",
                             convex_hull_alm_progress_ratio, 0.5);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_constraint_tolerance",
                             convex_hull_alm_constraint_tolerance, 1.0e-2);
            loader.LoadParam("traj_opt" + ns + "convex_hull_alm_require_certification",
                             convex_hull_alm_require_certification, true);
            loader.LoadParam("traj_opt" + ns + "init_profile_vel_ratio", init_profile_vel_ratio, 0.65);
            loader.LoadParam("traj_opt" + ns + "init_duration_scale", init_duration_scale, 1.25);
            loader.LoadParam("traj_opt" + ns + "terminal_vel_ratio", terminal_vel_ratio, 0.0);
            loader.LoadParam("traj_opt" + ns + "igo/enable", igo_enable, false);
            loader.LoadParam("traj_opt" + ns + "igo/fallback_to_legacy", igo_fallback_to_legacy, true);
            loader.LoadParam("traj_opt" + ns + "igo/antithetic", igo_antithetic, true);
            loader.LoadParam("traj_opt" + ns + "igo/unknown_as_occupied", igo_unknown_as_occupied, true);
            loader.LoadParam("traj_opt" + ns + "igo/use_inflated_map", igo_use_inflated_map, false);
            loader.LoadParam("traj_opt" + ns + "igo/population", igo_population, 24);
            loader.LoadParam("traj_opt" + ns + "igo/generations", igo_generations, 5);
            loader.LoadParam("traj_opt" + ns + "igo/threads", igo_threads, 4);
            loader.LoadParam("traj_opt" + ns + "igo/seed", igo_seed, 7);
            loader.LoadParam("traj_opt" + ns + "igo/hull_subdivision_depth", igo_hull_subdivision_depth, 1);
            loader.LoadParam("traj_opt" + ns + "igo/no_feasible_expand_generations",
                             igo_no_feasible_expand_generations, 3);
            loader.LoadParam("traj_opt" + ns + "igo/snapshot_max_voxels", igo_snapshot_max_voxels, 1000000);
            loader.LoadParam("traj_opt" + ns + "igo/elite_ratio", igo_elite_ratio, 0.25);
            loader.LoadParam("traj_opt" + ns + "igo/mean_learning_rate", igo_mean_learning_rate, 0.8);
            loader.LoadParam("traj_opt" + ns + "igo/covariance_learning_rate",
                             igo_covariance_learning_rate, 0.2);
            loader.LoadParam("traj_opt" + ns + "igo/initial_sigma", igo_initial_sigma, 0.65);
            loader.LoadParam("traj_opt" + ns + "igo/min_eigenvalue", igo_min_eigenvalue, 1.0e-6);
            loader.LoadParam("traj_opt" + ns + "igo/max_condition_number", igo_max_condition_number, 1.0e6);
            loader.LoadParam("traj_opt" + ns + "igo/covariance_expand_factor",
                             igo_covariance_expand_factor, 1.6);
            loader.LoadParam("traj_opt" + ns + "igo/time_min_scale", igo_time_min_scale, 0.45);
            loader.LoadParam("traj_opt" + ns + "igo/time_max_scale", igo_time_max_scale, 2.75);
            loader.LoadParam("traj_opt" + ns + "igo/spatial_margin", igo_spatial_margin, 0.02);
            loader.LoadParam("traj_opt" + ns + "igo/sample_dt", igo_sample_dt, 0.03);
            loader.LoadParam("traj_opt" + ns + "igo/sfc_tolerance", igo_sfc_tolerance, 1.0e-6);
            loader.LoadParam("traj_opt" + ns + "igo/dynamic_tolerance", igo_dynamic_tolerance, 0.02);
            loader.LoadParam("traj_opt" + ns + "igo/energy_weight", igo_energy_weight, 0.01);
            loader.LoadParam("traj_opt" + ns + "igo/guide_weight", igo_guide_weight, 0.05);
            loader.LoadParam("traj_opt" + ns + "igo/unknown_weight", igo_unknown_weight, 0.02);
            loader.LoadParam("traj_opt" + ns + "igo/replan_budget_ratio", igo_replan_budget_ratio, 0.45);
            loader.LoadParam("traj_opt" + ns + "igo/plan_from_rest_budget", igo_plan_from_rest_budget, 0.15);
            loader.LoadParam("traj_opt/boundary/max_vel", max_vel, -1.0);
            loader.LoadParam("traj_opt/boundary/max_acc", max_acc, -1.0);
            loader.LoadParam("traj_opt/boundary/max_jerk", max_jerk, -1.0);
            loader.LoadParam("traj_opt/boundary/max_omg", max_omg, -1.0);
            loader.LoadParam("traj_opt/boundary/max_acc_thr", max_acc_thr, -1.0);
            loader.LoadParam("traj_opt/boundary/min_acc_thr", min_acc_thr, -1.0);
            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);

            loader.LoadParam("traj_opt" + ns + "penna_scale", penna_scale, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_t", penna_t, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_ts", penna_ts, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_pos", penna_pos, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_vel", penna_vel, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_acc", penna_acc, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_jerk", penna_jerk, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_attract", penna_attract, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_guide_path", penna_guide_path, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_guide_vel", penna_guide_vel, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_guide_z_tube", penna_guide_z_tube, -1.0);
            loader.LoadParam("traj_opt" + ns + "guide_z_tube_radius", guide_z_tube_radius, -1.0);
            loader.LoadParam("traj_opt" + ns + "guide_path_tube_radius", guide_path_tube_radius, 0.25);
            loader.LoadParam("traj_opt" + ns + "guide_path_z_tube_radius", guide_path_z_tube_radius, 0.10);
            loader.LoadParam("traj_opt" + ns + "guide_path_huber_delta", guide_path_huber_delta, 0.25);
            loader.LoadParam("traj_opt" + ns + "guide_path_time_gradient_en", guide_path_time_gradient_en, false);
            loader.LoadParam("traj_opt" + ns + "penna_omg", penna_omg, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_thr", penna_thr, -1.0);

            if (penna_scale > 0) {
                penna_t = penna_t * penna_scale;
                penna_ts = penna_ts * penna_scale;
                penna_pos = penna_pos * penna_scale;
                penna_vel = penna_vel * penna_scale;
                penna_acc = penna_acc * penna_scale;
                penna_jerk = penna_jerk * penna_scale;
                penna_attract = penna_attract * penna_scale;
                penna_guide_path = penna_guide_path * penna_scale;
                penna_guide_vel = penna_guide_vel * penna_scale;
                penna_guide_z_tube = penna_guide_z_tube * penna_scale;
                penna_omg = penna_omg * penna_scale;
                penna_theta = penna_theta * penna_scale;
                penna_thr = penna_thr * penna_scale;
            }

            quadrotot_flatness.reset(mass, grav, dh, dv, cp, v_eps);
        }
    };
}
