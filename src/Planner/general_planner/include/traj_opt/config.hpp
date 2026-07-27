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
        // 0: Bezier, 1: MINVO. The optimizer/certificate state machine is
        // shared; only the fixed linear control-basis operator changes.
        int convex_hull_basis{0};
        // The stable production route is deliberately unique: depth-2 V2
        // with a read-only post-solve P/V/A certificate. Hard rejection and
        // failure-triggered polish are an experimental opt-in.
        static constexpr int convex_hull_subdivision_depth{2};
        double convex_hull_position_scale{0.25};
        double convex_hull_robust_certificate_margin{0.05};
        double convex_hull_polish_initial_penalty{3.0e5};
        double convex_hull_polish_penalty_growth{5.0};
        double convex_hull_polish_progress_ratio{0.5};
        bool convex_hull_require_certification{false};
        int convex_hull_polish_top_k{32};
        int convex_hull_polish_max_constraints{64};
        int convex_hull_polish_max_outer{4};
        double convex_hull_polish_inner_tol_init{1.0e-2};
        double convex_hull_polish_inner_tol_final{1.0e-4};
        bool convex_hull_polish_append_en{true};
        bool convex_hull_polish_multiplier_reuse_en{true};
        // Keep exact FlatnessMap penalties in the real objective. In the
        // stable route this flag enables shadow flatness diagnostics only.
        bool convex_hull_flatness_en{false};
        double max_tilt{1.05};
        double opt_accuracy{0};
        // Optional fast LBFGS path used only by the ordinary ExpTrajOpt
        // state2state corridor backend.
        bool lbfgs_fast_en{false};
        int lbfgs_mem_size{32};
        bool lbfgs_step_bound_en{true};
        double lbfgs_time_ratio_min{0.5};
        double lbfgs_time_ratio_max{2.0};
        int lbfgs_fast_window{5};
        int lbfgs_fast_min_iterations{20};
        int lbfgs_fast_consecutive{2};
        double lbfgs_fast_rel_cost{1.0e-4};
        double lbfgs_fast_rel_step{2.0e-3};
        double lbfgs_fast_rel_penalty{5.0e-3};
        // Optional Phase-0 guards. When false, fast-stop uses only the
        // original cost / decision-step / penalty-log stability test.
        bool lbfgs_fast_phase0_guards_en{false};
        double lbfgs_fast_rel_time{2.0e-2};
        double lbfgs_fast_rel_waypoint{2.0e-2};
        double lbfgs_fast_scaled_grad{5.0e-2};
        double lbfgs_fast_min_step{1.0e-8};
        double lbfgs_fast_penalty_tol{1.0e-2};
        int lbfgs_fast_small_step_limit{3};
        // Cross-replan warm start for the ordinary State2State ExpTrajOpt.
        // The cached physical solution is used only when it remains compatible
        // with the new corridor, and only if its real objective beats the
        // freshly generated guide seed.
        bool lbfgs_warm_start_en{false};
        double lbfgs_warm_start_max_endpoint_shift{3.0};
        double lbfgs_warm_start_max_waypoint_shift{2.0};
        double lbfgs_warm_start_duration_blend{0.75};
        double lbfgs_warm_start_cost_ratio{0.75};
        double lbfgs_warm_start_gradient_ratio{1.0};
        double lbfgs_warm_start_penalty_ratio{1.0};
        double guide_initial_time_scale{1.0};
        double init_profile_vel_ratio{0.65};
        double init_duration_scale{1.25};
        double terminal_vel_ratio{0.0};

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
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_en",
                             lbfgs_fast_en, false);
            loader.LoadParam("traj_opt" + ns + "lbfgs_mem_size",
                             lbfgs_mem_size, 32);
            loader.LoadParam("traj_opt" + ns + "lbfgs_step_bound_en",
                             lbfgs_step_bound_en, true);
            loader.LoadParam("traj_opt" + ns + "lbfgs_time_ratio_min",
                             lbfgs_time_ratio_min, 0.5);
            loader.LoadParam("traj_opt" + ns + "lbfgs_time_ratio_max",
                             lbfgs_time_ratio_max, 2.0);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_window",
                             lbfgs_fast_window, 5);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_min_iterations",
                             lbfgs_fast_min_iterations, 20);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_consecutive",
                             lbfgs_fast_consecutive, 2);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_rel_cost",
                             lbfgs_fast_rel_cost, 1.0e-4);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_rel_step",
                             lbfgs_fast_rel_step, 2.0e-3);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_rel_penalty",
                             lbfgs_fast_rel_penalty, 5.0e-3);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_phase0_guards_en",
                             lbfgs_fast_phase0_guards_en, false);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_rel_time",
                             lbfgs_fast_rel_time, 2.0e-2);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_rel_waypoint",
                             lbfgs_fast_rel_waypoint, 2.0e-2);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_scaled_grad",
                             lbfgs_fast_scaled_grad, 5.0e-2);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_min_step",
                             lbfgs_fast_min_step, 1.0e-8);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_penalty_tol",
                             lbfgs_fast_penalty_tol, 1.0e-2);
            loader.LoadParam("traj_opt" + ns + "lbfgs_fast_small_step_limit",
                             lbfgs_fast_small_step_limit, 3);
            loader.LoadParam("traj_opt" + ns + "lbfgs_warm_start_en",
                             lbfgs_warm_start_en, false);
            loader.LoadParam("traj_opt" + ns +
                                 "lbfgs_warm_start_max_endpoint_shift",
                             lbfgs_warm_start_max_endpoint_shift, 3.0);
            loader.LoadParam("traj_opt" + ns +
                                 "lbfgs_warm_start_max_waypoint_shift",
                             lbfgs_warm_start_max_waypoint_shift, 2.0);
            loader.LoadParam("traj_opt" + ns +
                                 "lbfgs_warm_start_duration_blend",
                             lbfgs_warm_start_duration_blend, 0.75);
            loader.LoadParam("traj_opt" + ns +
                                 "lbfgs_warm_start_cost_ratio",
                             lbfgs_warm_start_cost_ratio, 0.75);
            loader.LoadParam("traj_opt" + ns +
                                 "lbfgs_warm_start_gradient_ratio",
                             lbfgs_warm_start_gradient_ratio, 1.0);
            loader.LoadParam("traj_opt" + ns +
                                 "lbfgs_warm_start_penalty_ratio",
                             lbfgs_warm_start_penalty_ratio, 1.0);
            loader.LoadParam("traj_opt" + ns + "guide_initial_time_scale",
                             guide_initial_time_scale, 1.0);
            loader.LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
            loader.LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
            loader.LoadParam("traj_opt" + ns + "convex_hull_en", convex_hull_en, false);
            loader.LoadParam("traj_opt" + ns + "convex_hull_basis",
                             convex_hull_basis, 0);
            loader.LoadParam("traj_opt" + ns + "convex_hull_position_scale",
                             convex_hull_position_scale, 0.25);
            loader.LoadParam("traj_opt" + ns + "convex_hull_robust_certificate_margin",
                             convex_hull_robust_certificate_margin, 0.05);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_initial_penalty",
                             convex_hull_polish_initial_penalty, 3.0e5);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_penalty_growth",
                             convex_hull_polish_penalty_growth, 5.0);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_progress_ratio",
                             convex_hull_polish_progress_ratio, 0.5);
            loader.LoadParam("traj_opt" + ns + "convex_hull_require_certification",
                             convex_hull_require_certification, false);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_top_k",
                             convex_hull_polish_top_k, 32);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_max_constraints",
                             convex_hull_polish_max_constraints, 64);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_max_outer",
                             convex_hull_polish_max_outer, 4);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_inner_tol_init",
                             convex_hull_polish_inner_tol_init, 1.0e-2);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_inner_tol_final",
                             convex_hull_polish_inner_tol_final, 1.0e-4);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_append_en",
                             convex_hull_polish_append_en, true);
            loader.LoadParam("traj_opt" + ns + "convex_hull_polish_multiplier_reuse_en",
                             convex_hull_polish_multiplier_reuse_en, true);
            loader.LoadParam("traj_opt" + ns + "convex_hull_flatness_en",
                             convex_hull_flatness_en, false);
            loader.LoadParam("traj_opt" + ns + "init_profile_vel_ratio", init_profile_vel_ratio, 0.65);
            loader.LoadParam("traj_opt" + ns + "init_duration_scale", init_duration_scale, 1.25);
            loader.LoadParam("traj_opt" + ns + "terminal_vel_ratio", terminal_vel_ratio, 0.0);
            loader.LoadParam("traj_opt/boundary/max_vel", max_vel, -1.0);
            loader.LoadParam("traj_opt/boundary/max_acc", max_acc, -1.0);
            loader.LoadParam("traj_opt/boundary/max_jerk", max_jerk, -1.0);
            loader.LoadParam("traj_opt/boundary/max_omg", max_omg, -1.0);
            loader.LoadParam("traj_opt/boundary/max_tilt", max_tilt, 1.05);
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
