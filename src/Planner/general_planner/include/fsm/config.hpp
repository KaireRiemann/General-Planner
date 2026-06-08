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


#ifndef GENERAL_FSM_CONFIG_HPP
#define GENERAL_FSM_CONFIG_HPP


#include <general_core/config.hpp>
#include <algorithm>
#include <cctype>
#include <vector>
#include <cstring>
#include <utils/header/yaml_loader.hpp>

namespace fsm {
    using namespace traj_opt;
    using namespace general_planner;
    static constexpr int MPC_PVAJ_MODE = 1;
    static constexpr int MPC_POLYTRAJ_MODE = 2;

    enum class TaskMode {
        STATE_TO_STATE = 0,
        TRACKING = 1,
        PERCHING = 2,
        EXPLORATION = 3,
        DYNAMIC_TAKEOFF = 4
    };

    inline std::string normalizeTaskMode(std::string mode) {
        std::transform(mode.begin(), mode.end(), mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (mode == "state_to_state" || mode == "state-2-state" || mode == "state2state" ||
            mode == "s2s" || mode == "corridor" || mode == "esdf" || mode == "plain") {
            return "state2state";
        }
        if (mode == "track" || mode == "tracking") {
            return "tracking";
        }
        if (mode == "perch" || mode == "perching") {
            return "perching";
        }
        if (mode == "takeoff" || mode == "dynamic_takeoff" ||
            mode == "dynamic-takeoff" || mode == "unperching") {
            return "dynamic_takeoff";
        }
        if (mode == "explore" || mode == "exploration") {
            return "exploration";
        }
        return "state2state";
    }

    inline TaskMode taskModeFromString(const std::string &mode) {
        const std::string normalized = normalizeTaskMode(mode);
        if (normalized == "tracking") {
            return TaskMode::TRACKING;
        }
        if (normalized == "perching") {
            return TaskMode::PERCHING;
        }
        if (normalized == "dynamic_takeoff") {
            return TaskMode::DYNAMIC_TAKEOFF;
        }
        if (normalized == "exploration") {
            return TaskMode::EXPLORATION;
        }
        return TaskMode::STATE_TO_STATE;
    }

    class Config {
    public:
        bool timer_en{true};

        // Fsm Params
        bool click_goal_en{},visualization_en{};
        bool auto_start{false};
        double replan_rate{}, resolution{};
        double click_height{};

        bool click_yaw_en{};
        string cmd_topic, mpc_cmd_topic, click_goal_topic;
        string task_mode_str{"state2state"};
        TaskMode task_mode{TaskMode::STATE_TO_STATE};
        bool task_planner_en{false};
        string task_mode_topic{"/planning/task_mode"};
        string tracking_target_odom_topic{"/tracking/target_odom"};
        string tracking_target_prediction_topic{"/tracking/target_prediction"};
        bool tracking_use_target_prediction_path{true};
        string perching_surface_odom_topic{"/perching/surface_odom"};
        double dynamic_takeoff_start_delay{0.0};
        bool tracking_perching_enable{false};
        double tracking_prediction_horizon{4.0};
        double tracking_prediction_dt{0.25};
        bool tracking_prediction_use_kinodynamic{true};
        double tracking_prediction_accel{3.0};
        double tracking_prediction_vmax{4.0};
        double tracking_prediction_rho_accel{1.0};
        double tracking_prediction_max_time{0.03};
        double tracking_static_position_epsilon{0.05};
        double tracking_static_velocity_epsilon{0.05};
        double tracking_static_yaw_epsilon{0.05};
        double tracking_static_replan_remaining_time{0.8};
        double tracking_static_task_position_epsilon{0.12};
        double tracking_static_task_velocity_epsilon{0.10};
        double tracking_static_prediction_filter_velocity_epsilon{0.08};
        double tracking_static_safety_check_horizon{1.5};
        double tracking_static_safety_check_dt{0.12};
        double tracking_static_replan_log_period{1.0};
        double task_timeout{0.6};
        int state2state_plan_from_rest_max_failures{0};
        bool state2state_clear_goal_on_plan_failure{false};
        double yaw_dot_max{};
        bool diagnostic_log_en{true};
        string diagnostic_event_topic{"/planning/diagnostics/events"};
        bool swarm_enable{false};
        int swarm_drone_id{-1};
        double swarm_des_clearance{0.75};
        bool swarm_broadcast_enable{true};
        string swarm_traj_broadcast_topic{"/swarm/trajectory"};
        string swarm_state_broadcast_topic{"/swarm/state"};
        vector<string> swarm_traj_topics;
        vector<int> swarm_traj_ids;

        Config() = default;

        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<double> tem_gain;
            loader.LoadParam("fsm/timer_en", timer_en, false);
            loader.LoadParam("fsm/auto_start", auto_start, false);
            loader.LoadParam("fsm/click_goal_en", click_goal_en, false);
            loader.LoadParam("fsm/click_yaw_en", click_yaw_en, false);
            loader.LoadParam("fsm/replan_rate", replan_rate, 10.0);
            loader.LoadParam("fsm/click_height", click_height, 1.5);
            loader.LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
            loader.LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));
            loader.LoadParam("fsm/task_mode", task_mode_str, string("state2state"));
            task_mode_str = normalizeTaskMode(task_mode_str);
            task_mode = taskModeFromString(task_mode_str);
            loader.LoadParam("fsm/task_planner_en", task_planner_en, false);
            loader.LoadParam("fsm/task_mode_topic", task_mode_topic, string("/planning/task_mode"));
            loader.LoadParam("fsm/tracking_target_odom_topic", tracking_target_odom_topic,
                             string("/tracking/target_odom"));
            loader.LoadParam("fsm/tracking_target_prediction_topic", tracking_target_prediction_topic,
                             string("/tracking/target_prediction"));
            loader.LoadParam("fsm/tracking_use_target_prediction_path", tracking_use_target_prediction_path, true);
            loader.LoadParam("fsm/perching_surface_odom_topic", perching_surface_odom_topic,
                             string("/perching/surface_odom"));
            loader.LoadParam("fsm/dynamic_takeoff_start_delay", dynamic_takeoff_start_delay, 0.0);
            loader.LoadParam("general_planner/tracking_perching/enable", tracking_perching_enable, false);
            loader.LoadParam("fsm/tracking_prediction_horizon", tracking_prediction_horizon, 4.0);
            loader.LoadParam("fsm/tracking_prediction_dt", tracking_prediction_dt, 0.25);
            loader.LoadParam("fsm/tracking_prediction_use_kinodynamic", tracking_prediction_use_kinodynamic, true);
            loader.LoadParam("fsm/tracking_prediction_accel", tracking_prediction_accel, 3.0);
            loader.LoadParam("fsm/tracking_prediction_vmax", tracking_prediction_vmax, 4.0);
            loader.LoadParam("fsm/tracking_prediction_rho_accel", tracking_prediction_rho_accel, 1.0);
            loader.LoadParam("fsm/tracking_prediction_max_time", tracking_prediction_max_time, 0.03);
            loader.LoadParam("fsm/tracking_static_position_epsilon", tracking_static_position_epsilon, 0.05);
            loader.LoadParam("fsm/tracking_static_velocity_epsilon", tracking_static_velocity_epsilon, 0.05);
            loader.LoadParam("fsm/tracking_static_yaw_epsilon", tracking_static_yaw_epsilon, 0.05);
            loader.LoadParam("fsm/tracking_static_replan_remaining_time", tracking_static_replan_remaining_time, 0.8);
            loader.LoadParam("fsm/tracking_static_task_position_epsilon", tracking_static_task_position_epsilon, 0.12);
            loader.LoadParam("fsm/tracking_static_task_velocity_epsilon", tracking_static_task_velocity_epsilon, 0.10);
            loader.LoadParam("fsm/tracking_static_prediction_filter_velocity_epsilon",
                             tracking_static_prediction_filter_velocity_epsilon, 0.08);
            loader.LoadParam("fsm/tracking_static_safety_check_horizon", tracking_static_safety_check_horizon, 1.5);
            loader.LoadParam("fsm/tracking_static_safety_check_dt", tracking_static_safety_check_dt, 0.12);
            loader.LoadParam("fsm/tracking_static_replan_log_period", tracking_static_replan_log_period, 1.0);
            loader.LoadParam("fsm/task_timeout", task_timeout, 0.6);
            loader.LoadParam("fsm/state2state_plan_from_rest_max_failures",
                             state2state_plan_from_rest_max_failures,
                             0);
            loader.LoadParam("fsm/state2state_clear_goal_on_plan_failure",
                             state2state_clear_goal_on_plan_failure,
                             false);
            loader.LoadParam("fsm/diagnostic_log_en", diagnostic_log_en, true);
            loader.LoadParam("fsm/diagnostic_event_topic", diagnostic_event_topic,
                             string("/planning/diagnostics/events"));
            loader.LoadParam("general_planner/swarm/enable", swarm_enable, false);
            loader.LoadParam("general_planner/swarm/drone_id", swarm_drone_id, -1);
            loader.LoadParam("general_planner/swarm/des_clearance", swarm_des_clearance, 0.75);
            loader.LoadParam("general_planner/swarm/broadcast_enable", swarm_broadcast_enable, true);
            loader.LoadParam("general_planner/swarm/traj_broadcast_topic", swarm_traj_broadcast_topic,
                             string("/swarm/trajectory"));
            loader.LoadParam("general_planner/swarm/state_broadcast_topic", swarm_state_broadcast_topic,
                             string("/swarm/state"));
            loader.LoadParam("general_planner/swarm/traj_topics", swarm_traj_topics, vector<string>{});
            loader.LoadParam("general_planner/swarm/traj_ids", swarm_traj_ids, vector<int>{});


            loader.LoadParam("general_planner/yaw_dot_max", yaw_dot_max, 1.0, true);
            loader.LoadParam("general_planner/visualization_en", visualization_en, false, true);
            loader.LoadParam("rog_map/resolution", resolution, 0.1, true);

        }
    };
}

#endif // GENERAL_FSM_CONFIG_H
