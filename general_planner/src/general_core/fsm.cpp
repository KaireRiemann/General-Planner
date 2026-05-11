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

#include <fsm/fsm.h>
#include <algorithm>
#include <cmath>
#include <memory>

using namespace super_utils;

namespace fsm {
    namespace {
        double yawDiff(const double lhs, const double rhs) {
            return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
        }
    }

    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) {
            return;
        }

        if (trackingMode()) {
            if (!trackingExecutionState()) {
                return;
            }
        } else if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        if (finish_plan) {
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = FAILED;
        bool replan_tracking_static = false;
        if (state2stateMode()) {
            planner_ptr_->getMapManager()->getNearestInfCellNot(GridType::OCCUPIED, gi_.goal_p, gi_.goal_p, 3.0);
            ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        } else if (trackingMode()) {
            traj_opt::DynamicTargetStates prediction;
            if (!getTrackingTargetPrediction(prediction)) {
                return;
            }
            replan_tracking_static = trackingPredictionStatic(prediction);
            if (shouldSkipStaticTrackingReplan(prediction)) {
                if (machine_state_ != HOLD_TRACKING) {
                    ChangeState("StaticTrackingHold", HOLD_TRACKING);
                }
                return;
            }
            ret_code = planner_ptr_->ReplanTrackingOnce(prediction, task_new_);
        } else if (perchingMode()) {
            traj_opt::PerchingSurfaceState surface;
            if (!getPerchingSurface(surface)) {
                return;
            }
            ret_code = planner_ptr_->ReplanPerchingOnce(surface, task_new_);
        }
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == NO_NEED && trackingMode()) {
            publishPolyTraj();
            if (replan_tracking_static) {
                ChangeState("ReplanTimerCallback", HOLD_TRACKING);
            } else if (machine_state_ != FOLLOW_TRAJ) {
                ChangeState("ReplanTimerCallback", FOLLOW_TRAJ);
            }
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            task_new_ = false;
            publishPolyTraj();
            if (trackingMode()) {
                if (replan_tracking_static) {
                    ChangeState("ReplanTimerCallback", STATIC_TRACKING);
                } else if (machine_state_ != FOLLOW_TRAJ) {
                    ChangeState("ReplanTimerCallback", FOLLOW_TRAJ);
                }
            }
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!activeTaskReady()) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                if (state2stateMode() && closeToGoal(0.1)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                int retcode = FAILED;
                bool planned_tracking_static = false;
                if (state2stateMode()) {
                    retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                } else if (trackingMode()) {
                    traj_opt::DynamicTargetStates prediction;
                    if (!getTrackingTargetPrediction(prediction)) {
                        return;
                    }
                    planned_tracking_static = trackingPredictionStatic(prediction);
                    retcode = planner_ptr_->PlanTrackingFromRest(prediction, task_new_);
                } else if (perchingMode()) {
                    traj_opt::PerchingSurfaceState surface;
                    if (!getPerchingSurface(surface)) {
                        return;
                    }
                    retcode = planner_ptr_->PlanPerchingFromRest(surface, task_new_);
                }
                if (state2stateMode() && !planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == NO_NEED && trackingMode()) {
                    plan_from_rest_ = true;
                    finish_plan = false;
                    publishPolyTraj();
                    ChangeState("MainFsmCallback",
                                planned_tracking_static ? HOLD_TRACKING : FOLLOW_TRAJ);
                } else if (retcode == SUCCESS || retcode == FINISH) {
                    gi_.new_goal = false;
                    task_new_ = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback",
                                trackingMode() && planned_tracking_static ? STATIC_TRACKING : FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                break;
            }
            case STATIC_TRACKING:
            case HOLD_TRACKING: {
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    bool Fsm::state2stateMode() const {
        return cfg_.task_mode == TaskMode::STATE_TO_STATE;
    }

    bool Fsm::trackingMode() const {
        return cfg_.task_mode == TaskMode::TRACKING;
    }

    bool Fsm::perchingMode() const {
        return cfg_.task_mode == TaskMode::PERCHING;
    }

    bool Fsm::trackingExecutionState() const {
        return machine_state_ == FOLLOW_TRAJ ||
               machine_state_ == STATIC_TRACKING ||
               machine_state_ == HOLD_TRACKING;
    }

    void Fsm::setTaskModeFromString(const std::string &mode) {
        const std::string normalized = normalizeTaskMode(mode);
        const TaskMode new_mode = taskModeFromString(normalized);
        if (new_mode == cfg_.task_mode) {
            return;
        }
        cout << YELLOW << " -- [Fsm] Task mode switch: " << cfg_.task_mode_str
             << " -> " << normalized << RESET << endl;
        cfg_.task_mode_str = normalized;
        cfg_.task_mode = new_mode;
        finish_plan = false;
        task_new_ = true;
    }

    bool Fsm::trackingTaskReady() {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (tracking_target_prediction_.empty() || tracking_target_rcv_time_ < 0.0) {
            return false;
        }
        return (ros_ptr_->getSimTime() - tracking_target_rcv_time_) <= cfg_.task_timeout;
    }

    bool Fsm::perchingTaskReady() {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (perching_surface_rcv_time_ < 0.0) {
            return false;
        }
        return (ros_ptr_->getSimTime() - perching_surface_rcv_time_) <= cfg_.task_timeout;
    }

    bool Fsm::activeTaskReady() {
        if (state2stateMode()) {
            return gi_.new_goal;
        }
        if (trackingMode()) {
            return trackingTaskReady();
        }
        if (perchingMode()) {
            return perchingTaskReady();
        }
        return false;
    }

    bool Fsm::shouldGenerateAfterTrajFinish() {
        if (state2stateMode()) {
            return !closeToGoal(0.1);
        }
        return activeTaskReady();
    }

    void Fsm::logStaticTrackingReplanDecision(const std::string &reason) {
        const double now = ros_ptr_->getSimTime();
        const double log_period = std::max(0.0, cfg_.tracking_static_replan_log_period);
        if (last_static_tracking_replan_log_time_ >= 0.0 &&
            now - last_static_tracking_replan_log_time_ < log_period) {
            return;
        }
        last_static_tracking_replan_log_time_ = now;
        ros_ptr_->info(" -- [Fsm] Static tracking replan decision: {}", reason);
    }

    bool Fsm::trackingCommittedTrajectoryUnsafe() const {
        const auto map_manager = planner_ptr_->getMapManager();
        if (map_manager == nullptr || !map_manager->ready()) {
            return false;
        }

        const Trajectory traj = planner_ptr_->getCommittedPositionTrajectory();
        if (traj.empty()) {
            return true;
        }

        const double remaining = planner_ptr_->getCommittedTrajectoryRemainingDuration();
        if (remaining <= 1.0e-3) {
            return false;
        }

        const double current_t = std::clamp(ros_ptr_->getSimTime() - traj.start_WT,
                                            0.0,
                                            traj.getTotalDuration());
        const double horizon = std::min(remaining, std::max(0.0, cfg_.tracking_static_safety_check_horizon));
        const double dt = std::max(0.05, cfg_.tracking_static_safety_check_dt);

        Vec3f last = traj.getPos(current_t);
        if (!last.allFinite()) {
            return true;
        }

        for (double offset = 0.0; offset <= horizon + 1.0e-6; offset += dt) {
            const double t = std::min(traj.getTotalDuration(), current_t + offset);
            const Vec3f pos = traj.getPos(t);
            if (!pos.allFinite() || !map_manager->insideLocalMap(pos)) {
                return true;
            }
            const GridType grid_type = map_manager->getInfGridType(pos);
            if (grid_type == GridType::OCCUPIED || grid_type == GridType::OUT_OF_MAP) {
                return true;
            }
            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager->isLineFree(last, pos, true, false)) {
                return true;
            }
            last = pos;
        }
        return false;
    }

    traj_opt::DynamicTargetStates Fsm::filterStaticTrackingPrediction(
            const traj_opt::DynamicTargetStates &prediction) const {
        if (prediction.empty()) {
            return prediction;
        }

        traj_opt::DynamicTargetStates filtered = prediction;
        const Vec3f ref = filtered.front().position;
        double max_span = 0.0;
        double max_vel = 0.0;
        for (const auto &state : filtered) {
            max_span = std::max(max_span, (state.position - ref).norm());
            max_vel = std::max(max_vel, state.velocity.norm());
        }

        const double pos_eps = std::max(0.0, cfg_.tracking_static_position_epsilon);
        const double vel_eps = std::max(0.0, cfg_.tracking_static_prediction_filter_velocity_epsilon);
        if (max_span > pos_eps && max_vel > vel_eps) {
            return filtered;
        }

        const double static_yaw = filtered.front().yaw;
        for (auto &state : filtered) {
            state.position = ref;
            state.velocity.setZero();
            state.acceleration.setZero();
            state.yaw = static_yaw;
            state.yaw_rate = 0.0;
        }
        return filtered;
    }

    bool Fsm::trackingPredictionChanged(const traj_opt::DynamicTargetStates &a,
                                        const traj_opt::DynamicTargetStates &b) const {
        if (a.empty() || b.empty()) {
            return true;
        }

        const auto &a_front = a.front();
        const auto &b_front = b.front();
        const auto &a_back = a.back();
        const auto &b_back = b.back();
        const double pos_eps = std::max(0.0, cfg_.tracking_static_position_epsilon);
        const double vel_eps = std::max(0.0, cfg_.tracking_static_velocity_epsilon);
        const double yaw_eps = std::max(0.0, cfg_.tracking_static_yaw_epsilon);

        const bool both_static = trackingPredictionStatic(a) && trackingPredictionStatic(b);
        const double task_pos_eps = both_static
                                        ? std::max(pos_eps, cfg_.tracking_static_task_position_epsilon)
                                        : pos_eps;
        const double task_vel_eps = both_static
                                        ? std::max(vel_eps, cfg_.tracking_static_task_velocity_epsilon)
                                        : vel_eps;
        const auto yawChanged = [yaw_eps](double lhs, double rhs) {
            return std::abs(yawDiff(lhs, rhs)) > yaw_eps;
        };

        return (a_front.position - b_front.position).norm() > task_pos_eps ||
               (a_back.position - b_back.position).norm() > task_pos_eps ||
               (a_front.velocity - b_front.velocity).norm() > task_vel_eps ||
               (a_back.velocity - b_back.velocity).norm() > task_vel_eps ||
               yawChanged(a_front.yaw, b_front.yaw) ||
               yawChanged(a_back.yaw, b_back.yaw);
    }

    bool Fsm::trackingPredictionStatic(const traj_opt::DynamicTargetStates &prediction) const {
        if (prediction.empty()) {
            return false;
        }

        const double pos_eps = std::max(0.0, cfg_.tracking_static_position_epsilon);
        const double vel_eps = std::max(0.0, cfg_.tracking_static_velocity_epsilon);
        const Vec3f ref = prediction.front().position;
        for (const auto &state : prediction) {
            if ((state.position - ref).norm() > pos_eps ||
                state.velocity.norm() > vel_eps) {
                return false;
            }
        }
        return true;
    }

    bool Fsm::shouldSkipStaticTrackingReplan(const traj_opt::DynamicTargetStates &prediction) {
        if (!trackingPredictionStatic(prediction)) {
            logStaticTrackingReplanDecision("target prediction is moving");
            return false;
        }
        if (task_new_) {
            logStaticTrackingReplanDecision("task_new is true; target moved outside static hold noise band");
            return false;
        }
        const double remaining = planner_ptr_->getCommittedTrajectoryRemainingDuration();
        const double replan_time = std::max(0.0, cfg_.tracking_static_replan_remaining_time);
        if (remaining <= replan_time) {
            logStaticTrackingReplanDecision(
                    fmt::format("trajectory ending soon: remaining={:.3f}s <= {:.3f}s", remaining, replan_time));
            return false;
        }
        if (trackingCommittedTrajectoryUnsafe()) {
            logStaticTrackingReplanDecision("committed trajectory has safety risk");
            return false;
        }
        logStaticTrackingReplanDecision(
                fmt::format("skip static hold replan: remaining={:.3f}s, target static, trajectory safe", remaining));
        return true;
    }

    void Fsm::setTrackingTargetPrediction(const traj_opt::DynamicTargetStates &prediction) {
        if (prediction.empty()) {
            return;
        }
        const traj_opt::DynamicTargetStates filtered_prediction = filterStaticTrackingPrediction(prediction);
        bool changed = true;
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            changed = trackingPredictionChanged(tracking_target_prediction_, filtered_prediction);
            tracking_target_prediction_ = filtered_prediction;
            tracking_target_rcv_time_ = ros_ptr_->getSimTime();
            task_new_ = task_new_ || changed;
        }
        gi_.goal_p = filtered_prediction.back().position;
        gi_.goal_yaw = filtered_prediction.back().yaw;
        gi_.new_goal = gi_.new_goal || changed;
        started_ = true;
    }

    void Fsm::setPerchingSurface(const traj_opt::PerchingSurfaceState &surface) {
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            perching_surface_ = surface;
            perching_surface_rcv_time_ = ros_ptr_->getSimTime();
            task_new_ = true;
        }
        gi_.goal_p = surface.position;
        gi_.goal_yaw = surface.yaw;
        gi_.new_goal = true;
        started_ = true;
    }

    bool Fsm::getTrackingTargetPrediction(traj_opt::DynamicTargetStates &prediction) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (tracking_target_prediction_.empty() || tracking_target_rcv_time_ < 0.0 ||
            (ros_ptr_->getSimTime() - tracking_target_rcv_time_) > cfg_.task_timeout) {
            return false;
        }
        prediction = tracking_target_prediction_;
        return true;
    }

    bool Fsm::getPerchingSurface(traj_opt::PerchingSurfaceState &surface) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (perching_surface_rcv_time_ < 0.0 ||
            (ros_ptr_->getSimTime() - perching_surface_rcv_time_) > cfg_.task_timeout) {
            return false;
        }
        surface = perching_surface_;
        return true;
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {

        auto click_point = p;
        if (cfg_.click_height > -5) {
            click_point.z() = cfg_.click_height;
        }
        const bool had_goal = started_;
        const Vec3f last_goal_p = gi_.goal_p;
        const double last_goal_yaw = gi_.goal_yaw;

        if (planner_ptr_->getMapManager()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)) {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return;
        }
        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }

        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        const bool same_yaw = (std::isnan(last_goal_yaw) && std::isnan(gi_.goal_yaw)) ||
                              (std::isfinite(last_goal_yaw) && std::isfinite(gi_.goal_yaw) &&
                               std::fabs(last_goal_yaw - gi_.goal_yaw) < 0.02);
        if (had_goal && (last_goal_p - gi_.goal_p).norm() < 0.05 && same_yaw) {
            return;
        }

        started_ = true;
        gi_.new_goal = true;
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
    }
}
