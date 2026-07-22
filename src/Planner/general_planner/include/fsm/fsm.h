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

#include <queue>
#include <memory>
#include <fstream>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <limits>
#include <utility>
#include <fmt/color.h>
#include <cereal/archives/binary_file_handler.hpp>
#include <fsm/config.hpp>
#include <fsm/goal_height_policy.hpp>
#include <general_core/commit_governor.hpp>
#include <general_core/general_planner.h>
#include <general_core/mission_orchestrator.hpp>
#include <general_core/planner_context.hpp>
#include <general_core/safety_monitor.hpp>
#include <general_core/task_plugin.hpp>


#ifndef LOG_FILE_DIR
#define LOG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#endif

namespace fsm {
    class Fsm {
    protected:
        bool stop{false};


        vector<string> log_time_str{
                "TIME_STAMPE", "EPX_TRAJ_FRONTEND",
                "EXP_TRAJ_OPT", "GENERATE_EXP_TRAJ",
                "BACK_TRAJ_FRONTEND", "BACK_TRAJ_OPT",
                "GENERATE_BACK_TRAJ", "TOTAL_REPLAN", "VISUALIZATION"
        };
        Config cfg_;
        general_planner::architecture::MissionOrchestrator mission_orchestrator_;
        general_planner::architecture::SafetyMonitor safety_monitor_;
        general_planner::architecture::CommitGovernor commit_governor_;
        // map, checker, planner
        general_planner::GeneralPlanner::Ptr planner_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;

        std::ofstream write_time_;
        vector<double> log_module_time;
        double yaw_{0}, yaw_dot_{0};

        rog_map::RobotState robot_state_;

        // params
        bool started_{false}, plan_from_rest_{false};

    public:
        struct DiagnosticEvent {
            double stamp{0.0};
            uint64_t replan_id{0};
            int replan_log_id{-1};
            std::string level;
            std::string event;
            std::string task_mode;
            std::string machine_state;
            std::string detail;
            Vec3f goal_p{Vec3f::Zero()};
            double goal_yaw{std::numeric_limits<double>::quiet_NaN()};
            Vec3f robot_p{Vec3f::Zero()};
            Vec3f robot_v{Vec3f::Zero()};
            bool robot_state_received{false};
            int traj_seq{-1};
            int ret_code{-1};
            double total_replan_time_ms{0.0};
            double remaining_traj_s{0.0};
            bool on_backup{false};
        };

    protected:
        struct GoalInfo {
            bool new_goal{false};
            Vec3f goal_p{Vec3f::Zero()};
            double goal_yaw{0.0};
        } gi_;

        Eigen::Vector3d auto_pilot_vel_w_;

        traj_opt::DynamicTargetStates tracking_target_prediction_;
        traj_opt::PerchingSurfaceState perching_surface_;
        double tracking_target_rcv_time_{-1.0};
        double perching_surface_rcv_time_{-1.0};
        double perching_surface_first_rcv_time_{-1.0};
        double last_dynamic_takeoff_wait_log_time_{-1.0};
        bool task_new_{false};
        bool perching_contact_reached_{false};
        Vec3f perching_contact_surface_position_{Vec3f::Zero()};
        double last_static_tracking_replan_log_time_{-1.0};
        std::mutex task_mutex_;

        // execution states
        enum MACHINE_STATE {
            INIT = 0,
            WAIT_GOAL,
            YAWING,
            GENERATE_TRAJ,
            FOLLOW_TRAJ,
            STATIC_TRACKING,
            HOLD_TRACKING,
            EMER_STOP
        };

        vector<string> MACHINE_STATE_STR{
                "INIT",
                "WAIT_GOAL",
                "YAWING",
                "GENERATE_TRAJ",
                "FOLLOW_TRAJ",
                "STATIC_TRACKING",
                "HOLD_TRACKING",
                "EMER_STOP"
        };


        MACHINE_STATE machine_state_{INIT};

        using TaskPlanContext = general_planner::architecture::TaskPlanContext;
        using PlanRequest = general_planner::architecture::PlanRequest;
        using PlanResult = general_planner::architecture::PlanResult;

        struct ExecutedTrajectoryFinishResult {
            bool tracking_unfinished{false};
            bool state_changed{false};
        };

        class TaskExecutor : public general_planner::architecture::TaskPlugin<Fsm> {
        public:
            virtual ~TaskExecutor() = default;

            virtual TaskMode mode() const = 0;
            virtual const char *name() const = 0;
            virtual general_planner::architecture::TaskType taskType() const {
                return taskTypeFromTaskMode(mode());
            }
            virtual general_planner::architecture::MissionMode missionMode() const {
                return missionModeFromTaskMode(mode());
            }
            virtual general_planner::architecture::TaskIdentity identity(const Fsm &fsm) const {
                general_planner::architecture::TaskIdentity identity;
                identity.mission = missionMode();
                identity.task = taskType();
                identity.backend = fsm.cfg_.backend_type;
                identity.plugin_name = name();
                identity.goal_like = goalLike();
                identity.tracking_like = trackingLike();
                return identity;
            }
            virtual bool trackingLike() const {
                return false;
            }
            virtual bool goalLike() const {
                return false;
            }
            virtual bool ready(Fsm &fsm) = 0;
            virtual bool replanAllowed(const Fsm &fsm) const = 0;
            virtual PlanResult plan(Fsm &fsm, const PlanRequest &request) = 0;
            virtual PlanResult replan(Fsm &fsm, const PlanRequest &request) = 0;
            virtual bool shouldGenerateAfterTrajFinish(Fsm &fsm) = 0;

        protected:
            PlanResult makeResult(const Fsm &fsm,
                                  const PlanRequest &request,
                                  const int ret_code,
                                  TaskPlanContext context = {},
                                  std::string detail = "") const {
                PlanResult result;
                result.request = request;
                result.request.identity = identity(fsm);
                result.context = std::move(context);
                result.ret_code = ret_code;
                result.detail = std::move(detail);
                return result;
            }
        };

        class State2StateTaskExecutor;
        class TrackingTaskExecutor;
        class TrackingPerchingMissionAdapter;
        class PerchingTaskExecutor;
        class DynamicTakeoffTaskExecutor;
        class ExplorationTaskExecutor;

        std::unique_ptr<TaskExecutor> task_executor_;
        TaskMode task_executor_mode_{TaskMode::STATE_TO_STATE};


    public:
        Fsm() = default;
        ~Fsm();

        void updateROGMap(const rog_map::PointCloud &cloud, const general_utils::Pose &pose) {
            planner_ptr_->updateROGMap(cloud, pose);
        }

        void callPlanOnce(const Vec3f &goal) {
            general_utils::TimeConsuming tc("Call replan once time", true);
            fmt::print(" -- [Fsm] Call plan once, cur state {}.\n", MACHINE_STATE_STR[machine_state_]);
            // check current state;
            Quatf q(NAN, NAN, NAN, NAN);
            setGoalPosiAndYaw(goal, q);

            callMainFsmOnce();

            if (machine_state_ == FOLLOW_TRAJ) {
                callReplanOnce();
            }

            // save on log
            const auto log_id = appendLatestReplanLog();
            fmt::print(fmt::fg(fmt::color::green), " -- Replan ID: {}, ret code: {}\n",
                       log_id.first, log_id.second);
        }

        Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
            double half_roll = roll * 0.5;
            double half_pitch = pitch * 0.5;
            double half_yaw = yaw * 0.5;

            double sin_r = std::sin(half_roll);
            double cos_r = std::cos(half_roll);
            double sin_p = std::sin(half_pitch);
            double cos_p = std::cos(half_pitch);
            double sin_y = std::sin(half_yaw);
            double cos_y = std::cos(half_yaw);

            // 计算四元数分量
            Eigen::Quaterniond q;
            q.w() = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y;
            q.x() = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y;
            q.y() = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y;
            q.z() = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y;

            return q;
        }

    protected:
        vector<LogOneReplan> replan_logs_;
        vector<LogOneReplan> tracking_replan_logs_;
        vector<DiagnosticEvent> diagnostic_events_;
        vector<DiagnosticEvent> tracking_diagnostic_events_;
        mutable std::mutex fsm_tick_mutex_;
        // State-to-state replanning may take longer than one FSM tick. Keep
        // the FSM lock free while it runs, but do not allow a plan-from-rest
        // request to enter the same planner concurrently.
        std::atomic<bool> state2state_replan_in_progress_{false};
        mutable std::mutex replan_logs_mutex_;
        mutable std::mutex diagnostic_events_mutex_;
        std::ofstream diagnostic_event_log_;
        std::ofstream tracking_diagnostic_event_log_;
        uint64_t next_replan_id_{1};
        uint64_t active_replan_id_{0};
        int state2state_plan_from_rest_fail_count_{0};
        int exploration_plan_from_rest_fail_count_{0};
        int tracking_plan_from_rest_fail_count_{0};
        double tracking_plan_from_rest_backoff_until_{-1.0};
        double last_tracking_plan_from_rest_backoff_log_time_{-1.0};
        bool perception_replan_requested_{false};
        bool perception_replan_emergency_{false};
        double last_perception_replan_request_time_{-1.0};
        double last_perception_replan_log_time_{-1.0};
        general_planner::GeneralPlanner::CommittedTrajectorySafetyReport perception_replan_report_;
        uint64_t tracking_target_input_seq_{0};
        double last_tracking_target_log_time_{-1.0};
        std::string last_tracking_target_log_source_;
        std::size_t last_tracking_target_log_size_{0};
        Vec3f last_tracking_target_log_front_{Vec3f::Zero()};
        Vec3f last_tracking_target_log_back_{Vec3f::Zero()};
        bool last_tracking_target_log_static_{false};
        /* Callback functions */
        bool finish_plan = false;
        double system_start_time_{0.0};

        bool traj_finish_{false};
        int last_tracking_unfinished_traj_seq_{-1};

        void WriteTimeToLog();

        std::pair<std::size_t, int> appendLatestReplanLog();

        vector<LogOneReplan> snapshotReplanLogs() const;

        vector<LogOneReplan> snapshotTrackingReplanLogs() const;

        void recordDiagnosticEvent(const std::string &level,
                                   const std::string &event,
                                   const std::string &detail = "",
                                   int ret_code = -1,
                                   int traj_seq = -1,
                                   bool on_backup = false,
                                   int replan_log_id = -1,
                                   uint64_t replan_id_override = std::numeric_limits<uint64_t>::max());

        void recordTrackingReplanContext(const std::string &event,
                                         int ret_code,
                                         bool prediction_static,
                                         std::size_t input_prediction_size,
                                         int replan_log_id = -1);

        void recordTrackingTargetInput(const std::string &source,
                                       const traj_opt::DynamicTargetStates &prediction,
                                       double source_stamp = -1.0,
                                       std::size_t raw_sample_count = 0);

        vector<DiagnosticEvent> snapshotDiagnosticEvents() const;

        vector<DiagnosticEvent> snapshotTrackingDiagnosticEvents() const;

        void openDiagnosticLogFile(const std::string &path);

        void openTrackingDiagnosticLogFile(const std::string &path);

        void saveDiagnosticLogToFile(const std::string &name = "");

        void saveTrackingDiagnosticLogToFile(const std::string &name = "");

        std::string diagnosticEventToString(const DiagnosticEvent &event) const;

        bool ensureLogParentDirectory(const std::string &path) const;

        bool useTrackingLogStream() const;

        virtual void publishDiagnosticEvent(const DiagnosticEvent &event) {}

        void callReplanOnce();

        void callPerceptionSafetyCheckOnce();

        void callMainFsmOnce();

        TaskExecutor &taskExecutor();

        std::unique_ptr<TaskExecutor> makeTaskExecutor(TaskMode mode) const;

        void resetTaskExecutor();

        general_planner::architecture::PlannerContext makePlannerContext();

        bool closeToGoal(const double &thresh_dis);

        void setGoalPosiAndYaw(
                const Vec3f &p,
                const Quatf &q,
                GoalHeightMode height_mode = GoalHeightMode::CONFIGURED_CLICK_HEIGHT);

        bool state2stateMode() const;

        bool trackingMode() const;

        bool trackingPerchingMode() const;

        bool perchingMode() const;

        bool dynamicTakeoffMode() const;

        bool explorationMode() const;

        bool se3AggressiveMode() const;

        void setTaskModeFromString(const std::string &mode);

        bool trackingTaskReady();

        bool perchingTaskReady();

        bool dynamicTakeoffTaskReady();

        bool activeTaskReady();

        bool shouldGenerateAfterTrajFinish();

        general_planner::architecture::ExecutionPhase executionPhase() const;

        general_planner::architecture::TaskIdentity activeTaskIdentity();

        const general_planner::architecture::MissionSnapshot &missionSnapshot() const;

        void resetTrackingPlanFromRestFailureState();

        bool trackingPlanFromRestBackoffActive();

        void handleTrackingPlanFromRestFailure(int retcode,
                                               bool prediction_static,
                                               std::size_t input_prediction_size);

        bool trackingExecutionState() const;

        bool trackingPerchingPerchingActive() const;

        bool markTrackingFinishedIfStaticTarget();

        ExecutedTrajectoryFinishResult handleExecutedTrajectoryFinished(
                const std::string &source,
                int trajectory_id,
                int trajectory_seq,
                bool on_backup,
                bool record_regular_finish,
                bool mark_static_target_finished);

        void logStaticTrackingReplanDecision(const std::string &reason);

        bool trackingCommittedTrajectoryUnsafe() const;

        traj_opt::DynamicTargetStates filterStaticTrackingPrediction(
                const traj_opt::DynamicTargetStates &prediction) const;

        bool trackingPredictionChanged(const traj_opt::DynamicTargetStates &a,
                                       const traj_opt::DynamicTargetStates &b) const;

        bool trackingPredictionStatic(const traj_opt::DynamicTargetStates &prediction) const;

        bool shouldSkipStaticTrackingReplan(const traj_opt::DynamicTargetStates &prediction);

        void setTrackingTargetPrediction(const traj_opt::DynamicTargetStates &prediction);

        void setPerchingSurface(const traj_opt::PerchingSurfaceState &surface);

        bool getTrackingTargetPrediction(traj_opt::DynamicTargetStates &prediction);

        bool getPerchingSurface(traj_opt::PerchingSurfaceState &surface);

        void ChangeState(const string &call_func, const MACHINE_STATE &new_state);

        virtual void publishPolyTraj() = 0;

        virtual void publishCurPoseToPath() = 0;

        virtual void resetVisualizedPath() = 0;
    };
}
