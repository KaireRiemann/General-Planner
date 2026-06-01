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

#include <iostream>
#include <fstream>
#include <cmath>
#include <memory>
#include "Eigen/Eigen"


#include <general_core/config.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/traj_manager.h"
#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "map_manager/map_manager.hpp"
#include "general_core/corridor_generator.h"
#include "general_core/fov_checker.h"
#include "general_core/tracking_perching_frontend.hpp"
#include "general_core/tracking_runtime_manager.hpp"
#include "general_core/perching_runtime_manager.hpp"
#include "general_core/takeoff_frontend.hpp"
#include "general_core/takeoff_runtime_manager.hpp"
#include "general_core/tracking_perching_transition_manager.hpp"
#include "general_core/tracking_to_perching_initializer.hpp"
#include "general_core/se3_aggressive_manager.hpp"
#include "exploration/epic_exploration_manager.hpp"

#include "general_core/general_ret_code.hpp"
#include "utils/header/fmt_eigen.hpp"

#include <general_core/log_utils.hpp>
#include <data_structure/exp_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/backup_traj.h>


namespace general_planner {
    using namespace color_text;
    using namespace geometry_utils;

    class GeneralPlanner {
        LogOneReplan latest_replan;
        general_planner::Config cfg_;
        MapManager::Ptr map_manager_;
        CorridorGenerator::Ptr cg_ptr_;
        path_search::Astar::Ptr astar_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;
        Vec3f shifted_sfc_start_pt_;

        traj_opt::TrajManager::Ptr traj_manager_;
        traj_opt::SwarmTrajectoriesConstPtr swarm_trajs_;

        CIRI::Ptr ciri_;

        super_utils::RobotState robot_state_;

        std::mutex drone_state_mutex_;
        mutable std::mutex replan_lock_;
        std::mutex swarm_traj_mutex_;

        Vec3f local_start_p_;

        bool robot_on_backup_traj_{false};
        // use negative value to indicate the traj is not available
        double on_backup_start_WT{-1}, on_backup_end_WT{-1};

        double planner_process_start_WT_;

        struct GoalInfo {
            Vec3f goal_p{0, 0, 0};
            double goal_yaw{0};
            bool new_goal{true};
            bool goal_valid{true};
        } gi_;

        FOVChecker::Ptr fov_checker_;

        CmdTraj cmd_traj_info_;
        ExpTraj last_exp_traj_info_;
        std::unique_ptr<TrackingRuntimeManager> tracking_runtime_manager_;
        std::unique_ptr<PerchingRuntimeManager> perching_runtime_manager_;
        std::unique_ptr<TakeoffFrontend> takeoff_frontend_;
        std::unique_ptr<TakeoffRuntimeManager> takeoff_runtime_manager_;
        std::unique_ptr<traj_opt::DynamicTakeoffSnapTrajOpt> takeoff_optimizer_;
        traj_opt::DynamicTakeoffProblem active_takeoff_problem_;
        bool active_takeoff_problem_valid_{false};
        std::unique_ptr<TrackingPerchingTransitionManager> tracking_perching_manager_;
        std::unique_ptr<TrackingToPerchingInitializer> tracking_to_perching_initializer_;
        std::unique_ptr<SE3AggressiveManager> se3_aggressive_manager_;
        std::unique_ptr<exploration::EpicExplorationManager> exploration_manager_;
        exploration::ExplorationGoal latest_exploration_goal_;
        exploration::ExplorationPlan latest_exploration_plan_;
        exploration::ExplorationPlan active_exploration_plan_;
        bool active_exploration_guide_{false};
        enum class ExplorationRuntimeState {
            WAIT_OBSERVATION,
            UPDATE_GLOBAL,
            PLAN_LOCAL,
            EXEC_LOCAL,
            RECOVER,
            FINISH
        };
        ExplorationRuntimeState exploration_runtime_state_{ExplorationRuntimeState::WAIT_OBSERVATION};
        double exploration_last_global_update_wt_{-1.0};
        double exploration_last_local_commit_wt_{-1.0};
        double exploration_last_runtime_log_wt_{-1.0};
        bool exploration_has_committed_local_traj_{false};

        vector<double> time_consuming_;

        int tracking_consecutive_keep_old_{0};
        int tracking_consecutive_reject_{0};
        double last_tracking_commit_wt_{-1.0};
        std::string last_tracking_commit_reject_reason_;
        std::size_t last_tracking_diag_guide_path_size_{0};
        std::size_t last_tracking_diag_sfc_size_{0};
        std::size_t last_tracking_diag_target_prediction_size_{0};
        double last_tracking_diag_out_traj_duration_{0.0};
        traj_opt::DynamicTargetStates last_tracking_frontend_prediction_;
        vec_E<Vec3f> last_tracking_frontend_viewpoints_;

        struct TrackingTrajectoryActivity {
            bool valid{false};
            bool safe{false};
            bool active{false};
            bool target_moving{false};
            bool target_vertical_moving{false};

            double remaining{0.0};
            double speed0{0.0};
            double speed_xy{0.0};
            double speed_z{0.0};
            double speed_3d{0.0};
            double displacement{0.0};
            double displacement_xy{0.0};
            double displacement_z{0.0};
            double displacement_3d{0.0};
            double progress{0.0};
            double progress_xy{0.0};
            double progress_3d{0.0};
            double expected_progress{0.0};
            double target_speed_xy{0.0};
            double target_speed_z{0.0};
            double target_speed_3d{0.0};
            double tracking_error{0.0};
            double avg_tracking_error{0.0};

            std::string reason;
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit GeneralPlanner(const std::string &cfg_path,
                              const ros_interface::RosInterface::Ptr &ros_ptr,
                              const rog_map::ROGMapROS::Ptr &map_ptr);

        ~GeneralPlanner() = default;

        void lockCommittedTraj() {
            cmd_traj_info_.lock();
        }

        void unlockCommittedTraj() {
            cmd_traj_info_.unlock();
        }

        bool goalValid() const {
            return gi_.goal_valid;
        }

        typedef std::shared_ptr<GeneralPlanner> Ptr;

        void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);

        Trajectory getCommittedPositionTrajectory();

        Trajectory getCommittedYawTrajectory();

        double getCommittedTrajectoryRemainingDuration();

        bool trackingPerchingPerchingActive() const;

        bool trackingPerchingContactReached() const;

        TrackingPerchingTransitionManager::Status trackingPerchingStatus() const;

        void markTrackingPerchingContact();

        void getOneCommandFromTraj(StatePVAJ &pvaj,
                                   double &yaw,
                                   double &yaw_dot,
                                   bool &on_backup_traj,
                                   bool &traj_finish);

        void getModuleTimeConsuming(vector<double> &time);

        double getLatestOptimizationTime() const {
            if (time_consuming_.size() <= LogTime::BACK_TRAJ_OPT) {
                return 0.0;
            }
            return time_consuming_[LogTime::EXP_TRAJ_OPT] +
                   time_consuming_[LogTime::BACK_TRAJ_OPT];
        }

        double getLatestExpFrontendTime() const {
            return time_consuming_.size() > LogTime::EPX_TRAJ_FRONTEND
                   ? time_consuming_[LogTime::EPX_TRAJ_FRONTEND]
                   : 0.0;
        }

        double getLatestExpOptimizationTime() const {
            return time_consuming_.size() > LogTime::EXP_TRAJ_OPT
                   ? time_consuming_[LogTime::EXP_TRAJ_OPT]
                   : 0.0;
        }

        double getLatestBackupFrontendTime() const {
            return time_consuming_.size() > LogTime::BACK_TRAJ_FRONTEND
                   ? time_consuming_[LogTime::BACK_TRAJ_FRONTEND]
                   : 0.0;
        }

        double getLatestBackupOptimizationTime() const {
            return time_consuming_.size() > LogTime::BACK_TRAJ_OPT
                   ? time_consuming_[LogTime::BACK_TRAJ_OPT]
                   : 0.0;
        }

        double getLatestTotalReplanTime() const {
            return time_consuming_.size() > LogTime::TOTAL_REPLAN
                   ? time_consuming_[LogTime::TOTAL_REPLAN]
                   : 0.0;
        }

        double getLatestCorridorTime() const {
            return cg_ptr_ ? cg_ptr_->getCiriComputationTime() : -1.0;
        }

        int getLatestMvieLbfgsIterations() const {
            return cg_ptr_ ? cg_ptr_->getCiriMvieLbfgsIterations() : 0;
        }

        std::string getEllipsoidOptimizerName() const {
            return cfg_.ellipsoid_optimizer;
        }

        void setSwarmTrajectories(const traj_opt::SwarmTrajectories &trajectories);

        /* Tow type of replan strategy */
        RET_CODE PlanFromRest(const Vec3f &goal_p,
                              const double &goal_yaw,
                              const bool &new_goal);

        RET_CODE
        ReplanOnce(const Vec3f &goal_p,
                   const double &goal_yaw,
                   const bool &new_goal);

        RET_CODE PlanTrackingFromRest(const traj_opt::DynamicTargetStates &target_prediction,
                                      const bool &new_task);

        RET_CODE ReplanTrackingOnce(const traj_opt::DynamicTargetStates &target_prediction,
                                    const bool &new_task);

        RET_CODE ReplanTrackingOnce(const traj_opt::DynamicTargetStates &target_prediction,
                                    const traj_opt::PerchingSurfaceState &surface,
                                    const bool &new_task);

        void setTrackingPerchingRequest(bool request);

        RET_CODE TryCommitPerchingFromTracking(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            RET_CODE tracking_ret);

        RET_CODE PlanPerchingFromRest(const traj_opt::PerchingSurfaceState &surface,
                                      const bool &new_task);

        RET_CODE ReplanPerchingOnce(const traj_opt::PerchingSurfaceState &surface,
                                    const bool &new_task);

        RET_CODE PlanDynamicTakeoffFromRest(const traj_opt::PerchingSurfaceState &surface,
                                            const bool &new_task);

        RET_CODE ReplanDynamicTakeoffOnce(const traj_opt::PerchingSurfaceState &surface,
                                          const bool &new_task);

        RET_CODE PlanExplorationFromRest(const bool &new_task);

        RET_CODE ReplanExplorationOnce(const bool &new_task);

        RET_CODE PlanExplorationOnce(const bool &new_task,
                                     const bool &from_rest);

        struct ExplorationRuntimePolicy {
            double global_update_dt{0.2};
            double replan_time_after_traj_start{0.5};
            double replan_time_before_traj_end{0.5};
            double replan_forward_dt{0.2};
            double min_remaining_for_replan{0.25};
            double stop_traj_time{0.2};
            double collision_replan_time{0.5};
        };

        struct ExplorationExecutionStatus {
            bool has_observation{false};
            bool has_active_trajectory{false};
            bool trajectory_unsafe{false};
            double traj_elapsed{0.0};
            double traj_remaining{0.0};
            double collision_time{-1.0};
        };

        struct ExplorationFrontendUpdateResult {
            bool ready{false};
            bool updated{false};
            bool finished{false};
            bool no_frontier{false};
            exploration::ExplorationPlan plan;
            std::string reason;
        };

        ExplorationRuntimePolicy getExplorationRuntimePolicy() const;

        ExplorationExecutionStatus getExplorationExecutionStatus(double now);

        ExplorationFrontendUpdateResult refreshExplorationGlobalPlan();

        void resetExplorationTaskRuntime(bool hard_reset);

        bool truncateActiveExplorationTrajectory(double stop_time);

        bool getLatestExplorationGoal(exploration::ExplorationGoal &goal) const;

        bool explorationObservationReady() const {
            return exploration_manager_ != nullptr && exploration_manager_->hasObservation();
        }

        double latestExplorationObservationStamp() const {
            return exploration_manager_ != nullptr ? exploration_manager_->lastObservationStamp() : -1.0;
        }

        bool globalExplorationMapReady() const {
            return map_manager_ != nullptr && map_manager_->globalExplorationMapReady();
        }

        int globalFrontierCount() const {
            return map_manager_ != nullptr ? map_manager_->globalFrontierCount() : 0;
        }

        double globalExploredVolume() const {
            return map_manager_ != nullptr ? map_manager_->globalExploredVolume() : 0.0;
        }

        int globalPointCloudSize() const {
            return map_manager_ != nullptr ? map_manager_->globalPointCloudSize() : 0;
        }

        RET_CODE PlanSE3AggressiveFromRest(const Vec3f &goal_p,
                                           double goal_yaw,
                                           bool new_task);

        RET_CODE ReplanSE3AggressiveOnce(const Vec3f &goal_p,
                                         double goal_yaw,
                                         bool new_task);

    private:
        RET_CODE generateExpTraj(ExpTraj &last_exp_traj_info,
                                 ExpTraj &out_exp_traj_info);

        RET_CODE generateExpTrajFromGuidePath(const exploration::ExplorationPlan &plan,
                                              ExpTraj &last_exp_traj_info,
                                              ExpTraj &out_exp_traj_info);

        /* For Backup traj generation */
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info);

        RET_CODE tryCommitExplorationBackupFallback(const std::string &reason);

        void resetExplorationRuntimeState(bool hard_reset);

        static const char *explorationRuntimeStateName(ExplorationRuntimeState state);

        bool getExplorationCommittedTrajectoryActivity(double now,
                                                       double &elapsed,
                                                       double &remaining);

        bool currentExplorationTrajectoryUnsafe(double now,
                                                double *collision_time = nullptr);

        bool truncateExplorationCommittedTrajectory(double stop_time);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path);

        bool prepareESDFGuideEndpoint(vec_Vec3f &guide_path,
                                      std::vector<double> &guide_stamp);

        bool buildTrackingGuideCorridor(traj_opt::TrackingProblem &problem,
                                        std::string *failure_reason = nullptr);

        bool tryGenerateTrackingCorridor(const vec_Vec3f &guide_path,
                                         PolytopeVec &sfcs,
                                         std::string *failure_reason = nullptr);

        bool repairTrackingGuideWithAstar(const vec_Vec3f &guide_path,
                                          const std::vector<double> &guide_t,
                                          vec_Vec3f &repaired_path,
                                          std::vector<double> &repaired_t,
                                          std::string *failure_reason = nullptr);

        bool densifyTrackingGuideForCorridor(const vec_Vec3f &guide_path,
                                             const std::vector<double> &guide_t,
                                             vec_Vec3f &dense_path,
                                             std::vector<double> &dense_t) const;

        bool truncateTrackingProblemForCorridor(traj_opt::TrackingProblem &problem,
                                                const vec_Vec3f &candidate_guide,
                                                const std::vector<double> &candidate_guide_t,
                                                PolytopeVec &sfcs,
                                                std::string *failure_reason = nullptr);

        bool trackingGuidePointSafe(const Vec3f &point) const;

        void refreshTrackingGuideTiming(traj_opt::TrackingProblem &problem) const;
        void refreshTrackingGuideEndpoint(traj_opt::TrackingProblem &problem) const;
        bool findTrackingViewpointReference(
            const traj_opt::DynamicTargetStates &target_prediction,
            Vec3f &reference_viewpoint,
            traj_opt::DynamicTargetState &reference_target) const;
        void rememberTrackingViewpointReference(const traj_opt::TrackingProblem &problem);

        StatePVAJ makeTaskHeadState(const bool &from_rest);

        bool commitTaskTrajectory(const Trajectory &pos_traj,
                                  const double &terminal_yaw,
                                  const bool &fix_terminal_yaw,
                                  const std::string &traj_ns);

        bool buildTrackingTargetYawTrajectory(const Trajectory &pos_traj,
                                              const traj_opt::DynamicTargetStates &target_prediction,
                                              Trajectory &yaw_traj);

        bool commitTrackingTrajectory(const Trajectory &pos_traj,
                                      const Trajectory &yaw_traj,
                                      const traj_opt::DynamicTargetStates &target_prediction,
                                      const std::string &traj_ns);

        bool buildPerchingYawTrajectory(const Trajectory &pos_traj,
                                        const traj_opt::PerchingSurfaceState &surface,
                                        Trajectory &yaw_traj);

        bool buildPerchingYawTrajectoryFromHead(const Trajectory &pos_traj,
                                                const traj_opt::PerchingSurfaceState &surface,
                                                const Eigen::Matrix<double, 1, 2> &head_yaw,
                                                Trajectory &yaw_traj);

        bool commitPerchingTrajectory(const Trajectory &pos_traj,
                                      const Trajectory &yaw_traj,
                                      const std::string &traj_ns);

        bool commitTakeoffTrajectory(const Trajectory &pos_traj,
                                     const std::string &traj_ns);

        bool commitTrackingToPerchingTrajectory(const Trajectory &tracking_pos,
                                                const Trajectory &tracking_yaw,
                                                double current_tracking_local_t,
                                                double handover_delay,
                                                const Trajectory &perching_pos,
                                                const Trajectory &perching_yaw,
                                                const std::string &traj_ns);

        bool trackingTrajectorySafeForHorizon(const Trajectory &traj,
                                              double start_t,
                                              double horizon,
                                              double dt) const;

        bool currentTrackingTrajectorySafeForHorizon(double horizon);

        bool keepOldTrackingTrajectory(const std::string &reason);

        TrackingTrajectoryActivity evaluateTrackingTrajectoryActivity(
            const Trajectory &traj,
            double local_start_t,
            const traj_opt::DynamicTargetStates &target_prediction,
            double horizon,
            double dt) const;

        bool currentTrackingTrajectorySafeAndActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            TrackingTrajectoryActivity *activity = nullptr) const;

        bool candidateTrackingTrajectoryCommandable(
            const Trajectory &candidate_pos_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            double candidate_eval_start_t = 0.0,
            double target_eval_start_t = 0.0,
            std::string *reason = nullptr) const;

        bool keepOldTrackingTrajectoryIfActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            const std::string &reason);

        bool trackingCandidateSafeForCommit(const Trajectory &candidate_pos_traj) const;

        bool trackingSnapshotSatisfiesFovForKeepOld(
            const Trajectory &pos_traj,
            const Trajectory &yaw_traj,
            double local_start_t,
            const traj_opt::DynamicTargetStates &target_prediction,
            std::string *reason = nullptr) const;

        bool trackingTrajectorySatisfiesFov(const Trajectory &pos_traj,
                                            const Trajectory &yaw_traj,
                                            const traj_opt::DynamicTargetStates &target_prediction,
                                            double start_t,
                                            double horizon,
                                            double dt,
                                            double target_start_t,
                                            std::string *reason = nullptr) const;

        void resetTrackingCommitCounters();

        double trackingViewpointErrorScore(const Vec3f &tracker,
                                           const Vec3f &target) const;

        bool trackingCommitPassesAntiRollback(const Trajectory &candidate_pos_traj,
                                              const traj_opt::DynamicTargetStates &target_prediction,
                                              double commit_wt,
                                              double candidate_eval_start_t = 0.0,
                                              double target_eval_start_t = 0.0,
                                              bool candidate_safe = true,
                                              bool candidate_fov_ok = true,
                                              int *worse_count_out = nullptr,
                                              double *max_regression_out = nullptr,
                                              std::string *reason = nullptr);

        bool optimizeTrackingProblemWithRetries(
            const traj_opt::TrackingProblem &normal_problem,
            const traj_opt::DynamicTargetStates &active_target_prediction,
            Trajectory &out_traj,
            Trajectory &out_yaw_traj,
            std::string *failure_reason);

        bool applyTrackingNarrowPassageSoftDistance(traj_opt::TrackingProblem &problem,
                                                    std::string *reason = nullptr) const;

        RET_CODE optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                      const bool &from_rest);

        RET_CODE optimizePerchingTask(const traj_opt::PerchingSurfaceState &surface,
                                      const bool &from_rest);

        RET_CODE optimizeDynamicTakeoffTask(const traj_opt::PerchingSurfaceState &surface,
                                            const bool &from_rest);

        RET_CODE optimizeSE3AggressiveTask(const Vec3f &goal_p,
                                           double goal_yaw,
                                           const bool &from_rest);

        bool commitSE3AggressiveTrajectory(const Trajectory &pos_traj,
                                           const std::string &traj_ns);

        PerchingFrontend::Config makePerchingFrontendConfig() const;

        TakeoffFrontend::Config makeTakeoffFrontendConfig() const;

        exploration::EpicExplorationManager::Config makeExplorationConfig() const;

        GlobalExplorationMapConfig makeGlobalExplorationMapConfig() const;

        GlobalPointCloudMapConfig makeGlobalPointCloudMapConfig() const;

        GlobalRegionGridConfig makeGlobalRegionGridConfig() const;

        RET_CODE tryCommitPerchingFromTracking(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            RET_CODE tracking_ret);

    public:
        void getRobotState(rog_map::RobotState &out);

        bool isEasyGoal(const Vec3f &goal_position);

        MapManager::Ptr getMapManager() const {
            return map_manager_;
        }

        double ft{0}, bt{0};
        int ft_cnt{0}, bt_cnt{0};

        double getFrontendTime() {
            if (ft_cnt == 0) return -1;
            double ave_t = ft / ft_cnt;
            ft = 0;
            ft_cnt = 0;
            return ave_t;
        }

        double getBackendTime() {
            if (bt_cnt == 0) return -1;
            double ave_t = bt / bt_cnt;
            bt = 0;
            bt_cnt = 0;
            return ave_t;
        }

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) {
            updateExplorationMaps(cloud, pose, CloudFrame::WORLD);
        }

        void updateROGMapWithGlobal(const rog_map::PointCloud &cloud,
                                    const super_utils::Pose &pose,
                                    CloudFrame frame) {
            updateExplorationMaps(cloud, pose, frame);
        }

        void updateExplorationMaps(const rog_map::PointCloud &cloud,
                                   const super_utils::Pose &pose,
                                   CloudFrame frame) {
            const double stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
            rog_map::RobotState robot = map_manager_->getRobotState();
            updateExplorationMapsWithRobot(cloud, pose, frame, robot, stamp);
        }

        void updateExplorationMapsWithRobot(const rog_map::PointCloud &cloud,
                                            const super_utils::Pose &pose,
                                            CloudFrame frame,
                                            rog_map::RobotState robot,
                                            double stamp) {
            if (stamp <= 0.0) {
                stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
            }
            const bool use_epic_exploration_maps =
                    cfg_.exploration_enable && cfg_.exploration_use_epic_frontend;
            const bool update_rog_map =
                    !use_epic_exploration_maps || cfg_.exploration_update_rog_map;
            if (update_rog_map) {
                map_manager_->updateMapWithGlobal(cloud, pose, frame, stamp, true);
            }
            if (exploration_manager_ != nullptr) {
                if (!robot.rcv) {
                    robot.rcv = true;
                    robot.p = pose.first;
                    robot.q = pose.second;
                    robot.v.setZero();
                    robot.a.setZero();
                    robot.j.setZero();
                    robot.yaw = std::atan2(2.0 * (pose.second.w() * pose.second.z() +
                                                   pose.second.x() * pose.second.y()),
                                                       1.0 - 2.0 * (pose.second.y() * pose.second.y() +
                                                                    pose.second.z() * pose.second.z()));
                }
                robot.rcv_time = stamp;
                map_manager_->updateEpicLioMap(cloud, pose, frame, robot);
                exploration_manager_->onCloudOdom(cloud, pose, frame, robot, stamp);
            }
        }

        void updateGlobalMapOnly(const rog_map::PointCloud &cloud,
                                 const super_utils::Pose &pose,
                                 CloudFrame frame) {
            const double stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
            map_manager_->updateGlobalMapsOnly(cloud, pose, frame, stamp);
            if (exploration_manager_ != nullptr) {
                auto robot = map_manager_->getRobotState();
                if (!robot.rcv) {
                    robot.rcv = true;
                    robot.p = pose.first;
                    robot.q = pose.second;
                    robot.v.setZero();
                    robot.a.setZero();
                    robot.j.setZero();
                }
                robot.rcv_time = stamp;
                map_manager_->updateEpicLioMap(cloud, pose, frame, robot);
                exploration_manager_->onCloudOdom(cloud, pose, frame, robot, stamp);
            }
        }

        LogOneReplan getLatestReplanLog() {
            latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
            latest_replan.setComptT(time_consuming_);
            return latest_replan;
        }
    };
}
