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
#include <memory>
#include "Eigen/Eigen"


#include <general_core/config.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/traj_manager.h"
#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "general_core/map_manager.hpp"
#include "general_core/corridor_generator.h"
#include "general_core/fov_checker.h"
#include "general_core/exploration_frontend.hpp"
#include "general_core/exploration_runtime_manager.hpp"
#include "general_core/tracking_perching_frontend.hpp"
#include "general_core/tracking_runtime_manager.hpp"
#include "general_core/perching_runtime_manager.hpp"
#include "general_core/tracking_perching_transition_manager.hpp"
#include "general_core/tracking_to_perching_initializer.hpp"

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
        std::unique_ptr<TrackingPerchingTransitionManager> tracking_perching_manager_;
        std::unique_ptr<TrackingToPerchingInitializer> tracking_to_perching_initializer_;
        std::unique_ptr<ExplorationFrontend> exploration_frontend_;
        std::unique_ptr<ExplorationRuntimeManager> exploration_runtime_manager_;

        vector<double> time_consuming_;

        int tracking_consecutive_keep_old_{0};
        int tracking_consecutive_reject_{0};
        double last_tracking_commit_wt_{-1.0};

        struct TrackingTrajectoryActivity {
            bool valid{false};
            bool safe{false};
            bool active{false};
            bool target_moving{false};

            double remaining{0.0};
            double speed0{0.0};
            double displacement{0.0};
            double progress{0.0};
            double expected_progress{0.0};
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

        RET_CODE PlanPerchingFromRest(const traj_opt::PerchingSurfaceState &surface,
                                      const bool &new_task);

        RET_CODE ReplanPerchingOnce(const traj_opt::PerchingSurfaceState &surface,
                                    const bool &new_task);

        RET_CODE PlanExplorationFromRest(const bool &new_task);

        RET_CODE ReplanExplorationOnce(const bool &new_task);

        bool getLatestExplorationGoal(ExplorationGoal &goal) const;

    private:
        RET_CODE generateExpTraj(ExpTraj &last_exp_traj_info,
                                 ExpTraj &out_exp_traj_info);

        /* For Backup traj generation */
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info);

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
            std::string *reason = nullptr) const;

        bool keepOldTrackingTrajectoryIfActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            const std::string &reason);

        bool trackingCandidateSafeForCommit(const Trajectory &candidate_pos_traj) const;

        void resetTrackingCommitCounters();

        double trackingViewpointErrorScore(const Vec3f &tracker,
                                           const Vec3f &target) const;

        bool trackingCommitPassesAntiRollback(const Trajectory &candidate_pos_traj,
                                              const traj_opt::DynamicTargetStates &target_prediction,
                                              double commit_wt);

        RET_CODE optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                      const bool &from_rest);

        RET_CODE optimizePerchingTask(const traj_opt::PerchingSurfaceState &surface,
                                      const bool &from_rest);

        PerchingFrontend::Config makePerchingFrontendConfig() const;

        ExplorationFrontend::Config makeExplorationFrontendConfig() const;

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

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) const {
            map_manager_->updateMap(cloud, pose);
        }

        LogOneReplan getLatestReplanLog() {
            latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
            latest_replan.setComptT(time_consuming_);
            return latest_replan;
        }
    };
}
