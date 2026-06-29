/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

namespace general_planner {

    void GeneralPlanner::updateDynamicObstacleCloud(const rog_map::PointCloud &cloud,
                                                    const Vec3f &robot_pos,
                                                    const double stamp) {
        if (dynamic_obstacle_layer_ == nullptr || !dynamic_obstacle_layer_->enabled()) {
            return;
        }
        dynamic_obstacle_layer_->updateCloud(cloud, robot_pos, stamp);
    }

    bool GeneralPlanner::dynamicObstacleLayerEnabled() const {
        return dynamic_obstacle_layer_ != nullptr && dynamic_obstacle_layer_->enabled();
    }

    void GeneralPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_manager_->getRobotState();
        out = robot_state_;
    }

    void GeneralPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        double eval_t = (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        traj_finish = false;
        double total_dur = cmd_traj_info_.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = cmd_traj_info_.getStartWallTime();
        if (cmd_traj_info_.backupTrajAvilibale() && eval_t > cmd_traj_info_.getBackupTrajStartTT()) {
            robot_on_backup_traj_ = true;
        } else {
            robot_on_backup_traj_ = false;
        }
    }

    Trajectory GeneralPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory GeneralPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }

    double GeneralPlanner::getCommittedTrajectoryRemainingDuration() {
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty()) {
            cmd_traj_info_.unlock();
            return 0.0;
        }
        const double remaining = cmd_traj_info_.getTotalDuration() -
                                 (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        cmd_traj_info_.unlock();
        return std::max(0.0, remaining);
    }

}
