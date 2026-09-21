#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <cmath>

namespace general_planner {
bool GeneralPlanner::commitTrackingHoldTrajectory(const std::string &reason,
                                                   double duration,bool /*require_safe*/) {
    if(!robot_state_.rcv || !robot_state_.p.allFinite()) return false;
    const double now=ros_ptr_->getSimTime();
    general_utils::Vec3f p=robot_state_.p;
    // Elastic hover holds the last commanded yaw (traj_server last_yaw_),
    // not the unslewed setpoint stored in the previous polynomial.
    double yaw=tracking_yaw_servo_active_.load()?trackingYawServoYaw():robot_state_.yaw;
    if(!tracking_yaw_servo_active_.load() && !cmd_traj_info_.empty()) {
        cmd_traj_info_.lock();
        const auto old=cmd_traj_info_.posTraj(), old_yaw=cmd_traj_info_.yawTraj();
        const double t=now-cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();
        if(t>=0.0 && t<=old.getTotalDuration()) {
            p=old.getPos(t);
            if(!old_yaw.empty()) yaw=old_yaw.getPos(std::min(t,old_yaw.getTotalDuration())).x();
        }
    } else if(!cmd_traj_info_.empty()) {
        cmd_traj_info_.lock();
        const auto old=cmd_traj_info_.posTraj();
        const double t=now-cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();
        if(t>=0.0 && t<=old.getTotalDuration()) p=old.getPos(t);
    }
    if(!p.allFinite() || !std::isfinite(yaw)) return false;
    duration=std::max(1.0,duration);
    Eigen::Matrix<double,3,6> pc=Eigen::Matrix<double,3,6>::Zero();
    Eigen::Matrix<double,3,6> yc=Eigen::Matrix<double,3,6>::Zero();
    pc.col(5)=p;
    yc(0,5)=yaw;
    Trajectory position,hover_yaw;
    position.emplace_back(duration,pc);
    hover_yaw.emplace_back(duration,yc);
    position.start_WT=hover_yaw.start_WT=now;
    ExpTraj command;
    command.setGoalConnectedFlag(false);command.setWholeTrajKnownFreeFlag(true);
    command.setTrajectory(now,position,hover_yaw);
    cmd_traj_info_.setTrajectory(command);last_exp_traj_info_=command;
    setTrackingYawServo(true);
    robot_on_backup_traj_.store(false);gi_.new_goal=false;
    latest_replan.setExpTraj(position);latest_replan.setExpYawTraj(hover_yaw);
    latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
    tracking_runtime_manager_->onHold();
    setTrackingDiagnostic("hover",reason,0,0,0,duration);
    if(cfg_.visualization_en) {
        ros_ptr_->vizExpTraj(position,"tracking_hover");ros_ptr_->vizYawTraj(position,hover_yaw);
    }
    return true;
}
} // namespace general_planner
