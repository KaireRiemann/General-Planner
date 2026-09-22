#include <general_core/general_planner.h>

#include <algorithm>
#include <cmath>

namespace general_planner {

// In-place / near-field yaw adjustment.  Unlike YawTrajOpt (which allocates
// yaw waypoints along a moving position trajectory and back-loads the
// rotation when the position is stationary), this builds a single cubic yaw
// piece whose peak rate is exactly bounded by yaw_dot_max:
//   yaw(t) = yaw0 + 3*d/T^2 * t^2 - 2*d/T^3 * t^3,  peak rate = 1.5*|d|/T.
bool GeneralPlanner::commitReorientTrajectory(const Vec3f &target_position,
                                              const double &goal_yaw,
                                              const double &yaw_dot_max,
                                              const double &min_duration,
                                              const double &yaw_tolerance,
                                              const std::string &traj_ns) {
    if (!robot_state_.rcv || !robot_state_.p.allFinite() ||
        !std::isfinite(robot_state_.yaw) || !std::isfinite(goal_yaw) ||
        !target_position.allFinite()) {
        return false;
    }

    const double now = ros_ptr_->getSimTime();
    Vec3f start_p = robot_state_.p;
    double start_yaw = robot_state_.yaw;
    // Mid-trajectory commits must chain from the commanded state, not the
    // delayed odometry, or the new polynomial jumps at its head.
    if (!cmd_traj_info_.empty()) {
        cmd_traj_info_.lock();
        const auto old_pos = cmd_traj_info_.posTraj();
        const auto old_yaw = cmd_traj_info_.yawTraj();
        const double t = now - cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();
        if (t >= 0.0 && t <= old_pos.getTotalDuration()) {
            start_p = old_pos.getPos(t);
            if (!old_yaw.empty()) {
                start_yaw = old_yaw.getPos(std::min(t, old_yaw.getTotalDuration())).x();
            }
        }
    }
    if (!start_p.allFinite() || !std::isfinite(start_yaw)) {
        return false;
    }

    const double yaw_delta =
            std::atan2(std::sin(goal_yaw - start_yaw), std::cos(goal_yaw - start_yaw));
    const Vec3f pos_delta = target_position - start_p;
    const double pos_dist = pos_delta.norm();

    const double rate_limit = std::max(0.1, yaw_dot_max);
    double duration = std::max(min_duration, 1.5 * std::fabs(yaw_delta) / rate_limit);
    // Keep any allowed position drift slow: cubic peak velocity is
    // 1.5*|dp|/T, cap it at 0.5 m/s.
    if (pos_dist > 1.0e-3) {
        duration = std::max(duration, 3.0 * pos_dist);
    }
    duration = std::clamp(duration, 0.2, 30.0);

    if (std::fabs(yaw_delta) <= yaw_tolerance && pos_dist <= 1.0e-3) {
        ros_ptr_->info(" -- [GeneralPlanner] Reorient goal already satisfied: yaw_err={:.3f} rad",
                       std::fabs(yaw_delta));
    }

    const double t2 = duration * duration;
    const double t3 = t2 * duration;

    Eigen::Matrix<double, 3, 6> pc = Eigen::Matrix<double, 3, 6>::Zero();
    pc.col(5) = start_p;
    pc.col(3) = 3.0 * pos_delta / t2;
    pc.col(2) = -2.0 * pos_delta / t3;

    Eigen::Matrix<double, 3, 6> yc = Eigen::Matrix<double, 3, 6>::Zero();
    yc(0, 5) = start_yaw;
    yc(0, 3) = 3.0 * yaw_delta / t2;
    yc(0, 2) = -2.0 * yaw_delta / t3;

    Trajectory position, yaw;
    position.emplace_back(duration, pc);
    yaw.emplace_back(duration, yc);
    position.start_WT = yaw.start_WT = now;

    ExpTraj command;
    command.setGoalConnectedFlag(true);
    command.setWholeTrajKnownFreeFlag(true);
    command.setTrajectory(now, position, yaw);
    cmd_traj_info_.setTrajectory(command);
    last_exp_traj_info_ = command;
    // Non-tracking tasks fly their committed yaw polynomial exactly.
    setTrackingYawServo(false);
    robot_on_backup_traj_.store(false);
    gi_.new_goal = false;
    latest_replan.setExpTraj(position);
    latest_replan.setExpYawTraj(yaw);
    latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);

    ros_ptr_->info(" -- [GeneralPlanner] Reorient committed: yaw {:.1f} -> {:.1f} deg over {:.2f}s, drift={:.3f}m",
                   start_yaw * 57.3, (start_yaw + yaw_delta) * 57.3, duration, pos_dist);
    if (cfg_.visualization_en) {
        ros_ptr_->vizExpTraj(position, traj_ns);
        ros_ptr_->vizYawTraj(position, yaw);
    }
    return true;
}

} // namespace general_planner
