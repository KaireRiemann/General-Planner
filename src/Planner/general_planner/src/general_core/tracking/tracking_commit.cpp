#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <fmt/format.h>

using namespace general_utils;
namespace general_planner {

bool GeneralPlanner::commitTrackingTrajectory(const Trajectory &position,
        const Trajectory &yaw, const traj_opt::DynamicTargetStates &prediction,
        const std::string &traj_ns, double head_wt, bool /*relax_fov*/,
        bool /*allow_old_prefix*/, const vec_Vec3f &guide) {
    const auto reject = [&](const std::string &stage, const std::string &detail) {
        setTrackingCommitRejectInfo(stage, detail);
        return false;
    };
    if (trackingPerchingPerchingActive())
        return reject("perching owns committed trajectory", "tracking commit blocked");
    if (position.empty() || yaw.empty()) return reject("empty candidate", "position or yaw missing");
    if (!std::isfinite(head_wt))
        return reject("HEAD_TIME", "non-finite candidate head");
    // Elastic stamps at now+30ms and still publishes if optimization overruns.
    Trajectory committed_pos = position, committed_yaw = yaw;
    committed_pos.start_WT = committed_yaw.start_WT = head_wt;
    const auto aligned = trackingPredictionAtTime(prediction, head_wt);
    if (aligned.size() < 2) return reject("NO_PREDICTION", "prediction expired during validation");

    std::string safety_reason, safety_detail;
    const double check_dur = std::min(1.0, committed_pos.getTotalDuration());
    if (!trackingTrajectorySafeForHorizonDetailed(committed_pos, 0.0,
            check_dur, 0.01,
            &safety_reason, &safety_detail))
        return reject(safety_reason, safety_detail);

    ExpTraj command;
    command.setGoalConnectedFlag(true);
    command.setWholeTrajKnownFreeFlag(true);
    command.setTrajectory(head_wt, committed_pos, committed_yaw);
    cmd_traj_info_.setTrajectory(command);
    // The committed yaw is a scalar setpoint (constant polynomial); the
    // command sampler slews toward it like Elastic's traj_server.
    setTrackingYawServo(true);
    last_exp_traj_info_ = command;
    robot_on_backup_traj_.store(false);
    gi_.new_goal = false;
    latest_replan.setExpTraj(committed_pos);
    latest_replan.setExpYawTraj(committed_yaw);
    latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
    tracking_runtime_manager_->onCommitted(head_wt, position.getPos(0.0), guide);
    last_tracking_commit_wt_ = ros_ptr_->getSimTime();
    clearTrackingCommitRejectInfo();
    if (cfg_.visualization_en) {
        ros_ptr_->vizExpTraj(committed_pos, traj_ns);
        ros_ptr_->vizYawTraj(committed_pos, committed_yaw);
        ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
    }
    if (cfg_.print_log) ros_ptr_->info(
        " -- [Tracking] TRACKING_CANDIDATE_COMMITTED candidate_duration={:.3f}, candidate_head_wt={:.3f}",
        position.getTotalDuration(), head_wt);
    return true;
}
} // namespace general_planner
