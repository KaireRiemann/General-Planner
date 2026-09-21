#include <general_core/general_planner.h>

namespace general_planner {
bool GeneralPlanner::keepOldTrackingTrajectoryIfActive(
        const traj_opt::DynamicTargetStates &prediction, const std::string &reason) {
    if (!tracking_runtime_manager_ || !tracking_runtime_manager_->hasCommittedTracking() ||
        cmd_traj_info_.empty()) return false;
    cmd_traj_info_.lock();
    const auto position=cmd_traj_info_.posTraj(), yaw=cmd_traj_info_.yawTraj();
    const double start=cmd_traj_info_.getStartWallTime();
    cmd_traj_info_.unlock();
    const double local=std::max(0.0,ros_ptr_->getSimTime()-start);
    const auto activity=tracking_runtime_manager_->evaluateActivity(position,local,prediction,
        cfg_.tracking_keep_old_horizon,cfg_.tracking_keep_old_safety_dt);
    if (!activity.valid || !activity.safe || !activity.active) return false;
    tracking_runtime_manager_->onKeepOld();
    setTrackingDiagnostic("keep_old",reason,last_tracking_diag_guide_path_size_,
        last_tracking_diag_sfc_size_,prediction.size(),position.getTotalDuration());
    latest_replan.setExpTraj(position); latest_replan.setExpYawTraj(yaw);
    latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
    return true;
}
bool GeneralPlanner::trackingTrajectorySafeForHorizonDetailed(
        const Trajectory &trajectory,double start,double horizon,double dt,
        std::string *reason,std::string *detail) const {
    std::string failure;
    const bool safe=tracking_runtime_manager_ &&
        tracking_runtime_manager_->trajectorySafe(trajectory,start,horizon,dt,&failure);
    if(reason) *reason=failure;
    if(detail) *detail=failure;
    return safe;
}
} // namespace general_planner
