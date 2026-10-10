#include <general_core/general_planner.h>

namespace general_planner {

bool GeneralPlanner::checkVerticalSegment(const Vec3f &start, const Vec3f &end,
                                         bool unknown_as_occupied,
                                         std::string &reason) const {
    const auto reject = [&](const char *why) { reason = why; return false; };
    if (!map_manager_ || !map_manager_->ready()) return reject("vertical_map_not_ready");
    if (!start.allFinite() || !end.allFinite()) return reject("vertical_invalid_position");
    if (!map_manager_->insideLocalMap(start) || !map_manager_->insideLocalMap(end))
        return reject("vertical_out_of_local_map");
    // Include both endpoints explicitly. Query raw unknown cells even when
    // unknown inflation is disabled, and inflated occupancy for body clearance.
    const double step = 0.5 * std::min(map_manager_->getResolution(),
                                      map_manager_->getMapConfig().inflation_resolution);
    if (!std::isfinite(step) || step <= 0.0) return reject("vertical_invalid_map_resolution");
    const double count = std::ceil((end - start).norm() / step);
    if (!std::isfinite(count) || count > 100000) return reject("vertical_segment_too_long");
    const int samples = std::max(1, static_cast<int>(count));
    for (int i = 0; i <= samples; ++i) {
        const Vec3f p = start + (end - start) * (static_cast<double>(i) / samples);
        const auto raw = map_manager_->getGridType(p);
        const auto inflated = map_manager_->getInfGridType(p);
        if (!map_manager_->insideLocalMap(p) || raw == rog_map::OUT_OF_MAP ||
            inflated == rog_map::OUT_OF_MAP) return reject("vertical_out_of_local_map");
        if (raw == rog_map::OCCUPIED || inflated == rog_map::OCCUPIED)
            return reject("vertical_path_occupied");
        if (unknown_as_occupied && raw != rog_map::KNOWN_FREE)
            return reject("vertical_path_unknown");
    }
    if (!map_manager_->isLineFree(start, end, true, unknown_as_occupied) ||
        (unknown_as_occupied && !map_manager_->isLineFree(start, end, false, true)))
        return reject("vertical_line_collision");
    if (dynamic_obstacle_layer_ && dynamic_obstacle_layer_->enabled() &&
        dynamic_obstacle_layer_->lineOccupied(start, end, ros_ptr_->getSimTime()))
        return reject("vertical_dynamic_obstacle");
    reason = "ok";
    return true;
}

bool GeneralPlanner::prepareVerticalTrajectory(double target_z,
                                              const VerticalMotionConfig &limits,
                                              Trajectory &position, Trajectory &yaw,
                                              std::string &reason) {
    rog_map::RobotState state;
    getRobotStateSnapshot(state);
    const double now = ros_ptr_->getSimTime();
    if (!state.rcv || !state.p.allFinite() || !state.v.allFinite() ||
        !std::isfinite(state.yaw) || !std::isfinite(state.rcv_time) ||
        !std::isfinite(now) || now < state.rcv_time || now - state.rcv_time > 0.2) {
        reason = "vertical_odometry_unavailable";
        return false;
    }
    if (state.v.norm() > limits.start_speed_tolerance) {
        reason = "vertical_requires_hover";
        return false;
    }
    VerticalMotionConfig effective = limits;
    // Vertical acceleration changes collective thrust directly. Keep both
    // ends of g +/- a inside the configured thrust envelope as well as the
    // normal velocity/acceleration/jerk envelope.
    const auto &dynamics = cfg_.esdf_traj_cfg;
    if (dynamics.max_vel > 0) effective.max_vel = std::min(effective.max_vel, dynamics.max_vel);
    if (dynamics.max_acc > 0) effective.max_acc = std::min(effective.max_acc, dynamics.max_acc);
    if (dynamics.max_jerk > 0) effective.max_jerk = std::min(effective.max_jerk, dynamics.max_jerk);
    const double gravity = std::abs(dynamics.grav);
    effective.max_acc = std::min(effective.max_acc, gravity - std::max(0.0, dynamics.min_acc_thr));
    if (dynamics.max_acc_thr > 0)
        effective.max_acc = std::min(effective.max_acc, dynamics.max_acc_thr - gravity);
    if (!buildVerticalTrajectory(state.p, state.yaw, target_z, effective, position, yaw, reason))
        return false;
    Vec3f target = state.p;
    target.z() = target_z;
    if (!checkVerticalSegment(state.p, target, limits.unknown_as_occupied, reason)) return false;
    position.start_WT = yaw.start_WT = now;
    return true;
}

bool GeneralPlanner::commitVerticalTrajectory(double target_z,
                                             const VerticalMotionConfig &limits,
                                             std::string &reason) {
    Trajectory position, yaw;
    if (!prepareVerticalTrajectory(target_z, limits, position, yaw, reason)) return false;
    ExpTraj command;
    command.setGoalConnectedFlag(true);
    command.setWholeTrajKnownFreeFlag(limits.unknown_as_occupied);
    command.setTrajectory(position.start_WT, position, yaw);
    cmd_traj_info_.setTrajectory(command);
    last_exp_traj_info_ = command;
    setTrackingYawServo(false);
    robot_on_backup_traj_.store(false);
    gi_.new_goal = false;
    latest_replan.setExpTraj(position);
    latest_replan.setExpYawTraj(yaw);
    latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
    ros_ptr_->info(" -- [GeneralPlanner] Vertical committed: z {:.3f} -> {:.3f} over {:.2f}s",
                   position.getPos(0).z(), target_z, position.getTotalDuration());
    if (cfg_.visualization_en) {
        ros_ptr_->vizExpTraj(position, "vertical");
        ros_ptr_->vizYawTraj(position, yaw);
    }
    return true;
}

} // namespace general_planner
