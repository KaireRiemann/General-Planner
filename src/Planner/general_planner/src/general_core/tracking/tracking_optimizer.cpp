#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <general_utils/scope_timer.hpp>
#include <chrono>

using namespace general_utils;
namespace general_planner {
RET_CODE GeneralPlanner::optimizeTrackingTask(const traj_opt::DynamicTargetStates &input,
                                             const bool &from_rest) {
    clearTrackingCommitRejectInfo();
    const double now = ros_ptr_->getSimTime();
    const auto prediction = trackingPredictionAtTime(input, now);
    if (tracking_runtime_manager_) tracking_runtime_manager_->observeExecution(now, robot_state_.p);
    auto fail = [&](const std::string &phase, const std::string &reason) -> RET_CODE {
        setTrackingDiagnostic(phase, reason, 0, 0, prediction.size(), 0.0);
        setTrackingCommitRejectInfo(phase, reason);
        const auto current = trackingPredictionAtTime(prediction, ros_ptr_->getSimTime());
        if (!current.empty() && keepOldTrackingTrajectoryIfActive(current, reason)) return NO_NEED;
        if (commitTrackingHoldTrajectory("tracking fallback: " + reason, 0.0, !from_rest)) return NO_NEED;
        if (tracking_runtime_manager_) tracking_runtime_manager_->onRejected();
        return FAILED;
    };
    if (prediction.size() < 2) return fail("input", "NO_PREDICTION: expired or invalid target prediction");
    const double head_time = now + std::max(0.03, cfg_.tracking_replan_forward_dt);
    const auto trusted_prediction = trackingPredictionAtTime(prediction, head_time);
    if (trusted_prediction.size() < 2 || trusted_prediction.back().t < 0.15)
        return fail("input", "NO_PREDICTION: insufficient trusted execution interval");
    const StatePVAJ head = makeTaskHeadState(from_rest, head_time);
    TrackingFrontend::Config frontend_cfg;
    frontend_cfg.tracking_distance = cfg_.tracking_distance;
    frontend_cfg.distance_tolerance = cfg_.tracking_distance_tolerance;
    frontend_cfg.distance_lower_tolerance = cfg_.tracking_distance_lower_tolerance;
    frontend_cfg.distance_upper_tolerance = cfg_.tracking_distance_upper_tolerance;
    frontend_cfg.height_offset = cfg_.tracking_height_offset;
    frontend_cfg.height_tolerance = cfg_.tracking_height_tolerance;
    frontend_cfg.target_half_width = cfg_.tracking_target_half_width;
    frontend_cfg.target_half_height = cfg_.tracking_target_half_height;
    frontend_cfg.safe_distance = trackingHardSafeDistance(cfg_);
    frontend_cfg.searching_horizon = cfg_.tracking_planning_horizon;
    frontend_cfg.max_speed = cfg_.tracking_traj_cfg.max_vel;
    frontend_cfg.max_acc = cfg_.tracking_traj_cfg.max_acc;
    frontend_cfg.max_jerk = cfg_.tracking_traj_cfg.max_jerk;
    frontend_cfg.unknown_as_occupied = cfg_.tracking_unknown_as_occupied;
    frontend_cfg.use_astar = cfg_.tracking_frontend_astar;
    frontend_cfg.use_visible_region = cfg_.tracking_use_visible_region;
    frontend_cfg.visibility_angle_clearance = cfg_.tracking_visibility_angle_clearance;
    frontend_cfg.nominal_horizon = cfg_.tracking_nominal_horizon;
    frontend_cfg.max_extrapolation = cfg_.tracking_max_extrapolation;
    frontend_cfg.search_budget_seconds = cfg_.tracking_frontend_budget;
    TrackingFrontend frontend(frontend_cfg, map_manager_);
    Vec3f reference_viewpoint;
    traj_opt::DynamicTargetState reference_target;
    const bool reference = findTrackingViewpointReference(trusted_prediction, reference_viewpoint, reference_target);
    traj_opt::TrackingProblem problem;
    TimeConsuming frontend_timer("tracking_frontend", false);
    if (!frontend.buildProblem(head, trusted_prediction, problem,
            reference ? &reference_viewpoint : nullptr, reference ? &reference_target : nullptr)) {
        time_consuming_[EPX_TRAJ_FRONTEND] = frontend_timer.stop();
        return fail("frontend", "no connected observable region within search budget");
    }
    std::string reason;
    if (!buildTrackingGuideCorridor(problem, &reason)) {
        time_consuming_[EPX_TRAJ_FRONTEND] = frontend_timer.stop();
        return fail("corridor", reason);
    }
    time_consuming_[EPX_TRAJ_FRONTEND] = frontend_timer.stop();
    problem.safe_distance = cfg_.tracking_safe_distance;
    problem.max_yaw_rate = cfg_.tracking_yaw_rate_limit;
    problem.max_yaw_acceleration = cfg_.tracking_yaw_acceleration_limit;
    problem.head_yaw << robot_state_.yaw, 0.0;
    if (!from_rest && !cmd_traj_info_.empty()) {
        cmd_traj_info_.lock();
        const Trajectory yaw = cmd_traj_info_.yawTraj();
        const double start = cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();
        StatePVAJ state;
        const double t = head_time - start;
        if (!yaw.empty() && t >= 0.0 && t <= yaw.getTotalDuration() && yaw.getState(t, state)) {
            problem.head_yaw = state.row(0).head<2>();
            problem.head_yaw_acceleration = state(0,2);
        }
    }
    problem.weight_od_near = cfg_.tracking_weight_od_near;
    problem.weight_od_far = cfg_.tracking_weight_od_far;
    problem.weight_od_vertical = cfg_.tracking_weight_od_vertical;
    problem.weight_oa = cfg_.tracking_weight_oa;
    problem.weight_oe = cfg_.tracking_weight_oe;
    problem.weight_visibility = cfg_.tracking_weight_oe;
    problem.weight_relative_velocity = cfg_.tracking_weight_relative_velocity;
    problem.weight_tangent_velocity = cfg_.tracking_weight_tangent_velocity;
    problem.weight_visible_region = cfg_.tracking_weight_visible_region;
    problem.weight_fov = cfg_.tracking_weight_fov;
    problem.adaptive_occlusion_enable = cfg_.tracking_adaptive_occlusion_enable;
    problem.adaptive_occlusion_activation_distance = cfg_.tracking_adaptive_occlusion_activation_distance;
    problem.adaptive_occlusion_max_weight_scale = cfg_.tracking_adaptive_occlusion_max_weight_scale;
    problem.adaptive_occlusion_od_far_weight_scale = cfg_.tracking_adaptive_occlusion_od_far_weight_scale;
    problem.adaptive_occlusion_distance_upper_scale = cfg_.tracking_adaptive_occlusion_distance_upper_scale;
    problem.adaptive_occlusion_min_horizontal_upper = cfg_.tracking_adaptive_occlusion_min_horizontal_upper;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    problem.fov_horizontal = std::max(1.0, cfg_.tracking_fov_horizontal_deg) * kDegToRad;
    problem.fov_vertical = std::max(1.0, cfg_.tracking_fov_vertical_deg) * kDegToRad;
    problem.fov_front_margin = cfg_.tracking_fov_front_margin;
    for (int row = 0; row < 3; ++row) {
        problem.camera_translation(row) = cfg_.tracking_camera_p[row];
        for (int col = 0; col < 3; ++col)
            problem.camera_rotation(row,col) = cfg_.tracking_camera_R[row*3+col];
    }
    problem.target_half_height = cfg_.tracking_target_half_height;
    problem.target_half_width = cfg_.tracking_target_half_width;
    problem.joint_sample_dt = cfg_.tracking_joint_sample_dt;
    problem.dense_joint_sample_enable = cfg_.tracking_dense_joint_sample_enable;

    problem.solve_budget_seconds = cfg_.tracking_solver_budget;
    problem.max_iterations = cfg_.tracking_solver_iterations;
    problem.should_stop = [this, head_time]() { return ros_ptr_->getSimTime() + 0.01 >= head_time; };
    latest_replan.setGuidePath(problem.guide_path);
    latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj, problem.tail_pvaj, problem.sfcs);
    Trajectory position, yaw;
    TimeConsuming optimizer_timer("tracking_opt", false);
    const bool optimized = cfg_.tracking_use_snap
        ? traj_manager_->trackingSnap()->optimize(problem, position, &yaw, &reason)
        : traj_manager_->trackingJerk()->optimize(problem, position, &yaw, &reason);
    time_consuming_[EXP_TRAJ_OPT] = optimizer_timer.stop();
    if (!optimized || position.empty()) return fail("optimizer", reason);
    const auto dynamics = checkTrackingDynamics(position, yaw, cfg_);
    if (!dynamics.valid) return fail("dynamics", dynamics.reason);
    if (position.getTotalDuration() < cfg_.tracking_min_commit_duration)
        return fail("duration", "candidate shorter than minimum commit duration");
    position.start_WT = head_time; yaw.start_WT = head_time;
    // Commit owns collision, FOV, splice and old/candidate arbitration. No second
    // yaw solve or alternate problem is generated by the optimization layer.
    const bool relax = problem.reacquire_mode && cfg_.tracking_reacquire_fov_relax_enable;
    if (!commitTrackingTrajectory(position, yaw, prediction,
            cfg_.tracking_use_snap ? "tracking_snap" : "tracking_jerk",
            head_time, relax, !from_rest, problem.guide_path))
        return fail("commit", last_tracking_commit_reject_reason_);
    if (last_tracking_diag_phase_ == "keep_old") return NO_NEED;
    rememberTrackingViewpointReference(problem);
    setTrackingDiagnostic("success", "elastic_candidate_committed", problem, position.getTotalDuration());
    if (cfg_.visualization_en) {
        ros_ptr_->vizFrontendPath(problem.guide_path);
        ros_ptr_->vizExpSfc(problem.sfcs);
        ros_ptr_->vizTrackingFov(position, yaw, cfg_.tracking_fov_horizontal_deg,
                               cfg_.tracking_fov_vertical_deg, trackingAdaptiveFovRange(cfg_));
    }
    return SUCCESS;
}

} // namespace general_planner
