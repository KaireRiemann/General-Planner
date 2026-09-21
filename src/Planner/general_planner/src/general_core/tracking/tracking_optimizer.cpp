#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <general_core/tracking/tracking_map_query.hpp>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;
namespace general_planner {
RET_CODE GeneralPlanner::optimizeTrackingTask(const traj_opt::DynamicTargetStates &input,
                                             const bool &from_rest) {
    clearTrackingCommitRejectInfo();
    if (!input.empty()) {
        // Elastic lifts the target by the tracking height before predicting,
        // so every occupancy query runs at flight altitude. GP keeps the
        // prediction in raw target coordinates; the query-time exclusion
        // cylinder must therefore be centered at the lifted ring height to
        // mask the target's own cloud where the search actually happens.
        setTrackingOccupancyExclusion(input.front().position +
                                      Vec3f(0.0, 0.0, cfg_.tracking_height_offset),
                                      cfg_.tracking_target_exclusion_xy_radius,
                                      cfg_.tracking_target_exclusion_z_radius);
    }
    if (from_rest) invalidateTrackingCommand("tracking_start_from_measured_state");
    traj_opt::TrackingSolveReport solve_report;
    const auto solveDetail = [&]() {
        return fmt::format("solver_status={};solver_iterations={};solver_ms={:.3f};solver_cost={:.3f};solver_gradient_inf={:.3f};solver_budget_exhausted={};solver_feasible_iterate={};solver_warm_start={}",
            solve_report.status, solve_report.iterations, solve_report.elapsed_ms,
            solve_report.cost, solve_report.gradient_inf, solve_report.budget_exhausted,
            solve_report.used_feasible_iterate, solve_report.warm_start_used);
    };
    const double now = ros_ptr_->getSimTime();
    const auto prediction = trackingPredictionAtTime(input, now);
    if (tracking_runtime_manager_) tracking_runtime_manager_->observeExecution(now, robot_state_.p);
    if (tracking_runtime_manager_) tracking_runtime_manager_->beginAttempt();
    auto fail = [&](const std::string phase, const std::string reason) -> RET_CODE {
        setTrackingDiagnostic(phase, reason, 0, 0, prediction.size(), 0.0);
        setTrackingCommitRejectInfo(phase, reason + ";" + solveDetail());
        // Elastic: a failed replan keeps the last 1 s-safe trajectory. Hover
        // only if that command is gone or colliding. A from-rest failure
        // leaves the vehicle where it already is.
        if (!from_rest && !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory last = cmd_traj_info_.posTraj();
            const Trajectory last_yaw = cmd_traj_info_.yawTraj();
            const double start = cmd_traj_info_.getStartWallTime();
            cmd_traj_info_.unlock();
            const double local = ros_ptr_->getSimTime() - start;
            if (local >= 0.0 && local < last.getTotalDuration() &&
                trackingTrajectorySafeForHorizon(last, local, 1.0, 0.01)) {
                if (tracking_runtime_manager_) tracking_runtime_manager_->onKeepOld();
                setTrackingDiagnostic("keep_old", reason, 0, 0, prediction.size(),
                                      last.getTotalDuration() - local);
                latest_replan.setExpTraj(last);
                latest_replan.setExpYawTraj(last_yaw);
                latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                return NO_NEED;
            }
        }
        if (!from_rest && commitTrackingHoldTrajectory("tracking fallback: " + reason, 1.0, false))
            return NO_NEED;
        return FAILED;
    };
    if (prediction.size() < 2) return fail("input", "NO_PREDICTION: expired or invalid target prediction");
    const double head_time = now + std::max(0.03, cfg_.tracking_replan_forward_dt);
    const auto trusted_prediction = trackingPredictionAtTime(prediction, head_time);
    if (trusted_prediction.size() < 2 || trusted_prediction.back().t < 0.15)
        return fail("input", "NO_PREDICTION: insufficient trusted execution interval");
    StatePVAJ head = makeTaskHeadState(false, head_time);
    TrackingFrontend::Config frontend_cfg;
    frontend_cfg.tracking_distance = cfg_.tracking_distance;
    frontend_cfg.distance_tolerance = cfg_.tracking_distance_tolerance;
    frontend_cfg.height_offset = cfg_.tracking_height_offset;
    frontend_cfg.height_tolerance = cfg_.tracking_height_tolerance;
    frontend_cfg.max_speed = cfg_.tracking_traj_cfg.max_vel;
    frontend_cfg.unknown_as_occupied = cfg_.tracking_unknown_as_occupied;
    frontend_cfg.use_visible_region = cfg_.tracking_use_visible_region;
    frontend_cfg.visibility_angle_clearance = cfg_.tracking_visibility_angle_clearance;
    frontend_cfg.nominal_horizon = cfg_.tracking_nominal_horizon;
    frontend_cfg.sample_dt = cfg_.tracking_sample_dt;
    frontend_cfg.search_budget_seconds = cfg_.tracking_frontend_budget;
    TrackingFrontend frontend(frontend_cfg, tracking_map_manager_);
    traj_opt::TrackingProblem problem;
    TimeConsuming frontend_timer("tracking_frontend", false);
    if (!frontend.buildProblem(head, trusted_prediction, problem)) {
        time_consuming_[EPX_TRAJ_FRONTEND] = frontend_timer.stop();
        return fail("frontend", "no connected observable region within search budget");
    }
    std::string reason;
    if (!buildTrackingGuideCorridor(problem, &reason)) {
        time_consuming_[EPX_TRAJ_FRONTEND] = frontend_timer.stop();
        return fail("corridor", reason);
    }
    time_consuming_[EPX_TRAJ_FRONTEND] = frontend_timer.stop();
    problem.max_yaw_rate = cfg_.tracking_yaw_rate_limit;
    problem.max_yaw_acceleration = cfg_.tracking_yaw_acceleration_limit;
    problem.head_yaw << robot_state_.yaw, 0.0;
    problem.weight_tracking = cfg_.tracking_weight_tracking;
    problem.weight_visible_region = cfg_.tracking_weight_visible_region;
    problem.solve_budget_seconds = cfg_.tracking_solver_budget;
    problem.max_iterations = cfg_.tracking_solver_iterations;
    problem.min_total_duration = cfg_.tracking_nominal_horizon;
    problem.corridor_clearance = 0.2;
    problem.tail_pvaj.col(1) = prediction.front().velocity;
    if (problem.tail_pvaj.col(1).norm() > cfg_.tracking_traj_cfg.max_vel)
        problem.tail_pvaj.col(1) *= cfg_.tracking_traj_cfg.max_vel /
                                    problem.tail_pvaj.col(1).norm();
    problem.solve_report = &solve_report;
    problem.candidate_feasible = [this](const Trajectory &position, const Trajectory &) {
        return trackingTrajectorySafeForHorizon(position, 0.0, 1.0, 0.01);
    };
    latest_replan.setGuidePath(problem.guide_path);
    latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj, problem.tail_pvaj, problem.sfcs);
    Trajectory position;
    TimeConsuming optimizer_timer("tracking_opt", false);
    // Elastic never shapes a yaw polynomial in the optimizer. It publishes a
    // scalar target-facing heading and lets traj_server slew toward it, so no
    // yaw state enters the planning problem here.
    const bool optimized = traj_manager_->trackingJerk()->optimize(problem, position, nullptr, &reason);
    time_consuming_[EXP_TRAJ_OPT] = optimizer_timer.stop();
    if (!optimized || position.empty()) return fail("optimizer", reason);
    position.start_WT = head_time;
    // Elastic-Tracker planning_nodelet: yaw = atan2 of the vector from the
    // trajectory head toward the target at the command instant.
    const Vec3f yaw_ref = trusted_prediction.front().position - head.col(0);
    const double desired_yaw = yaw_ref.head<2>().squaredNorm() > 1.0e-6
        ? std::atan2(static_cast<double>(yaw_ref.y()), static_cast<double>(yaw_ref.x()))
        : (std::isfinite(robot_state_.yaw) ? static_cast<double>(robot_state_.yaw) : 0.0);
    Trajectory yaw;
    Eigen::Matrix<double, 3, 6> yaw_coeff = Eigen::Matrix<double, 3, 6>::Zero();
    yaw_coeff(0, 5) = desired_yaw;
    yaw.emplace_back(position.getTotalDuration(), yaw_coeff);
    yaw.start_WT = head_time;
    if (!commitTrackingTrajectory(position, yaw, prediction,
            "elastic_tracking",
            head_time, false, false, problem.guide_path))
        return fail(last_tracking_commit_reject_reason_, last_tracking_commit_reject_detail_);
    setTrackingDiagnostic("success", "elastic_candidate_committed;" + solveDetail(), problem, position.getTotalDuration());
    if (cfg_.visualization_en) {
        ros_ptr_->vizFrontendPath(problem.guide_path);
        ros_ptr_->vizExpSfc(problem.sfcs);
        ros_ptr_->vizTrackingFov(position, yaw, cfg_.tracking_fov_horizontal_deg,
                               cfg_.tracking_fov_vertical_deg, cfg_.tracking_fov_range);
    }
    return SUCCESS;
}

} // namespace general_planner
