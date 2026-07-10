#include <fsm/fsm.h>

#include <general_core/perching/perching_planner.hpp>
#include <general_core/state2state/state2state_planner.hpp>
#include <general_core/takeoff/takeoff_planner.hpp>
#include <general_core/tracking/tracking_planner.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using namespace general_utils;

namespace fsm {

class Fsm::State2StateTaskExecutor final : public Fsm::TaskExecutor {
public:
    TaskMode mode() const override {
        return TaskMode::STATE_TO_STATE;
    }

    const char *name() const override {
        return "state2state";
    }

    bool goalLike() const override {
        return true;
    }

    bool ready(Fsm &fsm) override {
        return fsm.gi_.new_goal;
    }

    bool replanAllowed(const Fsm &fsm) const override {
        return fsm.machine_state_ == Fsm::FOLLOW_TRAJ;
    }

    PlanResult plan(Fsm &fsm, const PlanRequest &request) override {
        const auto result = planner(fsm).plan(
                general_planner::architecture::StateToStateRequest{
                        request,
                        fsm.gi_.goal_p,
                        fsm.gi_.goal_yaw,
                        fsm.gi_.new_goal});
        return result.plan_result;
    }

    PlanResult replan(Fsm &fsm, const PlanRequest &request) override {
        const auto result = planner(fsm).replan(
                general_planner::architecture::StateToStateRequest{
                        request,
                        fsm.gi_.goal_p,
                        fsm.gi_.goal_yaw,
                        fsm.gi_.new_goal});
        if (result.goal_projected) {
            fsm.gi_.goal_p = result.resolved_goal;
        }
        return result.plan_result;
    }

    bool shouldGenerateAfterTrajFinish(Fsm &fsm) override {
        return !fsm.closeToGoal(0.1);
    }

private:
    general_planner::architecture::StateToStatePlanner planner(Fsm &fsm) const {
        return general_planner::architecture::StateToStatePlanner(
                fsm.makePlannerContext());
    }
};

class Fsm::TrackingTaskExecutor : public Fsm::TaskExecutor {
public:
    TaskMode mode() const override {
        return TaskMode::TRACKING;
    }

    const char *name() const override {
        return "tracking";
    }

    bool trackingLike() const override {
        return true;
    }

    bool ready(Fsm &fsm) override {
        if (fsm.trackingPerchingPerchingActive()) {
            return false;
        }
        return fsm.trackingTaskReady();
    }

    bool replanAllowed(const Fsm &fsm) const override {
        return fsm.trackingExecutionState();
    }

    PlanResult plan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::DynamicTargetStates prediction;
        if (!fsm.getTrackingTargetPrediction(prediction)) {
            fsm.recordDiagnosticEvent(
                    "WARN",
                    "tracking_prediction_unavailable",
                    fmt::format("tracking_target_rcv_time={:.3f};timeout={:.3f}",
                                fsm.tracking_target_rcv_time_,
                                fsm.cfg_.task_timeout),
                    FAILED);
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "tracking_prediction_unavailable");
        }
        fillTrackingContext(fsm, prediction, context);
        return planner(fsm).plan(
                general_planner::architecture::TrackingPlanRequest{
                        request,
                        prediction,
                        fsm.task_new_},
                context,
                "track_target");
    }

    PlanResult replan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::DynamicTargetStates prediction;
        if (!fsm.getTrackingTargetPrediction(prediction)) {
            fsm.recordDiagnosticEvent(
                    "WARN",
                    "tracking_prediction_unavailable",
                    fmt::format("tracking_target_rcv_time={:.3f};timeout={:.3f}",
                                fsm.tracking_target_rcv_time_,
                                fsm.cfg_.task_timeout),
                    FAILED);
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "tracking_prediction_unavailable");
        }
        fillTrackingContext(fsm, prediction, context);
        if (fsm.shouldSkipStaticTrackingReplan(prediction)) {
            fsm.recordDiagnosticEvent(
                    "INFO",
                    "tracking_static_hold_skip_replan",
                    fmt::format("prediction.size()={};prediction_static=1;remaining_traj_s={:.3f}",
                                prediction.size(),
                                fsm.planner_ptr_->getCommittedTrajectoryRemainingDuration()),
                    NO_NEED);
            if (fsm.machine_state_ != Fsm::STATIC_TRACKING) {
                fsm.ChangeState("StaticTrackingContinue", Fsm::STATIC_TRACKING);
            }
            context.handled = true;
            return makeResult(fsm, request, NO_NEED, context, "static_tracking_hold");
        }
        return planner(fsm).replan(
                general_planner::architecture::TrackingPlanRequest{
                        request,
                        prediction,
                        fsm.task_new_},
                context,
                "track_target");
    }

    bool shouldGenerateAfterTrajFinish(Fsm &fsm) override {
        if (fsm.trackingPerchingPerchingActive()) {
            return false;
        }
        return fsm.activeTaskReady();
    }

protected:
    void fillTrackingContext(Fsm &fsm,
                             const traj_opt::DynamicTargetStates &prediction,
                             TaskPlanContext &context) const {
        context.tracking_context = true;
        context.tracking_input_prediction_size = prediction.size();
        context.tracking_prediction_static = fsm.trackingPredictionStatic(prediction);
    }

    general_planner::architecture::TrackingPlanner planner(Fsm &fsm) const {
        return general_planner::architecture::TrackingPlanner(
                fsm.makePlannerContext());
    }
};

class Fsm::TrackingPerchingMissionAdapter final : public Fsm::TrackingTaskExecutor {
public:
    TaskMode mode() const override {
        return TaskMode::TRACKING_PERCHING;
    }

    const char *name() const override {
        return "tracking_perching_mission";
    }

    bool ready(Fsm &fsm) override {
        return !fsm.perching_contact_reached_ &&
               fsm.planner_ptr_ &&
               !fsm.planner_ptr_->trackingPerchingContactReached() &&
               fsm.trackingTaskReady();
    }

    PlanResult plan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::DynamicTargetStates prediction;
        if (!fsm.getTrackingTargetPrediction(prediction)) {
            fsm.recordDiagnosticEvent(
                    "WARN",
                    "tracking_prediction_unavailable",
                    fmt::format("tracking_target_rcv_time={:.3f};timeout={:.3f}",
                                fsm.tracking_target_rcv_time_,
                                fsm.cfg_.task_timeout),
                    FAILED);
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "tracking_prediction_unavailable");
        }
        fillTrackingContext(fsm, prediction, context);
        context.mission_node = "track_target";
        const general_planner::architecture::TrackingPlanRequest tracking_request{
                request,
                prediction,
                fsm.task_new_};
        PlanResult result = planner(fsm).plan(tracking_request, context, "track_target");
        int ret = result.ret_code;
        traj_opt::PerchingSurfaceState surface;
        std::string readiness_detail;
        if ((ret == SUCCESS || ret == NO_NEED) &&
            fsm.getPerchingSurface(surface) &&
            handoverReady(prediction, surface, readiness_detail) &&
            !fsm.planner_ptr_->trackingPerchingPerchingActive()) {
            context.mission_node = "handover_to_perching";
            context.decision_detail = readiness_detail;
            return planner(fsm).tryCommitPerchingFromTracking(
                    tracking_request,
                    surface,
                    ret,
                    context,
                    context.mission_node);
        }
        return result;
    }

    PlanResult replan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::DynamicTargetStates prediction;
        if (!fsm.getTrackingTargetPrediction(prediction)) {
            fsm.recordDiagnosticEvent(
                    "WARN",
                    "tracking_prediction_unavailable",
                    fmt::format("tracking_target_rcv_time={:.3f};timeout={:.3f}",
                                fsm.tracking_target_rcv_time_,
                                fsm.cfg_.task_timeout),
                    FAILED);
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "tracking_prediction_unavailable");
        }
        fillTrackingContext(fsm, prediction, context);

        traj_opt::PerchingSurfaceState surface;
        const bool has_surface = fsm.getPerchingSurface(surface);
        if (fsm.perching_contact_reached_ ||
            (fsm.planner_ptr_ && fsm.planner_ptr_->trackingPerchingContactReached())) {
            context.handled = true;
            context.mission_node = "contact_reached";
            return makeResult(fsm, request, FINISH, context, "contact_reached");
        }
        if (fsm.trackingPerchingPerchingActive()) {
            context.mission_node = "perching";
        } else {
            context.mission_node = "track_target";
        }
        if (!has_surface && fsm.shouldSkipStaticTrackingReplan(prediction)) {
            fsm.recordDiagnosticEvent(
                    "INFO",
                    "tracking_static_hold_skip_replan",
                    fmt::format("prediction.size()={};prediction_static=1;remaining_traj_s={:.3f}",
                                prediction.size(),
                                fsm.planner_ptr_->getCommittedTrajectoryRemainingDuration()),
                    NO_NEED);
            if (fsm.machine_state_ != Fsm::STATIC_TRACKING) {
                fsm.ChangeState("StaticTrackingContinue", Fsm::STATIC_TRACKING);
            }
            context.handled = true;
            return makeResult(fsm, request, NO_NEED, context, "static_tracking_hold");
        }
        if (has_surface) {
            std::string readiness_detail;
            if (handoverReady(prediction, surface, readiness_detail)) {
                context.mission_node = fsm.trackingPerchingPerchingActive()
                                           ? "perching"
                                           : "handover_to_perching";
                context.decision_detail = readiness_detail;
                return planner(fsm).replanWithPerchingSurface(
                        general_planner::architecture::TrackingPlanRequest{
                                request,
                                prediction,
                                fsm.task_new_},
                        surface,
                        context,
                        context.mission_node);
            }
        }
        return planner(fsm).replan(
                general_planner::architecture::TrackingPlanRequest{
                        request,
                        prediction,
                        fsm.task_new_},
                context,
                context.mission_node);
    }

private:
    bool handoverReady(const traj_opt::DynamicTargetStates &prediction,
                       const traj_opt::PerchingSurfaceState &surface,
                       std::string &detail) const {
        if (prediction.empty()) {
            detail = "readiness=0;reason=empty_prediction";
            return false;
        }
        const auto &target = prediction.front();
        const double prediction_horizon =
                std::max(0.0, prediction.back().t - prediction.front().t);
        const double distance = (surface.position - target.position).norm();
        const double relative_speed = (surface.velocity - target.velocity).norm();
        const bool finite =
                std::isfinite(prediction_horizon) &&
                std::isfinite(distance) &&
                std::isfinite(relative_speed) &&
                surface.position.allFinite() &&
                target.position.allFinite();
        const bool ready = finite && prediction_horizon >= 0.0;
        detail = fmt::format("readiness={};surface=1;distance={:.3f};relative_speed={:.3f};prediction_horizon={:.3f}",
                             static_cast<int>(ready),
                             distance,
                             relative_speed,
                             prediction_horizon);
        return ready;
    }
};

class Fsm::PerchingTaskExecutor final : public Fsm::TaskExecutor {
public:
    TaskMode mode() const override {
        return TaskMode::PERCHING;
    }

    const char *name() const override {
        return "perching";
    }

    bool ready(Fsm &fsm) override {
        return !fsm.perching_contact_reached_ && fsm.task_new_ && fsm.perchingTaskReady();
    }

    bool replanAllowed(const Fsm &fsm) const override {
        return fsm.machine_state_ == Fsm::FOLLOW_TRAJ;
    }

    PlanResult plan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::PerchingSurfaceState surface;
        if (!fsm.getPerchingSurface(surface)) {
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "perching_surface_unavailable");
        }
        return planner(fsm).plan(
                general_planner::architecture::PerchingPlanRequest{
                        request,
                        surface,
                        fsm.task_new_},
                context);
    }

    PlanResult replan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::PerchingSurfaceState surface;
        if (!fsm.getPerchingSurface(surface)) {
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "perching_surface_unavailable");
        }
        return planner(fsm).replan(
                general_planner::architecture::PerchingPlanRequest{
                        request,
                        surface,
                        fsm.task_new_},
                context);
    }

    bool shouldGenerateAfterTrajFinish(Fsm &) override {
        return false;
    }

private:
    general_planner::architecture::PerchingPlanner planner(Fsm &fsm) const {
        return general_planner::architecture::PerchingPlanner(
                fsm.makePlannerContext());
    }
};

class Fsm::DynamicTakeoffTaskExecutor final : public Fsm::TaskExecutor {
public:
    TaskMode mode() const override {
        return TaskMode::DYNAMIC_TAKEOFF;
    }

    const char *name() const override {
        return "dynamic_takeoff";
    }

    bool ready(Fsm &fsm) override {
        return fsm.task_new_ && fsm.dynamicTakeoffTaskReady();
    }

    bool replanAllowed(const Fsm &fsm) const override {
        return fsm.machine_state_ == Fsm::FOLLOW_TRAJ;
    }

    PlanResult plan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::PerchingSurfaceState surface;
        if (!fsm.getPerchingSurface(surface)) {
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "takeoff_surface_unavailable");
        }
        return planner(fsm).plan(
                general_planner::architecture::TakeoffPlanRequest{
                        request,
                        surface,
                        fsm.task_new_},
                context);
    }

    PlanResult replan(Fsm &fsm, const PlanRequest &request) override {
        TaskPlanContext context;
        traj_opt::PerchingSurfaceState surface;
        if (!fsm.getPerchingSurface(surface)) {
            context.missing_input = true;
            return makeResult(fsm, request, FAILED, context, "takeoff_surface_unavailable");
        }
        return planner(fsm).replan(
                general_planner::architecture::TakeoffPlanRequest{
                        request,
                        surface,
                        fsm.task_new_},
                context);
    }

    bool shouldGenerateAfterTrajFinish(Fsm &) override {
        return false;
    }

private:
    general_planner::architecture::TakeoffPlanner planner(Fsm &fsm) const {
        return general_planner::architecture::TakeoffPlanner(
                fsm.makePlannerContext());
    }
};

class Fsm::ExplorationTaskExecutor final : public Fsm::TaskExecutor {
public:
    TaskMode mode() const override {
        return TaskMode::EXPLORATION;
    }

    const char *name() const override {
        return "exploration_dedicated_node";
    }

    bool goalLike() const override {
        return true;
    }

    bool ready(Fsm &) override {
        return false;
    }

    bool replanAllowed(const Fsm &) const override {
        return false;
    }

    PlanResult plan(Fsm &fsm, const PlanRequest &request) override {
        return makeResult(fsm, request, FAILED, {},
                          "exploration is provided by general_planner/exploration_node");
    }

    PlanResult replan(Fsm &fsm, const PlanRequest &request) override {
        return makeResult(fsm, request, FAILED, {},
                          "exploration is provided by general_planner/exploration_node");
    }

    bool shouldGenerateAfterTrajFinish(Fsm &) override {
        return false;
    }
};

std::unique_ptr<Fsm::TaskExecutor> Fsm::makeTaskExecutor(const TaskMode mode) const {
    switch (mode) {
        case TaskMode::TRACKING:
            return std::make_unique<TrackingTaskExecutor>();
        case TaskMode::TRACKING_PERCHING:
            return std::make_unique<TrackingPerchingMissionAdapter>();
        case TaskMode::PERCHING:
            return std::make_unique<PerchingTaskExecutor>();
        case TaskMode::DYNAMIC_TAKEOFF:
            return std::make_unique<DynamicTakeoffTaskExecutor>();
        case TaskMode::EXPLORATION:
            return std::make_unique<ExplorationTaskExecutor>();
        case TaskMode::STATE_TO_STATE:
        default:
            return std::make_unique<State2StateTaskExecutor>();
    }
}

Fsm::TaskExecutor &Fsm::taskExecutor() {
    if (!task_executor_ || task_executor_mode_ != cfg_.task_mode) {
        task_executor_ = makeTaskExecutor(cfg_.task_mode);
        task_executor_mode_ = cfg_.task_mode;
        const auto identity = task_executor_->identity(*this);
        mission_orchestrator_.setActiveTask(identity, "task_executor_created");
        if (ros_ptr_) {
            const auto *backend = mission_orchestrator_.activeBackend();
            ros_ptr_->info(" -- [Fsm] Active task executor: plugin={}, mission={}, task={}, backend={}, resolved_backend={}",
                           identity.plugin_name,
                           general_planner::architecture::toString(identity.mission),
                           general_planner::architecture::toString(identity.task),
                           general_planner::architecture::toString(identity.backend),
                           backend ? backend->descriptor().name : "none");
        }
    }
    return *task_executor_;
}

void Fsm::resetTaskExecutor() {
    task_executor_.reset();
}

general_planner::architecture::PlannerContext Fsm::makePlannerContext() {
    return general_planner::architecture::PlannerContext{
            planner_ptr_,
            [this](const std::string &level,
                   const std::string &event,
                   const std::string &detail,
                   const int ret_code) {
                recordDiagnosticEvent(level, event, detail, ret_code);
            }};
}

} // namespace fsm
