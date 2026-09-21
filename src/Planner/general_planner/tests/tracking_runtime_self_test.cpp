#include <general_core/commit_governor.hpp>
#include <general_core/tracking/tracking_runtime_manager.hpp>

#include <iostream>
#include <stdexcept>

namespace {
using general_planner::TrackingRuntimeManager;
using general_utils::Vec3f;
using geometry_utils::Trajectory;


void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

Trajectory straight(double speed, double epoch = 100.0) {
    Eigen::Matrix<double, 3, 6> coefficients = Eigen::Matrix<double, 3, 6>::Zero();
    coefficients.col(4) = Vec3f(speed, 0.0, 0.0);
    coefficients.col(5) = Vec3f(-5.0, 0.0, 1.5);
    Trajectory trajectory;
    trajectory.emplace_back(2.0, coefficients);
    trajectory.start_WT = epoch;
    return trajectory;
}

traj_opt::DynamicTargetStates prediction(double epoch = 100.0) {
    traj_opt::DynamicTargetStates out;
    for (int i = 0; i <= 3; ++i) {
        traj_opt::DynamicTargetState sample;
        sample.t = 0.25 * i;
        sample.reference_time = epoch;
        sample.position = Vec3f(2.0 * sample.t, 0.0, 0.8);
        sample.velocity = Vec3f(2.0, 0.0, 0.0);
        out.push_back(sample);
    }
    return out;
}

void runtimeLifecycle() {
    general_planner::Config cfg;
    cfg.tracking_distance = 5.0;
    cfg.tracking_height_offset = 0.7;
    TrackingRuntimeManager manager(cfg, nullptr);
    const auto moving = straight(2.0);
    const auto target = prediction();

    using namespace general_planner::architecture;
    CommitGovernor governor;
    CommitGovernorInput input;
    input.result.request.identity.tracking_like = true;
    input.result.ret_code = general_utils::NO_NEED;
    for (bool from_rest : {false,true}) {
        const auto decide = [&] { return from_rest ? governor.decidePlanFromRest(input) : governor.decideReplan(input); };
        input.result.context.tracking_outcome = TrackingPlanOutcome::COMMITTED_RECOVERY;
        require(decide().action == CommitAction::HOLD && decide().publish_trajectory &&
                decide().next_phase == ExecutionPhase::RECOVERING, "new recovery must publish and enter recovery");
        input.result.context.tracking_outcome = TrackingPlanOutcome::KEPT_RECOVERY;
        require(decide().action == CommitAction::HOLD && !decide().publish_trajectory,
                "continuing recovery must not recommit its trajectory");
        input.result.context.tracking_outcome = TrackingPlanOutcome::KEPT_TRACKING;
        require(decide().action == CommitAction::KEEP_OLD_TRAJECTORY && !decide().publish_trajectory,
                "safe old tracking command must retain its epoch");
        input.result.context.tracking_outcome = TrackingPlanOutcome::UNAVAILABLE;
        require(decide().action == CommitAction::EMERGENCY_STOP,
                "failed planning without a safe command cannot keep the old command");
        input.result.context.tracking_outcome = TrackingPlanOutcome::COMMITTED;
        require(decide().action == CommitAction::COMMIT_CANDIDATE && decide().publish_trajectory,
                "fresh tracking exits recovery");
    }
    manager.onCommitted();
    manager.onKeepOld();
    manager.onKeepOld();
    manager.onKeepOld();
    require(manager.consecutiveKeepOld() == 3 && manager.hasCommittedTracking(),
            "keep counter is diagnostic; elapsed safety and progress bound retention");
    require(!manager.evaluateActivity(moving, 0.8, target, 0.2, 0.05).active,
            "expired prediction cannot keep a trajectory active");

    manager.onRejected();
    manager.onHold();
    require(!manager.hasCommittedTracking() && manager.consecutiveKeepOld() == 0 &&
            manager.consecutiveReject() == 2, "HOLD must preserve failures and retire tracking ownership");
    require(manager.hasRecoveryCommand() && manager.outcome() == TrackingPlanOutcome::COMMITTED_RECOVERY,
            "recovery ownership must be explicit");
    manager.onRecoveryKept();
    require(manager.outcome() == TrackingPlanOutcome::KEPT_RECOVERY && manager.consecutiveReject() == 3,
            "continuing recovery counts one failed attempt without another commit");
    manager.onCommitted();
    require(!manager.hasRecoveryCommand(), "tracking commit retires recovery ownership");
    require(manager.hasCommittedTracking() && manager.consecutiveReject() == 0 &&
            manager.consecutiveKeepOld() == 0, "successful commit must reset both counters");

    manager.reset();
    const general_utils::vec_Vec3f guide{Vec3f(-5.0, 0.0, 1.5), Vec3f(-1.0, 0.0, 1.5)};
    manager.onCommitted(100.0, guide.front(), guide);
    manager.observeExecution(100.0, guide.front());
    manager.onHold();
    require(manager.hasExecutionHistory(), "HOLD must preserve the execution watchdog history");
    manager.onCommitted(101.5, guide.front(), guide);
    manager.observeExecution(101.5, guide.front());
    const auto stalled = manager.evaluateActivity(straight(2.0, 101.5), 0.0,
                                                   prediction(101.5), 0.35, 0.05);
    require(stalled.active,
            "safe command retention must not be vetoed by measured progress");
    manager.observeExecution(101.6, guide.front() + Vec3f(0.2, 0.0, 0.0));
    require(manager.evaluateActivity(straight(2.0, 101.6), 0.0,
                                      prediction(101.6), 0.35, 0.05).active,
            "measured forward progress must recover trajectory activity");
    manager.reset();
    {
        general_planner::Config lost_cfg;
        lost_cfg.tracking_distance = 5.0;
        lost_cfg.tracking_height_offset = 0.7;
        lost_cfg.tracking_distance_tolerance = 0.5;
        TrackingRuntimeManager lost_manager(lost_cfg, nullptr);
        auto far = prediction();
        for (auto &sample : far) sample.position = Vec3f(20.0, 0.0, 0.8);
        const auto lost = lost_manager.evaluateActivity(straight(2.0), 0.0, far, 0.35, 0.05);
        require(lost.valid && lost.safe && !lost.active,
                "keep-old must not retain a command that has left the observation ring");
    }
    manager.reset();
    require(!manager.hasExecutionHistory() && !manager.hasCommittedTracking() &&
            manager.status() == TrackingRuntimeManager::Status::IDLE,
            "explicit reset must clear the previous task lifecycle");
}

} // namespace

int main() {
    try {
        runtimeLifecycle();
        std::cout << "Tracking runtime: safety, keep-old, prediction expiry and HOLD lifecycle passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
