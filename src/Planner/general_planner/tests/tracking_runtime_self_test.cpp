#include <general_core/tracking/tracking_runtime_manager.hpp>

#include <iostream>
#include <stdexcept>

namespace {
using general_planner::TrackingRuntimeManager;
using general_utils::Vec3f;
using geometry_utils::Trajectory;
using Decision = TrackingRuntimeManager::DecisionType;

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
    cfg.tracking_keep_old_startup_grace = 1.0;
    cfg.tracking_max_consecutive_keep_old = 2;
    TrackingRuntimeManager manager(cfg, nullptr);
    const auto moving = straight(2.0);
    const auto stopped = straight(0.0);
    const auto target = prediction();

    require(manager.decide(nullptr, 0.0, moving, target, true, true).type ==
            Decision::COMMIT_CANDIDATE, "safe moving candidate must be accepted");
    require(manager.decide(nullptr, 0.0, moving, target, false, true).type ==
            Decision::REJECT_AND_FAIL, "unsafe candidate must be rejected without a command");
    require(manager.decide(nullptr, 0.0, stopped, target, true, true).type ==
            Decision::REJECT_AND_FAIL, "stationary candidate cannot follow a moving target");
    require(manager.decide(nullptr, 0.0, moving, {}, true, true).type ==
            Decision::REJECT_AND_FAIL, "missing prediction cannot produce a command");

    manager.onCommitted();
    require(manager.decide(&moving, 0.0, moving, target, false, true).type ==
            Decision::KEEP_OLD, "unsafe replacement must keep a safe active command");
    require(manager.decide(&stopped, 0.0, moving, target, false, true).type ==
            Decision::REJECT_AND_FAIL, "unsafe replacement cannot keep an inactive command");
    require(manager.decide(&moving, 0.0, stopped, target, true, true).type ==
            Decision::KEEP_OLD, "no-motion replacement must keep the moving command");
    require(manager.decide(&moving, 0.0, moving, target, true, false).type ==
            Decision::KEEP_OLD, "active command must honor anti-rollback");
    manager.onKeepOld();
    manager.onKeepOld();
    require(manager.consecutiveKeepOld() == 2, "keep-old must count each event once");
    require(manager.decide(&moving, 0.0, moving, target, true, false).type ==
            Decision::FORCE_COMMIT_CANDIDATE, "keep-old budget must allow a safe moving replacement");
    require(manager.decide(&moving, 0.0, moving, target, false, false).type ==
            Decision::KEEP_OLD, "keep-old budget cannot override collision safety");
    require(manager.decide(&moving, 0.0, stopped, target, true, false).type ==
            Decision::KEEP_OLD, "keep-old budget cannot override the motion gate");
    require(!manager.evaluateActivity(moving, 0.8, target, 0.2, 0.05).active,
            "expired prediction cannot keep a trajectory active");

    manager.onRejected();
    manager.onHold();
    require(!manager.hasCommittedTracking() && manager.consecutiveKeepOld() == 0 &&
            manager.consecutiveReject() == 2, "HOLD must preserve failures and retire tracking ownership");
    manager.onCommitted();
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
    require(!stalled.active && stalled.reason.find("EXECUTION_STALLED") != std::string::npos,
            "HOLD and recommit must not restart startup grace for a stalled vehicle");
    manager.observeExecution(101.6, guide.front() + Vec3f(0.2, 0.0, 0.0));
    require(manager.evaluateActivity(straight(2.0, 101.6), 0.0,
                                      prediction(101.6), 0.35, 0.05).active,
            "measured forward progress must recover trajectory activity");
    manager.reset();
    require(!manager.hasExecutionHistory() && !manager.hasCommittedTracking() &&
            manager.status() == TrackingRuntimeManager::Status::IDLE,
            "explicit reset must clear the previous task lifecycle");
}
} // namespace

int main() {
    try {
        runtimeLifecycle();
        std::cout << "Tracking runtime: safety, motion, keep-old, prediction expiry and HOLD lifecycle passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
