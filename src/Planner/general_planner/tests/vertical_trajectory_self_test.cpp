#include <general_core/vertical/vertical_trajectory.hpp>
#include <general_core/planner_runtime/planner_status.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

void require(bool ok, const char *message) {
    if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

int main() {
    using namespace general_planner;
    using namespace general_planner::planner_runtime;
    PlannerMode mode;
    require(parsePlannerMode("VERTICAL", mode) && mode == PlannerMode::VERTICAL &&
            isNavigationMode(mode) && ownerForMode(mode) == CommandOwner::STATE2STATE,
            "vertical mode must use the navigation command owner");
    VerticalMotionConfig cfg;
    geometry_utils::Trajectory position, yaw;
    std::string reason;
    const Eigen::Vector3d start(2.0, -3.0, 1.5);
    for (double delta : {-4.0, -0.2, -1e-5, 0.0, 1e-5, 0.2, 4.0}) {
        require(buildVerticalTrajectory(start, 0.7, start.z() + delta, cfg, position, yaw, reason),
                "valid vertical trajectory rejected");
        const double duration = position.getTotalDuration();
        require((position.getPos(0) - start).norm() < 1e-9, "start position mismatch");
        require(std::abs(position.getPos(duration).z() - start.z() - delta) < 1e-9,
                "endpoint mismatch");
        for (double t : {0.0, duration}) {
            require(position.getVel(t).norm() < 1e-9 && position.getAcc(t).norm() < 1e-9 &&
                    position.getJer(t).norm() < 1e-8, "endpoint derivatives must all vanish");
        }
        double last_z = start.z();
        for (int i = 0; i <= 1000; ++i) {
            const double t = duration * i / 1000;
            const auto p = position.getPos(t);
            require((p.head<2>() - start.head<2>()).norm() < 1e-12 &&
                    std::abs(yaw.getPos(t).x() - .7) < 1e-12 && yaw.getVel(t).norm() == 0,
                    "x/y/yaw drifted");
            require(position.getVel(t).norm() <= cfg.max_vel + 1e-9 &&
                    position.getAcc(t).norm() <= cfg.max_acc + 1e-9 &&
                    position.getJer(t).norm() <= cfg.max_jerk + 1e-9, "dynamics limit exceeded");
            require((p.z() - last_z) * delta >= -1e-10, "vertical motion must be monotonic");
            last_z = p.z();
        }
    }
    cfg.max_duration = .5;
    require(!buildVerticalTrajectory(start, 0, 5, cfg, position, yaw, reason),
            "duration must not be clamped below feasibility");
    cfg = VerticalMotionConfig{}; cfg.max_vel = 0;
    require(!buildVerticalTrajectory(start, 0, 2, cfg, position, yaw, reason), "zero limit accepted");
    cfg = VerticalMotionConfig{};
    require(!buildVerticalTrajectory(start, 0, std::numeric_limits<double>::infinity(), cfg,
                                    position, yaw, reason), "non-finite target accepted");
    std::cout << "vertical_trajectory_self_test passed\n";
}
