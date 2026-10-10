#pragma once

#include <data_structure/base/trajectory.h>
#include <algorithm>
#include <cmath>
#include <string>

namespace general_planner {

struct VerticalMotionConfig {
    double max_vel{0.5};
    double max_acc{0.5};
    double max_jerk{1.0};
    double min_duration{1.0};
    double max_duration{60.0};
    double start_speed_tolerance{0.1};
    double tracking_error_max{0.3};
    double goal_tolerance{0.1};
    double completion_timeout{3.0};
    bool unknown_as_occupied{false};
};

// Rest-to-rest motion: s(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7.
// Its exact derivative maxima are 35/16, 84*sqrt(5)/25 and 105/2.
// No optimizer or time search is needed; scaling T enforces all three bounds.
inline bool buildVerticalTrajectory(const Eigen::Vector3d &start, double yaw0,
                                    double target_z, const VerticalMotionConfig &cfg,
                                    geometry_utils::Trajectory &position,
                                    geometry_utils::Trajectory &yaw,
                                    std::string &reason) {
    if (!start.allFinite() || !std::isfinite(yaw0) || !std::isfinite(target_z)) {
        reason = "non_finite_vertical_goal";
        return false;
    }
    for (double value : {cfg.max_vel, cfg.max_acc, cfg.max_jerk,
                         cfg.min_duration, cfg.max_duration,
                         cfg.start_speed_tolerance, cfg.tracking_error_max,
                         cfg.goal_tolerance, cfg.completion_timeout}) {
        if (!std::isfinite(value) || value <= 0.0) {
            reason = "invalid_vertical_limits";
            return false;
        }
    }
    const double delta = target_z - start.z();
    const double distance = std::abs(delta);
    const double duration = std::max({cfg.min_duration,
        (35.0 / 16.0) * distance / cfg.max_vel,
        std::sqrt((84.0 * std::sqrt(5.0) / 25.0) * distance / cfg.max_acc),
        std::cbrt(52.5 * distance / cfg.max_jerk)});
    // Never clamp the duration down: that would violate the requested limits.
    if (!std::isfinite(duration) || duration > cfg.max_duration) {
        reason = "vertical_duration_exceeds_limit";
        return false;
    }
    Eigen::Matrix<double, 3, 8> pc = Eigen::Matrix<double, 3, 8>::Zero();
    pc.col(7) = start;
    pc(2, 3) = 35.0 * delta / std::pow(duration, 4);
    pc(2, 2) = -84.0 * delta / std::pow(duration, 5);
    pc(2, 1) = 70.0 * delta / std::pow(duration, 6);
    pc(2, 0) = -20.0 * delta / std::pow(duration, 7);
    Eigen::Matrix<double, 3, 8> yc = Eigen::Matrix<double, 3, 8>::Zero();
    yc(0, 7) = yaw0;
    if (!pc.allFinite()) {
        reason = "invalid_vertical_coefficients";
        return false;
    }
    position.clear(); yaw.clear();
    position.emplace_back(duration, pc);
    yaw.emplace_back(duration, yc);
    reason = "ok";
    return true;
}

} // namespace general_planner
