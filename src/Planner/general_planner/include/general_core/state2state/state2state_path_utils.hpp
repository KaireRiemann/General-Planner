/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#pragma once

#include <algorithm>
#include <utils/header/eigen_alias.hpp>

namespace general_planner {
    using general_utils::Vec3f;
    using general_utils::vec_Vec3f;

    inline void appendPathPointUnique(const Vec3f &point, vec_Vec3f &path) {
        if (!point.allFinite()) {
            return;
        }
        if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
            path.emplace_back(point);
        }
    }

    inline double pathForwardProgress(const vec_Vec3f &path, const Vec3f &origin, const Vec3f &dir) {
        if (path.empty() || dir.norm() < 1.0e-6) {
            return 0.0;
        }
        return (path.back() - origin).dot(dir.normalized());
    }

    inline Vec3f state2stateGoalAxis(const Vec3f &axis_start,
                                     const Vec3f &fallback_start,
                                     const Vec3f &goal) {
        Vec3f axis(goal.x() - axis_start.x(), goal.y() - axis_start.y(), 0.0);
        if (axis.norm() < 1.0e-3) {
            axis = Vec3f(goal.x() - fallback_start.x(), goal.y() - fallback_start.y(), 0.0);
        }
        if (axis.norm() < 1.0e-3) {
            axis = goal - fallback_start;
        }
        if (axis.norm() < 1.0e-6) {
            return Vec3f::Zero();
        }
        return axis.normalized();
    }

    inline double state2stateGoalOvershoot(const Vec3f &point,
                                           const Vec3f &axis_start,
                                           const Vec3f &fallback_start,
                                           const Vec3f &goal) {
        const Vec3f axis = state2stateGoalAxis(axis_start, fallback_start, goal);
        if (axis.norm() < 1.0e-6) {
            return 0.0;
        }
        return (point - goal).dot(axis);
    }

    inline double state2stateMaxGoalOvershoot(const vec_Vec3f &path,
                                              const Vec3f &axis_start,
                                              const Vec3f &fallback_start,
                                              const Vec3f &goal) {
        double max_over = 0.0;
        for (const auto &point : path) {
            max_over = std::max(max_over,
                                state2stateGoalOvershoot(point,
                                                        axis_start,
                                                        fallback_start,
                                                        goal));
        }
        return max_over;
    }
}
