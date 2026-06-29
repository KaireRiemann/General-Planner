#pragma once

#include <cmath>
#include <string>

#include <checker/check_result.hpp>
#include <utils/header/eigen_alias.hpp>

namespace general_planner::checker {
    inline bool finiteScalar(const double value) {
        return std::isfinite(value);
    }

    template <typename Derived>
    bool finiteEigen(const Eigen::MatrixBase<Derived> &value) {
        return value.allFinite();
    }

    inline bool yawValueValid(const double yaw) {
        return std::isnan(yaw) || std::isfinite(yaw);
    }

    inline bool quaternionValidOrDisabled(const general_utils::Quatf &q) {
        const bool all_nan = std::isnan(q.w()) && std::isnan(q.x()) &&
                             std::isnan(q.y()) && std::isnan(q.z());
        if (all_nan) {
            return true;
        }
        if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
            !std::isfinite(q.y()) || !std::isfinite(q.z())) {
            return false;
        }
        return q.norm() > 1.0e-6;
    }

    inline CheckResult checkStateFinite(const general_utils::StatePVAJ &state,
                                        const std::string &name) {
        if (!state.allFinite()) {
            return CheckResult::Reject(name + "_NON_FINITE", name + " contains NaN or Inf");
        }
        return CheckResult::Ok();
    }
}
