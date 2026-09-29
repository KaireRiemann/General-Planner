#pragma once

#include <vector>

#include <Eigen/Eigen>
#include <traj_opt/costfunctional/penalty_utils.hpp>

namespace cost_functional
{
struct LinearTimeCost
{
    double weight = 0.0;
    // Negative preserves the ordinary linear objective. Native EPICON uses
    // smoothed |total_time - yaw_time_lower_bound| instead.
    double reference_duration = -1.0;

    double operator()(const std::vector<double> &Ts, Eigen::VectorXd &grad) const
    {
        double cost = 0.0;
        if (reference_duration >= 0.0) {
            double total = 0.0;
            for (double t : Ts) total += t;
            const double delta = total - reference_duration;
            double value = 0.0, derivative = 0.0;
            smoothedL1(std::abs(delta), 0.15, value, derivative);
            grad.array() += weight * (delta >= 0.0 ? derivative : -derivative);
            return weight * value;
        }
        for (size_t i = 0; i < Ts.size(); ++i)
        {
            cost += weight * Ts[i];
            grad(i) += weight;
        }
        return cost;
    }
};
}
