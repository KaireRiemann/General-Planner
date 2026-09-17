#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>

namespace temporal_map {

// One bounded total duration and N-1 independent allocation coordinates.
// Physical segment times are shared by position and yaw. The segment floor
// and total bounds hold throughout the line search, not just at acceptance.
class TrackingTimeMap {
public:
    bool configure(int pieces, double minimum_piece, double minimum_total,
                   double maximum_total) {
        pieces_ = pieces;
        floor_ = std::max(1.e-4, minimum_piece);
        lower_ = std::max(minimum_total, pieces_ * floor_ + 1.e-3);
        upper_ = maximum_total;
        return pieces_ > 0 && std::isfinite(lower_) &&
               std::isfinite(upper_) && upper_ > lower_ + 1.e-4;
    }

    Eigen::VectorXd decode(const Eigen::Ref<const Eigen::VectorXd> &x) const {
        const auto weights = allocation(x);
        const double total = lower_ + (upper_ - lower_) * sigmoid(x(0));
        return Eigen::VectorXd::Constant(pieces_, floor_) +
               (total - pieces_ * floor_) * weights;
    }

    Eigen::VectorXd encode(const Eigen::Ref<const Eigen::VectorXd> &times) const {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(pieces_);
        const double total = std::clamp(times.sum(), lower_ + 1.e-6, upper_ - 1.e-6);
        const double ratio = std::clamp((total - lower_) / (upper_ - lower_), 1.e-8, 1.0 - 1.e-8);
        x(0) = std::log(ratio / (1.0 - ratio));
        const double tail = std::max(1.e-6, times(pieces_ - 1) - floor_);
        for (int i = 0; i + 1 < pieces_; ++i)
            x(i + 1) = std::log(std::max(1.e-6, times(i) - floor_) / tail);
        return x;
    }

    Eigen::VectorXd backward(const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &gradient) const {
        const auto weights = allocation(x);
        const double s = sigmoid(x(0));
        const double mean = weights.dot(gradient);
        const double remaining = lower_ + (upper_ - lower_) * s - pieces_ * floor_;
        Eigen::VectorXd result(pieces_);
        result(0) = mean * (upper_ - lower_) * s * (1.0 - s);
        for (int i = 0; i + 1 < pieces_; ++i)
            result(i + 1) = remaining * weights(i) * (gradient(i) - mean);
        return result;
    }

private:
    static double sigmoid(double x) {
        if (x >= 0.0) return 1.0 / (1.0 + std::exp(-x));
        const double e = std::exp(x);
        return e / (1.0 + e);
    }
    Eigen::VectorXd allocation(const Eigen::Ref<const Eigen::VectorXd> &x) const {
        Eigen::VectorXd logits = Eigen::VectorXd::Zero(pieces_);
        if (pieces_ > 1) logits.head(pieces_ - 1) = x.tail(pieces_ - 1);
        logits = (logits.array() - logits.maxCoeff()).exp();
        return logits / logits.sum();
    }
    int pieces_{0};
    double floor_{0.01}, lower_{0.1}, upper_{4.0};
};

} // namespace temporal_map
