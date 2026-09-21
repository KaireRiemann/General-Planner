#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace temporal_map {
// Elastic-Tracker: T_total = horizon + slack^2, with N-1 positive allocation
// coordinates and a fixed final weight. The optional segment floor prevents
// singular MINCO systems without imposing an upper bound on catch-up time.
class TrackingTimeMap {
public:
    bool configure(int pieces, double minimum_piece, double horizon) {
        pieces_ = pieces;
        floor_ = std::max(1.e-4, minimum_piece);
        lower_ = std::max(horizon, pieces*floor_+1.e-3);
        return pieces > 0 && std::isfinite(lower_) && horizon > 0.0;
    }
    double minimumTotal() const { return lower_; }
    Eigen::VectorXd decode(const Eigen::Ref<const Eigen::VectorXd> &x) const {
        return Eigen::VectorXd::Constant(pieces_, floor_) +
            (lower_+x(0)*x(0)-pieces_*floor_)*allocation(x);
    }
    Eigen::VectorXd encode(const Eigen::Ref<const Eigen::VectorXd> &times) const {
        Eigen::VectorXd x(pieces_);
        x(0) = std::sqrt(std::max(1.e-4, times.sum()-lower_));
        const double last = std::max(1.e-8, times(pieces_-1)-floor_);
        for (int i = 1; i < pieces_; ++i) x(i) = inverse(std::max(1.e-8, times(i-1)-floor_)/last);
        return x;
    }
    Eigen::VectorXd backward(const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &g) const {
        const auto weights = allocation(x);
        const double mean = weights.dot(g);
        double denominator = 1.0;
        for (int i = 1; i < pieces_; ++i) denominator += positive(x(i));
        Eigen::VectorXd result(pieces_);
        result(0) = 2.0*x(0)*mean;
        const double free = lower_+x(0)*x(0)-pieces_*floor_;
        for (int i = 1; i < pieces_; ++i)
            result(i) = free*derivative(x(i))/denominator*(g(i-1)-mean);
        return result;
    }
private:
    static double positive(double x) {
        return x > 0.0 ? (0.5*x+1.0)*x+1.0 : 1.0/((0.5*x-1.0)*x+1.0);
    }
    static double derivative(double x) {
        const double denominator = (0.5*x-1.0)*x+1.0;
        return x > 0.0 ? x+1.0 : (1.0-x)/(denominator*denominator);
    }
    static double inverse(double t) {
        return t > 1.0 ? std::sqrt(2.0*t-1.0)-1.0 : 1.0-std::sqrt(2.0/t-1.0);
    }
    Eigen::VectorXd allocation(const Eigen::Ref<const Eigen::VectorXd> &x) const {
        Eigen::VectorXd weights = Eigen::VectorXd::Ones(pieces_);
        for (int i = 1; i < pieces_; ++i) weights(i-1) = positive(x(i));
        return weights/weights.sum();
    }
    int pieces_{0};
    double floor_{0.01}, lower_{3.0};
};
} // namespace temporal_map
