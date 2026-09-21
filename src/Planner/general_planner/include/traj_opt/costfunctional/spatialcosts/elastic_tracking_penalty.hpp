#pragma once
// Elastic-Tracker traj_opt.cc penalties, adapted to additive GP gradients.
// Copyright Jialin Ji, Neng Pan, Fei Gao / ZJU FAST Lab. GPL-3.0.
// Source: KaireRiemann/Elastic-Tracker, commit 0a302a2 (modified 2026-09-20).
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace cost_functional {
inline double elasticCubic(double violation, double weight, double &gradient) {
    if (violation <= 0.0 || weight <= 0.0) return 0.0;
    gradient += 3.0*weight*violation*violation;
    return weight*violation*violation*violation;
}
inline double elasticFarPenalty(double x, double &gradient) {
    constexpr double epsilon = 0.05;
    if (x <= 0.0) return 0.0;
    if (x >= 2.0*epsilon) { gradient += 16.0; return 16.0*(x-epsilon); }
    gradient += 12.0*x*x/(epsilon*epsilon)-4.0*x*x*x/(epsilon*epsilon*epsilon);
    return 4.0*x*x*x/(epsilon*epsilon)-x*x*x*x/(epsilon*epsilon*epsilon);
}
inline double elasticTrackingDistance(const Eigen::Vector3d &position,
                                      const Eigen::Vector3d &center,
                                      double distance, double tolerance,
                                      double height_tolerance, double weight,
                                      Eigen::Vector3d &gradient) {
    const Eigen::Vector3d delta = position-center;
    const double r2 = delta.head<2>().squaredNorm();
    const double upper = distance+tolerance, lower = std::max(0.0, distance-tolerance);
    double cost = 0.0, g = 0.0;
    if (r2 > upper*upper) {
        cost += weight*elasticFarPenalty(r2-upper*upper, g);
        gradient.head<2>() += 2.0*weight*g*delta.head<2>();
    } else if (r2 < lower*lower) {
        cost += elasticCubic(lower*lower-r2, weight, g);
        gradient.head<2>() -= 2.0*g*delta.head<2>();
    }
    g = 0.0;
    cost += elasticCubic(delta.z()*delta.z()-height_tolerance*height_tolerance, weight, g);
    gradient.z() += 2.0*g*delta.z();
    return cost;
}
inline double elasticVisibility(const Eigen::Vector3d &position,
                               const Eigen::Vector3d &center,
                               const Eigen::Vector3d &visible_point,
                               double theta, double clearance, double weight,
                               Eigen::Vector3d &gradient) {
    const Eigen::Vector3d a = position-center, b = visible_point-center;
    const double an = a.norm(), bn = b.norm();
    if (an < 1.e-8 || bn < 1.e-8) return 0.0;
    const double cosine = a.dot(b)/(an*bn);
    const double violation = std::cos(std::max(0.0, theta-clearance))-cosine;
    double g = 0.0;
    const double cost = elasticCubic(violation, weight, g);
    gradient += g*(cosine*a/(an*an)-b/(an*bn));
    return cost;
}
} // namespace cost_functional
