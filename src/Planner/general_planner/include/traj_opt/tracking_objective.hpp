#pragma once
#include <traj_opt/costfunctional/temporalmap/identity_time_map.hpp>
#include <traj_opt/costfunctional/temporalmap/tracking_time_map.hpp>
#include <traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp>
#include <traj_opt/costfunctional_manager/tracking_cost_manager.hpp>
#include <traj_opt/minco/minco_optimizer.hpp>

namespace traj_opt {
// Upstream Elastic-Tracker's position-only minimum-jerk objective, expressed
// through the common MINCOOptimizer and GP's spatial/cost-manager interfaces.
class TrackingObjective {
public:
    using Optimizer = minco::MINCOOptimizer<3, 3, temporal_map::IdentityTimeMap,
                                           spatial_map::PolytopeSpatialMap>;
    using Trajectory = Optimizer::TrajType;
    TrackingObjective(const Config &cfg, const TrackingProblem &problem,
                      const spatial_map::PolyhedraH &corridors)
        : costs_(cfg, problem, corridors), time_weight_(cfg.penna_t) {
        optimizer_.setEnergyWeight(cfg.block_energy_cost ? 0.0 : 1.0);
        optimizer_.setSamplesPerPiece(std::max(1, cfg.integral_reso));
    }
    bool initialize(const TrackingProblem &problem, const std::vector<double> &times,
                    const Optimizer::WaypointsType &points,
                    const spatial_map::PolytopeSpatialMap *spatial = nullptr) {
        pieces_ = static_cast<int>(times.size());
        if (!time_map_.configure(pieces_, problem.min_piece_duration, problem.min_total_duration)) return false;
        optimizer_.setSpatialMap(spatial);
        optimizer_.setInitState(times, points, problem.head_pvaj.leftCols<3>(), problem.tail_pvaj.leftCols<3>());
        physical_ = optimizer_.generateInitialGuess();
        initial_ = physical_;
        if (initial_.size() < pieces_) return false;
        initial_.head(pieces_) = time_map_.encode(Eigen::Map<const Eigen::VectorXd>(times.data(), pieces_));
        return initial_.allFinite();
    }
    const Eigen::VectorXd &initialGuess() const { return initial_; }
    const Trajectory &position() const { return optimizer_.getTrajectory(); }
    double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &gradient) {
        gradient.setZero(x.size());
        if (x.size() != initial_.size() || !x.allFinite()) return std::numeric_limits<double>::infinity();
        physical_ = x;
        physical_.head(pieces_) = time_map_.decode(x.head(pieces_));
        const auto time_cost = [&](const std::vector<double> &times, Eigen::VectorXd &g) {
            g.setConstant(time_weight_);
            double total = 0.0; for (double t : times) total += t;
            return time_weight_*(total-time_map_.minimumTotal());
        };
        const double cost = optimizer_.evaluateWithBoundaryMapping(physical_, gradient, time_cost, costs_, nullptr);
        const Eigen::VectorXd duration_gradient = gradient.head(pieces_);
        gradient.head(pieces_) = time_map_.backward(x.head(pieces_), duration_gradient);
        return cost;
    }
private:
    Optimizer optimizer_;
    cost_functional_manager::TrackingCostManager costs_;
    temporal_map::TrackingTimeMap time_map_;
    double time_weight_;
    int pieces_{0};
    Eigen::VectorXd initial_, physical_;
};
} // namespace traj_opt
