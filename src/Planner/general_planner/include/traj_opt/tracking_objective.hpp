#pragma once

#include "traj_opt/costfunctional/temporalmap/identity_time_map.hpp"
#include "traj_opt/costfunctional/temporalmap/tracking_time_map.hpp"
#include "traj_opt/costfunctional/spatialmap/identify_spatial_map.hpp"
#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/costfunctional_manager/tracking_joint_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"

namespace traj_opt {

// Adapter for a precomputed joint coefficient cost. Both MINCO instances use
// the same physical times; only the position side accounts for its scalar value.
template <int DIM, int S>
struct TrackingCoefficientCost {
    using Traj = minco::MINCOTrajectory<DIM, S>;
    using Coeff = typename Traj::CoeffMat;
    using Vec = Eigen::Matrix<double, DIM, 1>;
    double value{0.0};
    Coeff coefficients;
    Eigen::VectorXd times;
    bool usesDenseSampling() const { return false; }
    bool usesSampleCost() const { return false; }
    double evaluateCoefficient(const Traj &, Coeff &gc, Eigen::VectorXd &gt) const {
        gc += coefficients; gt += times; return value;
    }
    double evaluateIntegral(int, double, double, int, int,
                            const Vec &, const Vec &, const Vec &, const Vec &,
                            Vec &, Vec &, Vec &, Vec &, double &) const { return 0.0; }
    template <typename Samples>
    double evaluateSample(const Samples &, Eigen::Matrix<double, DIM, Eigen::Dynamic> &,
                          Eigen::VectorXd &) const { return 0.0; }
};

// x = [bounded shared time coordinates, position coordinates, yaw coordinates].
// No task-specific MINCO adjoint or spatial-gradient implementation lives here.
template <int S>
class TrackingObjective {
public:
    using PosOptimizer = minco::MINCOOptimizer<3, S, temporal_map::IdentityTimeMap,
                                               spatial_map::PolytopeSpatialMap>;
    using YawOptimizer = minco::MINCOOptimizer<1, 3, temporal_map::IdentityTimeMap,
                                               spatial_map::IdentitySpatialMap<1>>;
    using PosTraj = typename PosOptimizer::TrajType;
    using YawTraj = typename YawOptimizer::TrajType;

    TrackingObjective(const Config &cfg, const TrackingProblem &problem,
                      const general_planner::MapManager::Ptr &map,
                      const spatial_map::PolyhedraH &corridors)
        : joint_(cfg, problem, map, corridors), time_weight_(std::max(0.0, cfg.penna_t)) {
        position_.setEnergyWeight(cfg.block_energy_cost ? 0.0 : 1.0);
        yaw_.setEnergyWeight(0.05);
    }

    bool initialize(const TrackingProblem &problem, const std::vector<double> &times,
                    const typename PosOptimizer::WaypointsType &points,
                    const typename YawOptimizer::WaypointsType &yaws,
                    const typename YawTraj::BoundaryState &yaw_head,
                    const typename YawTraj::BoundaryState &yaw_tail,
                    const spatial_map::PolytopeSpatialMap *spatial) {
        pieces_ = static_cast<int>(times.size());
        if (pieces_ == 0 || !time_map_.configure(pieces_, problem.min_piece_duration,
                std::max(problem.min_total_duration, problem.target_prediction.empty()
                    ? 0.0 : problem.target_prediction.back().t), problem.max_total_duration)) return false;
        position_.setSpatialMap(spatial);
        position_.setInitState(times, points, problem.head_pvaj.template leftCols<S>(),
                              problem.tail_pvaj.template leftCols<S>());
        yaw_.setInitState(times, yaws, yaw_head, yaw_tail);
        pos_x_ = position_.generateInitialGuess();
        yaw_x_ = yaw_.generateInitialGuess();
        if (pos_x_.size() < pieces_ || yaw_x_.size() < pieces_) return false;
        pos_dim_ = pos_x_.size() - pieces_;
        initial_.resize(pos_x_.size() + yaw_x_.size() - pieces_);
        initial_.head(pieces_) = time_map_.encode(Eigen::Map<const Eigen::VectorXd>(times.data(), pieces_));
        initial_.segment(pieces_, pos_dim_) = pos_x_.tail(pos_dim_);
        initial_.tail(pieces_ - 1) = yaw_x_.tail(pieces_ - 1);
        return initial_.allFinite();
    }

    const Eigen::VectorXd &initialGuess() const { return initial_; }
    const PosTraj &position() const { return position_.getTrajectory(); }
    const YawTraj &yaw() const { return yaw_.getTrajectory(); }

    double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &gradient) {
        gradient.setZero(x.size());
        if (x.size() != initial_.size() || !x.allFinite())
            return std::numeric_limits<double>::infinity();
        const Eigen::VectorXd times = time_map_.decode(x.head(pieces_));
        pos_x_.head(pieces_) = times;
        yaw_x_.head(pieces_) = times;
        pos_x_.tail(pos_dim_) = x.segment(pieces_, pos_dim_);
        yaw_x_.tail(pieces_ - 1) = x.tail(pieces_ - 1);
        if (!position_.updateTrajectoryFromDecisionVector(pos_x_) ||
            !yaw_.updateTrajectoryFromDecisionVector(yaw_x_))
            return std::numeric_limits<double>::infinity();

        pos_cost_.value = joint_.evaluate(position(), yaw(), pos_cost_.coefficients,
            yaw_cost_.coefficients, pos_cost_.times, yaw_cost_.times);
        yaw_cost_.value = 0.0;
        const auto no_time = [](const std::vector<double> &, Eigen::VectorXd &g) {
            g.setZero(); return 0.0;
        };
        Eigen::VectorXd gp = Eigen::VectorXd::Zero(pos_x_.size());
        Eigen::VectorXd gy = Eigen::VectorXd::Zero(yaw_x_.size());
        double cost = position_.evaluate(pos_x_, gp, no_time, pos_cost_) +
                      yaw_.evaluate(yaw_x_, gy, no_time, yaw_cost_);
        cost += time_weight_ * times.sum();
        Eigen::VectorXd gt = gp.head(pieces_) + gy.head(pieces_);
        gt.array() += time_weight_;
        gradient.head(pieces_) = time_map_.backward(x.head(pieces_), gt);
        gradient.segment(pieces_, pos_dim_) = gp.tail(pos_dim_);
        gradient.tail(pieces_ - 1) = gy.tail(pieces_ - 1);
        return cost;
    }

private:
    PosOptimizer position_;
    YawOptimizer yaw_;
    temporal_map::TrackingTimeMap time_map_;
    cost_functional_manager::TrackingJointCostManager<S> joint_;
    TrackingCoefficientCost<3, S> pos_cost_;
    TrackingCoefficientCost<1, 3> yaw_cost_;
    double time_weight_;
    int pieces_{0}, pos_dim_{0};
    Eigen::VectorXd initial_, pos_x_, yaw_x_;
};

} // namespace traj_opt
