#pragma once
#include <traj_opt/config.hpp>
#include <traj_opt/tracking_problem.hpp>
#include <traj_opt/costfunctional/spatialmap/sfc_common_types.hpp>
#include <traj_opt/costfunctional/spatialcosts/elastic_tracking_penalty.hpp>

namespace cost_functional_manager {
// Integral terms and fixed prediction-time observations are deliberately
// separate. MINCOOptimizer supplies the absolute-time duration adjoints.
class TrackingCostManager {
public:
    TrackingCostManager(const traj_opt::Config &cfg, const traj_opt::TrackingProblem &problem,
                        const spatial_map::PolyhedraH &corridors)
        : cfg_(cfg), problem_(problem), corridors_(corridors) {
        const std::size_t count = problem_.target_prediction.size()*4/5;
        for (std::size_t i = 0; i < count; ++i) times_.push_back(problem_.target_prediction[i].t);
    }
    const std::vector<double> &discreteSampleTimes() const { return times_; }
    double evaluateIntegral(int, double, double, int piece, int,
                            const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                            const Eigen::Vector3d &a, const Eigen::Vector3d &,
                            Eigen::Vector3d &gp, Eigen::Vector3d &gv,
                            Eigen::Vector3d &ga, Eigen::Vector3d &, double &) const {
        double cost = 0.0;
        if (!corridors_.empty()) {
            const auto &planes = corridors_.at(static_cast<std::size_t>(piece/2));
            const double clearance = std::max(0.0, problem_.corridor_clearance);
            for (int row = 0; row < planes.rows(); ++row) {
                const Eigen::Vector3d normal = planes.row(row).head<3>();
                double g = 0.0;
                cost += cost_functional::elasticCubic(
                    normal.dot(p)+planes(row,3)+clearance, cfg_.penna_pos, g);
                gp += g*normal;
            }
        }
        double g = 0.0;
        cost += cost_functional::elasticCubic(v.squaredNorm()-cfg_.max_vel*cfg_.max_vel, cfg_.penna_vel, g);
        gv += 2.0*g*v; g = 0.0;
        cost += cost_functional::elasticCubic(a.squaredNorm()-cfg_.max_acc*cfg_.max_acc, cfg_.penna_acc, g);
        ga += 2.0*g*a;
        return cost;
    }
    template <typename Samples>
    double evaluateSample(const Samples &samples,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &gp,
                          Eigen::VectorXd &) const {
        double cost = 0.0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto &target = problem_.target_prediction[i];
            const Eigen::Vector3d center = target.position+Eigen::Vector3d(0,0,problem_.height_offset);
            Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
            double sample_cost = cost_functional::elasticTrackingDistance(samples[i].p, center,
                problem_.tracking_distance, problem_.distance_tolerance,
                problem_.height_tolerance, problem_.weight_tracking, gradient);
            if (problem_.use_visible_region && i < problem_.visible_regions.size()) {
                const auto &region = problem_.visible_regions[i];
                if (region.valid) sample_cost += cost_functional::elasticVisibility(samples[i].p,
                    region.target_position, region.visible_point, region.theta,
                    problem_.visibility_angle_clearance, problem_.weight_visible_region, gradient);
            }
            const double dt = problem_.target_prediction[i+1].t-target.t;
            const double weight = dt*std::exp2(-3.0*static_cast<double>(i)/times_.size());
            cost += weight*sample_cost; gp.col(static_cast<Eigen::Index>(i)) += weight*gradient;
        }
        return cost;
    }
private:
    traj_opt::Config cfg_;
    traj_opt::TrackingProblem problem_;
    spatial_map::PolyhedraH corridors_;
    std::vector<double> times_;
};
} // namespace cost_functional_manager
