#pragma once

#include <algorithm>
#include <utility>

#include "traj_opt/costfunctional/spatialcosts/tracking_visibility_penalty.hpp"
#include "traj_opt/costfunctional_manager/task_dynamics_penalty.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace cost_functional_manager
{

class TrackingCostManager
{
public:
    void reset(const traj_opt::Config &cfg,
               const general_planner::MapManager::Ptr &map_manager,
               traj_opt::TrackingProblem problem,
               flatness::FlatnessMap *flatness)
    {
        cfg_ = &cfg;
        map_manager_ = map_manager;
        problem_ = std::move(problem);
        dynamics_ = detail::makeDynamicsPenaltyConfig(cfg,
                                                       map_manager_.get(),
                                                       problem_.safe_distance,
                                                       flatness);
    }

    double evaluateIntegral(int,
                            double,
                            double,
                            int,
                            int,
                            const Eigen::Vector3d &position,
                            const Eigen::Vector3d &velocity,
                            const Eigen::Vector3d &acceleration,
                            const Eigen::Vector3d &jerk,
                            Eigen::Vector3d &grad_position,
                            Eigen::Vector3d &grad_velocity,
                            Eigen::Vector3d &grad_acceleration,
                            Eigen::Vector3d &grad_jerk,
                            double &) const
    {
        return detail::accumulateDynamicsPenalty(dynamics_,
                                                 position,
                                                 velocity,
                                                 acceleration,
                                                 jerk,
                                                 grad_position,
                                                 grad_velocity,
                                                 grad_acceleration,
                                                 grad_jerk);
    }

    template <typename SampleBuffer>
    double evaluateSample(const SampleBuffer &samples,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_p,
                          Eigen::VectorXd &grad_t_global) const
    {
        if (cfg_ == nullptr || problem_.target_prediction.empty())
        {
            return 0.0;
        }

        double cost = 0.0;
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(samples.size()); ++i)
        {
            const auto &sample = samples[static_cast<std::size_t>(i)];
            const auto target = interpolateTarget(sample.t_global);
            Eigen::Vector3d grad_track = Eigen::Vector3d::Zero();
            Eigen::Vector3d grad_target = Eigen::Vector3d::Zero();
            cost += cost_functional::accumulateTrackingDistancePenalty(sample.p,
                                                                       target.position,
                                                                       problem_.tracking_distance,
                                                                       problem_.distance_tolerance,
                                                                       problem_.height_offset,
                                                                       problem_.height_tolerance,
                                                                       cfg_->smooth_eps,
                                                                       problem_.weight_tracking,
                                                                       grad_track,
                                                                       &grad_target);
            grad_p.col(i) += grad_track;

            if (problem_.use_esdf_visibility &&
                problem_.weight_visibility > 0.0 &&
                problem_.visibility_samples > 0 &&
                map_manager_ != nullptr &&
                map_manager_->hasESDF())
            {
                Eigen::Vector3d grad_visibility = Eigen::Vector3d::Zero();
                Eigen::Vector3d grad_visibility_target = Eigen::Vector3d::Zero();
                cost += cost_functional::accumulateBallLineOfSightESDFPenalty(map_manager_.get(),
                                                                              sample.p,
                                                                              target.position,
                                                                              problem_.visibility_safe_distance,
                                                                              problem_.visibility_cone_ratio,
                                                                              cfg_->smooth_eps,
                                                                              problem_.weight_visibility,
                                                                              problem_.visibility_samples,
                                                                              grad_visibility,
                                                                              &grad_visibility_target);
                grad_p.col(i) += grad_visibility;
                grad_target += grad_visibility_target;
            }
            grad_t_global(i) += grad_target.dot(target.velocity);

            Eigen::Vector3d guide_position = Eigen::Vector3d::Zero();
            Eigen::Vector3d guide_velocity = Eigen::Vector3d::Zero();
            if (cfg_->penna_attract > 0.0 &&
                interpolateGuide(sample.t_global, guide_position, guide_velocity))
            {
                const Eigen::Vector3d diff = sample.p - guide_position;
                cost += 0.5 * cfg_->penna_attract * diff.squaredNorm();
                grad_p.col(i) += cfg_->penna_attract * diff;
                grad_t_global(i) -= cfg_->penna_attract * diff.dot(guide_velocity);
            }
        }
        return cost;
    }

private:
    bool interpolateGuide(double t,
                          Eigen::Vector3d &position,
                          Eigen::Vector3d &velocity) const
    {
        const auto &path = problem_.guide_path;
        const auto &times = problem_.guide_t;
        if (path.size() < 2 || path.size() != times.size())
        {
            return false;
        }

        if (t <= times.front())
        {
            const double dt = std::max(1.0e-9, times[1] - times[0]);
            position = path.front();
            velocity = (path[1] - path[0]) / dt;
            return position.allFinite() && velocity.allFinite();
        }
        if (t >= times.back())
        {
            position = path.back();
            velocity.setZero();
            return position.allFinite();
        }

        const auto it = std::lower_bound(times.begin(), times.end(), t);
        const int idx = static_cast<int>(std::distance(times.begin(), it));
        if (idx <= 0 || idx >= static_cast<int>(path.size()))
        {
            return false;
        }

        const double left_t = times[static_cast<std::size_t>(idx - 1)];
        const double right_t = times[static_cast<std::size_t>(idx)];
        const double dt = std::max(1.0e-9, right_t - left_t);
        const double alpha = std::clamp((t - left_t) / dt, 0.0, 1.0);
        const Eigen::Vector3d &left_p = path[static_cast<std::size_t>(idx - 1)];
        const Eigen::Vector3d &right_p = path[static_cast<std::size_t>(idx)];
        position = left_p + alpha * (right_p - left_p);
        velocity = (right_p - left_p) / dt;
        return position.allFinite() && velocity.allFinite();
    }

    traj_opt::DynamicTargetState interpolateTarget(double t) const
    {
        if (problem_.target_prediction.empty())
        {
            return {};
        }
        if (problem_.target_prediction.size() == 1 || t <= problem_.target_prediction.front().t)
        {
            return problem_.target_prediction.front();
        }
        if (t >= problem_.target_prediction.back().t)
        {
            return problem_.target_prediction.back();
        }

        const auto it = std::lower_bound(problem_.target_prediction.begin(),
                                         problem_.target_prediction.end(),
                                         t,
                                         [](const traj_opt::DynamicTargetState &state, double query_t) {
                                             return state.t < query_t;
                                         });
        const int idx = static_cast<int>(std::distance(problem_.target_prediction.begin(), it));
        const auto &right = problem_.target_prediction[static_cast<std::size_t>(idx)];
        const auto &left = problem_.target_prediction[static_cast<std::size_t>(idx - 1)];
        const double alpha = (t - left.t) / std::max(1.0e-9, right.t - left.t);

        traj_opt::DynamicTargetState out;
        out.t = t;
        out.position = left.position + alpha * (right.position - left.position);
        out.velocity = left.velocity + alpha * (right.velocity - left.velocity);
        out.acceleration = left.acceleration + alpha * (right.acceleration - left.acceleration);
        out.yaw = left.yaw + alpha * (right.yaw - left.yaw);
        out.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
        return out;
    }

private:
    const traj_opt::Config *cfg_{nullptr};
    general_planner::MapManager::Ptr map_manager_;
    traj_opt::TrackingProblem problem_;
    detail::DynamicsPenaltyConfig dynamics_;
};

} // namespace cost_functional_manager
