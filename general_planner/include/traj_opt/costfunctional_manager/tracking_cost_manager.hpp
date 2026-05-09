#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "traj_opt/costfunctional/spatialcosts/tracking_esdf_obstacle_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/tracking_observation_penalty.hpp"
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
        if (problem_.use_esdf_obstacle && problem_.weight_esdf_obstacle > 0.0)
        {
            dynamics_.weight_esdf = 0.0;
        }
    }

    double evaluateIntegral(int,
                            double,
                            double t_global,
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
                            double &grad_time) const
    {
        double cost = detail::accumulateDynamicsPenalty(dynamics_,
                                                        position,
                                                        velocity,
                                                        acceleration,
                                                        jerk,
                                                        grad_position,
                                                        grad_velocity,
                                                        grad_acceleration,
                                                        grad_jerk);
        cost += evaluatePositionTrackingSample(t_global,
                                               position,
                                               velocity,
                                               grad_position,
                                               grad_velocity,
                                               grad_time);
        return cost;
    }

    double evaluateJointSample(double t_global,
                               const Eigen::Vector3d &position,
                               const Eigen::Vector3d &velocity,
                               double yaw,
                               double,
                               Eigen::Vector3d &grad_position,
                               Eigen::Vector3d &grad_velocity,
                               double &grad_yaw,
                               double &,
                               double &grad_t_global) const
    {
        if (cfg_ == nullptr || problem_.target_prediction.empty())
        {
            return 0.0;
        }

        const auto target = interpolateTarget(t_global);
        Eigen::Vector3d grad_target = Eigen::Vector3d::Zero();
        double cost = 0.0;

        cost += addObservationDistanceCost(position,
                                           target,
                                           grad_position,
                                           grad_target);
        cost += addESDFObstacleCost(position,
                                    grad_position);
        cost += addObservationAngleCost(position,
                                        yaw,
                                        target,
                                        grad_position,
                                        grad_target,
                                        grad_yaw);
        cost += addStabilityCost(position,
                                 velocity,
                                 target,
                                 grad_position,
                                 grad_velocity,
                                 grad_target,
                                 grad_t_global);
        cost += addViewpointAttractorCost(position,
                                          t_global,
                                          grad_position,
                                          grad_t_global);
        cost += addVisibleRegionCost(position,
                                     t_global,
                                     target,
                                     grad_position,
                                     grad_t_global);
        cost += addESDFVisibilityCost(position,
                                      target,
                                      grad_position,
                                      grad_target);

        grad_t_global += grad_target.dot(target.velocity);
        return cost;
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
            Eigen::Vector3d grad_position = Eigen::Vector3d::Zero();
            Eigen::Vector3d grad_velocity = Eigen::Vector3d::Zero();
            double grad_time = 0.0;
            cost += evaluatePositionTrackingSample(sample.t_global,
                                                   sample.p,
                                                   sample.v,
                                                   grad_position,
                                                   grad_velocity,
                                                   grad_time);
            grad_p.col(i) += grad_position;
            grad_t_global(i) += grad_time;
        }
        return cost;
    }

    template <typename SampleBuffer>
    double evaluateSample(const SampleBuffer &samples,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_p,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_v,
                          Eigen::VectorXd &grad_yaw,
                          Eigen::VectorXd &grad_yaw_dot,
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
            Eigen::Vector3d grad_position = Eigen::Vector3d::Zero();
            Eigen::Vector3d grad_velocity = Eigen::Vector3d::Zero();
            double gyaw = 0.0;
            double gyaw_dot = 0.0;
            double gt = 0.0;
            cost += evaluateJointSample(sample.t_global,
                                        sample.p,
                                        sample.v,
                                        sample.yaw,
                                        sample.yaw_dot,
                                        grad_position,
                                        grad_velocity,
                                        gyaw,
                                        gyaw_dot,
                                        gt);
            grad_p.col(i) += grad_position;
            grad_v.col(i) += grad_velocity;
            grad_yaw(i) += gyaw;
            grad_yaw_dot(i) += gyaw_dot;
            grad_t_global(i) += gt;
        }
        return cost;
    }

private:
    double evaluatePositionTrackingSample(double t_global,
                                          const Eigen::Vector3d &position,
                                          const Eigen::Vector3d &velocity,
                                          Eigen::Vector3d &grad_position,
                                          Eigen::Vector3d &grad_velocity,
                                          double &grad_t_global) const
    {
        if (cfg_ == nullptr || problem_.target_prediction.empty())
        {
            return 0.0;
        }

        const auto target = interpolateTarget(t_global);
        Eigen::Vector3d grad_target = Eigen::Vector3d::Zero();
        double cost = 0.0;

        cost += addObservationDistanceCost(position,
                                           target,
                                           grad_position,
                                           grad_target);
        cost += addESDFObstacleCost(position,
                                    grad_position);
        cost += addStabilityCost(position,
                                 velocity,
                                 target,
                                 grad_position,
                                 grad_velocity,
                                 grad_target,
                                 grad_t_global);
        cost += addViewpointAttractorCost(position,
                                          t_global,
                                          grad_position,
                                          grad_t_global);
        cost += addVisibleRegionCost(position,
                                     t_global,
                                     target,
                                     grad_position,
                                     grad_t_global);
        cost += addESDFVisibilityCost(position,
                                      target,
                                      grad_position,
                                      grad_target);
        grad_t_global += grad_target.dot(target.velocity);
        return cost;
    }

    double addObservationDistanceCost(const Eigen::Vector3d &position,
                                      const traj_opt::DynamicTargetState &target,
                                      Eigen::Vector3d &grad_position,
                                      Eigen::Vector3d &grad_target) const
    {
        cost_functional::TrackingObservationDistanceConfig config;
        config.horizontal_lower = problem_.od_h_lower;
        config.horizontal_upper = problem_.od_h_upper;
        config.vertical_lower = problem_.od_v_lower;
        config.vertical_upper = problem_.od_v_upper;
        config.weight_near = problem_.weight_od_near;
        config.weight_far = problem_.weight_od_far;
        config.weight_vertical = problem_.weight_od_vertical;
        config.smooth_eps = cfg_->smooth_eps;
        return cost_functional::accumulateTrackingObservationDistancePenalty(position,
                                                                             target.position,
                                                                             config,
                                                                             grad_position,
                                                                             &grad_target);
    }

    double addESDFObstacleCost(const Eigen::Vector3d &position,
                               Eigen::Vector3d &grad_position) const
    {
        if (!problem_.use_esdf_obstacle ||
            problem_.weight_esdf_obstacle <= 0.0 ||
            map_manager_ == nullptr ||
            !map_manager_->hasESDF())
        {
            return 0.0;
        }

        return cost_functional::accumulateTrackingESDFObstaclePenalty(map_manager_.get(),
                                                                      position,
                                                                      problem_.safe_distance,
                                                                      cfg_->smooth_eps,
                                                                      problem_.weight_esdf_obstacle,
                                                                      grad_position);
    }

    double addObservationAngleCost(const Eigen::Vector3d &position,
                                   double yaw,
                                   const traj_opt::DynamicTargetState &target,
                                   Eigen::Vector3d &grad_position,
                                   Eigen::Vector3d &grad_target,
                                   double &grad_yaw) const
    {
        cost_functional::TrackingObservationAngleConfig config;
        config.weight = problem_.weight_oa;
        return cost_functional::accumulateTrackingObservationAnglePenalty(position,
                                                                          yaw,
                                                                          target.position,
                                                                          config,
                                                                          grad_position,
                                                                          grad_yaw,
                                                                          &grad_target);
    }

    double addStabilityCost(const Eigen::Vector3d &position,
                            const Eigen::Vector3d &velocity,
                            const traj_opt::DynamicTargetState &target,
                            Eigen::Vector3d &,
                            Eigen::Vector3d &grad_velocity,
                            Eigen::Vector3d &,
                            double &grad_t_global) const
    {
        cost_functional::TrackingVelocityConfig config;
        config.weight_relative = problem_.weight_relative_velocity;
        config.weight_tangent = problem_.weight_tangent_velocity;
        return cost_functional::accumulateTrackingVelocityPenalty(position,
                                                                  velocity,
                                                                  target.position,
                                                                  target.velocity,
                                                                  target.acceleration,
                                                                  config,
                                                                  grad_velocity,
                                                                  grad_t_global);
    }

    double addViewpointAttractorCost(const Eigen::Vector3d &position,
                                     double t_global,
                                     Eigen::Vector3d &grad_position,
                                     double &grad_t_global) const
    {
        if (problem_.weight_viewpoint_attractor <= 0.0)
        {
            return 0.0;
        }

        Eigen::Vector3d ref = Eigen::Vector3d::Zero();
        Eigen::Vector3d ref_v = Eigen::Vector3d::Zero();
        if (!interpolateViewpoint(t_global, ref, ref_v))
        {
            return 0.0;
        }

        cost_functional::TrackingPointAttractorConfig config;
        config.weight = problem_.weight_viewpoint_attractor;
        return cost_functional::accumulateTrackingPointAttractorPenalty(position,
                                                                        ref,
                                                                        ref_v,
                                                                        config,
                                                                        grad_position,
                                                                        grad_t_global);
    }

    double addVisibleRegionCost(const Eigen::Vector3d &position,
                                double t_global,
                                const traj_opt::DynamicTargetState &target,
                                Eigen::Vector3d &grad_position,
                                double &grad_t_global) const
    {
        if (!problem_.use_visible_region ||
            problem_.weight_visible_region <= 0.0 ||
            problem_.visible_regions.empty())
        {
            return 0.0;
        }

        Eigen::Vector3d visible_ref = Eigen::Vector3d::Zero();
        Eigen::Vector3d visible_ref_v = Eigen::Vector3d::Zero();
        double theta = 0.0;
        double confidence = 0.0;
        if (!interpolateVisibleRegion(t_global, visible_ref, visible_ref_v, theta, confidence) ||
            confidence <= 0.0)
        {
            return 0.0;
        }

        cost_functional::TrackingVisibleRegionConfig config;
        config.theta = theta;
        config.confidence = confidence;
        config.angle_clearance = problem_.visibility_angle_clearance;
        config.weight = problem_.weight_visible_region;
        config.smooth_eps = cfg_->smooth_eps;
        return cost_functional::accumulateTrackingVisibleRegionPenalty(position,
                                                                       target.position,
                                                                       target.velocity,
                                                                       visible_ref,
                                                                       visible_ref_v,
                                                                       config,
                                                                       grad_position,
                                                                       grad_t_global);
    }

    double addESDFVisibilityCost(const Eigen::Vector3d &position,
                                 const traj_opt::DynamicTargetState &target,
                                 Eigen::Vector3d &grad_position,
                                 Eigen::Vector3d &grad_target) const
    {
        const double weight = problem_.weight_oe > 0.0 ? problem_.weight_oe : problem_.weight_visibility;
        if (!problem_.use_esdf_visibility ||
            weight <= 0.0 ||
            problem_.visibility_samples <= 0 ||
            map_manager_ == nullptr ||
            !map_manager_->hasESDF())
        {
            return 0.0;
        }

        Eigen::Vector3d local_grad_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d local_grad_target = Eigen::Vector3d::Zero();
        const double cost =
            cost_functional::accumulateBallLineOfSightESDFPenalty(map_manager_.get(),
                                                                  position,
                                                                  target.position,
                                                                  problem_.visibility_safe_distance,
                                                                  problem_.visibility_cone_ratio,
                                                                  cfg_->smooth_eps,
                                                                  weight,
                                                                  problem_.visibility_samples,
                                                                  local_grad_position,
                                                                  &local_grad_target);
        grad_position += local_grad_position;
        grad_target += local_grad_target;
        return cost;
    }

    bool interpolateViewpoint(double t,
                              Eigen::Vector3d &position,
                              Eigen::Vector3d &velocity) const
    {
        const auto &path = problem_.viewpoints;
        const auto &times = problem_.target_sample_times;
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

    bool interpolateVisibleRegion(double t,
                                  Eigen::Vector3d &visible_point,
                                  Eigen::Vector3d &visible_velocity,
                                  double &theta,
                                  double &confidence) const
    {
        const auto &regions = problem_.visible_regions;
        if (regions.empty())
        {
            return false;
        }

        auto applyRegion = [&](const traj_opt::TrackingVisibleRegion &region) {
            if (!region.valid)
            {
                return false;
            }
            visible_point = region.visible_point;
            visible_velocity.setZero();
            theta = region.theta;
            confidence = std::clamp(region.confidence, 0.0, 1.0);
            return true;
        };

        if (t <= regions.front().t)
        {
            return applyRegion(regions.front());
        }
        if (t >= regions.back().t)
        {
            return applyRegion(regions.back());
        }

        int left_idx = -1;
        int right_idx = -1;
        for (int i = 0; i < static_cast<int>(regions.size()); ++i)
        {
            if (!regions[static_cast<std::size_t>(i)].valid)
            {
                continue;
            }
            if (regions[static_cast<std::size_t>(i)].t <= t)
            {
                left_idx = i;
            }
            if (regions[static_cast<std::size_t>(i)].t >= t)
            {
                right_idx = i;
                break;
            }
        }

        if (left_idx < 0 && right_idx < 0)
        {
            return false;
        }
        if (left_idx < 0)
        {
            return applyRegion(regions[static_cast<std::size_t>(right_idx)]);
        }
        if (right_idx < 0)
        {
            return applyRegion(regions[static_cast<std::size_t>(left_idx)]);
        }
        if (left_idx == right_idx)
        {
            return applyRegion(regions[static_cast<std::size_t>(left_idx)]);
        }
        if (right_idx - left_idx > 1)
        {
            return false;
        }

        const auto &left = regions[static_cast<std::size_t>(left_idx)];
        const auto &right = regions[static_cast<std::size_t>(right_idx)];
        if (!left.valid || !right.valid)
        {
            return false;
        }

        const double dt = std::max(1.0e-9, right.t - left.t);
        const double alpha = std::clamp((t - left.t) / dt, 0.0, 1.0);
        visible_point = left.visible_point + alpha * (right.visible_point - left.visible_point);
        visible_velocity = (right.visible_point - left.visible_point) / dt;
        theta = left.theta + alpha * (right.theta - left.theta);
        confidence = std::clamp(left.confidence + alpha * (right.confidence - left.confidence), 0.0, 1.0);
        return visible_point.allFinite() && visible_velocity.allFinite();
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
