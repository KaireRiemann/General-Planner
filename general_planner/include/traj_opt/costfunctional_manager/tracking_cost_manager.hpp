#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/tracking_observation_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/tracking_visibility_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
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
	        (void)flatness;
	        cfg_ = &cfg;
	        map_manager_ = map_manager;
	        problem_ = std::move(problem);
	    }

	    const std::vector<double> &discreteSampleTimes() const
	    {
	        return problem_.target_sample_times;
	    }

    double evaluateIntegral(int,
                            double,
                            double t_global,
                            int,
                            int,
	                            const Eigen::Vector3d &position,
	                            const Eigen::Vector3d &velocity,
	                            const Eigen::Vector3d &acceleration,
	                            const Eigen::Vector3d &,
	                            Eigen::Vector3d &grad_position,
	                            Eigen::Vector3d &grad_velocity,
	                            Eigen::Vector3d &grad_acceleration,
	                            Eigen::Vector3d &,
	                            double &) const
	    {
	        (void)t_global;
	        double cost = 0.0;
	        cost += addObstacleAvoidanceCost(position, grad_position);
	        cost += addVelocityBoundCost(velocity, grad_velocity);
	        cost += addAccelerationBoundCost(acceleration, grad_acceleration);
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
	                               double &grad_time) const
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
	        cost += addObservationAngleCost(position,
	                                        yaw,
	                                        target,
	                                        grad_position,
	                                        grad_target,
	                                        grad_yaw);
            cost += addCameraFovCost(position,
                                     yaw,
                                     target,
                                     grad_position,
                                     grad_target,
                                     grad_yaw);
	        cost += addESDFVisibilityCost(position,
	                                      target,
	                                      grad_position,
	                                      grad_target);
            cost += addTargetForwardCost(position,
                                         target,
                                         grad_position,
                                         grad_target);
            cost += addTrackingVelocityCost(position,
                                            velocity,
                                            target,
                                            grad_velocity,
                                            grad_time);
            cost += addViewpointAttractorCost(t_global,
                                              position,
                                              grad_position,
                                              grad_time);
            cost += addVisibleRegionCost(t_global,
                                         position,
                                         target,
                                         grad_position,
                                         grad_time);

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
                double grad_time = 0.0;
	            cost += evaluateDiscretePositionSample(sample.t_global,
	                                                   sample.p,
	                                                   grad_position,
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
	    double addObstacleAvoidanceCost(const Eigen::Vector3d &position,
	                                    Eigen::Vector3d &grad_position) const
	    {
	        if (cfg_ == nullptr)
	        {
	            return 0.0;
	        }
	        return cost_functional::accumulateESDFDistancePenalty(map_manager_.get(),
	                                                              position,
	                                                              problem_.safe_distance,
	                                                              cfg_->smooth_eps,
	                                                              cfg_->penna_pos,
	                                                              grad_position);
	    }

	    double addVelocityBoundCost(const Eigen::Vector3d &velocity,
	                                Eigen::Vector3d &grad_velocity) const
	    {
	        if (cfg_ == nullptr)
	        {
	            return 0.0;
	        }
	        const double max_vel = clampPositive(cfg_->max_vel, 2.0);
	        return cost_functional::accumulateVelocityBoundPenalty(velocity,
	                                                               max_vel * max_vel,
	                                                               cfg_->smooth_eps,
	                                                               cfg_->penna_vel,
	                                                               grad_velocity);
	    }

	    double addAccelerationBoundCost(const Eigen::Vector3d &acceleration,
	                                    Eigen::Vector3d &grad_acceleration) const
	    {
	        if (cfg_ == nullptr)
	        {
	            return 0.0;
	        }
	        const double max_acc = clampPositive(cfg_->max_acc, 2.0);
	        return cost_functional::accumulateAccelerationBoundPenalty(acceleration,
	                                                                   max_acc * max_acc,
	                                                                   cfg_->smooth_eps,
	                                                                   cfg_->penna_acc,
	                                                                   grad_acceleration);
	    }

	    double evaluateDiscretePositionSample(double t_global,
	                                          const Eigen::Vector3d &position,
	                                          Eigen::Vector3d &grad_position,
                                              double &grad_time) const
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
	        cost += addESDFVisibilityCost(position,
	                                      target,
	                                      grad_position,
	                                      grad_target);
            cost += addTargetForwardCost(position,
                                         target,
                                         grad_position,
                                         grad_target);
            cost += addViewpointAttractorCost(t_global,
                                              position,
                                              grad_position,
                                              grad_time);
            cost += addVisibleRegionCost(t_global,
                                         position,
                                         target,
                                         grad_position,
                                         grad_time);
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

    double addCameraFovCost(const Eigen::Vector3d &position,
                            double yaw,
                            const traj_opt::DynamicTargetState &target,
                            Eigen::Vector3d &grad_position,
                            Eigen::Vector3d &grad_target,
                            double &grad_yaw) const
    {
        cost_functional::TrackingCameraFovConfig config;
        config.weight = problem_.weight_fov;
        config.horizontal_fov = problem_.fov_horizontal;
        config.vertical_fov = problem_.fov_vertical;
        config.range = problem_.fov_range;
        config.angle_clearance = problem_.visibility_angle_clearance;
        config.min_forward = 0.05;
        config.smooth_eps = cfg_->smooth_eps;
        return cost_functional::accumulateTrackingCameraFovPenalty(position,
                                                                   yaw,
                                                                   target.position,
                                                                   config,
                                                                   grad_position,
                                                                   grad_yaw,
                                                                   &grad_target);
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

    double addTargetForwardCost(const Eigen::Vector3d &position,
                                const traj_opt::DynamicTargetState &target,
                                Eigen::Vector3d &grad_position,
                                Eigen::Vector3d &grad_target) const
    {
        cost_functional::TrackingTargetForwardConfig config;
        config.weight = problem_.weight_target_forward;
        config.margin = problem_.target_front_margin;
        config.speed_threshold = problem_.target_motion_speed_threshold;
        config.smooth_eps = cfg_->smooth_eps;
        return cost_functional::accumulateTrackingTargetForwardPenalty(position,
                                                                       target.position,
                                                                       target.velocity,
                                                                       config,
                                                                       grad_position,
                                                                       &grad_target);
    }

    double addTrackingVelocityCost(const Eigen::Vector3d &position,
                                   const Eigen::Vector3d &velocity,
                                   const traj_opt::DynamicTargetState &target,
                                   Eigen::Vector3d &grad_velocity,
                                   double &grad_time) const
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
                                                                  grad_time);
    }

    struct TimedPointReference
    {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        bool valid{false};
    };

    TimedPointReference interpolateViewpoint(double t) const
    {
        TimedPointReference out;
        if (problem_.viewpoints.empty() ||
            problem_.target_sample_times.size() != problem_.viewpoints.size())
        {
            return out;
        }

        if (problem_.viewpoints.size() == 1 || t <= problem_.target_sample_times.front())
        {
            out.position = problem_.viewpoints.front();
            if (problem_.viewpoints.size() >= 2)
            {
                const double dt = std::max(1.0e-9,
                                           problem_.target_sample_times[1] -
                                               problem_.target_sample_times[0]);
                out.velocity = (problem_.viewpoints[1] - problem_.viewpoints[0]) / dt;
            }
            out.valid = true;
            return out;
        }
        if (t >= problem_.target_sample_times.back())
        {
            const std::size_t last = problem_.viewpoints.size() - 1;
            out.position = problem_.viewpoints[last];
            if (problem_.viewpoints.size() >= 2)
            {
                const std::size_t prev = last - 1;
                const double dt = std::max(1.0e-9,
                                           problem_.target_sample_times[last] -
                                               problem_.target_sample_times[prev]);
                out.velocity = (problem_.viewpoints[last] - problem_.viewpoints[prev]) / dt;
            }
            out.valid = true;
            return out;
        }

        const auto it = std::lower_bound(problem_.target_sample_times.begin(),
                                         problem_.target_sample_times.end(),
                                         t);
        const std::size_t idx = static_cast<std::size_t>(
            std::distance(problem_.target_sample_times.begin(), it));
        const std::size_t left = idx - 1;
        const std::size_t right = idx;
        const double dt = std::max(1.0e-9,
                                   problem_.target_sample_times[right] -
                                       problem_.target_sample_times[left]);
        const double alpha = (t - problem_.target_sample_times[left]) / dt;
        out.position = problem_.viewpoints[left] +
                       alpha * (problem_.viewpoints[right] - problem_.viewpoints[left]);
        out.velocity = (problem_.viewpoints[right] - problem_.viewpoints[left]) / dt;
        out.valid = true;
        return out;
    }

    double addViewpointAttractorCost(double t_global,
                                     const Eigen::Vector3d &position,
                                     Eigen::Vector3d &grad_position,
                                     double &grad_time) const
    {
        if (problem_.weight_viewpoint_attractor <= 0.0)
        {
            return 0.0;
        }
        const TimedPointReference ref = interpolateViewpoint(t_global);
        if (!ref.valid)
        {
            return 0.0;
        }

        cost_functional::TrackingPointAttractorConfig config;
        config.weight = problem_.weight_viewpoint_attractor;
        return cost_functional::accumulateTrackingPointAttractorPenalty(position,
                                                                        ref.position,
                                                                        ref.velocity,
                                                                        config,
                                                                        grad_position,
                                                                        grad_time);
    }

    bool interpolateVisibleRegion(double t,
                                  traj_opt::TrackingVisibleRegion &region,
                                  Eigen::Vector3d &visible_velocity) const
    {
        region = traj_opt::TrackingVisibleRegion{};
        visible_velocity.setZero();
        if (!problem_.use_visible_region ||
            problem_.visible_regions.empty())
        {
            return false;
        }

        const auto validRegionAt = [&](std::size_t idx) {
            return idx < problem_.visible_regions.size() &&
                   problem_.visible_regions[idx].valid;
        };

        if (problem_.visible_regions.size() == 1 || t <= problem_.visible_regions.front().t)
        {
            if (!validRegionAt(0))
            {
                return false;
            }
            region = problem_.visible_regions.front();
            return true;
        }
        if (t >= problem_.visible_regions.back().t)
        {
            const std::size_t last = problem_.visible_regions.size() - 1;
            if (!validRegionAt(last))
            {
                return false;
            }
            region = problem_.visible_regions[last];
            return true;
        }

        const auto it = std::lower_bound(problem_.visible_regions.begin(),
                                         problem_.visible_regions.end(),
                                         t,
                                         [](const traj_opt::TrackingVisibleRegion &lhs,
                                            double query_t) {
                                             return lhs.t < query_t;
                                         });
        const std::size_t right = static_cast<std::size_t>(
            std::distance(problem_.visible_regions.begin(), it));
        const std::size_t left = right - 1;
        if (!validRegionAt(left) || !validRegionAt(right))
        {
            return false;
        }

        const auto &lhs = problem_.visible_regions[left];
        const auto &rhs = problem_.visible_regions[right];
        const double dt = std::max(1.0e-9, rhs.t - lhs.t);
        const double alpha = (t - lhs.t) / dt;
        region.valid = true;
        region.t = t;
        region.target_position =
            lhs.target_position + alpha * (rhs.target_position - lhs.target_position);
        region.visible_point =
            lhs.visible_point + alpha * (rhs.visible_point - lhs.visible_point);
        region.theta = lhs.theta + alpha * (rhs.theta - lhs.theta);
        region.confidence = lhs.confidence + alpha * (rhs.confidence - lhs.confidence);
        visible_velocity = (rhs.visible_point - lhs.visible_point) / dt;
        return true;
    }

    double addVisibleRegionCost(double t_global,
                                const Eigen::Vector3d &position,
                                const traj_opt::DynamicTargetState &target,
                                Eigen::Vector3d &grad_position,
                                double &grad_time) const
    {
        if (problem_.weight_visible_region <= 0.0)
        {
            return 0.0;
        }

        traj_opt::TrackingVisibleRegion region;
        Eigen::Vector3d visible_velocity = Eigen::Vector3d::Zero();
        if (!interpolateVisibleRegion(t_global, region, visible_velocity))
        {
            return 0.0;
        }

        cost_functional::TrackingVisibleRegionConfig config;
        config.theta = region.theta;
        config.confidence = region.confidence;
        config.angle_clearance = problem_.visibility_angle_clearance;
        config.weight = problem_.weight_visible_region;
        config.smooth_eps = cfg_->smooth_eps;
        return cost_functional::accumulateTrackingVisibleRegionPenalty(position,
                                                                       target.position,
                                                                       target.velocity,
                                                                       region.visible_point,
                                                                       visible_velocity,
                                                                       config,
                                                                       grad_position,
                                                                       grad_time);
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

	    static double clampPositive(double value, double fallback)
	    {
	        if (!std::isfinite(value) || value <= 0.0)
	        {
	            return fallback;
	        }
	        return value;
	    }

	private:
	    const traj_opt::Config *cfg_{nullptr};
	    general_planner::MapManager::Ptr map_manager_;
	    traj_opt::TrackingProblem problem_;
	};

} // namespace cost_functional_manager
