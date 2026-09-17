#pragma once

#include <algorithm>
#include <array>
#include "utils/geometry/tracking_attitude.hpp"
#include <cmath>
#include <utility>
#include "traj_opt/config.hpp"

#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/tracking_observation_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/tracking_visibility_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/tracking_problem.hpp"

namespace cost_functional_manager
{

class TrackingCostManager
{
public:
    void reset(const traj_opt::Config &cfg,
               const general_planner::MapManager::Ptr &map_manager,
                   traj_opt::TrackingProblem problem)
        {
            cfg_ = &cfg;
            map_manager_ = map_manager;
            problem_ = std::move(problem);
            rebuildJointSampleTimes();
        }

        const std::vector<double> &discreteSampleTimes() const
        {
            return joint_sample_times_;
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
                                double &) const
        {
            (void)t_global;
            double cost = 0.0;
            cost += addObstacleAvoidanceCost(position, grad_position);
            cost += addVelocityBoundCost(velocity, grad_velocity);
            cost += addAccelerationBoundCost(acceleration, grad_acceleration);
            if (cfg_ != nullptr) {
                if (cfg_->penna_thr > 0.0 && cfg_->max_acc_thr > 0.0) {
                    const Eigen::Vector3d force = acceleration + Eigen::Vector3d(0, 0, 9.81);
                    const double thrust = force.norm();
                    double gradient = 0.0;
                    cost += cost_functional::accumulateThrustBandPenalty(
                        thrust, cfg_->min_acc_thr, cfg_->max_acc_thr,
                        cfg_->smooth_eps, cfg_->penna_thr, gradient);
                    if (thrust > 1.e-9) grad_acceleration += gradient * force / thrust;
                }
                cost += cost_functional::accumulateSquaredNormBoundPenalty(
                    jerk, cfg_->max_jerk * cfg_->max_jerk,
                    cfg_->smooth_eps, cfg_->penna_jerk, grad_jerk);
                // Tilt cone: ||a_xy|| <= (g+a_z) tan(max_tilt).
                const double horizontal = acceleration.head<2>().norm();
                const double tangent = std::tan(cfg_->max_tilt);
                const double violation = horizontal - (9.81 + acceleration.z()) * tangent;
                if (violation > 0.0 && cfg_->max_tilt > 0.0) {
                    const double weight = std::max(0.0, cfg_->penna_acc);
                    cost += weight * violation * violation;
                    if (horizontal > 1.e-9)
                        grad_acceleration.head<2>() += 2.0 * weight * violation *
                                                       acceleration.head<2>() / horizontal;
                    grad_acceleration.z() -= 2.0 * weight * violation * tangent;
                }
            }
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
            cost += addESDFVisibilityCost(position,
                                          target,
                                          grad_position,
                                          grad_target);
            cost += addTrackingVelocityCost(position,
                                            velocity,
                                            target,
                                            grad_position,
                                            grad_velocity,
                                            grad_time);
            cost += addVisibleRegionCost(t_global,
                                         position,
                                         target,
                                         grad_position,
                                         grad_time);

            return cost;
        }

    double evaluateAttitudeIntegral(double t_global,
                                    const Eigen::Vector3d &position,
                                    const Eigen::Vector3d &acceleration,
                                    const Eigen::Vector3d &jerk, double yaw, double yaw_rate,
                                    Eigen::Vector3d &grad_position,
                                    Eigen::Vector3d &grad_acceleration,
                                    Eigen::Vector3d &grad_jerk,
                                    double &grad_yaw, double &grad_yaw_rate,
                                    double &grad_time) const {
        if (cfg_ == nullptr) return 0.0;
        const geometry_utils::TrackingAttitude frame(acceleration, yaw);
        if (!frame.valid) return std::numeric_limits<double>::infinity();
        double cost = 0.0;
        if (cfg_->max_omg > 0.0 && cfg_->penna_omg > 0.0) {
            Eigen::Vector3d ga = Eigen::Vector3d::Zero(), gj = Eigen::Vector3d::Zero();
            double gy = 0.0, gyr = 0.0;
            const double rate2 = frame.bodyRateSquared(jerk, yaw_rate, &ga, &gj, &gy, &gyr);
            const double violation = rate2 - cfg_->max_omg * cfg_->max_omg;
            if (violation > 0.0) {
                cost += cfg_->penna_omg * violation * violation;
                const double scale = 2.0 * cfg_->penna_omg * violation;
                grad_acceleration += scale * ga;
                grad_jerk += scale * gj;
                grad_yaw += scale * gy;
                grad_yaw_rate += scale * gyr;
            }
        }
        if (problem_.target_prediction.empty() || problem_.weight_fov <= 0.0) return cost;
        const auto target = interpolateTarget(t_global);
        const double half_h = std::clamp(0.5 * problem_.fov_horizontal -
            problem_.visibility_angle_clearance, 0.01, 1.56);
        const double half_v = std::clamp(0.5 * problem_.fov_vertical -
            problem_.visibility_angle_clearance, 0.01, 1.56);
        const double tan_h = std::tan(half_h), tan_v = std::tan(half_v);
        const double hh = problem_.target_half_height, hw = problem_.target_half_width;
        const std::array<Eigen::Vector3d,7> offsets{{
            {0,0,0}, {0,0,-hh}, {0,0,hh}, {hw,0,0}, {-hw,0,0}, {0,hw,0}, {0,-hw,0}}};
        Eigen::Matrix3d grad_rotation = Eigen::Matrix3d::Zero();
        Eigen::Vector3d grad_target = Eigen::Vector3d::Zero();
        for (const auto &offset : offsets) {
            const Eigen::Vector3d delta = target.position + offset - position;
            const Eigen::Vector3d optical = problem_.camera_rotation.transpose() *
                (frame.rotation.transpose() * delta - problem_.camera_translation);
            Eigen::Vector3d go = Eigen::Vector3d::Zero();
            const auto plane = [&](double violation, const Eigen::Vector3d &normal) {
                if (violation > 0.0) {
                    cost += problem_.weight_fov * violation * violation;
                    go += (2.0 * problem_.weight_fov * violation) * normal;
                }
            };
            // Both sides rather than abs(): a point behind the camera must
            // still receive a useful lateral and forward gradient.
            plane(optical.x() - tan_h * optical.z(), {1,0,-tan_h});
            plane(-optical.x() - tan_h * optical.z(), {-1,0,-tan_h});
            plane(optical.y() - tan_v * optical.z(), {0,1,-tan_v});
            plane(-optical.y() - tan_v * optical.z(), {0,-1,-tan_v});
            plane(problem_.fov_front_margin - optical.z(), {0,0,-1});
            // Range is handled by the observation-distance objective and
            // commit's explicit range grace; it must not block reacquisition.
            const Eigen::Vector3d gb = problem_.camera_rotation * go;
            const Eigen::Vector3d gw = frame.rotation * gb;
            grad_target += gw;
            grad_position -= gw;
            grad_rotation.noalias() += delta * gb.transpose();
        }
        frame.rotationGradient(grad_rotation, grad_acceleration, grad_yaw);
        grad_time += grad_target.dot(traj_opt::trackingTargetPositionDerivative(
            problem_.target_prediction, t_global));
        return cost;
    }

private:
        void rebuildJointSampleTimes()
        {
            joint_sample_times_.clear();
            joint_sample_times_.insert(joint_sample_times_.end(),
                                       problem_.target_sample_times.begin(),
                                       problem_.target_sample_times.end());
            if (problem_.dense_joint_sample_enable &&
                problem_.joint_sample_dt > 0.0 &&
                !problem_.target_prediction.empty())
            {
                const double end_t = std::max(0.0, problem_.target_prediction.back().t);
                const double dt = std::max(0.01, problem_.joint_sample_dt);
                for (double t = 0.0; t <= end_t + 1.0e-6; t += dt)
                {
                    joint_sample_times_.push_back(std::min(t, end_t));
                }
                joint_sample_times_.push_back(0.0);
                joint_sample_times_.push_back(end_t);
            }
            std::sort(joint_sample_times_.begin(), joint_sample_times_.end());
            std::vector<double> unique_times;
            unique_times.reserve(joint_sample_times_.size());
            for (const double t : joint_sample_times_)
            {
                if (!std::isfinite(t))
                {
                    continue;
                }
                if (unique_times.empty() || std::abs(t - unique_times.back()) > 1.0e-4)
                {
                    unique_times.push_back(t);
                }
            }
            joint_sample_times_ = std::move(unique_times);
        }

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

        if (problem_.adaptive_occlusion_enable)
        {
            const auto occlusion = evaluateOcclusionStatus(position, target.position);
            if (occlusion.evaluated && occlusion.activation > 1.0e-6)
            {
                const double scaled_upper =
                    std::max(problem_.adaptive_occlusion_min_horizontal_upper,
                             problem_.tracking_distance *
                                 std::clamp(problem_.adaptive_occlusion_distance_upper_scale,
                                            0.1,
                                            1.0));
                const double elastic_upper =
                    std::max(config.horizontal_lower + 0.05,
                             std::min(config.horizontal_upper, scaled_upper));
                config.horizontal_upper +=
                    occlusion.activation * (elastic_upper - config.horizontal_upper);
                config.weight_far *=
                    1.0 + occlusion.activation *
                              (std::max(1.0,
                                        problem_.adaptive_occlusion_od_far_weight_scale) -
                               1.0);
            }
        }

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

    double addESDFVisibilityCost(const Eigen::Vector3d &position,
                                 const traj_opt::DynamicTargetState &target,
                                 Eigen::Vector3d &grad_position,
                                 Eigen::Vector3d &grad_target) const
    {
        double weight = problem_.weight_oe > 0.0 ? problem_.weight_oe : problem_.weight_visibility;
        if (!problem_.use_esdf_visibility ||
            weight <= 0.0 ||
            problem_.visibility_samples <= 0 ||
            map_manager_ == nullptr ||
            !map_manager_->hasESDF())
        {
            return 0.0;
        }

        if (problem_.adaptive_occlusion_enable)
        {
            const auto occlusion = evaluateOcclusionStatus(position, target.position);
            if (occlusion.evaluated && occlusion.activation > 1.0e-6)
            {
                weight *=
                    1.0 + occlusion.activation *
                              (std::max(1.0,
                                        problem_.adaptive_occlusion_max_weight_scale) -
                               1.0);
            }
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

    cost_functional::TrackingLineOfSightOcclusionStatus evaluateOcclusionStatus(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &target_position) const
    {
        if (map_manager_ == nullptr || !map_manager_->hasESDF())
        {
            return {};
        }
        return cost_functional::evaluateBallLineOfSightOcclusionStatus(
            map_manager_.get(),
            position,
            target_position,
            problem_.visibility_safe_distance,
            problem_.visibility_cone_ratio,
            problem_.adaptive_occlusion_activation_distance,
            problem_.visibility_samples);
    }



    double addTrackingVelocityCost(const Eigen::Vector3d &position,
                                   const Eigen::Vector3d &velocity,
                                   const traj_opt::DynamicTargetState &target,
                                   Eigen::Vector3d &grad_position,
                                   Eigen::Vector3d &grad_velocity,
                                   double &grad_time) const
    {
        const Eigen::Vector2d delta = target.position.head<2>() - position.head<2>();
        const double distance = delta.norm();
        Eigen::Vector3d reference = target.velocity;
        Eigen::Matrix2d derivative = Eigen::Matrix2d::Zero();
        Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
        if (distance > 1.e-6) {
            direction = delta / distance;
            const double cap = std::max(0.01, problem_.max_closing_speed);
            const double gain = std::max(0.0, problem_.closing_gain);
            const double u = std::tanh(gain * (distance - problem_.tracking_distance) / cap);
            const double closing = cap * u;
            reference.head<2>() += closing * direction;
            derivative = (closing / distance) * (Eigen::Matrix2d::Identity() - direction * direction.transpose())
                       + gain * (1.0 - u * u) * direction * direction.transpose();
        }
        const Eigen::Vector3d error = velocity - reference;
        const double weight = std::max(0.0, problem_.weight_relative_velocity);
        double cost = 0.5 * weight * error.squaredNorm();
        Eigen::Vector3d gv = weight * error;
        Eigen::Vector3d gp = Eigen::Vector3d::Zero();
        gp.head<2>() = derivative.transpose() * gv.head<2>();
        // The tangential direction also depends on position. Its derivative
        // must participate in the shared-time objective's coefficient adjoint.
        if (distance > 1.e-6 && problem_.weight_tangent_velocity > 0.0) {
            const Eigen::Vector2d tangent(-direction.y(), direction.x());
            const Eigen::Vector2d relative = (velocity - target.velocity).head<2>();
            const double lateral = relative.dot(tangent);
            const double scale = problem_.weight_tangent_velocity * lateral;
            cost += 0.5 * problem_.weight_tangent_velocity * lateral * lateral;
            gv.head<2>() += scale * tangent;
            Eigen::Matrix2d rotate; rotate << 0, -1, 1, 0;
            gp.head<2>() -= scale / distance *
                (Eigen::Matrix2d::Identity() - direction * direction.transpose()) * rotate.transpose() * relative;
        }
        grad_position += gp;
        grad_velocity += gv;
        grad_time -= gp.dot(target.velocity) + gv.dot(target.acceleration);
        return cost;
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
        return traj_opt::sampleTrackingTarget(problem_.target_prediction, t);
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
    std::vector<double> joint_sample_times_;
};

} // namespace cost_functional_manager
