#pragma once

#include <map_manager/map_manager.hpp>
#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/angular_rate_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/flatness_state.hpp"
#include "traj_opt/costfunctional/spatialcosts/jerk_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/swarm_collision_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "utils/geometry/quadrotor_flatness.hpp"
#include "utils/header/type_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cost_functional_manager
{
class ESDFIntegralCostManager
{
public:
    const general_planner::MapManager *map = nullptr;
    double safe_distance = 0.0;
    double smooth_eps = 0.0;
    double weight = 0.0;
    general_utils::VecDf magnitude_bounds;
    general_utils::VecDf penalty_weights;
    flatness::FlatnessMap *quadrotor_flatness = nullptr;
    traj_opt::SwarmPenaltyConfig swarm_config;
    traj_opt::SwarmTrajectoriesConstPtr swarm_trajs;
    double swarm_current_wall_time{0.0};
    const general_utils::vec_E<general_utils::Vec3f> *guide_path = nullptr;
    const general_utils::vec_E<general_utils::Vec3f> *guide_velocities = nullptr;
    const general_utils::Mat3Df *guide_points = nullptr;
    double weight_guide{0.0};
    double weight_guide_integral{0.0};
    double weight_guide_vel_integral{0.0};
    int junction_sample_step{0};

    void reset(const general_planner::MapManager *map_in,
               const double safe_distance_in,
               const double smooth_eps_in,
               const double weight_in,
               const general_utils::VecDf &magnitude_bounds_in,
               const general_utils::VecDf &penalty_weights_in,
               flatness::FlatnessMap *quadrotor_flatness_in,
               const traj_opt::SwarmPenaltyConfig &swarm_config_in,
               const traj_opt::SwarmTrajectoriesConstPtr &swarm_trajs_in,
               double swarm_current_wall_time_in,
               const general_utils::vec_E<general_utils::Vec3f> *guide_path_in = nullptr,
               const general_utils::vec_E<general_utils::Vec3f> *guide_velocities_in = nullptr,
               const general_utils::Mat3Df *guide_points_in = nullptr,
               double weight_guide_in = 0.0,
               double weight_guide_integral_in = 0.0,
               double weight_guide_vel_integral_in = 0.0,
               int junction_sample_step_in = 0)
    {
        map = map_in;
        safe_distance = safe_distance_in;
        smooth_eps = smooth_eps_in;
        weight = weight_in;
        magnitude_bounds = magnitude_bounds_in;
        penalty_weights = penalty_weights_in;
        quadrotor_flatness = quadrotor_flatness_in;
        swarm_config = swarm_config_in;
        swarm_trajs = swarm_trajs_in;
        swarm_current_wall_time = swarm_current_wall_time_in;
        guide_path = guide_path_in;
        guide_velocities = guide_velocities_in;
        guide_points = guide_points_in;
        weight_guide = std::max(0.0, weight_guide_in);
        weight_guide_integral = std::max(0.0, weight_guide_integral_in);
        weight_guide_vel_integral = std::max(0.0, weight_guide_vel_integral_in);
        junction_sample_step = std::max(0, junction_sample_step_in);
    }

    void beginEvaluation(const std::vector<double> *)
    {
        max_violation_.resize(9);
        max_violation_.setZero();
        guide_cost_log_ = 0.0;
    }

    double evaluateIntegral(int logical_idx,
                            double t,
                            double t_global,
                            int seg_idx,
                            int step_in_seg,
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
        (void)logical_idx;
        Eigen::Vector3d grad_snap = Eigen::Vector3d::Zero();
        const Eigen::Vector3d snap = Eigen::Vector3d::Zero();
        double cost = operator()(t,
                                 t_global,
                                 seg_idx,
                                 step_in_seg,
                                 position,
                                 velocity,
                                 acceleration,
                                 jerk,
                                 snap,
                                 grad_position,
                                 grad_velocity,
                                 grad_acceleration,
                                 grad_jerk,
                                 grad_snap,
                                 grad_time);

        const bool use_guide_position =
            weight_guide_integral > 0.0 &&
            guide_path != nullptr &&
            guide_path->size() >= 2;
        const bool use_guide_velocity =
            weight_guide_vel_integral > 0.0 &&
            guide_path != nullptr &&
            guide_velocities != nullptr &&
            guide_path->size() >= 2 &&
            guide_velocities->size() == guide_path->size();

        if (use_guide_position || use_guide_velocity)
        {
            const ClosestGuidePV ref = closestPVOnPolyline(*guide_path,
                                                           guide_velocities != nullptr ? *guide_velocities : empty_velocities_,
                                                           position);
            if (use_guide_position)
            {
                const Eigen::Vector3d diff = position - ref.position;
                const double guide_sample_cost = 0.5 * weight_guide_integral * diff.squaredNorm();
                cost += guide_sample_cost;
                grad_position += weight_guide_integral * diff;
                guide_cost_log_ += guide_sample_cost;
            }
            if (use_guide_velocity)
            {
                const Eigen::Vector3d diff_v = velocity - ref.velocity;
                const double guide_velocity_sample_cost =
                    0.5 * weight_guide_vel_integral * diff_v.squaredNorm();
                cost += guide_velocity_sample_cost;
                grad_velocity += weight_guide_vel_integral * diff_v;
                guide_cost_log_ += guide_velocity_sample_cost;
            }
        }
        return cost;
    }

    template <typename SampleBuffer>
    double evaluateSample(const SampleBuffer &samples,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_positions,
                          Eigen::VectorXd &) const
    {
        if (weight_guide <= 0.0 ||
            guide_points == nullptr ||
            guide_points->cols() <= 0)
        {
            return 0.0;
        }

        double cost = 0.0;
        for (std::size_t sample_id = 0; sample_id < samples.size(); ++sample_id)
        {
            const auto &sample = samples[sample_id];
            if (sample.step_in_seg != junction_sample_step ||
                sample.seg_idx < 0 ||
                sample.seg_idx >= guide_points->cols())
            {
                continue;
            }

            const Eigen::Vector3d diff = sample.p - guide_points->col(sample.seg_idx);
            const double sample_cost = 0.5 * weight_guide * diff.squaredNorm();
            cost += sample_cost;
            guide_cost_log_ += sample_cost;
            grad_positions.col(static_cast<Eigen::Index>(sample_id)) += weight_guide * diff;
        }
        return cost;
    }

    double operator()(double /*t*/, double t_global, int /*seg_idx*/, int /*step_in_seg*/,
                      const Eigen::Vector3d &position,
                      const Eigen::Vector3d &velocity,
                      const Eigen::Vector3d &acceleration,
                      const Eigen::Vector3d &jerk,
                      const Eigen::Vector3d &/*snap*/,
                      Eigen::Vector3d &grad_position,
                      Eigen::Vector3d &grad_velocity,
                      Eigen::Vector3d &grad_acceleration,
                      Eigen::Vector3d &grad_jerk,
                      Eigen::Vector3d &/*grad_snap*/,
                      double &grad_time) const
    {
        if (map == nullptr || !quadrotor_flatness || magnitude_bounds.size() < 6 || penalty_weights.size() < 7)
        {
            return 0.0;
        }

        const double weight_vel = penalty_weights(1);
        const double weight_acc = penalty_weights(2);
        const double weight_jer = penalty_weights(3);
        const double weight_omg = penalty_weights(5);
        const double weight_acc_thr = penalty_weights(6);

        double local_cost = 0.0;
        Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_jer = Eigen::Vector3d::Zero();

        local_cost += cost_functional::accumulateESDFDistancePenalty(map,
                                                                     position,
                                                                     safe_distance,
                                                                     smooth_eps,
                                                                     weight,
                                                                     grad_pos,
                                                                     &max_violation_(1));
        local_cost += cost_functional::accumulateVelocityBoundPenalty(velocity,
                                                                      magnitude_bounds(0) * magnitude_bounds(0),
                                                                      smooth_eps,
                                                                      weight_vel,
                                                                      grad_vel,
                                                                      &max_violation_(2));
        local_cost += cost_functional::accumulateAccelerationBoundPenalty(acceleration,
                                                                          magnitude_bounds(1) * magnitude_bounds(1),
                                                                          smooth_eps,
                                                                          weight_acc,
                                                                          grad_acc,
                                                                          &max_violation_(3));
        local_cost += cost_functional::accumulateJerkBoundPenalty(jerk,
                                                                  magnitude_bounds(2) * magnitude_bounds(2),
                                                                  smooth_eps,
                                                                  weight_jer,
                                                                  grad_jer,
                                                                  &max_violation_(4));

        Eigen::Vector3d total_grad_pos = grad_pos;
        Eigen::Vector3d total_grad_vel = grad_vel;
        Eigen::Vector3d total_grad_acc = grad_acc;
        Eigen::Vector3d total_grad_jer = grad_jer;

        local_cost += cost_functional::accumulateSwarmCollisionPenalty(swarm_config,
                                                                       swarm_trajs.get(),
                                                                       swarm_current_wall_time,
                                                                       t_global,
                                                                       position,
                                                                       velocity,
                                                                       grad_pos,
                                                                       grad_time,
                                                                       &max_violation_(8));
        local_cost += cost_functional::accumulateSwarmFormationPenalty(swarm_config,
                                                                       swarm_trajs.get(),
                                                                       swarm_current_wall_time,
                                                                       t_global,
                                                                       position,
                                                                       velocity,
                                                                       grad_pos,
                                                                       grad_time);
        total_grad_pos = grad_pos;

        if (weight_omg > 0.0 || weight_acc_thr > 0.0)
        {
            const auto flatness = cost_functional::evaluateFlatnessPenaltyState(quadrotor_flatness,
                                                                                velocity,
                                                                                acceleration,
                                                                                jerk);
            Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
            double grad_thr = 0.0;

            local_cost += cost_functional::accumulateAngularRateBoundPenalty(flatness.angular_rate,
                                                                             magnitude_bounds(3) * magnitude_bounds(3),
                                                                             smooth_eps,
                                                                             weight_omg,
                                                                             grad_omg,
                                                                             &max_violation_(6));
            local_cost += cost_functional::accumulateThrustBandPenalty(flatness.thrust,
                                                                       magnitude_bounds(4),
                                                                       magnitude_bounds(5),
                                                                       smooth_eps,
                                                                       weight_acc_thr,
                                                                       grad_thr,
                                                                       &max_violation_(7));

            double total_grad_psi = 0.0;
            double total_grad_psi_d = 0.0;
            quadrotor_flatness->backward(grad_pos,
                                         grad_vel,
                                         grad_acc,
                                         grad_jer,
                                         grad_thr,
                                         general_utils::Vec4f::Zero(),
                                         grad_omg,
                                         total_grad_pos,
                                         total_grad_vel,
                                         total_grad_acc,
                                         total_grad_jer,
                                         total_grad_psi,
                                         total_grad_psi_d);
        }

        grad_position += total_grad_pos;
        grad_velocity += total_grad_vel;
        grad_acceleration += total_grad_acc;
        grad_jerk += total_grad_jer;
        return local_cost;
    }

    double getMaxViolation() const { return max_violation_.size() > 1 ? max_violation_(1) : 0.0; }
    const general_utils::VecDf &getPenaltyLog() const { return max_violation_; }
    double getGuideCostLog() const { return guide_cost_log_; }

private:
    struct ClosestGuidePV
    {
        general_utils::Vec3f position{general_utils::Vec3f::Zero()};
        general_utils::Vec3f velocity{general_utils::Vec3f::Zero()};
    };

    static ClosestGuidePV closestPVOnPolyline(const general_utils::vec_E<general_utils::Vec3f> &path,
                                             const general_utils::vec_E<general_utils::Vec3f> &velocities,
                                             const Eigen::Vector3d &query)
    {
        ClosestGuidePV best;
        if (path.empty())
        {
            best.position = query;
            return best;
        }
        if (path.size() == 1)
        {
            best.position = path.front();
            best.velocity = velocities.size() == path.size() ? velocities.front() : general_utils::Vec3f::Zero();
            return best;
        }

        double best_sq = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
        {
            const Eigen::Vector3d a = path[i];
            const Eigen::Vector3d b = path[i + 1];
            const Eigen::Vector3d ab = b - a;
            const double denom = ab.squaredNorm();
            const double s = denom > 1.0e-9 ? std::clamp((query - a).dot(ab) / denom, 0.0, 1.0) : 0.0;
            const Eigen::Vector3d candidate = a + s * ab;
            const double sq = (query - candidate).squaredNorm();
            if (sq < best_sq)
            {
                best_sq = sq;
                best.position = candidate;
                if (velocities.size() == path.size())
                {
                    best.velocity = (1.0 - s) * velocities[i] + s * velocities[i + 1];
                }
                else
                {
                    best.velocity.setZero();
                }
            }
        }
        return best;
    }

    general_utils::vec_E<general_utils::Vec3f> empty_velocities_;
    mutable general_utils::VecDf max_violation_{general_utils::VecDf::Zero(9)};
    mutable double guide_cost_log_{0.0};
};
} // namespace cost_functional_manager
