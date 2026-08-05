#ifndef EXP_INTEGRAL_COST_MANAGER_HPP
#define EXP_INTEGRAL_COST_MANAGER_HPP

#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/angular_rate_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/flatness_state.hpp"
#include "traj_opt/costfunctional/spatialcosts/guide_path_consistency_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/jerk_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/polytope_position_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/segment_boundary_attractor_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/swarm_collision_penalty.hpp"
#include "utils/geometry/quadrotor_flatness.hpp"
#include "utils/header/type_utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cost_functional_manager
{
    class ExpIntegralCostManager
    {
    public:
        const general_utils::PolyhedraH *h_polys = nullptr;
        const Eigen::VectorXi *h_poly_idx = nullptr;
        const general_utils::Mat3Df *waypoint_attractors = nullptr;
        const general_utils::VecDf *waypoint_attractor_dead_d = nullptr;
        double smooth_eps = 0.0;
        general_utils::VecDf magnitude_bounds;
        general_utils::VecDf penalty_weights;
        flatness::FlatnessMap *quadrotor_flatness = nullptr; 
        traj_opt::SwarmPenaltyConfig swarm_config;
        traj_opt::SwarmTrajectoriesConstPtr swarm_trajs;
        double swarm_current_wall_time{0.0};
        const general_utils::vec_E<general_utils::Vec3f> *guide_path = nullptr;
        const std::vector<double> *guide_t = nullptr;
        double weight_guide_integral{0.0};
        double guide_path_tube_radius{0.0};
        double guide_path_z_tube_radius{0.0};
        double guide_path_huber_delta{0.0};
        bool guide_path_time_gradient_en{false};
        double weight_guide_z_lower{0.0};
        double guide_z_lower_tolerance{0.0};
        double guide_z_floor_reference{std::numeric_limits<double>::quiet_NaN()};

        void reset(const general_utils::PolyhedraH *h_polys_in,
                   const Eigen::VectorXi *h_poly_idx_in,
                   const general_utils::Mat3Df *waypoint_attractors_in,
                   const general_utils::VecDf *waypoint_attractor_dead_d_in,
                   double smooth_eps_in,
                   const general_utils::VecDf &magnitude_bounds_in,
                   const general_utils::VecDf &penalty_weights_in,
                   flatness::FlatnessMap *quadrotor_flatness_in,
                   const traj_opt::SwarmPenaltyConfig &swarm_config_in,
                   const traj_opt::SwarmTrajectoriesConstPtr &swarm_trajs_in,
                   double swarm_current_wall_time_in,
                   const general_utils::vec_E<general_utils::Vec3f> *guide_path_in = nullptr,
                   const std::vector<double> *guide_t_in = nullptr,
                   double weight_guide_integral_in = 0.0,
                   double guide_path_tube_radius_in = 0.0,
                   double guide_path_z_tube_radius_in = 0.0,
                   double guide_path_huber_delta_in = 0.0,
                   bool guide_path_time_gradient_en_in = false,
                   double weight_guide_z_lower_in = 0.0,
                   double guide_z_lower_tolerance_in = 0.0,
                   double guide_z_floor_reference_in = std::numeric_limits<double>::quiet_NaN())
        {
            h_polys = h_polys_in;
            h_poly_idx = h_poly_idx_in;
            waypoint_attractors = waypoint_attractors_in;
            waypoint_attractor_dead_d = waypoint_attractor_dead_d_in;
            smooth_eps = smooth_eps_in;
            magnitude_bounds = magnitude_bounds_in;
            penalty_weights = penalty_weights_in;
            quadrotor_flatness = quadrotor_flatness_in;
            swarm_config = swarm_config_in;
            swarm_trajs = swarm_trajs_in;
            swarm_current_wall_time = swarm_current_wall_time_in;
            guide_path = guide_path_in;
            guide_t = guide_t_in;
            weight_guide_integral = std::max(0.0, weight_guide_integral_in);
            guide_path_tube_radius = std::max(0.0, guide_path_tube_radius_in);
            guide_path_z_tube_radius = std::max(0.0, guide_path_z_tube_radius_in);
            guide_path_huber_delta = std::max(0.0, guide_path_huber_delta_in);
            guide_path_time_gradient_en = guide_path_time_gradient_en_in;
            weight_guide_z_lower = std::max(0.0, weight_guide_z_lower_in);
            guide_z_lower_tolerance = std::max(0.0, guide_z_lower_tolerance_in);
            guide_z_floor_reference = guide_z_floor_reference_in;
        }

        void beginEvaluation(const std::vector<double> *times)
        {
            segment_times_ = times;
            max_violation_.resize(9);
            max_violation_.setZero();    
            guide_integral_violation_ = 0.0;
            guide_cost_log_ = 0.0;
            guide_max_abs_time_grad_ = 0.0;
            guide_out_of_time_range_samples_ = 0;
            guide_z_lower_violation_ = 0.0;
        }

        const general_utils::VecDf &getPenaltyLog() const { return max_violation_; }
        double guideIntegralViolation() const { return guide_integral_violation_; }
        double guideCostLog() const { return guide_cost_log_; }
        double guideMaxAbsTimeGrad() const { return guide_max_abs_time_grad_; }
        int guideOutOfTimeRangeSamples() const { return guide_out_of_time_range_samples_; }
        double guideZLowerViolation() const { return guide_z_lower_violation_; }
        bool usesSampleCost() const { return false; }

        double evaluateIntegral(int logical_idx,
                                double t,
                                double t_global,
                                int seg_idx,
                                int step_in_seg,
                                const Eigen::Vector3d &p,
                                const Eigen::Vector3d &v,
                                const Eigen::Vector3d &a,
                                const Eigen::Vector3d &j,
                                Eigen::Vector3d &gp,
                                Eigen::Vector3d &gv,
                                Eigen::Vector3d &ga,
                                Eigen::Vector3d &gj,
                                double &gt) const
        {
            (void)logical_idx;
            Eigen::Vector3d gs = Eigen::Vector3d::Zero();
            const Eigen::Vector3d snap = Eigen::Vector3d::Zero();
            double cost = operator()(t, t_global, seg_idx, step_in_seg,
                                     p, v, a, j, snap,
                                     gp, gv, ga, gj, gs, gt);
            return cost;
        }

        template <typename SampleBuffer>
        double evaluateSample(const SampleBuffer &,
                              Eigen::Matrix<double, 3, Eigen::Dynamic> &,
                              Eigen::VectorXd &) const
        {
            return 0.0;
        }

        double operator()(double t,
                        double t_global,
                        int seg_idx,
                        int /*step_in_seg*/,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        const Eigen::Vector3d &a,
                        const Eigen::Vector3d &j,
                        const Eigen::Vector3d & /*s*/,
                        Eigen::Vector3d &gp,
                        Eigen::Vector3d &gv,
                        Eigen::Vector3d &ga,
                        Eigen::Vector3d &gj,
                        Eigen::Vector3d & /*gs*/,
                        double &gt) const
        {
            if (!h_polys || !h_poly_idx || !quadrotor_flatness)
            {
                return 0.0;
            }

            const double weight_pos = penalty_weights(0);
            const double weight_vel = penalty_weights(1);
            const double weight_acc = penalty_weights(2);
            const double weight_jer = penalty_weights(3);
            const double weight_att = penalty_weights(4);
            const double weight_omg = penalty_weights(5);
            const double weight_acc_thr = penalty_weights(6);

            double local_cost = 0.0;
            Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
            Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
            Eigen::Vector3d grad_jer = Eigen::Vector3d::Zero();

            const int poly_id = (*h_poly_idx)(seg_idx);
            local_cost += cost_functional::accumulatePolytopePositionPenalty((*h_polys)[poly_id],
                                                                                p,
                                                                                smooth_eps,
                                                                                weight_pos,
                                                                                grad_pos,
                                                                                &max_violation_(1));
            local_cost += cost_functional::accumulateSegmentBoundaryAttractorPenalty(t,
                                                                                        seg_idx,
                                                                                        segment_times_,
                                                                                        waypoint_attractors,
                                                                                        waypoint_attractor_dead_d,
                                                                                        p,
                                                                                        smooth_eps,
                                                                                        weight_att,
                                                                                        grad_pos,
                                                                                        &max_violation_(5));
            local_cost += cost_functional::accumulateGuidePathConsistencyPenalty(guide_path,
                                                                                 guide_t,
                                                                                 t_global,
                                                                                 p,
                                                                                 weight_guide_integral,
                                                                                 guide_path_tube_radius,
                                                                                 guide_path_z_tube_radius,
                                                                                 guide_path_huber_delta,
                                                                                 guide_path_time_gradient_en,
                                                                                 grad_pos,
                                                                                 gt,
                                                                                 weight_guide_z_lower,
                                                                                 guide_z_lower_tolerance,
                                                                                 &guide_integral_violation_,
                                                                                 &guide_cost_log_,
                                                                                 &guide_max_abs_time_grad_,
                                                                                 &guide_out_of_time_range_samples_,
                                                                                 &guide_z_lower_violation_,
                                                                                 guide_z_floor_reference);
            local_cost += cost_functional::accumulateVelocityBoundPenalty(v,
                                                                            magnitude_bounds(0) * magnitude_bounds(0),
                                                                            smooth_eps,
                                                                            weight_vel,
                                                                            grad_vel,
                                                                            &max_violation_(2));
            local_cost += cost_functional::accumulateAccelerationBoundPenalty(a,
                                                                                magnitude_bounds(1) * magnitude_bounds(1),
                                                                                smooth_eps,
                                                                                weight_acc,
                                                                                grad_acc,
                                                                                &max_violation_(3));
            local_cost += cost_functional::accumulateJerkBoundPenalty(j,
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
                                                                            p,
                                                                            v,
                                                                            grad_pos,
                                                                            gt,
                                                                            &max_violation_(8));
            local_cost += cost_functional::accumulateSwarmFormationPenalty(swarm_config,
                                                                            swarm_trajs.get(),
                                                                            swarm_current_wall_time,
                                                                            t_global,
                                                                            p,
                                                                            v,
                                                                            grad_pos,
                                                                            gt);
            total_grad_pos = grad_pos;

            if (weight_omg > 0.0 || weight_acc_thr > 0.0)
            {
                const auto flatness_state =
                    cost_functional::evaluateFlatnessPenaltyState(quadrotor_flatness, v, a, j);
                Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
                double grad_thr = 0.0;

                local_cost += cost_functional::accumulateAngularRateBoundPenalty(flatness_state.angular_rate,
                                                                                    magnitude_bounds(3) * magnitude_bounds(3),
                                                                                    smooth_eps,
                                                                                    weight_omg,
                                                                                    grad_omg,
                                                                                    &max_violation_(6));
                local_cost += cost_functional::accumulateThrustBandPenalty(flatness_state.thrust,
                                                                            magnitude_bounds(4),
                                                                            magnitude_bounds(5),
                                                                            smooth_eps,
                                                                            weight_acc_thr,
                                                                            grad_thr,
                                                                            &max_violation_(7));

                // Forward evaluation is required to check the bounds, but the
                // considerably more expensive reverse map is an exact no-op
                // when neither flatness penalty is active at this sample.
                if (grad_omg.squaredNorm() != 0.0 || grad_thr != 0.0)
                {
                    double total_grad_psi = 0.0;
                    double total_grad_psid = 0.0;
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
                                    total_grad_psid);
                }
            }

            gp += total_grad_pos;
            gv += total_grad_vel;
            ga += total_grad_acc;
            gj += total_grad_jer;
            return local_cost;
        }

    private:
        mutable const std::vector<double> *segment_times_ = nullptr;
        mutable general_utils::VecDf max_violation_{general_utils::VecDf::Zero(9)};
        mutable double guide_integral_violation_{0.0};
        mutable double guide_cost_log_{0.0};
        mutable double guide_max_abs_time_grad_{0.0};
        mutable int guide_out_of_time_range_samples_{0};
        mutable double guide_z_lower_violation_{0.0};
    };

}//namespace cost_functional_manager

#endif
