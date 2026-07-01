#ifndef BACKUP_INTEGRAL_COST_MANAGER_HPP
#define BACKUP_INTEGRAL_COST_MANAGER_HPP

#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/angular_rate_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/flatness_state.hpp"
#include "traj_opt/costfunctional/spatialcosts/jerk_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/polytope_position_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "utils/geometry/quadrotor_flatness.hpp"
#include "utils/header/type_utils.hpp"

namespace cost_functional_manager
{
class BackupIntegralCostManager
{
public:
    const general_utils::PolyhedronH *h_polys = nullptr;
    double smooth_eps = 0.0;
    general_utils::VecDf magnitude_bounds;
    general_utils::VecDf penalty_weights;
    flatness::FlatnessMap *quadrotor_flatness = nullptr;

    void reset(const general_utils::PolyhedronH *h_polys,
               double smooth_eps,
               const general_utils::VecDf &magnitude_bounds,
               const general_utils::VecDf &penalty_weights,
               flatness::FlatnessMap *quadrotor_flatness)
    {
        this->h_polys = h_polys;
        this->smooth_eps = smooth_eps;
        this->magnitude_bounds = magnitude_bounds;
        this->penalty_weights = penalty_weights;
        this->quadrotor_flatness = quadrotor_flatness;
    }

    void beginEvaluation()
    {   
        max_violation_.resize(8);
        max_violation_.setZero();
    }

    const general_utils::VecDf &getPenaltyLog() const
    {
        return max_violation_;
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
        return cost;
    }

    template <typename SampleBuffer>
    double evaluateSample(const SampleBuffer &,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &,
                          Eigen::VectorXd &) const
    {
        return 0.0;
    }

    double operator()(double /*t*/, double /*t_global*/, int /*seg_idx*/, int /*step_in_seg*/,
                      const Eigen::Vector3d &position,
                      const Eigen::Vector3d &velocity,
                      const Eigen::Vector3d &acceleration,
                      const Eigen::Vector3d &jerk,
                      const Eigen::Vector3d &/*snap*/,
                      Eigen::Vector3d &grad_position,
                      Eigen::Vector3d &grad_velocity,
                      Eigen::Vector3d &grad_acceleration,
                      Eigen::Vector3d &grad_jerk,
                      Eigen::Vector3d &/*gs*/,
                      double & /*grad_time*/) const
    {
        if(!h_polys || !quadrotor_flatness)
        {
            return 0.0;
        }

        const double weight_pos = penalty_weights[0];
        const double weight_vel = penalty_weights[1];
        const double weight_acc = penalty_weights[2];
        const double weight_jer = penalty_weights[3];
        const double weight_att = penalty_weights[4];
        const double weight_omg = penalty_weights[5];
        const double weight_acc_thr = penalty_weights[6];

        double local_cost = 0.0;
        Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_jer = Eigen::Vector3d::Zero();

        local_cost += cost_functional::accumulatePolytopePositionPenalty(*h_polys, position, smooth_eps, weight_pos, grad_pos, &max_violation_[1]);
        local_cost += cost_functional::accumulateVelocityBoundPenalty(velocity, magnitude_bounds[0] * magnitude_bounds[0], smooth_eps, weight_vel, grad_vel, &max_violation_[2]);
        local_cost += cost_functional::accumulateAccelerationBoundPenalty(acceleration, magnitude_bounds[1] * magnitude_bounds[1], smooth_eps, weight_acc, grad_acc, &max_violation_[3]);
        local_cost += cost_functional::accumulateJerkBoundPenalty(jerk, magnitude_bounds[2] * magnitude_bounds[2], smooth_eps, weight_jer, grad_jer,&max_violation_[4]);

        Eigen::Vector3d total_grad_pos = grad_pos;
        Eigen::Vector3d total_grad_vel = grad_vel;
        Eigen::Vector3d total_grad_acc = grad_acc;  
        Eigen::Vector3d total_grad_jer = grad_jer;

        if(weight_omg > 0.0 || weight_acc_thr > 0.0)
        {
            const auto flatness = cost_functional::evaluateFlatnessPenaltyState(quadrotor_flatness,velocity,acceleration,jerk);
            Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
            double grad_acc_thr = 0.0;

            local_cost += cost_functional::accumulateAngularRateBoundPenalty(flatness.angular_rate, magnitude_bounds[3] * magnitude_bounds[3], smooth_eps, weight_omg, grad_omg, &max_violation_[6]);
            local_cost += cost_functional::accumulateThrustBandPenalty(flatness.thrust, magnitude_bounds[4], magnitude_bounds[5], smooth_eps, weight_acc_thr, grad_acc_thr, &max_violation_[7]);

            double total_grad_psi = 0.0;
            double total_grad_psi_d = 0.0;

            quadrotor_flatness->backward(grad_pos, grad_vel, grad_acc, grad_jer, grad_acc_thr, general_utils::Vec4f::Zero(), grad_omg,total_grad_pos, total_grad_vel, total_grad_acc, total_grad_jer, total_grad_psi, total_grad_psi_d);
        }

        grad_position += total_grad_pos;
        grad_velocity += total_grad_vel;
        grad_acceleration += total_grad_acc;
        grad_jerk += total_grad_jer;
        
        return local_cost;
    }
    private:
        mutable general_utils::VecDf max_violation_{general_utils::VecDf::Zero(8)};
};
} // namespace cost_functional_manager

#endif // BACKUP_INTEGRAL_COST_MANAGER_HPP
