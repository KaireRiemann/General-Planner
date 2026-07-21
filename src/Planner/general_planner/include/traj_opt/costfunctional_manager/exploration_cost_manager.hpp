#pragma once

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include "utils/geometry/quadrotor_flatness.hpp"
#include "utils/header/type_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cost_functional_manager
{
class ExplorationCostManager
{
public:
  void reset(const general_utils::PolyhedraH *h_polys,
             const Eigen::VectorXi *h_poly_idx,
             const general_utils::VecDf *piece_velocity_bounds,
             double smooth_eps,
             const general_utils::VecDf &magnitude_bounds,
             const general_utils::VecDf &penalty_weights,
             flatness::FlatnessMap *quadrotor_flatness)
  {
    h_polys_ = h_polys;
    h_poly_idx_ = h_poly_idx;
    piece_velocity_bounds_ = piece_velocity_bounds;
    smooth_eps_ = std::max(1.0e-6, smooth_eps);
    magnitude_bounds_ = magnitude_bounds;
    penalty_weights_ = penalty_weights;
    quadrotor_flatness_ = quadrotor_flatness;
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

  double guideIntegralViolation() const
  {
    return 0.0;
  }

  double guideCostLog() const
  {
    return 0.0;
  }

  double guideMaxAbsTimeGrad() const
  {
    return 0.0;
  }

  int guideOutOfTimeRangeSamples() const
  {
    return 0;
  }

  double evaluateIntegral(int logical_idx,
                          double t_local,
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
    (void)t_local;
    (void)t_global;
    (void)step_in_seg;

    if (!ready() || seg_idx < 0 || seg_idx >= h_poly_idx_->size())
    {
      return 0.0;
    }

    const double vel_max = segmentVelocityBound(seg_idx);
    const double acc_max = boundOrDefault(1, 1.0);
    const double omg_max = boundOrDefault(2, std::numeric_limits<double>::infinity());
    const double theta_max = boundOrDefault(3, std::numeric_limits<double>::infinity());
    const double min_thrust = boundOrDefault(4, -std::numeric_limits<double>::infinity());
    const double max_thrust = boundOrDefault(5, std::numeric_limits<double>::infinity());

    const double weight_pos = weightOrDefault(0);
    const double weight_vel = weightOrDefault(1);
    const double weight_acc = weightOrDefault(2);
    const double weight_omg = weightOrDefault(3);
    const double weight_theta = weightOrDefault(4);
    const double weight_thrust = weightOrDefault(5);

    double thrust = 0.0;
    Eigen::Vector4d quat = Eigen::Vector4d::Zero();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    quadrotor_flatness_->forward(velocity, acceleration, jerk, 0.0, 0.0, thrust, quat, omega);

    double cost = 0.0;
    Eigen::Vector3d flat_grad_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d flat_grad_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d flat_grad_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d flat_grad_jer = Eigen::Vector3d::Zero();
    Eigen::Vector3d direct_grad_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d flat_grad_omega = Eigen::Vector3d::Zero();
    Eigen::Vector4d grad_quat = Eigen::Vector4d::Zero();
    double grad_thrust = 0.0;

    const int h_id = (*h_poly_idx_)(seg_idx);
    if (h_id >= 0 && h_id < static_cast<int>(h_polys_->size()) && weight_pos > 0.0)
    {
      const general_utils::PolyhedronH &poly = (*h_polys_)[h_id];
      for (int i = 0; i < poly.rows(); ++i)
      {
        const Eigen::Vector3d normal = poly.template block<1, 3>(i, 0).transpose();
        const double violation = normal.dot(position) + poly(i, 3);
        max_violation_(1) = std::max(max_violation_(1), violation);
        double value = 0.0;
        double deriv = 0.0;
        if (cost_functional::smoothedL1(violation, smooth_eps_, value, deriv))
        {
          flat_grad_pos += weight_pos * deriv * normal;
          cost += weight_pos * value;
        }
      }
    }

    double value = 0.0;
    double deriv = 0.0;
    const double vel_violation = velocity.squaredNorm() - vel_max * vel_max;
    max_violation_(2) = std::max(max_violation_(2), vel_violation);
    if (weight_vel > 0.0 && cost_functional::smoothedL1(vel_violation, smooth_eps_, value, deriv))
    {
      flat_grad_vel += weight_vel * deriv * 2.0 * velocity;
      cost += weight_vel * value;
    }

    const double acc_violation = acceleration.squaredNorm() - acc_max * acc_max;
    max_violation_(3) = std::max(max_violation_(3), acc_violation);
    if (weight_acc > 0.0 && cost_functional::smoothedL1(acc_violation, smooth_eps_, value, deriv))
    {
      direct_grad_acc += weight_acc * deriv * 2.0 * acceleration;
      cost += weight_acc * value;
    }

    if (std::isfinite(omg_max))
    {
      const double omg_violation = omega.squaredNorm() - omg_max * omg_max;
      max_violation_(6) = std::max(max_violation_(6), omg_violation);
      if (weight_omg > 0.0 && cost_functional::smoothedL1(omg_violation, smooth_eps_, value, deriv))
      {
        flat_grad_omega += weight_omg * deriv * 2.0 * omega;
        cost += weight_omg * value;
      }
    }

    if (std::isfinite(theta_max))
    {
      const double cos_theta_raw = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
      const double cos_theta = std::max(-1.0, std::min(1.0, cos_theta_raw));
      const double theta_violation = std::acos(cos_theta) - theta_max;
      max_violation_(4) = std::max(max_violation_(4), theta_violation);
      if (weight_theta > 0.0 &&
          cost_functional::smoothedL1(theta_violation, smooth_eps_, value, deriv))
      {
        const double denom = std::sqrt(std::max(1.0e-9, 1.0 - cos_theta * cos_theta));
        grad_quat += weight_theta * deriv / denom *
                     4.0 * Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
        cost += weight_theta * value;
      }
    }

    if (std::isfinite(min_thrust) && std::isfinite(max_thrust) && max_thrust > min_thrust)
    {
      const double thrust_mean = 0.5 * (min_thrust + max_thrust);
      const double thrust_radius = 0.5 * std::abs(max_thrust - min_thrust);
      const double thrust_violation =
          (thrust - thrust_mean) * (thrust - thrust_mean) - thrust_radius * thrust_radius;
      max_violation_(7) = std::max(max_violation_(7), thrust_violation);
      if (weight_thrust > 0.0 &&
          cost_functional::smoothedL1(thrust_violation, smooth_eps_, value, deriv))
      {
        grad_thrust += weight_thrust * deriv * 2.0 * (thrust - thrust_mean);
        cost += weight_thrust * value;
      }
    }

    double grad_psi = 0.0;
    double grad_dpsi = 0.0;
    Eigen::Vector3d total_grad_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_grad_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_grad_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_grad_jer = Eigen::Vector3d::Zero();

    quadrotor_flatness_->backward(flat_grad_pos,
                                  flat_grad_vel,
                                  flat_grad_acc,
                                  flat_grad_jer,
                                  grad_thrust,
                                  grad_quat,
                                  flat_grad_omega,
                                  total_grad_pos,
                                  total_grad_vel,
                                  total_grad_acc,
                                  total_grad_jer,
                                  grad_psi,
                                  grad_dpsi);
    total_grad_acc += direct_grad_acc;

    grad_position += total_grad_pos;
    grad_velocity += total_grad_vel;
    grad_acceleration += total_grad_acc;
    grad_jerk += total_grad_jer;
    grad_time = 0.0;

    return cost;
  }

  template <typename SampleBuffer, typename GradPositionMat, typename GradTimeVec>
  double evaluateSample(const SampleBuffer &,
                        GradPositionMat &grad_position,
                        GradTimeVec &grad_time) const
  {
    grad_position.setZero();
    grad_time.setZero();
    return 0.0;
  }

private:
  bool ready() const
  {
    return h_polys_ != nullptr &&
           h_poly_idx_ != nullptr &&
           quadrotor_flatness_ != nullptr &&
           magnitude_bounds_.size() >= 6 &&
           penalty_weights_.size() >= 6;
  }

  double boundOrDefault(int idx, double fallback) const
  {
    if (idx >= 0 &&
        idx < magnitude_bounds_.size() &&
        std::isfinite(magnitude_bounds_(idx)) &&
        magnitude_bounds_(idx) > 0.0)
    {
      return magnitude_bounds_(idx);
    }
    return fallback;
  }

  double weightOrDefault(int idx) const
  {
    if (idx >= 0 && idx < penalty_weights_.size() && std::isfinite(penalty_weights_(idx)))
    {
      return std::max(0.0, penalty_weights_(idx));
    }
    return 0.0;
  }

  double segmentVelocityBound(int seg_idx) const
  {
    if (piece_velocity_bounds_ != nullptr &&
        seg_idx >= 0 &&
        seg_idx < piece_velocity_bounds_->size() &&
        std::isfinite((*piece_velocity_bounds_)(seg_idx)) &&
        (*piece_velocity_bounds_)(seg_idx) > 1.0e-3)
    {
      return (*piece_velocity_bounds_)(seg_idx);
    }
    return boundOrDefault(0, 1.0);
  }

  const general_utils::PolyhedraH *h_polys_{nullptr};
  const Eigen::VectorXi *h_poly_idx_{nullptr};
  const general_utils::VecDf *piece_velocity_bounds_{nullptr};
  double smooth_eps_{1.0e-3};
  general_utils::VecDf magnitude_bounds_;
  general_utils::VecDf penalty_weights_;
  flatness::FlatnessMap *quadrotor_flatness_{nullptr};
  mutable general_utils::VecDf max_violation_{general_utils::VecDf::Zero(8)};
};
} // namespace cost_functional_manager
