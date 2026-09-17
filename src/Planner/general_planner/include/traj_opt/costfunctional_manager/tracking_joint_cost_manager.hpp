#pragma once

#include "traj_opt/costfunctional_manager/tracking_cost_manager.hpp"
#include "traj_opt/costfunctional/spatialcosts/polytope_position_penalty.hpp"
#include "traj_opt/costfunctional/spatialmap/sfc_common_types.hpp"
#include "traj_opt/minco/minco_trajectory.hpp"

namespace cost_functional_manager {

// Coupled physical costs only. Each MINCO optimizer owns coefficient elimination,
// energy and adjoint propagation; this manager supplies partials for both curves.
template <int SPos>
class TrackingJointCostManager {
public:
  using PosTraj = minco::MINCOTrajectory<3, SPos>;
  using YawTraj = minco::MINCOTrajectory<1, 3>;
  using PosCoeffMat = typename PosTraj::CoeffMat;
  using YawCoeffMat = typename YawTraj::CoeffMat;
  using Vec3f = Eigen::Vector3d;

  TrackingJointCostManager(const traj_opt::Config &cfg,
                           const traj_opt::TrackingProblem &problem,
                           const general_planner::MapManager::Ptr &map,
                           const spatial_map::PolyhedraH &corridors)
      : cfg_(cfg), problem_(problem), h_polytopes_(corridors),
        samples_per_piece_(std::max(4, cfg.integral_reso)),
        use_corridor_(problem.use_corridor && !corridors.empty()) {
    cost_manager_.reset(cfg_, map, problem_);
  }

  double evaluate(const PosTraj &position, const YawTraj &yaw_trajectory,
                  PosCoeffMat &gdC_pos, YawCoeffMat &gdC_yaw,
                  Eigen::VectorXd &gdT_pos, Eigen::VectorXd &gdT_yaw) const {
    const Eigen::VectorXd &durations = position.getDurations();
    const int piece_num_ = durations.size();
    gdC_pos.setZero(PosTraj::COEFF_NUM * piece_num_, 3);
    gdC_yaw.setZero(YawTraj::COEFF_NUM * piece_num_, 1);
    gdT_pos.setZero(piece_num_); gdT_yaw.setZero(piece_num_);
    double total_cost = 0.0;
    const auto &pos_coeffs = position.getCoefficients();
    const auto &yaw_coeffs = yaw_trajectory.getCoefficients();

    double seg_start_time = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      const double T = durations(i);
      const double inv_K = 1.0 / static_cast<double>(samples_per_piece_);
      const double dt = T * inv_K;
      const int pos_base = i * PosTraj::COEFF_NUM;
      const int yaw_base = i * YawTraj::COEFF_NUM;
      const auto pos_block = pos_coeffs.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0);
      const auto yaw_block = yaw_coeffs.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0);

      for (int k = 0; k <= samples_per_piece_; ++k)
      {
        const double alpha = static_cast<double>(k) * inv_K;
        const double t_local = alpha * T;
        const double t_global = seg_start_time + t_local;
        const double trap_weight = (k == 0 || k == samples_per_piece_) ? 0.5 : 1.0;
        const double common_weight = trap_weight * dt;

        typename PosTraj::BasisRow bp, bv, ba, bj, bs;
        PosTraj::computeBasisFunctions(t_local, bp, bv, ba, bj, bs);
        Vec3f p = Vec3f::Zero();
        Vec3f v = Vec3f::Zero();
        Vec3f a = Vec3f::Zero();
        Vec3f j = Vec3f::Zero();
        Vec3f s = Vec3f::Zero();
        p.transpose().noalias() = bp * pos_block;
        v.transpose().noalias() = bv * pos_block;
        a.transpose().noalias() = ba * pos_block;
        j.transpose().noalias() = bj * pos_block;
        s.transpose().noalias() = bs * pos_block;

        Vec3f gp_integral = Vec3f::Zero();
        Vec3f gv_integral = Vec3f::Zero();
        Vec3f ga_integral = Vec3f::Zero();
        Vec3f gj_integral = Vec3f::Zero();
        double gt_integral = 0.0;
        double c_corridor = 0.0;
        if (use_corridor_ && i < static_cast<int>(h_polytopes_.size()) && cfg_.penna_pos > 0.0)
        {
          c_corridor = cost_functional::accumulatePolytopePositionPenalty(
              h_polytopes_[static_cast<std::size_t>(i)],
              p,
              cfg_.smooth_eps,
              cfg_.penna_pos * corridor_penalty_scale_,
              gp_integral);
        }
        const double c_continuous = cost_manager_.evaluateIntegral(i * samples_per_piece_ + k,
                                                                   t_local,
                                                                   t_global,
                                                                   i,
                                                                   k,
                                                                   p,
                                                                   v,
                                                                   a,
                                                                   j,
                                                                   gp_integral,
                                                                   gv_integral,
                                                                   ga_integral,
                                                                   gj_integral,
                                                                   gt_integral);
        // The final commit enforces yaw dynamics. Include the same limits
        // in optimization, with gradients through coefficients and duration.
        typename YawTraj::BasisRow ybp, ybv, yba, ybj, ybs;
        YawTraj::computeBasisFunctions(t_local, ybp, ybv, yba, ybj, ybs);
        const double yaw = (ybp * yaw_block)(0, 0);
        const double rate = (ybv * yaw_block)(0, 0);
        const double angular_acc = (yba * yaw_block)(0, 0);
        const double angular_jerk = (ybj * yaw_block)(0, 0);
        const double rate_cap = std::max(problem_.max_yaw_rate, std::abs(problem_.head_yaw(0,1)));
        double yaw_cost = 0.0, grad_rate = 0.0, grad_acc = 0.0;
        auto bound = [&](double value, double cap, double &gradient) {
          const double excess = std::abs(value) - cap;
          if (excess <= 0.0) return 0.0;
          const double weight = std::max(0.0, cfg_.penna_omg);
          gradient = 2.0 * weight * excess * (value < 0.0 ? -1.0 : 1.0);
          return weight * excess * excess;
        };
        yaw_cost += bound(rate, rate_cap, grad_rate);
        yaw_cost += bound(angular_acc, std::max(problem_.max_yaw_acceleration,
                                              std::abs(problem_.head_yaw_acceleration)), grad_acc);
        total_cost += yaw_cost * common_weight;
        gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
            (ybv.transpose() * grad_rate + yba.transpose() * grad_acc) * common_weight;
        gdT_yaw(i) += yaw_cost * trap_weight * inv_K +
            (grad_rate * angular_acc + grad_acc * angular_jerk) * alpha * common_weight;
        double gy_attitude = 0.0, gyr_attitude = 0.0;
        const double c_attitude = cost_manager_.evaluateAttitudeIntegral(
            t_global, p, a, j, yaw, rate, gp_integral, ga_integral, gj_integral,
            gy_attitude, gyr_attitude, gt_integral);
        gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
            (ybp.transpose() * gy_attitude + ybv.transpose() * gyr_attitude) * common_weight;
        gdT_yaw(i) += (gy_attitude * rate + gyr_attitude * angular_acc) * alpha * common_weight;
        const double c_integral = c_corridor + c_continuous + c_attitude;
        total_cost += c_integral * common_weight;
        gdC_pos.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0).noalias() +=
            (bp.transpose() * gp_integral.transpose() +
             bv.transpose() * gv_integral.transpose() +
             ba.transpose() * ga_integral.transpose() +
             bj.transpose() * gj_integral.transpose()) *
            common_weight;
        gdT_pos(i) += c_integral * trap_weight * inv_K;
        gdT_pos(i) += (gp_integral.dot(v) + gv_integral.dot(a) +
                       ga_integral.dot(j) + gj_integral.dot(s)) *
                      alpha * common_weight;
        gdT_pos(i) += gt_integral * alpha * common_weight;
        for (int previous = 0; previous < i; ++previous)
          gdT_pos(previous) += gt_integral * common_weight;
      }

      seg_start_time += T;
    }

    const auto accumulate_discrete_visibility_sample = [&](const double sample_t, const double sample_weight) {
      if (!std::isfinite(sample_t))
      {
        return;
      }
      const double total_duration = durations.sum();
      if (sample_t < -1.0e-6 || sample_t > total_duration + 1.0e-6)
      {
        return;
      }

      const double t_global = std::clamp(sample_t, 0.0, total_duration);
      double sample_seg_start = 0.0;
      for (int i = 0; i < piece_num_; ++i)
      {
        const double T = durations(i);
        const bool in_segment =
            i == piece_num_ - 1 || t_global <= sample_seg_start + T + 1.0e-9;
        if (!in_segment)
        {
          sample_seg_start += T;
          continue;
        }

        const double t_local = std::clamp(t_global - sample_seg_start, 0.0, T);
        const int pos_base = i * PosTraj::COEFF_NUM;
        const int yaw_base = i * YawTraj::COEFF_NUM;
        const auto pos_block = pos_coeffs.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0);
        const auto yaw_block = yaw_coeffs.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0);

        typename PosTraj::BasisRow bp, bv, ba, bj, bs;
        PosTraj::computeBasisFunctions(t_local, bp, bv, ba, bj, bs);
        Vec3f p = Vec3f::Zero();
        Vec3f v = Vec3f::Zero();
        Vec3f a = Vec3f::Zero();
        p.transpose().noalias() = bp * pos_block;
        v.transpose().noalias() = bv * pos_block;
        a.transpose().noalias() = ba * pos_block;

        typename YawTraj::BasisRow ybp, ybv, yba, ybj, ybs;
        YawTraj::computeBasisFunctions(t_local, ybp, ybv, yba, ybj, ybs);
        const double yaw = (ybp * yaw_block)(0, 0);
        const double yaw_dot = (ybv * yaw_block)(0, 0);
        const double yaw_acc = (yba * yaw_block)(0, 0);

        Vec3f gp = Vec3f::Zero();
        Vec3f gv = Vec3f::Zero();
        double gyaw = 0.0;
        double gyaw_dot = 0.0;
        double gt_global = 0.0;
        const double c_track = cost_manager_.evaluateJointSample(t_global,
                                                                 p,
                                                                 v,
                                                                 yaw,
                                                                 yaw_dot,
                                                                 gp,
                                                                 gv,
                                                                 gyaw,
                                                                 gyaw_dot,
                                                                 gt_global);
        total_cost += sample_weight * c_track;
        gp *= sample_weight; gv *= sample_weight;
        gyaw *= sample_weight; gyaw_dot *= sample_weight;
        gdC_pos.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0).noalias() +=
            bp.transpose() * gp.transpose() +
            bv.transpose() * gv.transpose();
        gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
            ybp.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw) +
            ybv.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw_dot);

        const double pos_time_grad = gp.dot(v) + gv.dot(a);
        const double yaw_time_grad = gyaw * yaw_dot + gyaw_dot * yaw_acc;
        for (int time_idx = 0; time_idx < i; ++time_idx)
        {
          gdT_pos(time_idx) -= pos_time_grad;
          gdT_yaw(time_idx) -= yaw_time_grad;
        }
        break;
      }
    };

    const auto &sample_times = cost_manager_.discreteSampleTimes();
    for (std::size_t k = 0; k < sample_times.size(); ++k) {
      const double left = k == 0 ? sample_times[k] : sample_times[k - 1];
      const double right = k + 1 == sample_times.size() ? sample_times[k] : sample_times[k + 1];
      const double confidence = problem_.trusted_horizon > 0.0
          ? std::exp(-1.5 * std::max(0.0, sample_times[k] - problem_.trusted_horizon)) : 1.0;
      accumulate_discrete_visibility_sample(sample_times[k],
          0.5 * (right - left) / 0.05 * confidence);
    }
    return total_cost;
  }

private:
  traj_opt::Config cfg_;
  traj_opt::TrackingProblem problem_;
  spatial_map::PolyhedraH h_polytopes_;
  int samples_per_piece_;
  bool use_corridor_;
  const double corridor_penalty_scale_{1.0};
  TrackingCostManager cost_manager_;
};

} // namespace cost_functional_manager
