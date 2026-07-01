#ifndef MINCO_BOUNDARY_MAPPING_HPP
#define MINCO_BOUNDARY_MAPPING_HPP

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

#include "data_structure/base/trajectory.h"
#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/perching_surface_state.hpp"

namespace minco
{

/**
 * @brief Task-layer boundary-state mapping hook.
 *
 * Fixed-boundary MINCO eliminates polynomial coefficients with
 * M(T)c = b(head, inner, tail). State-to-state and current tracking tasks keep
 * head/tail fixed, so no chain rule beyond MINCO's own variables is required.
 *
 * Perching-style tasks need a higher-level tail boundary model such as
 * tail_state = F(T, Xi(T), nu, tau_f, ...). In that case the optimizer first
 * exposes dJ/d(head_state) and dJ/d(tail_state), then the task mapping adds
 * chain-rule terms to outer decision variables and to physical segment-time
 * gradients.
 */
template <int DIM, int S>
class BoundaryStateMappingBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BoundaryState = Eigen::Matrix<double, DIM, S>;

  virtual ~BoundaryStateMappingBase() = default;

  virtual bool enabled() const = 0;

  /**
   * @brief Number of extra unconstrained variables appended after [tau, xi].
   *
   * Fixed-boundary tasks keep this zero. Dynamic boundary tasks can expose
   * variables like tangential closing velocity or thrust phase here.
   */
  virtual int extraVariableDim() const
  {
    return 0;
  }

  /**
   * @brief Fill the initial guess of extra boundary variables.
   */
  virtual void setInitialExtraVariables(Eigen::Ref<Eigen::VectorXd> extra_vars) const
  {
    extra_vars.setZero();
  }

  /**
   * @brief Forward boundary mapping hook.
   *
   * MINCOOptimizer first decodes [tau, xi] into physical segment times and
   * inner points. Dynamic boundary tasks then map
   *   head/tail = F(cache_T, extra_vars, nominal_head, nominal_tail)
   * before generating the trajectory coefficients.
   */
  virtual void mapBoundaryStates(const BoundaryState &nominal_head_state,
                                 const BoundaryState &nominal_tail_state,
                                 const Eigen::VectorXd &cache_T,
                                 const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                 BoundaryState &mapped_head_state,
                                 BoundaryState &mapped_tail_state) const
  {
    mapped_head_state = nominal_head_state;
    mapped_tail_state = nominal_tail_state;
    (void)cache_T;
    (void)extra_vars;
  }

  /**
   * @brief Optional regularization on extra boundary variables.
   *
   * This cost is accumulated directly in the unconstrained decision space.
   */
  virtual double addExtraVariableCost(const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                      Eigen::Ref<Eigen::VectorXd> grad_extra) const
  {
    grad_extra.setZero();
    (void)extra_vars;
    return 0.0;
  }

  /**
   * @brief Backpropagate boundary-state gradients to extra outer variables.
   *
   * grad_out is the full unconstrained decision-gradient vector supplied by
   * the caller. MINCO writes its own [tau, xi] entries; task-specific boundary
   * variables, if present, should be written by the mapping implementation.
   */
  virtual void backwardBoundaryGradient(const BoundaryState &,
                                        const BoundaryState &,
                                        const Eigen::VectorXd &,
                                        const Eigen::Ref<const Eigen::VectorXd> &,
                                        Eigen::Ref<Eigen::VectorXd>) const
  {
  }

  /**
   * @brief Add physical-time chain-rule terms before TimeMap::backward.
   *
   * Fixed-boundary MINCO produces dJ/dT | fixed_boundary. If a task has
   * boundary_state = F(T, ...), it must add
   * (d boundary_state / dT)^T * (dJ / d boundary_state)
   * to grad_by_times here. MINCOOptimizer then maps that physical dJ/dT to
   * the unconstrained tau gradient through the active TimeMap.
   */
  virtual void backwardBoundaryTimeGradient(const BoundaryState &,
                                            const BoundaryState &,
                                            const Eigen::VectorXd &,
                                            const Eigen::Ref<const Eigen::VectorXd> &,
                                            Eigen::Ref<Eigen::VectorXd>) const
  {
  }
};

template <int DIM, int S>
class FixedBoundaryMapping final : public BoundaryStateMappingBase<DIM, S>
{
public:
  bool enabled() const override
  {
    return false;
  }
};

template <int DIM, int S>
class BackupBoundaryMapping final : public BoundaryStateMappingBase<DIM, S>
{
public:
  static_assert(DIM == 3, "BackupBoundaryMapping currently assumes 3D position trajectories.");
  static_assert(S <= 4, "BackupBoundaryMapping supports PVAJ trajectories up to snap-backed time gradients.");

  using BoundaryState = typename BoundaryStateMappingBase<DIM, S>::BoundaryState;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void configure(const geometry_utils::Trajectory *reference_traj,
                 double min_ts,
                 double max_ts,
                 double ts_seed,
                 const Eigen::Vector3d &tail_seed,
                 const general_utils::PolyhedronV *tail_polytope,
                 int piece_num,
                 bool identity_tail_map,
                 double weight_ts)
  {
    reference_traj_ = reference_traj;
    min_ts_ = min_ts;
    max_ts_ = std::max(max_ts, min_ts + 1.0e-6);
    ts_seed_ = std::clamp(ts_seed, min_ts_ + 1.0e-6, max_ts_ - 1.0e-6);
    tail_seed_ = tail_seed;
    tail_polytope_ = tail_polytope;
    piece_num_ = std::max(1, piece_num);
    identity_tail_map_ = identity_tail_map;
    weight_ts_ = std::max(0.0, weight_ts);
    if (tail_polytope_ != nullptr)
    {
      tail_map_.reset(tail_polytope_, piece_num_, identity_tail_map_);
    }
  }

  bool enabled() const override
  {
    return reference_traj_ != nullptr;
  }

  int extraVariableDim() const override
  {
    return 1 + tail_map_.getUnconstrainedDim(piece_num_);
  }

  void setInitialExtraVariables(Eigen::Ref<Eigen::VectorXd> extra_vars) const override
  {
    if (extra_vars.size() != extraVariableDim())
    {
      return;
    }
    extra_vars(0) = toEta(ts_seed_);
    const Eigen::VectorXd tail_xi = tail_map_.toUnconstrained(tail_seed_, piece_num_);
    extra_vars.tail(tail_xi.size()) = tail_xi;
  }

  void mapBoundaryStates(const BoundaryState &nominal_head_state,
                         const BoundaryState &nominal_tail_state,
                         const Eigen::VectorXd &,
                         const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                         BoundaryState &mapped_head_state,
                         BoundaryState &mapped_tail_state) const override
  {
    mapped_head_state = nominal_head_state;
    mapped_tail_state = nominal_tail_state;
    if (extra_vars.size() != extraVariableDim())
    {
      return;
    }

    const double ts = toTs(extra_vars(0));
    general_utils::StatePVAJ state = general_utils::StatePVAJ::Zero();
    if (reference_traj_ != nullptr && reference_traj_->getState(ts, state))
    {
      for (int d = 0; d < S; ++d)
      {
        mapped_head_state.col(d) = state.col(d);
      }
    }

    const Eigen::VectorXd tail_xi = extra_vars.tail(extra_vars.size() - 1);
    mapped_tail_state.col(0) = tail_map_.toPhysical(tail_xi, piece_num_);
    last_ts_ = ts;
    last_tail_ = mapped_tail_state.col(0);
  }

  double addExtraVariableCost(const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                              Eigen::Ref<Eigen::VectorXd> grad_extra) const override
  {
    grad_extra.setZero();
    if (extra_vars.size() != extraVariableDim())
    {
      return 0.0;
    }

    const double ts = toTs(extra_vars(0));
    double cost = weight_ts_ * (max_ts_ - ts);
    grad_extra(0) += -weight_ts_ * dTsDeta(extra_vars(0));

    const Eigen::VectorXd tail_xi = extra_vars.tail(extra_vars.size() - 1);
    Eigen::VectorXd tail_grad = grad_extra.tail(extra_vars.size() - 1);
    tail_map_.addNormPenalty(tail_xi, cost, tail_grad);
    grad_extra.tail(extra_vars.size() - 1) = tail_grad;
    return cost;
  }

  void backwardBoundaryGradient(const BoundaryState &grad_head_state,
                                const BoundaryState &grad_tail_state,
                                const Eigen::VectorXd &,
                                const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                Eigen::Ref<Eigen::VectorXd> grad_out) const override
  {
    const int extra_dim = extraVariableDim();
    if (extra_vars.size() != extra_dim || grad_out.size() < extra_dim)
    {
      return;
    }

    const int extra_offset = static_cast<int>(grad_out.size()) - extra_dim;
    const double ts = toTs(extra_vars(0));
    double grad_ts = 0.0;
    if (reference_traj_ != nullptr)
    {
      if constexpr (S >= 1)
      {
        grad_ts += grad_head_state.col(0).dot(reference_traj_->getVel(ts));
      }
      if constexpr (S >= 2)
      {
        grad_ts += grad_head_state.col(1).dot(reference_traj_->getAcc(ts));
      }
      if constexpr (S >= 3)
      {
        grad_ts += grad_head_state.col(2).dot(reference_traj_->getJer(ts));
      }
      if constexpr (S >= 4)
      {
        grad_ts += grad_head_state.col(3).dot(reference_traj_->getSnap(ts));
      }
    }
    grad_out(extra_offset) += grad_ts * dTsDeta(extra_vars(0));

    const Eigen::VectorXd tail_xi = extra_vars.tail(extra_dim - 1);
    grad_out.segment(extra_offset + 1, extra_dim - 1) +=
        tail_map_.backwardGrad(tail_xi, grad_tail_state.col(0), piece_num_);
  }

  double lastTs() const
  {
    return last_ts_;
  }

  const Eigen::Vector3d &lastTail() const
  {
    return last_tail_;
  }

private:
  double toTs(double eta) const
  {
    const double sigma = 1.0 / (1.0 + std::exp(-eta));
    return min_ts_ + (max_ts_ - min_ts_) * sigma;
  }

  double dTsDeta(double eta) const
  {
    const double sigma = 1.0 / (1.0 + std::exp(-eta));
    return (max_ts_ - min_ts_) * sigma * (1.0 - sigma);
  }

  double toEta(double ts) const
  {
    const double span = std::max(1.0e-6, max_ts_ - min_ts_);
    const double ratio = std::clamp((ts - min_ts_) / span, 1.0e-6, 1.0 - 1.0e-6);
    return std::log(ratio / (1.0 - ratio));
  }

  const geometry_utils::Trajectory *reference_traj_{nullptr};
  const general_utils::PolyhedronV *tail_polytope_{nullptr};
  spatial_map::PolytopeSpatialMap tail_map_;
  int piece_num_{1};
  bool identity_tail_map_{false};
  double min_ts_{0.0};
  double max_ts_{1.0};
  double ts_seed_{0.0};
  double weight_ts_{0.0};
  Eigen::Vector3d tail_seed_{Eigen::Vector3d::Zero()};
  mutable double last_ts_{0.0};
  mutable Eigen::Vector3d last_tail_{Eigen::Vector3d::Zero()};
};

/**
 * @brief Simplified Fast-Perching-style dynamic tail boundary mapping.
 *
 * This mapping follows the paper semantics in a lightweight way:
 *   tail_pos(T) = Xi_ref + Xi_dot * (T - T_ref) + l * z_s
 *   tail_vel(T) = Xi_dot + nu_x * x_s + nu_y * y_s - v_plus * z_s
 *   tail_acc(T) = (tau_m + tau_r * sin(tau_f)) * z_s + g
 *   tail_jerk(T) = 0                              (when S >= 4)
 *
 * Here Xi(T) is predicted with a constant-acceleration landing-plate model,
 * and the surface frame {x_s, y_s, z_s} can rotate with yaw_rate over one
 * planning cycle. That keeps the current perching pipeline compatible with the
 * generic MINCO backend while exposing the correct chain rule:
 *
 *   dJ / d extra = (d tail_state / d extra)^T * dJ / d tail_state
 *   dJ / d T_i  += (d tail_state / d T)^T * dJ / d tail_state
 *
 * The same mapping works for S=3 (minimum jerk, P/V/A tail constraints) and
 * for S=4 (minimum snap, P/V/A/J tail constraints). When S=4, tail jerk is
 * explicitly forced to zero as in the paper.
 */
struct PerchingSemanticConfig
{
  Eigen::Vector3d plate_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_acceleration{Eigen::Vector3d::Zero()};
  double reference_time{0.0};
  Eigen::Vector3d surface_x{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d surface_y{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d surface_z{Eigen::Vector3d::UnitZ()};
  double yaw{0.0};
  double yaw_rate{0.0};
  bool rotate_surface_with_yaw_rate{true};
  double gravity{9.81};
  double terminal_time_seed{0.0};
  double robot_l{0.0};
  double v_plus{0.0};
  double thrust_nominal{9.81};
  double thrust_range{0.0};
  bool use_dynamics_terminal_accel{false};
  Eigen::Vector2d nu_seed{Eigen::Vector2d::Zero()};
  double tau_f_seed{0.0};
  double pre_contact_distance{0.4};
  double terminal_relax_time{0.35};
  double weight_nu{1.0e-2};
  double weight_tau_f{1.0e-3};
};

struct TakeoffBoundaryConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  traj_opt::PerchingSurfaceState surface;
  double robot_l{0.28};
  double thrust_nominal{9.81};
  double thrust_range{2.0};
  double gravity{9.81};
  bool use_tangent_release_velocity{false};
  double weight_eta{1.0};
  double weight_tau_f{1.0e-3};
  double tau_f_seed{0.0};
  Eigen::Vector2d eta_seed{Eigen::Vector2d::Zero()};
  bool rotate_surface_with_yaw_rate{true};
};

template <int DIM, int S>
class PerchingBoundaryMapping final : public BoundaryStateMappingBase<DIM, S>
{
public:
  static_assert(DIM == 3, "PerchingBoundaryMapping currently assumes 3D position trajectories.");
  static_assert(S >= 3, "PerchingBoundaryMapping requires boundary derivatives up to acceleration.");

  using BoundaryState = typename BoundaryStateMappingBase<DIM, S>::BoundaryState;
  using PerchingSemanticConfig = minco::PerchingSemanticConfig;

  enum ExtraIndex
  {
    IDX_NU_X = 0,
    IDX_NU_Y = 1,
    IDX_TAU_F = 2,
    EXTRA_DIM = 3
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PerchingBoundaryMapping() = default;

  /**
   * @brief Primary perching configuration path.
   *
   * The intended caller is the task/frontend layer after it has decoded rich
   * perching semantics such as contact frame, pre-contact distance and the
   * relaxed terminal window. The mapping itself only consumes the subset that
   * affects boundary-state forward/backward passes, but it keeps the richer
   * semantic payload so downstream acceptance / debug code can stay aligned.
   */
  void configure(const PerchingSemanticConfig &config)
  {
    semantic_config_ = config;
    semantic_config_.reference_time = std::max(0.0, semantic_config_.reference_time);
    semantic_config_.surface_x = normalizedOr(semantic_config_.surface_x, Eigen::Vector3d::UnitX());
    semantic_config_.surface_y = normalizedOr(semantic_config_.surface_y, Eigen::Vector3d::UnitY());
    semantic_config_.surface_z = normalizedOr(semantic_config_.surface_z, Eigen::Vector3d::UnitZ());

    // Keep the tangent frame right-handed and orthonormal enough for chain
    // rule projections.
    semantic_config_.surface_y =
        normalizedOr(semantic_config_.surface_z.cross(semantic_config_.surface_x),
                     Eigen::Vector3d::UnitY());
    semantic_config_.surface_x =
        normalizedOr(semantic_config_.surface_y.cross(semantic_config_.surface_z),
                     Eigen::Vector3d::UnitX());

    semantic_config_.robot_l = std::max(0.0, semantic_config_.robot_l);
    semantic_config_.v_plus = std::max(0.0, semantic_config_.v_plus);
    semantic_config_.thrust_nominal = std::max(0.0, semantic_config_.thrust_nominal);
    semantic_config_.thrust_range = std::max(0.0, semantic_config_.thrust_range);
    semantic_config_.pre_contact_distance = std::max(0.0, semantic_config_.pre_contact_distance);
    semantic_config_.terminal_relax_time = std::max(0.0, semantic_config_.terminal_relax_time);
    semantic_config_.weight_nu = std::max(0.0, semantic_config_.weight_nu);
    semantic_config_.weight_tau_f = std::max(0.0, semantic_config_.weight_tau_f);
    semantic_config_.gravity = std::abs(semantic_config_.gravity);
    if (!std::isfinite(semantic_config_.terminal_time_seed))
    {
      semantic_config_.terminal_time_seed = 0.0;
    }
    configured_ = true;
  }

  void configure(const Eigen::Vector3d &plate_position,
                 const Eigen::Vector3d &plate_velocity,
                 const double reference_time,
                 const Eigen::Vector3d &surface_x,
                 const Eigen::Vector3d &surface_y,
                 const Eigen::Vector3d &surface_z,
                 const double robot_l,
                 const double v_plus,
                 const double thrust_nominal,
                 const double thrust_range,
                 const bool use_dynamics_terminal_accel,
                 const Eigen::Vector2d &nu_seed = Eigen::Vector2d::Zero(),
                 const double tau_f_seed = 0.0,
                 const double pre_contact_distance = 0.4,
                 const double terminal_relax_time = 0.35,
                 const double weight_nu = 1.0e-2,
                 const double weight_tau_f = 1.0e-3)
  {
    PerchingSemanticConfig config;
    config.plate_position = plate_position;
    config.plate_velocity = plate_velocity;
    config.reference_time = reference_time;
    config.surface_x = surface_x;
    config.surface_y = surface_y;
    config.surface_z = surface_z;
    config.robot_l = robot_l;
    config.v_plus = v_plus;
    config.thrust_nominal = thrust_nominal;
    config.thrust_range = thrust_range;
    config.use_dynamics_terminal_accel = use_dynamics_terminal_accel;
    config.nu_seed = nu_seed;
    config.tau_f_seed = tau_f_seed;
    config.pre_contact_distance = pre_contact_distance;
    config.terminal_relax_time = terminal_relax_time;
    config.weight_nu = weight_nu;
    config.weight_tau_f = weight_tau_f;
    configure(config);
  }

  bool enabled() const override
  {
    return configured_;
  }

  int extraVariableDim() const override
  {
    return EXTRA_DIM;
  }

  const PerchingSemanticConfig &semanticConfig() const
  {
    return semantic_config_;
  }

  void setInitialExtraVariables(Eigen::Ref<Eigen::VectorXd> extra_vars) const override
  {
    if (extra_vars.size() != EXTRA_DIM)
    {
      return;
    }
    extra_vars.setZero();
    extra_vars(IDX_NU_X) = semantic_config_.nu_seed.x();
    extra_vars(IDX_NU_Y) = semantic_config_.nu_seed.y();
    extra_vars(IDX_TAU_F) = semantic_config_.tau_f_seed;
  }

  void mapBoundaryStates(const BoundaryState &nominal_head_state,
                         const BoundaryState &nominal_tail_state,
                         const Eigen::VectorXd &cache_T,
                         const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                         BoundaryState &mapped_head_state,
                         BoundaryState &mapped_tail_state) const override
  {
    mapped_head_state = nominal_head_state;
    mapped_tail_state = nominal_tail_state;
    if (!configured_)
    {
      return;
    }

    const double total_T = cache_T.size() > 0 ? cache_T.sum() : 0.0;
    const double nu_x = extra_vars.size() > IDX_NU_X ? extra_vars(IDX_NU_X) : 0.0;
    const double nu_y = extra_vars.size() > IDX_NU_Y ? extra_vars(IDX_NU_Y) : 0.0;
    const double tau_f = extra_vars.size() > IDX_TAU_F ? extra_vars(IDX_TAU_F) : 0.0;
    const auto frame = surfaceFrameAt(total_T);

    mapped_tail_state.col(0) = semantic_config_.plate_position +
                               semantic_config_.plate_velocity * total_T +
                               0.5 * semantic_config_.plate_acceleration *
                                   total_T * total_T +
                               semantic_config_.robot_l * frame.z;
    mapped_tail_state.col(1) = semantic_config_.plate_velocity +
                               semantic_config_.plate_acceleration * total_T +
                               nu_x * frame.x +
                               nu_y * frame.y -
                               semantic_config_.v_plus * frame.z;
    if (semantic_config_.use_dynamics_terminal_accel)
    {
      const double terminal_thrust =
          semantic_config_.thrust_nominal +
          semantic_config_.thrust_range * std::sin(tau_f);
      mapped_tail_state.col(2) =
          terminal_thrust * frame.z + gravityVector();
    }
    else if (S > 2)
    {
      mapped_tail_state.col(2) = nominal_tail_state.col(2);
    }

    for (int d = 3; d < S; ++d)
    {
      mapped_tail_state.col(d).setZero();
    }
  }

  double addExtraVariableCost(const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                              Eigen::Ref<Eigen::VectorXd> grad_extra) const override
  {
    grad_extra.setZero();
    if (!configured_ || extra_vars.size() != EXTRA_DIM)
    {
      return 0.0;
    }

    const double nu_x = extra_vars(IDX_NU_X);
    const double nu_y = extra_vars(IDX_NU_Y);
    const double tau_f = extra_vars(IDX_TAU_F);

    double cost = 0.0;
    cost += semantic_config_.weight_nu * (nu_x * nu_x + nu_y * nu_y);
    cost += semantic_config_.weight_tau_f * tau_f * tau_f;

    grad_extra(IDX_NU_X) = 2.0 * semantic_config_.weight_nu * nu_x;
    grad_extra(IDX_NU_Y) = 2.0 * semantic_config_.weight_nu * nu_y;
    grad_extra(IDX_TAU_F) = 2.0 * semantic_config_.weight_tau_f * tau_f;
    return cost;
  }

  void backwardBoundaryGradient(const BoundaryState &,
                                const BoundaryState &grad_tail_state,
                                const Eigen::VectorXd &cache_T,
                                const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                Eigen::Ref<Eigen::VectorXd> grad_out) const override
  {
    if (!configured_ || grad_out.size() < EXTRA_DIM || extra_vars.size() != EXTRA_DIM)
    {
      return;
    }

    const Eigen::Index offset = grad_out.size() - EXTRA_DIM;
    const double tau_f = extra_vars(IDX_TAU_F);
    const double total_T = cache_T.size() > 0 ? cache_T.sum() : 0.0;
    const auto frame = surfaceFrameAt(total_T);

    grad_out(offset + IDX_NU_X) += frame.x.dot(grad_tail_state.col(1));
    grad_out(offset + IDX_NU_Y) += frame.y.dot(grad_tail_state.col(1));
    if (semantic_config_.use_dynamics_terminal_accel)
    {
      grad_out(offset + IDX_TAU_F) +=
          semantic_config_.thrust_range * std::cos(tau_f) *
          frame.z.dot(grad_tail_state.col(2));
    }
  }

  void backwardBoundaryTimeGradient(const BoundaryState &,
                                    const BoundaryState &grad_tail_state,
                                    const Eigen::VectorXd &cache_T,
                                    const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                    Eigen::Ref<Eigen::VectorXd> grad_by_times) const override
  {
    if (!configured_ || cache_T.size() == 0 || grad_by_times.size() != cache_T.size())
    {
      return;
    }

    const double total_T = cache_T.sum();
    const double nu_x = extra_vars.size() > IDX_NU_X ? extra_vars(IDX_NU_X) : 0.0;
    const double nu_y = extra_vars.size() > IDX_NU_Y ? extra_vars(IDX_NU_Y) : 0.0;
    const double tau_f = extra_vars.size() > IDX_TAU_F ? extra_vars(IDX_TAU_F) : 0.0;
    const auto frame = surfaceFrameAt(total_T);
    const double terminal_thrust =
        semantic_config_.thrust_nominal +
        semantic_config_.thrust_range * std::sin(tau_f);

    const Eigen::Vector3d dp_dT =
        semantic_config_.plate_velocity +
        semantic_config_.plate_acceleration * total_T +
        semantic_config_.robot_l * frame.dz;
    const Eigen::Vector3d dv_dT =
        semantic_config_.plate_acceleration +
        nu_x * frame.dx +
        nu_y * frame.dy -
        semantic_config_.v_plus * frame.dz;
    Eigen::Vector3d da_dT = Eigen::Vector3d::Zero();
    if (semantic_config_.use_dynamics_terminal_accel)
    {
      da_dT = terminal_thrust * frame.dz;
    }

    const double time_chain =
        dp_dT.dot(grad_tail_state.col(0)) +
        dv_dT.dot(grad_tail_state.col(1)) +
        da_dT.dot(grad_tail_state.col(2));
    grad_by_times.array() += time_chain;
  }

private:
  struct SurfaceFrame
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d x{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d y{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};
    Eigen::Vector3d dx{Eigen::Vector3d::Zero()};
    Eigen::Vector3d dy{Eigen::Vector3d::Zero()};
    Eigen::Vector3d dz{Eigen::Vector3d::Zero()};
  };

  static Eigen::Vector3d normalizedOr(const Eigen::Vector3d &v,
                                      const Eigen::Vector3d &fallback)
  {
    if (!v.allFinite() || v.norm() < 1.0e-6)
    {
      return fallback;
    }
    return v.normalized();
  }

  Eigen::Vector3d gravityVector() const
  {
    return Eigen::Vector3d(0.0, 0.0, -std::abs(semantic_config_.gravity));
  }

  SurfaceFrame surfaceFrameAt(const double total_T) const
  {
    SurfaceFrame frame;
    frame.x = semantic_config_.surface_x;
    frame.y = semantic_config_.surface_y;
    frame.z = semantic_config_.surface_z;
    if (semantic_config_.rotate_surface_with_yaw_rate &&
        std::abs(semantic_config_.yaw_rate) > 1.0e-9)
    {
      const double theta = semantic_config_.yaw_rate * total_T;
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const Eigen::Matrix3d R =
          (Eigen::Matrix3d() << c, -s, 0.0,
                                s, c, 0.0,
                                0.0, 0.0, 1.0)
              .finished();
      frame.x = R * frame.x;
      frame.y = R * frame.y;
      frame.z = R * frame.z;

      const Eigen::Vector3d omega_z(0.0, 0.0, semantic_config_.yaw_rate);
      frame.dx = omega_z.cross(frame.x);
      frame.dy = omega_z.cross(frame.y);
      frame.dz = omega_z.cross(frame.z);
    }
    frame.z = normalizedOr(frame.z, Eigen::Vector3d::UnitZ());
    frame.x = normalizedOr(frame.x, Eigen::Vector3d::UnitX());
    frame.y = normalizedOr(frame.z.cross(frame.x), Eigen::Vector3d::UnitY());
    frame.x = normalizedOr(frame.y.cross(frame.z), Eigen::Vector3d::UnitX());
    return frame;
  }

private:
  bool configured_{false};
  PerchingSemanticConfig semantic_config_{};
};

template <int DIM, int S>
class TakeoffHeadBoundaryMapping final : public BoundaryStateMappingBase<DIM, S>
{
public:
  static_assert(DIM == 3, "TakeoffHeadBoundaryMapping currently assumes 3D position trajectories.");
  static_assert(S == 4, "TakeoffHeadBoundaryMapping is intended for T4 MINCO.");

  using BoundaryState = typename BoundaryStateMappingBase<DIM, S>::BoundaryState;
  using TakeoffBoundaryConfig = minco::TakeoffBoundaryConfig;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TakeoffHeadBoundaryMapping() = default;

  void configure(const TakeoffBoundaryConfig &config)
  {
    config_ = config;
    config_.surface.surface_z = normalizedOr(config_.surface.surface_z, Eigen::Vector3d::UnitZ());
    config_.surface.surface_x = normalizedOr(config_.surface.surface_x, Eigen::Vector3d::UnitX());
    config_.surface.surface_y =
        normalizedOr(config_.surface.surface_z.cross(config_.surface.surface_x),
                     Eigen::Vector3d::UnitY());
    config_.surface.surface_x =
        normalizedOr(config_.surface.surface_y.cross(config_.surface.surface_z),
                     Eigen::Vector3d::UnitX());
    config_.robot_l = std::max(0.0, config_.robot_l);
    config_.thrust_nominal = std::max(0.0, config_.thrust_nominal);
    config_.thrust_range = std::max(0.0, config_.thrust_range);
    config_.gravity = std::abs(config_.gravity);
    config_.weight_eta = std::max(0.0, config_.weight_eta);
    config_.weight_tau_f = std::max(0.0, config_.weight_tau_f);
    if (!std::isfinite(config_.tau_f_seed))
    {
      config_.tau_f_seed = 0.0;
    }
    configured_ = true;
  }

  bool enabled() const override
  {
    return configured_;
  }

  int extraVariableDim() const override
  {
    return config_.use_tangent_release_velocity ? 3 : 1;
  }

  const TakeoffBoundaryConfig &config() const
  {
    return config_;
  }

  void setInitialExtraVariables(Eigen::Ref<Eigen::VectorXd> extra_vars) const override
  {
    extra_vars.setZero();
    if (extra_vars.size() != extraVariableDim())
    {
      return;
    }
    const double tau_seed = std::clamp(config_.tau_f_seed, -1.3, 1.3);
    if (config_.use_tangent_release_velocity)
    {
      extra_vars(0) = config_.eta_seed.x();
      extra_vars(1) = config_.eta_seed.y();
      extra_vars(2) = tau_seed;
    }
    else
    {
      extra_vars(0) = tau_seed;
    }
  }

  void mapBoundaryStates(const BoundaryState &nominal_head_state,
                         const BoundaryState &nominal_tail_state,
                         const Eigen::VectorXd &,
                         const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                         BoundaryState &mapped_head_state,
                         BoundaryState &mapped_tail_state) const override
  {
    mapped_head_state = nominal_head_state;
    mapped_tail_state = nominal_tail_state;
    if (!configured_)
    {
      return;
    }

    const auto frame = surfaceFrame();
    const double tau_f = tauF(extra_vars);
    const double tau =
        config_.thrust_nominal + config_.thrust_range * std::sin(tau_f);

    mapped_head_state.col(0) =
        config_.surface.position + config_.robot_l * frame.z;
    mapped_head_state.col(1) = config_.surface.velocity;
    if (config_.use_tangent_release_velocity && extra_vars.size() >= 3)
    {
      mapped_head_state.col(1) += extra_vars(0) * frame.x + extra_vars(1) * frame.y;
    }
    mapped_head_state.col(2) = tau * frame.z + gravityVector();
    mapped_head_state.col(3).setZero();
  }

  double addExtraVariableCost(const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                              Eigen::Ref<Eigen::VectorXd> grad_extra) const override
  {
    grad_extra.setZero();
    if (!configured_ || extra_vars.size() != extraVariableDim())
    {
      return 0.0;
    }

    double cost = 0.0;
    if (config_.use_tangent_release_velocity)
    {
      const double eta_x = extra_vars(0);
      const double eta_y = extra_vars(1);
      cost += config_.weight_eta * (eta_x * eta_x + eta_y * eta_y);
      grad_extra(0) = 2.0 * config_.weight_eta * eta_x;
      grad_extra(1) = 2.0 * config_.weight_eta * eta_y;
    }

    const int tau_idx = tauIndex();
    const double tau_f = extra_vars(tau_idx);
    cost += config_.weight_tau_f * tau_f * tau_f;
    grad_extra(tau_idx) = 2.0 * config_.weight_tau_f * tau_f;
    return cost;
  }

  void backwardBoundaryGradient(const BoundaryState &grad_head_state,
                                const BoundaryState &,
                                const Eigen::VectorXd &,
                                const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                Eigen::Ref<Eigen::VectorXd> grad_out) const override
  {
    if (!configured_ || extra_vars.size() != extraVariableDim() ||
        grad_out.size() < extraVariableDim())
    {
      return;
    }

    const Eigen::Index offset = grad_out.size() - extraVariableDim();
    const auto frame = surfaceFrame();
    if (config_.use_tangent_release_velocity)
    {
      grad_out(offset) += frame.x.dot(grad_head_state.col(1));
      grad_out(offset + 1) += frame.y.dot(grad_head_state.col(1));
    }

    const int tau_idx = tauIndex();
    const double tau_f = extra_vars(tau_idx);
    grad_out(offset + tau_idx) +=
        config_.thrust_range * std::cos(tau_f) *
        frame.z.dot(grad_head_state.col(2));
  }

  void backwardBoundaryTimeGradient(const BoundaryState &,
                                    const BoundaryState &,
                                    const Eigen::VectorXd &,
                                    const Eigen::Ref<const Eigen::VectorXd> &,
                                    Eigen::Ref<Eigen::VectorXd>) const override
  {
    // First standalone takeoff fixes release time at t=0. If release_delay is
    // later optimized, add d(head_state)/d(release_delay) chain terms here.
  }

private:
  struct SurfaceFrame
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d x{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d y{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};
  };

  static Eigen::Vector3d normalizedOr(const Eigen::Vector3d &v,
                                      const Eigen::Vector3d &fallback)
  {
    if (!v.allFinite() || v.norm() < 1.0e-6)
    {
      return fallback;
    }
    return v.normalized();
  }

  Eigen::Vector3d gravityVector() const
  {
    return Eigen::Vector3d(0.0, 0.0, -std::abs(config_.gravity));
  }

  SurfaceFrame surfaceFrame() const
  {
    SurfaceFrame frame;
    frame.z = normalizedOr(config_.surface.surface_z, Eigen::Vector3d::UnitZ());
    frame.x = normalizedOr(config_.surface.surface_x, Eigen::Vector3d::UnitX());
    frame.y = normalizedOr(frame.z.cross(frame.x), Eigen::Vector3d::UnitY());
    frame.x = normalizedOr(frame.y.cross(frame.z), Eigen::Vector3d::UnitX());
    return frame;
  }

  int tauIndex() const
  {
    return config_.use_tangent_release_velocity ? 2 : 0;
  }

  double tauF(const Eigen::Ref<const Eigen::VectorXd> &extra_vars) const
  {
    const int idx = tauIndex();
    return extra_vars.size() > idx ? extra_vars(idx) : 0.0;
  }

private:
  bool configured_{false};
  TakeoffBoundaryConfig config_{};
};

} // namespace minco

#endif // MINCO_BOUNDARY_MAPPING_HPP
