#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "tracking/mmknet.hpp"
#include "tracking/imm_filter.hpp"
#include "tracking/imm_mot_filter.hpp"

namespace person_tracker
{

// Position/velocity EKF with selectable analytic or MMKNet motion prediction.
// In analytic mode, the posterior remains a compact 6D EKF and prediction evaluates
// constant velocity (CV), constant acceleration (CA), and coordinated turn
// (CT). Measurement likelihoods update the model probabilities. This keeps
// the public tracker lightweight while exposing multiple spatial hypotheses
// to the crossing/occlusion association logic. MMKNet instead supplies the
// horizontal turn rate for a single CT prediction and re-predicts on observations.
class TargetEkf
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix3x6d = Eigen::Matrix<double, 3, 6>;

  // Configure once before constructing tracks; copies retain independent histories.
  static MmkNetConfig & defaultMmkNetConfig()
  {
    static MmkNetConfig config;
    return config;
  }

  static bool & defaultImmEnabled() {static bool enabled=false;return enabled;}
  static ImmFilter::Config & defaultImmConfig() {static ImmFilter::Config config;return config;}
  static std::string & defaultUpstreamMode() {static std::string mode;return mode;}
  bool usesUpstream() const {return !upstream_mode_.empty();}
  const std::array<double,4> & upstreamProbabilities() const {return upstream_.probabilities;}
  bool usesImm() const {return imm_enabled_;}
  bool usesMmkNet() const {return bool(mmknet_config_.network);}
  bool mmknetReady() const {return mmknet_history_.ready();}
  std::size_t mmknetSamples() const {return mmknet_history_.samples();}

  struct MeasurementUpdate
  {
    bool velocity_valid{false};
    double displacement{0.0};
    double dt{0.0};
    double measured_speed{0.0};
  };

  TargetEkf(
    int id, const Eigen::Vector3d & position, const Eigen::Vector3d & size,
    double stamp_sec, double acceleration_noise, double measurement_noise,
    bool adaptive_size_enabled = true, double size_smoothing = 0.12,
    double size_max_relative_step = 0.25)
  : id_(id), size_(canonicalSize(size)), last_stamp_sec_(stamp_sec),
    last_measurement_position_(position), last_measurement_stamp_sec_(stamp_sec),
    acceleration_noise_(acceleration_noise), measurement_noise_(measurement_noise),
    adaptive_size_enabled_(adaptive_size_enabled),
    size_smoothing_(std::clamp(size_smoothing, 0.0, 1.0)),
    size_max_relative_step_(std::max(0.0, size_max_relative_step))
  {
    mmknet_history_.clear();
    network_prior_valid_ = false;
    state_.setZero();
    state_.head<3>() = position;
    association_position_ = position;
    covariance_.setZero();
    covariance_.diagonal() << 0.09, 0.09, 0.09, 1.0, 1.0, 1.0;
    model_predictions_.fill(position);
    model_velocities_.fill(Eigen::Vector3d::Zero());
    if (usesUpstream()) {upstream_.initialize(upstream_mode_,position,size,stamp_sec);syncUpstream();}
    if (usesImm()) imm_.initialize(position.head<2>(), acceleration_noise_, measurement_noise_, imm_config_);
  }

  void predict(double stamp_sec)
  {
    double dt = stamp_sec - last_stamp_sec_;
    if (usesUpstream()) {
      if (!std::isfinite(dt) || dt<=0) return;
      upstream_.apply("predict",{dt});
      covariance_(2,2) += .25*std::pow(dt,4)*acceleration_noise_*acceleration_noise_;
      syncUpstream();
      last_stamp_sec_=stamp_sec;last_prediction_dt_=dt;++age_;return;
    }
    if (usesImm()) {
      if (!std::isfinite(dt) || dt <= 0.0) return;
      imm_.predict(dt);
      state_(2) += state_(5)*dt;
      covariance_(2,2) += dt*dt*covariance_(5,5) + .25*std::pow(dt,4)*acceleration_noise_*acceleration_noise_;
      syncImm();
      last_stamp_sec_=stamp_sec;last_prediction_dt_=dt;++age_;
      return;
    }
    if (usesMmkNet() && (!std::isfinite(dt) || dt <= 0.0 ||
        std::abs(dt - mmknet_config_.sample_period) >
        mmknet_config_.sample_period * mmknet_config_.period_tolerance)) {
      mmknet_history_.clear();
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 0.01;
    }
    // Preserve the old clamp for analytic mode. The network transition must
    // use actual elapsed time, including checkpoints evaluated at dt=1 s.
    if (!usesMmkNet()) dt = std::clamp(dt, 0.005, 0.5);

    if (usesMmkNet()) {
      network_prior_state_ = state_;
      network_prior_covariance_ = covariance_;
      network_prior_valid_ = true;
      propagateMmkNet(dt, mmknet_history_.ready() ? mmknet_history_.turnRate() : 0.0);
      last_stamp_sec_ = stamp_sec;
      last_prediction_dt_ = dt;
      ++age_;
      return;
    }

    const Eigen::Vector3d position = state_.head<3>();
    const Eigen::Vector3d velocity = state_.tail<3>();

    model_predictions_[0] = position + velocity * dt;
    model_velocities_[0] = velocity;

    model_predictions_[1] = position + velocity * dt + 0.5 * acceleration_ * dt * dt;
    model_velocities_[1] = velocity + acceleration_ * dt;

    Eigen::Vector3d turn_velocity = velocity;
    const double angle = turn_rate_ * dt;
    if (std::abs(angle) > 1e-5) {
      const double cosine = std::cos(angle);
      const double sine = std::sin(angle);
      turn_velocity.x() = cosine * velocity.x() - sine * velocity.y();
      turn_velocity.y() = sine * velocity.x() + cosine * velocity.y();
    }
    model_predictions_[2] = position + 0.5 * (velocity + turn_velocity) * dt;
    model_velocities_[2] = turn_velocity;

    // A small transition probability prevents one historical model from
    // permanently suppressing alternatives before an abrupt manoeuvre.
    std::array<double, 3> transitioned{};
    for (std::size_t i = 0; i < model_probabilities_.size(); ++i) {
      transitioned[i] = 0.88 * model_probabilities_[i] +
        0.06 * (1.0 - model_probabilities_[i]);
    }
    model_probabilities_ = transitioned;
    normalizeModelProbabilities();

    // Keep the posterior propagation conservative and deterministic. CA and CT
    // remain real association hypotheses, but an uncertain manoeuvre model is
    // not allowed to drag the published track without a lidar measurement.
    state_.head<3>() = model_predictions_[0];
    state_.tail<3>() = model_velocities_[0];

    Matrix6d transition = Matrix6d::Identity();
    transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Eigen::Matrix<double, 6, 3> noise_gain;
    noise_gain.setZero();
    noise_gain.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
    noise_gain.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
    const Eigen::Matrix3d acceleration_covariance =
      Eigen::Matrix3d::Identity() * acceleration_noise_ * acceleration_noise_;
    covariance_ = transition * covariance_ * transition.transpose() +
      noise_gain * acceleration_covariance * noise_gain.transpose();

    last_stamp_sec_ = stamp_sec;
    last_prediction_dt_ = dt;
    ++age_;
  }

  void update(
    const Eigen::Vector3d & position, const Eigen::Vector3d & size,
    bool allow_size_update = true, bool allow_network_observation = true)
  {
    association_position_ = position;
    if (usesUpstream()) {
      upstream_.apply("update",{position.x(),position.y(),position.z(),last_stamp_sec_});syncUpstream();
      // Keep the measured ground-referenced height; the upstream bank supplies XY motion.
      const double vertical_gain=covariance_(2,2)/(covariance_(2,2)+measurement_noise_*measurement_noise_);
      state_(2)+=vertical_gain*(position.z()-state_(2));covariance_(2,2)*=(1.-vertical_gain);
      if (adaptive_size_enabled_ && allow_size_update) updateSize(size);
      ++hits_;missed_=0;return;
    }
    if (usesImm()) {
      imm_.update(position.head<2>());
    } else if (usesMmkNet()) {
      if (allow_network_observation && network_prior_valid_) {
        const Eigen::Vector2d residual = position.head<2>() - state_.head<2>();
        if (mmknet_history_.observe(position.head<2>(), residual, last_stamp_sec_, mmknet_config_)) {
          // Like MMKNet/filter.py: infer omega with the current innovation,
          // then redo this frame's prediction from the same previous posterior.
          state_ = network_prior_state_;
          covariance_ = network_prior_covariance_;
          propagateMmkNet(last_prediction_dt_, mmknet_history_.turnRate());
          mmknet_history_.correctResidual(position.head<2>() - state_.head<2>());
        }
      } else {
        mmknet_history_.clear();
      }
      network_prior_valid_ = false;
    } else {
      updateModelProbabilities(position);
    }

    Matrix3x6d observation = Matrix3x6d::Zero();
    observation.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d measurement_covariance =
      Eigen::Matrix3d::Identity() * measurement_noise_ * measurement_noise_;
    const Eigen::Matrix3d innovation_covariance =
      observation * covariance_ * observation.transpose() + measurement_covariance;
    const Eigen::Matrix<double, 6, 3> gain =
      covariance_ * observation.transpose() * innovation_covariance.inverse();

    state_ += gain * (position - observation * state_);
    covariance_ = (Matrix6d::Identity() - gain * observation) * covariance_;
    if (usesImm()) syncImm();
    if (adaptive_size_enabled_ && allow_size_update) {
      updateSize(size);
    }
    ++hits_;
    missed_ = 0;
  }

  MeasurementUpdate updateWithMeasurement(
    const Eigen::Vector3d & position, const Eigen::Vector3d & size, double stamp_sec,
    double velocity_blend, double minimum_displacement, double maximum_dt,
    double maximum_speed, double acceleration_smoothing = 0.45,
    double turn_rate_smoothing = 0.45, double maximum_acceleration = 8.0,
    double maximum_turn_rate = 4.0, bool allow_size_update = true,
    bool allow_network_observation = true)
  {
    MeasurementUpdate result;
    result.dt = stamp_sec - last_measurement_stamp_sec_;
    result.displacement =
      (position.head<2>() - last_measurement_position_.head<2>()).norm();
    const Eigen::Vector3d velocity_before_update = state_.tail<3>();

    const bool plausible_motion = std::isfinite(result.dt) && result.dt > 0.0 &&
      result.displacement / result.dt <= 1.25 * maximum_speed;
    update(position, size, allow_size_update, allow_network_observation && plausible_motion);
    if (usesImm() || usesUpstream()) {
      // IMM estimates velocity/acceleration/turn rate from measurement
      // innovations. Do not inject finite-difference velocities a second time.
      limitHorizontalSpeed(maximum_speed);
      last_measurement_position_=position;last_measurement_stamp_sec_=stamp_sec;
      return result;
    }
    if (
      std::isfinite(result.dt) && result.dt >= 0.02 && result.dt <= maximum_dt &&
      result.displacement >= minimum_displacement)
    {
      Eigen::Vector2d measured_velocity =
        (position.head<2>() - last_measurement_position_.head<2>()) / result.dt;
      result.measured_speed = measured_velocity.norm();
      // A single wrong identity association can imply an impossible 10-20 m/s
      // lidar velocity. Correct position through the EKF, but never teach that
      // outlier to the acceleration/turn models or velocity state.
      if (result.measured_speed <= 1.25 * maximum_speed) {
        if (result.measured_speed > maximum_speed && result.measured_speed > 1e-6) {
          measured_velocity *= maximum_speed / result.measured_speed;
        }

        acceleration_smoothing = std::clamp(acceleration_smoothing, 0.0, 1.0);
        Eigen::Vector2d measured_acceleration =
          (measured_velocity - velocity_before_update.head<2>()) / result.dt;
        const double acceleration_norm = measured_acceleration.norm();
        if (acceleration_norm > maximum_acceleration && acceleration_norm > 1e-6) {
          measured_acceleration *= maximum_acceleration / acceleration_norm;
        }
        acceleration_.head<2>() =
          (1.0 - acceleration_smoothing) * acceleration_.head<2>() +
          acceleration_smoothing * measured_acceleration;

        if (has_observed_velocity_) {
          const double previous_speed = last_observed_velocity_.norm();
          const double current_speed = measured_velocity.norm();
          if (previous_speed > 0.10 && current_speed > 0.10) {
            const double cross =
              last_observed_velocity_.x() * measured_velocity.y() -
              last_observed_velocity_.y() * measured_velocity.x();
            const double dot = last_observed_velocity_.dot(measured_velocity);
            double measured_turn_rate = std::atan2(cross, dot) / result.dt;
            measured_turn_rate = std::clamp(
              measured_turn_rate, -maximum_turn_rate, maximum_turn_rate);
            turn_rate_smoothing = std::clamp(turn_rate_smoothing, 0.0, 1.0);
            turn_rate_ = (1.0 - turn_rate_smoothing) * turn_rate_ +
              turn_rate_smoothing * measured_turn_rate;
          }
        }
        last_observed_velocity_ = measured_velocity;
        has_observed_velocity_ = true;
        blendHorizontalVelocity(measured_velocity, velocity_blend);
        result.velocity_valid = true;
      }
    }

    limitHorizontalSpeed(maximum_speed);
    last_measurement_position_ = position;
    last_measurement_stamp_sec_ = stamp_sec;
    return result;
  }

  // Association uses only the last accepted observation, never propagated state.
  Eigen::Vector3d associationPosition() const {return association_position_;}
  double associationDistance(const Eigen::Vector3d & position) const
  {
    const Eigen::Vector3d offset = position - association_position_;
    return std::sqrt(offset.x()*offset.x()+offset.y()*offset.y()+0.25*offset.z()*offset.z());
  }
  std::vector<Eigen::Vector3d> motionSeeds(double = 0.35) const
  {
    return {association_position_};
  }

  void snapPosition(const Eigen::Vector3d & position)
  {
    if (usesMmkNet() && (state_.head<2>() - position.head<2>()).norm() > 1e-6) {
      mmknet_history_.clear();
      network_prior_valid_ = false;
    }
    if (usesUpstream()) {upstream_.apply("snap",{position.x(),position.y(),position.z()});state_(2)=position.z();syncUpstream();return;}
    if (usesImm()) imm_.snap(position.head<2>());
    state_.head<3>() = position;
    covariance_.block<3, 3>(0, 0).diagonal().array() =
      std::max(measurement_noise_ * measurement_noise_, 0.0025);
    if (usesImm()) syncImm();
  }

  // A semantic relock is not an ordinary innovation: the previous posterior
  // may belong to a wrong lidar component. Re-anchor position, uncertainty and
  // motion models while preserving the externally visible target ID.
  void resetFromMeasurement(
    const Eigen::Vector3d & position, const Eigen::Vector3d & size,
    double stamp_sec, bool allow_size_update = true)
  {
    mmknet_history_.clear();
    network_prior_valid_ = false;
    state_.setZero();
    state_.head<3>() = position;
    association_position_ = position;
    covariance_.setZero();
    covariance_.diagonal() << 0.09, 0.09, 0.09, 1.0, 1.0, 1.0;
    if (adaptive_size_enabled_ && allow_size_update) size_ = canonicalSize(size);
    acceleration_.setZero();
    last_observed_velocity_.setZero();
    has_observed_velocity_ = false;
    turn_rate_ = 0.0;
    model_probabilities_ = {{0.45, 0.30, 0.25}};
    model_predictions_.fill(position);
    model_velocities_.fill(Eigen::Vector3d::Zero());
    last_stamp_sec_ = stamp_sec;
    last_measurement_position_ = position;
    last_measurement_stamp_sec_ = stamp_sec;
    last_prediction_dt_ = 0.01;
    missed_ = 0;
    if (usesUpstream()) {upstream_.initialize(upstream_mode_,position,size,stamp_sec);syncUpstream();}
    if (usesImm()) {imm_.initialize(position.head<2>(),acceleration_noise_,measurement_noise_,imm_config_);syncImm();}
    ++hits_;
  }

  void dampVelocity(double factor)
  {
    factor = std::clamp(factor, 0.0, 1.0);
    if (usesUpstream()) {upstream_.apply("damp",{factor});syncUpstream();return;}
    if (usesImm()) {imm_.damp(factor);syncImm();return;}
    state_.tail<3>() *= factor;
    acceleration_ *= factor;
    turn_rate_ *= factor;
  }

  void limitHorizontalSpeed(double maximum_speed)
  {
    maximum_speed = std::max(0.0, maximum_speed);
    if (usesUpstream()) {upstream_.apply("limit",{maximum_speed});syncUpstream();return;}
    if (usesImm()) {imm_.limitSpeed(maximum_speed);syncImm();return;}
    auto horizontal_velocity = state_.segment<2>(3);
    const double speed = horizontal_velocity.norm();
    if (speed > maximum_speed && speed > 1e-6) {
      horizontal_velocity *= maximum_speed / speed;
    }
  }

  void blendHorizontalVelocity(const Eigen::Vector2d & measured_velocity, double factor)
  {
    factor = std::clamp(factor, 0.0, 1.0);
    if (usesUpstream()) {upstream_.apply("blend",{measured_velocity.x(),measured_velocity.y(),factor});syncUpstream();return;}
    if (usesImm()) {imm_.blendVelocity(measured_velocity,factor);syncImm();return;}
    state_.segment<2>(3) =
      (1.0 - factor) * state_.segment<2>(3) + factor * measured_velocity;
  }

  // Read-only rollout: never advance the live filter or its neural history.
  Eigen::Vector3d futurePosition(double seconds) const
  {
    const double t = std::max(0.0, seconds);
    if (usesUpstream()) {auto f=upstream_;f.apply("future",{t});auto p=position();p.head<2>()=f.state.head<2>();return p;}
    if (usesImm()) {
      auto future=imm_;future.predict(t);
      Eigen::Vector3d result=position();result.head<2>()=future.state().head<2>();return result;
    }
    const double omega = usesMmkNet() && mmknetReady() ? mmknet_history_.turnRate() : 0.0;
    const double angle = omega * t;
    const double a = std::abs(angle) < 1e-4 ?
      t * (1.0 - angle * angle / 6.0) : std::sin(angle) / omega;
    const double b = std::abs(angle) < 1e-4 ?
      t * (0.5 * angle - angle * angle * angle / 24.0) : (1.0 - std::cos(angle)) / omega;
    Eigen::Vector3d result = position();
    const auto v = velocity();
    result.x() += a * v.x() - b * v.y();
    result.y() += b * v.x() + a * v.y();
    // Unknown future terrain is not extrapolated from a noisy vertical velocity.
    return result;
  }

  double futureYaw(double seconds) const
  {
    if (usesUpstream()) {auto f=upstream_;f.apply("future",{std::max(0.0,seconds)});return std::atan2(f.state(3),f.state(2));}
    if (usesImm()) {
      auto future=imm_;future.predict(std::max(0.0,seconds));
      const auto v=future.state().segment<2>(2);
      return v.norm()>1e-3 ? std::atan2(v.y(),v.x()) : 0.;
    }
    const auto v = velocity();
    const double omega = usesMmkNet() && mmknetReady() ? mmknet_history_.turnRate() : 0.0;
    return v.head<2>().norm() > 1e-3 ?
      std::atan2(v.y(), v.x()) + omega * std::max(0.0, seconds) : 0.0;
  }

  int id() const {return id_;}
  int age() const {return age_;}
  int hits() const {return hits_;}
  int missed() const {return missed_;}
  void markMissed()
  {
    ++missed_;
    mmknet_history_.clear();
    network_prior_valid_ = false;
  }

  Eigen::Vector3d position() const {return state_.head<3>();}
  Eigen::Vector3d velocity() const {return state_.tail<3>();}
  const Matrix6d & covariance() const {return covariance_;}
  const Eigen::Vector3d & size() const {return size_;}
  const Eigen::Vector3d & lastMeasurementPosition() const {return last_measurement_position_;}
  double lastMeasurementStamp() const {return last_measurement_stamp_sec_;}
  double stateStamp() const {return last_stamp_sec_;}
  const std::array<double, 3> & modelProbabilities() const {return model_probabilities_;}
  double turnRate() const {return usesMmkNet() ? mmknet_history_.turnRate() : turn_rate_;}
  const Eigen::Vector3d & acceleration() const {return acceleration_;}

private:
  std::string upstream_mode_{defaultUpstreamMode()};
  ImmMotFilter upstream_;
  void syncUpstream() {
    state_.head<2>()=upstream_.state.head<2>();state_.segment<2>(3)=upstream_.state.segment<2>(2);
    acceleration_.head<2>()=upstream_.state.segment<2>(4);turn_rate_=upstream_.state(6);
    const int indices[]={0,1,3,4};
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)covariance_(indices[i],indices[j])=upstream_.covariance(i,j);
    model_predictions_.fill(position());model_velocities_.fill(velocity());
    // One fused association hypothesis. Four mode weights are published separately.
    model_probabilities_={{1,0,0}};
  }
  void syncImm()
  {
    state_.head<2>()=imm_.state().head<2>();
    state_.segment<2>(3)=imm_.state().segment<2>(2);
    acceleration_.head<2>()=imm_.state().segment<2>(4);turn_rate_=imm_.state()(6);
    const std::array<int,4> index{{0,1,3,4}};
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) covariance_(index[i],index[j])=imm_.covariance()(i,j);
    model_probabilities_=imm_.probabilities();
    for(int i=0;i<3;++i) {
      model_predictions_[i]=position();model_predictions_[i].head<2>()=imm_.modes()[i].x.head<2>();
      model_velocities_[i]=velocity();model_velocities_[i].head<2>()=imm_.modes()[i].x.segment<2>(2);
    }
  }
  bool imm_enabled_{defaultImmEnabled()};
  ImmFilter::Config imm_config_{defaultImmConfig()};
  ImmFilter imm_;
  // XY coordinated-turn transition with a numerically stable CV limit.
  // Omega is held fixed for covariance propagation; the network does not
  // estimate uncertainty in omega. Z remains constant-velocity + ground updates.
  void propagateMmkNet(double dt, double omega)
  {
    const double angle = omega * dt;
    const double sine = std::sin(angle), cosine = std::cos(angle);
    const double a = std::abs(angle) < 1e-4 ?
      dt * (1.0 - angle * angle / 6.0) : sine / omega;
    const double b = std::abs(angle) < 1e-4 ?
      dt * (0.5 * angle - angle * angle * angle / 24.0) : (1.0 - cosine) / omega;
    Matrix6d transition = Matrix6d::Identity();
    transition(0, 3) = a;
    transition(0, 4) = -b;
    transition(1, 3) = b;
    transition(1, 4) = a;
    transition(2, 5) = dt;
    transition(3, 3) = cosine;
    transition(3, 4) = -sine;
    transition(4, 3) = sine;
    transition(4, 4) = cosine;
    state_ = (transition * state_).eval();
    Eigen::Matrix<double, 6, 3> noise_gain = Eigen::Matrix<double, 6, 3>::Zero();
    noise_gain.topRows<3>() = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
    noise_gain.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt;
    covariance_ = transition * covariance_ * transition.transpose() +
      acceleration_noise_ * acceleration_noise_ * noise_gain * noise_gain.transpose();
    // Existing crossing code expects three slots. In network mode these all
    // describe the same learned CT prediction, not three independent models.
    model_predictions_.fill(state_.head<3>());
    model_velocities_.fill(state_.tail<3>());
    model_probabilities_ = mmknet_history_.ready() ?
      std::array<double, 3>{{0.0, 0.0, 1.0}} : std::array<double, 3>{{1.0, 0.0, 0.0}};
  }

  static Eigen::Vector3d canonicalSize(const Eigen::Vector3d & size)
  {
    Eigen::Vector3d canonical = size.cwiseAbs().cwiseMax(Eigen::Vector3d::Constant(1e-3));
    if (canonical.x() > canonical.y()) {
      std::swap(canonical.x(), canonical.y());
    }
    return canonical;
  }

  void updateSize(const Eigen::Vector3d & measurement)
  {
    const Eigen::Vector3d observed = canonicalSize(measurement);
    const Eigen::Vector3d maximum_change =
      size_.cwiseMax(Eigen::Vector3d::Constant(1e-3)) * size_max_relative_step_;
    const Eigen::Vector3d bounded = observed.cwiseMax(size_ - maximum_change).cwiseMin(
      size_ + maximum_change);
    size_ = (1.0 - size_smoothing_) * size_ + size_smoothing_ * bounded;
  }

  void normalizeModelProbabilities()
  {
    double sum = 0.0;
    for (double & probability : model_probabilities_) {
      probability = std::max(1e-4, probability);
      sum += probability;
    }
    for (double & probability : model_probabilities_) {
      probability /= sum;
    }
  }

  void updateModelProbabilities(const Eigen::Vector3d & measurement)
  {
    const double sigma = std::max(
      0.08, measurement_noise_ + 0.5 * acceleration_noise_ *
      last_prediction_dt_ * last_prediction_dt_);
    const double inverse_two_variance = 0.5 / (sigma * sigma);
    for (std::size_t i = 0; i < model_predictions_.size(); ++i) {
      const Eigen::Vector3d residual = measurement - model_predictions_[i];
      const double squared_distance =
        residual.x() * residual.x() + residual.y() * residual.y() +
        0.25 * residual.z() * residual.z();
      const double likelihood = std::max(
        1e-6, std::exp(-squared_distance * inverse_two_variance));
      model_probabilities_[i] *= likelihood;
    }
    normalizeModelProbabilities();
  }

  MmkNetConfig mmknet_config_{defaultMmkNetConfig()};
  MmkNetHistory mmknet_history_;
  Eigen::Matrix<double, 6, 1> network_prior_state_{Eigen::Matrix<double, 6, 1>::Zero()};
  Matrix6d network_prior_covariance_{Matrix6d::Identity()};
  bool network_prior_valid_{false};

  int id_{0};
  int age_{1};
  int hits_{1};
  int missed_{0};
  Eigen::Matrix<double, 6, 1> state_{Eigen::Matrix<double, 6, 1>::Zero()};
  Matrix6d covariance_{Matrix6d::Identity()};
  Eigen::Vector3d size_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d association_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d last_observed_velocity_{Eigen::Vector2d::Zero()};
  bool has_observed_velocity_{false};
  double turn_rate_{0.0};
  std::array<double, 3> model_probabilities_{{0.45, 0.30, 0.25}};
  std::array<Eigen::Vector3d, 3> model_predictions_;
  std::array<Eigen::Vector3d, 3> model_velocities_;
  double last_stamp_sec_{0.0};
  double last_prediction_dt_{0.1};
  Eigen::Vector3d last_measurement_position_{Eigen::Vector3d::Zero()};
  double last_measurement_stamp_sec_{0.0};
  double acceleration_noise_{2.0};
  double measurement_noise_{0.15};
  bool adaptive_size_enabled_{true};
  double size_smoothing_{0.12};
  double size_max_relative_step_{0.25};
};

}  // namespace person_tracker
