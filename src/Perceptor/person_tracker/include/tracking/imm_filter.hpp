#pragma once
#include <array>
#include <Eigen/Dense>

namespace person_tracker {
// Common Cartesian embedding [x,y,vx,vy,ax,ay,omega]. CT is constant speed
// and yaw rate in Cartesian coordinates (no heading singularity at zero speed).
class ImmFilter {
public:
  using State = Eigen::Matrix<double,7,1>;
  using Covariance = Eigen::Matrix<double,7,7>;
  struct Mode {State x{State::Zero()}; Covariance p{Covariance::Identity()};};
  struct Config {double jerk_noise{4.0}; double turn_noise{0.6}; double stay_probability{0.94};};
  void initialize(const Eigen::Vector2d & position, double acceleration_noise,
                  double measurement_noise, const Config & config);
  void predict(double dt);
  void update(const Eigen::Vector2d & measurement);
  void snap(const Eigen::Vector2d & position);
  void damp(double factor);
  void limitSpeed(double maximum);
  void blendVelocity(const Eigen::Vector2d & velocity, double factor);
  const State & state() const {return mean_;}
  const Covariance & covariance() const {return covariance_;}
  const std::array<Mode,3> & modes() const {return modes_;}
  const std::array<double,3> & probabilities() const {return probabilities_;}
  static State transition(const State & x, double dt, int mode);
private:
  void step(double dt);
  void combine();
  std::array<Mode,3> modes_;
  std::array<double,3> probabilities_{{0.45,0.30,0.25}};
  State mean_{State::Zero()};
  Covariance covariance_{Covariance::Identity()};
  double acceleration_noise_{2.0}, measurement_noise_{0.15};
  Config config_;
};
}
