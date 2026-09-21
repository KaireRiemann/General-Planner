#pragma once

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <Eigen/Core>

namespace person_tracker
{

// Immutable weights shared by tracks; feature histories belong to each filter.
class MmkNet
{
public:
  using Feature = Eigen::Matrix<float, 1, 6>;
  explicit MmkNet(const std::string & path);
  double infer(const std::deque<Feature> & history) const;

private:
  Eigen::MatrixXf linear(const Eigen::MatrixXf & x, const std::string & name) const;
  Eigen::MatrixXf norm(const Eigen::MatrixXf & x, const std::string & name) const;
  std::unordered_map<std::string, Eigen::MatrixXf> weights_;
};

struct MmkNetConfig
{
  std::shared_ptr<const MmkNet> network;
  double sample_period{0.1};
  double period_tolerance{0.35};
  double maximum_turn_rate{1.0};
};

// Copyable state: crossing hypotheses must never share observation histories.
class MmkNetHistory
{
public:
  void clear();
  bool observe(const Eigen::Vector2d & measurement, const Eigen::Vector2d & residual,
    double stamp, const MmkNetConfig & config);
  void correctResidual(const Eigen::Vector2d & residual)
  {
    if (!features_.empty()) features_.back().head<2>() = residual.cast<float>().transpose();
  }
  double turnRate() const {return turn_rate_;}
  bool ready() const {return ready_;}
  std::size_t samples() const {return features_.size();}

private:
  std::deque<MmkNet::Feature> features_;
  Eigen::Vector2d last_measurement_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_difference_{Eigen::Vector2d::Zero()};
  double last_stamp_{-1.0};
  double turn_rate_{0.0};
  bool has_difference_{false};
  bool ready_{false};
};
}  // namespace person_tracker
