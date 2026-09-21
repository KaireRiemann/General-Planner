#include "tracking/mmknet.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace person_tracker
{
namespace
{
std::uint32_t readU32(std::ifstream & stream)
{
  unsigned char bytes[4];
  if (!stream.read(reinterpret_cast<char *>(bytes), 4)) {
    throw std::runtime_error("Truncated MMKNet weights");
  }
  return std::uint32_t(bytes[0]) | (std::uint32_t(bytes[1]) << 8) |
    (std::uint32_t(bytes[2]) << 16) | (std::uint32_t(bytes[3]) << 24);
}
}

MmkNet::MmkNet(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  char magic[8];
  if (!stream.read(magic, 8) || std::string(magic, 8) != "MMKNET01") {
    throw std::runtime_error("Cannot load MMKNet weights: " + path);
  }
  const auto count = readU32(stream);
  if (count > 100) throw std::runtime_error("Invalid MMKNet tensor count");
  for (std::uint32_t i = 0; i < count; ++i) {
    const auto length = readU32(stream);
    if (length == 0 || length > 200) throw std::runtime_error("Invalid tensor name");
    std::string name(length, '\0');
    if (!stream.read(name.data(), length)) throw std::runtime_error("Truncated tensor name");
    const auto rows = readU32(stream), cols = readU32(stream);
    if (rows == 0 || cols == 0 || rows > 5000 || cols > 5000 || rows * cols > 1000000) {
      throw std::runtime_error("Invalid MMKNet tensor dimensions");
    }
    Eigen::MatrixXf value(rows, cols);
    for (std::uint32_t row = 0; row < rows; ++row) {
      for (std::uint32_t col = 0; col < cols; ++col) {
        // Export is explicitly little endian float32.
        const auto bits = readU32(stream);
        float number;
        static_assert(sizeof(number) == sizeof(bits), "float32 required");
        std::memcpy(&number, &bits, sizeof(number));
        value(row, col) = number;
      }
    }
    if (!value.allFinite() || !weights_.emplace(name, value).second) {
      throw std::runtime_error("Invalid/duplicate MMKNet tensor: " + name);
    }
  }
  const auto require = [&](const std::string & name, int rows, int cols) {
    const auto found = weights_.find(name);
    if (found == weights_.end() || found->second.rows() != rows || found->second.cols() != cols) {
      throw std::runtime_error("MMKNet architecture mismatch: " + name);
    }
  };
  require("input_proj.weight", 64, 6);
  require("input_proj.bias", 1, 64);
  require("pos_encoder.pe", 500, 64);
  for (int layer = 0; layer < 2; ++layer) {
    const std::string prefix = "encoder.layers." + std::to_string(layer) + ".";
    require(prefix + "self_attn.in_proj_weight", 192, 64);
    require(prefix + "self_attn.in_proj_bias", 1, 192);
    require(prefix + "self_attn.out_proj.weight", 64, 64);
    require(prefix + "self_attn.out_proj.bias", 1, 64);
    require(prefix + "linear1.weight", 256, 64);
    require(prefix + "linear1.bias", 1, 256);
    require(prefix + "linear2.weight", 64, 256);
    require(prefix + "linear2.bias", 1, 64);
    for (const auto & norm_name : {"norm1", "norm2"}) {
      require(prefix + norm_name + ".weight", 1, 64);
      require(prefix + norm_name + ".bias", 1, 64);
    }
  }
  require("fc_out.0.weight", 32, 64);
  require("fc_out.0.bias", 1, 32);
  require("fc_out.2.weight", 1, 32);
  require("fc_out.2.bias", 1, 1);
}

Eigen::MatrixXf MmkNet::linear(const Eigen::MatrixXf & x, const std::string & name) const
{
  Eigen::MatrixXf result = x * weights_.at(name + ".weight").transpose();
  result.rowwise() += weights_.at(name + ".bias").row(0);
  return result;
}

Eigen::MatrixXf MmkNet::norm(const Eigen::MatrixXf & x, const std::string & name) const
{
  Eigen::MatrixXf result = x;
  for (Eigen::Index row = 0; row < x.rows(); ++row) {
    const float mean = x.row(row).mean();
    const float variance = (x.row(row).array() - mean).square().mean();
    result.row(row) = (((x.row(row).array() - mean) / std::sqrt(variance + 1e-5f)) *
      weights_.at(name + ".weight").row(0).array() +
      weights_.at(name + ".bias").row(0).array()).matrix();
  }
  return result;
}

double MmkNet::infer(const std::deque<Feature> & history) const
{
  if (history.size() != 8) throw std::runtime_error("MMKNet requires eight feature frames");
  Eigen::MatrixXf input(8, 6);
  for (int i = 0; i < 8; ++i) input.row(i) = history[i];
  if (!input.allFinite()) throw std::runtime_error("Non-finite MMKNet input");
  Eigen::MatrixXf x = linear(input, "input_proj") + weights_.at("pos_encoder.pe").topRows(8);
  for (int layer = 0; layer < 2; ++layer) {
    const std::string prefix = "encoder.layers." + std::to_string(layer) + ".";
    Eigen::MatrixXf qkv = x * weights_.at(prefix + "self_attn.in_proj_weight").transpose();
    qkv.rowwise() += weights_.at(prefix + "self_attn.in_proj_bias").row(0);
    Eigen::MatrixXf attention(8, 64);
    for (int head = 0; head < 4; ++head) {
      Eigen::MatrixXf scores = qkv.middleCols(head * 16, 16) *
        qkv.middleCols(64 + head * 16, 16).transpose() / 4.0f;
      for (int row = 0; row < 8; ++row) {
        scores.row(row) = (scores.row(row).array() - scores.row(row).maxCoeff()).exp().matrix();
        scores.row(row) /= scores.row(row).sum();
      }
      attention.middleCols(head * 16, 16) = scores * qkv.middleCols(128 + head * 16, 16);
    }
    x = norm(x + linear(attention, prefix + "self_attn.out_proj"), prefix + "norm1");
    const Eigen::MatrixXf hidden = linear(x, prefix + "linear1").cwiseMax(0.0f);
    x = norm(x + linear(hidden, prefix + "linear2"), prefix + "norm2");
  }
  const Eigen::MatrixXf hidden = linear(x.bottomRows(1), "fc_out.0").cwiseMax(0.0f);
  const double omega = std::tanh(linear(hidden, "fc_out.2")(0, 0));
  if (!std::isfinite(omega)) throw std::runtime_error("Non-finite MMKNet output");
  return omega;
}

void MmkNetHistory::clear()
{
  *this = MmkNetHistory{};
}

bool MmkNetHistory::observe(
  const Eigen::Vector2d & measurement, const Eigen::Vector2d & residual,
  double stamp, const MmkNetConfig & config)
{
  if (!config.network || !measurement.allFinite() || !residual.allFinite() || !std::isfinite(stamp)) {
    clear();
    return false;
  }
  const double dt = stamp - last_stamp_;
  if (last_stamp_ < 0.0 || dt <= 0.0 ||
      std::abs(dt - config.sample_period) > config.sample_period * config.period_tolerance) {
    clear();
    last_measurement_ = measurement;
    last_stamp_ = stamp;
    return false;
  }
  const Eigen::Vector2d difference = measurement - last_measurement_;
  const Eigen::Vector2d second_difference = difference - last_difference_;
  if (has_difference_) {
    MmkNet::Feature feature;
    feature << residual.x(), residual.y(), difference.x(), difference.y(),
      second_difference.x(), second_difference.y();
    features_.push_back(feature);
    if (features_.size() > 8) features_.pop_front();
  }
  last_measurement_ = measurement;
  last_difference_ = difference;
  last_stamp_ = stamp;
  has_difference_ = true;
  ready_ = features_.size() == 8;
  if (ready_) {
    turn_rate_ = std::clamp(config.network->infer(features_),
      -config.maximum_turn_rate, config.maximum_turn_rate);
  }
  return ready_;
}
}  // namespace person_tracker
