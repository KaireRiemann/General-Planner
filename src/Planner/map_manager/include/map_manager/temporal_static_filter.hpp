#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <rog_map/rog_map.h>

namespace general_planner {

/**
 * Promotes a point to the static-map stream only after its spatial voxel has
 * been observed in distinct cloud frames for a continuous time interval.
 *
 * This gates persistent hits, never raw free-space rays or current obstacles.
 * Temporal persistence is not semantic classification: an object that stops
 * can be promoted, so measured free-space evidence must also remove old hits.
 */
class TemporalStaticFilter {
 public:
  struct Config {
    bool enabled{false};
    double voxel_size{0.30};
    int min_observations{4};
    double min_observation_span{0.30};
    double min_observer_baseline{0.0};  // Legacy compatibility; not a gate.
    double max_observation_gap{0.60};
    std::size_t max_voxels{250000};
  };

  void configure(Config config) {
    config.voxel_size = std::max(0.01, config.voxel_size);
    config.min_observations = std::max(1, config.min_observations);
    config.min_observation_span = std::max(0.0, config.min_observation_span);
    config.min_observer_baseline =
        std::max(0.0, config.min_observer_baseline);
    config.max_observation_gap = std::max(0.01, config.max_observation_gap);
    config.max_voxels = std::max<std::size_t>(1, config.max_voxels);
    config_ = config;
    reset();
  }

  void reset() {
    observations_.clear();
    last_observation_stamp_ = std::numeric_limits<double>::quiet_NaN();
    synthetic_stamp_ = 0.0;
  }

  rog_map::PointCloud filter(
      const rog_map::PointCloud& input, const double observation_stamp,
      const rog_map::Vec3f* observer_position = nullptr) {
    if (!config_.enabled) {
      return input;
    }

    bool timestamp_valid = std::isfinite(observation_stamp);
    double stamp = observation_stamp;
    if (timestamp_valid && std::isfinite(last_observation_stamp_) &&
        stamp + 1.0e-3 < last_observation_stamp_) {
      // A replay or clock reset must not turn old evidence into a new
      // observation sequence.
      reset();
    }
    if (timestamp_valid) {
      last_observation_stamp_ = stamp;
    } else {
      stamp = ++synthetic_stamp_;
    }

    prune(stamp, timestamp_valid);
    const bool observer_valid =
        observer_position != nullptr && observer_position->allFinite();

    std::unordered_set<VoxelKey, VoxelKeyHash> seen_this_frame;
    seen_this_frame.reserve(input.size());
    for (const auto& point : input.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const VoxelKey key = keyFor(point);
      if (!seen_this_frame.insert(key).second) {
        continue;
      }

      const auto found = observations_.find(key);
      if (found == observations_.end()) {
        if (observations_.size() >= config_.max_voxels) {
          continue;
        }
        observations_.emplace(
            key, makeObservation(stamp, observer_position, observer_valid));
        continue;
      }

      Observation& observation = found->second;
      if (timestamp_valid &&
          stamp - observation.last_stamp > config_.max_observation_gap) {
        observation =
            makeObservation(stamp, observer_position, observer_valid);
      } else {
        ++observation.count;
        observation.last_stamp = stamp;
        updateObserverBaseline(observation, observer_position, observer_valid);
      }
    }

    rog_map::PointCloud output;
    output.header = input.header;
    output.is_dense = input.is_dense;
    output.points.reserve(input.points.size());
    for (const auto& point : input.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const auto found = observations_.find(keyFor(point));
      if (found == observations_.end()) {
        continue;
      }
      const Observation& observation = found->second;
      const bool sufficient_duration =
          !timestamp_valid || stamp - observation.first_stamp >=
                                  config_.min_observation_span;
      // Observer motion is not evidence of object stationarity and cannot be
      // a prerequisite for building the map needed to plan that motion.
      // Keep the legacy configuration readable, but do not gate promotion on it.
      if (observation.count >= config_.min_observations &&
          sufficient_duration) {
        output.points.push_back(point);
      }
    }
    output.width = static_cast<std::uint32_t>(output.points.size());
    output.height = 1;
    return output;
  }

 private:
  struct VoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
      const auto mix = [](std::uint64_t value) {
        value ^= value >> 30U;
        value *= 0xbf58476d1ce4e5b9ULL;
        value ^= value >> 27U;
        value *= 0x94d049bb133111ebULL;
        return value ^ (value >> 31U);
      };
      const std::uint64_t x = mix(static_cast<std::uint64_t>(key.x));
      const std::uint64_t y = mix(static_cast<std::uint64_t>(key.y));
      const std::uint64_t z = mix(static_cast<std::uint64_t>(key.z));
      return static_cast<std::size_t>(x ^ (y << 1U) ^ (z << 7U));
    }
  };

  struct Observation {
    int count;
    double first_stamp;
    double last_stamp;
    rog_map::Vec3f first_observer_position{rog_map::Vec3f::Zero()};
    double max_observer_baseline{0.0};
    bool has_observer_position{false};
  };

  static Observation makeObservation(
      const double stamp, const rog_map::Vec3f* observer_position,
      const bool observer_valid) {
    Observation observation;
    observation.count = 1;
    observation.first_stamp = stamp;
    observation.last_stamp = stamp;
    if (observer_valid) {
      observation.first_observer_position = *observer_position;
      observation.has_observer_position = true;
    }
    return observation;
  }

  static void updateObserverBaseline(
      Observation& observation, const rog_map::Vec3f* observer_position,
      const bool observer_valid) {
    if (!observer_valid) {
      return;
    }
    if (!observation.has_observer_position) {
      observation.first_observer_position = *observer_position;
      observation.has_observer_position = true;
      return;
    }
    observation.max_observer_baseline = std::max(
        observation.max_observer_baseline,
        (*observer_position - observation.first_observer_position).norm());
  }

  VoxelKey keyFor(const rog_map::PclPoint& point) const {
    return VoxelKey{
        static_cast<std::int64_t>(std::floor(point.x / config_.voxel_size)),
        static_cast<std::int64_t>(std::floor(point.y / config_.voxel_size)),
        static_cast<std::int64_t>(std::floor(point.z / config_.voxel_size))};
  }

  void prune(const double stamp, const bool timestamp_valid) {
    if (!timestamp_valid) {
      return;
    }
    for (auto it = observations_.begin(); it != observations_.end();) {
      if (stamp - it->second.last_stamp > config_.max_observation_gap) {
        it = observations_.erase(it);
      } else {
        ++it;
      }
    }
  }

  Config config_;
  std::unordered_map<VoxelKey, Observation, VoxelKeyHash> observations_;
  double last_observation_stamp_{std::numeric_limits<double>::quiet_NaN()};
  double synthetic_stamp_{0.0};
};

}  // namespace general_planner
