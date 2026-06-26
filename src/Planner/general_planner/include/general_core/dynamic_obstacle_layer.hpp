#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <rog_map/rog_map.h>

namespace general_planner {

class DynamicObstacleLayer {
public:
    struct Config {
        bool enable{false};
        double ttl{0.35};
        double voxel_size{0.20};
        double inflation_radius{0.35};
        double line_check_step{0.05};
        int max_points_per_frame{30000};
        rog_map::Vec3f local_half_size{10.0, 10.0, 3.0};
    };

    struct QueryResult {
        bool occupied{false};
        rog_map::Vec3f hit_pos{rog_map::Vec3f::Zero()};
        std::size_t active_voxel_count{0};
        double newest_age{std::numeric_limits<double>::infinity()};
    };

    void configure(const Config &config) {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg_ = config;
        cfg_.ttl = std::max(0.0, cfg_.ttl);
        cfg_.voxel_size = std::max(1.0e-3, cfg_.voxel_size);
        cfg_.inflation_radius = std::max(0.0, cfg_.inflation_radius);
        cfg_.line_check_step = std::max(1.0e-3, cfg_.line_check_step);
        cfg_.local_half_size = cfg_.local_half_size.cwiseMax(rog_map::Vec3f::Zero());
        clearLocked();
    }

    bool enabled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cfg_.enable;
    }

    double ttl() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cfg_.ttl;
    }

    double lineCheckStep() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cfg_.line_check_step;
    }

    std::size_t activeVoxelCount(const double now) const {
        std::lock_guard<std::mutex> lock(mutex_);
        pruneExpiredLocked(now);
        return active_voxels_.size();
    }

    void updateCloud(const rog_map::PointCloud &cloud,
                     const rog_map::Vec3f &robot_pos,
                     const double stamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!cfg_.enable || !std::isfinite(stamp)) {
            return;
        }

        pruneExpiredLocked(stamp);

        Frame frame;
        frame.stamp = stamp;
        frame.voxels.reserve(cloud.size());

        const int inflate_step =
                static_cast<int>(std::ceil(cfg_.inflation_radius / cfg_.voxel_size));
        const double inflate_radius_sq = cfg_.inflation_radius * cfg_.inflation_radius;
        const double voxel_sq = cfg_.voxel_size * cfg_.voxel_size;

        int accepted_points = 0;
        for (const auto &pcl_p : cloud) {
            if (cfg_.max_points_per_frame > 0 &&
                accepted_points >= cfg_.max_points_per_frame) {
                break;
            }

            const rog_map::Vec3f point(pcl_p.x, pcl_p.y, pcl_p.z);
            if (!point.allFinite() || !insideLocalBox(point, robot_pos)) {
                continue;
            }

            ++accepted_points;
            const VoxelKey base_key = positionToKey(point);
            for (int dx = -inflate_step; dx <= inflate_step; ++dx) {
                for (int dy = -inflate_step; dy <= inflate_step; ++dy) {
                    for (int dz = -inflate_step; dz <= inflate_step; ++dz) {
                        const double offset_sq =
                                static_cast<double>(dx * dx + dy * dy + dz * dz) * voxel_sq;
                        if (offset_sq > inflate_radius_sq + 1.0e-9) {
                            continue;
                        }
                        frame.voxels.insert({base_key.x + dx, base_key.y + dy, base_key.z + dz});
                    }
                }
            }
        }

        if (frame.voxels.empty()) {
            newest_stamp_ = stamp;
            return;
        }

        for (const auto &key : frame.voxels) {
            ++active_voxels_[key];
        }
        newest_stamp_ = stamp;
        frames_.emplace_back(std::move(frame));
    }

    bool pointOccupied(const rog_map::Vec3f &position,
                       const double now,
                       QueryResult *result = nullptr) const {
        std::lock_guard<std::mutex> lock(mutex_);
        QueryResult local;
        local.hit_pos = position;
        if (!freshLocked(now) || !position.allFinite()) {
            fillResult(result, local);
            return false;
        }
        pruneExpiredLocked(now);
        local.active_voxel_count = active_voxels_.size();
        local.newest_age = now - newest_stamp_;

        const VoxelKey key = positionToKey(position);
        local.occupied = active_voxels_.find(key) != active_voxels_.end();
        fillResult(result, local);
        return local.occupied;
    }

    bool lineOccupied(const rog_map::Vec3f &start,
                      const rog_map::Vec3f &end,
                      const double now,
                      QueryResult *result = nullptr) const {
        std::lock_guard<std::mutex> lock(mutex_);
        QueryResult local;
        local.hit_pos = end;
        if (!freshLocked(now) || !start.allFinite() || !end.allFinite()) {
            fillResult(result, local);
            return false;
        }
        pruneExpiredLocked(now);
        local.active_voxel_count = active_voxels_.size();
        local.newest_age = now - newest_stamp_;

        const rog_map::Vec3f delta = end - start;
        const double length = delta.norm();
        const int sample_num =
                std::max(1, static_cast<int>(std::ceil(length / cfg_.line_check_step)));
        for (int i = 0; i <= sample_num; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(sample_num);
            const rog_map::Vec3f pos = start + ratio * delta;
            const VoxelKey key = positionToKey(pos);
            if (active_voxels_.find(key) != active_voxels_.end()) {
                local.occupied = true;
                local.hit_pos = pos;
                fillResult(result, local);
                return true;
            }
        }

        fillResult(result, local);
        return false;
    }

private:
    struct VoxelKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const VoxelKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey &key) const {
            const std::uint64_t x = static_cast<std::uint32_t>(key.x);
            const std::uint64_t y = static_cast<std::uint32_t>(key.y);
            const std::uint64_t z = static_cast<std::uint32_t>(key.z);
            std::uint64_t h = x * 73856093ull;
            h ^= y * 19349663ull;
            h ^= z * 83492791ull;
            return static_cast<std::size_t>(h);
        }
    };

    struct Frame {
        double stamp{0.0};
        std::unordered_set<VoxelKey, VoxelKeyHash> voxels;
    };

    VoxelKey positionToKey(const rog_map::Vec3f &position) const {
        return {
                static_cast<int>(std::floor(position.x() / cfg_.voxel_size)),
                static_cast<int>(std::floor(position.y() / cfg_.voxel_size)),
                static_cast<int>(std::floor(position.z() / cfg_.voxel_size))
        };
    }

    bool insideLocalBox(const rog_map::Vec3f &point,
                        const rog_map::Vec3f &robot_pos) const {
        const rog_map::Vec3f diff = (point - robot_pos).cwiseAbs();
        return (diff.array() <= cfg_.local_half_size.array()).all();
    }

    bool freshLocked(const double now) const {
        if (!cfg_.enable || !std::isfinite(now) || newest_stamp_ < 0.0) {
            return false;
        }
        return now - newest_stamp_ <= cfg_.ttl;
    }

    void pruneExpiredLocked(const double now) const {
        if (!std::isfinite(now) || cfg_.ttl <= 0.0) {
            clearLocked();
            return;
        }
        while (!frames_.empty() && now - frames_.front().stamp > cfg_.ttl) {
            for (const auto &key : frames_.front().voxels) {
                auto it = active_voxels_.find(key);
                if (it == active_voxels_.end()) {
                    continue;
                }
                --it->second;
                if (it->second <= 0) {
                    active_voxels_.erase(it);
                }
            }
            frames_.pop_front();
        }
    }

    void clearLocked() const {
        frames_.clear();
        active_voxels_.clear();
        newest_stamp_ = -1.0;
    }

    static void fillResult(QueryResult *result, const QueryResult &local) {
        if (result != nullptr) {
            *result = local;
        }
    }

    mutable std::mutex mutex_;
    Config cfg_;
    mutable std::deque<Frame> frames_;
    mutable std::unordered_map<VoxelKey, int, VoxelKeyHash> active_voxels_;
    mutable double newest_stamp_{-1.0};
};

} // namespace general_planner
