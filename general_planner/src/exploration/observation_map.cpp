#include "exploration/observation_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace general_planner {
namespace exploration {

namespace {
double clamp01(const double v) {
    return std::max(0.0, std::min(1.0, v));
}
}  // namespace

ObservationMap::ObservationMap(Config cfg) : cfg_(std::move(cfg)) {
    cfg_.resolution = std::max(1.0e-3, cfg_.resolution);
    cfg_.cloud_downsample_step = std::max(1, cfg_.cloud_downsample_step);
    cfg_.max_points_per_update = std::max(0, cfg_.max_points_per_update);
    cfg_.frontier_cluster_radius = std::max(cfg_.resolution, cfg_.frontier_cluster_radius);
}

void ObservationMap::update(const rog_map::PointCloud &cloud,
                            const super_utils::Pose &pose,
                            const CloudFrame frame,
                            const super_utils::Vec3f &sensor_position,
                            const double travel_distance,
                            const double stamp) {
    if (!cfg_.enable || cloud.empty()) {
        return;
    }

    int step = cfg_.cloud_downsample_step;
    if (cfg_.max_points_per_update > 0 &&
        static_cast<int>(cloud.size()) > cfg_.max_points_per_update) {
        step = std::max(step,
                        static_cast<int>(std::ceil(static_cast<double>(cloud.size()) /
                                                   static_cast<double>(cfg_.max_points_per_update))));
    }

    std::vector<VoxelKey, Eigen::aligned_allocator<VoxelKey>> touched;
    touched.reserve((cloud.size() + static_cast<std::size_t>(step) - 1U) /
                    static_cast<std::size_t>(step));

    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < cloud.size(); i += static_cast<std::size_t>(step)) {
        const super_utils::Vec3f world = transformPointToWorld(cloud.points[i], pose, frame);
        if (!world.allFinite() || !insideBounds(world)) {
            continue;
        }
        const double range = (world - sensor_position).norm();
        if (range < cfg_.min_observation_distance || range > cfg_.max_observation_distance) {
            continue;
        }

        const VoxelKey key = posToKey(world);
        touched.push_back(key);

        const super_utils::Vec3f center = keyToPos(key);
        super_utils::Vec3f normal = sensor_position - center;
        if (normal.norm() < 1.0e-6) {
            normal = super_utils::Vec3f::UnitX();
        } else {
            normal.normalize();
        }

        const double quality = clamp01((cfg_.max_observation_distance - range) /
                                       std::max(1.0e-3,
                                                cfg_.max_observation_distance -
                                                cfg_.well_observed_distance));
        const bool well_observed = range <= cfg_.well_observed_distance;
        auto it = voxels_.find(key);
        if (it == voxels_.end()) {
            SurfaceVoxel voxel;
            voxel.center = center;
            voxel.normal = normal;
            voxel.quality = quality;
            voxel.state = well_observed ? SurfaceVoxelState::WELL_OBSERVED
                                        : SurfaceVoxelState::POORLY_OBSERVED;
            voxel.first_seen_time = stamp;
            voxel.last_seen_time = stamp;
            voxel.generated_travel_distance = travel_distance;
            voxel.generated_position = sensor_position;
            voxels_.emplace(key, voxel);
        } else {
            SurfaceVoxel &voxel = it->second;
            voxel.center = center;
            voxel.normal = (1.0 - cfg_.normal_ema_alpha) * voxel.normal +
                           cfg_.normal_ema_alpha * normal;
            if (voxel.normal.norm() > 1.0e-6) {
                voxel.normal.normalize();
            }
            voxel.quality = std::max(voxel.quality, quality);
            voxel.last_seen_time = stamp;
            if (well_observed) {
                voxel.state = SurfaceVoxelState::WELL_OBSERVED;
            } else if (voxel.state != SurfaceVoxelState::WELL_OBSERVED) {
                voxel.state = SurfaceVoxelState::POORLY_OBSERVED;
            }
        }
    }

    for (const auto &key : touched) {
        updateFrontierAround(key);
    }
}

void ObservationMap::getFrontierClusters(std::vector<SurfaceFrontierCluster> &clusters) const {
    clusters.clear();

    struct FrontierPoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VoxelKey key;
        super_utils::Vec3f pos;
        super_utils::Vec3f normal;
        double stamp{0.0};
        double travel_distance{0.0};
        super_utils::Vec3f generated_position{super_utils::Vec3f::Zero()};
    };

    std::vector<FrontierPoint, Eigen::aligned_allocator<FrontierPoint>> points;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        points.reserve(frontier_keys_.size());
        for (const auto &key : frontier_keys_) {
            const auto it = voxels_.find(key);
            if (it == voxels_.end() || it->second.state != SurfaceVoxelState::FRONTIER) {
                continue;
            }
            FrontierPoint point;
            point.key = key;
            point.pos = it->second.center;
            point.normal = it->second.normal;
            point.stamp = it->second.last_seen_time;
            point.travel_distance = it->second.generated_travel_distance;
            point.generated_position = it->second.generated_position;
            points.push_back(point);
        }
    }

    if (points.empty()) {
        return;
    }

    const double radius = cfg_.frontier_cluster_radius;
    const double radius_sq = radius * radius;
    const int bucket_scale = 1;
    auto bucketKey = [radius, bucket_scale](const super_utils::Vec3f &p) {
        return Eigen::Vector3i(static_cast<int>(std::floor(p.x() / (radius * bucket_scale))),
                               static_cast<int>(std::floor(p.y() / (radius * bucket_scale))),
                               static_cast<int>(std::floor(p.z() / (radius * bucket_scale))));
    };

    std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash, VoxelKeyEqual> buckets;
    buckets.reserve(points.size());
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        buckets[bucketKey(points[static_cast<std::size_t>(i)].pos)].push_back(i);
    }

    std::vector<char> visited(points.size(), 0);
    int transient_id = 0;
    std::queue<int> queue;
    for (int seed = 0; seed < static_cast<int>(points.size()); ++seed) {
        if (visited[static_cast<std::size_t>(seed)] != 0) {
            continue;
        }
        SurfaceFrontierCluster cluster;
        cluster.transient_id = transient_id++;
        visited[static_cast<std::size_t>(seed)] = 1;
        queue.push(seed);
        while (!queue.empty()) {
            const int current = queue.front();
            queue.pop();
            const auto &current_point = points[static_cast<std::size_t>(current)];
            cluster.cells.push_back(current_point.pos);
            cluster.normals.push_back(current_point.normal);
            cluster.stamp = std::max(cluster.stamp, current_point.stamp);
            if (cluster.cells.size() == 1U) {
                cluster.generated_position = current_point.generated_position;
                cluster.generated_travel_distance = current_point.travel_distance;
            }

            const VoxelKey b = bucketKey(current_point.pos);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const VoxelKey nb = b + VoxelKey(dx, dy, dz);
                        const auto bit = buckets.find(nb);
                        if (bit == buckets.end()) {
                            continue;
                        }
                        for (const int candidate : bit->second) {
                            if (visited[static_cast<std::size_t>(candidate)] != 0) {
                                continue;
                            }
                            const auto &candidate_point = points[static_cast<std::size_t>(candidate)];
                            if ((candidate_point.pos - current_point.pos).squaredNorm() > radius_sq) {
                                continue;
                            }
                            if (candidate_point.normal.dot(current_point.normal) <
                                cfg_.frontier_normal_similarity) {
                                continue;
                            }
                            visited[static_cast<std::size_t>(candidate)] = 1;
                            queue.push(candidate);
                        }
                    }
                }
            }
        }

        cluster.raw_size = static_cast<int>(cluster.cells.size());
        if (cluster.raw_size < cfg_.min_frontier_cluster_size) {
            continue;
        }
        cluster.center.setZero();
        cluster.normal.setZero();
        cluster.bbox_min = cluster.cells.front();
        cluster.bbox_max = cluster.cells.front();
        for (std::size_t i = 0; i < cluster.cells.size(); ++i) {
            cluster.center += cluster.cells[i];
            cluster.normal += cluster.normals[i];
            cluster.bbox_min = cluster.bbox_min.cwiseMin(cluster.cells[i]);
            cluster.bbox_max = cluster.bbox_max.cwiseMax(cluster.cells[i]);
        }
        cluster.center /= static_cast<double>(cluster.cells.size());
        if (cluster.normal.norm() > 1.0e-6) {
            cluster.normal.normalize();
        } else {
            cluster.normal = super_utils::Vec3f::UnitX();
        }
        clusters.push_back(cluster);
    }
}

bool ObservationMap::getVoxel(const super_utils::Vec3f &position, SurfaceVoxel &voxel) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = voxels_.find(posToKey(position));
    if (it == voxels_.end()) {
        return false;
    }
    voxel = it->second;
    return true;
}

double ObservationMap::nearestSurfaceDistance(const super_utils::Vec3f &position,
                                              const double max_search_radius) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (voxels_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const int r = std::max(1, static_cast<int>(std::ceil(max_search_radius / cfg_.resolution)));
    const VoxelKey center = posToKey(position);
    double best_sq = max_search_radius * max_search_radius;
    bool found = false;
    for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dz = -r; dz <= r; ++dz) {
                const VoxelKey key = center + VoxelKey(dx, dy, dz);
                const auto it = voxels_.find(key);
                if (it == voxels_.end()) {
                    continue;
                }
                const double sq = (it->second.center - position).squaredNorm();
                if (sq < best_sq) {
                    best_sq = sq;
                    found = true;
                }
            }
        }
    }
    return found ? std::sqrt(best_sq) : std::numeric_limits<double>::infinity();
}

bool ObservationMap::lineOfSightFree(const super_utils::Vec3f &start,
                                     const super_utils::Vec3f &end,
                                     const double safe_distance,
                                     const double step) const {
    const super_utils::Vec3f delta = end - start;
    const double length = delta.norm();
    if (length < 1.0e-6) {
        return true;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(0.05, step))));
    for (int i = 1; i < samples; ++i) {
        const super_utils::Vec3f p = start + delta * (static_cast<double>(i) / static_cast<double>(samples));
        if (nearestSurfaceDistance(p, safe_distance) < safe_distance) {
            return false;
        }
    }
    return true;
}

int ObservationMap::observedVoxelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(voxels_.size());
}

int ObservationMap::frontierVoxelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(frontier_keys_.size());
}

void ObservationMap::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    voxels_.clear();
    frontier_keys_.clear();
}

ObservationMap::VoxelKey ObservationMap::posToKey(const super_utils::Vec3f &position) const {
    return VoxelKey(static_cast<int>(std::floor(position.x() / cfg_.resolution)),
                    static_cast<int>(std::floor(position.y() / cfg_.resolution)),
                    static_cast<int>(std::floor(position.z() / cfg_.resolution)));
}

super_utils::Vec3f ObservationMap::keyToPos(const VoxelKey &key) const {
    return (key.cast<double>() + super_utils::Vec3f(0.5, 0.5, 0.5)) * cfg_.resolution;
}

super_utils::Vec3f ObservationMap::transformPointToWorld(const rog_map::PclPoint &point,
                                                         const super_utils::Pose &pose,
                                                         const CloudFrame frame) const {
    const super_utils::Vec3f local(point.x, point.y, point.z);
    if (frame == CloudFrame::WORLD) {
        return local;
    }
    return pose.first + pose.second * local;
}

bool ObservationMap::insideBounds(const super_utils::Vec3f &position) const {
    return position.x() >= cfg_.bbox_min_x && position.y() >= cfg_.bbox_min_y &&
           position.z() >= cfg_.bbox_min_z && position.x() <= cfg_.bbox_max_x &&
           position.y() <= cfg_.bbox_max_y && position.z() <= cfg_.bbox_max_z;
}

bool ObservationMap::computeIsFrontier(const VoxelKey &key) const {
    const auto it = voxels_.find(key);
    if (it == voxels_.end() || it->second.state == SurfaceVoxelState::WELL_OBSERVED) {
        return false;
    }
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const auto nit = voxels_.find(key + VoxelKey(dx, dy, dz));
                if (nit != voxels_.end() && nit->second.state == SurfaceVoxelState::WELL_OBSERVED) {
                    return true;
                }
            }
        }
    }
    return false;
}

void ObservationMap::updateFrontierAround(const VoxelKey &key) {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const VoxelKey current = key + VoxelKey(dx, dy, dz);
                auto it = voxels_.find(current);
                if (it == voxels_.end()) {
                    continue;
                }
                const bool frontier = computeIsFrontier(current);
                if (frontier) {
                    it->second.state = SurfaceVoxelState::FRONTIER;
                    frontier_keys_.insert(current);
                } else {
                    frontier_keys_.erase(current);
                    if (it->second.state == SurfaceVoxelState::FRONTIER) {
                        it->second.state = SurfaceVoxelState::POORLY_OBSERVED;
                    }
                }
            }
        }
    }
}

}  // namespace exploration
}  // namespace general_planner
