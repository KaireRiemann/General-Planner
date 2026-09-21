#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tracking_detector {

struct Voxel {
    int x{0};
    int y{0};
    int z{0};
    bool operator==(const Voxel &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash {
    std::size_t operator()(const Voxel &v) const {
        std::size_t h = std::hash<int>{}(v.x);
        h ^= std::hash<int>{}(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(v.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

inline Voxel makeVoxel(double x, double y, double z, double resolution) {
    const double inv = 1.0 / resolution;
    return {static_cast<int>(std::floor(x * inv)),
            static_cast<int>(std::floor(y * inv)),
            static_cast<int>(std::floor(z * inv))};
}

inline std::unordered_set<Voxel, VoxelHash>
voxelsFromPoints(const std::vector<Eigen::Vector3d> &points, double resolution) {
    std::unordered_set<Voxel, VoxelHash> voxels;
    voxels.reserve(points.size());
    for (const auto &p : points) {
        if (p.allFinite()) {
            voxels.insert(makeVoxel(p.x(), p.y(), p.z(), resolution));
        }
    }
    return voxels;
}

// Keep scan points whose 0.1 m voxel is not in the selected target cluster.
inline std::vector<char> keepMask(const std::vector<Eigen::Vector3d> &cloud,
                                  const std::unordered_set<Voxel, VoxelHash> &target_voxels,
                                  double resolution) {
    std::vector<char> keep(cloud.size(), 1);
    if (target_voxels.empty() || !(resolution > 0.0)) {
        return keep;
    }
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto &p = cloud[i];
        if (p.allFinite() &&
            target_voxels.count(makeVoxel(p.x(), p.y(), p.z(), resolution))) {
            keep[i] = 0;
        }
    }
    return keep;
}

struct ImageBox {
    double xmin{0.0};
    double ymin{0.0};
    double xmax{0.0};
    double ymax{0.0};
};

struct ProjectionParameters {
    double fx{415.6921977};
    double fy{415.6921977};
    double cx{320.0};
    double cy{240.0};
    int width{640};
    int height{480};
    double box_inset{0.05};
    double min_depth{0.3};
    double max_depth{30.0};
    double cluster_tolerance{0.3};
    int min_points{8};
    double min_extent{0.40};
    double max_extent{6.0};
    double min_box_coverage{0.02};
    double min_world_z{-std::numeric_limits<double>::infinity()};
    double max_world_z{std::numeric_limits<double>::infinity()};
};

struct ProjectedSeed {
    bool valid{false};
    std::vector<int> indices;
    double depth{0.0};
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
};

struct CroppedHit {
    Eigen::Vector3d world{Eigen::Vector3d::Zero()};
    Eigen::Vector2d uv{Eigen::Vector2d::Zero()};
    double depth{0.0};
    int original_index{-1};
};

inline bool validProjectionParameters(const ProjectionParameters &p) {
    return std::isfinite(p.fx) && std::isfinite(p.fy) && p.fx > 0.0 && p.fy > 0.0 &&
           std::isfinite(p.cx) && std::isfinite(p.cy) && p.width > 1 && p.height > 1 &&
           p.min_points >= 1 && p.cluster_tolerance > 0.0 &&
           p.box_inset >= 0.0 && p.box_inset < 0.5 &&
           p.max_depth > p.min_depth && p.max_extent > p.min_extent;
}

inline ImageBox insetBox(const ImageBox &box, const ProjectionParameters &p) {
    ImageBox out;
    const double dx = (box.xmax - box.xmin) * p.box_inset;
    const double dy = (box.ymax - box.ymin) * p.box_inset;
    out.xmin = std::max(0.0, box.xmin + dx);
    out.xmax = std::min(static_cast<double>(p.width - 1), box.xmax - dx);
    out.ymin = std::max(0.0, box.ymin + dy);
    out.ymax = std::min(static_cast<double>(p.height - 1), box.ymax - dy);
    return out;
}

inline std::vector<CroppedHit> cropBoxHits(const std::vector<Eigen::Vector3d> &cloud,
                                           const ImageBox &box,
                                           const Eigen::Isometry3d &world_from_camera,
                                           const ProjectionParameters &p) {
    std::vector<CroppedHit> hits;
    if (cloud.empty() || !world_from_camera.matrix().allFinite() ||
        !validProjectionParameters(p) || box.xmax <= box.xmin || box.ymax <= box.ymin) {
        return hits;
    }
    const ImageBox inset = insetBox(box, p);
    if (inset.xmax <= inset.xmin || inset.ymax <= inset.ymin) {
        return hits;
    }
    const Eigen::Isometry3d camera_from_world = world_from_camera.inverse();
    hits.reserve(cloud.size() / 8);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const Eigen::Vector3d &point = cloud[i];
        if (!point.allFinite()) {
            continue;
        }
        if (point.z() < p.min_world_z || point.z() > p.max_world_z) {
            continue;
        }
        const Eigen::Vector3d camera_point = camera_from_world * point;
        if (!camera_point.allFinite() || camera_point.z() < p.min_depth ||
            camera_point.z() > p.max_depth) {
            continue;
        }
        const double u = p.fx * camera_point.x() / camera_point.z() + p.cx;
        const double v = p.fy * camera_point.y() / camera_point.z() + p.cy;
        if (u < inset.xmin || u > inset.xmax || v < inset.ymin || v > inset.ymax) {
            continue;
        }
        CroppedHit hit;
        hit.world = point;
        hit.uv = Eigen::Vector2d(u, v);
        hit.depth = camera_point.z();
        hit.original_index = static_cast<int>(i);
        hits.push_back(hit);
    }
    return hits;
}

inline std::vector<std::vector<int>> clusterCroppedHits(const std::vector<CroppedHit> &hits,
                                                        double resolution, int min_points) {
    std::vector<std::vector<int>> clusters;
    if (hits.empty() || !(resolution > 0.0) || min_points < 1) {
        return clusters;
    }
    std::unordered_map<Voxel, int, VoxelHash> voxel_id;
    voxel_id.reserve(hits.size());
    std::vector<Voxel> voxels;
    voxels.reserve(hits.size());
    std::vector<int> point_voxel(hits.size(), -1);
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const Voxel v = makeVoxel(hits[i].world.x(), hits[i].world.y(), hits[i].world.z(),
                                  resolution);
        const auto inserted = voxel_id.emplace(v, static_cast<int>(voxels.size()));
        if (inserted.second) {
            voxels.push_back(v);
        }
        point_voxel[i] = inserted.first->second;
    }
    const int n = static_cast<int>(voxels.size());
    std::vector<int> parent(static_cast<std::size_t>(n));
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&parent](int x) {
        while (parent[static_cast<std::size_t>(x)] != x) {
            parent[static_cast<std::size_t>(x)] =
                parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(x)])];
            x = parent[static_cast<std::size_t>(x)];
        }
        return x;
    };
    const auto unite = [&parent, &find](int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent[static_cast<std::size_t>(a)] = b;
        }
    };
    for (int i = 0; i < n; ++i) {
        const Voxel &v = voxels[static_cast<std::size_t>(i)];
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const auto it = voxel_id.find(Voxel{v.x + dx, v.y + dy, v.z + dz});
                    if (it != voxel_id.end()) {
                        unite(i, it->second);
                    }
                }
            }
        }
    }
    std::unordered_map<int, std::vector<int>> grouped;
    grouped.reserve(static_cast<std::size_t>(n));
    for (std::size_t i = 0; i < hits.size(); ++i) {
        grouped[find(point_voxel[i])].push_back(static_cast<int>(i));
    }
    clusters.reserve(grouped.size());
    for (auto &entry : grouped) {
        if (static_cast<int>(entry.second.size()) >= min_points) {
            clusters.push_back(std::move(entry.second));
        }
    }
    return clusters;
}

// Project one registered scan into the YOLO box and keep the nearest supported
// cluster. Averaging every hit in the rectangle would include the wall behind.
inline ProjectedSeed selectProjectedSeed(const std::vector<Eigen::Vector3d> &cloud,
                                         const ImageBox &box,
                                         const Eigen::Isometry3d &world_from_camera,
                                         const ProjectionParameters &p) {
    ProjectedSeed best;
    const std::vector<CroppedHit> hits = cropBoxHits(cloud, box, world_from_camera, p);
    if (static_cast<int>(hits.size()) < p.min_points) {
        return best;
    }
    const ImageBox inset = insetBox(box, p);
    const double box_area = std::max(1.0, (inset.xmax - inset.xmin) * (inset.ymax - inset.ymin));
    const std::vector<std::vector<int>> clusters =
        clusterCroppedHits(hits, p.cluster_tolerance, p.min_points);
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto &cluster : clusters) {
        Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector3d high = -low;
        Eigen::Vector2d uv_low = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector2d uv_high = -uv_low;
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        std::vector<double> depths;
        depths.reserve(cluster.size());
        for (int index : cluster) {
            const CroppedHit &hit = hits[static_cast<std::size_t>(index)];
            low = low.cwiseMin(hit.world);
            high = high.cwiseMax(hit.world);
            uv_low = uv_low.cwiseMin(hit.uv);
            uv_high = uv_high.cwiseMax(hit.uv);
            sum += hit.world;
            depths.push_back(hit.depth);
        }
        const double extent = (high - low).maxCoeff();
        const double coverage = (uv_high - uv_low).prod() / box_area;
        if (extent < p.min_extent || extent > p.max_extent || coverage < p.min_box_coverage) {
            continue;
        }
        const Eigen::Vector2d centre = 0.5 * (uv_low + uv_high);
        const double centre_offset = std::hypot(
            (centre.x() - 0.5 * (inset.xmin + inset.xmax)) / std::max(1.0, inset.xmax - inset.xmin),
            (centre.y() - 0.5 * (inset.ymin + inset.ymax)) / std::max(1.0, inset.ymax - inset.ymin));
        std::nth_element(depths.begin(), depths.begin() + static_cast<std::ptrdiff_t>(depths.size() / 2),
                         depths.end());
        const double depth = depths[depths.size() / 2];
        const double score = depth + 0.5 * centre_offset;
        if (score >= best_score) {
            continue;
        }
        best_score = score;
        best.valid = true;
        best.depth = depth;
        best.centroid = sum / static_cast<double>(cluster.size());
        best.indices.clear();
        best.indices.reserve(cluster.size());
        for (int index : cluster) {
            best.indices.push_back(hits[static_cast<std::size_t>(index)].original_index);
        }
    }
    return best;
}

inline Eigen::Isometry3d worldFromCamera(const Eigen::Vector3d &body_p,
                                         const Eigen::Matrix3d &body_R,
                                         const Eigen::Vector3d &cam2body_p,
                                         const Eigen::Matrix3d &cam2body_R) {
    Eigen::Isometry3d world_from_body = Eigen::Isometry3d::Identity();
    world_from_body.linear() = body_R;
    world_from_body.translation() = body_p;
    Eigen::Isometry3d body_from_camera = Eigen::Isometry3d::Identity();
    body_from_camera.linear() = cam2body_R;
    body_from_camera.translation() = cam2body_p;
    return world_from_body * body_from_camera;
}

}  // namespace tracking_detector
