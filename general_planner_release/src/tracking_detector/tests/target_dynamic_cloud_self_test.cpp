#include <tracking_detector/target_dynamic_cloud.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

namespace {

void require(bool ok, const char *message) {
    if (!ok) {
        throw std::runtime_error(message);
    }
}

tracking_detector::ProjectionParameters testParams() {
    tracking_detector::ProjectionParameters p;
    p.fx = 400.0;
    p.fy = 400.0;
    p.cx = 320.0;
    p.cy = 240.0;
    p.width = 640;
    p.height = 480;
    p.box_inset = 0.0;
    p.min_depth = 0.3;
    p.max_depth = 30.0;
    p.cluster_tolerance = 0.3;
    p.min_points = 5;
    p.min_extent = 0.2;
    p.max_extent = 6.0;
    p.min_box_coverage = 0.0;
    p.min_world_z = -10.0;
    p.max_world_z = 10.0;
    return p;
}

}  // namespace

int main() {
    try {
        std::vector<Eigen::Vector3d> cloud;
        cloud.emplace_back(1.0, 2.0, 0.5);
        cloud.emplace_back(1.04, 2.01, 0.52);
        cloud.emplace_back(8.0, 0.0, 1.0);
        const auto voxels = tracking_detector::voxelsFromPoints(
            {cloud[0], cloud[1]}, 0.1);
        require(voxels.size() >= 1, "target voxels missing");
        const auto keep = tracking_detector::keepMask(cloud, voxels, 0.1);
        require(keep.size() == 3 && keep[0] == 0 && keep[1] == 0 && keep[2] != 0,
                "same-stamp voxel subtract must drop only the target body");

        const Eigen::Matrix3d cam2body_R =
            (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
        const Eigen::Isometry3d world_from_camera = tracking_detector::worldFromCamera(
            Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
            cam2body_R);
        std::vector<Eigen::Vector3d> scan;
        for (int i = 0; i < 9; ++i) {
            const double y = (i % 3 - 1) * 0.15;
            const double z = (i / 3 - 1) * 0.15;
            scan.emplace_back(5.0, y, z);
        }
        for (int i = 0; i < 16; ++i) {
            const double y = (i % 4 - 1.5) * 0.25;
            const double z = (i / 4 - 1.5) * 0.25;
            scan.emplace_back(12.0, y, z);
        }
        tracking_detector::ImageBox box;
        box.xmin = 200;
        box.ymin = 120;
        box.xmax = 440;
        box.ymax = 360;
        const tracking_detector::ProjectedSeed seed =
            tracking_detector::selectProjectedSeed(scan, box, world_from_camera, testParams());
        require(seed.valid, "YOLO box must select a LiDAR cluster");
        require(seed.indices.size() >= 5, "near cluster too small");
        require(std::abs(seed.centroid.x() - 5.0) < 0.5,
                "must keep the near object, not the wall behind the box");
        require(seed.depth < 8.0, "selected cluster is not the foreground");

        tracking_detector::ImageBox empty_box;
        empty_box.xmin = 0;
        empty_box.ymin = 0;
        empty_box.xmax = 10;
        empty_box.ymax = 10;
        const tracking_detector::ProjectedSeed none =
            tracking_detector::selectProjectedSeed(scan, empty_box, world_from_camera, testParams());
        require(!none.valid, "empty box must not invent a cluster");

        std::cout << "target_dynamic_cloud_self_test: PASS\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "target_dynamic_cloud_self_test FAIL: " << e.what() << '\n';
        return 1;
    }
}
