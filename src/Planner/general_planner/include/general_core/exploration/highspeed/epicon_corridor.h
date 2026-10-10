#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace fast_planner {
// Pure point-cloud geometry. No occupancy map or optimizer ownership here.
struct EpiconCorridor {
  std::vector<Eigen::Vector3d> path;
  std::vector<Eigen::MatrixX4d> planes;
  bool truncated{false};
  std::string failure;
};

bool buildEpiconCorridor(const std::vector<Eigen::Vector3d> &path,
                        const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &lower,
                        const Eigen::Vector3d &upper,
                        double range, double clearance,
                        EpiconCorridor &result, bool simplify=true);
bool epiconRecoveryPoint(const Eigen::Vector3d &start,
                         const std::vector<Eigen::Vector3d> &points,
                         Eigen::Vector3d &goal);
bool clipEpiconCorridorToBoxes(const Eigen::Vector3d &head, const Eigen::Vector3d &tail,
    const std::vector<Eigen::Vector3d> &lower, const std::vector<Eigen::Vector3d> &upper,
    std::vector<Eigen::MatrixX4d> &polys, double min_overlap=1e-2);
}
