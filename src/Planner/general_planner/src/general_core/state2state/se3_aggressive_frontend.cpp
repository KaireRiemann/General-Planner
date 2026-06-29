#include "general_core/state2state/se3_aggressive_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "utils/header/color_msg_utils.hpp"

namespace general_planner {
namespace {

void appendUnique(const general_utils::Vec3f &point,
                  general_utils::vec_E<general_utils::Vec3f> &path) {
  if (!point.allFinite()) {
    return;
  }
  if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
    path.emplace_back(point);
  }
}

} // namespace

SE3AggressiveFrontend::SE3AggressiveFrontend(
    const Config &cfg,
    const MapManager::Ptr &map_manager,
    const path_search::Astar::Ptr &astar,
    const CorridorGenerator::Ptr &corridor_generator)
    : cfg_(cfg),
      map_manager_(map_manager),
      astar_(astar),
      corridor_generator_(corridor_generator) {}

void SE3AggressiveFrontend::fillConfig(traj_opt::SE3AggressiveProblem &problem) const {
  problem.piece_num = std::max(1, cfg_.se3_piece_num);
  problem.reference_speed = std::max(1.0e-3, cfg_.se3_reference_speed);
  problem.min_duration = std::max(1.0e-3, cfg_.se3_min_duration);
  problem.max_duration = std::max(problem.min_duration, cfg_.se3_max_duration);
  problem.horiz_half_len = cfg_.se3_horiz_half_len;
  problem.vert_half_len = cfg_.se3_vert_half_len;
  problem.safe_margin = cfg_.se3_safe_margin;
  problem.max_vel = cfg_.se3_max_vel;
  problem.thrust_acc_min = cfg_.se3_thrust_acc_min;
  problem.thrust_acc_max = cfg_.se3_thrust_acc_max;
  problem.body_rate_max = cfg_.se3_body_rate_max;
  problem.yaw_rate_max = cfg_.se3_yaw_rate_max;
  problem.weight_time = cfg_.se3_weight_time;
  problem.weight_corridor = cfg_.se3_weight_corridor;
  problem.weight_vel = cfg_.se3_weight_vel;
  problem.weight_thrust = cfg_.se3_weight_thrust;
  problem.weight_body_rate = cfg_.se3_weight_body_rate;
  problem.use_yaw = cfg_.se3_use_yaw;
  problem.yaw_heading_to_velocity = cfg_.se3_yaw_heading_to_velocity;
  problem.use_corridor = cfg_.se3_use_corridor;
  problem.runtime_check_enable = cfg_.se3_runtime_check_enable;
  problem.use_numeric_shape_gradient = cfg_.se3_use_numeric_shape_gradient;
}

bool SE3AggressiveFrontend::buildGuidePath(
    const general_utils::Vec3f &start,
    const general_utils::Vec3f &goal,
    general_utils::vec_E<general_utils::Vec3f> &guide_path,
    std::string *reason) const {
  guide_path.clear();
  appendUnique(start, guide_path);

  if (astar_ && map_manager_ && map_manager_->ready()) {
    using namespace path_search;
    general_utils::vec_E<general_utils::Vec3f> astar_path;
    const double horizon =
        std::max(cfg_.planning_horizon, 1.25 * std::max(1.0, (goal - start).norm()));
    const int flag = ON_INF_MAP |
                     (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                     DONT_USE_INF_NEIGHBOR;
    const general_utils::RET_CODE ret =
        astar_->pointToPointPathSearch(start,
                                       goal,
                                       flag,
                                       horizon,
                                       astar_path,
                                       cfg_.frontend_astar_time_out);
    if ((ret == general_utils::REACH_GOAL || ret == general_utils::REACH_HORIZON) &&
        !astar_path.empty()) {
      for (const auto &point : astar_path) {
        appendUnique(point, guide_path);
      }
    }
  }

  appendUnique(goal, guide_path);
  if (guide_path.size() < 2) {
    if (reason != nullptr) {
      *reason = "GUIDE_PATH_EMPTY";
    }
    return false;
  }
  return true;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> SE3AggressiveFrontend::polytopeToHpoly(
    const geometry_utils::Polytope &polytope) {
  const auto planes = polytope.GetPlanes();
  Eigen::Matrix<double, 6, Eigen::Dynamic> hpoly(6, planes.rows());
  for (int row = 0; row < planes.rows(); ++row) {
    Eigen::Vector3d n = planes.row(row).head<3>().transpose();
    const double d = planes(row, 3);
    const double n_sqr = std::max(1.0e-12, n.squaredNorm());
    const Eigen::Vector3d q = -d * n / n_sqr;
    const double n_norm = std::sqrt(n_sqr);
    hpoly.col(row).head<3>() = n / n_norm;
    hpoly.col(row).tail<3>() = q;
  }
  return hpoly;
}

bool SE3AggressiveFrontend::buildCorridor(
    const general_utils::vec_E<general_utils::Vec3f> &guide_path,
    std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> &hpolys,
    std::string *reason) const {
  hpolys.clear();
  if (!cfg_.se3_use_corridor) {
    return true;
  }
  if (!corridor_generator_) {
    if (reason != nullptr) {
      *reason = "CORRIDOR_GENERATOR_MISSING";
    }
    return false;
  }

  geometry_utils::PolytopeVec sfcs;
  general_utils::Vec3f shifted_start = guide_path.front();
  if (!corridor_generator_->SearchPolytopeOnPath(guide_path, sfcs, shifted_start, false)) {
    if (reason != nullptr) {
      *reason = "CORRIDOR_GENERATION_FAILED";
    }
    return false;
  }
  for (const auto &polytope : sfcs) {
    if (!polytope.empty() && polytope.SurfNum() > 0) {
      hpolys.emplace_back(polytopeToHpoly(polytope));
    }
  }
  if (hpolys.empty()) {
    if (reason != nullptr) {
      *reason = "CORRIDOR_EMPTY";
    }
    return false;
  }
  return true;
}

void SE3AggressiveFrontend::fillGuideTiming(traj_opt::SE3AggressiveProblem &problem) const {
  problem.guide_t.clear();
  problem.guide_t.reserve(problem.guide_path.size());
  problem.guide_t.emplace_back(0.0);
  for (std::size_t i = 1; i < problem.guide_path.size(); ++i) {
    const double dt =
        (problem.guide_path[i] - problem.guide_path[i - 1]).norm() /
        std::max(1.0e-3, problem.reference_speed);
    problem.guide_t.emplace_back(problem.guide_t.back() + std::max(0.03, dt));
  }
}

void SE3AggressiveFrontend::fillPieceCorridorMap(traj_opt::SE3AggressiveProblem &problem) const {
  problem.piece_to_corridor.assign(static_cast<std::size_t>(std::max(1, problem.piece_num)), 0);
  if (problem.hpolys.empty()) {
    return;
  }
  for (int i = 0; i < problem.piece_num; ++i) {
    const double ratio = (static_cast<double>(i) + 0.5) / static_cast<double>(problem.piece_num);
    int corridor_id = static_cast<int>(std::floor(ratio * static_cast<double>(problem.hpolys.size())));
    corridor_id = std::clamp(corridor_id, 0, static_cast<int>(problem.hpolys.size()) - 1);
    problem.piece_to_corridor[static_cast<std::size_t>(i)] = corridor_id;
  }
}

bool SE3AggressiveFrontend::buildProblem(const general_utils::StatePVAJ &head,
                                         const general_utils::StatePVAJ &tail,
                                         traj_opt::SE3AggressiveProblem &problem) {
  problem = traj_opt::SE3AggressiveProblem();
  fillConfig(problem);
  problem.head_pvaj = head;
  problem.tail_pvaj = tail;

  std::string reason;
  if (!buildGuidePath(head.col(0), tail.col(0), problem.guide_path, &reason)) {
    std::cout << " -- [SE3AggressiveFrontend] SE3_FRONTEND_FAILED reason="
              << reason << std::endl;
    return false;
  }
  fillGuideTiming(problem);

  if (!buildCorridor(problem.guide_path, problem.hpolys, &reason)) {
    std::cout << " -- [SE3AggressiveFrontend] SE3_FRONTEND_FAILED reason="
              << reason << std::endl;
    return false;
  }
  fillPieceCorridorMap(problem);

  int hpoly_plane_num = 0;
  for (const auto &hpoly : problem.hpolys) {
    hpoly_plane_num += static_cast<int>(hpoly.cols());
  }
  std::cout << " -- [SE3AggressiveFrontend] SE3_FRONTEND_SUCCESS guide_size="
            << problem.guide_path.size()
            << ", hpoly_num=" << hpoly_plane_num
            << ", pieces=" << problem.piece_num << std::endl;
  return true;
}

} // namespace general_planner
