#pragma once

#include <memory>
#include <string>

#include "general_core/config.hpp"
#include "general_core/corridor_generator.h"
#include "general_core/map_manager.hpp"
#include "path_search/astar.h"
#include "traj_opt/se3_aggressive_traj_opt.hpp"

namespace general_planner {

class SE3AggressiveFrontend {
public:
  using Ptr = std::shared_ptr<SE3AggressiveFrontend>;

  SE3AggressiveFrontend(const Config &cfg,
                        const MapManager::Ptr &map_manager,
                        const path_search::Astar::Ptr &astar,
                        const CorridorGenerator::Ptr &corridor_generator);

  bool buildProblem(const super_utils::StatePVAJ &head,
                    const super_utils::StatePVAJ &tail,
                    traj_opt::SE3AggressiveProblem &problem);

private:
  bool buildGuidePath(const super_utils::Vec3f &start,
                      const super_utils::Vec3f &goal,
                      super_utils::vec_E<super_utils::Vec3f> &guide_path,
                      std::string *reason) const;

  bool buildCorridor(const super_utils::vec_E<super_utils::Vec3f> &guide_path,
                     std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> &hpolys,
                     std::string *reason) const;

  static Eigen::Matrix<double, 6, Eigen::Dynamic> polytopeToHpoly(
      const geometry_utils::Polytope &polytope);

  void fillGuideTiming(traj_opt::SE3AggressiveProblem &problem) const;
  void fillPieceCorridorMap(traj_opt::SE3AggressiveProblem &problem) const;
  void fillConfig(traj_opt::SE3AggressiveProblem &problem) const;

  Config cfg_;
  MapManager::Ptr map_manager_;
  path_search::Astar::Ptr astar_;
  CorridorGenerator::Ptr corridor_generator_;
};

} // namespace general_planner
