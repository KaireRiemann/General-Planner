#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>
#include <map_manager/map_manager.hpp>
#include <ros_interface/ros_interface.hpp>
#include <traj_opt/config.hpp>
#include <traj_opt/minco/minco_trajectory.hpp>
#include <utils/header/type_utils.hpp>

namespace traj_opt
{

class StateToStateIgoTrajOpt
{
public:
  using Ptr = std::shared_ptr<StateToStateIgoTrajOpt>;

  struct Report
  {
    bool success{false};
    bool timed_out{false};
    std::string reason{"NOT_RUN"};
    int decision_dimension{0};
    int population{0};
    int generations{0};
    int evaluations{0};
    double feasible_ratio{0.0};
    double elapsed_seconds{0.0};
    double best_duration{0.0};
    double best_quality{0.0};
    int best_occupied_samples{0};
    int best_outside_samples{0};
    int best_unknown_samples{0};
    double best_sfc_violation{0.0};
    double best_dynamic_violation{0.0};
  };

  StateToStateIgoTrajOpt(const Config &config,
                         const ros_interface::RosInterface::Ptr &ros_ptr,
                         const general_planner::MapManager::Ptr &map_manager);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);

  bool optimize(const general_utils::StatePVAJ &head,
                const general_utils::StatePVAJ &tail,
                const general_utils::vec_E<general_utils::Vec3f> &guide_path,
                const std::vector<double> &guide_stamps,
                geometry_utils::PolytopeVec &corridor,
                double wall_time_budget,
                geometry_utils::Trajectory &trajectory);

  // Independent live-map gate.  The distribution result is never publishable
  // until this method accepts it against the current map and corridor.
  bool validateForCommit(const geometry_utils::Trajectory &trajectory,
                         const geometry_utils::PolytopeVec &corridor,
                         std::string &reason) const;

  // Reuse a previously committed trajectory as one injected candidate.  The
  // guide-overlap decision remains the distribution mean.
  void setWarmStart(const geometry_utils::Trajectory &trajectory);

  void getInitValue(general_utils::VecDf &durations,
                    general_utils::vec_Vec3f &points) const;

  const Report &lastReport() const { return last_report_; }

private:
  Config config_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  Report last_report_;
  general_utils::VecDf last_durations_;
  general_utils::vec_Vec3f last_points_;
  Eigen::VectorXd previous_decision_;
  int previous_piece_count_{0};
};

}  // namespace traj_opt
