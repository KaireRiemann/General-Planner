#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "data_structure/base/trajectory.h"
#include <map_manager/map_manager.hpp>
#include "ros_interface/ros_interface.hpp"
#include "traj_opt/config.hpp"
#include "traj_opt/minco/minco_trajectory.hpp"
#include "utils/header/type_utils.hpp"

namespace traj_opt {

struct SE3AggressiveProblem {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  general_utils::StatePVAJ head_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::StatePVAJ tail_pvaj{general_utils::StatePVAJ::Zero()};

  general_utils::vec_E<general_utils::Vec3f> guide_path;
  std::vector<double> guide_t;

  std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> hpolys;
  std::vector<int> piece_to_corridor;

  int piece_num{4};
  double min_duration{0.5};
  double max_duration{8.0};
  double reference_speed{3.0};

  double horiz_half_len{0.35};
  double vert_half_len{0.12};
  double safe_margin{0.05};

  double max_vel{8.0};
  double thrust_acc_min{4.0};
  double thrust_acc_max{20.0};
  double body_rate_max{5.0};
  double yaw_rate_max{3.0};

  double weight_time{10.0};
  double weight_corridor{1.0e4};
  double weight_vel{1.0e3};
  double weight_thrust{1.0e3};
  double weight_body_rate{1.0e3};

  bool use_yaw{false};
  bool yaw_heading_to_velocity{true};
  double yaw{0.0};
  double yaw_rate{0.0};
  bool use_corridor{true};
  bool runtime_check_enable{true};
  bool use_numeric_shape_gradient{true};
};

class SE3AggressiveTrajOpt {
public:
  using Ptr = std::shared_ptr<SE3AggressiveTrajOpt>;
  using JerkTraj = minco::MINCO_S3<3>;

  SE3AggressiveTrajOpt(const traj_opt::Config &cfg,
                       const ros_interface::RosInterface::Ptr &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);

  bool optimize(const SE3AggressiveProblem &problem,
                geometry_utils::Trajectory &out_traj);

private:
  struct OptimizationVariables {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SE3AggressiveProblem problem;
    int piece_num{0};
    int integral_res{10};
    int iter_num{0};
    general_utils::VecDf times;
    general_utils::Mat3Df inner_points;
    general_utils::VecDf penalty_log;
    double max_vel{0.0};
    double max_thrust{0.0};
    double max_body_rate{0.0};
    double max_corridor_violation{0.0};
  };

  static double costFunctional(void *ptr,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &g);

  double evaluateCurrentCost(const Eigen::VectorXd &x,
                             Eigen::VectorXd &g);

  bool initialize(const SE3AggressiveProblem &problem);
  void decodeOptimizationVector(const Eigen::VectorXd &x,
                                Eigen::VectorXd &times,
                                Eigen::Matrix<double, 3, Eigen::Dynamic> &inner) const;
  double optimizeInternal(geometry_utils::Trajectory &traj, double rel_cost_tol);

  static JerkTraj::BoundaryState toJerkBoundary(const general_utils::StatePVAJ &state);
  static geometry_utils::Trajectory toGeometryTrajectory(const JerkTraj &traj);

  traj_opt::Config cfg_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  JerkTraj minco_traj_;
  OptimizationVariables opt_vars_;
};

} // namespace traj_opt
