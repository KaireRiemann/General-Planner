#include <data_structure/base/trajectory.h>
#include <fmt/color.h>
#include <general_core/corridor_generator.h>
#include <general_core/map_manager.hpp>
#include <rog_map_ros/rog_map_ros1.hpp>
#include <ros_interface/ros_interface.hpp>
#include <traj_opt/traj_manager.h>
#include <utils/header/type_utils.hpp>
#include <utils/geometry/geometry_utils.h>
#include <utils/optimization/polynomial_interpolation.h>

namespace general_planner
{
using namespace general_utils;
using namespace geometry_utils;
}

#include <data_structure/backup_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/exp_traj.h>

#include <general_core/exploration/highspeed/planner_manager.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace fast_planner
{
struct GeneralCommitStore
{
  general_planner::CmdTraj cmd_traj_info;
  general_planner::ExpTraj last_exp_traj_info;
};

namespace
{
class NullRosInterface final : public ros_interface::RosInterface
{
public:
  void debug(const std::string &msg) override { ROS_DEBUG_STREAM(msg); }
  void info(const std::string &msg) override { ROS_INFO_STREAM(msg); }
  void warn(const std::string &msg) override { ROS_WARN_STREAM(msg); }
  void error(const std::string &msg) override { ROS_ERROR_STREAM(msg); }
  void fatal(const std::string &msg) override { ROS_FATAL_STREAM(msg); }

  void setSimTime(const double &) override {}

  double getSimTime() override
  {
    return ros::Time::now().toSec();
  }

  void getSimTime(int32_t &sec, uint32_t &nsec) override
  {
    const ros::Time now = ros::Time::now();
    sec = now.sec;
    nsec = now.nsec;
  }

  void vizExpTraj(const geometry_utils::Trajectory &, const std::string & = "exp_traj") override {}
  void vizBackupTraj(const geometry_utils::Trajectory &) override {}
  void vizFrontendPath(const general_utils::vec_Vec3f &) override {}
  void vizExpSfc(const geometry_utils::PolytopeVec &) override {}
  void vizBackupSfc(const geometry_utils::Polytope &) override {}
  void vizGoalPath(const general_utils::vec_Vec3f &) override {}
  void vizCommittedTraj(const geometry_utils::Trajectory &, const double &) override {}
  void vizYawTraj(const geometry_utils::Trajectory &, const geometry_utils::Trajectory &) override {}
  void vizAstarBoundingBox(const general_utils::Vec3f &, const general_utils::Vec3f &) override {}
  void vizAstarPoints(const general_utils::Vec3f &, const Color &, const std::string &,
                      const double & = 0.1, const int & = 0) override {}
  void vizReplanLog(const geometry_utils::Trajectory &, const geometry_utils::Trajectory &,
                    const geometry_utils::Trajectory &, const geometry_utils::Trajectory &,
                    const geometry_utils::PolytopeVec &, const geometry_utils::Polytope &,
                    const general_utils::vec_Vec3f &, const int &) override {}
  void vizCiriSeedLine(const general_utils::Vec3f &, const general_utils::Vec3f &, const double &) override {}
  void vizCiriEllipsoid(const geometry_utils::Ellipsoid &) override {}
  void vizCiriInfeasiblePoint(const general_utils::Vec3f) override {}
  void vizCiriPolytope(const geometry_utils::Polytope &, const std::string &) override {}
  void vizCiriPointCloud(const general_utils::vec_Vec3f &) override {}
};

double yawDelta(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double pathLength(const std::vector<Eigen::Vector3d> &path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

double knownFreeAdaptiveVelocity(const GcopterConfig &cfg,
                                 double known_free_remaining)
{
  const double v_min = std::max(0.5, cfg.minSegmentVel);
  const double v_short =
      std::clamp(cfg.velocityShortKnownFree, v_min, cfg.maxVelMag);
  const double v_medium =
      std::clamp(cfg.velocityMediumKnownFree, v_short, cfg.maxVelMag);
  const double v_long =
      std::clamp(cfg.velocityLongKnownFree, v_medium, cfg.maxVelMag);
  if (known_free_remaining >= cfg.knownFreeLongLength)
  {
    return v_long;
  }
  if (known_free_remaining >= cfg.knownFreeMediumLength)
  {
    return v_medium;
  }
  if (known_free_remaining >= cfg.knownFreeShortLength)
  {
    return std::min(v_medium, std::max(v_short, 0.5 * (v_short + v_medium)));
  }
  return v_short;
}

bool pointInsideHPoly(const Eigen::MatrixX4d &hpoly, const Eigen::Vector3d &pt)
{
  if (hpoly.rows() == 0 || hpoly.cols() != 4)
  {
    return false;
  }
  Eigen::Vector4d hp;
  hp << pt, 1.0;
  return (hpoly * hp).maxCoeff() <= 1.0e-5;
}

traj_opt::Config makeGeneralExpConfig(const GcopterConfig &cfg)
{
  traj_opt::Config out;
  out.uniform_time_en = false;
  out.print_optimizer_log = false;
  out.save_log_en = false;
  out.mass = std::max(1.0e-3, cfg.vehicleMass);
  out.dh = cfg.horizDrag;
  out.dv = cfg.vertDrag;
  out.grav = cfg.gravAcc;
  out.cp = cfg.parasDrag;
  out.v_eps = cfg.speedEps;
  out.pos_constraint_type = traj_opt::CORRIDOR;
  out.block_energy_cost = false;
  out.max_vel = std::max(0.2, cfg.maxVelMag);
  out.max_acc = std::max(0.2, cfg.maxAccMag);
  out.max_jerk = std::max(10.0, cfg.maxBdrMag);
  out.max_omg = std::max(0.2, cfg.yaw_max_vel);
  out.min_acc_thr = std::max(0.0, cfg.minThrust / out.mass);
  out.max_acc_thr = std::max(out.min_acc_thr + 1.0e-3, cfg.maxThrust / out.mass);
  out.penna_t = std::max(0.0, cfg.weightT);
  const bool explicit_acc_weight = cfg.chiVec.size() >= 6;
  out.penna_pos = cfg.chiVec.size() > 0 ? cfg.chiVec[0] : 1.0e5;
  out.penna_vel = cfg.chiVec.size() > 1 ? cfg.chiVec[1] : 1.0e4;
  out.penna_acc = explicit_acc_weight
                      ? cfg.chiVec[2]
                      : (cfg.chiVec.size() > 1 ? cfg.chiVec[1] : 1.0e4);
  out.penna_jerk = 0.0;
  out.penna_attract = 0.0;
  out.penna_guide_path = 0.0;
  out.penna_guide_vel = 0.0;
  out.penna_guide_z_tube = 0.0;
  out.guide_z_tube_radius = 0.0;
  out.guide_path_tube_radius = 0.0;
  out.guide_path_z_tube_radius = 0.0;
  out.guide_path_huber_delta = 0.25;
  out.guide_path_time_gradient_en = false;
  out.penna_ts = 0.0;
  out.piece_num = 0;
  out.penna_omg = explicit_acc_weight
                      ? cfg.chiVec[3]
                      : (cfg.chiVec.size() > 2 ? cfg.chiVec[2] : 1.0e4);
  out.penna_theta = explicit_acc_weight
                        ? cfg.chiVec[4]
                        : (cfg.chiVec.size() > 3 ? cfg.chiVec[3] : 1.0e4);
  out.penna_thr = explicit_acc_weight
                      ? cfg.chiVec[5]
                      : (cfg.chiVec.size() > 4 ? cfg.chiVec[4] : 1.0e5);
  out.penna_margin = 0.05;
  out.smooth_eps = std::max(1.0e-4, cfg.smoothingEps);
  out.integral_reso = std::max(1, cfg.integralIntervs);
  out.opt_accuracy = std::max(1.0e-8, cfg.relCostTol);
  out.init_profile_vel_ratio = 0.75;
  out.init_duration_scale = 1.0;
  out.terminal_vel_ratio = 0.0;
  out.quadrotot_flatness.reset(out.mass, out.grav, out.dh, out.dv, out.cp, out.v_eps);
  return out;
}

traj_opt::Config makeGeneralBackupConfig(const GcopterConfig &cfg)
{
  traj_opt::Config out = makeGeneralExpConfig(cfg);
  out.uniform_time_en = false;
  out.pos_constraint_type = traj_opt::CORRIDOR;
  out.block_energy_cost = false;
  out.max_vel = std::max(0.2, cfg.backupMaxVel);
  out.max_acc = std::max(0.2, cfg.backupMaxAcc);
  out.max_jerk = std::max(10.0, cfg.maxBdrMag);
  out.penna_t = std::max(0.0, cfg.WeightSafeT);
  out.penna_ts = std::max(1.0e5, 100.0 * std::max(1.0, cfg.weightT));
  out.penna_pos = cfg.chiVec.size() > 0 ? cfg.chiVec[0] : 1.0e5;
  out.penna_vel = cfg.chiVec.size() > 1 ? cfg.chiVec[1] : 1.0e4;
  out.penna_acc = cfg.chiVec.size() > 2 ? cfg.chiVec[2] : 1.0e4;
  out.penna_jerk = 0.0;
  out.penna_attract = 0.0;
  out.penna_guide_path = 0.0;
  out.penna_guide_vel = 0.0;
  out.penna_guide_z_tube = 0.0;
  out.guide_z_tube_radius = 0.0;
  out.piece_num = std::max(1, cfg.backupPieceNum);
  out.integral_reso = std::max(1, cfg.integralIntervs);
  out.opt_accuracy = std::max(1.0e-8, cfg.relCostTol);
  out.quadrotot_flatness.reset(out.mass, out.grav, out.dh, out.dv, out.cp, out.v_eps);
  return out;
}

std::vector<Eigen::Vector3d> shortenPath(const std::vector<Eigen::Vector3f> &path,
                                         double max_length)
{
  std::vector<Eigen::Vector3d> out;
  if (path.empty())
  {
    return out;
  }
  out.reserve(path.size());
  out.emplace_back(path.front().cast<double>());
  const double limit = max_length > 0.1 ? max_length : std::numeric_limits<double>::infinity();
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    const double seg_len = static_cast<double>((path[i] - path[i - 1]).norm());
    if (length + seg_len > limit && seg_len > 1.0e-6)
    {
      const double ratio = std::clamp((limit - length) / seg_len, 0.0, 1.0);
      out.emplace_back((path[i - 1] + ratio * (path[i] - path[i - 1])).cast<double>());
      break;
    }
    out.emplace_back(path[i].cast<double>());
    length += seg_len;
    if (length >= limit)
    {
      break;
    }
  }
  if (out.size() == 1 && path.size() > 1)
  {
    out.emplace_back(path[1].cast<double>());
  }
  return out;
}

struct PieceVelocityProfile
{
  general_utils::VecDf bounds;
  std::vector<int> segment_piece;
  std::vector<double> piece_length;
};

PieceVelocityProfile computePieceVelocityProfile(
    const std::vector<Eigen::Vector3d> &path,
    const geometry_utils::PolytopeVec &sfcs,
    const Eigen::Vector3d &head_velocity,
    double head_yaw,
    const GcopterConfig &cfg,
    const SegmentVelocityLimit &global_limit)
{
  PieceVelocityProfile profile;
  const int piece_num = static_cast<int>(sfcs.size());
  if (piece_num <= 0 || path.size() < 2)
  {
    return profile;
  }

  const double v_min = std::max(0.5, cfg.minSegmentVel);
  const double non_turn_limit = std::clamp(
      std::min({global_limit.open,
                global_limit.known_free,
                global_limit.brake,
                global_limit.clearance,
                global_limit.yaw,
                global_limit.backup}),
      v_min,
      cfg.maxVelMag);
  profile.bounds = general_utils::VecDf::Constant(piece_num, non_turn_limit);
  profile.segment_piece.assign(path.size() - 1U, 0);
  profile.piece_length.assign(piece_num, 0.0);
  std::vector<double> max_turn(piece_num, 0.0);
  std::vector<double> min_radius(
      piece_num, std::numeric_limits<double>::infinity());

  int current_piece = 0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i)
  {
    const Eigen::Vector3d midpoint = 0.5 * (path[i] + path[i + 1]);
    if (!pointInsideHPoly(sfcs[current_piece].GetPlanes(), midpoint))
    {
      int matched_piece = -1;
      for (int p = current_piece + 1; p < piece_num; ++p)
      {
        if (pointInsideHPoly(sfcs[p].GetPlanes(), midpoint))
        {
          matched_piece = p;
          break;
        }
      }
      if (matched_piece < 0)
      {
        for (int p = 0; p < piece_num; ++p)
        {
          if (pointInsideHPoly(sfcs[p].GetPlanes(), midpoint))
          {
            matched_piece = p;
            break;
          }
        }
      }
      if (matched_piece >= 0)
      {
        current_piece = std::max(current_piece, matched_piece);
      }
    }
    profile.segment_piece[i] = current_piece;
    profile.piece_length[current_piece] += (path[i + 1] - path[i]).norm();
  }

  for (std::size_t i = 1; i + 1 < path.size(); ++i)
  {
    const Eigen::Vector3d a = path[i] - path[i - 1];
    const Eigen::Vector3d b = path[i + 1] - path[i];
    const double a_len = a.norm();
    const double b_len = b.norm();
    if (a_len < 0.20 || b_len < 0.20)
    {
      continue;
    }
    const double turn = std::acos(std::clamp(
        a.dot(b) / (a_len * b_len), -1.0, 1.0));
    const double twice_area = a.cross(b).norm();
    const double chord = (path[i + 1] - path[i - 1]).norm();
    const double radius =
        turn > 0.05 && twice_area > 1.0e-4 && chord > 0.20
            ? a_len * b_len * chord / (2.0 * twice_area)
            : std::numeric_limits<double>::infinity();
    const int incoming_piece = profile.segment_piece[i - 1];
    const int outgoing_piece = profile.segment_piece[i];
    for (const int piece : {incoming_piece, outgoing_piece})
    {
      max_turn[piece] = std::max(max_turn[piece], turn);
      if (std::isfinite(radius) && radius > 0.05)
      {
        min_radius[piece] = std::min(min_radius[piece], radius);
      }
    }
  }

  Eigen::Vector3d first_heading = path[1] - path[0];
  first_heading.z() = 0.0;
  Eigen::Vector3d boundary_heading = head_velocity;
  boundary_heading.z() = 0.0;
  if (boundary_heading.norm() < 0.50)
  {
    boundary_heading =
        Eigen::Vector3d(std::cos(head_yaw), std::sin(head_yaw), 0.0);
  }
  if (first_heading.norm() > 1.0e-3 && boundary_heading.norm() > 1.0e-3)
  {
    max_turn.front() = std::max(
        max_turn.front(),
        std::acos(std::clamp(first_heading.normalized().dot(
                                 boundary_heading.normalized()),
                             -1.0,
                             1.0)));
  }

  auto angleVelocityCap = [&](double turn) {
    if (turn >= cfg.turnHardAngle)
    {
      return cfg.turnHardVelocity;
    }
    if (turn < cfg.turnSoftAngle)
    {
      return cfg.maxVelMag;
    }
    const double ratio = std::clamp(
        (turn - cfg.turnSoftAngle) /
            std::max(0.05, cfg.turnHardAngle - cfg.turnSoftAngle),
        0.0,
        1.0);
    return (1.0 - ratio) * cfg.turnSoftVelocity +
           ratio * cfg.turnHardVelocity;
  };

  if (cfg.turnVelocityEnable)
  {
    for (int p = 0; p < piece_num; ++p)
    {
      profile.bounds(p) =
          std::min(profile.bounds(p), angleVelocityCap(max_turn[p]));
      if (max_turn[p] > 0.10 && std::isfinite(min_radius[p]))
      {
        const double radius = std::max(cfg.curvatureMinRadius, min_radius[p]);
        profile.bounds(p) = std::min(
            profile.bounds(p),
            std::sqrt(std::max(0.1, cfg.turnLateralAcceleration * radius)));
      }
      profile.bounds(p) = std::clamp(profile.bounds(p), v_min, cfg.maxVelMag);
    }

    const double envelope_acc = std::max(1.0, cfg.brakeAccel);
    for (int p = piece_num - 2; p >= 0; --p)
    {
      const double available_length = std::max(0.05, profile.piece_length[p]);
      profile.bounds(p) = std::min(
          profile.bounds(p),
          std::sqrt(profile.bounds(p + 1) * profile.bounds(p + 1) +
                    2.0 * envelope_acc * available_length));
    }
    for (int p = 1; p < piece_num; ++p)
    {
      const double available_length =
          std::max(0.05, profile.piece_length[p - 1]);
      profile.bounds(p) = std::min(
          profile.bounds(p),
          std::sqrt(profile.bounds(p - 1) * profile.bounds(p - 1) +
                    2.0 * envelope_acc * available_length));
    }
  }
  return profile;
}

std::vector<double> allocateGuideTimes(
    const std::vector<Eigen::Vector3d> &path,
    const PieceVelocityProfile &profile,
    const general_utils::VecDf &piece_velocity_bounds)
{
  std::vector<double> times;
  if (path.empty())
  {
    return times;
  }
  times.reserve(path.size());
  times.emplace_back(0.0);
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    const int piece = i - 1 < profile.segment_piece.size()
                          ? profile.segment_piece[i - 1]
                          : 0;
    const double speed =
        piece_velocity_bounds.size() > 0
            ? piece_velocity_bounds(std::clamp(
                  piece, 0, static_cast<int>(piece_velocity_bounds.size()) - 1))
            : 1.0;
    const double dt =
        std::max(0.05, (path[i] - path[i - 1]).norm() / std::max(0.2, speed));
    times.emplace_back(times.back() + dt);
  }
  return times;
}

void publishHighspeedTrajectoryViz(Visualizer *viz,
                                   const geometry_utils::Trajectory &traj,
                                   const std::vector<Eigen::Vector3d> &waypoints,
                                   double max_vel)
{
  if (!viz || traj.empty())
  {
    return;
  }
  std::vector<Eigen::Vector3d> samples;
  std::vector<double> speed_ratios;
  const double duration = traj.getTotalDuration();
  const double sample_dt = 0.01;
  const double vel_norm = std::max(0.1, max_vel);
  if (!std::isfinite(duration) || duration <= 1.0e-6)
  {
    return;
  }
  const std::size_t reserve_num =
      static_cast<std::size_t>(std::ceil(duration / sample_dt)) + 1U;
  samples.reserve(reserve_num);
  speed_ratios.reserve(reserve_num);
  for (double t = sample_dt; t < duration; t += sample_dt)
  {
    samples.emplace_back(traj.getPos(t));
    speed_ratios.emplace_back(traj.getVel(t).norm() / vel_norm);
  }
  viz->visualizeTrajectorySamples(samples, speed_ratios, waypoints);
}

geometry_utils::Trajectory makeHoldYawTrajectory(double yaw, double duration)
{
  geometry_utils::Trajectory out;
  if (!std::isfinite(duration) || duration <= 1.0e-6)
  {
    return out;
  }
  Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 6);
  coeff(0, 5) = yaw;
  out.emplace_back(std::max(1.0e-4, duration), coeff);
  out.start_WT = ros::Time::now().toSec();
  return out;
}

void fillPolyTrajMsg(const geometry_utils::Trajectory &traj,
                     int order,
                     int coeff_num,
                     traj_utils::PolyTraj &poly_msg,
                     const ros::Time &start_time,
                     int traj_id)
{
  poly_msg.drone_id = 0;
  poly_msg.traj_id = traj_id;
  poly_msg.start_time = start_time;
  poly_msg.order = static_cast<uint8_t>(order);
  poly_msg.duration.clear();
  poly_msg.coef_x.clear();
  poly_msg.coef_y.clear();
  poly_msg.coef_z.clear();
  const int piece_num = traj.getPieceNum();
  if (piece_num <= 0)
  {
    return;
  }
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(static_cast<std::size_t>(coeff_num * piece_num));
  poly_msg.coef_y.resize(static_cast<std::size_t>(coeff_num * piece_num));
  poly_msg.coef_z.resize(static_cast<std::size_t>(coeff_num * piece_num));
  for (int i = 0; i < piece_num; ++i)
  {
    poly_msg.duration[i] = static_cast<float>(traj[i].getDuration());
    const Eigen::MatrixXd &cMat = traj[i].getCoeffMat();
    const int available = static_cast<int>(cMat.cols());
    const int start_col = std::max(0, available - coeff_num);
    const int base = i * coeff_num;
    for (int j = 0; j < coeff_num; ++j)
    {
      const int col = start_col + j;
      const double x = col < available ? cMat(0, col) : 0.0;
      const double y = col < available ? cMat(1, col) : 0.0;
      const double z = col < available ? cMat(2, col) : 0.0;
      poly_msg.coef_x[base + j] = static_cast<float>(x);
      poly_msg.coef_y[base + j] = static_cast<float>(y);
      poly_msg.coef_z[base + j] = static_cast<float>(z);
    }
  }
}

general_utils::vec_E<general_utils::Vec3f> toVec3fPath(const std::vector<Eigen::Vector3d> &path)
{
  general_utils::vec_E<general_utils::Vec3f> out;
  out.reserve(path.size());
  for (const auto &p : path)
  {
    out.emplace_back(p);
  }
  return out;
}

std::vector<Eigen::MatrixX4d> hPolysFromSfcs(const geometry_utils::PolytopeVec &sfcs)
{
  std::vector<Eigen::MatrixX4d> h_polys;
  h_polys.reserve(sfcs.size());
  for (const auto &sfc : sfcs)
  {
    h_polys.emplace_back(sfc.GetPlanes());
  }
  return h_polys;
}

bool polytopesOverlap(const geometry_utils::Polytope &a,
                      const geometry_utils::Polytope &b,
                      double min_depth)
{
  if (a.empty() || b.empty())
  {
    return false;
  }
  const geometry_utils::Polytope overlap = a.CrossWith(b);
  if (overlap.empty())
  {
    return false;
  }
  general_utils::Vec3f interior_pt = general_utils::Vec3f::Zero();
  const double depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
  return std::isfinite(depth) && depth > min_depth;
}

bool validateSfcSequence(const Eigen::Vector3d &head,
                         const Eigen::Vector3d &tail,
                         const geometry_utils::PolytopeVec &sfcs,
                         double min_overlap_depth,
                         std::string &reason)
{
  if (sfcs.empty())
  {
    reason = "empty_sfc";
    return false;
  }
  if (!pointInsideHPoly(sfcs.front().GetPlanes(), head))
  {
    reason = "head_outside_first_poly";
    return false;
  }
  if (!pointInsideHPoly(sfcs.back().GetPlanes(), tail))
  {
    reason = "tail_outside_last_poly";
    return false;
  }
  for (std::size_t i = 1; i < sfcs.size(); ++i)
  {
    if (!polytopesOverlap(sfcs[i - 1], sfcs[i], min_overlap_depth))
    {
      reason = "disconnected_poly_" + std::to_string(i - 1) + "_" + std::to_string(i);
      return false;
    }
  }
  return true;
}

bool validateTrajectoryForCommit(const geometry_utils::Trajectory &pos_traj,
                                 const geometry_utils::Trajectory &yaw_traj,
                                 double max_vel,
                                 double max_acc,
                                 double max_yaw_rate,
                                 double sample_dt,
                                 std::string &reason)
{
  if (pos_traj.empty() || yaw_traj.empty())
  {
    reason = "empty_traj";
    return false;
  }
  if (!std::isfinite(pos_traj.start_WT) || !std::isfinite(yaw_traj.start_WT))
  {
    reason = "nonfinite_start_time";
    return false;
  }
  const double pos_duration = pos_traj.getTotalDuration();
  const double yaw_duration = yaw_traj.getTotalDuration();
  if (!std::isfinite(pos_duration) || !std::isfinite(yaw_duration) ||
      pos_duration <= 1.0e-5 || yaw_duration <= 1.0e-5)
  {
    reason = "bad_duration";
    return false;
  }
  if (std::abs(pos_duration - yaw_duration) > std::max(0.05, 2.0 * sample_dt))
  {
    reason = "pos_yaw_duration_mismatch";
    return false;
  }
  for (int i = 0; i < pos_traj.getPieceNum(); ++i)
  {
    if (!std::isfinite(pos_traj[i].getDuration()) || pos_traj[i].getDuration() <= 1.0e-6 ||
        !pos_traj[i].getCoeffMat().allFinite())
    {
      reason = "bad_pos_piece_" + std::to_string(i);
      return false;
    }
  }
  for (int i = 0; i < yaw_traj.getPieceNum(); ++i)
  {
    if (!std::isfinite(yaw_traj[i].getDuration()) || yaw_traj[i].getDuration() <= 1.0e-6 ||
        !yaw_traj[i].getCoeffMat().allFinite())
    {
      reason = "bad_yaw_piece_" + std::to_string(i);
      return false;
    }
  }

  sample_dt = std::max(0.02, sample_dt);
  const int max_samples = 2000;
  if (pos_duration / sample_dt > max_samples)
  {
    sample_dt = pos_duration / static_cast<double>(max_samples);
  }
  const double vel_limit = std::max(0.2, max_vel) * 1.35;
  const double acc_limit = std::max(0.2, max_acc) * 1.35;
  const double yaw_rate_limit = std::max(0.2, max_yaw_rate) * 1.35;
  for (double t = 0.0; t <= pos_duration + 1.0e-9; t += sample_dt)
  {
    const double tt = std::min(t, pos_duration);
    const Eigen::Vector3d pos = pos_traj.getPos(tt);
    const Eigen::Vector3d vel = pos_traj.getVel(tt);
    const Eigen::Vector3d acc = pos_traj.getAcc(tt);
    const Eigen::Vector3d jer = pos_traj.getJer(tt);
    const Eigen::Vector3d yaw_rate = yaw_traj.getVel(std::min(tt, yaw_duration));
    if (!pos.allFinite() || !vel.allFinite() || !acc.allFinite() ||
        !jer.allFinite() || !yaw_rate.allFinite())
    {
      reason = "sample_nonfinite";
      return false;
    }
    if (vel.norm() > vel_limit + 1.0e-6)
    {
      reason = "velocity_limit";
      return false;
    }
    if (acc.norm() > acc_limit + 1.0e-6)
    {
      reason = "acceleration_limit";
      return false;
    }
    if (yaw_rate.norm() > yaw_rate_limit + 1.0e-6)
    {
      reason = "yaw_rate_limit";
      return false;
    }
    if (tt >= pos_duration)
    {
      break;
    }
  }
  return true;
}

struct PathProjectionInfo
{
  double distance{std::numeric_limits<double>::infinity()};
  std::size_t segment{0U};
  double alpha{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  bool valid{false};
};

PathProjectionInfo nearestPathProjection(const std::vector<Eigen::Vector3d> &path,
                                         const Eigen::Vector3d &start)
{
  PathProjectionInfo info;
  if (path.empty())
  {
    return info;
  }
  if (path.size() == 1U)
  {
    info.distance = (path.front() - start).norm();
    info.point = path.front();
    info.valid = std::isfinite(info.distance);
    return info;
  }

  for (std::size_t i = 0; i + 1U < path.size(); ++i)
  {
    const Eigen::Vector3d seg = path[i + 1U] - path[i];
    const double len2 = seg.squaredNorm();
    double alpha = 0.0;
    Eigen::Vector3d proj = path[i];
    if (len2 > 1.0e-8)
    {
      alpha = std::clamp((start - path[i]).dot(seg) / len2, 0.0, 1.0);
      proj = path[i] + alpha * seg;
    }
    const double dist = (start - proj).norm();
    if (dist < info.distance)
    {
      info.distance = dist;
      info.segment = i;
      info.alpha = alpha;
      info.point = proj;
      info.valid = std::isfinite(dist);
    }
  }
  return info;
}

bool appendUniquePathPoint(std::vector<Eigen::Vector3d> &path,
                           const Eigen::Vector3d &point,
                           double min_distance)
{
  if (!point.allFinite())
  {
    return false;
  }
  if (path.empty() || (path.back() - point).norm() > min_distance)
  {
    path.emplace_back(point);
  }
  else
  {
    path.back() = point;
  }
  return true;
}

bool alignPathStart(std::vector<Eigen::Vector3d> &path,
                    const Eigen::Vector3d &start,
                    bool trim_to_projection,
                    double max_projection_distance)
{
  if (!start.allFinite())
  {
    return false;
  }
  if (path.empty())
  {
    path.emplace_back(start);
    return true;
  }
  if ((path.front() - start).norm() <= 0.05)
  {
    path.front() = start;
    return true;
  }
  if (trim_to_projection && path.size() >= 2U)
  {
    const PathProjectionInfo projection = nearestPathProjection(path, start);
    if (!projection.valid ||
        projection.distance > std::max(0.05, max_projection_distance))
    {
      return false;
    }

    std::vector<Eigen::Vector3d> trimmed;
    trimmed.reserve(path.size() - projection.segment + 2U);
    appendUniquePathPoint(trimmed, start, 0.05);

    std::size_t suffix_start = projection.segment + 1U;
    if (projection.alpha >= 1.0 - 1.0e-3)
    {
      suffix_start = projection.segment + 2U;
    }
    for (std::size_t i = suffix_start; i < path.size(); ++i)
    {
      appendUniquePathPoint(trimmed, path[i], 0.05);
    }
    if (trimmed.size() < 2U && !path.empty())
    {
      appendUniquePathPoint(trimmed, path.back(), 0.05);
    }
    if (trimmed.size() < 2U)
    {
      return false;
    }
    path.swap(trimmed);
    return true;
  }
  path.insert(path.begin(), start);
  return true;
}

bool pointCloud2ToRogCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                           rog_map::PointCloud &cloud)
{
  cloud.clear();
  if (!cloud_msg)
  {
    return false;
  }
  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    const std::size_t reserve_num =
        static_cast<std::size_t>(cloud_msg->width) * static_cast<std::size_t>(cloud_msg->height);
    cloud.reserve(reserve_num);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        continue;
      }
      rog_map::PointCloud::PointType p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = 0.0f;
      cloud.push_back(p);
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "[highspeed_exp adapter] cannot parse PointCloud2 for ROGMap: "
                                      << e.what());
    cloud.clear();
    return false;
  }
  return !cloud.empty();
}
} // namespace

void GcopterConfig::init(const ros::NodeHandle &nh_priv)
{
  nh_priv.param("DilateRadiusSoft", dilateRadiusSoft, dilateRadiusSoft);
  nh_priv.param("DilateRadiusHard", dilateRadiusHard, dilateRadiusHard);
  nh_priv.param("MaxVelMag", maxVelMag, maxVelMag);
  nh_priv.param("MaxAccMag", maxAccMag, maxAccMag);
  nh_priv.param("maxBdrMag", maxBdrMag, maxBdrMag);
  nh_priv.param("MaxTiltAngle", maxTiltAngle, maxTiltAngle);
  nh_priv.param("MinThrust", minThrust, minThrust);
  nh_priv.param("MaxThrust", maxThrust, maxThrust);
  nh_priv.param("VehicleMass", vehicleMass, vehicleMass);
  nh_priv.param("GravAcc", gravAcc, gravAcc);
  nh_priv.param("HorizDrag", horizDrag, horizDrag);
  nh_priv.param("VertDrag", vertDrag, vertDrag);
  nh_priv.param("ParasDrag", parasDrag, parasDrag);
  nh_priv.param("SpeedEps", speedEps, speedEps);
  nh_priv.param("WeightT", weightT, weightT);
  nh_priv.param("WeightSafeT", WeightSafeT, WeightSafeT);
  nh_priv.param("EnergyWeight", energyWeight, energyWeight);
  nh_priv.getParam("ChiVec", chiVec);
  nh_priv.param("SmoothingEps", smoothingEps, smoothingEps);
  nh_priv.param("IntegralIntervs", integralIntervs, integralIntervs);
  nh_priv.param("RelCostTol", relCostTol, relCostTol);
  nh_priv.param("MaxCorridorSize", corridor_size, corridor_size);
  nh_priv.param("yaw_rho_vis", yaw_rho_vis, yaw_rho_vis);
  nh_priv.param("yaw_max_vel", yaw_max_vel, yaw_max_vel);
  nh_priv.param("yaw_time_fwd", yaw_time_fwd, yaw_time_fwd);
  nh_priv.param("RogMapEnable", rogMapEnable, rogMapEnable);
  nh_priv.param("GeneralCorridorEnable", generalCorridorEnable, generalCorridorEnable);
  nh_priv.param("CorridorUseRogOccPoints", corridorUseRogOccPoints, corridorUseRogOccPoints);
  nh_priv.param("RogKnownFreeFallbackToLio", rogKnownFreeFallbackToLio, rogKnownFreeFallbackToLio);
  nh_priv.param("RogMapConfigPath", rogMapConfigPath, rogMapConfigPath);
  nh_priv.param("CorridorLineMaxLength", corridorLineMaxLength, corridorLineMaxLength);
  nh_priv.param("CorridorMinOverlapThreshold", corridorMinOverlapThreshold, corridorMinOverlapThreshold);
  nh_priv.param("CorridorRobotRadius", corridorRobotRadius, corridorRobotRadius);
  corridorRobotRadius = std::max(corridorRobotRadius, dilateRadiusHard);
  nh_priv.param("CorridorMaxStartShift", corridorMaxStartShift, corridorMaxStartShift);
  nh_priv.param("CorridorBoxSearchSkipNum", corridorBoxSearchSkipNum, corridorBoxSearchSkipNum);
  nh_priv.param("CorridorIrisIterNum", corridorIrisIterNum, corridorIrisIterNum);
  nh_priv.param("CorridorVirtualGroundHeight", corridorVirtualGroundHeight, corridorVirtualGroundHeight);
  nh_priv.param("CorridorVirtualCeilHeight", corridorVirtualCeilHeight, corridorVirtualCeilHeight);
  nh_priv.param("DynamicVelocityEnable", dynamicVelocityEnable, dynamicVelocityEnable);
  nh_priv.param("MinSegmentVel", minSegmentVel, minSegmentVel);
  nh_priv.param("TrajectoryRetryMinVel", trajectoryRetryMinVel,
                trajectoryRetryMinVel);
  trajectoryRetryMinVel =
      std::clamp(trajectoryRetryMinVel, 0.3, std::max(0.3, minSegmentVel));
  nh_priv.param("OpenSegmentVel", openSegmentVel, maxVelMag);
  nh_priv.param("DynamicVelocityMinClearance", dynamicVelocityMinClearance, dynamicVelocityMinClearance);
  nh_priv.param("DynamicVelocityOpenClearance", dynamicVelocityOpenClearance, dynamicVelocityOpenClearance);
  nh_priv.param("DynamicVelocityClearanceMargin", dynamicVelocityClearanceMargin,
                std::max(dynamicVelocityClearanceMargin, dilateRadiusHard + 0.15));
  nh_priv.param("TurnVelocityEnable", turnVelocityEnable, turnVelocityEnable);
  nh_priv.param("TurnLateralAcceleration", turnLateralAcceleration,
                turnLateralAcceleration);
  nh_priv.param("TurnSoftAngle", turnSoftAngle, turnSoftAngle);
  nh_priv.param("TurnHardAngle", turnHardAngle, turnHardAngle);
  nh_priv.param("TurnSoftVelocity", turnSoftVelocity, turnSoftVelocity);
  nh_priv.param("TurnHardVelocity", turnHardVelocity, turnHardVelocity);
  nh_priv.param("ReorientationHeadingAngle", reorientationHeadingAngle,
                reorientationHeadingAngle);
  turnLateralAcceleration = std::max(0.5, turnLateralAcceleration);
  turnSoftAngle = std::clamp(turnSoftAngle, 0.05, M_PI - 0.10);
  turnHardAngle = std::clamp(turnHardAngle, turnSoftAngle + 0.05, M_PI);
  turnSoftVelocity = std::clamp(turnSoftVelocity, minSegmentVel, maxVelMag);
  turnHardVelocity = std::clamp(turnHardVelocity, minSegmentVel,
                                turnSoftVelocity);
  reorientationHeadingAngle =
      std::clamp(reorientationHeadingAngle, turnHardAngle, M_PI);
  nh_priv.param("NonstopTerminalVelocityEnable", nonstopTerminalVelocityEnable, nonstopTerminalVelocityEnable);
  nh_priv.param("NonstopTerminalVelocityRatio", nonstopTerminalVelocityRatio, nonstopTerminalVelocityRatio);
  nh_priv.param("NonstopTerminalMinPathLength", nonstopTerminalMinPathLength, nonstopTerminalMinPathLength);
  nh_priv.param("NonstopTerminalMaxTurnAngle", nonstopTerminalMaxTurnAngle,
                nonstopTerminalMaxTurnAngle);
  nh_priv.param("NonstopTerminalMaxYawDelta", nonstopTerminalMaxYawDelta,
                nonstopTerminalMaxYawDelta);
  nh_priv.param("BackupTrajEnable", backupTrajEnable, backupTrajEnable);
  nh_priv.param("BackupStartRatio", backupStartRatio, backupStartRatio);
  nh_priv.param("BackupMinStartTime", backupMinStartTime, backupMinStartTime);
  nh_priv.param("BackupMaxStartTime", backupMaxStartTime, backupMaxStartTime);
  nh_priv.param("BackupSampleDt", backupSampleDt, backupSampleDt);
  nh_priv.param("BackupSearchMargin", backupSearchMargin, backupSearchMargin);
  nh_priv.param("BackupPieceNum", backupPieceNum, backupPieceNum);
  nh_priv.param("BackupMaxVel", backupMaxVel, std::min(maxVelMag, backupMaxVel));
  nh_priv.param("BackupMaxAcc", backupMaxAcc, maxAccMag);
  nh_priv.param("ReplanCommitDelay", replanCommitDelay, replanCommitDelay);
  nh_priv.param("CommitMinDuration", commitMinDuration, commitMinDuration);
  nh_priv.param("CommitMaxDuration", commitMaxDuration, commitMaxDuration);
  nh_priv.param("CommitSampleDt", commitSampleDt, commitSampleDt);
  nh_priv.param("CommitKnownFreeSafeDistance", commitKnownFreeSafeDistance,
                std::max(commitKnownFreeSafeDistance, dilateRadiusHard + 0.15));
  nh_priv.param("CommitBackupTimeBuffer", commitBackupTimeBuffer, commitBackupTimeBuffer);
  nh_priv.param("KnownFreeShortLength", knownFreeShortLength, knownFreeShortLength);
  nh_priv.param("KnownFreeMediumLength", knownFreeMediumLength, knownFreeMediumLength);
  nh_priv.param("KnownFreeLongLength", knownFreeLongLength, knownFreeLongLength);
  nh_priv.param("VelocityShortKnownFree", velocityShortKnownFree, velocityShortKnownFree);
  nh_priv.param("VelocityMediumKnownFree", velocityMediumKnownFree, velocityMediumKnownFree);
  nh_priv.param("VelocityLongKnownFree", velocityLongKnownFree, maxVelMag);
  nh_priv.param("SafetyMapQueryStep", safetyMapQueryStep, safetyMapQueryStep);
  nh_priv.param("SafetyMapUnknownAsOccupiedForCommit",
                safetyMapUnknownAsOccupiedForCommit, safetyMapUnknownAsOccupiedForCommit);
  nh_priv.param("SafetyMapUnknownAsOccupiedForBackup",
                safetyMapUnknownAsOccupiedForBackup, safetyMapUnknownAsOccupiedForBackup);
  nh_priv.param("SafetyMapUnknownAllowedForExplore",
                safetyMapUnknownAllowedForExplore, safetyMapUnknownAllowedForExplore);
  nh_priv.param("BrakeAccel", brakeAccel, backupMaxAcc);
  nh_priv.param("PlannerLatency", plannerLatency, plannerLatency);
  nh_priv.param("ControlLatency", controlLatency, controlLatency);
  nh_priv.param("SafetyBrakeMargin", safetyBrakeMargin, safetyBrakeMargin);
  nh_priv.param("CurvatureMinRadius", curvatureMinRadius, curvatureMinRadius);
  nh_priv.param("VelocityLogEnable", velocityLogEnable, velocityLogEnable);
  nh_priv.param("HighSpeedModeThreshold", highSpeedModeThreshold, highSpeedModeThreshold);
  nh_priv.param("HighSpeedModeExitThreshold", highSpeedModeExitThreshold,
                highSpeedModeExitThreshold);
  highSpeedModeExitThreshold =
      std::clamp(highSpeedModeExitThreshold, 0.0, highSpeedModeThreshold);
  nh_priv.param("ViewScoreGainWeight", viewScoreGainWeight, viewScoreGainWeight);
  nh_priv.param("ViewScoreProgressWeight", viewScoreProgressWeight, viewScoreProgressWeight);
  nh_priv.param("ViewScoreVelocityAlignWeight", viewScoreVelocityAlignWeight, viewScoreVelocityAlignWeight);
  nh_priv.param("ViewScoreKnownFreeWeight", viewScoreKnownFreeWeight, viewScoreKnownFreeWeight);
  nh_priv.param("ViewScoreClearanceWeight", viewScoreClearanceWeight, viewScoreClearanceWeight);
  nh_priv.param("ViewScoreYawWeight", viewScoreYawWeight, viewScoreYawWeight);
  nh_priv.param("ViewScoreTurnWeight", viewScoreTurnWeight, viewScoreTurnWeight);
  nh_priv.param("ViewScoreBackupPenalty", viewScoreBackupPenalty, viewScoreBackupPenalty);
  nh_priv.param("ViewScoreKnownFreeMaxLen", viewScoreKnownFreeMaxLen, viewScoreKnownFreeMaxLen);
  nh_priv.param("ViewScoreHardGateEnable", viewScoreHardGateEnable, viewScoreHardGateEnable);
  nh_priv.param("ViewScoreHardGateMinKnownFreeRatio",
                viewScoreHardGateMinKnownFreeRatio, viewScoreHardGateMinKnownFreeRatio);
  nh_priv.param("ViewScoreHardGateMaxTurnAngle",
                viewScoreHardGateMaxTurnAngle, viewScoreHardGateMaxTurnAngle);
  nh_priv.param("ViewScoreHardGateMaxYawDelta",
                viewScoreHardGateMaxYawDelta, viewScoreHardGateMaxYawDelta);
  nh_priv.param("ViewScoreHardGateMinClearance",
                viewScoreHardGateMinClearance, commitKnownFreeSafeDistance);
  nh_priv.param("ViewScoreTopCandidateNum", viewScoreTopCandidateNum, viewScoreTopCandidateNum);
  nh_priv.param("EdgeTurnPenaltyWeight", edgeTurnPenaltyWeight, edgeTurnPenaltyWeight);
  nh_priv.param("EdgeKnownFreePenaltyWeight", edgeKnownFreePenaltyWeight, edgeKnownFreePenaltyWeight);
  nh_priv.param("EdgeBackupPenaltyWeight", edgeBackupPenaltyWeight, edgeBackupPenaltyWeight);
  nh_priv.param("EdgeYawPenaltyWeight", edgeYawPenaltyWeight, edgeYawPenaltyWeight);
  nh_priv.param("CorridorCruiseEnable", corridorCruiseEnable, corridorCruiseEnable);
  nh_priv.param("CorridorCruiseKnownFreeLength",
                corridorCruiseKnownFreeLength, knownFreeLongLength);
  nh_priv.param("CorridorCruiseMinAlignment", corridorCruiseMinAlignment, corridorCruiseMinAlignment);
  nh_priv.param("CorridorCruiseForwardWeight", corridorCruiseForwardWeight, corridorCruiseForwardWeight);
  nh_priv.param("CorridorCruiseLateralPenalty", corridorCruiseLateralPenalty, corridorCruiseLateralPenalty);
  nh_priv.param("CorridorCruiseMaxBacktrackDistance",
                corridorCruiseMaxBacktrackDistance, corridorCruiseMaxBacktrackDistance);
  nh_priv.param("CorridorCruiseMinProgress", corridorCruiseMinProgress, corridorCruiseMinProgress);
  nh_priv.param("CorridorCruiseMaxGoalDistance", corridorCruiseMaxGoalDistance, corridorCruiseMaxGoalDistance);
}

FastPlannerManager::FastPlannerManager()
{
  gcopter_config_ = std::make_unique<GcopterConfig>();
  commit_store_ = std::make_unique<GeneralCommitStore>();
  committed_pos_traj_ = std::make_shared<geometry_utils::Trajectory>();
  committed_yaw_traj_ = std::make_shared<geometry_utils::Trajectory>();
  latest_exp_pos_traj_ = std::make_shared<geometry_utils::Trajectory>();
  latest_exp_yaw_traj_ = std::make_shared<geometry_utils::Trajectory>();
}

FastPlannerManager::~FastPlannerManager() = default;

void FastPlannerManager::printTimeCost(double time_threshold, double time_cost, std::string print_info)
{
  if (time_cost >= time_threshold)
  {
    ROS_WARN_STREAM(print_info << " time cost: " << time_cost << " ms");
  }
}

void FastPlannerManager::initPlanModules(ros::NodeHandle &nh,
                                         ParallelBubbleAstar::Ptr &parallel_path_finder,
                                         TopoGraph::Ptr &graph)
{
  gcopter_config_->init(nh);
  nh.param("max_traj_len", max_traj_len_, 12.0);
  parallel_path_finder_ = parallel_path_finder;
  topo_graph_ = graph;
  lidar_map_interface_ = topo_graph_ ? topo_graph_->lidar_map_interface_ : nullptr;
  ros_ptr_ = std::make_shared<NullRosInterface>();
  exploration_traj_opt_ =
      std::make_shared<traj_opt::ExplorationTrajOpt>(makeGeneralExpConfig(*gcopter_config_), ros_ptr_);
  backup_traj_opt_ =
      std::make_shared<traj_opt::BackupTrajOpt>(makeGeneralBackupConfig(*gcopter_config_), ros_ptr_);
  yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(std::max(0.2, gcopter_config_->yaw_max_vel));

  rog_map_updated_ = false;
  if (gcopter_config_->rogMapEnable)
  {
    if (gcopter_config_->rogMapConfigPath.empty())
    {
      ROS_WARN("[highspeed_exp adapter] RogMapEnable is true, but RogMapConfigPath is empty.");
    }
    else
    {
      try
      {
        rog_map_ = std::make_shared<rog_map::ROGMapROS>(nh, gcopter_config_->rogMapConfigPath);
        map_manager_ = std::make_shared<general_planner::MapManager>(rog_map_);
        corridor_generator_ = std::make_shared<general_planner::CorridorGenerator>(
            ros_ptr_,
            map_manager_,
            std::max(0.2, gcopter_config_->corridor_size),
            std::max(0.2, gcopter_config_->corridorLineMaxLength),
            std::max(0.01, gcopter_config_->corridorMinOverlapThreshold),
            gcopter_config_->corridorVirtualGroundHeight,
            gcopter_config_->corridorVirtualCeilHeight,
            std::max(0.0, gcopter_config_->corridorRobotRadius),
            std::max(1, gcopter_config_->corridorBoxSearchSkipNum),
            std::max(1, gcopter_config_->corridorIrisIterNum));
        const int neighbor_step = static_cast<int>(std::ceil(
            std::max(0.0, gcopter_config_->corridorRobotRadius) /
            std::max(0.01, map_manager_->getResolution())));
        rog_map::vec_E<rog_map::Vec3i> seed_neighbors;
        for (int x = -neighbor_step; x <= neighbor_step; ++x)
        {
          for (int y = -neighbor_step; y <= neighbor_step; ++y)
          {
            for (int z = -neighbor_step; z <= neighbor_step; ++z)
            {
              if (x * x + y * y + z * z <= neighbor_step * neighbor_step)
              {
                seed_neighbors.emplace_back(x, y, z);
              }
            }
          }
        }
        std::sort(seed_neighbors.begin(), seed_neighbors.end(),
                  [](const rog_map::Vec3i &a, const rog_map::Vec3i &b) {
                    return a.squaredNorm() < b.squaredNorm();
                  });
        corridor_generator_->SetLineNeighborList(seed_neighbors);
        ROS_INFO_STREAM("[highspeed_exp adapter] ROG corridor backend initialized: "
                        << gcopter_config_->rogMapConfigPath);
      }
      catch (const std::exception &e)
      {
        ROS_WARN_STREAM("[highspeed_exp adapter] Failed to initialize ROG corridor backend: "
                        << e.what());
        rog_map_.reset();
        map_manager_.reset();
        corridor_generator_.reset();
      }
    }
  }

  bubble_path_finder_ = std::make_shared<BubbleAstar>();
  if (lidar_map_interface_)
  {
    bubble_path_finder_->init(nh, lidar_map_interface_);
  }
  fast_searcher_ = std::make_shared<FastSearcher>();
  fast_searcher_->init(topo_graph_, bubble_path_finder_);
  graph_visualizer_ = std::make_shared<GraphVisualizer>();
  graph_visualizer_->init(nh);
  gcopter_viz_ = std::make_unique<Visualizer>();
  gcopter_viz_->init(nh);

  local_data_.start_time_ = ros::Time::now();
  local_data_.duration_ = 0.0;
  local_data_.traj_id_ = 0;
}

bool FastPlannerManager::getCommittedReplanHeadState(Eigen::Vector3d &pos,
                                                     Eigen::Vector3d &vel,
                                                     double &yaw,
                                                     double *traj_time,
                                                     double *switch_delay)
{
  if (!commit_store_ || !gcopter_config_)
  {
    return false;
  }

  geometry_utils::Trajectory pos_traj;
  geometry_utils::Trajectory yaw_traj;
  commit_store_->cmd_traj_info.lock();
  const bool empty = commit_store_->cmd_traj_info.empty();
  if (!empty)
  {
    pos_traj = commit_store_->cmd_traj_info.posTraj();
    yaw_traj = commit_store_->cmd_traj_info.yawTraj();
  }
  commit_store_->cmd_traj_info.unlock();
  if (empty || pos_traj.empty())
  {
    return false;
  }

  const double duration = pos_traj.getTotalDuration();
  if (!std::isfinite(duration) || duration <= 1.0e-4 ||
      !std::isfinite(pos_traj.start_WT))
  {
    return false;
  }

  const double delay = std::clamp(gcopter_config_->replanCommitDelay, 0.05, 1.50);
  const double now_tt = std::clamp(ros::Time::now().toSec() - pos_traj.start_WT,
                                  0.0,
                                  duration);
  const double sample_tt = now_tt + delay;
  if (sample_tt > duration - 1.0e-4)
  {
    return false;
  }

  general_utils::StatePVAJ state = general_utils::StatePVAJ::Zero();
  if (!pos_traj.getState(sample_tt, state) || !state.allFinite())
  {
    return false;
  }

  pos = state.col(0);
  vel = state.col(1);
  yaw = local_data_.curr_yaw_;
  if (!yaw_traj.empty())
  {
    const double yaw_duration = yaw_traj.getTotalDuration();
    if (std::isfinite(yaw_duration) && sample_tt <= yaw_duration + 1.0e-6)
    {
      const double yaw_tt = std::clamp(sample_tt, 0.0, yaw_duration);
      const Eigen::Vector3d yaw_pos = yaw_traj.getPos(yaw_tt);
      if (yaw_pos.allFinite())
      {
        yaw = yaw_pos.x();
      }
    }
  }

  if (!pos.allFinite() || !vel.allFinite() || !std::isfinite(yaw))
  {
    return false;
  }
  if (traj_time)
  {
    *traj_time = sample_tt;
  }
  if (switch_delay)
  {
    *switch_delay = delay;
  }
  return true;
}

bool FastPlannerManager::planExploreTraj(const std::vector<Eigen::Vector3f> &path, bool is_static)
{
  const ros::Time plan_process_start = ros::Time::now();
  last_frontend_path_ = path;
  if (!exploration_traj_opt_ || !yaw_traj_opt_)
  {
    ROS_ERROR("[highspeed_exp adapter] General MINCO backend is not initialized.");
    return false;
  }
  if (path.size() < 2)
  {
    ROS_WARN_STREAM("[highspeed_exp adapter] frontend path too short: " << path.size());
    return false;
  }

  geometry_utils::Trajectory guide_pos_traj;
  geometry_utils::Trajectory guide_yaw_traj;
  const double replan_process_start_wt = plan_process_start.toSec();
  double replan_process_start_tt = 0.0;
  double replan_state_tt = 0.0;
  double committed_duration = 0.0;
  double switch_delay = 0.0;
  bool use_committed_replan_state = false;

  general_utils::StatePVAJ head = general_utils::StatePVAJ::Zero();
  general_utils::Vec4f yaw_init = general_utils::Vec4f::Zero();
  yaw_init(0) = local_data_.curr_yaw_;
  auto useCurrentOdomHead = [&]() {
    head.setZero();
    head.col(0) = local_data_.curr_pos_;
    yaw_init.setZero();
    yaw_init(0) = local_data_.curr_yaw_;
    if (!is_static && local_data_.curr_vel_.allFinite() &&
        local_data_.curr_vel_.norm() <= 1.5 * std::max(0.2, gcopter_config_->maxVelMag))
    {
      head.col(1) = local_data_.curr_vel_;
    }
  };

  if (!is_static && !commit_store_->cmd_traj_info.empty() &&
      !commit_store_->last_exp_traj_info.empty())
  {
    commit_store_->cmd_traj_info.lock();
    guide_pos_traj = commit_store_->cmd_traj_info.posTraj();
    guide_yaw_traj = commit_store_->cmd_traj_info.yawTraj();
    commit_store_->cmd_traj_info.unlock();
    committed_duration = guide_pos_traj.getTotalDuration();
    if (!guide_pos_traj.empty() && std::isfinite(committed_duration) &&
        committed_duration > 1.0e-4 && std::isfinite(guide_pos_traj.start_WT))
    {
      replan_process_start_tt =
          std::clamp(replan_process_start_wt - guide_pos_traj.start_WT,
                     0.0,
                     committed_duration);
      switch_delay = std::clamp(gcopter_config_->replanCommitDelay, 0.05, 1.50);
      replan_state_tt = replan_process_start_tt + switch_delay;
      use_committed_replan_state =
          replan_state_tt <= committed_duration + 1.0e-6 &&
          replan_state_tt > replan_process_start_tt + 1.0e-4 &&
          guide_pos_traj.getState(replan_state_tt, head) &&
          head.allFinite();
      if (use_committed_replan_state && !guide_yaw_traj.empty())
      {
        const double yaw_duration = guide_yaw_traj.getTotalDuration();
        if (std::isfinite(yaw_duration) && replan_state_tt <= yaw_duration + 1.0e-6)
        {
          const double yaw_t = std::clamp(replan_state_tt, 0.0, yaw_duration);
          yaw_init(0) = guide_yaw_traj.getPos(yaw_t).x();
          yaw_init(1) = guide_yaw_traj.getVel(yaw_t).x();
          yaw_init(2) = guide_yaw_traj.getAcc(yaw_t).x();
          yaw_init(3) = guide_yaw_traj.getJer(yaw_t).x();
        }
      }
    }
  }

  if (!use_committed_replan_state)
  {
    useCurrentOdomHead();
  }

  std::vector<Eigen::Vector3d> local_path = shortenPath(path, max_traj_len_);
  if (local_path.size() < 2)
  {
    return false;
  }
  double committed_head_path_dist = std::numeric_limits<double>::infinity();
  if (use_committed_replan_state)
  {
    const PathProjectionInfo projection = nearestPathProjection(local_path, head.col(0));
    committed_head_path_dist = projection.valid ? projection.distance
                                                : std::numeric_limits<double>::infinity();
  }
  const double max_head_projection_dist =
      std::max({0.75,
                2.5 * std::max(0.05, gcopter_config_->corridorMaxStartShift),
                0.25 * local_data_.curr_vel_.norm() * std::max(0.05, switch_delay) + 0.50});
  if (!alignPathStart(local_path,
                      head.col(0),
                      use_committed_replan_state,
                      max_head_projection_dist))
  {
    ROS_WARN_STREAM("[highspeed_exp adapter] reject frontend path: failed to align optimization head."
                    << " head_path_dist=" << committed_head_path_dist
                    << " max_allowed=" << max_head_projection_dist
                    << " switch_delay=" << switch_delay
                    << " speed=" << local_data_.curr_vel_.norm());
    return false;
  }
  if (local_path.size() < 2)
  {
    return false;
  }

  const general_utils::vec_E<general_utils::Vec3f> guide_path = toVec3fPath(local_path);

  geometry_utils::PolytopeVec sfcs;
  bool used_general_corridor = false;
  double shifted_start_dist = 0.0;
  const bool try_general_corridor = gcopter_config_->generalCorridorEnable;
  const bool general_corridor_ready =
      try_general_corridor && corridor_generator_ && rog_map_updated_;
  if (try_general_corridor && !general_corridor_ready)
  {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[highspeed_exp adapter] General corridor backend/map is not ready; keep the committed trajectory:"
            << " generator=" << static_cast<bool>(corridor_generator_)
            << " rog_updated=" << rog_map_updated_);
    return false;
  }
  if (general_corridor_ready)
  {
    try
    {
      general_utils::Vec3f shifted_start_pt = local_path.front();
      used_general_corridor =
          corridor_generator_->SearchPolytopeOnPath(guide_path, sfcs, shifted_start_pt, false);
      shifted_start_dist = (shifted_start_pt - local_path.front()).norm();
      if (shifted_start_dist > 0.05)
      {
        ROS_WARN_STREAM_THROTTLE(
            1.0,
                "[highspeed_exp adapter] corridor generator shifted occupied start by "
                << shifted_start_dist << " m; keep optimization head at committed/current state.");
      }
    }
    catch (const std::exception &e)
    {
      ROS_WARN_STREAM("[highspeed_exp adapter] general corridor generation threw exception: "
                      << e.what());
      sfcs.clear();
    }
  }

  if (!used_general_corridor || sfcs.empty())
  {
    ROS_WARN("[highspeed_exp adapter] General corridor generation failed; keep the committed trajectory.");
    return false;
  }
  std::vector<Eigen::MatrixX4d> h_polys = hPolysFromSfcs(sfcs);
  if (!pointInsideHPoly(h_polys.front(), local_path.front()))
  {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[highspeed_exp adapter] General corridor does not contain optimization head.");
    return false;
  }
  if (!pointInsideHPoly(h_polys.back(), local_path.back()))
  {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[highspeed_exp adapter] General corridor does not contain optimization tail.");
    return false;
  }
  if (sfcs.empty())
  {
      ROS_WARN("[highspeed_exp adapter] empty safe flight corridor.");
      return false;
  }
  const std::size_t raw_sfc_count = sfcs.size();
  if (!geometry_utils::SimplifySFC(local_path.front(), local_path.back(), sfcs) ||
      sfcs.empty())
  {
    ROS_WARN("[highspeed_exp adapter] General corridor simplification failed.");
    return false;
  }
  const std::size_t simplified_sfc_count = sfcs.size();
  h_polys = hPolysFromSfcs(sfcs);
  std::string sfc_reject_reason;
  if (!validateSfcSequence(local_path.front(),
                           local_path.back(),
                           sfcs,
                           std::max(1.0e-3, 0.25 * gcopter_config_->corridorMinOverlapThreshold),
                           sfc_reject_reason))
  {
    ROS_WARN_STREAM("[highspeed_exp adapter] invalid safe flight corridor after simplify/shortcut: "
                    << sfc_reject_reason);
    return false;
  }
  h_polys = hPolysFromSfcs(sfcs);
  if (gcopter_viz_)
  {
    try
    {
      gcopter_viz_->visualizePolytope(h_polys);
      gcopter_viz_->visualizeRoute(path);
    }
    catch (const std::exception &e)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[highspeed_exp adapter] local planner visualization failed: "
                                        << e.what());
    }
  }

  double end_yaw = local_data_.end_yaw_;
  if (local_path.size() >= 2)
  {
    const Eigen::Vector3d dir = local_path.back() - local_path[local_path.size() - 2U];
    if (std::hypot(dir.x(), dir.y()) > 1.0e-3)
    {
      end_yaw = std::atan2(dir.y(), dir.x());
    }
  }

  const SegmentSafetyInfo segment_safety =
      evaluatePathSegmentSafety(local_path, local_data_.curr_yaw_, end_yaw);
  const SegmentVelocityLimit velocity_limit = computeSegmentVelocityLimit(segment_safety);
  const PieceVelocityProfile piece_velocity_profile = computePieceVelocityProfile(
      local_path, sfcs, head.col(1), yaw_init(0), *gcopter_config_, velocity_limit);
  if (piece_velocity_profile.bounds.size() != static_cast<int>(sfcs.size()))
  {
    ROS_WARN("[highspeed_exp adapter] cannot map path velocity profile to simplified corridor.");
    return false;
  }
  // Map-adaptive velocity may be disabled for compatibility, but geometric
  // turn limits are a flight-dynamics constraint and must never be bypassed.
  const double scheduled_speed =
      std::clamp(piece_velocity_profile.bounds.minCoeff(),
                 std::max(0.1, gcopter_config_->minSegmentVel),
                 std::max(0.2, gcopter_config_->maxVelMag));
  general_utils::StatePVAJ tail = general_utils::StatePVAJ::Zero();
  tail.col(0) = local_path.back();
  bool terminal_velocity_used = false;
  double terminal_speed = 0.0;
  if (gcopter_config_->nonstopTerminalVelocityEnable &&
      gcopter_config_->backupTrajEnable &&
      !is_static && hasCommittedTrajectory() &&
      local_path.size() >= 2 &&
      segment_safety.path_length >=
          std::max(0.0, gcopter_config_->nonstopTerminalMinPathLength) &&
      segment_safety.turn_angle <=
          gcopter_config_->nonstopTerminalMaxTurnAngle &&
      segment_safety.yaw_delta <=
          gcopter_config_->nonstopTerminalMaxYawDelta)
  {
    const Eigen::Vector3d dir = local_path.back() - local_path[local_path.size() - 2U];
    if (dir.norm() > 1.0e-3)
    {
      terminal_speed =
          std::clamp(gcopter_config_->openSegmentVel * gcopter_config_->nonstopTerminalVelocityRatio,
                     0.0,
                     std::min(std::max(0.2, gcopter_config_->maxVelMag),
                              piece_velocity_profile.bounds(
                                  piece_velocity_profile.bounds.size() - 1)));
      tail.col(1) = dir.normalized() * terminal_speed;
      terminal_velocity_used = terminal_speed > 1.0e-3;
    }
  }
  geometry_utils::Trajectory pos_traj;
  // Do not clamp retry attempts to MinSegmentVel.  That value is the nominal
  // cruise floor; using it here made all four retries identical on sharp
  // paths and caused an infinite acceleration_limit retry loop.
  const double min_opt_speed =
      std::clamp(gcopter_config_->trajectoryRetryMinVel, 0.3,
                 std::max(0.3, gcopter_config_->minSegmentVel));
  const double max_opt_speed = std::max(min_opt_speed, gcopter_config_->maxVelMag);
  const double head_boundary_speed = head.col(1).norm();
  const double tail_boundary_speed = tail.col(1).norm();
  const double boundary_speed =
      std::max(head_boundary_speed, tail_boundary_speed);
  std::vector<double> opt_speed_attempts;
  auto add_speed_attempt = [&](double speed) {
    speed = std::clamp(speed, min_opt_speed, max_opt_speed);
    for (const double existing : opt_speed_attempts)
    {
      if (std::abs(existing - speed) < 0.10)
      {
        return;
      }
    }
    opt_speed_attempts.emplace_back(speed);
  };
  add_speed_attempt(scheduled_speed);
  add_speed_attempt(0.75 * scheduled_speed);
  add_speed_attempt(0.55 * scheduled_speed);
  add_speed_attempt(min_opt_speed);

  bool ok = false;
  double accepted_opt_speed = scheduled_speed;
  double accepted_velocity_bound = piece_velocity_profile.bounds.maxCoeff();
  double accepted_max_speed = 0.0;
  double accepted_max_acc = 0.0;
  int opt_attempt = 0;
  const double check_dt = std::max(0.03, gcopter_config_->commitSampleDt);
  auto trajectoryPassesSafetyCheck =
      [&](const geometry_utils::Trajectory &candidate,
          const int attempt,
          const double guide_speed) -> bool {
    double last_t = 0.0;
    Eigen::Vector3d last_p = candidate.getPos(0.0);
    for (double t = check_dt;
         t <= candidate.getTotalDuration() + 1.0e-6; t += check_dt)
    {
      const double tt = std::min(t, candidate.getTotalDuration());
      const Eigen::Vector3d p = candidate.getPos(tt);
      const double step = std::max(
          0.05, (tt - last_t) * std::max(1.0, candidate.getVel(tt).norm()));
      const RaycastSafetyInfo safety = raycastSafety(
          last_p,
          p,
          !gcopter_config_->safetyMapUnknownAllowedForExplore,
          std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance),
          step);
      if (safety.blocked_by_occupied ||
          (!gcopter_config_->safetyMapUnknownAllowedForExplore &&
           safety.blocked_by_unknown))
      {
        ROS_WARN_STREAM(
            "[highspeed_exp adapter] optimized trajectory safety check "
            "failed; retry slower: attempt="
            << attempt << " guide_speed=" << guide_speed << " t=" << tt
            << " state=" << safetyStateName(safety.first_blocked_state));
        return false;
      }
      last_t = tt;
      last_p = p;
      if (tt >= candidate.getTotalDuration())
      {
        break;
      }
    }
    return true;
  };
  for (const double opt_speed : opt_speed_attempts)
  {
    ++opt_attempt;
    geometry_utils::PolytopeVec attempt_sfcs = sfcs;
    const double attempt_scale =
        scheduled_speed > 1.0e-3 ? opt_speed / scheduled_speed : 1.0;
    general_utils::VecDf piece_velocity_bounds =
        piece_velocity_profile.bounds * attempt_scale;
    for (int i = 0; i < piece_velocity_bounds.size(); ++i)
    {
      piece_velocity_bounds(i) =
          std::clamp(piece_velocity_bounds(i), min_opt_speed, max_opt_speed);
    }
    piece_velocity_bounds(0) = std::min(
        1.05 * max_opt_speed,
        std::max(piece_velocity_bounds(0), head_boundary_speed + 0.30));
    piece_velocity_bounds(piece_velocity_bounds.size() - 1) = std::min(
        1.05 * max_opt_speed,
        std::max(piece_velocity_bounds(piece_velocity_bounds.size() - 1),
                 tail_boundary_speed + 0.30));
    const double velocity_bound = piece_velocity_bounds.maxCoeff();
    const std::vector<double> guide_t =
        allocateGuideTimes(local_path, piece_velocity_profile,
                           piece_velocity_bounds);
    pos_traj.clear();
    ok = exploration_traj_opt_->optimize(head,
                                         tail,
                                         guide_path,
                                         guide_t,
                                         attempt_sfcs,
                                         piece_velocity_bounds,
                                         pos_traj);
    if (ok && !pos_traj.empty())
    {
      const double candidate_max_speed = pos_traj.getMaxVelRate();
      const double candidate_max_acc = pos_traj.getMaxAccRate();
      const double velocity_commit_limit =
          1.03 * std::max(0.2, gcopter_config_->maxVelMag);
      const double acceleration_commit_limit =
          1.05 * std::max(0.2, gcopter_config_->maxAccMag);
      if (!std::isfinite(candidate_max_speed) ||
          !std::isfinite(candidate_max_acc) ||
          candidate_max_speed > velocity_commit_limit + 1.0e-6 ||
          candidate_max_acc > acceleration_commit_limit + 1.0e-6)
      {
        ROS_WARN_STREAM(
            "[highspeed_exp adapter] optimized candidate violates commit "
            "dynamics; retry slower: attempt="
            << opt_attempt << " guide_speed=" << opt_speed
            << " vel_bound=" << velocity_bound
            << " max_v=" << candidate_max_speed
            << " max_a=" << candidate_max_acc);
        ok = false;
        pos_traj.clear();
        continue;
      }
      if (!trajectoryPassesSafetyCheck(pos_traj, opt_attempt, opt_speed))
      {
        ok = false;
        pos_traj.clear();
        continue;
      }
      accepted_opt_speed = opt_speed;
      accepted_velocity_bound = piece_velocity_bounds.maxCoeff();
      accepted_max_speed = candidate_max_speed;
      accepted_max_acc = candidate_max_acc;
      sfcs.swap(attempt_sfcs);
      break;
    }
    ROS_WARN_STREAM("[highspeed_exp adapter] MINCO optimization attempt "
                    << opt_attempt << " failed:"
                    << " guide_speed=" << opt_speed
                    << " vel_bound=" << velocity_bound
                    << " boundary_speed=" << boundary_speed);
  }
  if (!ok || pos_traj.empty())
  {
    ROS_WARN("[highspeed_exp adapter] General exploration MINCO optimization failed.");
    return false;
  }
  general_utils::Vec4f yaw_goal = general_utils::Vec4f::Zero();
  yaw_goal(0) = end_yaw;

  geometry_utils::Trajectory yaw_traj;
  if (!yaw_traj_opt_->optimize(yaw_init, yaw_goal, pos_traj, yaw_traj, 3, false, false) ||
      yaw_traj.empty())
  {
    yaw_traj = makeHoldYawTrajectory(yaw_init(0), pos_traj.getTotalDuration());
    if (yaw_traj.empty())
    {
      ROS_WARN("[highspeed_exp adapter] yaw trajectory generation failed.");
      return false;
    }
  }

  geometry_utils::Trajectory committed_pos = pos_traj;
  geometry_utils::Trajectory committed_yaw = yaw_traj;
  ros::Time commit_start_time = ros::Time::now();
  bool stitched_prefix = false;
  double stitched_prefix_duration = 0.0;

  if (use_committed_replan_state)
  {
    const double replan_total_t = (ros::Time::now() - plan_process_start).toSec();
    if (replan_total_t > switch_delay)
    {
      ROS_WARN_STREAM("[highspeed_exp adapter] General commit rejected overtime replan:"
                      << " replan_total_t=" << replan_total_t
                      << " replan_forward_dt=" << switch_delay);
      return false;
    }

    const double prefix_start_tt = std::clamp(
        (commit_start_time.toSec() - guide_pos_traj.start_WT),
        0.0,
        committed_duration);
    if (replan_state_tt <= prefix_start_tt + 1.0e-4)
    {
      ROS_WARN_STREAM("[highspeed_exp adapter] missed replan switch point; keep committed trajectory: "
                      << "prefix_start=" << prefix_start_tt
                      << " switch=" << replan_state_tt);
      return false;
    }

    geometry_utils::Trajectory pos_prefix;
    if (!guide_pos_traj.getPartialTrajectoryByTime(prefix_start_tt,
                                                   replan_state_tt,
                                                   pos_prefix) ||
        pos_prefix.empty())
    {
      ROS_WARN("[highspeed_exp adapter] failed to cut committed position prefix.");
      return false;
    }

    geometry_utils::Trajectory yaw_prefix;
    const double yaw_duration = guide_yaw_traj.empty() ? 0.0 : guide_yaw_traj.getTotalDuration();
    const bool have_yaw_prefix =
        !guide_yaw_traj.empty() && std::isfinite(yaw_duration) &&
        prefix_start_tt + 1.0e-6 < replan_state_tt &&
        prefix_start_tt < yaw_duration &&
        guide_yaw_traj.getPartialTrajectoryByTime(std::min(prefix_start_tt, yaw_duration),
                                                  std::min(replan_state_tt, yaw_duration),
                                                  yaw_prefix) &&
        !yaw_prefix.empty();
    if (!have_yaw_prefix)
    {
      double held_yaw = yaw_init(0);
      if (!guide_yaw_traj.empty() && yaw_duration > 1.0e-4)
      {
        held_yaw = guide_yaw_traj.getPos(
            std::min(prefix_start_tt, yaw_duration)).x();
      }
      yaw_prefix = makeHoldYawTrajectory(held_yaw, pos_prefix.getTotalDuration());
      if (yaw_prefix.empty())
      {
        ROS_WARN("[highspeed_exp adapter] failed to construct committed yaw prefix.");
        return false;
      }
    }

    if (yaw_prefix.getTotalDuration() + 1.0e-4 < pos_prefix.getTotalDuration())
    {
      const double held_yaw = yaw_prefix.getPos(yaw_prefix.getTotalDuration()).x();
      yaw_prefix = yaw_prefix + makeHoldYawTrajectory(
                                    held_yaw,
                                    pos_prefix.getTotalDuration() - yaw_prefix.getTotalDuration());
    }

    pos_prefix.start_WT = commit_start_time.toSec();
    yaw_prefix.start_WT = commit_start_time.toSec();
    committed_pos = pos_prefix + pos_traj;
    committed_yaw = yaw_prefix + yaw_traj;
    stitched_prefix = true;
    stitched_prefix_duration = pos_prefix.getTotalDuration();
  }

  committed_pos.start_WT = commit_start_time.toSec();
  committed_yaw.start_WT = commit_start_time.toSec();

  std::string commit_reject_reason;
  if (!validateTrajectoryForCommit(committed_pos,
                                   committed_yaw,
                                   gcopter_config_->maxVelMag,
                                   gcopter_config_->maxAccMag,
                                   gcopter_config_->yaw_max_vel,
                                   std::max(0.02, gcopter_config_->commitSampleDt),
                                   commit_reject_reason))
  {
    ROS_WARN_STREAM("[highspeed_exp adapter] reject invalid committed trajectory before CmdTraj submit: "
                    << commit_reject_reason);
    return false;
  }

  auto estimateKnownFreeEndTime = [&](const geometry_utils::Trajectory &traj,
                                      double &known_free_length) -> double {
    known_free_length = 0.0;
    if (traj.empty())
    {
      return 0.0;
    }
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 1.0e-6)
    {
      return 0.0;
    }
    const double sample_dt = std::max(0.03, gcopter_config_->backupSampleDt);
    const double safe_distance =
        std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance);
    double last_t = 0.0;
    Eigen::Vector3d last_p = traj.getPos(0.0);
    double known_end_t = 0.0;
    for (double t = sample_dt; t <= duration + 1.0e-6; t += sample_dt)
    {
      const double tt = std::min(t, duration);
      const Eigen::Vector3d p = traj.getPos(tt);
      const double step = std::max(0.05, (tt - last_t) *
                                             std::max(1.0, traj.getVel(tt).norm()));
      const RaycastSafetyInfo safety =
          raycastSafety(last_p,
                        p,
                        gcopter_config_->safetyMapUnknownAsOccupiedForBackup,
                        safe_distance,
                        step);
      known_free_length += safety.known_free_length;
      if (safety.blocked_by_occupied || safety.blocked_by_unknown)
      {
        const double ratio = safety.length > 1.0e-6
                                 ? std::clamp(safety.known_free_length / safety.length,
                                              0.0,
                                              1.0)
                                 : 0.0;
        return std::clamp(last_t + ratio * (tt - last_t), 0.0, duration);
      }
      known_end_t = tt;
      last_t = tt;
      last_p = p;
      if (tt >= duration)
      {
        break;
      }
    }
    return known_end_t;
  };

  auto makeBackupSfc = [&](const Eigen::Vector3d &start,
                           const Eigen::Vector3d &end,
                           geometry_utils::Polytope &backup_sfc) -> bool {
    backup_sfc.Reset();
    const double safe_distance =
        std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance);
    const RaycastSafetyInfo line_safety = raycastSafety(
        start,
        end,
        gcopter_config_->safetyMapUnknownAsOccupiedForBackup,
        safe_distance,
        std::max(0.05, gcopter_config_->safetyMapQueryStep));
    if (line_safety.blocked_by_occupied || line_safety.blocked_by_unknown)
    {
      return false;
    }

    // A backup optimizer accepts one convex polytope. Generate a dedicated,
    // tight General corridor around the known-free braking line first.
    if (corridor_generator_ && rog_map_updated_)
    {
      try
      {
        general_utils::Line line{start, end};
        geometry_utils::Polytope line_sfc;
        if (corridor_generator_->GeneratePolytopeFromLine(line, line_sfc) &&
            !line_sfc.empty() &&
            pointInsideHPoly(line_sfc.GetPlanes(), start) &&
            pointInsideHPoly(line_sfc.GetPlanes(), end))
        {
          backup_sfc = line_sfc;
          return true;
        }
      }
      catch (const std::exception &e)
      {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[highspeed_exp adapter] backup line corridor failed: "
                     << e.what());
      }
    }

    // An existing exploration SFC is a valid fallback only when it contains
    // both ends. Returning a start-only SFC made the seed infeasible and was
    // the deterministic MINCO failure seen in the supplied log.
    for (const auto &sfc : sfcs)
    {
      const Eigen::MatrixX4d planes = sfc.GetPlanes();
      if (pointInsideHPoly(planes, start) && pointInsideHPoly(planes, end))
      {
        backup_sfc = sfc;
        return true;
      }
    }
    return false;
  };

  auto backupTrajectoryKnownFree = [&](const geometry_utils::Trajectory &traj) -> bool {
    if (traj.empty())
    {
      return false;
    }
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 1.0e-6)
    {
      return false;
    }
    const double sample_dt = std::max(0.03, gcopter_config_->backupSampleDt);
    const double safe_distance =
        std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance);
    double last_t = 0.0;
    Eigen::Vector3d last_p = traj.getPos(0.0);
    for (double t = sample_dt; t <= duration + 1.0e-6; t += sample_dt)
    {
      const double tt = std::min(t, duration);
      const Eigen::Vector3d p = traj.getPos(tt);
      const double step = std::max(0.05, (tt - last_t) *
                                             std::max(1.0, traj.getVel(tt).norm()));
      const RaycastSafetyInfo safety =
          raycastSafety(last_p,
                        p,
                        gcopter_config_->safetyMapUnknownAsOccupiedForBackup,
                        safe_distance,
                        step);
      if (safety.blocked_by_occupied || safety.blocked_by_unknown)
      {
        return false;
      }
      last_t = tt;
      last_p = p;
      if (tt >= duration)
      {
        break;
      }
    }
    return true;
  };

  auto backupTrajectoryIsMonotonic = [&](const geometry_utils::Trajectory &traj,
                                         const Eigen::Vector3d &seed_point) -> bool {
    if (traj.empty())
    {
      return false;
    }
    const double duration = traj.getTotalDuration();
    const Eigen::Vector3d start = traj.getPos(0.0);
    Eigen::Vector3d forward = seed_point - start;
    if (!std::isfinite(duration) || duration <= 1.0e-4 || forward.norm() < 0.20)
    {
      return false;
    }
    const double forward_length = forward.norm();
    forward /= forward_length;
    const double max_lateral =
        std::max(1.0, 0.5 * gcopter_config_->backupSearchMargin);
    const double sample_dt = std::max(0.02, gcopter_config_->backupSampleDt);
    double last_progress = -0.05;
    bool has_moved = false;
    bool nearly_stopped = false;
    for (double t = 0.0; t <= duration + 1.0e-6; t += sample_dt)
    {
      const double tt = std::min(t, duration);
      const Eigen::Vector3d offset = traj.getPos(tt) - start;
      const double progress = offset.dot(forward);
      const double lateral = (offset - progress * forward).norm();
      const Eigen::Vector3d vel = traj.getVel(tt);
      if (progress + 0.10 < last_progress ||
          vel.dot(forward) < -0.20 || lateral > max_lateral)
      {
        return false;
      }
      const double speed = vel.norm();
      if (nearly_stopped && speed > 1.0)
      {
        return false;
      }
      has_moved = has_moved || speed > 1.0;
      nearly_stopped = nearly_stopped || (has_moved && speed < 0.35);
      last_progress = std::max(last_progress, progress);
      if (tt >= duration)
      {
        break;
      }
    }
    return (traj.getPos(duration) - seed_point).norm() <= 0.50 &&
           traj.getVel(duration).norm() <= 0.30;
  };

  auto buildBackupTrajectory = [&](general_planner::BackupTraj &backup_info,
                                   geometry_utils::Trajectory &backup_pos,
                                   geometry_utils::Trajectory &backup_yaw,
                                   double &backup_known_len) -> bool {
    backup_info.setEmpty();
    backup_pos.clear();
    backup_yaw.clear();
    backup_known_len = 0.0;
    if (!gcopter_config_->backupTrajEnable || !backup_traj_opt_)
    {
      return false;
    }
    const double exp_duration = committed_pos.getTotalDuration();
    if (!std::isfinite(exp_duration) ||
        exp_duration <= std::max(0.20, gcopter_config_->backupMinStartTime + 0.10))
    {
      return false;
    }

    const double known_end_t = estimateKnownFreeEndTime(committed_pos, backup_known_len);
    const double search_end_t =
        std::min({std::max(0.10, gcopter_config_->backupMaxStartTime),
                  exp_duration - 0.05,
                  known_end_t -
                      std::max(0.03, gcopter_config_->commitBackupTimeBuffer)});
    const double min_start_t =
        std::max(0.10, gcopter_config_->backupMinStartTime);
    if (!std::isfinite(search_end_t) ||
        search_end_t <= min_start_t + 0.15)
    {
      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "[highspeed_exp adapter] skip backup: known-free horizon too short."
              << " known_end_t=" << known_end_t
              << " known_len=" << backup_known_len
              << " exp_duration=" << exp_duration);
      return false;
    }
    const double sample_dt = std::max(0.03, gcopter_config_->backupSampleDt);

    std::vector<double> start_candidates;
    auto addStartCandidate = [&](double t) {
      t = std::clamp(t, min_start_t, search_end_t - 0.15);
      for (const double existing : start_candidates)
      {
        if (std::abs(existing - t) < 0.05)
        {
          return;
        }
      }
      start_candidates.emplace_back(t);
    };
    addStartCandidate(min_start_t);
    addStartCandidate(min_start_t + 0.15);
    addStartCandidate(exp_duration * gcopter_config_->backupStartRatio);

    int attempt_count = 0;
    for (const double start_t : start_candidates)
    {
      const Eigen::Vector3d start_point = committed_pos.getPos(start_t);
      const double start_speed = committed_pos.getVel(start_t).norm();
      const double conservative_acc = std::max(
          1.0,
          0.70 * std::min(gcopter_config_->backupMaxAcc,
                          gcopter_config_->maxAccMag));
      const double brake_dist =
          start_speed * start_speed / (2.0 * conservative_acc) +
          std::max(0.1, gcopter_config_->dilateRadiusHard) + 0.30;

      auto findSeedTime = [&](double requested_distance) {
        double walked_distance = 0.0;
        double seed_t = start_t;
        Eigen::Vector3d last_point = start_point;
        for (double t = start_t + sample_dt;
             t <= search_end_t + 1.0e-6;
             t += sample_dt)
        {
          const double tt = std::min(t, search_end_t);
          const Eigen::Vector3d point = committed_pos.getPos(tt);
          walked_distance += (point - last_point).norm();
          last_point = point;
          seed_t = tt;
          if (walked_distance >= requested_distance || tt >= search_end_t)
          {
            break;
          }
        }
        return seed_t;
      };

      std::vector<double> seed_candidates;
      auto addSeedCandidate = [&](double requested_distance) {
        const double t = findSeedTime(std::max(0.60, requested_distance));
        if (t <= start_t + 0.10)
        {
          return;
        }
        for (const double existing : seed_candidates)
        {
          if (std::abs(existing - t) < 0.05)
          {
            return;
          }
        }
        seed_candidates.emplace_back(t);
      };
      addSeedCandidate(brake_dist);
      addSeedCandidate(0.80 * brake_dist);
      addSeedCandidate(1.20 * brake_dist);

      for (const double seed_t : seed_candidates)
      {
        ++attempt_count;
        const Eigen::Vector3d seed_point = committed_pos.getPos(seed_t);
        geometry_utils::Polytope backup_sfc;
        if (!makeBackupSfc(start_point, seed_point, backup_sfc))
        {
          continue;
        }

        bool reference_window_inside = true;
        for (double t = start_t; t <= seed_t + 1.0e-6; t += sample_dt)
        {
          if (!pointInsideHPoly(
                  backup_sfc.GetPlanes(),
                  committed_pos.getPos(std::min(t, seed_t))))
          {
            reference_window_inside = false;
            break;
          }
        }
        if (!reference_window_inside)
        {
          continue;
        }

        const double heu_ts = std::clamp(
            start_t + std::min(0.10, 0.20 * (seed_t - start_t)),
            start_t + 1.0e-4,
            seed_t - 1.0e-4);
        const double heuristic_speed = committed_pos.getVel(heu_ts).norm();
        const double direct_distance =
            (seed_point - committed_pos.getPos(heu_ts)).norm();
        const double brake_time =
            heuristic_speed / std::max(1.0, gcopter_config_->backupMaxAcc);
        const double average_speed =
            std::max(0.75, 0.5 * std::max(heuristic_speed,
                                         gcopter_config_->minSegmentVel));
        double heu_dur = std::max(
            {0.35,
             1.15 * brake_time,
             1.15 * direct_distance / average_speed});
        general_utils::VecDf heu_p(3);
        heu_p = seed_point;
        double opt_ts = heu_ts;
        geometry_utils::Trajectory candidate_backup_pos;
        const bool minco_ok =
            backup_traj_opt_->optimize(committed_pos,
                                       start_t,
                                       seed_t,
                                       heu_ts,
                                       heu_p,
                                       heu_dur,
                                       backup_sfc,
                                       candidate_backup_pos,
                                       opt_ts) &&
            !candidate_backup_pos.empty();
        bool motion_ok = minco_ok &&
            backupTrajectoryKnownFree(candidate_backup_pos) &&
            backupTrajectoryIsMonotonic(candidate_backup_pos, seed_point);
        if (!motion_ok)
        {
          ROS_WARN_STREAM_THROTTLE(
              0.5,
              "[highspeed_exp adapter] backup MINCO unusable; try analytic brake:"
                  << " attempt=" << attempt_count
                  << " start_t=" << start_t
                  << " seed_t=" << seed_t
                  << " heu_ts=" << heu_ts
                  << " heu_dur=" << heu_dur);

          const general_utils::StatePVAJ analytic_head =
              committed_pos.getState(heu_ts);
          Eigen::Matrix<double, 3, 4> analytic_tail =
              Eigen::Matrix<double, 3, 4>::Zero();
          analytic_tail.col(0) = seed_point;
          Eigen::Matrix<double, 3, Eigen::Dynamic> no_waypoints(3, 0);
          for (const double duration_scale : {1.0, 1.35, 1.75, 2.20})
          {
            general_utils::VecDf analytic_times(1);
            analytic_times(0) = duration_scale * heu_dur;
            geometry_utils::Trajectory analytic =
                geometry_utils::poly_interpo::minimumSnapInterpolation<3>(
                    analytic_head, analytic_tail, no_waypoints, analytic_times);
            bool inside = !analytic.empty();
            if (inside)
            {
              for (double t = 0.0;
                   t <= analytic.getTotalDuration() + 1.0e-6;
                   t += sample_dt)
              {
                if (!pointInsideHPoly(
                        backup_sfc.GetPlanes(),
                        analytic.getPos(std::min(t, analytic.getTotalDuration()))))
                {
                  inside = false;
                  break;
                }
              }
            }
            if (inside && backupTrajectoryKnownFree(analytic) &&
                backupTrajectoryIsMonotonic(analytic, seed_point))
            {
              candidate_backup_pos = analytic;
              opt_ts = heu_ts;
              motion_ok = true;
              ROS_WARN_STREAM_THROTTLE(
                  0.5,
                  "[highspeed_exp adapter] use analytic braking fallback:"
                      << " attempt=" << attempt_count
                      << " duration=" << analytic.getTotalDuration());
              break;
            }
          }
          if (!motion_ok)
          {
            continue;
          }
        }

        const double yaw_t = std::clamp(
            opt_ts, 0.0, std::max(0.0, committed_yaw.getTotalDuration()));
        general_utils::Vec4f yaw_init_vec = committed_yaw.getState(yaw_t).row(0);
        general_utils::Vec4f yaw_goal = general_utils::Vec4f::Zero();
        geometry_utils::Trajectory candidate_backup_yaw;
        if (!yaw_traj_opt_->optimize(yaw_init_vec,
                                     yaw_goal,
                                     candidate_backup_pos,
                                     candidate_backup_yaw,
                                     3,
                                     false,
                                     true) ||
            candidate_backup_yaw.empty())
        {
          candidate_backup_yaw = makeHoldYawTrajectory(
              yaw_init_vec(0), candidate_backup_pos.getTotalDuration());
        }

        const double backup_start_wt = commit_start_time.toSec() + opt_ts;
        candidate_backup_pos.start_WT = backup_start_wt;
        candidate_backup_yaw.start_WT = backup_start_wt;
        std::string backup_reject_reason;
        bool valid = validateTrajectoryForCommit(
            candidate_backup_pos,
            candidate_backup_yaw,
            gcopter_config_->backupMaxVel,
            gcopter_config_->backupMaxAcc,
            gcopter_config_->yaw_max_vel,
            std::max(0.02, gcopter_config_->commitSampleDt),
            backup_reject_reason);
        if (!valid)
        {
          // A successful yaw optimizer can still violate yaw-rate limits. A
          // braking backup does not need to scan; hold the switching yaw and
          // validate again before rejecting the position trajectory.
          candidate_backup_yaw = makeHoldYawTrajectory(
              yaw_init_vec(0), candidate_backup_pos.getTotalDuration());
          candidate_backup_yaw.start_WT = backup_start_wt;
          valid = validateTrajectoryForCommit(
              candidate_backup_pos,
              candidate_backup_yaw,
              gcopter_config_->backupMaxVel,
              gcopter_config_->backupMaxAcc,
              gcopter_config_->yaw_max_vel,
              std::max(0.02, gcopter_config_->commitSampleDt),
              backup_reject_reason);
        }
        if (!valid)
        {
          ROS_WARN_STREAM_THROTTLE(
              0.5,
              "[highspeed_exp adapter] reject invalid backup attempt: "
                  << backup_reject_reason);
          continue;
        }

        backup_pos = candidate_backup_pos;
        backup_yaw = candidate_backup_yaw;
        backup_info.setSFC(backup_sfc);
        backup_info.setRobotPos(local_data_.curr_pos_);
        backup_info.setTrajectory(
            backup_start_wt, opt_ts, backup_pos, backup_yaw);
        ROS_INFO_STREAM("[highspeed_exp adapter] backup generated after "
                        << attempt_count << " attempt(s): start_window="
                        << start_t << " seed_t=" << seed_t
                        << " opt_ts=" << opt_ts
                        << " duration=" << backup_pos.getTotalDuration());
        return true;
      }
    }
    ROS_WARN_STREAM_THROTTLE(
        0.5,
        "[highspeed_exp adapter] all backup candidates failed: attempts="
            << attempt_count << " known_end_t=" << known_end_t
            << " exp_duration=" << exp_duration);
    return false;
  };

  general_planner::ExpTraj exp_traj_info;
  exp_traj_info.setSFC(sfcs);
  exp_traj_info.setGoalConnectedFlag(true);
  // The exploration suffix can intentionally enter unknown space. The command
  // trajectory below is the only trajectory that is required to be known-free.
  exp_traj_info.setWholeTrajKnownFreeFlag(false);
  exp_traj_info.setTrajectory(commit_start_time.toSec(), committed_pos, committed_yaw);

  general_planner::BackupTraj backup_traj_info;
  geometry_utils::Trajectory backup_pos_traj;
  geometry_utils::Trajectory backup_yaw_traj;
  double backup_known_len = 0.0;
  bool known_free_terminal_stop = false;
  if (!terminal_velocity_used && !committed_pos.empty())
  {
    double primary_known_length = 0.0;
    const double primary_known_end =
        estimateKnownFreeEndTime(committed_pos, primary_known_length);
    const double primary_duration = committed_pos.getTotalDuration();
    known_free_terminal_stop =
        primary_known_end + std::max(0.03, gcopter_config_->backupSampleDt) >=
        primary_duration &&
        committed_pos.getVel(primary_duration).norm() <= 0.30 &&
        backupTrajectoryKnownFree(committed_pos);
    if (known_free_terminal_stop)
    {
      backup_known_len = primary_known_length;
      exp_traj_info.setWholeTrajKnownFreeFlag(true);
      ROS_INFO_STREAM_THROTTLE(
          0.5,
          "[highspeed_exp adapter] backup not required for verified "
          "known-free terminal-stop primary. duration="
              << primary_duration << " known_len=" << primary_known_length);
    }
  }
  // Do not spend 50--150 ms generating a redundant braking branch when the
  // whole command is already known-free and ends at rest. In shorter-sensing
  // or real-world unknown-space cases the backup path below remains mandatory.
  bool backup_available = false;
  if (!known_free_terminal_stop)
  {
    backup_available = buildBackupTrajectory(
        backup_traj_info, backup_pos_traj, backup_yaw_traj, backup_known_len);
  }
  if (gcopter_config_->backupTrajEnable &&
      committed_pos.getTotalDuration() > gcopter_config_->backupMinStartTime + 0.2 &&
      !backup_available && !known_free_terminal_stop)
  {
    ROS_WARN("[highspeed_exp adapter] backup is required for a highspeed committed trajectory.");
    return false;
  }

  general_planner::CmdTraj candidate_cmd;
  const bool candidate_ok = backup_available
                                ? candidate_cmd.setTrajectory(exp_traj_info, backup_traj_info)
                                : (candidate_cmd.setTrajectory(exp_traj_info), true);
  if (!candidate_ok)
  {
    ROS_WARN("[highspeed_exp adapter] failed to compose candidate command trajectory.");
    return false;
  }

  if (backup_available)
  {
    const geometry_utils::Trajectory &candidate_pos = candidate_cmd.posTraj();
    const double sample_dt = std::max(0.02, gcopter_config_->commitSampleDt);
    Eigen::Vector3d last_commit_p = candidate_pos.getPos(0.0);
    for (double t = sample_dt; t <= candidate_pos.getTotalDuration() + 1.0e-6; t += sample_dt)
    {
      const double tt = std::min(t, candidate_pos.getTotalDuration());
      const Eigen::Vector3d p = candidate_pos.getPos(tt);
      const double step = std::max(
          0.05,
          sample_dt * std::max(1.0, candidate_pos.getVel(tt).norm()));
      const RaycastSafetyInfo safety = raycastSafety(
          last_commit_p,
          p,
          true,
          std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance),
          step);
      bool segment_safe = !safety.blocked_by_occupied && !safety.blocked_by_unknown;
      if (!segment_safe)
      {
        ROS_WARN_STREAM("[highspeed_exp adapter] reject committed trajectory outside known-free map at t="
                        << tt << ", state=" << safetyStateName(safety.first_blocked_state));
        return false;
      }
      last_commit_p = p;
      if (tt >= candidate_pos.getTotalDuration())
      {
        break;
      }
    }
  }

  if (backup_available)
  {
    if (!commit_store_->cmd_traj_info.setTrajectory(exp_traj_info, backup_traj_info))
    {
      ROS_WARN("[highspeed_exp adapter] failed to commit backup command trajectory.");
      return false;
    }
  }
  else
  {
    commit_store_->cmd_traj_info.setTrajectory(exp_traj_info);
  }
  commit_store_->last_exp_traj_info = exp_traj_info;
  committed_stop_active_ = false;

  *committed_pos_traj_ = commit_store_->cmd_traj_info.posTraj();
  *committed_yaw_traj_ = commit_store_->cmd_traj_info.yawTraj();
  *latest_exp_pos_traj_ = exp_traj_info.posTraj();
  *latest_exp_yaw_traj_ = exp_traj_info.yawTraj();

  local_data_.start_time_ = commit_start_time;
  local_data_.duration_ = committed_pos_traj_->getTotalDuration();
  local_data_.traj_id_ += 1;
  local_data_.start_pos_ = committed_pos_traj_->getPos(0.0);
  const double committed_yaw_duration = committed_yaw_traj_->getTotalDuration();
  if (std::isfinite(committed_yaw_duration) && committed_yaw_duration > 1.0e-5)
  {
    local_data_.end_yaw_ =
        committed_yaw_traj_->getPos(std::min(local_data_.duration_, committed_yaw_duration)).x();
  }
  else
  {
    local_data_.end_yaw_ = end_yaw;
  }
  local_data_.backup_available_ = backup_available;
  local_data_.backup_start_t_ =
      backup_available ? backup_traj_info.getStartTT()
                       : std::numeric_limits<double>::infinity();
  local_data_.minco_traj_.setGeometryTrajectory(*committed_pos_traj_);
  local_data_.minco_yaw_traj_.setGeometryTrajectory(*committed_yaw_traj_);
  local_data_.exp_traj_.setGeometryTrajectory(*latest_exp_pos_traj_);
  local_data_.exp_yaw_traj_.setGeometryTrajectory(*latest_exp_yaw_traj_);
  if (backup_available)
  {
    local_data_.backup_traj_.setGeometryTrajectory(backup_pos_traj);
    local_data_.backup_yaw_traj_.setGeometryTrajectory(backup_yaw_traj);
  }
  else
  {
    local_data_.backup_traj_.clear();
    local_data_.backup_yaw_traj_.clear();
  }
  std::vector<Eigen::Vector3d> viz_path = local_path;
  if (!viz_path.empty() && (viz_path.front() - committed_pos_traj_->getPos(0.0)).norm() > 0.05)
  {
    viz_path.insert(viz_path.begin(), committed_pos_traj_->getPos(0.0));
  }
  publishHighspeedTrajectoryViz(gcopter_viz_.get(),
                                *committed_pos_traj_,
                                viz_path,
                                gcopter_config_->maxVelMag);

  ROS_INFO_STREAM("[highspeed_exp adapter] planned General MINCO exploration traj: pieces="
                  << committed_pos_traj_->getPieceNum()
                  << ", duration=" << committed_pos_traj_->getTotalDuration()
		                  << ", max_v=" << committed_pos_traj_->getMaxVelRate()
		                  << ", max_a=" << committed_pos_traj_->getMaxAccRate()
		                  << ", sfc(raw/simplified/final)=" << raw_sfc_count
                      << "/" << simplified_sfc_count << "/" << sfcs.size()
		                  << ", corridor=" << (used_general_corridor ? "general" : "box")
                      << ", prefix=" << (stitched_prefix ? "yes" : "no")
                      << ", prefix_dur=" << stitched_prefix_duration
                      << ", switch_delay=" << switch_delay
                      << ", head_path_dist=" << committed_head_path_dist
                      << ", shifted_start=" << shifted_start_dist
			                  << ", sched_v=" << scheduled_speed
                      << ", opt_v=" << accepted_opt_speed
                      << ", opt_bound=" << accepted_velocity_bound
                      << ", suffix_max_v=" << accepted_max_speed
                      << ", suffix_max_a=" << accepted_max_acc
                      << ", terminal_v=" << (terminal_velocity_used ? terminal_speed : 0.0)
                      << ", backup=" << (backup_available ? "yes" : "no")
                      << ", backup_start=" << local_data_.backup_start_t_
                      << ", backup_known_len=" << backup_known_len
			                  << ", sched_reason=" << velocity_limit.reason);
  return true;
}

bool FastPlannerManager::planControlledStopTrajectory()
{
  if (!gcopter_config_ || !commit_store_)
  {
    return false;
  }
  if (committed_stop_active_ && committedTrajectoryRemainingTime() > 0.05)
  {
    return true;
  }

  geometry_utils::Trajectory guide_pos;
  geometry_utils::Trajectory guide_yaw;
  double guide_start_wt = 0.0;
  commit_store_->cmd_traj_info.lock();
  const bool have_committed = !commit_store_->cmd_traj_info.empty();
  if (have_committed)
  {
    guide_pos = commit_store_->cmd_traj_info.posTraj();
    guide_yaw = commit_store_->cmd_traj_info.yawTraj();
    guide_start_wt = commit_store_->cmd_traj_info.getStartWallTime();
  }
  commit_store_->cmd_traj_info.unlock();

  const ros::Time commit_start_time = ros::Time::now();
  general_utils::StatePVAJ head = general_utils::StatePVAJ::Zero();
  head.col(0) = local_data_.curr_pos_;
  if (local_data_.curr_vel_.allFinite())
  {
    head.col(1) = local_data_.curr_vel_;
  }
  double stop_yaw = local_data_.curr_yaw_;
  geometry_utils::Trajectory pos_prefix;
  geometry_utils::Trajectory yaw_prefix;
  double prefix_duration = 0.0;

  if (have_committed && !guide_pos.empty() && std::isfinite(guide_start_wt))
  {
    const double guide_duration = guide_pos.getTotalDuration();
    const double now_tt = std::clamp(
        commit_start_time.toSec() - guide_start_wt, 0.0, guide_duration);
    const double remaining = guide_duration - now_tt;
    if (remaining > 0.08)
    {
      const double switch_delay = std::min(
          remaining - 0.02,
          std::clamp(gcopter_config_->controlLatency, 0.03, 0.12));
      const double switch_tt = now_tt + switch_delay;
      general_utils::StatePVAJ committed_head = general_utils::StatePVAJ::Zero();
      if (switch_delay > 1.0e-4 &&
          guide_pos.getState(switch_tt, committed_head) &&
          committed_head.allFinite() &&
          guide_pos.getPartialTrajectoryByTime(now_tt, switch_tt, pos_prefix) &&
          !pos_prefix.empty())
      {
        head = committed_head;
        prefix_duration = pos_prefix.getTotalDuration();
        const double yaw_duration = guide_yaw.getTotalDuration();
        if (!guide_yaw.empty() && std::isfinite(yaw_duration) &&
            now_tt < yaw_duration - 1.0e-4)
        {
          const double yaw_end_tt = std::min(switch_tt, yaw_duration);
          if (yaw_end_tt > now_tt + 1.0e-4)
          {
            guide_yaw.getPartialTrajectoryByTime(now_tt, yaw_end_tt, yaw_prefix);
            stop_yaw = guide_yaw.getPos(yaw_end_tt).x();
          }
        }
        if (yaw_prefix.empty() ||
            yaw_prefix.getTotalDuration() + 1.0e-4 < prefix_duration)
        {
          double initial_yaw = local_data_.curr_yaw_;
          if (!guide_yaw.empty() && yaw_duration > 1.0e-4)
          {
            initial_yaw = guide_yaw.getPos(std::min(now_tt, yaw_duration)).x();
          }
          yaw_prefix = makeHoldYawTrajectory(initial_yaw, prefix_duration);
          stop_yaw = initial_yaw;
        }
      }
    }
  }

  if (!head.allFinite() || !std::isfinite(stop_yaw))
  {
    return false;
  }

  const double speed = head.col(1).norm();
  const bool already_stopped =
      speed <= 0.05 && head.col(2).norm() <= 0.10 &&
      head.col(3).norm() <= 0.30;
  const double brake_acc = std::max(1.0, gcopter_config_->brakeAccel);
  const double base_duration = std::max(0.65, 2.0 * speed / brake_acc);
  const double sample_dt =
      std::max(0.02, gcopter_config_->commitSampleDt);
  geometry_utils::Trajectory accepted_pos;
  geometry_utils::Trajectory accepted_yaw;
  std::string last_reject_reason = "no_candidate";

  for (const double duration_scale : {1.0, 1.25, 1.60, 2.10, 2.80})
  {
    const double stop_duration =
        already_stopped ? 0.50 : duration_scale * base_duration;
    geometry_utils::Trajectory stop_suffix;
    if (already_stopped)
    {
      Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 8);
      coeff.col(7) = head.col(0);
      stop_suffix.emplace_back(stop_duration, coeff);
    }
    else
    {
      general_utils::StatePVAJ tail = general_utils::StatePVAJ::Zero();
      // A zero-terminal-velocity seventh-order segment has approximately half
      // the initial speed as its average speed.  This target is deliberately
      // conservative compared with v^2/(2a), leaving room for zero terminal
      // acceleration and jerk.
      tail.col(0) = head.col(0) + 0.5 * stop_duration * head.col(1) +
                    (stop_duration * stop_duration / 12.0) * head.col(2);
      Eigen::Matrix<double, 3, Eigen::Dynamic> no_waypoints(3, 0);
      general_utils::VecDf times(1);
      times(0) = stop_duration;
      stop_suffix =
          geometry_utils::poly_interpo::minimumSnapInterpolation<3>(
              head, tail, no_waypoints, times);
    }
    if (stop_suffix.empty())
    {
      last_reject_reason = "empty_stop_suffix";
      continue;
    }

    geometry_utils::Trajectory candidate_pos =
        pos_prefix.empty() ? stop_suffix : pos_prefix + stop_suffix;
    geometry_utils::Trajectory yaw_suffix =
        makeHoldYawTrajectory(stop_yaw, stop_suffix.getTotalDuration());
    geometry_utils::Trajectory candidate_yaw =
        yaw_prefix.empty() ? makeHoldYawTrajectory(
                                 stop_yaw, candidate_pos.getTotalDuration())
                           : yaw_prefix + yaw_suffix;
    candidate_pos.start_WT = commit_start_time.toSec();
    candidate_yaw.start_WT = commit_start_time.toSec();

    if (!validateTrajectoryForCommit(
            candidate_pos, candidate_yaw,
            std::max(gcopter_config_->maxVelMag, speed + 0.30),
            std::max(gcopter_config_->maxAccMag, brake_acc),
            gcopter_config_->yaw_max_vel, sample_dt,
            last_reject_reason))
    {
      continue;
    }

    bool known_free = true;
    Eigen::Vector3d previous = candidate_pos.getPos(0.0);
    for (double t = sample_dt;
         t <= candidate_pos.getTotalDuration() + 1.0e-6; t += sample_dt)
    {
      const double tt = std::min(t, candidate_pos.getTotalDuration());
      const Eigen::Vector3d point = candidate_pos.getPos(tt);
      const RaycastSafetyInfo ray = raycastSafety(
          previous, point, true,
          std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance),
          std::max(0.05,
                   sample_dt * std::max(1.0, candidate_pos.getVel(tt).norm())));
      if (ray.blocked_by_occupied || ray.blocked_by_unknown)
      {
        known_free = false;
        last_reject_reason = std::string("stop_not_known_free_") +
                             safetyStateName(ray.first_blocked_state);
        break;
      }
      previous = point;
    }
    const double terminal_t = candidate_pos.getTotalDuration();
    if (!known_free || candidate_pos.getVel(terminal_t).norm() > 0.10 ||
        candidate_pos.getAcc(terminal_t).norm() > 0.20)
    {
      if (known_free)
      {
        last_reject_reason = "nonzero_stop_terminal_state";
      }
      continue;
    }
    accepted_pos = candidate_pos;
    accepted_yaw = candidate_yaw;
    break;
  }

  if (accepted_pos.empty() || accepted_yaw.empty())
  {
    ROS_ERROR_STREAM_THROTTLE(
        0.5, "[controlled stop] no valid braking polynomial: speed="
                 << speed << " prefix=" << prefix_duration
                 << " reason=" << last_reject_reason);
    return false;
  }

  general_planner::ExpTraj stop_info;
  stop_info.setGoalConnectedFlag(false);
  stop_info.setWholeTrajKnownFreeFlag(true);
  stop_info.setTrajectory(commit_start_time.toSec(), accepted_pos, accepted_yaw);
  commit_store_->cmd_traj_info.setTrajectory(stop_info);
  commit_store_->last_exp_traj_info = stop_info;
  *committed_pos_traj_ = accepted_pos;
  *committed_yaw_traj_ = accepted_yaw;
  *latest_exp_pos_traj_ = accepted_pos;
  *latest_exp_yaw_traj_ = accepted_yaw;

  local_data_.start_time_ = commit_start_time;
  local_data_.duration_ = accepted_pos.getTotalDuration();
  local_data_.traj_id_ += 1;
  local_data_.start_pos_ = accepted_pos.getPos(0.0);
  local_data_.end_yaw_ = stop_yaw;
  local_data_.backup_available_ = false;
  local_data_.backup_start_t_ = std::numeric_limits<double>::infinity();
  local_data_.minco_traj_.setGeometryTrajectory(accepted_pos);
  local_data_.minco_yaw_traj_.setGeometryTrajectory(accepted_yaw);
  local_data_.exp_traj_.setGeometryTrajectory(accepted_pos);
  local_data_.exp_yaw_traj_.setGeometryTrajectory(accepted_yaw);
  local_data_.backup_traj_.clear();
  local_data_.backup_yaw_traj_.clear();
  committed_stop_active_ = true;

  ROS_WARN_STREAM("[controlled stop] committed braking trajectory: id="
                  << local_data_.traj_id_
                  << " speed=" << speed
                  << " prefix=" << prefix_duration
                  << " duration=" << accepted_pos.getTotalDuration()
                  << " distance="
                  << (accepted_pos.getPos(accepted_pos.getTotalDuration()) -
                      accepted_pos.getPos(0.0)).norm()
                  << " terminal_v="
                  << accepted_pos.getVel(accepted_pos.getTotalDuration()).norm()
                  << " terminal_a="
                  << accepted_pos.getAcc(accepted_pos.getTotalDuration()).norm());
  return true;
}

bool FastPlannerManager::flyToSafeRegion(bool is_static)
{
  if (!gcopter_config_ || !lidar_map_interface_ || !corridor_generator_ ||
      !exploration_traj_opt_ || !rog_map_updated_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[highspeed_exp adapter] safe-region recovery backend is not ready.");
    return false;
  }

  const Eigen::Vector3d start = local_data_.curr_pos_;
  if (!start.allFinite())
  {
    return false;
  }

  const double hard_clearance =
      std::max(0.05, gcopter_config_->dilateRadiusHard);
  const double target_clearance =
      std::max(gcopter_config_->dilateRadiusSoft,
               gcopter_config_->commitKnownFreeSafeDistance);
  const double start_clearance = safetyDistanceToOcc(start);
  if (std::isfinite(start_clearance) && start_clearance > target_clearance)
  {
    // CAUTION will leave the recovery state on this same condition.  Do not
    // replace a valid committed trajectory when no escape motion is needed.
    return false;
  }

  struct SafeCandidate
  {
    Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
    double score{std::numeric_limits<double>::infinity()};
  };
  std::vector<SafeCandidate> candidates;
  candidates.reserve(64);

  // The original implementation builds one local FIRI polytope in a 4x4x2 m
  // box and flies toward its interior.  General Planner owns corridor
  // generation here, so sample the same-size neighbourhood, retain points with
  // original highspeedExp known-free/LIO-clearance semantics, and let the
  // General corridor generator and MINCO optimizer validate the escape path.
  std::vector<Eigen::Vector3d> directions;
  directions.reserve(26);
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dz = -1; dz <= 1; ++dz)
      {
        if (dx == 0 && dy == 0 && dz == 0)
        {
          continue;
        }
        // Match the original recovery box's smaller vertical extent.
        Eigen::Vector3d dir(dx, dy, 0.5 * dz);
        directions.emplace_back(dir.normalized());
      }
    }
  }

  const double radial_step =
      map_manager_ ? std::max(0.20, map_manager_->getResolution()) : 0.20;
  constexpr double kMaxRecoveryRadius = 2.0;
  for (const Eigen::Vector3d &dir : directions)
  {
    double previous_clearance = start_clearance;
    for (double radius = radial_step;
         radius <= kMaxRecoveryRadius + 1.0e-6;
         radius += radial_step)
    {
      const Eigen::Vector3d pos = start + radius * dir;
      const MapVoxelState state = querySafetyState(pos);
      const double clearance = safetyDistanceToOcc(pos);
      if (state == MapVoxelState::OCCUPIED ||
          state == MapVoxelState::OUT_OF_MAP || !std::isfinite(clearance))
      {
        break;
      }

      // Do not propose an escape direction that first moves materially closer
      // to an obstacle.  A slightly non-monotone profile is allowed because KD
      // nearest-neighbour clearance is discretized.
      if (std::isfinite(previous_clearance) &&
          clearance + 0.05 < std::min(previous_clearance, hard_clearance))
      {
        break;
      }
      previous_clearance = clearance;

      if (state == MapVoxelState::KNOWN_FREE && clearance >= target_clearance)
      {
        SafeCandidate candidate;
        candidate.pos = pos;
        candidate.score = radius - 0.20 * clearance;
        candidates.emplace_back(candidate);
        break;
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const SafeCandidate &lhs, const SafeCandidate &rhs) {
              return lhs.score < rhs.score;
            });
  if (candidates.empty())
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[highspeed_exp adapter] no known-free recovery point within 2 m.");
    return false;
  }

  const std::size_t max_attempts = std::min<std::size_t>(6, candidates.size());
  for (std::size_t i = 0; i < max_attempts; ++i)
  {
    std::vector<Eigen::Vector3f> recovery_path;
    recovery_path.reserve(2);
    recovery_path.emplace_back(start.cast<float>());
    recovery_path.emplace_back(candidates[i].pos.cast<float>());
    if (planExploreTraj(recovery_path, is_static))
    {
      ROS_WARN_STREAM("[highspeed_exp adapter] safe-region recovery planned: start=("
                      << start.transpose() << ") goal=("
                      << candidates[i].pos.transpose() << ") start_clearance="
                      << start_clearance << " target_clearance="
                      << safetyDistanceToOcc(candidates[i].pos));
      return true;
    }
  }

  ROS_WARN_THROTTLE(
      1.0,
      "[highspeed_exp adapter] General corridor/MINCO rejected all recovery paths.");
  return false;
}

void FastPlannerManager::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time)
{
  (void)start_time;
  if (commit_store_->cmd_traj_info.empty())
  {
    return;
  }
  commit_store_->cmd_traj_info.lock();
  const geometry_utils::Trajectory pos_traj = commit_store_->cmd_traj_info.posTraj();
  const ros::Time cmd_start_time(commit_store_->cmd_traj_info.getStartWallTime());
  commit_store_->cmd_traj_info.unlock();
  fillPolyTrajMsg(pos_traj, 7, 8, poly_msg, cmd_start_time, local_data_.traj_id_);
}

void FastPlannerManager::polyYawTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time)
{
  (void)start_time;
  if (commit_store_->cmd_traj_info.empty())
  {
    return;
  }
  commit_store_->cmd_traj_info.lock();
  const geometry_utils::Trajectory yaw_traj = commit_store_->cmd_traj_info.yawTraj();
  const ros::Time cmd_start_time(commit_store_->cmd_traj_info.getStartWallTime());
  commit_store_->cmd_traj_info.unlock();
  fillPolyTrajMsg(yaw_traj, 5, 6, poly_msg, cmd_start_time, local_data_.traj_id_);
}

bool FastPlannerManager::checkTrajCollision(double &collision_time)
{
  collision_time = committedTrajectoryRemainingTime();
  if (!lidar_map_interface_ || !gcopter_config_ ||
      commit_store_->cmd_traj_info.empty())
  {
    return true;
  }
  commit_store_->cmd_traj_info.lock();
  const geometry_utils::Trajectory committed_pos = commit_store_->cmd_traj_info.posTraj();
  const double start_wt = commit_store_->cmd_traj_info.getStartWallTime();
  commit_store_->cmd_traj_info.unlock();
  if (committed_pos.empty() || !std::isfinite(start_wt))
  {
    return true;
  }
  const double total_duration = committed_pos.getTotalDuration();
  const double horizon =
      local_data_.duration_ > 1.0e-4 ? std::min(local_data_.duration_, total_duration)
                                     : total_duration;
  const double now_t = std::clamp(ros::Time::now().toSec() - start_wt,
                                  0.0,
                                  horizon);
  if (now_t >= horizon)
  {
    collision_time = 0.0;
    return true;
  }

  // Match the original highspeedExp collision checker: one expensive nearest-
  // obstacle query defines a free sphere, then trajectory samples inside that
  // sphere are accepted without another map query.  The old adapter raycasted
  // every 30--50 ms segment over the complete remaining horizon and this
  // function is called both by the 100 Hz FSM and every cloud callback.
  constexpr double kTrajectorySampleDt = 0.05;
  double probe_t = now_t;
  Eigen::Vector3d sphere_center = committed_pos.getPos(probe_t);
  double sphere_radius = safetyDistanceToOcc(sphere_center) -
                         gcopter_config_->dilateRadiusHard;
  if (!std::isfinite(sphere_radius) || sphere_radius <= 0.0)
  {
    collision_time = 0.0;
    return false;
  }

  while (probe_t < horizon)
  {
    const Eigen::Vector3d pos = committed_pos.getPos(probe_t);
    if ((pos - sphere_center).norm() < sphere_radius)
    {
      probe_t += kTrajectorySampleDt;
      continue;
    }

    sphere_center = pos;
    sphere_radius = safetyDistanceToOcc(pos) - gcopter_config_->dilateRadiusHard;
    if (!std::isfinite(sphere_radius) || sphere_radius <= 0.0)
    {
      collision_time = std::max(0.0, probe_t - now_t);
      return false;
    }
  }
  return true;
}

bool FastPlannerManager::checkTrajVelocity()
{
  if (commit_store_->cmd_traj_info.empty())
  {
    return true;
  }
  commit_store_->cmd_traj_info.lock();
  const geometry_utils::Trajectory committed_pos = commit_store_->cmd_traj_info.posTraj();
  const bool backup_available =
      commit_store_->cmd_traj_info.backupTrajAvilibale();
  commit_store_->cmd_traj_info.unlock();
  if (committed_pos.empty())
  {
    return true;
  }
  const double max_v = committed_pos.getMaxVelRate();
  const double max_a = committed_pos.getMaxAccRate();
  double allowed_v = 1.35 * std::max(0.2, gcopter_config_->maxVelMag);
  double allowed_a = 1.50 * std::max(0.2, gcopter_config_->maxAccMag);
  if (backup_available)
  {
    allowed_v = std::max(allowed_v, 1.05 * gcopter_config_->backupMaxVel);
    allowed_a = std::max(allowed_a, 1.05 * gcopter_config_->backupMaxAcc);
  }
  return max_v <= allowed_v && max_a <= allowed_a;
}

bool FastPlannerManager::hasCommittedTrajectory() const
{
  return commit_store_ && !commit_store_->cmd_traj_info.empty() &&
         local_data_.duration_ > 1.0e-4;
}

bool FastPlannerManager::hasCommittedBackup() const
{
  return commit_store_ && !commit_store_->cmd_traj_info.empty() &&
         commit_store_->cmd_traj_info.backupTrajAvilibale();
}

bool FastPlannerManager::hasCommittedStopTrajectory() const
{
  return committed_stop_active_ && hasCommittedTrajectory() &&
         committedTrajectoryRemainingTime() > 0.0;
}

double FastPlannerManager::timeToCommittedBackup() const
{
  if (!commit_store_ || commit_store_->cmd_traj_info.empty() ||
      !commit_store_->cmd_traj_info.backupTrajAvilibale())
  {
    return std::numeric_limits<double>::infinity();
  }
  const double start_wt = commit_store_->cmd_traj_info.getStartWallTime();
  const double backup_start_tt = commit_store_->cmd_traj_info.getBackupTrajStartTT();
  if (!std::isfinite(start_wt) || !std::isfinite(backup_start_tt))
  {
    return std::numeric_limits<double>::infinity();
  }
  const double now_tt = ros::Time::now().toSec() - start_wt;
  return backup_start_tt - now_tt;
}

double FastPlannerManager::committedTrajectoryRemainingTime() const
{
  if (!commit_store_ || commit_store_->cmd_traj_info.empty())
  {
    return 0.0;
  }
  const double start_wt = commit_store_->cmd_traj_info.getStartWallTime();
  const double total_duration = commit_store_->cmd_traj_info.getTotalDuration();
  if (!std::isfinite(start_wt) || total_duration <= 0.0)
  {
    return 0.0;
  }
  const double duration =
      local_data_.duration_ > 1.0e-4 ? std::min(local_data_.duration_, total_duration)
                                     : total_duration;
  return std::max(0.0, duration - (ros::Time::now().toSec() - start_wt));
}

bool FastPlannerManager::isOnCommittedBackup() const
{
  if (!commit_store_ || commit_store_->cmd_traj_info.empty())
  {
    return false;
  }
  const double start_wt = commit_store_->cmd_traj_info.getStartWallTime();
  if (!std::isfinite(start_wt))
  {
    return false;
  }
  const double now_tt = ros::Time::now().toSec() - start_wt;
  return commit_store_->cmd_traj_info.isTTOnBackupTraj(now_tt);
}

bool FastPlannerManager::updateRogMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                      const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  if (!gcopter_config_ || !gcopter_config_->rogMapEnable || !map_manager_ || !odom_msg)
  {
    return false;
  }

  rog_map::PointCloud cloud;
  if (!pointCloud2ToRogCloud(cloud_msg, cloud))
  {
    return false;
  }

  general_utils::Pose pose;
  pose.first = general_utils::Vec3f(odom_msg->pose.pose.position.x,
                                    odom_msg->pose.pose.position.y,
                                    odom_msg->pose.pose.position.z);
  pose.second = general_utils::Quatf(odom_msg->pose.pose.orientation.w,
                                     odom_msg->pose.pose.orientation.x,
                                     odom_msg->pose.pose.orientation.y,
                                     odom_msg->pose.pose.orientation.z);
  try
  {
    map_manager_->updateMap(cloud, pose);
    rog_map_updated_ = true;
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "[highspeed_exp adapter] ROGMap update failed: "
                                      << e.what());
    return false;
  }
  return true;
}

bool FastPlannerManager::sampleCoverageMap(const CoverageMapSpec &spec,
                                           CoverageMapDelta &delta) const
{
  delta.samples.clear();
  if (!spec.valid() || !map_manager_ || !rog_map_updated_)
  {
    return false;
  }

  rog_map::Vec3f updated_min;
  rog_map::Vec3f updated_max;
  if (!map_manager_->getUpdatedBox(updated_min, updated_max))
  {
    return false;
  }
  Eigen::Vector3d bounded_min = updated_min.cast<double>();
  Eigen::Vector3d bounded_max = updated_max.cast<double>();
  bounded_min = bounded_min.cwiseMax(spec.min);
  bounded_max = bounded_max.cwiseMin(spec.max);
  if ((bounded_min.array() > bounded_max.array()).any())
  {
    return false;
  }

  Eigen::Vector3i min_index = spec.positionToIndex(bounded_min);
  Eigen::Vector3i max_index = spec.positionToIndex(
      bounded_max - Eigen::Vector3d::Constant(1.0e-6));
  min_index = min_index.cwiseMax(Eigen::Vector3i::Zero());
  max_index = max_index.cwiseMin(spec.dims - Eigen::Vector3i::Ones());
  if ((min_index.array() > max_index.array()).any())
  {
    return false;
  }

  const int estimated_count =
      (max_index - min_index + Eigen::Vector3i::Ones()).prod();
  delta.samples.reserve(std::max(0, estimated_count / 3));
  for (int z = min_index.z(); z <= max_index.z(); ++z)
  {
    for (int y = min_index.y(); y <= max_index.y(); ++y)
    {
      for (int x = min_index.x(); x <= max_index.x(); ++x)
      {
        const Eigen::Vector3i index(x, y, z);
        const Eigen::Vector3d position = spec.indexToPosition(index);
        const general_utils::Vec3f position_f = position;
        if (!map_manager_->insideLocalMap(position_f))
        {
          continue;
        }
        const rog_map::GridType raw_state =
            map_manager_->getGridType(position_f);
        CoverageVoxelState coverage_state = CoverageVoxelState::UNKNOWN;
        if (raw_state == rog_map::GridType::OCCUPIED)
        {
          coverage_state = CoverageVoxelState::OCCUPIED;
        }
        else if (raw_state == rog_map::GridType::KNOWN_FREE)
        {
          const rog_map::GridType inflated_state =
              map_manager_->getInfGridType(position_f);
          coverage_state =
              inflated_state == rog_map::GridType::OCCUPIED ||
                      map_manager_->isOccupiedInflate(position_f)
                  ? CoverageVoxelState::UNSAFE_FREE
                  : CoverageVoxelState::KNOWN_FREE;
        }
        // Persistent coverage starts unknown. Omitting unknown samples both
        // reduces the cross-thread copy and prevents a sliding local map from
        // erasing previously observed global evidence.
        if (coverage_state != CoverageVoxelState::UNKNOWN)
        {
          delta.samples.push_back({spec.flatten(index), coverage_state});
        }
      }
    }
  }
  return true;
}

bool FastPlannerManager::isSafetyMapReady() const
{
  const bool lio_ready = lidar_map_interface_ && lidar_map_interface_->ld_ &&
                         !lidar_map_interface_->ld_->lidar_cloud_.points.empty();
  return rog_map_updated_ || lio_ready;
}

MapVoxelState FastPlannerManager::querySafetyState(const Eigen::Vector3d &pos) const
{
  if (!pos.allFinite())
  {
    return MapVoxelState::OUT_OF_MAP;
  }

  const double safe_distance =
      gcopter_config_ ? std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance)
                      : 0.45;
  const double lio_dist = safetyDistanceToOcc(pos);
  const bool lio_safe = std::isfinite(lio_dist) && lio_dist >= safe_distance;

  if (!map_manager_ || !rog_map_updated_)
  {
    if (!lidar_map_interface_)
    {
      return MapVoxelState::UNKNOWN;
    }
    return lio_safe ? MapVoxelState::KNOWN_FREE : MapVoxelState::OCCUPIED;
  }

  const general_utils::Vec3f pos_f = pos;
  if (!map_manager_->insideLocalMap(pos_f))
  {
    return MapVoxelState::OUT_OF_MAP;
  }
  const auto inf_state = map_manager_->getInfGridType(pos_f);
  const auto raw_state = map_manager_->getGridType(pos_f);

  if (inf_state == rog_map::GridType::OUT_OF_MAP ||
      raw_state == rog_map::GridType::OUT_OF_MAP)
  {
    return MapVoxelState::OUT_OF_MAP;
  }
  if (raw_state == rog_map::GridType::OCCUPIED ||
      (inf_state == rog_map::GridType::OCCUPIED &&
       raw_state == rog_map::GridType::KNOWN_FREE))
  {
    return MapVoxelState::OCCUPIED;
  }

  if (inf_state == rog_map::GridType::KNOWN_FREE ||
      raw_state == rog_map::GridType::KNOWN_FREE)
  {
    bool rog_safe = !map_manager_->isOccupiedInflate(pos_f);
    if (rog_safe && map_manager_->hasESDF())
    {
      double dist = 0.0;
      general_utils::Vec3f grad = general_utils::Vec3f::Zero();
      rog_safe = map_manager_->evaluateESDF(pos_f, dist, grad) &&
                 std::isfinite(dist) && dist >= safe_distance;
    }
    if (rog_safe ||
        (gcopter_config_ && gcopter_config_->rogKnownFreeFallbackToLio &&
         lio_safe))
    {
      return MapVoxelState::KNOWN_FREE;
    }
    return MapVoxelState::OCCUPIED;
  }

  const bool rog_unknown_like =
      inf_state == rog_map::GridType::UNKNOWN ||
      inf_state == rog_map::GridType::UNDEFINED ||
      inf_state == rog_map::GridType::FRONTIER ||
      (inf_state == rog_map::GridType::OCCUPIED &&
       (raw_state == rog_map::GridType::UNKNOWN ||
        raw_state == rog_map::GridType::UNDEFINED ||
        raw_state == rog_map::GridType::FRONTIER));
  if (gcopter_config_ && gcopter_config_->rogKnownFreeFallbackToLio &&
      lio_safe && rog_unknown_like)
  {
    return MapVoxelState::KNOWN_FREE;
  }
  return MapVoxelState::UNKNOWN;
}

const char *FastPlannerManager::safetyStateName(MapVoxelState state) const
{
  switch (state)
  {
  case MapVoxelState::OCCUPIED:
    return "occupied";
  case MapVoxelState::KNOWN_FREE:
    return "known_free";
  case MapVoxelState::UNKNOWN:
    return "unknown";
  case MapVoxelState::OUT_OF_MAP:
    return "out_of_map";
  }
  return "invalid";
}

double FastPlannerManager::safetyDistanceToOcc(const Eigen::Vector3d &pos) const
{
  // Keep highspeedExp's clearance semantics.  In particular, ROG is normally
  // configured without ESDF and its inflated occupancy must not turn every
  // unknown frontier sample into zero clearance.  The LIO KD-tree returns the
  // same 10 m empty-neighbour placeholder used by the original frontend.
  if (!pos.allFinite() || !lidar_map_interface_)
  {
    return -std::numeric_limits<double>::infinity();
  }
  return lidar_map_interface_->getDisToOcc(pos);
}

RaycastSafetyInfo FastPlannerManager::raycastSafety(const Eigen::Vector3d &start,
                                                    const Eigen::Vector3d &end,
                                                    bool unknown_as_occupied,
                                                    double safe_distance,
                                                    double step) const
{
  RaycastSafetyInfo info;
  if (!start.allFinite() || !end.allFinite())
  {
    info.blocked_by_unknown = true;
    info.first_blocked_state = MapVoxelState::OUT_OF_MAP;
    return info;
  }

  const Eigen::Vector3d delta = end - start;
  info.length = delta.norm();
  const int samples = std::max(
      1, static_cast<int>(std::ceil(info.length / std::max(1.0e-3, step))));
  Eigen::Vector3d prev = start;
  info.all_known_free = true;
  for (int i = 0; i <= samples; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector3d p = start + ratio * delta;
    const MapVoxelState state = querySafetyState(p);
    const double clearance = safetyDistanceToOcc(p);
    if (std::isfinite(clearance))
    {
      info.min_clearance = std::min(info.min_clearance, clearance);
    }

    const bool too_close =
        std::isfinite(clearance) && clearance < std::max(0.0, safe_distance);
    const bool unknown_block =
        unknown_as_occupied &&
        (state == MapVoxelState::UNKNOWN || state == MapVoxelState::OUT_OF_MAP);
    const bool occupied_block = state == MapVoxelState::OCCUPIED || too_close;
    if (occupied_block || unknown_block)
    {
      info.all_known_free = false;
      info.first_blocked_pos = p;
      info.first_blocked_state = state;
      info.blocked_by_occupied = occupied_block;
      info.blocked_by_unknown = unknown_block;
      return info;
    }

    if (i > 0)
    {
      info.known_free_length += (p - prev).norm();
    }
    prev = p;
  }
  if (!std::isfinite(info.min_clearance))
  {
    info.min_clearance = safetyDistanceToOcc(start);
  }
  return info;
}

double FastPlannerManager::forwardKnownFreeLength(const Eigen::Vector3d &start,
                                                  const Eigen::Vector3d &direction,
                                                  double max_len,
                                                  double safe_distance,
                                                  double step) const
{
  if (!start.allFinite() || !direction.allFinite() ||
      direction.norm() < 1.0e-4 || max_len <= 0.0)
  {
    return 0.0;
  }
  const Eigen::Vector3d dir = direction.normalized();
  const int samples = std::max(
      1, static_cast<int>(std::ceil(max_len / std::max(1.0e-3, step))));
  double known_free_len = 0.0;
  Eigen::Vector3d prev = start;
  for (int i = 0; i <= samples; ++i)
  {
    const double len = std::min(max_len, i * std::max(1.0e-3, step));
    const Eigen::Vector3d p = start + len * dir;
    if (querySafetyState(p) != MapVoxelState::KNOWN_FREE ||
        safetyDistanceToOcc(p) < safe_distance)
    {
      break;
    }
    if (i > 0)
    {
      known_free_len += (p - prev).norm();
    }
    prev = p;
  }
  return known_free_len;
}

bool FastPlannerManager::checkTrajectoryKnownFree(const Trajectory<7> &traj,
                                                  double safe_distance,
                                                  double step,
                                                  bool unknown_as_occupied) const
{
  if (traj.getPieceNum() <= 0)
  {
    return false;
  }
  const double duration = traj.getTotalDuration();
  if (!std::isfinite(duration) || duration <= 0.0)
  {
    return false;
  }
  const double query_step = std::max(1.0e-3, step);
  Eigen::Vector3d last = traj.getPos(0.0);
  for (double t = query_step; t <= duration + 1.0e-6; t += query_step)
  {
    const double tt = std::min(t, duration);
    const Eigen::Vector3d p = traj.getPos(tt);
    if (!raycastSafety(last, p, unknown_as_occupied, safe_distance, query_step)
             .all_known_free)
    {
      return false;
    }
    last = p;
    if (tt >= duration)
    {
      break;
    }
  }
  return true;
}

double FastPlannerManager::estimatePathKnownFreeLength(const std::vector<Eigen::Vector3d> &path,
                                                       double safe_distance,
                                                       double step) const
{
  if (path.size() < 2)
  {
    return 0.0;
  }
  double known_free_len = 0.0;
  Eigen::Vector3d last = path.front();
  const MapVoxelState start_state = querySafetyState(last);
  const double start_clearance = safetyDistanceToOcc(last);
  if (start_state != MapVoxelState::KNOWN_FREE ||
      (std::isfinite(start_clearance) && start_clearance < safe_distance))
  {
    return 0.0;
  }
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    const Eigen::Vector3d next = path[i];
    const double seg_len = (next - last).norm();
    if (seg_len < 1.0e-6)
    {
      continue;
    }
    const int samples = std::max(
        1, static_cast<int>(std::ceil(seg_len / std::max(1.0e-3, step))));
    Eigen::Vector3d prev = last;
    for (int s = 1; s <= samples; ++s)
    {
      const double ratio = static_cast<double>(s) / static_cast<double>(samples);
      const Eigen::Vector3d p = last + ratio * (next - last);
      if (!raycastSafety(prev, p, true, safe_distance, step).all_known_free)
      {
        return known_free_len;
      }
      known_free_len += (p - prev).norm();
      prev = p;
    }
    last = next;
  }
  return known_free_len;
}

double FastPlannerManager::estimatePathMinClearance(const std::vector<Eigen::Vector3d> &path,
                                                    double step) const
{
  if (path.empty())
  {
    return -std::numeric_limits<double>::infinity();
  }
  double min_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    const Eigen::Vector3d start = path[i - 1];
    const Eigen::Vector3d end = path[i];
    const double len = (end - start).norm();
    const int samples = std::max(
        1, static_cast<int>(std::ceil(len / std::max(1.0e-3, step))));
    for (int s = 0; s <= samples; ++s)
    {
      const double ratio = static_cast<double>(s) / static_cast<double>(samples);
      const double dist = safetyDistanceToOcc(start + ratio * (end - start));
      if (std::isfinite(dist))
      {
        min_clearance = std::min(min_clearance, dist);
      }
    }
  }
  if (!std::isfinite(min_clearance))
  {
    min_clearance = safetyDistanceToOcc(path.front());
  }
  return min_clearance;
}

double FastPlannerManager::estimatePathTurnAngle(const std::vector<Eigen::Vector3d> &path) const
{
  if (path.size() < 3)
  {
    return 0.0;
  }
  double turn_sum = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i)
  {
    Eigen::Vector3d a = path[i] - path[i - 1];
    Eigen::Vector3d b = path[i + 1] - path[i];
    if (a.norm() < 1.0e-3 || b.norm() < 1.0e-3)
    {
      continue;
    }
    const double dot = std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0);
    turn_sum += std::acos(dot);
  }
  return turn_sum;
}

SegmentSafetyInfo FastPlannerManager::evaluatePathSegmentSafety(const std::vector<Eigen::Vector3d> &path,
                                                                double yaw1,
                                                                double yaw2) const
{
  SegmentSafetyInfo info;
  if (!gcopter_config_)
  {
    return info;
  }
  info.path_length = pathLength(path);
  const double step = std::max(0.05, gcopter_config_->safetyMapQueryStep);
  const double safe_distance =
      std::max(0.05, gcopter_config_->commitKnownFreeSafeDistance);
  info.known_free_length =
      estimatePathKnownFreeLength(path, safe_distance, step);
  info.min_clearance = estimatePathMinClearance(path, step);
  info.turn_angle = estimatePathTurnAngle(path);
  for (std::size_t i = 1; i + 1 < path.size(); ++i)
  {
    const Eigen::Vector3d a = path[i] - path[i - 1];
    const Eigen::Vector3d b = path[i + 1] - path[i];
    const double a_len = a.norm();
    const double b_len = b.norm();
    if (a_len < 0.20 || b_len < 0.20)
    {
      continue;
    }
    const double dot = std::clamp(a.dot(b) / (a_len * b_len), -1.0, 1.0);
    const double local_turn = std::acos(dot);
    info.max_local_turn = std::max(info.max_local_turn, local_turn);
    const double twice_area = a.cross(b).norm();
    const double chord = (path[i + 1] - path[i - 1]).norm();
    if (local_turn > 0.05 && twice_area > 1.0e-4 && chord > 0.20)
    {
      const double radius = a_len * b_len * chord / (2.0 * twice_area);
      if (std::isfinite(radius) && radius > 0.05)
      {
        info.min_turn_radius = std::min(info.min_turn_radius, radius);
      }
    }
  }

  Eigen::Vector3d path_heading = Eigen::Vector3d::Zero();
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    path_heading = path[i] - path.front();
    path_heading.z() = 0.0;
    if (path_heading.norm() > 0.20)
    {
      break;
    }
  }
  Eigen::Vector3d motion_heading = local_data_.curr_vel_;
  motion_heading.z() = 0.0;
  if (motion_heading.norm() < 0.50)
  {
    motion_heading = Eigen::Vector3d(std::cos(yaw1), std::sin(yaw1), 0.0);
  }
  if (path_heading.norm() > 1.0e-3 && motion_heading.norm() > 1.0e-3)
  {
    info.initial_heading_delta = std::acos(std::clamp(
        path_heading.normalized().dot(motion_heading.normalized()), -1.0, 1.0));
  }
  info.yaw_delta = std::fabs(yawDelta(yaw1, yaw2));
  info.current_speed = local_data_.curr_vel_.norm();

  const double brake_acc = std::max(1.0, gcopter_config_->brakeAccel);
  const double latency =
      std::max(0.0, gcopter_config_->plannerLatency) +
      std::max(0.0, gcopter_config_->controlLatency);
  const double stop_distance =
      info.current_speed * latency +
      info.current_speed * info.current_speed / (2.0 * brake_acc) +
      std::max(0.0, gcopter_config_->safetyBrakeMargin);

  const bool entire_short_path_known_free =
      info.path_length > 0.05 &&
      info.known_free_length + std::max(0.05, step) >= info.path_length;
  // A stationary vehicle does not need a full cruise-speed braking runway.
  // Requiring knownFreeShortLength here rejects safe rolling steps that end at
  // a nearby junction and can deadlock the planner after a path reversal is
  // truncated. Once moving, retain the conservative braking requirement.
  info.backup_feasible =
      info.current_speed <= 0.20
          ? entire_short_path_known_free
          : info.known_free_length >=
                std::max(gcopter_config_->knownFreeShortLength, stop_distance);
  return info;
}

SegmentVelocityLimit FastPlannerManager::computeSegmentVelocityLimit(const SegmentSafetyInfo &info) const
{
  SegmentVelocityLimit limit;
  if (!gcopter_config_)
  {
    limit.final_limit = 3.0;
    return limit;
  }

  const double v_min = std::max(0.5, gcopter_config_->minSegmentVel);
  const double v_global = std::max(v_min, gcopter_config_->maxVelMag);
  limit.open = std::min(v_global,
                        std::max(v_min, gcopter_config_->openSegmentVel));
  limit.known_free =
      knownFreeAdaptiveVelocity(*gcopter_config_, info.known_free_length);

  const double brake_acc = std::max(1.0, gcopter_config_->brakeAccel);
  const double latency =
      std::max(0.0, gcopter_config_->plannerLatency) +
      std::max(0.0, gcopter_config_->controlLatency);
  const double brake_available =
      info.known_free_length - info.current_speed * latency -
      std::max(0.0, gcopter_config_->safetyBrakeMargin);
  limit.brake = brake_available > 0.0
                    ? std::sqrt(2.0 * brake_acc * brake_available)
                    : v_min;

  const double clearance_margin =
      std::max(0.0, gcopter_config_->dynamicVelocityClearanceMargin);
  if (std::isfinite(info.min_clearance) &&
      info.min_clearance > clearance_margin)
  {
    const double clearance_for_speed =
        std::max(0.05, info.min_clearance - clearance_margin);
    limit.clearance =
        0.92 * std::sqrt(2.0 * gcopter_config_->maxAccMag * clearance_for_speed);
  }
  else
  {
    limit.clearance = v_min;
  }
  if (!gcopter_config_->dynamicVelocityEnable)
  {
    limit.clearance = limit.open;
  }

  limit.curvature = limit.open;
  if (gcopter_config_->turnVelocityEnable)
  {
    const double effective_turn =
        std::max(info.max_local_turn, info.initial_heading_delta);
    if (effective_turn > 0.10 && std::isfinite(info.min_turn_radius))
    {
      const double radius = std::max(
          gcopter_config_->curvatureMinRadius,
          info.min_turn_radius);
      limit.curvature = std::min(
          limit.curvature,
          std::sqrt(std::max(0.1,
                             gcopter_config_->turnLateralAcceleration * radius)));
    }
    if (effective_turn >= gcopter_config_->turnHardAngle)
    {
      limit.curvature =
          std::min(limit.curvature, gcopter_config_->turnHardVelocity);
    }
    else if (effective_turn >= gcopter_config_->turnSoftAngle)
    {
      const double ratio = std::clamp(
          (effective_turn - gcopter_config_->turnSoftAngle) /
              std::max(0.05, gcopter_config_->turnHardAngle -
                                 gcopter_config_->turnSoftAngle),
          0.0,
          1.0);
      const double angle_cap =
          (1.0 - ratio) * gcopter_config_->turnSoftVelocity +
          ratio * gcopter_config_->turnHardVelocity;
      limit.curvature = std::min(limit.curvature, angle_cap);
    }
    limit.curvature = std::clamp(limit.curvature, v_min, limit.open);
  }

  limit.yaw = limit.open;
  if (info.yaw_delta > 0.15 && gcopter_config_->yaw_max_vel > 1.0e-3)
  {
    const double yaw_time = info.yaw_delta / gcopter_config_->yaw_max_vel;
    limit.yaw = info.path_length / std::max(0.2, yaw_time);
  }

  limit.backup = info.backup_feasible
                     ? limit.open
                     : gcopter_config_->velocityShortKnownFree;
  const bool corridor_cruise =
      gcopter_config_->corridorCruiseEnable && info.backup_feasible &&
      info.known_free_length >= gcopter_config_->knownFreeMediumLength &&
      info.turn_angle < 0.25 && info.yaw_delta < 0.35;
  if (corridor_cruise)
  {
    limit.clearance = limit.open;
    if (!gcopter_config_->turnVelocityEnable)
    {
      limit.curvature = limit.open;
    }
    limit.yaw = limit.open;
  }

  const double raw_limit =
      std::min({limit.open, limit.known_free, limit.brake, limit.clearance,
                limit.curvature, limit.yaw, limit.backup});
  limit.final_limit =
      std::clamp(raw_limit, v_min, gcopter_config_->maxVelMag);

  if (raw_limit == limit.known_free)
  {
    limit.reason = "known_free";
  }
  else if (raw_limit == limit.brake)
  {
    limit.reason = "brake";
  }
  else if (raw_limit == limit.clearance)
  {
    limit.reason = "clearance";
  }
  else if (raw_limit == limit.curvature)
  {
    limit.reason = "curvature";
  }
  else if (raw_limit == limit.yaw)
  {
    limit.reason = "yaw";
  }
  else if (raw_limit == limit.backup)
  {
    limit.reason = "backup";
  }
  else
  {
    limit.reason = "open";
  }
  return limit;
}

EdgeSafetyCost FastPlannerManager::estimateHighSpeedEdgeCost(const std::vector<Eigen::Vector3f> &path,
                                                             const Eigen::Vector3d &start_vel,
                                                             double yaw1,
                                                             double yaw2) const
{
  EdgeSafetyCost cost;
  if (path.size() < 2 || !gcopter_config_)
  {
    cost.total_cost = 2e3;
    return cost;
  }

  std::vector<Eigen::Vector3d> path_d;
  path_d.reserve(path.size());
  for (const auto &p : path)
  {
    path_d.emplace_back(p.cast<double>());
  }
  SegmentSafetyInfo safety = evaluatePathSegmentSafety(path_d, yaw1, yaw2);
  safety.current_speed = start_vel.norm();
  Eigen::Vector3d edge_heading = path_d[1] - path_d[0];
  edge_heading.z() = 0.0;
  Eigen::Vector3d motion_heading = start_vel;
  motion_heading.z() = 0.0;
  if (motion_heading.norm() < 0.50)
  {
    motion_heading = Eigen::Vector3d(std::cos(yaw1), std::sin(yaw1), 0.0);
  }
  if (edge_heading.norm() > 1.0e-3 && motion_heading.norm() > 1.0e-3)
  {
    safety.initial_heading_delta = std::acos(std::clamp(
        edge_heading.normalized().dot(motion_heading.normalized()), -1.0, 1.0));
  }
  const double brake_acc = std::max(1.0, gcopter_config_->brakeAccel);
  const double latency =
      std::max(0.0, gcopter_config_->plannerLatency) +
      std::max(0.0, gcopter_config_->controlLatency);
  const double stop_distance =
      safety.current_speed * latency +
      safety.current_speed * safety.current_speed / (2.0 * brake_acc) +
      std::max(0.0, gcopter_config_->safetyBrakeMargin);
  safety.backup_feasible =
      safety.known_free_length >=
      std::max(gcopter_config_->knownFreeShortLength, stop_distance);
  const SegmentVelocityLimit limit = computeSegmentVelocityLimit(safety);

  cost.path_length = safety.path_length;
  cost.known_free_length = safety.known_free_length;
  cost.min_clearance = safety.min_clearance;
  cost.turn_angle = safety.turn_angle;
  cost.initial_heading_delta = safety.initial_heading_delta;
  cost.backup_feasible = safety.backup_feasible;

  const double acc = std::max(1.0, gcopter_config_->maxAccMag);
  const double vmax = std::max(0.5, limit.final_limit);
  const double accel_dist = vmax * vmax / acc;
  if (cost.path_length <= accel_dist)
  {
    cost.time_cost = 2.0 * std::sqrt(cost.path_length / acc);
  }
  else
  {
    cost.time_cost =
        2.0 * vmax / acc + (cost.path_length - accel_dist) / vmax;
  }

  cost.turn_penalty = gcopter_config_->edgeTurnPenaltyWeight *
                      (safety.turn_angle + 1.5 * safety.initial_heading_delta);
  const double required_known =
      std::min(cost.path_length,
               std::max(gcopter_config_->knownFreeShortLength,
                        start_vel.norm() *
                                (gcopter_config_->plannerLatency +
                                 gcopter_config_->controlLatency) +
                            start_vel.squaredNorm() /
                                (2.0 * std::max(1.0, gcopter_config_->brakeAccel)) +
                            gcopter_config_->safetyBrakeMargin));
  if (cost.known_free_length + 1.0e-3 < required_known)
  {
    cost.known_free_penalty =
        gcopter_config_->edgeKnownFreePenaltyWeight *
        (required_known - cost.known_free_length);
  }
  if (!cost.backup_feasible)
  {
    cost.backup_penalty = gcopter_config_->edgeBackupPenaltyWeight;
  }
  cost.yaw_penalty =
      gcopter_config_->edgeYawPenaltyWeight *
      std::fabs(yawDelta(yaw1, yaw2)) /
      std::max(0.1, gcopter_config_->yaw_max_vel);
  cost.total_cost = cost.time_cost + cost.turn_penalty + cost.known_free_penalty +
                    cost.backup_penalty + cost.yaw_penalty;

  if (gcopter_config_->velocityLogEnable)
  {
    ROS_INFO_STREAM_THROTTLE(
        0.5,
        "[edge cost] len=" << cost.path_length
                            << " total=" << cost.total_cost
                            << " time=" << cost.time_cost
                            << " turn_pen=" << cost.turn_penalty
                            << " known_pen=" << cost.known_free_penalty
                            << " backup_pen=" << cost.backup_penalty
                            << " yaw_pen=" << cost.yaw_penalty
                            << " known_free=" << cost.known_free_length
                            << " min_clearance=" << cost.min_clearance
                            << " vel_limit=" << limit.final_limit
                            << " reason=" << limit.reason
                            << " backup_feasible=" << cost.backup_feasible);
  }
  return cost;
}

void FastPlannerManager::printSafetyMapSummary() const
{
  ROS_INFO_STREAM_THROTTLE(2.0, "[highspeed_exp adapter] safety map ready="
	                                    << isSafetyMapReady()
	                                    << " rog_ready=" << rog_map_updated_
	                                    << " cloud_size="
	                                    << (lidar_map_interface_ && lidar_map_interface_->ld_
	                                            ? lidar_map_interface_->ld_->lidar_cloud_.points.size()
                                            : 0U));
}

bool FastPlannerManager::YawTrajOpt(double &, double &, bool, bool)
{
  return false;
}

bool FastPlannerManager::YawTrajwithoutOpt(double &, double &, bool, bool)
{
  return false;
}

void FastPlannerManager::goalCallback(const geometry_msgs::PoseStampedConstPtr &)
{
}

void FastPlannerManager::posCallback(const nav_msgs::OdometryConstPtr &)
{
}

bool FastPlannerManager::YawInterpolationwithoutOpt(double &,
                                                    double &,
                                                    std::vector<double> &,
                                                    std::vector<double> &,
                                                    double &)
{
  return false;
}

void FastPlannerManager::YawLookforward(const Trajectory<5> &,
                                        double &,
                                        double &,
                                        std::vector<double> &,
                                        std::vector<double> &,
                                        double &)
{
}

void FastPlannerManager::YawLookforwardwithoutOpt(double &,
                                                  double &,
                                                  std::vector<double> &,
                                                  std::vector<double> &,
                                                  double &,
                                                  bool)
{
}

void FastPlannerManager::angleLimite(double &angle)
{
  angle = std::atan2(std::sin(angle), std::cos(angle));
}

void FastPlannerManager::calculateTimelb(const std::vector<Eigen::Vector3d> &path2next_goal,
                                         const double &current_yaw,
                                         const double &goal_yaw,
                                         double &time_lb)
{
  const double len = pathLength(path2next_goal);
  const double yaw_time = std::abs(yawDelta(current_yaw, goal_yaw)) /
                          std::max(0.1, gcopter_config_->yaw_max_vel);
  time_lb = std::max(len / std::max(0.1, gcopter_config_->maxVelMag), yaw_time);
}
} // namespace fast_planner
