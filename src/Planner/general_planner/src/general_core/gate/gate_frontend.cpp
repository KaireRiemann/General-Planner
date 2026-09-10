#include <general_core/gate/gate_frontend.hpp>
#include <traj_opt/costfunctional/spatialcosts/se3_shape_corridor_penalty.hpp>
#include <traj_opt/flatness/se3_flatness_map.hpp>
#include <algorithm>
#include <cmath>

namespace general_planner::gate {
namespace {
using Poly = Eigen::Matrix<double, 6, Eigen::Dynamic>;
void plane(Poly &poly, const Eigen::Vector3d &n, const Eigen::Vector3d &point) {
  const int k = poly.cols(); poly.conservativeResize(6, k + 1);
  poly.col(k).head<3>() = n; poly.col(k).tail<3>() = point;
}
Poly box(const Eigen::Vector3d &center, const Eigen::Matrix3d &R,
         const Eigen::Vector3d &lo, const Eigen::Vector3d &hi) {
  Poly p(6, 0);
  for (int i = 0; i < 3; ++i) {
    plane(p, R.col(i), center + R.col(i) * hi(i));
    plane(p, -R.col(i), center + R.col(i) * lo(i));
  }
  return p;
}
}
bool buildProblem(const aperture_detector::ApertureObservation &obs,
                  const Eigen::Vector3d &start, double yaw, const Config &cfg,
                  Plan &plan, std::string &reason) {
  auto fail = [&](const char *text) { reason = text; return false; };
  if (!obs.geometry_valid || obs.boundary.points.size() < 4 || obs.boundary.points.size() > 64)
    return fail("INVALID_APERTURE");
  if (!start.allFinite() || !std::isfinite(yaw)) return fail("INVALID_START");
  const Eigen::Vector3d c(obs.center.x, obs.center.y, obs.center.z);
  Eigen::Vector3d n(obs.normal.x, obs.normal.y, obs.normal.z);
  if (!c.allFinite() || !n.allFinite() || n.norm() < 1e-6) return fail("INVALID_NORMAL");
  n.normalize();
  if ((c - start).norm() > cfg.max_distance) return fail("APERTURE_TOO_FAR");
  if (n.dot(c - start) < 0) n = -n;
  const double depth = obs.depth_known ? obs.depth : cfg.wall_depth;
  if (!std::isfinite(depth) || depth <= 0 || depth > 2.0) return fail("INVALID_WALL_DEPTH");
  const double half_depth = depth * .5;
  if (n.dot(c - start) < half_depth + cfg.body_radius + cfg.margin + .05)
    return fail("START_TOO_CLOSE_TO_GATE");
  Eigen::Vector3d y = Eigen::Vector3d::UnitZ().cross(n);
  if (y.norm() < .1) return fail("HORIZONTAL_APERTURE_UNSUPPORTED");
  y.normalize();
  Eigen::Matrix3d R; R.col(0) = n; R.col(1) = y; R.col(2) = n.cross(y);
  Poly sides(6, 0);
  std::vector<Eigen::Vector3d> vertices;
  for (const auto &v : obs.boundary.points) {
    const Eigen::Vector3d p(v.x, v.y, v.z);
    if (!p.allFinite() || std::abs(n.dot(p - c)) > .04 || (p - c).norm() > 5)
      return fail("NON_PLANAR_APERTURE");
    vertices.push_back(p);
  }
  double winding = 0.0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto &a = vertices[i]; const auto &b = vertices[(i + 1) % vertices.size()];
    Eigen::Vector3d outward = (b - a).cross(n);
    if (outward.norm() < 1e-5) return fail("DEGENERATE_EDGE");
    outward.normalize();
    const double side = outward.dot(c - a);
    if (std::abs(side) < .01) return fail("CENTER_ON_BOUNDARY");
    if (i == 0) winding = side;
    if (side * winding <= 0) return fail("UNORDERED_APERTURE");
    if (side > 0) outward = -outward;
    for (const auto &v : vertices)
      if (outward.dot(v - a) > .005) return fail("NON_CONVEX_APERTURE");
    plane(sides, outward, a);
  }
  const double tunnel_half = std::max(cfg.tunnel_half_depth, half_depth + 2 * (std::max(cfg.body_radius,cfg.body_height) + cfg.margin) + cfg.overlap_slack);
  Eigen::Vector3d goal = c + std::max(cfg.exit_distance, tunnel_half + cfg.body_radius + .1) * n;
  if(cfg.restore_start_height) goal.z()=start.z();
  const Eigen::Vector3d start_local = R.transpose() * (start - c);
  const Eigen::Vector3d goal_local = R.transpose() * (goal - c);
  Eigen::Vector3d lo, hi;
  lo << start_local.x() - .6, std::min(-cfg.corridor_half_width, start_local.y() - .5),
                            std::min(-cfg.corridor_half_height, start_local.z() - .5);
  hi << -half_depth, std::max(cfg.corridor_half_width, start_local.y() + .5),
                          std::max(cfg.corridor_half_height, start_local.z() + .5);
  Poly before = box(c, R, lo, hi);
  Poly through = sides;
  plane(through, n, c + tunnel_half * n);
  plane(through, -n, c - tunnel_half * n);
  lo << half_depth, -cfg.corridor_half_width, std::min(-cfg.corridor_half_height,goal_local.z()-.5);
  hi << goal_local.x() + .6, cfg.corridor_half_width, std::max(cfg.corridor_half_height,goal_local.z()+.5);
  Poly after = box(c, R, lo, hi);
  plan = Plan{};
  plan.center = c; plan.normal = n; plan.wall_half_depth = half_depth; plan.frame = obs.header.frame_id;
  auto &p = plan.problem;
  p.head_pvaj.col(0) = start; p.tail_pvaj.col(0) = goal;
  p.hpolys = {before, through, after};
  const double seam = .5 * (tunnel_half + half_depth);
  p.guide_path = {start, c - seam * n, c + seam * n, goal};
  p.piece_num = 0;
  for (int i = 0; i < 3; ++i) {
    const int count = std::max(1, static_cast<int>(std::ceil((p.guide_path[i + 1] - p.guide_path[i]).norm() / cfg.piece_length)));
    p.piece_num += count;
    p.piece_to_corridor.insert(p.piece_to_corridor.end(), count, i);
  }
  p.reference_speed = cfg.speed * .7;
  p.min_duration = .5; p.max_duration = 20.0;
  p.horiz_half_len = cfg.body_radius; p.vert_half_len = cfg.body_height; p.safe_margin = cfg.margin;
  p.max_vel = cfg.speed; p.thrust_acc_min = cfg.thrust_min; p.thrust_acc_max = cfg.thrust_max;
  p.body_rate_max = cfg.body_rate; p.max_tilt = cfg.tilt;
  p.weight_time = cfg.weight_time; p.weight_corridor = cfg.weight_corridor;
  p.weight_vel = cfg.weight_velocity; p.weight_thrust = cfg.weight_thrust; p.weight_body_rate = cfg.weight_body_rate; p.weight_tilt = cfg.weight_tilt;
  p.use_yaw = true; p.yaw_heading_to_velocity = false; p.yaw = yaw; p.yaw_rate = 0.0;
  p.max_iterations = cfg.max_iterations;
  reason = "OK"; return true;
}
bool validateSample(const Plan &plan, double time, const Config &cfg,
                    const BodyClear &body_clear, std::string &reason) {
  auto fail = [&](const char *s) { reason = std::string(s)+" t="+std::to_string(time); return false; };
  const auto &traj = plan.trajectory;
  const auto &problem = plan.problem;
  const double t = std::clamp(time, 0.0, traj.getTotalDuration());
  const auto p = traj.getPos(t); const auto v = traj.getVel(t); const auto a = traj.getAcc(t);
  const auto j = traj.getJer(t); const auto s = traj.getSnap(t);
  traj_opt::SE3FlatnessMap flatness; flatness.setYawMode(true, false);
  traj_opt::SE3FlatnessOutput flat;
  if (!p.allFinite() || !v.allFinite() || !a.allFinite() || !j.allFinite() ||
      !flatness.forward(v, a, j, s, problem.yaw, 0.0, 9.81, flat)) return fail("NON_FINITE_TRAJECTORY");
  if (v.norm() > cfg.speed + .01 || flat.thrust < cfg.thrust_min - .01 ||
      flat.thrust > cfg.thrust_max + .01 || flat.omega.norm() > cfg.body_rate + .01 ||
      flat.z_b.z() < std::cos(cfg.tilt) - .001) return fail("DYNAMIC_LIMIT");
  double local = t;
  const int piece = traj.locatePieceIdx(local);
  const int id = problem.piece_to_corridor.at(piece);
  cost_functional::SE3ShapeConfig shape;
  shape.ellipsoid = Eigen::Vector3d(cfg.body_radius, cfg.body_radius, cfg.body_height);
  shape.safe_margin = cfg.margin;
  if (cost_functional::maxSE3ShapeCorridorViolation(p, v, a, j, s, problem.yaw, 0., 9.81,
          problem.hpolys, id, shape) > .001) return fail("SE3_CORRIDOR_VIOLATION");
  if (cfg.validation_policy!="corridor_only" && cfg.validation_policy!="corridor_and_map")
    return fail("INVALID_VALIDATION_POLICY");
  if (cfg.validation_policy=="corridor_and_map" &&
      (!body_clear || !body_clear(p, flat.R, shape.ellipsoid)))
    return fail("MAP_COLLISION_OR_UNKNOWN");
  reason = "OK"; return true;
}
bool validateTrajectory(const Plan &plan, const Config &cfg,
                        const BodyClear &body_clear, std::string &reason) {
  if (plan.trajectory.empty()) { reason = "EMPTY_TRAJECTORY"; return false; }
  const double total = plan.trajectory.getTotalDuration();
  if (!std::isfinite(total) || total <= 0 || total > 25) { reason = "INVALID_DURATION"; return false; }
  const int count = std::max(1, static_cast<int>(std::ceil(total / std::min(cfg.validation_dt,cfg.max_spatial_step / (cfg.speed+.01)))));
  for (int i = 0; i <= count; ++i) {
    if (plan.problem.should_stop && plan.problem.should_stop()) { reason = "CANCELED_OR_TIMEOUT"; return false; }
    if (!validateSample(plan, total * i / count, cfg, body_clear, reason)) return false;
  }
  // Sample both sides of every MINCO knot: a transition belongs to two cells.
  double knot = 0.0;
  for (int i = 0; i + 1 < plan.trajectory.getPieceNum(); ++i) {
    knot += plan.trajectory[i].getDuration();
    if (!validateSample(plan, knot - 1e-7, cfg, body_clear, reason) ||
        !validateSample(plan, knot + 1e-7, cfg, body_clear, reason)) return false;
  }
  return true;
}
} // namespace general_planner::gate

#include <general_core/gate/body_voxel_collision.hpp>
namespace general_planner::gate {
bool constrainTunnelToMap(Plan &plan, const Config &cfg,
                          const general_utils::vec_E<general_utils::Vec3f> &input,
                          double resolution, std::string &reason) {
  const Eigen::Vector3d r(cfg.body_radius,cfg.body_radius,cfg.body_height);
  const Eigen::Matrix3d R=Eigen::AngleAxisd(plan.problem.yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d ey=Eigen::Vector3d::UnitZ().cross(plan.normal).normalized();
  const Eigen::Vector3d ez=plan.normal.cross(ey);
  general_utils::vec_E<general_utils::Vec3f> cells;
  for(const auto &c:input)
    if(std::abs(plan.normal.dot(c-plan.center))<=plan.wall_half_depth+.5*resolution*plan.normal.cwiseAbs().sum()) cells.push_back(c);
  if(cells.empty()) return true;
  const auto original=plan.problem.hpolys[1];
  auto free=[&](const Eigen::Vector3d &p) {
    for(int k=0;k<original.cols();++k) {
      const Eigen::Vector3d n=original.col(k).head<3>();
      if(n.dot(p-original.col(k).tail<3>())+(r.array()*(R.transpose()*n).array()).matrix().norm()+cfg.margin>0) return false;
    }
    for(const auto &c:cells) if(bodyIntersectsVoxel(p,R,r,c,.5*resolution+cfg.margin)) return false;
    return true;
  };
  Eigen::Vector3d anchor=plan.center;
  bool found=free(anchor);
  // This is a local anchor search, not a change to the observed boundary.
  for(int ring=1;ring<=static_cast<int>(std::floor(cfg.map_anchor_search_radius/cfg.map_anchor_search_step)) && !found;++ring)
    for(int y=-ring;y<=ring && !found;++y) for(int z=-ring;z<=ring && !found;++z) {
      if(std::max(std::abs(y),std::abs(z))!=ring) continue;
      const Eigen::Vector3d candidate=plan.center+cfg.map_anchor_search_step*(y*ey+z*ez);
      if(free(candidate)) {anchor=candidate;found=true;}
    }
  if(!found) {
    // Some apertures need a non-level attitude. Do not silently replace their
    // geometry with a point-robot corridor; let the SE3 solve and validator decide.
    reason="NO_LEVEL_MAP_ANCHOR"; return true;
  }
  auto &through=plan.problem.hpolys[1];
  const Eigen::Matrix3d Q=R*r.cwiseInverse().cwiseAbs2().asDiagonal()*R.transpose();
  for(const auto &c:cells) {
    Eigen::Vector3d closest;
    if(bodyIntersectsVoxel(anchor,R,r,c,.5*resolution,&closest)) {reason="MAP_ANCHOR_OCCUPIED";return false;}
    Eigen::Vector3d n=Q*(closest-anchor); n.normalize();
    if(n.dot(anchor-closest)+(r.array()*(R.transpose()*n).array()).matrix().norm()+cfg.margin>1e-6) {
      reason="MAP_ANCHOR_MARGIN";return false;
    }
    bool merged=false;
    for(int k=0;k<through.cols();++k) {
      if((through.col(k).head<3>()-n).norm()<1e-6) {
        if(n.dot(closest)<n.dot(through.col(k).tail<3>())) through.col(k).tail<3>()=closest;
        merged=true;break;
      }
    }
    if(!merged) plane(through,n,closest);
  }
  const Eigen::Vector3d shift=anchor-plan.center;
  plan.problem.guide_path[1]+=shift; plan.problem.guide_path[2]+=shift;
  reason="OK";return true;
}
}
