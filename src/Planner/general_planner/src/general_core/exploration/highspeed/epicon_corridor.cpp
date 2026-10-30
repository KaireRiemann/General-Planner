#include <chrono>
#include <general_core/exploration/highspeed/epicon_corridor.h>
#include <epicon_native/corridor/firi.hpp>
#include <epicon_native/corridor/geo_utils.hpp>
#include <algorithm>
#include <deque>

namespace fast_planner {
namespace {
bool contains(const Eigen::MatrixX4d &poly, const Eigen::Vector3d &p) {
  return poly.rows() >= 4 && poly.allFinite() &&
      (poly.leftCols<3>() * p + poly.col(3)).maxCoeff() <= 1e-6;
}
}

bool clipEpiconCorridorToBoxes(const Eigen::Vector3d &head, const Eigen::Vector3d &tail,
    const std::vector<Eigen::Vector3d> &lower, const std::vector<Eigen::Vector3d> &upper,
    std::vector<Eigen::MatrixX4d> &polys, double min_overlap) {
  if(polys.empty() || lower.empty() || lower.size()!=upper.size()) return false;
  // A single convex cell may span several overlapping task boxes. Keep all
  // valid intersections and search a connected sequence; greedily choosing
  // one box per cell can discard the only transition to the next floor.
  struct Cell { Eigen::MatrixX4d poly; int layer; int parent{-2}; };
  std::vector<Cell> cells;
  std::vector<std::vector<int>> layers(polys.size());
  for(size_t i=0;i<polys.size();++i) for(size_t box=0;box<lower.size();++box) {
    const Eigen::Vector3d lo=lower[box].array()+.03,hi=upper[box].array()-.03;
    if(!lo.allFinite() || !hi.allFinite() || (lo.array()>=hi.array()).any()) continue;
    Eigen::MatrixX4d clipped(polys[i].rows()+6,4);
    clipped.topRows(polys[i].rows())=polys[i]; clipped.bottomRows(6).setZero();
    for(int axis=0;axis<3;++axis) {
      const int row=polys[i].rows()+2*axis;
      clipped(row,axis)=1; clipped(row,3)=-hi[axis];
      clipped(row+1,axis)=-1; clipped(row+1,3)=lo[axis];
    }
    Eigen::Vector3d interior;
    if(!epicon_native::geo_utils::findInterior(clipped,interior)) continue;
    layers[i].push_back(cells.size()); cells.push_back({clipped,static_cast<int>(i),-2});
  }
  std::deque<int> queue;
  for(int index:layers.front()) if(contains(cells[index].poly,head)) {
    cells[index].parent=-1; queue.push_back(index);
  }
  int reached=-1;
  while(!queue.empty()) {
    const int current=queue.front(); queue.pop_front();
    const int layer=cells[current].layer;
    if(contains(cells[current].poly,tail)) {
      reached=current; break;
    }
    for(int next_layer=layer;next_layer<static_cast<int>(layers.size());++next_layer)
      for(int next:layers[next_layer]) {
        if(cells[next].parent!=-2 ||
           !epicon_native::geo_utils::overlap(cells[current].poly,cells[next].poly,min_overlap)) continue;
        cells[next].parent=current; queue.push_back(next);
      }
  }
  if(reached<0) return false;
  std::vector<Eigen::MatrixX4d> bounded;
  for(int index=reached;index>=0;index=cells[index].parent) bounded.push_back(cells[index].poly);
  std::reverse(bounded.begin(),bounded.end());
  if(bounded.size()==1) bounded.push_back(bounded.front());
  polys.swap(bounded); return true;
}

bool epiconRecoveryPoint(const Eigen::Vector3d &start,
                         const std::vector<Eigen::Vector3d> &points,
                         Eigen::Vector3d &goal) {
  if (!start.allFinite()) return false;
  const Eigen::Vector3d extent(2,2,1);
  Eigen::MatrixX4d box=Eigen::MatrixX4d::Zero(6,4);
  for (int axis=0;axis<3;++axis) {
    box(2*axis,axis)=1; box(2*axis,3)=-start(axis)-extent(axis);
    box(2*axis+1,axis)=-1; box(2*axis+1,3)=start(axis)-extent(axis);
  }
  std::vector<Eigen::Vector3d> local;
  for (const auto &p:points)
    if (p.allFinite() && ((p-start).cwiseAbs().array()<extent.array()).all()) local.push_back(p);
  if (local.empty()) return false; // No obstacle requiring a clearance escape.
  Eigen::Matrix3Xd cloud(3,local.size());
  for (std::size_t k=0;k<local.size();++k) cloud.col(k)=local[k];
  Eigen::MatrixX4d poly;
  if (!epicon_native::firi::firi(box,cloud,start,start,poly) || !poly.allFinite() ||
      !epicon_native::geo_utils::findInterior(poly,goal)) return false;
  return goal.allFinite() && (goal-start).norm()>.10;
}

bool buildEpiconCorridor(const std::vector<Eigen::Vector3d> &path,
                        const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &lower,
                        const Eigen::Vector3d &upper,
                        double range, double clearance,
                        EpiconCorridor &result, bool simplify) {
  result = {};
  result.backend="firi"; result.map_representation="obstacle_points";
  result.clearance=clearance;
  const auto fail = [&](const char *reason) { result.failure = reason; return false; };
  if (path.size() < 2 || !lower.allFinite() || !upper.allFinite() ||
      (upper.array() <= lower.array()).any() || !std::isfinite(range) ||
      !std::isfinite(clearance) || range <= clearance || clearance < 0)
    return fail("invalid input");
  for (const auto &p : path) if (!p.allFinite()) return fail("non-finite path");
  result.path = path;
  Eigen::Vector3d b = path.front();
  // EPICON convexCover: 7 m progress, local boxes, FIRI, then inward offset.
  for (std::size_t i = 1; i < path.size();) {
    const Eigen::Vector3d a = b;
    if ((path[i] - a).norm() < 1e-6) { ++i; continue; }
    if ((path[i] - a).norm() > 7.0) b = a + (path[i] - a).normalized() * 7.0;
    else b = path[i++];
    const Eigen::Vector3d lo = (a.cwiseMin(b).array() - range).matrix().cwiseMax(lower);
    const Eigen::Vector3d hi = (a.cwiseMax(b).array() + range).matrix().cwiseMin(upper);
    Eigen::MatrixX4d bounds = Eigen::MatrixX4d::Zero(6, 4);
    for (int axis = 0; axis < 3; ++axis) {
      bounds(2*axis, axis) = 1; bounds(2*axis, 3) = -hi(axis);
      bounds(2*axis+1, axis) = -1; bounds(2*axis+1, 3) = lo(axis);
    }
    std::vector<Eigen::Vector3d> local;
    for (const auto &p : points)
      if (p.allFinite() && (p.array() > lo.array()).all() && (p.array() < hi.array()).all())
        local.push_back(p);
    Eigen::Matrix3Xd cloud(3, local.size());
    for (std::size_t k = 0; k < local.size(); ++k) cloud.col(k) = local[k];
    const auto inflate = [&](const Eigen::Vector3d &s, const Eigen::Vector3d &t,
                             int iterations, Eigen::MatrixX4d &poly) {
      // Upstream dereferenced valid_pc[0] for an empty crop. Empty space has
      // the bounded box as its corridor; it is not a failed map query.
      if (local.empty()) poly = bounds;
      else if (!epicon_native::firi::firi(bounds, cloud, s, t, poly, iterations)) return false;
      if (!poly.allFinite() || poly.rows() < 4) return false;
      poly.col(3).array() += clearance * poly.leftCols<3>().rowwise().norm().array();
      return true;
    };
    Eigen::MatrixX4d poly;
    if (!inflate(a, b, 4, poly)) return fail("FIRI segment");
    if (!contains(poly, b) || !contains(poly, a)) {
      // Preserve EPICON's point-seed fallback through tight turns. Every
      // fallback is inflated too; an uninflated gap is never a safety proof.
      for (const Eigen::Vector3d seed : {a, Eigen::Vector3d((a+b)*.5), b}) {
        if (!inflate(seed, seed, 1, poly)) return fail("FIRI point seed");
        result.planes.push_back(poly);
      }
    } else result.planes.push_back(poly);
  }
  auto &polys = result.planes;
  if (polys.empty()) return fail("empty cover");
  // Start at the last polytope containing the exact execution head.
  int start = -1;
  for (int k = static_cast<int>(polys.size())-1; k >= 0; --k)
    if (contains(polys[k], path.front())) { start=k; break; }
  if (start < 0) return fail("head outside point-cloud corridor");
  polys.erase(polys.begin(), polys.begin()+start);
  // When task boxes are present, intersect before simplifying. An obstacle-
  // free shortcut can pass outside the non-convex union and remove the cells
  // needed for a valid turn at the next floor.
  if(!simplify) return true;
  // Drop contained/redundant cells only when the shortcut has positive volume.
  std::vector<Eigen::MatrixX4d> simplified;
  for (std::size_t k=0; k<polys.size();) {
    simplified.push_back(polys[k]);
    if (k+1 == polys.size()) break;
    std::size_t next=k+1;
    for (std::size_t j=polys.size()-1; j>k+1; --j)
      if (epicon_native::geo_utils::overlap(polys[k],polys[j],1e-2)) { next=j; break; }
    if (!epicon_native::geo_utils::overlap(polys[k],polys[next],1e-2)) {
      Eigen::Vector3d terminal;
      if (!epicon_native::geo_utils::findInterior(polys[k],terminal) ||
          (terminal-path.front()).norm()<.15) return fail("disconnected first corridor");
      result.path = {path.front(),terminal};
      result.truncated = true;
      break;
    }
    k=next;
  }
  polys.swap(simplified);
  if (!contains(polys.back(),result.path.back())) return fail("tail outside point-cloud corridor");
  if (polys.size()==1) polys.push_back(polys.front());
  return true;
}
}
