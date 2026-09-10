#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace general_planner::gate {
// Exact ellipsoid/AABB intersection, including tangency. Enumerate the 27
// active sets of the convex box-constrained quadratic. No circumscribed sphere.
// Safety margin belongs on the voxel half extent, not on both bodies.
inline bool bodyIntersectsVoxel(const Eigen::Vector3d &p, const Eigen::Matrix3d &R,
                                const Eigen::Vector3d &r, const Eigen::Vector3d &cell,
                                double half_extent, Eigen::Vector3d *closest = nullptr) {
  if (!p.allFinite() || !R.allFinite() || !r.allFinite() || r.minCoeff()<=0 ||
      !cell.allFinite() || !std::isfinite(half_extent) || half_extent<0) return true;
  const Eigen::Vector3d extent=(R.array().square().matrix()*r.array().square().matrix()).array().sqrt();
  if (!closest && ((cell-p).cwiseAbs().array()>extent.array()+half_extent+1e-10).any()) return false;
  const Eigen::Vector3d lo=cell-p-Eigen::Vector3d::Constant(half_extent);
  const Eigen::Vector3d hi=cell-p+Eigen::Vector3d::Constant(half_extent);
  if ((lo.array()<=0).all() && (hi.array()>=0).all()) { if(closest) *closest=p; return true; }
  const Eigen::Matrix3d Q=R*r.cwiseInverse().cwiseAbs2().asDiagonal()*R.transpose();
  double best=std::numeric_limits<double>::infinity();
  for(int code=0;code<27;++code) {
    int state=code, free_ids[3], nf=0;
    Eigen::Vector3d x=Eigen::Vector3d::Zero();
    for(int k=0;k<3;++k) {
      const int s=state%3; state/=3;
      if(s==0) free_ids[nf++]=k; else x(k)=s==1 ? lo(k) : hi(k);
    }
    if(nf) {
      Eigen::Matrix3d A=Eigen::Matrix3d::Identity();
      Eigen::Vector3d b=Eigen::Vector3d::Zero();
      for(int i=0;i<nf;++i) {
        b(i)=-Q.row(free_ids[i]).dot(x);
        for(int j=0;j<nf;++j) A(i,j)=Q(free_ids[i],free_ids[j]);
      }
      const Eigen::Vector3d z=A.ldlt().solve(b);
      for(int i=0;i<nf;++i) x(free_ids[i])=z(i);
    }
    if((x.array()<lo.array()-1e-10).any() || (x.array()>hi.array()+1e-10).any()) continue;
    const double value=x.dot(Q*x);
    if(value<best) { best=value; if(closest) *closest=p+x; }
    if(!closest && value<=1.+1e-10) return true;
  }
  return best<=1.+1e-10;
}
} // namespace general_planner::gate
