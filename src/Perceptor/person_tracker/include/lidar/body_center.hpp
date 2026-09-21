#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace person_tracker {
struct BodyCenterFit {
  bool valid{false};
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
  double residual{0.0};
};

// Estimate the centre of a visible torso arc. Radius is fitted, not taken from
// simulator truth. Reject sparse, flat, short-arc and implausible mixed clusters.
inline BodyCenterFit fitBodyCenter(
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Vector2d & centroid, const Eigen::Vector2d & sensor)
{
  BodyCenterFit result;
  if (points.size() < 12 || !centroid.allFinite() || !sensor.allFinite()) return result;
  std::vector<double> heights;
  for (const auto & p : points) if (p.allFinite()) heights.push_back(p.z());
  if (heights.size() < 12) return result;
  std::sort(heights.begin(), heights.end());
  const double low = heights[heights.size()/5];
  const double high = heights[heights.size()*4/5];
  if (high-low < 0.25) return result;
  std::vector<Eigen::Vector2d> xy;
  for (const auto & p : points) {
    if (p.allFinite() && p.z() >= low && p.z() <= high) xy.push_back(p.head<2>()-centroid);
  }
  if (xy.size() < 10) return result;
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (const auto & p : xy) {
    const Eigen::Vector3d row(2*p.x(), 2*p.y(), 1.0);
    normal.noalias() += row*row.transpose();
    rhs.noalias() += row*p.squaredNorm();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spectrum(normal);
  if (spectrum.eigenvalues().minCoeff() < 1e-4*spectrum.eigenvalues().maxCoeff()) return result;
  const Eigen::Vector3d initial = normal.ldlt().solve(rhs);
  Eigen::Vector2d center = initial.head<2>();
  const double radius2 = initial.z()+center.squaredNorm();
  if (!initial.allFinite() || radius2 <= 0) return result;
  double radius = std::sqrt(radius2);
  for (int iteration=0; iteration<8; ++iteration) {
    normal.setZero(); rhs.setZero();
    for (const auto & p : xy) {
      const Eigen::Vector2d delta = center-p;
      const double distance = delta.norm();
      if (distance < 1e-8) return result;
      const double error = distance-radius;
      const double weight = std::min(1.0, 0.04/std::max(std::abs(error),1e-9));
      const Eigen::Vector3d row(delta.x()/distance,delta.y()/distance,-1.0);
      normal.noalias() += weight*row*row.transpose(); rhs.noalias() += weight*row*error;
    }
    if (normal.determinant() < 1e-9) return result;
    const Eigen::Vector3d step = normal.ldlt().solve(rhs);
    if (!step.allFinite() || step.norm() > 0.3) return result;
    center -= step.head<2>(); radius -= step.z();
    if (step.norm()<1e-6) break;
  }
  if (radius < 0.12 || radius > 0.40) return result;
  const Eigen::Vector2d absolute = centroid+center;
  Eigen::Vector2d ray = centroid-sensor;
  if (ray.norm()<0.5) return result;
  ray.normalize();
  const double radial = center.dot(ray);
  const double lateral = std::abs(center.x()*ray.y()-center.y()*ray.x());
  if (radial<0.02 || radial>0.35 || lateral>0.10) return result;
  double squared_error=0.0; int inliers=0; std::vector<double> angles;
  const double reference=std::atan2(-ray.y(),-ray.x());
  for (const auto & p : xy) {
    const Eigen::Vector2d delta=p-center;
    const double error=std::abs(delta.norm()-radius);
    squared_error+=error*error;
    if (error<=0.06) {
      ++inliers;
      const double angle=std::atan2(delta.y(),delta.x())-reference;
      angles.push_back(std::atan2(std::sin(angle),std::cos(angle)));
    }
  }
  result.residual=std::sqrt(squared_error/xy.size());
  if (result.residual>0.055 || inliers<0.85*xy.size()) return result;
  std::sort(angles.begin(),angles.end());
  // Require a visible arc spanning at least ~60 degrees. Do not extrapolate
  // a body centre from a tiny patch behind a desk or a nearly planar wall.
  const double span=angles[angles.size()*9/10]-angles[angles.size()/10];
  if (span<1.0 || span>3.5) return result;
  result.valid=true; result.center=absolute; result.radius=radius;
  return result;
}
// With an observed, identity-specific radius, a shorter arc can constrain the
// centre without re-estimating radius from an ill-conditioned surface patch.
inline BodyCenterFit fitBodyCenterWithRadius(
  const std::vector<Eigen::Vector3d> & points, const Eigen::Vector2d & centroid,
  const Eigen::Vector2d & sensor, double radius)
{
  BodyCenterFit result;
  if (points.size()<8 || radius<0.12 || radius>0.40) return result;
  std::vector<double> heights;
  for (const auto & p:points) if(p.allFinite()) heights.push_back(p.z());
  if(heights.size()<8) return result;
  std::sort(heights.begin(),heights.end());
  double low=heights[heights.size()/5],high=heights[heights.size()*4/5];
  if(high-low<0.15) return result;
  Eigen::Vector2d ray=centroid-sensor;
  if(ray.norm()<0.5) return result;
  ray.normalize(); Eigen::Vector2d center=centroid+0.75*radius*ray;
  std::vector<Eigen::Vector2d> xy;
  for(const auto & p:points) if(p.allFinite() && p.z()>=low && p.z()<=high)xy.push_back(p.head<2>());
  if(xy.size()<6)return result;
  for(int iteration=0;iteration<8;++iteration){
    Eigen::Matrix2d normal=Eigen::Matrix2d::Zero();Eigen::Vector2d rhs=Eigen::Vector2d::Zero();
    for(const auto & p:xy){
      Eigen::Vector2d delta=center-p;double distance=delta.norm();if(distance<1e-8)return result;
      const double error=distance-radius,weight=std::min(1.0,0.04/std::max(std::abs(error),1e-9));
      delta/=distance;normal.noalias()+=weight*delta*delta.transpose();rhs.noalias()+=weight*delta*error;
    }
    if(normal.determinant()<1e-6)return result;
    Eigen::Vector2d step=normal.ldlt().solve(rhs);if(!step.allFinite() || step.norm()>.2)return result;
    center-=step;if(step.norm()<1e-6)break;
  }
  const Eigen::Vector2d offset=center-centroid;
  const double radial=offset.dot(ray),lateral=std::abs(offset.x()*ray.y()-offset.y()*ray.x());
  if(radial<.02 || radial>.35 || lateral>.10)return result;
  double error2=0;int inliers=0;
  for(const auto & p:xy){double e=std::abs((p-center).norm()-radius);error2+=e*e;if(e<=.06)++inliers;}
  result.residual=std::sqrt(error2/xy.size());
  if(result.residual>.055 || inliers<.85*xy.size())return result;
  result.valid=true;result.center=center;result.radius=radius;return result;
}
} // namespace person_tracker
