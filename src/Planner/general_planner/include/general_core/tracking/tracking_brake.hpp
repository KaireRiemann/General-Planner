#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
namespace general_planner {
// C3-continuous stop. Coefficients are descending powers of physical time.
inline Eigen::Matrix<double,3,8> trackingBrakeCoefficients(
    const Eigen::Matrix<double,3,4>& start, double duration) {
  if (!start.allFinite() || !std::isfinite(duration) || duration <= 0.0)
    throw std::invalid_argument("invalid brake boundary");
  const double T = duration;
  Eigen::Matrix<double,3,8> c = Eigen::Matrix<double,3,8>::Zero();
  c.col(0)=start.col(0); c.col(1)=start.col(1)*T;
  c.col(2)=start.col(2)*T*T/2; c.col(3)=start.col(3)*T*T*T/6;
  Eigen::Matrix4d A;
  A << 1,1,1,1, 4,5,6,7, 12,20,30,42, 24,60,120,210;
  Eigen::Matrix<double,4,3> rhs;
  const Eigen::Vector3d end = start.col(0)+start.col(1)*T/2+start.col(2)*T*T/12;
  rhs.row(0)=(end-c.leftCols<4>().rowwise().sum()).transpose();
  rhs.row(1)=(-c.col(1)-2*c.col(2)-3*c.col(3)).transpose();
  rhs.row(2)=(-2*c.col(2)-6*c.col(3)).transpose();
  rhs.row(3)=(-6*c.col(3)).transpose();
  c.rightCols<4>()=A.fullPivLu().solve(rhs).transpose();
  Eigen::Matrix<double,3,8> descending;
  for(int i=0;i<8;++i) descending.col(7-i)=c.col(i)/std::pow(T,i);
  return descending;
}
}
