#pragma once
#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>
namespace person_tracker {
// Serialized upstream state gives copied tracking hypotheses independent histories.
class ImmMotFilter {
public:
 static void configure(const std::string & adapter,const std::string & repository,const std::string & dependencies,double position_std=0.16, double acceleration_psd=6.25, double jerk_psd=16.0,
   double turn_acceleration_psd=0.36, double vertical_position_psd=0.01);
 void initialize(const std::string & mode,const Eigen::Vector3d & p,const Eigen::Vector3d & size,double stamp);
 void apply(const std::string & operation,const std::vector<double> & values);
 Eigen::Matrix<double,7,1> state=Eigen::Matrix<double,7,1>::Zero();
 Eigen::Matrix4d covariance=Eigen::Matrix4d::Identity();
 std::array<double,4> probabilities{{0,0,0,1}};
private:
 void unpack(void * result);
 std::string snapshot_;
};
}
