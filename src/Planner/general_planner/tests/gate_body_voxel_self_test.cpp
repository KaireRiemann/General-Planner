#include <general_core/gate/body_voxel_collision.hpp>
#include <iostream>
#include <general_core/gate/tracking_alignment.hpp>
#include <stdexcept>
using namespace general_planner::gate;
void expect(bool b,const char *why) { if(!b) throw std::runtime_error(why); }
int main() {
 auto line=[](double t) { return Eigen::Vector3d(t,0,1-.3*t); };
 const double matched=matchFeedbackTime(line(2.08),2.2,.04,.2,5.,line);
 expect(std::abs(matched-2.08)<.0021,"delayed sample not aligned");
 expect(matchFeedbackTime(line(0),2.2,.04,.2,5.,line)>=1.96-1e-9,"unbounded stale matching");
 const Eigen::Vector3d displaced=line(2.08)+Eigen::Vector3d(0,.06,0);
 const double shifted=matchFeedbackTime(displaced,2.2,.04,.2,5.,line);
 expect((line(shifted)-displaced).norm()>=.059,"real cross-track error hidden");
 expect(std::abs(matchFeedbackTime(line(2.08),2.2,.04,0,5.,line)-2.16)<1e-9,"zero window ignored");
 const Eigen::Vector3d p(1.875,-.3,.9),r(.165,.165,.1);
 const Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
 // 15cm voxels centered on the actual Unity lower/upper frame. The gap is
 // 30cm; a 20cm body plus 3cm margin on each side fits when correctly aligned.
 expect(!bodyIntersectsVoxel(p,I,r,{1.875,-.3,.675},.075+.03),"narrow frame falsely blocked");
 expect(!bodyIntersectsVoxel(p,I,r,{1.875,-.3,1.125},.075+.03),"upper frame falsely blocked");
 expect(bodyIntersectsVoxel(p,I,r,{1.875,-.3,.72},.105),"real collision missed");
 expect(bodyIntersectsVoxel(p,I,r,{1.875,-.3,.695},.105),"tangency missed");
 Eigen::Matrix3d roll=Eigen::AngleAxisd(1.57079632679,Eigen::Vector3d::UnitX()).toRotationMatrix();
 expect(bodyIntersectsVoxel(p,roll,r,{1.875,-.3,.675},.105),"rotated body collision missed");
 expect(bodyIntersectsVoxel(p,I,r,p,.001),"contained voxel missed");
 expect(bodyIntersectsVoxel(p,I,r,p,1),"contained body missed");
 expect(!bodyIntersectsVoxel(p,I,r,{3,3,3},.075),"distant cell hit");
 // Rotation covariance around world Z preserves the AABB under 90 degrees.
 Eigen::Matrix3d rz=Eigen::AngleAxisd(1.5707963267948966,Eigen::Vector3d::UnitZ()).toRotationMatrix();
 for(int i=0;i<200;++i) {
   Eigen::Vector3d c=p+Eigen::Vector3d::Random()*.35;
   expect(bodyIntersectsVoxel(p,roll,r,c,.075)==bodyIntersectsVoxel(rz*p,rz*roll,r,rz*c,.075),"rotation covariance");
 }
 for(int i=0;i<500;++i) {
   Eigen::Vector3d c=Eigen::Vector3d::Random()*.4;
   const double d2=(c.cwiseAbs().array()-.075).max(0.).square().sum();
   expect(bodyIntersectsVoxel(Eigen::Vector3d::Zero(),roll,Eigen::Vector3d::Constant(.2),c,.075)==(d2<=.04+1e-10),"sphere/AABB analytic oracle");
 }
 std::cout<<"gate voxel geometry: narrow gap, tangency, rotation, containment passed\n";
}
