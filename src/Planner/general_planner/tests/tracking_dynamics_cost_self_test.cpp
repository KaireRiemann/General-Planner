#include <iostream>
#include <traj_opt/costfunctional_manager/tracking_cost_manager.hpp>
#include <cmath>
int main() {
  traj_opt::Config cfg;
  cfg.max_vel=3.; cfg.max_acc=3.; cfg.max_jerk=12.; cfg.max_tilt=.2;
  cfg.penna_vel=2.; cfg.penna_acc=3.; cfg.penna_jerk=4.;
  cfg.penna_pos=0.; cfg.smooth_eps=.01;
  cost_functional_manager::TrackingCostManager manager;
  manager.reset(cfg, nullptr, traj_opt::TrackingProblem(), nullptr);
  auto evaluate=[&](Eigen::Vector3d a,Eigen::Vector3d j,Eigen::Vector3d &ga,Eigen::Vector3d &gj) {
    Eigen::Vector3d gp=Eigen::Vector3d::Zero(),gv=gp;
    ga.setZero(); gj.setZero(); double gt=0.;
    return manager.evaluateIntegral(0,0,0,0,0,Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),a,j,gp,gv,ga,gj,gt);
  };
  Eigen::Vector3d a(3.2,.4,-.5), j(14.,1.,.5),ga,gj;
  const double value=evaluate(a,j,ga,gj);
  if (!(value>0. && gj.norm()>0.)) return 1;
  for (int axis=0;axis<6;++axis) {
    auto ap=a,am=a,jp=j,jm=j;
    if(axis<3){ap(axis)+=1.e-6;am(axis)-=1.e-6;}
    else {jp(axis-3)+=1.e-6;jm(axis-3)-=1.e-6;}
    Eigen::Vector3d g1,g2;
    double numeric=(evaluate(ap,jp,g1,g2)-evaluate(am,jm,g1,g2))/2.e-6;
    double analytic=axis<3?ga(axis):gj(axis-3);
    if(std::abs(numeric-analytic)>1.e-4*std::max(1.,std::abs(numeric))) {
      std::cerr<<"gradient mismatch "<<axis<<" "<<numeric<<" "<<analytic<<"\n"; return 2;
    }
  }
  if(evaluate(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),ga,gj)!=0.)return 3;
  std::cout<<"tracking jerk/tilt gradients and feasible zero cost passed\n";
}
