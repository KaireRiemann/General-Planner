#include <general_core/exploration/highspeed/epicon_corridor.h>
#include <traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp>
#include <iostream>
#include <stdexcept>

static void require(bool condition,const char *message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  try {
    using Eigen::Vector3d;
    fast_planner::EpiconCorridor corridor;
    const std::vector<Vector3d> path{{0,0,1.5},{1,0,1.5},{2,0,1.5},{3,0,1.5}};
    const Vector3d lower(-3,-3,.5),upper(6,3,2.5);
    require(fast_planner::buildEpiconCorridor(path,{},lower,upper,3.5,.6,corridor),
            "empty point-cloud crop failed or accessed an empty array");
    require(corridor.planes.size()>=2,"single corridor not made optimizable");
    for (const auto &poly:corridor.planes) {
      require(poly.allFinite(),"non-finite FIRI result");
      require((poly.leftCols<3>()*Vector3d(1,0,4.5)+poly.col(3)).maxCoeff()>0,
              "low guide admitted the recorded 4.5 m excursion");
    }
    std::vector<Vector3d> walls;
    for(double x=-2;x<=5;x+=.2) for(double z=0;z<=3;z+=.2) {
      walls.emplace_back(x,-.8,z); walls.emplace_back(x,.8,z);
    }
    require(fast_planner::buildEpiconCorridor(path,walls,lower,upper,3.5,.6,corridor),
            "point-cloud corridor wider than robot was rejected");
    for(auto &p:walls) p.y()=p.y()>0 ? .4 : -.4;
    require(!fast_planner::buildEpiconCorridor(path,walls,lower,upper,3.5,.6,corridor),
            "sub-clearance corridor accepted");
    Vector3d recovery;
    require(!fast_planner::epiconRecoveryPoint(Vector3d(0,0,1.5),{},recovery),
            "empty obstacle crop created an arbitrary upward recovery");
    cost_functional::LinearTimeCost time_cost;
    time_cost.weight=2000; time_cost.reference_duration=2;
    for (double total:{1.0,1.93,2.0,2.08,3.0}) {
      Eigen::VectorXd gradient=Eigen::VectorXd::Zero(2),scratch(2);
      const std::vector<double> times{total*.4,total*.6};
      time_cost(times,gradient);
      for(int j=0;j<2;++j) {
        auto hi=times,lo=times; hi[j]+=1e-6; lo[j]-=1e-6;
        scratch.setZero(); const double fhi=time_cost(hi,scratch);
        scratch.setZero(); const double flo=time_cost(lo,scratch);
        require(std::abs((fhi-flo)/2e-6-gradient(j))<1e-3,"yaw time cost gradient mismatch");
      }
    }
    std::cout<<"EPICON_CORRIDOR PASS: empty crop, vertical bounds, wide/narrow passage, recovery, temporal gradient\n";
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<'\n'; return 1; }
}
