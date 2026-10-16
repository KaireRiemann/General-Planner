#include <general_core/exploration/highspeed/epicon_corridor.h>
#include <traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp>
#include <iostream>
#include <stdexcept>
#include <limits>

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
    const auto unbounded=corridor.planes;
    std::vector<Vector3d> box_lo{{-.5,-.9,.7},{1.3,-.9,.7}};
    const std::vector<Vector3d> box_hi{{1.7,.9,2.3},{3.5,.9,2.3}};
    require(fast_planner::clipEpiconCorridorToBoxes(path.front(),path.back(),box_lo,box_hi,corridor.planes),
            "one convex corridor could not cross overlapping task boxes");
    require(corridor.planes.size()>=2,"cross-box corridor was not partitioned");
    for(const auto &poly:corridor.planes)
      require((poly.leftCols<3>()*Vector3d(1.5,1.1,1.5)+poly.col(3)).maxCoeff()>0,
              "clipped corridor admitted a point outside both task boxes");
    box_lo[1].x()=1.8; corridor.planes=unbounded;
    require(!fast_planner::clipEpiconCorridorToBoxes(path.front(),path.back(),box_lo,box_hi,corridor.planes),
            "disconnected task-box corridor was accepted");
    std::vector<Vector3d> walls;
    for(double x=-2;x<=5;x+=.2) for(double z=0;z<=3;z+=.2) {
      walls.emplace_back(x,-.8,z); walls.emplace_back(x,.8,z);
    }
    require(fast_planner::buildEpiconCorridor(path,walls,lower,upper,3.5,.6,corridor),
            "point-cloud corridor wider than robot was rejected");
    for(auto &p:walls) p.y()=p.y()>0 ? .4 : -.4;
    require(!fast_planner::buildEpiconCorridor(path,walls,lower,upper,3.5,.6,corridor),
            "sub-clearance corridor accepted");
    // A preferred but incomplete corridor cannot hide a complete hard-safe one.
    for(int scenario=0;scenario<5;++scenario) {
      int calls=0; bool used_hard=false;
      const bool ok=general_planner::selectCorridor(.6,.505,true,
          [&](double margin,general_planner::CorridorResult &out) {
            ++calls; out.clearance=margin;
            if(margin>.55) { out.truncated=scenario!=0; return scenario!=4; }
            out.truncated=scenario==3;
            return scenario!=2 && scenario!=4;
          },corridor,used_hard);
      if(scenario==0) require(ok && calls==1 && !used_hard,"full soft corridor retried");
      if(scenario==1) require(ok && calls==2 && used_hard && !corridor.truncated,
                             "soft prefix suppressed full hard corridor");
      if(scenario==2 || scenario==3)
        require(ok && !used_hard && corridor.clearance==.6 && corridor.truncated,
                "hard retry discarded a usable soft prefix");
      if(scenario==4) require(!ok && !used_hard,"two failures reported success");
    }
    for(double margin:{.48,.5,.505,.55,.6,.65}) {
      std::vector<Vector3d> passage;
      for(double x=-2;x<=5;x+=.1) for(double z=0;z<=3;z+=.1) {
        passage.emplace_back(x,-.8,z);passage.emplace_back(x,.8,z);
      }
      require(fast_planner::buildEpiconCorridor(path,passage,lower,upper,3.5,margin,corridor),
              "feasible passage failed clearance perturbation");
      require(corridor.backend=="firi" && corridor.clearance==margin,
              "corridor lost backend/clearance contract");
    }
    require(!fast_planner::buildEpiconCorridor(path,{},lower,upper,3.5,
        std::numeric_limits<double>::quiet_NaN(),corridor),"NaN clearance accepted");
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
    std::cout<<"EPICON_CORRIDOR PASS: empty crop, vertical bounds, overlapping/disconnected boxes, wide/narrow passage, recovery, temporal gradient\n";
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<'\n'; return 1; }
}
