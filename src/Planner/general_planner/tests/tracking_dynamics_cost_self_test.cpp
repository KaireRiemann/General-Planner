#include "elastic_tracking_test_utils.hpp"
#include <traj_opt/costfunctional/spatialcosts/elastic_tracking_penalty.hpp>
#include <traj_opt/tracking_objective.hpp>
#include <iostream>
using elastic_test::require;
namespace {
template<class Function> void gradientCheck(const Eigen::Vector3d &point,Function function) {
    Eigen::Vector3d gradient=Eigen::Vector3d::Zero();
    const double value=function(point,gradient);
    require(std::isfinite(value)&&gradient.allFinite(),"non-finite spatial cost");
    for(int axis=0;axis<3;++axis) {
        Eigen::Vector3d a=point,b=point,scratch=Eigen::Vector3d::Zero();
        a(axis)+=1.e-6;b(axis)-=1.e-6;
        const double numeric=(function(a,scratch)-function(b,scratch))/2.e-6;
        require(std::abs(numeric-gradient(axis))<1.e-4*(1.+std::abs(numeric)),"elastic spatial gradient mismatch");
    }
}
void spatialCosts() {
    const auto distance=[](const Eigen::Vector3d &p,Eigen::Vector3d &g) {
        return cost_functional::elasticTrackingDistance(p,{0,0,1.5},3.,.3,.2,2.,g);
    };
    for(const Eigen::Vector3d p:{Eigen::Vector3d(1.,.1,1.5),Eigen::Vector3d(4.,.3,2.),
                               Eigen::Vector3d(3.31,0,1.5),Eigen::Vector3d(3.,0.,1.8)}) gradientCheck(p,distance);
    Eigen::Vector3d g=Eigen::Vector3d::Zero();
    require(distance({3.,0,1.5},g)==0. && g.norm()==0.,"observation tolerance band must have zero cost");
    g.setZero();
    const double expected=2.*std::pow(2.7*2.7-1.,3);
    require(std::abs(distance({1.,0,1.5},g)-expected)<1.e-9,"near-distance penalty differs from upstream formula");
    gradientCheck({.1,2.,1.8},[](const Eigen::Vector3d &p,Eigen::Vector3d &g) {
        return cost_functional::elasticVisibility(p,{0,0,1.5},{3,0,1.5},.7,.2,10.,g);
    });
}
void objectiveGradient() {
    auto cfg=elastic_test::config();
    traj_opt::TrackingProblem problem;
    problem.target_prediction=elastic_test::prediction();
    problem.head_pvaj.col(0)=Eigen::Vector3d(-4.,.4,1.5);problem.head_pvaj.col(1)=Eigen::Vector3d(.4,.2,0.);
    problem.tail_pvaj.col(0)=Eigen::Vector3d(2.,.1,1.5);problem.tail_pvaj.col(1)=Eigen::Vector3d(1.5,0,0);
    problem.min_total_duration=3.;problem.height_offset=1.;
    for(const auto &target:problem.target_prediction) {
        traj_opt::TrackingVisibleRegion region;
        region.valid=true;region.t=target.t;region.target_position=target.position+Eigen::Vector3d(0,0,1.);
        region.visible_point=region.target_position+Eigen::Vector3d(-3,0,0);region.theta=.8;
        problem.visible_regions.push_back(region);
    }
    spatial_map::PolyhedraH corridors{elastic_test::box({-6,-2,1.},{1,2,2.}).GetPlanes(),
                                     elastic_test::box({-2,-2,1.},{4,2,2.}).GetPlanes()};
    traj_opt::TrackingObjective objective(cfg,problem,corridors);
    traj_opt::TrackingObjective::Optimizer::WaypointsType points(5,3);
    points << -4,.4,1.5,-2.3,.5,1.6,-1.,.3,1.4,.4,.2,1.6,2.,.1,1.5;
    require(objective.initialize(problem,{.81,.93,1.07,.79},points),"objective init failed");
    Eigen::VectorXd x=objective.initialGuess(),analytic(x.size());
    require(std::isfinite(objective.evaluate(x,analytic)),"objective not finite");
    double worst=0.;
    for(int i=0;i<x.size();++i) {
        auto a=x,b=x;Eigen::VectorXd scratch(x.size());a(i)+=1.e-6;b(i)-=1.e-6;
        const double numeric=(objective.evaluate(a,scratch)-objective.evaluate(b,scratch))/2.e-6;
        const double relative=std::abs(numeric-analytic(i))/(1.+std::abs(numeric)+std::abs(analytic(i)));
        worst=std::max(worst,relative);
        require(relative<2.e-5,"MINCO absolute-time/corridor adjoint mismatch at "+std::to_string(i));
    }
    std::cout<<"Elastic objective finite-difference maximum relative error: "<<worst<<'\n';
}
}
int main() {
    try {spatialCosts();objectiveGradient();std::cout<<"tracking_dynamics_cost_self_test PASS\n";return 0;}
    catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
