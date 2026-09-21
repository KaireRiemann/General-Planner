#include "elastic_tracking_test_utils.hpp"
#include <general_core/tracking/tracking_frontend.hpp>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <traj_opt/tracking_traj_opt.hpp>
#include <traj_opt/costfunctional/temporalmap/tracking_time_map.hpp>
#include <iostream>
using elastic_test::require;
namespace {
void timeMap() {
    temporal_map::TrackingTimeMap map;
    require(map.configure(3,.01,3.),"invalid time mapping");
    Eigen::Vector3d x(.4,-.5,.8),g(2.,-.8,.4);
    const auto analytic=map.backward(x,g);
    for(int i=0;i<3;++i) {
        auto a=x,b=x;a(i)+=1.e-6;b(i)-=1.e-6;
        const double numeric=(map.decode(a).dot(g)-map.decode(b).dot(g))/2.e-6;
        require(std::abs(numeric-analytic(i))<1.e-7,"elastic duration adjoint mismatch");
    }
    require(std::abs(map.decode(x).sum()-3.16)<1.e-9,"total must be horizon plus squared slack");
    require((map.decode(map.encode(map.decode(x)))-map.decode(x)).norm()<1.e-9,"time map round trip");
    x(0)=4.;require(map.decode(x).sum()>18.,"elastic catch-up time was artificially capped");
}
void checkTrajectory(const traj_opt::TrackingProblem &problem,const geometry_utils::Trajectory &position,
                     const geometry_utils::Trajectory &yaw) {
    require(!position.empty() && !yaw.empty(),"missing position/yaw output");
    require((position.getState(0.).leftCols<3>()-problem.head_pvaj.leftCols<3>()).norm()<1.e-7,"moving PVA head changed");
    require((position.getState(position.getTotalDuration()).leftCols<3>()-problem.tail_pvaj.leftCols<3>()).norm()<1.e-6,
            "fixed terminal PVA differs from Elastic-Tracker");
    require(position.getPieceNum()==4,"one corridor must produce four pieces as in upstream");
    require(position.getMaxVelRate()<=5.25 && position.getMaxAccRate()<=4.2,"dynamics bound violated");
    require(std::abs(position.getTotalDuration()-yaw.getTotalDuration())<1.e-8,"yaw clock differs from position");
    require(yaw.getMaxVelRate()<=problem.max_yaw_rate+.003,"yaw rate exceeds bound");
    require(std::abs(yaw.getPos(0.).x()-problem.head_yaw(0,0))<1.e-8 &&
            std::abs(yaw.getVel(0.).x()-problem.head_yaw(0,1))<1.e-8 &&
            std::abs(yaw.getAcc(0.).x()-problem.head_yaw_acceleration)<1.e-8,
            "yaw PVA head changed");
    require(yaw.getMaxAccRate()<=problem.max_yaw_acceleration+.003,"yaw acceleration exceeds bound");
    for(int i=1;i<yaw.getPieceNum();++i) {
        const auto &a=yaw[i-1],&b=yaw[i];
        require((a.getPos(a.getDuration())-b.getPos(0.)).norm()<1.e-8 &&
                (a.getVel(a.getDuration())-b.getVel(0.)).norm()<1.e-8 &&
                (a.getAcc(a.getDuration())-b.getAcc(0.)).norm()<1.e-8,"C2 yaw join broken");
    }
    for(int i=1;i<position.getPieceNum();++i) {
        const auto &a=position[i-1],&b=position[i];
        require((a.getPos(a.getDuration())-b.getPos(0.)).norm()<1.e-7 &&
                (a.getVel(a.getDuration())-b.getVel(0.)).norm()<1.e-7 &&
                (a.getAcc(a.getDuration())-b.getAcc(0.)).norm()<1.e-6,"C2 position join broken");
    }
}
void closedLoop(bool turn) {
    general_planner::TrackingFrontend::Config fc;
    fc.search_budget_seconds=.5;
    general_planner::TrackingFrontend front(fc,nullptr);
    traj_opt::TrackingJerkTrajOpt solver(elastic_test::config(),nullptr);
    general_utils::StatePVAJ head=general_utils::StatePVAJ::Zero();head.col(0)=Eigen::Vector3d(-6.,0.,1.5);
    Eigen::Vector2d yh=Eigen::Vector2d::Zero();double ya=0.;
    double error=0.,maximum_solve_ms=0.;
    for(int k=0;k<35;++k) {
        const double now=.2*k;
        traj_opt::TrackingProblem problem;
        require(front.buildProblem(head,elastic_test::prediction(100.+now,now,turn),problem),"visible ring search failed");
        elastic_test::addFreeCorridor(problem);problem.head_yaw=yh.transpose();problem.head_yaw_acceleration=ya;
        traj_opt::TrackingSolveReport report;problem.solve_report=&report;
        geometry_utils::Trajectory position,yaw;std::string why;
        const bool optimized=solver.optimize(problem,position,&yaw,&why);
        require(optimized,"closed-loop solve failed at "+std::to_string(k)+": "+why+";status="+std::to_string(report.status)+";iterations="+std::to_string(report.iterations)+";cost="+std::to_string(report.cost)+";gradient="+std::to_string(report.gradient_inf));
        maximum_solve_ms=std::max(maximum_solve_ms,report.elapsed_ms);
        checkTrajectory(problem,position,yaw);
        head=position.getState(.2);const auto ys=yaw.getState(.2);yh=ys.row(0).head<2>();ya=ys(0,2);
        const auto target=elastic_test::prediction(100.+now+.2,now+.2,turn).front();
        error=std::abs((head.col(0)-target.position).head<2>().norm()-fc.tracking_distance);
    }
    require(error<1.0,turn?"turning target did not settle into observation ring":"straight target did not converge");
    std::cout<<(turn?"turn":"straight")<<" final distance error="<<error<<" max solve ms="<<maximum_solve_ms<<'\n';
}
void invalidInputsAndHover() {
    general_planner::TrackingFrontend front({},nullptr);
    general_utils::StatePVAJ head=general_utils::StatePVAJ::Zero();head.col(0)=Eigen::Vector3d(-3.,0.,1.5);
    auto target=elastic_test::prediction();for(auto &s:target){s.position={0.,0.,.5};s.velocity.setZero();}
    traj_opt::TrackingProblem problem;
    require(front.buildProblem(head,target,problem),"stationary ring unavailable");
    elastic_test::addFreeCorridor(problem);geometry_utils::Trajectory position,yaw;std::string why;
    traj_opt::TrackingJerkTrajOpt solver(elastic_test::config(),nullptr);
    const bool optimized=solver.optimize(problem,position,&yaw,&why);
        require(optimized,"stationary target failed: "+why);
    require((position.getPos(.5)-head.col(0)).norm()<.02,"stationary target fabricated motion");
    target[2].t=target[1].t;
    require(!front.buildProblem(head,target,problem),"duplicate prediction timestamps accepted");
    require(general_planner::trackingPredictionAtTime(elastic_test::prediction(),104.).empty(),"expired input lifetime restarted");
}
}
int main() {
    try {timeMap();invalidInputsAndHover();closedLoop(false);closedLoop(true);
        std::cout<<"tracking_elastic_self_test PASS\n";return 0;}
    catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
