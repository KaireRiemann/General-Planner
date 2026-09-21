#pragma once
#include <traj_opt/tracking_problem.hpp>
#include <traj_opt/config.hpp>
#include <stdexcept>
#include <string>
namespace elastic_test {
inline void require(bool condition,const std::string &message) {
    if(!condition) throw std::runtime_error(message);
}
inline traj_opt::Config config() {
    traj_opt::Config cfg;
    cfg.max_vel=5.;cfg.max_acc=4.;cfg.penna_t=100.;
    cfg.penna_pos=10000.;cfg.penna_vel=10000.;cfg.penna_acc=10000.;
    cfg.integral_reso=16;cfg.opt_accuracy=1.e-5;
    return cfg;
}
inline geometry_utils::Polytope box(const Eigen::Vector3d &lower,const Eigen::Vector3d &upper) {
    general_utils::MatD4f planes=general_utils::MatD4f::Zero(6,4);
    for(int axis=0;axis<3;++axis) {
        planes(2*axis,axis)=1.;planes(2*axis,3)=-upper(axis);
        planes(2*axis+1,axis)=-1.;planes(2*axis+1,3)=lower(axis);
    }
    return geometry_utils::Polytope(planes);
}
inline traj_opt::DynamicTargetStates prediction(double epoch=100.,double elapsed=0.,bool turn=false) {
    traj_opt::DynamicTargetStates out;
    for(int i=0;i<=15;++i) {
        traj_opt::DynamicTargetState state;
        state.t=.2*i;state.reference_time=epoch;
        const double t=elapsed+state.t;
        if(turn && t>3.) {
            state.position={4.5,1.5*(t-3.),.5};state.velocity={0,1.5,0};
        } else {
            state.position={1.5*t,0,.5};state.velocity={1.5,0,0};
        }
        out.push_back(state);
    }
    return out;
}
inline void addFreeCorridor(traj_opt::TrackingProblem &problem) {
    problem.sfcs={box({-30.,-30.,.375},{30.,30.,5.})};
    problem.use_corridor=true;
    problem.solve_budget_seconds=0.10;problem.max_iterations=400;
}
} // namespace elastic_test
