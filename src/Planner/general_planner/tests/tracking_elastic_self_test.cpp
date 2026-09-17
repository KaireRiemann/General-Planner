#include <general_core/tracking/tracking_perching_frontend.hpp>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <traj_opt/tracking_objective.hpp>
#include <traj_opt/tracking_traj_opt.hpp>
#include <iostream>
#include <stdexcept>
#include <chrono>

namespace {
void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}

traj_opt::Config config() {
    traj_opt::Config c;
    c.max_vel=5; c.max_acc=4; c.max_jerk=30; c.max_omg=3; c.max_tilt=.5;
    c.max_acc_thr=20; c.min_acc_thr=3; c.grav=9.81; c.mass=1;
    c.dh=0; c.dv=0; c.cp=0; c.v_eps=.01;
    c.penna_t=1; c.penna_pos=10000; c.penna_vel=100; c.penna_acc=100;
    c.penna_jerk=10; c.penna_omg=100; c.penna_thr=100;
    c.smooth_eps=.01; c.integral_reso=10; c.opt_accuracy=1.e-6;
    c.piece_num=2;
    return c;
}

traj_opt::DynamicTargetStates prediction(double horizon=.75) {
    traj_opt::DynamicTargetStates out;
    for (int i=0;i<=3;++i) {
        traj_opt::DynamicTargetState s;
        s.t=horizon*i/3.; s.reference_time=100.;
        s.position << 2*s.t,.15*s.t, .8;
        s.velocity << 2,.15,0;
        out.push_back(s);
    }
    return out;
}

void timeMap() {
    temporal_map::TrackingTimeMap map;
    require(map.configure(3,.1,2.,4.),"time map configuration");
    Eigen::Vector3d x(.3,-.5,.8), g(2.,-.8,.4);
    const Eigen::VectorXd analytic=map.backward(x,g);
    for (int i=0;i<3;++i) {
        auto a=x,b=x; a(i)+=1.e-6; b(i)-=1.e-6;
        const double numeric=(map.decode(a).dot(g)-map.decode(b).dot(g))/2.e-6;
        require(std::abs(numeric-analytic(i))<1.e-7,"coupled time gradient");
    }
    for (double v : {-900.,900.}) {
        const Eigen::VectorXd t=map.decode(Eigen::Vector3d(v,-v,v));
        require(t.allFinite() && t.minCoeff()>=.1-1.e-12 && t.sum()>=2.-1.e-12 && t.sum()<=4.+1.e-12,
                "time bounds must hold under extreme line-search coordinates");
    }
}

traj_opt::TrackingProblem frontend() {
    general_planner::TrackingFrontend::Config cfg;
    cfg.tracking_distance=5.; cfg.height_offset=.7;
    cfg.search_budget_seconds=.2; cfg.max_speed=5.; cfg.max_acc=3.;
    general_planner::TrackingFrontend front(cfg,nullptr);
    general_utils::StatePVAJ head=general_utils::StatePVAJ::Zero();
    head.col(0)<<-5.,0.,1.5; head.col(1)<<1.,0.,0.;
    head.col(2)<<.1,0.,0.; head.col(3)<<.05,0.,0.;
    auto input=prediction();
    const auto rebased=general_planner::trackingPredictionAtTime(input,100.25);
    require(rebased.size()>=2 && std::abs(rebased.back().t-.5)<1.e-9,"prediction rebase");
    require(general_planner::trackingPredictionAtTime(rebased,100.76).empty(),"rebase cannot renew prediction lifetime");
    traj_opt::TrackingProblem problem;
    require(front.buildProblem(head,rebased,problem),"open-space frontend");
    require((problem.head_pvaj-head).norm()<1.e-12,"moving head preserved");
    require(problem.tail_pvaj(0,1)>1.5,"moving terminal velocity preserved");
    require(problem.tail_pvaj(0,0)>head(0,0)+2.,"guide follows target motion");
    require(problem.trusted_horizon==.5 && problem.target_prediction.back().t<=2.5+1.e-9,
            "explicit bounded prediction extension");
    require(std::abs(problem.target_prediction.back().reference_time-100.25)<1.e-9,"prediction epoch preserved");
    input[2].t=input[1].t;
    traj_opt::TrackingProblem invalid;
    require(!front.buildProblem(head,input,invalid),"nonmonotonic prediction rejected");
    return problem;
}

template<int S> void gradient() {
    auto cfg=config();
    traj_opt::TrackingProblem p;
    p.head_pvaj.col(0)<<-5.,.2,1.5; p.head_pvaj.col(1)<<1.,.2,0.;
    p.tail_pvaj.col(0)<<-.4,.6,1.5; p.tail_pvaj.col(1)<<2.,.15,0.;
    p.head_yaw<<.25,.1; p.head_yaw_acceleration=.04;
    p.tracking_distance=4.5; p.od_h_lower=4.; p.od_h_upper=5.;
    p.target_prediction=prediction(2.4); p.min_total_duration=2.4; p.max_total_duration=4.;
    p.trusted_horizon=.8; p.target_sample_times={.23,.81,1.57,2.21};
    p.dense_joint_sample_enable=false;
    p.use_visible_region=false; p.use_esdf_visibility=false;
    p.weight_fov=20.;
    p.fov_horizontal=.7; p.fov_vertical=.6;
    p.weight_relative_velocity=5.; p.weight_tangent_velocity=3.;
    std::vector<double> times{1.5,1.7};
    typename traj_opt::TrackingObjective<S>::PosOptimizer::WaypointsType points(3,3);
    points<<-5.,.2,1.5,-2.8,.8,1.8,-.4,.6,1.5;
    typename traj_opt::TrackingObjective<S>::YawOptimizer::WaypointsType yaws(3,1);
    yaws<<.25,.4,.1;
    typename traj_opt::TrackingObjective<S>::YawTraj::BoundaryState yh,yt;
    yh<<.25,.1,.04; yt<<.1,.05,0.;
    traj_opt::TrackingObjective<S> objective(cfg,p,nullptr,{});
    require(objective.initialize(p,times,points,yaws,yh,yt,nullptr),"joint objective initialization");
    const auto x=objective.initialGuess(); Eigen::VectorXd g(x.size()),scratch(x.size());
    const double value=objective.evaluate(x,g);
    require(std::isfinite(value)&&g.allFinite(),"finite joint objective");
    for (int i=0;i<x.size();++i) {
        auto a=x,b=x; const double h=1.e-5; a(i)+=h; b(i)-=h;
        const double numeric=(objective.evaluate(a,scratch)-objective.evaluate(b,scratch))/(2*h);
        const double tolerance=3.e-4*std::max({1.,std::abs(numeric),std::abs(g(i))});
        if (std::abs(numeric-g(i))>tolerance) {
            std::cerr<<"S="<<S<<" gradient "<<i<<" numeric="<<numeric<<" analytic="<<g(i)<<"\n";
            throw std::runtime_error("position/yaw/shared-time gradient mismatch");
        }
    }
}

void optimize(traj_opt::TrackingProblem p) {
    auto cfg=config();
    p.solve_budget_seconds=2.; p.max_iterations=150;
    p.head_yaw<<0.,0.;
    p.weight_fov=20.; p.use_esdf_visibility=false;
    Eigen::Matrix<double,6,4> planes;
    planes << 1,0,0,-20, -1,0,0,-20, 0,1,0,-10, 0,-1,0,-10, 0,0,1,-8, 0,0,-1,0;
    geometry_utils::Polytope poly; poly.SetPlanes(planes); p.sfcs={poly}; p.use_corridor=true;
    traj_opt::TrackingSnapTrajOpt solver(cfg,nullptr);
    geometry_utils::Trajectory pos,yaw; std::string reason;
    const bool ok=solver.optimize(p,pos,&yaw,&reason);
    if (!ok) std::cerr<<reason<<"\n";
    require(ok,"corridor tracking solve");
    require((pos.getState(0.)-p.head_pvaj).norm()<1.e-7,"solver preserves PVAJ splice");
    require((pos.getVel(pos.getTotalDuration())-p.tail_pvaj.col(1)).norm()<1.e-6,"solver preserves moving terminal");
    require(pos.getPos(1.).x()>pos.getPos(0.).x()+.7,"straight tracking makes forward progress");
    require(std::abs(pos.getTotalDuration()-yaw.getTotalDuration())<1.e-10,"shared position/yaw duration");
    require(pos.getTotalDuration()<=p.max_total_duration+1.e-9,"bounded solve duration");
    p.should_stop=[] {return true;};
    require(!solver.optimize(p,pos,&yaw,&reason)&&pos.empty()&&yaw.empty(),"expired solve returns no command");
}

void rollingStraight(double initial_distance) {
    const general_planner::Config cfg(
        std::string(ROOT_DIR) + "config/task_planner_runtime_state2state.yaml",
        std::string(ROOT_DIR) + "config/task_planner_runtime_tracking.yaml");
    general_planner::TrackingFrontend::Config fc;
    fc.tracking_distance=cfg.tracking_distance;
    fc.distance_lower_tolerance=cfg.tracking_distance_lower_tolerance;
    fc.distance_upper_tolerance=cfg.tracking_distance_upper_tolerance;
    fc.height_offset=cfg.tracking_height_offset; fc.height_tolerance=cfg.tracking_height_tolerance;
    fc.max_speed=cfg.tracking_traj_cfg.max_vel; fc.max_acc=cfg.tracking_traj_cfg.max_acc;
    fc.max_jerk=cfg.tracking_traj_cfg.max_jerk;
    fc.nominal_horizon=cfg.tracking_nominal_horizon; fc.max_extrapolation=cfg.tracking_max_extrapolation;
    general_planner::TrackingFrontend front(fc,nullptr);
    traj_opt::TrackingSnapTrajOpt solver(cfg.tracking_traj_cfg,nullptr);
    general_utils::StatePVAJ head=general_utils::StatePVAJ::Zero();
    head.col(0)<<-initial_distance,0.,.8+fc.height_offset;
    Eigen::Vector3d yaw_state=Eigen::Vector3d::Zero();
    constexpr double dt=.2;
    std::vector<double> solve_ms;
    for (int step=0;step<60;++step) {
        traj_opt::DynamicTargetStates targets;
        for (int i=0;i<=2;++i) {
            traj_opt::DynamicTargetState target;
            target.t=.25*i; target.reference_time=100.+step*dt;
            target.position<<2.*(step*dt+target.t),0.,.8;
            target.velocity<<2.,0.,0.; targets.push_back(target);
        }
        traj_opt::TrackingProblem p;
        require(front.buildProblem(head,targets,p),"rolling frontend");
        p.head_yaw<<yaw_state.x(),yaw_state.y(); p.head_yaw_acceleration=yaw_state.z();
        p.max_yaw_rate=cfg.tracking_yaw_rate_limit; p.max_yaw_acceleration=cfg.tracking_yaw_acceleration_limit;
        p.weight_od_near=cfg.tracking_weight_od_near; p.weight_od_far=cfg.tracking_weight_od_far;
        p.weight_od_vertical=cfg.tracking_weight_od_vertical; p.weight_oa=cfg.tracking_weight_oa;
        p.weight_oe=cfg.tracking_weight_oe; p.weight_relative_velocity=cfg.tracking_weight_relative_velocity;
        p.weight_tangent_velocity=cfg.tracking_weight_tangent_velocity;
        p.weight_visible_region=cfg.tracking_weight_visible_region; p.weight_fov=cfg.tracking_weight_fov;
        p.use_esdf_visibility=false; p.solve_budget_seconds=cfg.tracking_solver_budget;
        p.max_iterations=cfg.tracking_solver_iterations;
        p.fov_horizontal=cfg.tracking_fov_horizontal_deg*M_PI/180.;
        p.fov_vertical=cfg.tracking_fov_vertical_deg*M_PI/180.;
        p.target_half_height=cfg.tracking_target_half_height; p.target_half_width=cfg.tracking_target_half_width;
        for (int row=0;row<3;++row) {
            p.camera_translation(row)=cfg.tracking_camera_p[row];
            for (int col=0;col<3;++col) p.camera_rotation(row,col)=cfg.tracking_camera_R[3*row+col];
        }
        // An obstacle-free local corridor; execute only the first control period
        // and preserve its PVAJ/yaw state for the next receding-horizon solve.
        Eigen::Matrix<double,6,4> planes;
        const double center=.5*(head(0,0)+p.tail_pvaj(0,0));
        const double radius=.5*std::abs(head(0,0)-p.tail_pvaj(0,0))+2.;
        planes<<1,0,0,-center-radius, -1,0,0,center-radius,
                0,1,0,-2, 0,-1,0,-2, 0,0,1,-4, 0,0,-1,0;
        geometry_utils::Polytope poly; poly.SetPlanes(planes); p.sfcs={poly}; p.use_corridor=true;
        geometry_utils::Trajectory pos,yaw; std::string reason;
        const auto begin=std::chrono::steady_clock::now();
        const bool ok=solver.optimize(p,pos,&yaw,&reason);
        solve_ms.push_back(1000.*std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count());
        if (!ok) std::cerr<<"rolling step="<<step<<" "<<reason<<"\n";
        require(ok,"rolling solve");
        const auto dynamics=general_planner::checkTrackingDynamics(pos,yaw,cfg);
        if (!dynamics.valid) std::cerr<<"rolling step="<<step<<" "<<dynamics.reason
            <<" duration="<<pos.getTotalDuration()<<" head="<<head.row(0)
            <<" tail="<<p.tail_pvaj.row(0)<<"\n";
        require(dynamics.valid,"rolling trajectory dynamic limits");
        require(general_planner::trackingCandidateHasMotion(pos, targets, cfg, 0., 0.),
                "rolling candidate must pass the runtime motion gate using only trusted samples");
        require((pos.getState(0.)-head).norm()<1.e-6,"rolling PVAJ splice");
        head=pos.getState(dt); const auto ys=yaw.getState(dt);
        yaw_state<<ys(0,0),ys(0,1),ys(0,2);
        require(head(0,1)>-0.05,"straight tracking must not reverse");
    }
    const double error=24.-head(0,0)-fc.tracking_distance;
    require(std::abs(error)<1.0,"rolling straight tracking must converge to observation band");
    require(std::abs(head(0,1)-2.)<.3,"rolling straight tracking must match target speed");
    std::sort(solve_ms.begin(),solve_ms.end());
    std::cout<<"Rolling straight: initial_distance="<<initial_distance<<" final_band_error="<<error
             <<" final_speed="<<head(0,1)<<" solve_p95_ms="<<solve_ms[56]<<"\n";
}
}

int main() {
    try {
        timeMap(); const auto p=frontend(); gradient<3>(); gradient<4>(); optimize(p);
        rollingStraight(5.5); rollingStraight(10.);
        std::cout<<"Elastic tracking: bounded time, prediction lifetime, moving frontend, joint gradients and corridor solve passed\n";
        return 0;
    } catch (const std::exception &e) { std::cerr<<e.what()<<"\n"; return 1; }
}
