// Elastic-Tracker minimum-jerk spatial/temporal optimization, ported 2026-09-20.
// Upstream: KaireRiemann/Elastic-Tracker 0a302a2, GPL-3.0 (LICENSE.Elastic-Tracker).
#include <traj_opt/tracking_traj_opt.hpp>
#include <traj_opt/tracking_objective.hpp>
#include <utils/geometry/geometry_utils.h>
#include <utils/optimization/lbfgs.h>
#include <chrono>
#include <numeric>

namespace traj_opt {
namespace {
using geometry_utils::Trajectory;
using general_utils::Vec3f;

Trajectory geometryTrajectory(const TrackingObjective::Trajectory &source) {
    Trajectory out;
    for (int i = 0; i < source.getDurations().size(); ++i)
        out.emplace_back(source.getDurations()(i), source.getPieceCoeffMat(i));
    return out;
}

// Elastic-Tracker publishes a target-facing yaw command independently of its
// position optimizer. GP needs a time-indexed polynomial: integrate a bounded
// bearing servo into C2-continuous cubic pieces, preserving the issued yaw PVA.
bool facingYaw(const TrackingProblem &problem, const Trajectory &position, Trajectory &yaw) {
    yaw.clear();
    double angle = problem.head_yaw(0,0), rate = problem.head_yaw(0,1);
    double acceleration = problem.head_yaw_acceleration;
    const double max_rate = std::max(problem.max_yaw_rate, std::abs(rate))+1.e-5;
    const double max_acc = std::max(problem.max_yaw_acceleration, std::abs(acceleration))+1.e-5;
    if (!std::isfinite(angle) || !std::isfinite(rate) || !std::isfinite(acceleration) ||
        !(max_rate > 0.0) || !(max_acc > 0.0)) return false;
    const double total = position.getTotalDuration();
    const int pieces = std::max(1, static_cast<int>(std::ceil(total/0.05)));
    const double dt = total/pieces;
    for (int i = 0; i < pieces; ++i) {
        const double t = i*dt;
        const auto target = sampleTrackingTarget(problem.target_prediction, t);
        const Vec3f ray = target.position-position.getPos(t);
        const Vec3f relative = target.velocity-position.getVel(t);
        const double range2 = ray.head<2>().squaredNorm();
        const double bearing = range2 > 1.e-8 ? std::atan2(ray.y(),ray.x()) : angle;
        const double bearing_rate = range2 > 1.e-4 ?
            (ray.x()*relative.y()-ray.y()*relative.x())/range2 : 0.0;
        const double desired_rate = std::clamp(bearing_rate+2.0*std::remainder(bearing-angle,2.0*M_PI),
                                               -0.85*max_rate,0.85*max_rate);
        const double desired_acc = std::clamp(4.0*(desired_rate-rate),-0.85*max_acc,0.85*max_acc);
        const double lower = std::max((-max_acc-acceleration)/dt,
            2.0*(-max_rate-rate-acceleration*dt)/(dt*dt));
        const double upper = std::min((max_acc-acceleration)/dt,
            2.0*(max_rate-rate-acceleration*dt)/(dt*dt));
        const double jerk = lower > upper ? 0.0 :
            std::clamp((desired_acc-acceleration)/dt,lower,upper);
        Eigen::Matrix<double,3,6> c = Eigen::Matrix<double,3,6>::Zero();
        c(0,5)=angle; c(0,4)=rate; c(0,3)=acceleration/2.0; c(0,2)=jerk/6.0;
        yaw.emplace_back(dt,c);
        angle += rate*dt+acceleration*dt*dt/2.0+jerk*dt*dt*dt/6.0;
        rate += acceleration*dt+jerk*dt*dt/2.0;
        acceleration += jerk*dt;
    }
    return !yaw.empty();
}
}

struct TrackingJerkTrajOpt::Impl {
    explicit Impl(const Config &config) : cfg(config) {}
    Config cfg;
    TrackingProblem problem;
    spatial_map::PolyhedraH corridors;
    spatial_map::PolyhedraV vertices;
    Eigen::VectorXi vertex_indices;
    spatial_map::PolytopeSpatialMap spatial;
    std::unique_ptr<TrackingObjective> objective;
    std::chrono::steady_clock::time_point start;
    int iterations{0};
    bool expired() const {
        return (problem.should_stop && problem.should_stop()) ||
            (problem.solve_budget_seconds > 0.0 &&
             std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() >= problem.solve_budget_seconds);
    }
    static double cost(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
        return static_cast<Impl*>(ptr)->objective->evaluate(x,g);
    }
    static int progress(void *ptr, const Eigen::VectorXd &, const Eigen::VectorXd &,
                         double, double, int k, int) {
        auto &self = *static_cast<Impl*>(ptr);
        self.iterations = k;
        return k > std::max(1, self.problem.max_iterations);
    }
    bool initialize(std::string &reason) {
        corridors.clear(); vertices.clear();
        for (const auto &poly : problem.sfcs) {
            auto h = poly.GetPlanes();
            if (h.rows() < 4 || !h.allFinite()) { reason="invalid corridor planes"; return false; }
            for (int r=0;r<h.rows();++r) {
                const double norm=h.row(r).head<3>().norm();
                if (norm<1.e-9) { reason="zero corridor normal"; return false; }
                h.row(r)/=norm;
            }
            corridors.push_back(h);
        }
        if (corridors.empty()) { reason="tracking requires CIRI corridors"; return false; }
        // Upstream uses two polynomial pieces per corridor, alternating a
        // waypoint inside each polytope with a waypoint inside each overlap.
        if (corridors.size()==1) corridors.push_back(corridors.front());
        const int pieces=2*static_cast<int>(corridors.size());
        if (pieces>64) { reason="too many tracking corridor pieces"; return false; }
        if (!geometry_utils::pointInsidePolytope(problem.head_pvaj.col(0),corridors.front(),0.001) ||
            !geometry_utils::pointInsidePolytope(problem.tail_pvaj.col(0),corridors.back(),0.001)) {
            reason="corridor excludes fixed boundary"; return false;
        }
        TrackingObjective::Optimizer::WaypointsType points(pieces+1,3);
        points.row(0)=problem.head_pvaj.col(0).transpose();
        points.row(pieces)=problem.tail_pvaj.col(0).transpose();
        vertex_indices.resize(pieces-1);
        for (int i=0;i<pieces-1;++i) {
            spatial_map::PolyhedronH h=corridors[i/2];
            if (i%2) {
                const auto &next=corridors[i/2+1];
                h.conservativeResize(h.rows()+next.rows(),4);
                h.bottomRows(next.rows())=next;
            }
            spatial_map::PolyhedronV v;
            if (!geometry_utils::enumerateVs(h,v) || v.cols()<4 || !v.allFinite()) {
                reason="corridor or overlap has no volume"; return false;
            }
            points.row(i+1)=v.rowwise().mean().transpose();
            // Seed near the front-end route while remaining strictly inside
            // the waypoint's feasible polytope; the mapping enforces it later.
            Vec3f preferred=points.row(i+1).transpose();
            double best=std::numeric_limits<double>::infinity();
            for (std::size_t j=0;j<problem.guide_path.size();++j) {
                const auto &p=problem.guide_path[j];
                const double score=std::abs(static_cast<double>(j)/std::max<std::size_t>(1,problem.guide_path.size()-1)-
                                             static_cast<double>(i+1)/pieces);
                if (score<best && geometry_utils::pointInsidePolytope(p,h,-0.01)) { preferred=p; best=score; }
            }
            points.row(i+1)=0.9*preferred.transpose()+0.1*points.row(i+1);
            const Vec3f origin=v.col(0);
            if (v.cols()>1) v.rightCols(v.cols()-1).colwise()-=origin;
            vertices.push_back(v); vertex_indices(i)=i;
        }
        spatial.reset(&vertices,&vertex_indices,pieces-1,false);
        const double total=std::max(problem.min_total_duration, pieces*problem.min_piece_duration+0.001);
        const std::vector<double> times(pieces,total/pieces);
        objective=std::make_unique<TrackingObjective>(cfg,problem,corridors);
        if (!objective->initialize(problem,times,points,&spatial)) { reason="MINCO initialization failed"; return false; }
        return true;
    }
    bool validate(const Trajectory &trajectory, std::string &reason) const {
        if (trajectory.empty()) { reason="empty trajectory"; return false; }
        for (int i=0;i<trajectory.getPieceNum();++i) {
            const double duration=trajectory[i].getDuration();
            if (!std::isfinite(duration) || duration<=0.0) { reason="invalid piece duration"; return false; }
        }
        return true;
    }
    bool optimize(const TrackingProblem &input, Trajectory &out, Trajectory *yaw, std::string *failure) {
        out.clear(); if (yaw) yaw->clear(); if (failure) failure->clear();
        problem=input; iterations=0;
        start=std::chrono::steady_clock::now();
        if (problem.solve_report) *problem.solve_report={};
        const auto fail=[&](const std::string &why) { if(failure) *failure=why; return false; };
        if (problem.target_prediction.size()<2 || !problem.head_pvaj.allFinite() ||
            !problem.tail_pvaj.allFinite() || !(problem.min_total_duration>0.0)) return fail("invalid tracking problem");
        std::string reason;
        if (!initialize(reason)) return fail(reason);
        Eigen::VectorXd x=objective->initialGuess();
        math_utils::lbfgs::lbfgs_parameter_t params;
        params.mem_size=16; params.past=3; params.g_epsilon=1.e-10;
        params.min_step=1.e-32; params.delta=std::max(1.e-8,cfg.opt_accuracy);
        params.max_iterations=std::max(1,problem.max_iterations); params.max_linesearch=40;
        double value=0.0;
        const int status=math_utils::lbfgs::lbfgs_optimize(x,value,&Impl::cost,nullptr,&Impl::progress,this,params);
        Eigen::VectorXd gradient(x.size());
        value=objective->evaluate(x,gradient);
        if(problem.solve_report) {
            auto &r=*problem.solve_report;
            r.status=status; r.iterations=iterations; r.cost=value;
            r.elapsed_ms=1000.0*std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
            r.gradient_inf=gradient.lpNorm<Eigen::Infinity>(); r.budget_exhausted=expired();
        }
        if(!std::isfinite(value)||!gradient.allFinite()) return fail("non-finite Elastic-Tracker objective");
        Trajectory candidate=geometryTrajectory(objective->position());
        if(!validate(candidate,reason)) return fail(reason);
        Trajectory candidate_yaw;
        // Elastic-Tracker publishes a scalar yaw setpoint; a polynomial yaw is
        // only generated when a caller explicitly asks for the legacy output.
        if(yaw!=nullptr && !facingYaw(problem,candidate,candidate_yaw)) {
            candidate_yaw.clear();
            Eigen::Matrix<double,3,6> c=Eigen::Matrix<double,3,6>::Zero();
            c(0,5)=problem.head_yaw(0,0);
            candidate_yaw.emplace_back(candidate.getTotalDuration(),c);
        }
        if(problem.candidate_feasible && !problem.candidate_feasible(candidate,candidate_yaw))
            return fail("candidate rejected by runtime safety check");
        out=std::move(candidate);
        out.start_WT=problem.target_prediction.front().reference_time;
        if(yaw) { *yaw=std::move(candidate_yaw); yaw->start_WT=out.start_WT; }
        return true;
    }
};
TrackingJerkTrajOpt::TrackingJerkTrajOpt(const Config &cfg,
    const std::shared_ptr<ros_interface::RosInterface> &) : impl_(std::make_shared<Impl>(cfg)) {}
bool TrackingJerkTrajOpt::optimize(const TrackingProblem &problem, Trajectory &out,
    Trajectory *yaw, std::string *failure) { return impl_->optimize(problem,out,yaw,failure); }
} // namespace traj_opt
