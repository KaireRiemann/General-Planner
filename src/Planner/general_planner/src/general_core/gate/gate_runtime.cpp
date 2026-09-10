#include <general_core/gate/gate_runtime.hpp>
#include <traj_opt/flatness/se3_flatness_map.hpp>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <future>
#include <sstream>
#include <general_core/gate/body_voxel_collision.hpp>
#include <atomic>
#include <general_core/gate/tracking_alignment.hpp>
#include <chrono>
#include <mutex>

namespace general_planner::gate {
namespace {
Eigen::Vector3d position(const nav_msgs::Odometry &o) {
  return {o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z};
}
double speed(const nav_msgs::Odometry &o) {
  return Eigen::Vector3d(o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z).norm();
}
double yaw(const nav_msgs::Odometry &o) {
  const auto &q = o.pose.pose.orientation;
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
void assign(geometry_msgs::Vector3 &v, const Eigen::Vector3d &x) { v.x=x.x(); v.y=x.y(); v.z=x.z(); }
}
struct Runtime::Impl {
  ros::NodeHandle nh;
  planner_runtime::PlannerCommandGateway &gateway;
  MapManager::Ptr map;
  Environment environment;
  Config cfg;
  std::mutex diagnostic_mutex;
  std::string map_failure;
  mutable std::mutex mutex;
  Status state;
  std::atomic<std::uint64_t> generation{0};
  ros::Subscriber observation_sub, odom_sub;
  ros::Publisher detection_pub, path_pub, candidate_pub, locked_pub;
  ros::WallTimer timer;
  aperture_detector::ApertureObservation observation;
  ros::Time last_observation_stamp, request_stamp, execution_start;
  ros::WallTime received_observation, received_odom, request_wall, execution_wall, settle_since, last_map_check;
  nav_msgs::Odometry odom;
  bool have_odom{false};
  int stable_count{0};
  struct Result { bool ok{false}; Plan plan; std::string reason; std::uint64_t generation{0}; };
  std::future<Result> worker;
  std::uint64_t worker_generation{0};
  Plan active;

  Impl(ros::NodeHandle node, planner_runtime::PlannerCommandGateway &g, MapManager::Ptr m, Environment env)
      : nh(node), gateway(g), map(std::move(m)), environment(std::move(env)) {
#define PARAM(name) nh.param("gate/" #name, cfg.name, cfg.name)
    PARAM(validation_policy); PARAM(feedback_max_delay);
    PARAM(body_radius); PARAM(body_height); PARAM(margin); PARAM(wall_depth); PARAM(exit_distance);
    PARAM(max_distance); PARAM(corridor_half_width); PARAM(corridor_half_height); PARAM(piece_length);
    PARAM(speed); PARAM(thrust_min); PARAM(thrust_max); PARAM(body_rate); PARAM(tilt); PARAM(weight_time);
    PARAM(weight_corridor); PARAM(observation_timeout); PARAM(observation_max_age); PARAM(planning_timeout);
    PARAM(odom_timeout); PARAM(finish_timeout); PARAM(finish_radius); PARAM(tracking_error); PARAM(validation_dt);
    PARAM(mass); PARAM(stable_observations); PARAM(require_known_free);
    PARAM(restore_start_height); PARAM(tunnel_half_depth); PARAM(overlap_slack);
    PARAM(weight_velocity); PARAM(weight_body_rate); PARAM(weight_tilt); PARAM(weight_thrust);
    PARAM(smooth_eps); PARAM(relative_cost_tolerance); PARAM(integral_steps); PARAM(max_iterations);
    PARAM(center_tolerance); PARAM(normal_tolerance); PARAM(area_tolerance);
    PARAM(map_anchor_search_radius); PARAM(map_anchor_search_step);
    PARAM(near_gate_lateral_error); PARAM(max_spatial_step); PARAM(completion_hold_time);
#undef PARAM
    if (cfg.validation_policy!="corridor_only" && cfg.validation_policy!="corridor_and_map")
      throw std::runtime_error("gate/validation_policy must be corridor_only or corridor_and_map");
    ROS_INFO_STREAM("[internal gate] validation_policy=" << cfg.validation_policy);
    if (!(cfg.body_radius > 0 && cfg.body_height > 0 && cfg.margin >= 0 && cfg.piece_length >= .1 &&
          cfg.validation_dt >= .005 && cfg.validation_dt <= .05 && cfg.speed > 0 && cfg.thrust_min > 0 &&
          cfg.thrust_max > cfg.thrust_min && cfg.tilt > 0 && cfg.tilt < 1.55 && cfg.mass > 0 &&
          cfg.observation_timeout > 0 && cfg.planning_timeout > 0 && cfg.odom_timeout > 0 &&
          cfg.finish_timeout > 0 && cfg.observation_max_age > 0 && cfg.stable_observations > 0 &&
          cfg.max_spatial_step > 0 && cfg.max_spatial_step<=.05 && cfg.completion_hold_time>0 &&
          cfg.center_tolerance>0 && cfg.normal_tolerance>0 && cfg.area_tolerance>0 &&
          cfg.integral_steps>=4 && cfg.max_iterations>0 && cfg.smooth_eps>0 &&
          cfg.relative_cost_tolerance>0 && cfg.overlap_slack>0 && cfg.tunnel_half_depth>=0 &&
          cfg.weight_velocity>0 && cfg.weight_body_rate>0 && cfg.weight_tilt>0 && cfg.weight_thrust>0 &&
          cfg.near_gate_lateral_error>0 && cfg.map_anchor_search_radius>=0 &&
          cfg.map_anchor_search_radius<=.3 && cfg.map_anchor_search_step>=.005 &&
          cfg.map_anchor_search_step<=.05 && cfg.weight_time>0 && cfg.weight_corridor>0 &&
          cfg.feedback_max_delay>=0 && cfg.feedback_max_delay<=.3 &&
          cfg.tracking_error>0 && cfg.finish_radius>0 && cfg.body_rate>0 &&
          cfg.corridor_half_width>0 && cfg.corridor_half_height>0 && cfg.exit_distance>0 &&
          cfg.wall_depth>0 && cfg.max_distance>0))
      throw std::runtime_error("invalid gate configuration");
    std::string obs_topic="/polygon_hole_step_viz/observation", odom_topic="/lidar_slam/odom";
    std::string enable_topic="/polygon_hole_step_viz/detection_enable";
    nh.param("gate/observation_topic", obs_topic, obs_topic);
    nh.param("gate/detection_enable_topic", enable_topic, enable_topic);
    nh.param("odometry_topic", odom_topic, odom_topic);
    detection_pub = nh.advertise<std_msgs::Bool>(enable_topic, 1, true);
    path_pub = nh.advertise<nav_msgs::Path>("/planning/gate/trajectory", 1, true);
    candidate_pub=nh.advertise<nav_msgs::Path>("/planning/gate/candidate_trajectory",1,true);
    locked_pub=nh.advertise<aperture_detector::ApertureObservation>("/planning/gate/locked_observation",1,true);
    if (!environment.ready) environment.ready = [this] { return cfg.validation_policy=="corridor_only" || (map && map->ready()); };
    if (!environment.body_clear) environment.body_clear = [this](const Eigen::Vector3d &p,
        const Eigen::Matrix3d &R, const Eigen::Vector3d &r) {
      if (!map || !map->ready()) return false;
      const double res = map->getResolution();
      auto reject = [this,&p,&R](const char *kind, const Eigen::Vector3d &cell) {
        std::ostringstream out; out<<kind<<" body=["<<p.transpose()<<"] voxel=["<<cell.transpose()<<"] body_z=["<<R.col(2).transpose()<<"]";
        std::lock_guard<std::mutex> lock(diagnostic_mutex); map_failure=out.str(); return false;
      };
      const Eigen::Vector3d extent=(R.array().square().matrix()*r.array().square().matrix()).array().sqrt();
      // Box expansion conservatively contains Euclidean margin, without
      // scaling all ellipsoid axes by a voxel's circumscribed sphere.
      const double half=.5*res+cfg.margin;
      for(int mask=0;mask<8;++mask) {
        Eigen::Vector3d corner=p;
        for(int k=0;k<3;++k) corner(k)+=((mask>>k)&1 ? 1. : -1.)*(extent(k)+cfg.margin);
        if(map->getGridType(corner)==rog_map::GridType::OUT_OF_MAP) return reject("MAP_OUT_OF_BOUNDS",corner);
      }
      rog_map::vec_E<rog_map::Vec3f> cells;
      // ROG boxSearch excludes boundary indices; add two complete cells so
      // no voxel intersecting the body's AABB is omitted at a query boundary.
      const Eigen::Vector3d search=extent.array()+half+2*res;
      map->boxSearch(p-search,p+search,rog_map::GridType::OCCUPIED,cells);
      for(const auto &cell:cells)
        if(bodyIntersectsVoxel(p,R,r,cell,half)) return reject("MAP_OCCUPIED",cell);
      if(cfg.require_known_free) {
        if(map->getGridType(p)!=rog_map::GridType::KNOWN_FREE) return reject("MAP_CENTER_NOT_FREE",p);
        map->boxSearch(p-search,p+search,rog_map::GridType::UNKNOWN,cells);
        for(const auto &cell:cells)
          if(bodyIntersectsVoxel(p,R,r,cell,half)) return reject("MAP_UNKNOWN",cell);
      }
      return true;
    };
    odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 20, [this](const nav_msgs::OdometryConstPtr &msg) {
      if (!position(*msg).allFinite() || !std::isfinite(speed(*msg))) return;
      const auto &q = msg->pose.pose.orientation;
      const double norm = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
      if (!std::isfinite(norm) || std::abs(norm-1.) > .05) return;
      std::lock_guard<std::mutex> lock(mutex);
      odom=*msg; received_odom=ros::WallTime::now(); have_odom=true;
    });
    observation_sub = nh.subscribe<aperture_detector::ApertureObservation>(obs_topic, 1,
        [this](const aperture_detector::ApertureObservationConstPtr &msg) {
      std::lock_guard<std::mutex> lock(mutex);
      if (state.phase != Phase::OBSERVING) return;
      if (!msg->geometry_valid || msg->header.stamp.isZero() || msg->header.stamp <= request_stamp ||
          !have_odom || msg->header.frame_id != odom.header.frame_id ||
          (ros::Time::now()-msg->header.stamp).toSec() > cfg.observation_max_age ||
          (ros::Time::now()-msg->header.stamp).toSec() < -.1) { stable_count=0; return; }
      if (msg->header.stamp <= last_observation_stamp) return;
      const Eigen::Vector3d c(msg->center.x,msg->center.y,msg->center.z);
      const Eigen::Vector3d prev(observation.center.x,observation.center.y,observation.center.z);
      const Eigen::Vector3d n(msg->normal.x,msg->normal.y,msg->normal.z);
      const Eigen::Vector3d old_n(observation.normal.x,observation.normal.y,observation.normal.z);
      const bool consistent = stable_count>0 && (c-prev).norm()<cfg.center_tolerance && n.norm()>.5 && old_n.norm()>.5 &&
          std::abs(n.normalized().dot(old_n.normalized()))>std::cos(cfg.normal_tolerance) &&
          std::abs(msg->hole_area-observation.hole_area) < cfg.area_tolerance * std::max(.01,observation.hole_area);
      stable_count=consistent ? stable_count+1 : 1;
      observation=*msg; last_observation_stamp=msg->header.stamp; received_observation=ros::WallTime::now();
    });
    enable(false);
    timer=nh.createWallTimer(ros::WallDuration(.01), [this](const ros::WallTimerEvent &) { tick(); });
  }
  std::string mapReason(const std::string &reason) {
    if(reason.find("; MAP_")!=std::string::npos || reason.find("MAP_COLLISION_OR_UNKNOWN")==std::string::npos) return reason;
    std::lock_guard<std::mutex> lock(diagnostic_mutex);
    return map_failure.empty() ? reason : reason+"; "+map_failure;
  }
  void enable(bool value) { std_msgs::Bool msg; msg.data=value; detection_pub.publish(msg); }
  void fail(const std::string &reason) { state.phase=Phase::FAILED; state.reason=mapReason(reason); ROS_WARN_STREAM("[internal gate] "<<state.reason); ++generation; enable(false); }
  bool odomFresh() const {
    return have_odom && (ros::WallTime::now()-received_odom).toSec() <= cfg.odom_timeout &&
      !odom.header.stamp.isZero() && (ros::Time::now()-odom.header.stamp).toSec() <= cfg.odom_timeout &&
      (ros::Time::now()-odom.header.stamp).toSec() >= -.1;
  }
  void tick() {
    std::lock_guard<std::mutex> lock(mutex);
    if (worker.valid() && worker.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      Result result;
      try { result=worker.get(); } catch(const std::exception &e) {
        if (worker_generation==generation && state.phase==Phase::PLANNING) fail(e.what());
        return;
      }
      if (result.generation==generation && state.phase==Phase::PLANNING) {
        if(!result.plan.trajectory.empty()) {
          nav_msgs::Path candidate; candidate.header.frame_id=result.plan.frame;
          candidate.header.stamp=ros::Time::now();
          for(double t=0;t<=result.plan.trajectory.getTotalDuration();t+=.02) {
            geometry_msgs::PoseStamped pose; pose.header=candidate.header;
            const auto p=result.plan.trajectory.getPos(t);
            pose.pose.position.x=p.x(); pose.pose.position.y=p.y(); pose.pose.position.z=p.z();
            pose.pose.orientation.w=1.; candidate.poses.push_back(pose);
          }
          candidate_pub.publish(candidate);
        }
        if (!result.ok) { fail(result.reason); return; }
        if (!odomFresh() || odom.header.frame_id!=result.plan.frame ||
            (position(odom)-result.plan.problem.head_pvaj.col(0)).norm()>.08 || speed(odom)>.10) {
          fail("START_MOVED_DURING_PLANNING"); return;
        }
        active=std::move(result.plan); active.problem.should_stop={};
        state.phase=Phase::READY; state.reason="SE3 trajectory validated";
        nav_msgs::Path path; path.header.frame_id=active.frame; path.header.stamp=ros::Time::now();
        for (double t=0; t<=active.trajectory.getTotalDuration(); t+=.04) {
          geometry_msgs::PoseStamped pose; pose.header=path.header;
          const auto p=active.trajectory.getPos(t); pose.pose.position.x=p.x(); pose.pose.position.y=p.y(); pose.pose.position.z=p.z();
          pose.pose.orientation.w=1; path.poses.push_back(pose);
        }
        path_pub.publish(path);
      }
    }
    if (state.phase==Phase::OBSERVING) {
      if ((ros::WallTime::now()-request_wall).toSec()>cfg.observation_timeout) { fail("OBSERVATION_TIMEOUT"); return; }
      if (!odomFresh() || !environment.ready() || stable_count<cfg.stable_observations || worker.valid()) return;
      if ((ros::WallTime::now()-received_observation).toSec()>cfg.observation_max_age || speed(odom)>.10) return;
      locked_pub.publish(observation);
      const auto obs=observation; const auto start=position(odom); const auto heading=yaw(odom);
      const auto token=generation.load(); const auto config=cfg; const auto clear=environment.body_clear;
      state.phase=Phase::PLANNING; state.reason="locked aperture; solving SE3"; enable(false);
      worker_generation=token;
      worker=std::async(std::launch::async, [this,obs,start,heading,token,config,clear] {
        Result result; result.generation=token;
        if (!buildProblem(obs,start,heading,config,result.plan,result.reason)) return result;
        if(config.validation_policy=="corridor_and_map" && map && map->ready()) {
          Eigen::Vector3d lo(obs.center.x,obs.center.y,obs.center.z),hi=lo;
          for(const auto &v:obs.boundary.points) {Eigen::Vector3d p(v.x,v.y,v.z);lo=lo.cwiseMin(p);hi=hi.cwiseMax(p);}
          const double res=map->getResolution();
          const double padding=config.wall_depth+2*res+config.body_radius;
          rog_map::vec_E<rog_map::Vec3f> cells;
          map->boxSearch(lo.array()-padding,hi.array()+padding,rog_map::GridType::OCCUPIED,cells);
          if(!constrainTunnelToMap(result.plan,config,cells,res,result.reason)) return result;
        }
        const auto deadline=ros::WallTime::now()+ros::WallDuration(config.planning_timeout);
        result.plan.problem.should_stop=[this,token,deadline] { return generation!=token || ros::WallTime::now()>deadline || !ros::ok(); };
        traj_opt::Config opt_cfg{}; opt_cfg.grav=9.81; opt_cfg.mass=config.mass;
        opt_cfg.integral_reso=config.integral_steps; opt_cfg.smooth_eps=config.smooth_eps; opt_cfg.opt_accuracy=config.relative_cost_tolerance;
        traj_opt::SE3AggressiveTrajOpt optimizer(opt_cfg,nullptr);
        if (!optimizer.optimize(result.plan.problem,result.plan.trajectory)) { result.reason="SE3_SOLVE_FAILED_OR_TIMEOUT"; return result; }
        result.ok=validateTrajectory(result.plan,config,clear,result.reason);
        if(!result.ok) result.reason=mapReason(result.reason);
        return result;
      });
    }
    if (state.phase!=Phase::EXECUTING && state.phase!=Phase::SETTLING) return;
    if (!odomFresh() || odom.header.frame_id!=active.frame) { fail("ODOMETRY_STALE_OR_FRAME_CHANGED"); return; }
    const double total=active.trajectory.getTotalDuration();
    const double elapsed=(ros::Time::now()-execution_start).toSec();
    if (elapsed < -.01 || (ros::WallTime::now()-execution_wall).toSec()>total+cfg.finish_timeout+2) { fail("EXECUTION_CLOCK_TIMEOUT"); return; }
    const double t=std::clamp(elapsed,0.,total);
    const auto p=active.trajectory.getPos(t);
    const double receipt_age=std::max(0.0,(ros::WallTime::now()-received_odom).toSec());
    const double matched_t=matchFeedbackTime(position(odom),elapsed,receipt_age,
        cfg.feedback_max_delay,total,[this](double time) {return active.trajectory.getPos(time);});
    const Eigen::Vector3d matched=active.trajectory.getPos(matched_t);
    const Eigen::Vector3d error=position(odom)-matched;
    const double lateral=(error-active.normal*active.normal.dot(error)).norm();
    auto trackingFailure=[&](const char *kind) {
      std::ostringstream out;
      out<<kind<<" t="<<t<<" matched_t="<<matched_t<<" receipt_age="<<receipt_age
         <<" max_delay="<<cfg.feedback_max_delay<<" error="<<error.norm()<<" lateral="<<lateral
         <<" measured=["<<position(odom).transpose()<<"] reference=["<<matched.transpose()<<"]";
      fail(out.str());
    };
    // The current-time bound remains a separate guard against large lag.
    if ((position(odom)-p).norm()>cfg.tracking_error || error.norm()>cfg.tracking_error) {
      trackingFailure("TRACKING_ERROR"); return;
    }
    const double near_range=active.wall_half_depth+cfg.body_radius+.3;
    if((std::abs(active.normal.dot(position(odom)-active.center))<near_range ||
        std::abs(active.normal.dot(matched-active.center))<near_range) &&
        lateral>cfg.near_gate_lateral_error) {
      trackingFailure("GATE_LATERAL_TRACKING_ERROR"); return;
    }
    // Check the actual measured ellipsoid, not just the reference trajectory.
    // Time matching may tolerate transport lag but must not excuse a frame hit.
    const auto &measured_q=odom.pose.pose.orientation;
    const Eigen::Matrix3d body_R=Eigen::Quaterniond(measured_q.w,measured_q.x,measured_q.y,measured_q.z)
                                      .normalized().toRotationMatrix();
    const Eigen::Vector3d body_r(cfg.body_radius,cfg.body_radius,cfg.body_height);
    // Corridor interfaces are artificial: delayed feedback can still be in
    // the preceding overlap cell. Accept containment in any corridor cell,
    // rather than imposing the reference piece's time-dependent cell index.
    bool measured_inside=false;
    for(const auto &poly:active.problem.hpolys) {
      bool inside=true;
      for(int i=0;i<poly.cols();++i) {
        const Eigen::Vector3d normal=poly.col(i).head<3>();
        const double violation=normal.dot(position(odom)-poly.col(i).tail<3>())+
            (body_r.array()*(body_R.transpose()*normal).array()).matrix().norm()+cfg.margin;
        if(violation>.001) {inside=false;break;}
      }
      if(inside) {measured_inside=true;break;}
    }
    if(!measured_inside) {trackingFailure("MEASURED_CORRIDOR_VIOLATION"); return;}
    if ((ros::WallTime::now()-last_map_check).toSec()>.08) {
      std::string reason;
      for (double ahead=0.; ahead<=.20+1e-6; ahead+=std::min(cfg.validation_dt,cfg.max_spatial_step/(cfg.speed+.01))) {
        if (!validateSample(active,std::min(t+ahead,total),cfg,environment.body_clear,reason)) { fail(reason); return; }
      }
      const auto &q=odom.pose.pose.orientation;
      const Eigen::Matrix3d measured_R=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
      Eigen::Vector3d measured_r(cfg.body_radius,cfg.body_radius,cfg.body_height);

      if (cfg.validation_policy=="corridor_and_map" && !environment.body_clear(position(odom),measured_R,measured_r)) { fail("MAP_COLLISION_OR_UNKNOWN measured_body"); return; }
      last_map_check=ros::WallTime::now();
    }
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.frame_id=active.frame; cmd.header.stamp=ros::Time::now();
    cmd.position.x=p.x(); cmd.position.y=p.y(); cmd.position.z=p.z();
    const auto v=active.trajectory.getVel(t), a=active.trajectory.getAcc(t), j=active.trajectory.getJer(t);
    assign(cmd.velocity,v); assign(cmd.acceleration,a); assign(cmd.jerk,j);
    cmd.yaw=active.problem.yaw; cmd.yaw_dot=0.; cmd.trajectory_id=static_cast<std::uint32_t>(state.epoch);
    cmd.trajectory_flag=quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    traj_opt::SE3FlatnessMap flatness; flatness.setYawMode(true,false);
    traj_opt::SE3FlatnessOutput flat;
    if (!flatness.forward(v,a,j,active.trajectory.getSnap(t),cmd.yaw,0.,9.81,flat)) { fail("INVALID_COMMAND"); return; }
    assign(cmd.angular_velocity,flat.omega);
    // Existing FSM contract: thrust.z is collective acceleration, not world force.
    cmd.thrust.z=flat.thrust;
    // ZYX roll/pitch/yaw for the existing PositionCommand contract.
    assign(cmd.attitude,Eigen::Vector3d(std::atan2(flat.R(2,1),flat.R(2,2)),
          std::asin(std::clamp(-flat.R(2,0),-1.,1.)),std::atan2(flat.R(1,0),flat.R(0,0))));
    cmd.vel_norm=v.norm(); cmd.acc_norm=a.norm();
    if (elapsed>=total) {
      state.phase=Phase::SETTLING; state.reason="verifying measured exit and hover";
      const bool exited=active.normal.dot(position(odom)-active.center)>
          active.wall_half_depth+cfg.body_radius+cfg.margin;
      if (exited && (position(odom)-active.problem.tail_pvaj.col(0)).norm()<cfg.finish_radius &&
          speed(odom)<.10 && std::abs(odom.twist.twist.angular.z)<.10) {
        if (settle_since.isZero()) settle_since=ros::WallTime::now();
        if ((ros::WallTime::now()-settle_since).toSec()>cfg.completion_hold_time) {
          state.phase=Phase::SUCCEEDED; state.reason="measured gate crossing completed";
          cmd.trajectory_flag=quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
      } else settle_since=ros::WallTime();
      if (elapsed>total+cfg.finish_timeout) { fail("EXIT_NOT_REACHED"); return; }
    }
    gateway.submitGateCommand(cmd,state.epoch);
  }
};
Runtime::Runtime(ros::NodeHandle nh, planner_runtime::PlannerCommandGateway &g, MapManager::Ptr map, Environment e)
  :impl_(new Impl(nh,g,std::move(map),std::move(e))) {}
Runtime::~Runtime() { impl_->timer.stop(); cancel(); if(impl_->worker.valid()) impl_->worker.wait(); }
void Runtime::start(std::uint64_t epoch) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  ++impl_->generation; impl_->state={Phase::OBSERVING,epoch,"waiting for stable aperture observations"};
  impl_->stable_count=0; impl_->last_observation_stamp=ros::Time();
  impl_->request_stamp=ros::Time::now(); impl_->request_wall=ros::WallTime::now(); impl_->enable(true);
}
void Runtime::cancel() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  ++impl_->generation; impl_->state.phase=Phase::IDLE; impl_->enable(false);
}
bool Runtime::execute(std::uint64_t epoch) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->state.epoch!=epoch || impl_->state.phase!=Phase::READY || !impl_->odomFresh() ||
      (position(impl_->odom)-impl_->active.problem.head_pvaj.col(0)).norm()>.08 || speed(impl_->odom)>.1) return false;
  impl_->execution_start=ros::Time::now(); impl_->execution_wall=ros::WallTime::now();
  impl_->settle_since=ros::WallTime(); impl_->last_map_check=ros::WallTime();
  impl_->state.phase=Phase::EXECUTING; impl_->state.reason="executing internal SE3 gate trajectory"; return true;
}
Status Runtime::status() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->state; }
} // namespace general_planner::gate
