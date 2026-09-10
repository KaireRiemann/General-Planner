#include <general_core/gate/gate_frontend.hpp>
#include <general_core/gate/body_voxel_collision.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
int main(int argc,char **argv) {
 if(argc!=3) return 2;
 auto snapshot=YAML::LoadFile(argv[1]);
 aperture_detector::ApertureObservation obs;obs.header.frame_id="world";obs.geometry_valid=true;obs.normal.x=1.;
 obs.center.x=snapshot["center"][0].as<double>();obs.center.y=snapshot["center"][1].as<double>();obs.center.z=snapshot["center"][2].as<double>();
 for(auto v:snapshot["vertices_xyz"]) { geometry_msgs::Point32 p;p.x=v[0].as<double>();p.y=v[1].as<double>();p.z=v[2].as<double>();obs.boundary.points.push_back(p); }
 Eigen::Vector3d start;for(int k=0;k<3;++k)start(k)=snapshot["start"][k].as<double>();
 std::ifstream file(argv[2]);general_utils::vec_E<general_utils::Vec3f> cells;double x,y,z;
 while(file>>x>>y>>z)cells.emplace_back(x,y,z);
 general_planner::gate::Config cfg;cfg.restore_start_height=true;
 general_planner::gate::Plan plan;std::string reason;
 if(!general_planner::gate::buildProblem(obs,start,0,cfg,plan,reason)){std::cerr<<reason;return 1;}
 if(!general_planner::gate::constrainTunnelToMap(plan,cfg,cells,.15,reason)){std::cerr<<reason;return 1;}
 traj_opt::Config opt{};opt.grav=9.81;opt.mass=cfg.mass;opt.integral_reso=cfg.integral_steps;opt.smooth_eps=cfg.smooth_eps;opt.opt_accuracy=cfg.relative_cost_tolerance;
 traj_opt::SE3AggressiveTrajOpt solver(opt,nullptr);
 if(!solver.optimize(plan.problem,plan.trajectory))return 1;
 auto clear=[&](const Eigen::Vector3d &p,const Eigen::Matrix3d &R,const Eigen::Vector3d &r) {
   for(auto cell:cells) if(general_planner::gate::bodyIntersectsVoxel(p,R,r,cell,.075+cfg.margin)) {std::cerr<<"OCC p "<<p.transpose()<<" cell "<<cell.transpose()<<"\n";return false;}return true;
 };
 bool ok=general_planner::gate::validateTrajectory(plan,cfg,clear,reason);
 std::cout<<"snapshot occupied-only validation: "<<ok<<" "<<reason<<"\n";
 return ok?0:1;
}
