#include <epicon_native/epicon_planner/fast_exploration_manager.h>
#include <epicon_native/epicon_planner/expl_data.h>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>

int main(int argc,char **argv) {
  ros::init(argc,argv,"epicon_tour_self_test"); ros::NodeHandle nh("~");
  try {
    using namespace epicon_native;
    using namespace epicon_native::fast_planner;
    auto planner=std::make_shared<FastPlannerManager>();
    auto frontier=std::make_shared<FrontierManager>();
    std::string first_dir;
    for(int instance=0;instance<2;++instance) {
      FastExplorationManager manager;
      manager.initialize(nh,frontier,planner);
      if(manager.ep_->tsp_dir_==first_dir) throw std::runtime_error("LKH work directory reused");
      first_dir=manager.ep_->tsp_dir_;
      for(int dimension:{1,2,3,7}) {
        Eigen::MatrixXd costs=Eigen::MatrixXd::Constant(dimension,dimension,100.0);
        for(int i=0;i<dimension;++i) { costs(i,i)=0.0; costs(i,(i+1)%dimension)=1.0; }
        std::vector<int> tour;
        manager.solveLHK(costs,tour);
        if(tour.size()!=static_cast<size_t>(dimension) || tour.front()!=0 || std::set<int>(tour.begin(),tour.end()).size()!=tour.size())
          throw std::runtime_error("LKH returned malformed tour");
        for(int i=0;i<dimension;++i) if(tour[i]!=i) throw std::runtime_error("asymmetric optimum lost");
      }
      // Empty/one-entry tours must not index global_tour_[1].
      manager.ed_->global_tour_.clear(); manager.updateGoalNode();
      manager.ed_->global_tour_.push_back(Eigen::Vector3f::Zero()); manager.updateGoalNode();
    }
    if(std::filesystem::exists(first_dir)) throw std::runtime_error("LKH temporary directory leaked");
    std::cout<<"EPICON_TOUR PASS: tiny/asymmetric tours, instance isolation, incomplete-goal guards"<<std::endl;
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<std::endl; return 1; }
}
