#pragma once

#include <memory>
#include <vector>

#include <utils/header/type_utils.hpp>

namespace path_search {
class Astar;
}

namespace ros_interface {
class RosInterface;
}

namespace general_planner {

class Config;
class DynamicObstacleLayer;
class MapManager;

namespace state2state_task {

struct StateToStateFrontendServices {
    const Config &cfg;
    std::shared_ptr<MapManager> map_manager;
    std::shared_ptr<ros_interface::RosInterface> ros_ptr;
    std::shared_ptr<path_search::Astar> astar;
    DynamicObstacleLayer *dynamic_obstacle_layer{nullptr};
    general_utils::Vec3f &local_start_p;
    const general_utils::Vec3f &goal_p;
    bool &goal_valid;
};

bool pathSearch(StateToStateFrontendServices &services,
                const general_utils::Vec3f &start_pt,
                const general_utils::Vec3f &goal,
                double searching_horizon,
                general_utils::vec_Vec3f &path);

bool prepareESDFGuideEndpoint(StateToStateFrontendServices &services,
                              general_utils::vec_Vec3f &guide_path,
                              std::vector<double> &guide_stamp);

} // namespace state2state_task
} // namespace general_planner
