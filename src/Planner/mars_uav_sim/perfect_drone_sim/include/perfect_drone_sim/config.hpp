//
// Created by yunfan on 2022/3/26.
//

#ifndef PERFECT_DRONE_CONFIG_HPP
#define PERFECT_DRONE_CONFIG_HPP


#include "cstring"
#include "vector"
#include "iostream"
#include "utils/yaml_loader.hpp"

namespace perfect_drone {
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class Config {
    public:
        std::string mesh_resource;
        std::string cmd_topic{"/planning/pos_cmd"};
        std::string odom_topic{"/lidar_slam/odom"};
        std::string pose_topic{"/lidar_slam/pose"};
        std::string local_pc_topic{"/cloud_registered"};
        std::string global_pc_topic{"/global_pc"};
        std::string dynamic_cloud_topic{"/perfect_drone/dynamic_obstacle_cloud"};
        std::string robot_frame_id{"perfect_drone"};
        Eigen::Vector3d init_pos;
        double init_roll{0.0};
        double init_pitch{0.0};
        double init_yaw{0.0};
        double lidar_pitch{0.0};
        double sensing_rate;
        bool dynamic_cloud_en{false};

        Config() = default;

        Config(const std::string &cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            loader.LoadParam("mesh_resource", mesh_resource, std::string("package://perfect_drone_sim/meshes/f250.dae"),
                             false);
            loader.LoadParam("cmd_topic", cmd_topic, std::string("/planning/pos_cmd"));
            loader.LoadParam("odom_topic", odom_topic, std::string("/lidar_slam/odom"));
            loader.LoadParam("pose_topic", pose_topic, std::string("/lidar_slam/pose"));
            loader.LoadParam("local_pc_topic", local_pc_topic, std::string("/cloud_registered"));
            loader.LoadParam("global_pc_topic", global_pc_topic, std::string("/global_pc"));
            loader.LoadParam("dynamic_cloud_topic", dynamic_cloud_topic,
                             std::string("/perfect_drone/dynamic_obstacle_cloud"));
            loader.LoadParam("robot_frame_id", robot_frame_id, std::string("perfect_drone"));
            loader.LoadParam("init_position/x", init_pos.x(), 0.0);
            loader.LoadParam("init_position/y", init_pos.y(), 0.0);
            loader.LoadParam("init_position/z", init_pos.z(), 1.5);
            loader.LoadParam("init_roll", init_roll, 0.0);
            loader.LoadParam("init_pitch", init_pitch, 0.0);
            loader.LoadParam("init_yaw", init_yaw, 0.0);
            loader.LoadParam("lidar_pitch", lidar_pitch, 0.0);
            loader.LoadParam("sensing_rate", sensing_rate, 10.0);
            loader.LoadParam("dynamic_cloud_en", dynamic_cloud_en, false);
        }
    };
}

#endif
