#pragma once

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "utils/color_text.hpp"
#include "utils/eigen_alias.hpp"

namespace task_planner {
    using namespace color_text;
    using namespace std;
    using namespace super_utils;

    enum class ManagedTaskMode {
        STATE_TO_STATE = 0,
        TRACKING = 1,
        PERCHING = 2
    };

    inline std::string normalizeTaskMode(std::string mode) {
        std::transform(mode.begin(), mode.end(), mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (mode == "state_to_state" || mode == "state-2-state" || mode == "state2state" ||
            mode == "waypoint" || mode == "s2s" || mode == "goal") {
            return "state2state";
        }
        if (mode == "track" || mode == "tracking") {
            return "tracking";
        }
        if (mode == "perch" || mode == "perching") {
            return "perching";
        }
        return "state2state";
    }

    inline ManagedTaskMode taskModeFromString(const std::string &mode) {
        const std::string normalized = normalizeTaskMode(mode);
        if (normalized == "tracking") {
            return ManagedTaskMode::TRACKING;
        }
        if (normalized == "perching") {
            return ManagedTaskMode::PERCHING;
        }
        return ManagedTaskMode::STATE_TO_STATE;
    }

    inline const char *taskModeToString(const ManagedTaskMode mode) {
        switch (mode) {
            case ManagedTaskMode::TRACKING:
                return "tracking";
            case ManagedTaskMode::PERCHING:
                return "perching";
            case ManagedTaskMode::STATE_TO_STATE:
            default:
                return "state2state";
        }
    }

    struct ManagedTask {
        ManagedTaskMode mode{ManagedTaskMode::STATE_TO_STATE};
        std::string name{"task"};
        Vec3f position{Vec3f::Zero()};
        Vec3f velocity{Vec3f::Zero()};
        Vec3f acceleration{Vec3f::Zero()};
        Vec3f rpy{Vec3f::Zero()};
        double yaw{NAN};
        double switch_dis{1.0};
        double publish_dt{0.2};
        double speed{1.0};
        double hold_duration{-1.0};
        bool loop{false};
        vec_E<Vec3f> waypoints;
    };

    class TaskPlannerConfig {
    public:
        int start_trigger_type{2}; // 0: rviz click, 1: mavros rc, 2: auto delay
        double start_program_delay{1.0};
        double odom_timeout{0.5};
        double publish_dt{0.2};
        double switch_dis{1.0};
        std::string frame_id{"world"};
        std::string odom_topic{"/lidar_slam/odom"};
        std::string goal_pub_topic{"/planning/click_goal"};
        std::string task_mode_topic{"/planning/task_mode"};
        std::string tracking_target_odom_topic{"/tracking/target_odom"};
        std::string tracking_target_path_topic{"/tracking/target_path"};
        std::string tracking_target_prediction_topic{"/tracking/target_prediction"};
        std::string perching_surface_odom_topic{"/perching/surface_odom"};
        std::string path_pub_topic{"/task_planner/path"};
        std::string marker_topic{"mkr"};
        double tracking_prediction_horizon{4.0};
        double tracking_prediction_dt{0.25};
        std::string default_task_mode{"state2state"};
        std::vector<ManagedTask> tasks;

        TaskPlannerConfig() = default;

        explicit TaskPlannerConfig(const std::string &cfg_path) {
            loadFromYaml(cfg_path);
        }

        void loadFromYaml(const std::string &cfg_path) {
            std::cout << "Load task planner config file: " << cfg_path << std::endl;
            const YAML::Node file = YAML::LoadFile(cfg_path);
            const YAML::Node root = file["task_planner"] ? file["task_planner"]
                                  : (file["task_manager"] ? file["task_manager"] : file);

            start_trigger_type = readScalar(root, "start_trigger_type", start_trigger_type);
            start_program_delay = readScalar(root, "start_program_delay", start_program_delay);
            odom_timeout = readScalar(root, "odom_timeout", odom_timeout);
            publish_dt = readScalar(root, "publish_dt", publish_dt);
            switch_dis = readScalar(root, "switch_dis", switch_dis);
            frame_id = readScalar(root, "frame_id", frame_id);
            odom_topic = readScalar(root, "odom_topic", odom_topic);
            goal_pub_topic = readScalar(root, "goal_pub_topic", goal_pub_topic);
            task_mode_topic = readScalar(root, "task_mode_topic", task_mode_topic);
            tracking_target_odom_topic =
                readScalar(root, "tracking_target_odom_topic", tracking_target_odom_topic);
            tracking_target_path_topic =
                readScalar(root, "tracking_target_path_topic", tracking_target_path_topic);
            tracking_target_prediction_topic =
                readScalar(root, "tracking_target_prediction_topic", tracking_target_prediction_topic);
            perching_surface_odom_topic =
                readScalar(root, "perching_surface_odom_topic", perching_surface_odom_topic);
            path_pub_topic = readScalar(root, "path_pub_topic", path_pub_topic);
            marker_topic = readScalar(root, "marker_topic", marker_topic);
            tracking_prediction_horizon =
                readScalar(root, "tracking_prediction_horizon", tracking_prediction_horizon);
            tracking_prediction_dt = readScalar(root, "tracking_prediction_dt", tracking_prediction_dt);
            default_task_mode = normalizeTaskMode(readScalar(root, "default_task_mode", default_task_mode));

            tasks.clear();
            loadYamlTasks(root);
            loadTopLevelWaypoints(root);

            std::cout << GREEN << " -- [TASK_PLANNER] Load " << tasks.size()
                      << " managed tasks." << RESET << std::endl;
            for (size_t i = 0; i < tasks.size(); ++i) {
                const auto &task = tasks[i];
                std::cout << BLUE << " -- [TASK_PLANNER] Task " << i << " [" << task.name
                          << "] mode=" << taskModeToString(task.mode)
                          << " pos=(" << task.position.transpose() << ")"
                          << " switch_dis=" << task.switch_dis << RESET << std::endl;
            }
        }

        void loadLegacyWaypointFile(const std::string &file_name) {
            if (file_name.empty()) {
                return;
            }
            std::ifstream file(file_name);
            if (!file.good()) {
                std::cout << YELLOW << " -- [TASK_PLANNER] Cannot open legacy waypoint file: "
                          << file_name << RESET << std::endl;
                return;
            }
            std::string line;
            int id = 0;
            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#') {
                    continue;
                }
                std::istringstream iss(line);
                std::vector<double> values;
                double value = 0.0;
                while (iss >> std::setprecision(16) >> value) {
                    values.push_back(value);
                }
                if (values.size() < 3) {
                    continue;
                }
                ManagedTask task;
                task.mode = ManagedTaskMode::STATE_TO_STATE;
                task.name = "legacy_waypoint_" + std::to_string(id++);
                task.position = Vec3f(values[0], values[1], values[2]);
                task.switch_dis = values.size() > 3 ? values[3] : switch_dis;
                task.publish_dt = publish_dt;
                tasks.push_back(task);
            }
        }

    private:
        template <typename T>
        static T readScalar(const YAML::Node &node, const std::string &key, const T &default_value) {
            if (!node || !node[key]) {
                return default_value;
            }
            try {
                return node[key].as<T>();
            } catch (const YAML::Exception &) {
                return default_value;
            }
        }

        static Vec3f readVec3(const YAML::Node &node, const Vec3f &default_value) {
            if (!node || !node.IsSequence() || node.size() < 3) {
                return default_value;
            }
            try {
                return Vec3f(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
            } catch (const YAML::Exception &) {
                return default_value;
            }
        }

        static vec_E<Vec3f> readVec3List(const YAML::Node &node) {
            vec_E<Vec3f> points;
            if (!node || !node.IsSequence()) {
                return points;
            }
            for (const auto &pt_node: node) {
                const Vec3f pt = readVec3(pt_node, Vec3f::Constant(NAN));
                if (std::isfinite(pt.x()) && std::isfinite(pt.y()) && std::isfinite(pt.z())) {
                    points.push_back(pt);
                }
            }
            return points;
        }

        ManagedTask makeTaskFromNode(const YAML::Node &node, const size_t task_id) const {
            ManagedTask task;
            const std::string mode = normalizeTaskMode(readScalar(node, "mode", default_task_mode));
            task.mode = taskModeFromString(mode);
            task.name = readScalar(node, "name", std::string("task_") + std::to_string(task_id));
            task.position = readVec3(node["position"], task.position);
            task.velocity = readVec3(node["velocity"], task.velocity);
            task.acceleration = readVec3(node["acceleration"], task.acceleration);
            task.rpy = readVec3(node["rpy"], task.rpy);
            task.yaw = readScalar(node, "yaw", task.yaw);
            task.switch_dis = readScalar(node, "switch_dis", switch_dis);
            task.publish_dt = readScalar(node, "publish_dt", publish_dt);
            task.speed = readScalar(node, "speed", task.speed);
            task.hold_duration = readScalar(node, "hold_duration", task.hold_duration);
            task.loop = readScalar(node, "loop", task.loop);
            task.waypoints = readVec3List(node["waypoints"]);
            if (!task.waypoints.empty()) {
                task.position = task.waypoints.front();
            }
            return task;
        }

        void loadYamlTasks(const YAML::Node &root) {
            const YAML::Node task_nodes = root["tasks"];
            if (!task_nodes || !task_nodes.IsSequence()) {
                return;
            }
            for (size_t i = 0; i < task_nodes.size(); ++i) {
                ManagedTask task = makeTaskFromNode(task_nodes[i], i);
                if (task.mode == ManagedTaskMode::STATE_TO_STATE && task.waypoints.size() > 1) {
                    for (size_t k = 0; k < task.waypoints.size(); ++k) {
                        ManagedTask waypoint_task = task;
                        waypoint_task.name = task.name + "_wp_" + std::to_string(k);
                        waypoint_task.position = task.waypoints[k];
                        waypoint_task.waypoints.clear();
                        tasks.push_back(waypoint_task);
                    }
                } else {
                    tasks.push_back(task);
                }
            }
        }

        void loadTopLevelWaypoints(const YAML::Node &root) {
            if (!tasks.empty()) {
                return;
            }
            const vec_E<Vec3f> points = readVec3List(root["waypoints"]);
            for (size_t i = 0; i < points.size(); ++i) {
                ManagedTask task;
                task.mode = ManagedTaskMode::STATE_TO_STATE;
                task.name = "waypoint_" + std::to_string(i);
                task.position = points[i];
                task.switch_dis = switch_dis;
                task.publish_dt = publish_dt;
                tasks.push_back(task);
            }
        }
    };
}
