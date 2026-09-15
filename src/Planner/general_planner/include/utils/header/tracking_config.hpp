#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace general_planner::config_utils {

// No reference preserves standalone/legacy profiles such as interface.yaml.
// An explicit reference must load successfully; never silently inherit the
// state2state optimizer when a tracking file is missing or malformed.
inline std::string resolveTrackingConfig(const std::string &navigation_path,
                                        const std::string &override_path = "") {
    std::string path = override_path;
    if (path.empty()) {
        const YAML::Node navigation = YAML::LoadFile(navigation_path);
        if (navigation["tracking_config"]) {
            path = navigation["tracking_config"].as<std::string>();
        }
    }
    if (path.empty()) return {};

    std::filesystem::path resolved(path);
    if (resolved.is_relative()) {
        resolved = std::filesystem::path(navigation_path).parent_path() / resolved;
    }
    resolved = std::filesystem::absolute(resolved).lexically_normal();
    const YAML::Node profile = YAML::LoadFile(resolved.string());
    if (!profile.IsMap() || !profile["fsm"].IsMap() ||
        !profile["general_planner"].IsMap() ||
        !profile["general_planner"]["tracking"].IsMap() ||
        !profile["traj_opt"].IsMap() ||
        !profile["traj_opt"]["tracking_traj"].IsMap()) {
        throw std::invalid_argument("Invalid tracking config " + resolved.string() +
            ": expected fsm, general_planner/tracking and traj_opt/tracking_traj");
    }
    return resolved.string();
}

}  // namespace general_planner::config_utils
