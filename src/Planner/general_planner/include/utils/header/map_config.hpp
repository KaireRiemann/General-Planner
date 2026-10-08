#pragma once
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace general_planner::config_utils {
inline double mapResolution(const std::string &planner_path, double injected = 0.0) {
    if (injected == 0.0) {
        const auto planner = YAML::LoadFile(planner_path);
        // Standalone legacy profiles own a map; composed runtime injects its actual map.
        const auto embedded = planner["rog_map"];
        const auto map = embedded ? planner : YAML::LoadFile(
            (std::filesystem::path(planner_path).parent_path() / "map.yaml").string());
        if (!map["rog_map"] || !map["rog_map"]["resolution"])
            throw std::invalid_argument("Map resolution is missing: " + planner_path);
        injected = map["rog_map"]["resolution"].as<double>();
    }
    if (!std::isfinite(injected) || injected <= 0.0)
        throw std::invalid_argument("Map resolution must be finite and positive");
    return injected;
}
}  // namespace general_planner::config_utils
