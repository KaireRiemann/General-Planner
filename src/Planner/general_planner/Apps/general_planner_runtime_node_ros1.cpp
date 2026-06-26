#include "ros_interface/ros1/fsm_ros1.hpp"

#include "general_planner_runtime_preset.hpp"

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

struct RuntimePreset {
    const char *name{nullptr};
    const char *yaml{nullptr};
    std::string mode;
    std::string backend;
};

struct OverlayRule {
    std::string interface_path;
    std::string runtime_path;
};

std::string lowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](const unsigned char c) {
                       return static_cast<char>(std::tolower(c));
                   });
    return value;
}

std::string normalizeToken(std::string value) {
    value = lowerAscii(std::move(value));
    std::replace(value.begin(), value.end(), '-', '_');
    return value;
}

std::string normalizeMode(std::string mode) {
    mode = normalizeToken(std::move(mode));
    if (mode == "state_to_state" || mode == "state_2_state" ||
        mode == "state2state" || mode == "click") {
        return "state2state";
    }
    if (mode == "tracking_perching" || mode == "trackingperching") {
        return "tracking_perching";
    }
    return mode;
}

std::string readStringOrEmpty(const YAML::Node &interface,
                              const std::string &interface_key) {
    if (!interface[interface_key]) {
        return "";
    }
    return interface[interface_key].as<std::string>();
}

bool selectRuntimePreset(const YAML::Node &interface,
                         RuntimePreset &preset,
                         std::string &error) {
    preset.mode = normalizeMode(readStringOrEmpty(interface, "mode"));
    if (preset.mode.empty()) {
        preset.mode = "tracking";
    }

    std::string backend = readStringOrEmpty(interface, "planner_backend");
    if (backend.empty()) {
        backend = readStringOrEmpty(interface, "preset");
    }
    preset.backend = normalizeToken(backend);

    if (preset.mode == "tracking" || preset.mode == "tracking_perching") {
        if (!preset.backend.empty() && preset.backend != "tracking") {
            error = "tracking mode only supports preset/planner_backend 'tracking'";
            return false;
        }
        preset.name = general_planner_runtime_preset::kTrackingPresetName;
        preset.yaml = general_planner_runtime_preset::kTrackingPresetYaml;
        preset.backend = "tracking";
        return true;
    }

    if (preset.mode == "state2state") {
        if (preset.backend.empty()) {
            preset.backend = "corridor";
        }
        if (preset.backend == "corridor" || preset.backend == "backup" ||
            preset.backend == "smooth" || preset.backend == "click_smooth") {
            preset.name = general_planner_runtime_preset::kState2StateCorridorPresetName;
            preset.yaml = general_planner_runtime_preset::kState2StateCorridorPresetYaml;
            preset.backend = "corridor";
            return true;
        }
        if (preset.backend == "esdf" || preset.backend == "click_esdf") {
            preset.name = general_planner_runtime_preset::kState2StateEsdfPresetName;
            preset.yaml = general_planner_runtime_preset::kState2StateEsdfPresetYaml;
            preset.backend = "esdf";
            return true;
        }
        if (preset.backend == "plain" || preset.backend == "direct" ||
            preset.backend == "click_plain") {
            preset.name = general_planner_runtime_preset::kState2StatePlainPresetName;
            preset.yaml = general_planner_runtime_preset::kState2StatePlainPresetYaml;
            preset.backend = "plain";
            return true;
        }
        error = "state2state planner_backend must be one of: corridor, esdf, plain";
        return false;
    }

    error = "unsupported interface mode '" + preset.mode +
            "'. Supported modes: tracking, tracking_perching, state2state";
    return false;
}

std::vector<std::string> splitPath(const std::string &path) {
    std::vector<std::string> parts;
    std::stringstream ss(path);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (!part.empty()) {
            parts.emplace_back(part);
        }
    }
    return parts;
}

template<typename T>
void setByPathRecursive(YAML::Node node,
                        const std::vector<std::string> &parts,
                        const std::size_t index,
                        const T &value) {
    if (index + 1 >= parts.size()) {
        node[parts[index]] = value;
        return;
    }
    if (!node[parts[index]] || !node[parts[index]].IsMap()) {
        node[parts[index]] = YAML::Node(YAML::NodeType::Map);
    }
    setByPathRecursive(node[parts[index]], parts, index + 1, value);
}

template<typename T>
void setByPath(YAML::Node root, const std::string &path, const T &value) {
    const auto parts = splitPath(path);
    if (!parts.empty()) {
        setByPathRecursive(root, parts, 0, value);
    }
}

void setYamlNodeByPathRecursive(YAML::Node node,
                                const std::vector<std::string> &parts,
                                const std::size_t index,
                                const YAML::Node &value) {
    if (index + 1 >= parts.size()) {
        node.remove(parts[index]);
        if (value.IsScalar()) {
            const std::string scalar = normalizeToken(value.Scalar());
            if (scalar == "true" || scalar == "false" ||
                scalar == "yes" || scalar == "no" ||
                scalar == "on" || scalar == "off") {
                node[parts[index]] = value.as<bool>();
                return;
            }
        }
        node[parts[index]] = YAML::Clone(value);
        return;
    }
    if (!node[parts[index]] || !node[parts[index]].IsMap()) {
        node[parts[index]] = YAML::Node(YAML::NodeType::Map);
    }
    setYamlNodeByPathRecursive(node[parts[index]], parts, index + 1, value);
}

void setYamlNodeByPath(YAML::Node root,
                       const std::string &path,
                       const YAML::Node &value) {
    const auto parts = splitPath(path);
    if (!parts.empty()) {
        setYamlNodeByPathRecursive(root, parts, 0, value);
    }
}

bool getByParts(const YAML::Node &node,
                const std::vector<std::string> &parts,
                const std::size_t index,
                YAML::Node &value) {
    if (!node.IsDefined()) {
        return false;
    }
    if (index >= parts.size()) {
        value = node;
        return true;
    }
    const YAML::Node child = node[parts[index]];
    if (!child.IsDefined()) {
        return false;
    }
    return getByParts(child, parts, index + 1, value);
}

bool getByPath(const YAML::Node &root,
               const std::string &path,
               YAML::Node &value) {
    const auto parts = splitPath(path);
    return getByParts(root, parts, 0, value);
}

template<typename T>
void applyIfPresent(YAML::Node root,
                    const YAML::Node &interface,
                    const std::string &interface_key,
                    const std::string &runtime_path) {
    YAML::Node value;
    if (!getByPath(interface, interface_key, value) || value.IsNull()) {
        return;
    }
    setByPath(root, runtime_path, value.as<T>());
}

template<typename T>
T readOr(const YAML::Node &interface,
         const std::string &interface_key,
         const T &default_value) {
    YAML::Node value;
    if (!getByPath(interface, interface_key, value) || value.IsNull()) {
        return default_value;
    }
    return value.as<T>();
}

std::string defaultRuntimeConfigDir() {
    const char *home = std::getenv("HOME");
    if (home != nullptr && std::string(home).size() > 0) {
        return std::string(home) + "/.ros/general_planner_runtime";
    }
    return "/tmp/general_planner_runtime";
}

bool isFullRuntimeConfig(const YAML::Node &root) {
    return root.IsDefined() && root.IsMap() &&
           root["fsm"].IsDefined() && root["fsm"].IsMap() &&
           root["general_planner"].IsDefined() &&
           root["general_planner"].IsMap() &&
           root["rog_map"].IsDefined() && root["rog_map"].IsMap();
}

std::string detectFullConfigBackend(const YAML::Node &root) {
    if (root["fsm"] && root["fsm"]["task_mode"]) {
        const std::string mode =
                normalizeMode(root["fsm"]["task_mode"].as<std::string>());
        if (mode == "tracking" || mode == "tracking_perching") {
            return mode;
        }
    }
    if (!root["general_planner"]) {
        return "full";
    }
    const YAML::Node planner = root["general_planner"];
    if (planner["plain_traj_en"] && planner["plain_traj_en"].as<bool>()) {
        return "plain";
    }
    if (planner["backup_traj_en"] && planner["backup_traj_en"].as<bool>()) {
        return "corridor";
    }
    if (planner["esdf_traj_en"] && planner["esdf_traj_en"].as<bool>()) {
        return "esdf";
    }
    return "full";
}

void addFieldRules(std::vector<OverlayRule> &rules,
                   const std::string &interface_prefix,
                   const std::string &runtime_prefix,
                   std::initializer_list<const char *> fields) {
    for (const char *field: fields) {
        rules.push_back({interface_prefix + "/" + field,
                         runtime_prefix + "/" + field});
    }
}

std::vector<OverlayRule> releaseOverlayRules() {
    std::vector<OverlayRule> rules;

    addFieldRules(rules, "fsm", "fsm", {
            "auto_start",
            "timer_en",
            "click_goal_en",
            "click_goal_topic",
            "click_height",
            "click_yaw_en",
            "replan_rate",
            "cmd_topic",
            "mpc_cmd_topic",
            "publish_so3_cmd",
            "so3_cmd_topic",
            "so3_kR",
            "so3_kOm",
            "task_planner_en",
            "task_mode_topic",
            "tracking_target_odom_topic",
            "tracking_target_prediction_topic",
            "tracking_use_target_prediction_path",
            "perching_surface_odom_topic",
            "tracking_prediction_horizon",
            "tracking_prediction_dt",
            "tracking_prediction_use_kinodynamic",
            "tracking_prediction_accel",
            "tracking_prediction_vmax",
            "tracking_prediction_rho_accel",
            "tracking_prediction_max_time",
            "tracking_static_position_epsilon",
            "tracking_static_velocity_epsilon",
            "tracking_static_yaw_epsilon",
            "tracking_static_replan_remaining_time",
            "tracking_static_task_position_epsilon",
            "tracking_static_task_velocity_epsilon",
            "tracking_static_prediction_filter_velocity_epsilon",
            "tracking_static_safety_check_horizon",
            "tracking_static_safety_check_dt",
            "tracking_static_replan_log_period",
            "tracking_plan_from_rest_max_failures",
            "tracking_plan_from_rest_failure_backoff",
            "tracking_plan_from_rest_limited_backoff",
            "tracking_static_finish_on_plan_failure",
            "perception_replan_check_en",
            "perception_replan_check_rate",
            "perception_replan_check_horizon",
            "perception_replan_check_dt",
            "perception_replan_min_interval",
            "perception_replan_consecutive_hits",
            "perception_replan_unknown_as_occupied",
            "perception_replan_emergency_horizon",
            "perception_replan_log_period",
            "task_timeout",
            "state2state_plan_from_rest_max_failures",
            "state2state_clear_goal_on_plan_failure",
            "diagnostic_log_en",
            "diagnostic_event_topic"
    });

    const std::initializer_list<const char *> planner_fields = {
            "print_log",
            "detailed_log_en",
            "visualization_en",
            "esdf_safe_distance",
            "goal_vel_en",
            "goal_yaw_en",
            "visual_process",
            "use_fov_cut",
            "frontend_in_known_free",
            "safe_corridor_line_max_length",
            "sensing_horizon",
            "obs_skip_num",
            "replan_forward_dt",
            "corridor_bound_dis",
            "corridor_line_max_length",
            "planning_horizon",
            "receding_dis",
            "robot_r",
            "frontend_astar_time_out",
            "iris_iter_num",
            "ellipsoid_optimizer",
            "ellipsoid_optimizer_fallback",
            "yaw_mode",
            "mpc_horizon",
            "yaw_dot_max",
            "tracking/distance",
            "tracking/distance_tolerance",
            "tracking/distance_lower_tolerance",
            "tracking/distance_upper_tolerance",
            "tracking/height_offset",
            "tracking/height_tolerance",
            "tracking/safe_distance",
            "tracking/hard_safe_distance",
            "tracking/narrow_passage_enable",
            "tracking/narrow_passage_clearance_threshold",
            "tracking/narrow_passage_soft_safe_distance_scale",
            "tracking/visibility_safe_distance",
            "tracking/visibility_cone_ratio",
            "tracking/visibility_angle_clearance",
            "tracking/adaptive_occlusion_enable",
            "tracking/adaptive_occlusion_activation_distance",
            "tracking/adaptive_occlusion_max_weight_scale",
            "tracking/adaptive_occlusion_recovery_oe_scale",
            "tracking/adaptive_occlusion_od_far_weight_scale",
            "tracking/adaptive_occlusion_distance_upper_scale",
            "tracking/adaptive_occlusion_min_horizontal_upper",
            "tracking/adaptive_occlusion_postcheck_enable",
            "tracking/reacquire_distance",
            "tracking/min_commit_duration",
            "tracking/low_speed_velocity_threshold",
            "tracking/angular_hysteresis",
            "tracking/runtime_manager_enable",
            "tracking/anti_rollback_enable",
            "tracking/anti_rollback_horizon",
            "tracking/anti_rollback_dt",
            "tracking/anti_rollback_margin",
            "tracking/keep_old_horizon",
            "tracking/keep_old_safety_dt",
            "tracking/keep_old_short_safety_grace_enable",
            "tracking/keep_old_short_safety_grace_horizon",
            "tracking/keep_old_min_remaining",
            "tracking/keep_old_min_speed",
            "tracking/keep_old_min_displacement",
            "tracking/keep_old_min_progress_ratio",
            "tracking/keep_old_min_progress_3d_ratio",
            "tracking/keep_old_max_tracking_error_scale",
            "tracking/max_consecutive_keep_old",
            "tracking/no_motion_guard_enable",
            "tracking/no_motion_check_horizon",
            "tracking/no_motion_min_displacement",
            "tracking/no_motion_min_displacement_z",
            "tracking/no_motion_target_speed_threshold",
            "tracking/motion_3d_enable",
            "tracking/vertical_motion_threshold",
            "tracking/commit_start_time_tolerance",
            "tracking/detour_grace_enable",
            "tracking/detour_grace_horizon",
            "tracking/detour_max_tracking_error_scale",
            "tracking/anti_rollback_eval_after_prefix",
            "tracking/candidate_angle_step",
            "tracking/candidate_radius_num",
            "tracking/visibility_samples",
            "tracking/recovery_enable",
            "tracking/recovery_horizon",
            "tracking/recovery_distance_tolerance_scale",
            "tracking/recovery_height_tolerance_scale",
            "tracking/recovery_time_scale",
            "tracking/recovery_reduce_visible_region_weight",
            "tracking/recovery_reduce_target_forward_weight",
            "tracking/reacquire_recovery_enable",
            "tracking/reacquire_recovery_horizon",
            "tracking/reacquire_transit_enable",
            "tracking/reacquire_transit_horizon_scale",
            "tracking/reacquire_transit_max_horizon",
            "tracking/reacquire_visible_region_weight_scale",
            "tracking/reacquire_fov_relax_enable",
            "tracking/reacquire_fov_deferred_strict_enable",
            "tracking/reacquire_fov_entry_distance",
            "tracking/reacquire_min_progress_distance",
            "tracking/reacquire_min_progress_ratio",
            "tracking/reacquire_fov_range_grace",
            "tracking/reacquire_fov_angular_grace_deg",
            "tracking/optimizer_commit_safety_precheck_enable",
            "tracking/retry_without_corridor_enable",
            "tracking/fallback_relax_enable",
            "tracking/fallback_distance_tolerance_scale",
            "tracking/fallback_height_tolerance_scale",
            "tracking/fallback_candidate_radius_extra",
            "tracking/fallback_candidate_angle_step_scale",
            "tracking/fallback_search_horizon_scale",
            "tracking/frontend_elastic_enable",
            "tracking/frontend_elastic_distance_tolerance_scale",
            "tracking/frontend_elastic_height_tolerance_scale",
            "tracking/frontend_partial_guide_enable",
            "tracking/frontend_partial_min_duration",
            "tracking/frontend_partial_min_samples",
            "tracking/weight_od_near",
            "tracking/weight_od_far",
            "tracking/weight_od_vertical",
            "tracking/weight_oa",
            "tracking/weight_oe",
            "tracking/weight_relative_velocity",
            "tracking/weight_tangent_velocity",
            "tracking/weight_viewpoint_attractor",
            "tracking/weight_visible_region",
            "tracking/weight_fov",
            "tracking/weight_target_forward",
            "tracking/static_distance_tolerance_scale",
            "tracking/static_height_tolerance_scale",
            "tracking/static_tangent_weight_scale",
            "tracking/static_tail_speed_epsilon",
            "tracking/fov_horizontal_deg",
            "tracking/fov_vertical_deg",
            "tracking/fov_range",
            "tracking/target_front_margin",
            "tracking/fov_commit_check_enable",
            "tracking/fov_check_strict",
            "tracking/fov_check_dt",
            "tracking/fov_range_grace_enable",
            "tracking/fov_range_grace",
            "tracking/fov_keep_old_angular_grace_deg",
            "tracking/fov_keep_old_violation_ratio_grace",
            "tracking/fov_range_margin",
            "tracking/fov_front_margin",
            "tracking/fov_check_first_commit",
            "tracking/keep_old_requires_fov",
            "tracking/frontend_fov_feasibility_enable",
            "tracking/frontend_yaw_rate_feasibility_enable",
            "tracking/frontend_fov_range_margin",
            "tracking/frontend_yaw_rate_margin",
            "tracking/frontend_obstacle_recovery_enable",
            "tracking/frontend_grid_neighbor_mode",
            "tracking/frontend_side_pass_enable",
            "tracking/frontend_side_pass_width",
            "tracking/frontend_reacquire_relax_yaw_rate",
            "tracking/joint_sample_dt",
            "tracking/dense_joint_sample_enable",
            "tracking/unknown_as_occupied",
            "tracking/frontend_astar",
            "tracking/use_visible_region",
            "tracking/use_snap"
    };
    addFieldRules(rules, "planner", "general_planner", planner_fields);
    addFieldRules(rules, "general_planner", "general_planner", planner_fields);

    const std::initializer_list<const char *> map_fields = {
            "resolution",
            "inflation_resolution",
            "inflation_step",
            "unk_inflation_en",
            "unk_inflation_step",
            "map_size",
            "fix_map_origin",
            "frontier_extraction_en",
            "virtual_ceil_height",
            "virtual_ground_height",
            "load_pcd_en",
            "pcd_name",
            "map_sliding/enable",
            "map_sliding/threshold",
            "esdf/enable",
            "esdf/resolution",
            "esdf/local_update_box",
            "ros_callback/enable",
            "ros_callback/cloud_topic",
            "ros_callback/odom_topic",
            "ros_callback/odom_timeout",
            "visualization/enable",
            "visualization/use_dynamic_reconfigure",
            "visualization/pub_unknown_map_en",
            "visualization/frame_id",
            "visualization/time_rate",
            "visualization/frame_rate",
            "visualization/range",
            "intensity_thresh",
            "point_filt_num",
            "raycasting/enable",
            "raycasting/batch_update_size",
            "raycasting/local_update_box",
            "raycasting/ray_range",
            "raycasting/p_min",
            "raycasting/p_miss",
            "raycasting/p_free",
            "raycasting/p_occ",
            "raycasting/p_hit",
            "raycasting/p_max",
            "raycasting/unk_thresh"
    };
    addFieldRules(rules, "map", "rog_map", map_fields);
    addFieldRules(rules, "rog_map", "rog_map", map_fields);

    return rules;
}

void collectLeafPaths(const YAML::Node &node,
                      const std::string &prefix,
                      std::vector<std::string> &paths) {
    if (!node || !node.IsMap()) {
        if (!prefix.empty()) {
            paths.push_back(prefix);
        }
        return;
    }
    for (auto it = node.begin(); it != node.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        collectLeafPaths(it->second, prefix.empty() ? key : prefix + "/" + key, paths);
    }
}

void warnUnsupportedReleaseKeys(const YAML::Node &interface,
                                const std::vector<OverlayRule> &rules) {
    std::set<std::string> supported_paths;
    for (const auto &rule: rules) {
        supported_paths.insert(rule.interface_path);
    }

    const std::vector<std::string> configurable_roots = {
            "fsm",
            "planner",
            "general_planner",
            "map",
            "rog_map"
    };
    for (const auto &root_name: configurable_roots) {
        const YAML::Node section = interface[root_name];
        if (!section || !section.IsMap()) {
            continue;
        }
        std::vector<std::string> leaf_paths;
        collectLeafPaths(section, root_name, leaf_paths);
        for (const auto &path: leaf_paths) {
            if (supported_paths.find(path) == supported_paths.end()) {
                ROS_WARN_STREAM("Unsupported release config key ignored: interface."
                                << path);
            }
        }
    }
}

void applyReleaseOverlays(YAML::Node root, const YAML::Node &interface) {
    const auto rules = releaseOverlayRules();
    warnUnsupportedReleaseKeys(interface, rules);

    for (const auto &rule: rules) {
        YAML::Node value;
        if (getByPath(interface, rule.interface_path, value)) {
            setYamlNodeByPath(root, rule.runtime_path, value);
        }
    }
}

bool writeYamlFile(const YAML::Node &root,
                   const std::string &path,
                   std::string &error) {
    try {
        const boost::filesystem::path out_path(path);
        const boost::filesystem::path parent = out_path.parent_path();
        if (!parent.empty()) {
            boost::filesystem::create_directories(parent);
        }
        YAML::Emitter emitter;
        emitter << root;
        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out.good()) {
            error = "failed to open output file";
            return false;
        }
        out << emitter.c_str() << std::endl;
        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
}

bool buildRuntimeConfig(const std::string &interface_config_path,
                        const std::string &generated_config_path,
                        std::string &selected_preset_name,
                        std::string &selected_backend,
                        std::string &error) {
    YAML::Node root;
    YAML::Node interface_root;
    try {
        interface_root = YAML::LoadFile(interface_config_path);
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }

    if (isFullRuntimeConfig(interface_root)) {
        selected_preset_name =
                boost::filesystem::path(interface_config_path).filename().string();
        selected_backend = detectFullConfigBackend(interface_root);
        return writeYamlFile(interface_root, generated_config_path, error);
    }

    const YAML::Node interface =
            interface_root["interface"] ? interface_root["interface"] : interface_root;
    if (!interface || !interface.IsMap()) {
        error = "interface config must contain a map named 'interface'";
        return false;
    }

    RuntimePreset preset;
    if (!selectRuntimePreset(interface, preset, error)) {
        return false;
    }

    try {
        root = YAML::Load(preset.yaml);
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }

    selected_preset_name = preset.name;
    selected_backend = preset.backend;

    const std::string mode = preset.mode;
    setByPath(root, "fsm/task_mode", mode);
    setByPath(root, "fsm/timer_en", true);
    setByPath(root, "fsm/auto_start", readOr<bool>(interface, "auto_start", true));
    setByPath(root, "rog_map/ros_callback/enable", true);

    if (mode == "tracking" || mode == "tracking_perching") {
        setByPath(root, "fsm/click_goal_en", false);
        setByPath(root, "fsm/task_planner_en", false);
    }

    applyIfPresent<std::string>(root, interface, "odom_topic",
                                "rog_map/ros_callback/odom_topic");
    applyIfPresent<std::string>(root, interface, "cloud_topic",
                                "rog_map/ros_callback/cloud_topic");
    applyIfPresent<std::string>(root, interface, "target_odom_topic",
                                "fsm/tracking_target_odom_topic");
    applyIfPresent<std::string>(root, interface, "target_prediction_topic",
                                "fsm/tracking_target_prediction_topic");
    applyIfPresent<bool>(root, interface, "use_target_prediction_path",
                         "fsm/tracking_use_target_prediction_path");
    applyIfPresent<std::string>(root, interface, "pos_cmd_topic",
                                "fsm/cmd_topic");
    applyIfPresent<std::string>(root, interface, "poly_traj_topic",
                                "fsm/mpc_cmd_topic");
    applyIfPresent<std::string>(root, interface, "so3_cmd_topic",
                                "fsm/so3_cmd_topic");
    applyIfPresent<bool>(root, interface, "publish_so3_cmd",
                         "fsm/publish_so3_cmd");
    applyIfPresent<std::string>(root, interface, "diagnostics_topic",
                                "fsm/diagnostic_event_topic");
    applyIfPresent<bool>(root, interface, "diagnostic_log_en",
                         "fsm/diagnostic_log_en");
    applyIfPresent<double>(root, interface, "task_timeout",
                           "fsm/task_timeout");
    applyIfPresent<double>(root, interface, "odom_timeout",
                           "rog_map/ros_callback/odom_timeout");
    applyIfPresent<double>(root, interface, "replan_rate",
                           "fsm/replan_rate");

    applyIfPresent<std::string>(root, interface, "task_mode_topic",
                                "fsm/task_mode_topic");
    applyIfPresent<std::string>(root, interface, "click_goal_topic",
                                "fsm/click_goal_topic");
    applyIfPresent<bool>(root, interface, "click_goal_en",
                         "fsm/click_goal_en");
    applyIfPresent<double>(root, interface, "click_height",
                           "fsm/click_height");
    applyIfPresent<bool>(root, interface, "click_yaw_en",
                         "fsm/click_yaw_en");
    applyIfPresent<bool>(root, interface, "task_planner_en",
                         "fsm/task_planner_en");
    applyIfPresent<std::string>(root, interface, "perching_surface_odom_topic",
                                "fsm/perching_surface_odom_topic");

    const bool visualization_enable =
            readOr<bool>(interface, "visualization_enable", false);
    setByPath(root, "general_planner/visualization_en", visualization_enable);
    setByPath(root, "rog_map/visualization/enable", visualization_enable);

    applyReleaseOverlays(root, interface);

    return writeYamlFile(root, generated_config_path, error);
}

} // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "general_planner_runtime_node");
    ros::NodeHandle nh("~");

    std::string interface_config_path;
    if (!nh.getParam("interface_config_path", interface_config_path) ||
        interface_config_path.empty()) {
        ROS_FATAL("Missing required private param '~interface_config_path'.");
        return 1;
    }

    std::string generated_config_path;
    nh.param<std::string>("generated_config_path",
                          generated_config_path,
                          defaultRuntimeConfigDir() + "/runtime_config_" +
                                  std::to_string(static_cast<long long>(getpid())) +
                                  ".yaml");

    std::string error;
    std::string selected_preset_name;
    std::string selected_backend;
    if (!buildRuntimeConfig(interface_config_path,
                            generated_config_path,
                            selected_preset_name,
                            selected_backend,
                            error)) {
        ROS_FATAL("Failed to build runtime config from '%s': %s",
                  interface_config_path.c_str(),
                  error.c_str());
        return 1;
    }

    ROS_INFO("General planner runtime preset: %s (%s)",
             selected_preset_name.c_str(),
             selected_backend.c_str());
    ROS_INFO("General planner interface config: %s",
             interface_config_path.c_str());
    ROS_INFO("General planner generated runtime config: %s",
             generated_config_path.c_str());

    auto fsm_ptr = std::make_shared<fsm::FsmRos1>();
    fsm_ptr->init(nh, generated_config_path);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    spinner.stop();
    fsm_ptr.reset();
    return 0;
}
