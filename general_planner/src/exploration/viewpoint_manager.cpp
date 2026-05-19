#include "exploration/viewpoint_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace general_planner {
namespace exploration {

namespace {
double wrapYaw(double yaw) {
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}
}  // namespace

ViewpointManager::ViewpointManager(Config cfg) : cfg_(std::move(cfg)) {
    cfg_.radius_samples = std::max(1, cfg_.radius_samples);
    cfg_.yaw_samples = std::max(4, cfg_.yaw_samples);
    cfg_.height_samples = std::max(1, cfg_.height_samples);
    cfg_.max_cells_per_gain_eval = std::max(1, cfg_.max_cells_per_gain_eval);
}

bool ViewpointManager::generateBestViewpoints(const FrontierRecord &frontier,
                                              const ObservationMap &observation_map,
                                              const MapManager::Ptr &map_manager,
                                              const super_utils::Vec3f &robot_pos,
                                              const double current_yaw,
                                              const double stamp,
                                              std::vector<ExplorationViewpoint> &viewpoints,
                                              ExplorationViewpoint &best_viewpoint) const {
    viewpoints.clear();
    best_viewpoint = ExplorationViewpoint{};
    if (frontier.cells.empty()) {
        return false;
    }

    super_utils::Vec3f normal = frontier.normal.norm() > 1.0e-6
                                ? frontier.normal.normalized()
                                : robot_pos - frontier.center;
    if (normal.norm() < 1.0e-6) {
        normal = super_utils::Vec3f::UnitX();
    } else {
        normal.normalize();
    }
    int viewpoint_id = 0;
    double best_score = -std::numeric_limits<double>::infinity();
    const double radius_step =
            cfg_.radius_samples <= 1 ? 0.0 :
            (cfg_.max_distance - cfg_.min_distance) / static_cast<double>(cfg_.radius_samples - 1);
    const int height_mid = cfg_.height_samples / 2;

    for (int ri = 0; ri < cfg_.radius_samples; ++ri) {
        const double radius = cfg_.min_distance + radius_step * static_cast<double>(ri);
        for (int hi = 0; hi < cfg_.height_samples; ++hi) {
            const double dz = static_cast<double>(hi - height_mid) * cfg_.height_step;
            for (int yi = 0; yi < cfg_.yaw_samples; ++yi) {
                const double theta = 2.0 * M_PI * static_cast<double>(yi) /
                                     static_cast<double>(cfg_.yaw_samples);
                super_utils::Vec3f radial(std::cos(theta), std::sin(theta), 0.0);
                if (radial.dot(normal) < -0.2) {
                    radial = -radial;
                }
                super_utils::Vec3f pos = frontier.center + radius * radial;
                pos.z() += dz;

                bool local_safe = false;
                double surface_distance = 0.0;
                if (!viewpointSafe(pos, observation_map, map_manager, local_safe, surface_distance)) {
                    continue;
                }

                ExplorationViewpoint candidate;
                candidate.frontier_id = frontier.stable_id;
                candidate.viewpoint_id = viewpoint_id++;
                candidate.position = pos;
                candidate.local_safe = local_safe;
                candidate.global_safe = true;
                candidate.distance_to_surface = surface_distance;
                candidate.last_checked_time = stamp;

                double best_gain = 0.0;
                candidate.yaw = bestYawForViewpoint(frontier, pos, observation_map, best_gain);
                candidate.gain_raw = best_gain;
                candidate.gain_norm = std::min(1.0, best_gain / std::max(1.0, static_cast<double>(frontier.cell_count)));
                candidate.reachable = local_safe || candidate.global_safe;
                if (candidate.gain_raw < cfg_.min_gain) {
                    continue;
                }

                const double yaw_cost = std::abs(wrapYaw(candidate.yaw - current_yaw));
                const double distance_cost = (candidate.position - robot_pos).norm();
                const double score = candidate.gain_raw - 0.2 * distance_cost - 0.1 * yaw_cost;
                viewpoints.push_back(candidate);
                if (score > best_score) {
                    best_score = score;
                    best_viewpoint = candidate;
                }
            }
        }
    }

    return best_viewpoint.viewpoint_id >= 0;
}

bool ViewpointManager::viewpointSafe(const super_utils::Vec3f &position,
                                     const ObservationMap &observation_map,
                                     const MapManager::Ptr &map_manager,
                                     bool &local_safe,
                                     double &surface_distance) const {
    local_safe = false;
    surface_distance = observation_map.nearestSurfaceDistance(position,
                                                             std::max(cfg_.safe_distance * 2.0, 1.0));
    if (surface_distance < cfg_.safe_distance) {
        return false;
    }
    if (!cfg_.use_local_map_safety) {
        return true;
    }
    if (map_manager != nullptr && map_manager->insideLocalMap(position)) {
        const auto grid_type = map_manager->getGridType(position);
        const auto inf_type = map_manager->getInfGridType(position);
        local_safe = grid_type != rog_map::GridType::OCCUPIED &&
                     grid_type != rog_map::GridType::OUT_OF_MAP &&
                     inf_type != rog_map::GridType::OCCUPIED &&
                     inf_type != rog_map::GridType::OUT_OF_MAP;
        return local_safe;
    }
    return true;
}

double ViewpointManager::evaluateGain(const FrontierRecord &frontier,
                                      const ExplorationViewpoint &candidate,
                                      const ObservationMap &observation_map) const {
    double gain = 0.0;
    const int step = std::max(1, static_cast<int>(std::ceil(
            static_cast<double>(frontier.cells.size()) /
            static_cast<double>(cfg_.max_cells_per_gain_eval))));
    const double half_hfov = cfg_.horizontal_fov_deg * M_PI / 360.0;
    const double half_vfov = cfg_.vertical_fov_deg * M_PI / 360.0;
    const super_utils::Vec3f yaw_dir(std::cos(candidate.yaw), std::sin(candidate.yaw), 0.0);

    for (std::size_t i = 0; i < frontier.cells.size(); i += static_cast<std::size_t>(step)) {
        const super_utils::Vec3f &cell = frontier.cells[i];
        const super_utils::Vec3f to_cell = cell - candidate.position;
        const double dist = to_cell.norm();
        if (dist < 1.0e-6 || dist > cfg_.sensor_range) {
            continue;
        }
        const super_utils::Vec3f dir = to_cell / dist;
        const double horizontal_angle = std::atan2(dir.y(), dir.x()) - candidate.yaw;
        if (std::abs(wrapYaw(horizontal_angle)) > half_hfov) {
            continue;
        }
        const double vertical_angle = std::asin(std::max(-1.0, std::min(1.0, dir.z())));
        if (std::abs(vertical_angle) > half_vfov) {
            continue;
        }
        if (i < frontier.normals.size()) {
            const super_utils::Vec3f from_cell_to_view = -dir;
            if (frontier.normals[i].dot(from_cell_to_view) < cfg_.normal_dot_min) {
                continue;
            }
        }
        if (!observation_map.lineOfSightFree(candidate.position,
                                             cell,
                                             cfg_.safe_distance * 0.5,
                                             cfg_.line_of_sight_step)) {
            continue;
        }
        super_utils::Vec3f horizontal_dir(dir.x(), dir.y(), 0.0);
        if (horizontal_dir.norm() > 1.0e-6) {
            horizontal_dir.normalize();
        } else {
            horizontal_dir = yaw_dir;
        }
        const double facing = std::max(0.0, yaw_dir.dot(horizontal_dir));
        gain += 1.0 + 0.2 * facing;
    }
    return gain;
}

double ViewpointManager::bestYawForViewpoint(const FrontierRecord &frontier,
                                             const super_utils::Vec3f &position,
                                             const ObservationMap &observation_map,
                                             double &best_gain) const {
    double best_yaw = std::atan2(frontier.center.y() - position.y(),
                                frontier.center.x() - position.x());
    best_gain = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < cfg_.yaw_samples; ++i) {
        ExplorationViewpoint candidate;
        candidate.position = position;
        candidate.yaw = -M_PI + 2.0 * M_PI * static_cast<double>(i) /
                                  static_cast<double>(cfg_.yaw_samples);
        const double gain = evaluateGain(frontier, candidate, observation_map);
        if (gain > best_gain) {
            best_gain = gain;
            best_yaw = candidate.yaw;
        }
    }
    if (!std::isfinite(best_gain)) {
        best_gain = 0.0;
    }
    return wrapYaw(best_yaw);
}

}  // namespace exploration
}  // namespace general_planner
