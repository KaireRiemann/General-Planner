#include "exploration/viewpoint_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include "exploration/topo_graph.hpp"

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
    cfg_.max_viewpoint_clusters = std::max(1, cfg_.max_viewpoint_clusters);
    cfg_.viewpoint_cluster_connectivity_scale =
            std::max(0.1, cfg_.viewpoint_cluster_connectivity_scale);
    cfg_.epic_yaw_bins = std::max(4, cfg_.epic_yaw_bins);
}

bool ViewpointManager::generateBestViewpoints(const FrontierRecord &frontier,
                                              const ObservationMap &observation_map,
                                              const MapManager::Ptr &map_manager,
                                              const EpicMapAdapter::Ptr &map_adapter,
                                              const TopoGraph *topo_graph,
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

    const double radius_step =
            cfg_.radius_samples <= 1 ? 0.0 :
            (cfg_.max_distance - cfg_.min_distance) / static_cast<double>(cfg_.radius_samples - 1);
    const int height_mid = cfg_.height_samples / 2;
    std::vector<ExplorationViewpoint> sampled_viewpoints;

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
                if (!viewpointSafe(pos,
                                   observation_map,
                                   map_manager,
                                   map_adapter,
                                   local_safe,
                                   surface_distance)) {
                    continue;
                }

                ExplorationViewpoint candidate;
                candidate.frontier_id = frontier.stable_id;
                candidate.position = pos;
                candidate.local_safe = local_safe;
                candidate.distance_to_surface = surface_distance;
                candidate.last_checked_time = stamp;

                double best_gain = 0.0;
                candidate.yaw = bestYawForViewpoint(frontier, pos, observation_map, best_gain);
                candidate.gain_raw = best_gain;
                candidate.gain_norm = std::min(1.0, best_gain / std::max(1.0, static_cast<double>(frontier.cell_count)));
                if (candidate.gain_raw < cfg_.min_gain) {
                    continue;
                }
                sampled_viewpoints.push_back(candidate);
            }
        }
    }

    if (sampled_viewpoints.empty()) {
        return false;
    }

    std::vector<std::vector<int>> clusters;
    if (cfg_.cluster_by_visibility_sphere) {
        std::vector<char> visited(sampled_viewpoints.size(), 0);
        for (int seed = 0; seed < static_cast<int>(sampled_viewpoints.size()); ++seed) {
            if (visited[static_cast<std::size_t>(seed)] != 0) {
                continue;
            }
            clusters.emplace_back();
            std::queue<int> queue;
            visited[static_cast<std::size_t>(seed)] = 1;
            queue.push(seed);
            while (!queue.empty()) {
                const int current = queue.front();
                queue.pop();
                clusters.back().push_back(current);
                const double current_radius =
                        std::max(cfg_.safe_distance,
                                 sampled_viewpoints[static_cast<std::size_t>(current)].distance_to_surface *
                                 cfg_.viewpoint_cluster_connectivity_scale);
                for (int candidate = 0; candidate < static_cast<int>(sampled_viewpoints.size()); ++candidate) {
                    if (visited[static_cast<std::size_t>(candidate)] != 0) {
                        continue;
                    }
                    const double candidate_radius =
                            std::max(cfg_.safe_distance,
                                     sampled_viewpoints[static_cast<std::size_t>(candidate)].distance_to_surface *
                                     cfg_.viewpoint_cluster_connectivity_scale);
                    const double connect_radius = std::min(current_radius, candidate_radius);
                    if ((sampled_viewpoints[static_cast<std::size_t>(current)].position -
                         sampled_viewpoints[static_cast<std::size_t>(candidate)].position).norm() <= connect_radius) {
                        visited[static_cast<std::size_t>(candidate)] = 1;
                        queue.push(candidate);
                    }
                }
            }
        }
    } else {
        for (int i = 0; i < static_cast<int>(sampled_viewpoints.size()); ++i) {
            clusters.push_back({i});
        }
    }

    std::vector<int> representatives;
    representatives.reserve(clusters.size());
    for (const auto &cluster : clusters) {
        if (cluster.empty()) {
            continue;
        }
        int best_idx = cluster.front();
        for (const int idx : cluster) {
            const auto &candidate = sampled_viewpoints[static_cast<std::size_t>(idx)];
            const auto &best_candidate = sampled_viewpoints[static_cast<std::size_t>(best_idx)];
            if (candidate.distance_to_surface > best_candidate.distance_to_surface + 1.0e-6 ||
                (std::abs(candidate.distance_to_surface - best_candidate.distance_to_surface) < 1.0e-6 &&
                 candidate.gain_raw > best_candidate.gain_raw)) {
                best_idx = idx;
            }
        }
        representatives.push_back(best_idx);
    }

    std::sort(representatives.begin(),
              representatives.end(),
              [&sampled_viewpoints](const int lhs, const int rhs) {
                  const auto &a = sampled_viewpoints[static_cast<std::size_t>(lhs)];
                  const auto &b = sampled_viewpoints[static_cast<std::size_t>(rhs)];
                  if (std::abs(a.gain_raw - b.gain_raw) > 1.0e-6) {
                      return a.gain_raw > b.gain_raw;
                  }
                  return a.distance_to_surface > b.distance_to_surface;
              });
    if (static_cast<int>(representatives.size()) > cfg_.max_viewpoint_clusters) {
        representatives.resize(static_cast<std::size_t>(cfg_.max_viewpoint_clusters));
    }

    int viewpoint_id = 0;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const int idx : representatives) {
        ExplorationViewpoint candidate = sampled_viewpoints[static_cast<std::size_t>(idx)];
        double topo_cost = std::numeric_limits<double>::infinity();
        const bool topo_ok = topoReachable(topo_graph, candidate.position, topo_cost);
        candidate.topo_cost = topo_cost;
        candidate.reachable = topo_ok || candidate.local_safe;
        candidate.global_safe = topo_ok;
        if (!candidate.reachable) {
            continue;
        }
        candidate.viewpoint_id = viewpoint_id++;
        const double yaw_cost = std::abs(wrapYaw(candidate.yaw - current_yaw));
        const double distance_cost = std::isfinite(topo_cost) ? topo_cost : (candidate.position - robot_pos).norm();
        const double score = candidate.gain_raw - 0.2 * distance_cost - 0.1 * yaw_cost;
        viewpoints.push_back(candidate);
        if (score > best_score) {
            best_score = score;
            best_viewpoint = candidate;
        }
    }

    return best_viewpoint.viewpoint_id >= 0;
}

bool ViewpointManager::viewpointSafe(const super_utils::Vec3f &position,
                                     const ObservationMap &observation_map,
                                     const MapManager::Ptr &map_manager,
                                     const EpicMapAdapter::Ptr &map_adapter,
                                     bool &local_safe,
                                     double &surface_distance) const {
    local_safe = false;
    surface_distance = observation_map.nearestSurfaceDistance(position,
                                                             std::max(cfg_.safe_distance * 2.0, 1.0));
    if (map_adapter != nullptr) {
        const double map_distance = map_adapter->getDisToOcc(position);
        if (std::isfinite(map_distance) && map_distance > 0.0) {
            surface_distance = std::min(surface_distance, map_distance);
        }
        if (map_manager != nullptr && map_manager->hasPointCloudMap() &&
            (!map_adapter->isInBox(position.cast<float>()) ||
             !map_adapter->isInMap(position.cast<float>()))) {
            return false;
        }
    }
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

bool ViewpointManager::topoReachable(const TopoGraph *topo_graph,
                                     const super_utils::Vec3f &position,
                                     double &topo_cost) const {
    topo_cost = std::numeric_limits<double>::infinity();
    if (!cfg_.use_topo_reachability_filter || topo_graph == nullptr) {
        return true;
    }
    return topo_graph->routeToPosition(position, topo_cost, cfg_.topo_reachability_timeout);
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
        if (i < frontier.cell_states.size() &&
            frontier.cell_states[i] == ObservationCellState::FRONTIER_DIR &&
            dist > cfg_.min_distance + 0.5 * (cfg_.max_distance - cfg_.min_distance)) {
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
    const int yaw_bins = std::max(cfg_.epic_yaw_bins, cfg_.yaw_samples);
    for (int i = 0; i < yaw_bins; ++i) {
        ExplorationViewpoint candidate;
        candidate.position = position;
        candidate.yaw = -M_PI + 2.0 * M_PI * static_cast<double>(i) /
                                  static_cast<double>(yaw_bins);
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
