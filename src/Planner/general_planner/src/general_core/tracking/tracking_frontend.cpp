// Adapted 2026-09-20 from Elastic-Tracker env/env.hpp (GPL-3.0).
// Jialin Ji, Neng Pan, Fei Gao / ZJU FAST Lab.
// https://github.com/KaireRiemann/Elastic-Tracker/tree/0a302a2a9cc8b733e74941fdd4af3fb449447bea
#include <general_core/tracking/tracking_frontend.hpp>
#include <general_core/tracking/tracking_map_query.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace general_planner {
namespace {
constexpr double pi = 3.14159265358979323846;
using Key = std::array<int, 3>;
struct KeyHash {
    std::size_t operator()(const Key &k) const {
        std::size_t h = 0;
        for (int v : k) h ^= std::hash<int>{}(v) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
struct Node { Key key; double g; int parent; bool closed{false}; };
struct Entry {
    double f, g;
    int id;
    bool operator<(const Entry &rhs) const { return f > rhs.f; }
};
}
TrackingFrontend::TrackingFrontend(const Config &cfg, const MapManager::Ptr &map)
    : cfg_(cfg), map_manager_(map) {}

bool TrackingFrontend::safe(const Vec &p) const {
    if (!p.allFinite()) return false;
    if (!map_manager_) return true;
    return !trackingInflatedOccupied(map_manager_, p, cfg_.unknown_as_occupied);
}
bool TrackingFrontend::lineFree(const Vec &a, const Vec &b) const {
    return trackingSeedLineFree(map_manager_, a, b, cfg_.unknown_as_occupied);
}
bool TrackingFrontend::visible(const Vec &p, const Vec &center) const {
    return safe(p) && trackingVisibilityRayFree(map_manager_, p, center, cfg_.unknown_as_occupied);
}
bool TrackingFrontend::search(const Vec &start, const Vec &goal, bool ring,
                              Path &path, const Deadline &deadline) const {
    path.clear();
    if (!safe(start)) return false;
    const double resolution = map_manager_ ? map_manager_->getInfResolution() : 0.15;
    const auto reached = [&](const Vec &p) {
        if (!ring) return (p-goal).norm() < 1.8*resolution && lineFree(p, goal);
        const Eigen::Vector2d radial=(p-goal).head<2>();
        if (std::abs(radial.norm()-cfg_.tracking_distance) > std::max(cfg_.distance_tolerance,resolution) ||
            std::abs(p.z()-goal.z()) > std::max(cfg_.height_tolerance,resolution) ||
            radial.norm()<1.e-6 || !visible(p,goal)) return false;
        Vec projected=goal;
        projected.head<2>()+=cfg_.tracking_distance*radial.normalized();
        // Visibility regions are swept on the nominal ring. A point within
        // the distance tolerance can see around a wall while its projection
        // cannot; keep searching instead of creating an invalid fan seed.
        return visible(projected,goal);
    };
    if (reached(start)) { path = {start}; if (!ring) path.push_back(goal); return true; }
    Vec direct = goal;
    if (ring) {
        Eigen::Vector2d direction = (start-goal).head<2>();
        if (direction.norm() < 1.e-6) direction = Eigen::Vector2d(-1, 0);
        direct.head<2>() += cfg_.tracking_distance*direction.normalized();
    }
    if (lineFree(start, direct) && (!ring || visible(direct, goal))) {
        path = {start, direct}; return true;
    }
    const auto key = [&](const Vec &p) {
        const auto index = (p/resolution).array().floor().cast<int>().eval();
        return Key{index.x(), index.y(), index.z()};
    };
    const auto position = [&](const Key &k) {
        return Vec((k[0]+0.5)*resolution, (k[1]+0.5)*resolution, (k[2]+0.5)*resolution);
    };
    const auto heuristic = [&](const Vec &p) {
        Vec delta = goal-p;
        if (ring) {
            const double distance = delta.head<2>().norm();
            if (distance > 1.e-9) delta.head<2>() *= 1.0-cfg_.tracking_distance/distance;
            else delta.x() = cfg_.tracking_distance;
        }
        return delta.cwiseAbs().sum();
    };
    std::unordered_map<Key, int, KeyHash> ids;
    std::vector<Node> nodes;
    nodes.reserve(4096);
    std::priority_queue<Entry> open;
    const Key first = key(start);
    nodes.push_back({first, 0., -1}); ids.emplace(first, 0);
    open.push({heuristic(start), 0., 0});
    int found = -1;
    while (!open.empty() && nodes.size() < (1U << 18)) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        const Entry entry = open.top(); open.pop();
        if (nodes[entry.id].closed || entry.g != nodes[entry.id].g) continue;
        nodes[entry.id].closed = true;
        // Copy before expanding: insertion can reallocate the vector.
        const Node current = nodes[entry.id];
        const Vec p = entry.id == 0 ? start : position(current.key);
        if (reached(p)) { found = entry.id; break; }
        for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
            Key neighbor = current.key; neighbor[axis] += sign;
            const Vec next = position(neighbor);
            if (!safe(next)) continue;
            const double g = current.g+(next-p).norm();
            auto it = ids.find(neighbor);
            int id;
            if (it == ids.end()) {
                id = static_cast<int>(nodes.size());
                ids.emplace(neighbor, id); nodes.push_back({neighbor, g, entry.id});
            } else {
                id = it->second;
                if (g >= nodes[id].g) continue;
                nodes[id].g = g; nodes[id].parent = entry.id; nodes[id].closed = false;
            }
            // Immutable entries preserve heap ordering when g decreases.
            open.push({g+heuristic(next), g, id});
        }
    }
    if (found < 0) return false;
    for (int id = found; id >= 0; id = nodes[id].parent)
        path.push_back(id == 0 ? start : position(nodes[id].key));
    std::reverse(path.begin(), path.end());
    if (!ring) path.push_back(goal);
    return true;
}
bool TrackingFrontend::visibleRegion(const Vec &center, Vec &seed,
                                     traj_opt::TrackingVisibleRegion &region,
                                     const Deadline &deadline) const {
    const double theta0 = std::atan2(seed.y()-center.y(), seed.x()-center.x());
    const double resolution = map_manager_ ? map_manager_->getResolution() : 0.15;
    const double step = std::clamp(resolution/(2.0*cfg_.tracking_distance), 0.005, 0.1);
    const auto point = [&](double angle) {
        return Vec(center+Vec(cfg_.tracking_distance*std::cos(angle),
                              cfg_.tracking_distance*std::sin(angle), 0.0));
    };
    if (!visible(point(theta0), center)) return false;
    double left = theta0, right = theta0;
    for (int sign : {-1, 1}) {
        double &edge = sign < 0 ? left : right;
        for (double offset = step; offset <= pi; offset += step) {
            if (std::chrono::steady_clock::now() >= deadline) return false;
            const double candidate = theta0+sign*offset;
            if (!visible(point(candidate), center)) break;
            edge = candidate;
        }
    }
    region.target_position = center;
    region.visible_point = point(0.5*(left+right));
    region.theta = 0.5*(right-left);
    region.valid = true; region.confidence = 1.0;
    const double clearance = std::min(region.theta, cfg_.visibility_angle_clearance);
    const double angle = std::clamp(theta0, left+clearance, right-clearance);
    const Vec adjusted = point(angle);
    if (safe(adjusted) && visible(adjusted, center)) seed = adjusted;
    return true;
}
void TrackingFrontend::pts2path(const Path &way_pts, Path &path,
                                const Deadline &deadline) const {
    path.clear();
    if (way_pts.empty()) return;
    path.push_back(way_pts.front());
    for (std::size_t i = 0; i + 1 < way_pts.size(); ++i) {
        const Vec &p0 = path.back();
        const Vec &p1 = way_pts[i + 1];
        if ((p1 - p0).norm() < 1.e-5) continue;
        if (!trackingSeedLineFree(map_manager_, p0, p1, cfg_.unknown_as_occupied, 1.5)) {
            Path short_path;
            if (search(p0, p1, false, short_path, deadline)) {
                for (std::size_t k = 1; k < short_path.size(); ++k) {
                    if ((short_path[k] - path.back()).norm() < 1.e-5) continue;
                    path.push_back(short_path[k]);
                }
                continue;
            }
        }
        path.push_back(p1);
    }
    if (path.size() < 2) {
        path.push_back(path.front());
    }
}
bool TrackingFrontend::buildProblem(const general_utils::StatePVAJ &head,
                                    const traj_opt::DynamicTargetStates &prediction,
                                    traj_opt::TrackingProblem &problem) const {
    problem = {};
    if (!head.allFinite() || prediction.size() < 2 || !safe(head.col(0)) ||
        !(cfg_.tracking_distance > 0.0) || !(cfg_.sample_dt > 0.0)) return false;
    for (std::size_t i = 0; i < prediction.size(); ++i) {
        const auto &state = prediction[i];
        if (!std::isfinite(state.t) || !state.position.allFinite() ||
            !state.velocity.allFinite() || (i && state.t <= prediction[i-1].t)) return false;
    }
    if (std::abs(prediction.front().t) > 1.e-6) return false;
    const double horizon = std::min(cfg_.nominal_horizon, prediction.back().t);
    if (horizon < cfg_.sample_dt) return false;
    const int count = std::max(1, static_cast<int>(std::ceil(horizon/cfg_.sample_dt)));
    const auto deadline = std::chrono::steady_clock::now()+
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.001, cfg_.search_budget_seconds)));
    Path seeds;
    Vec cursor = head.col(0);
    for (int i = 0; i <= count; ++i) {
        const double t = horizon*i/count;
        auto target = traj_opt::sampleTrackingTarget(prediction, t);
        target.t = t;
        const Vec center = target.position+Vec(0, 0, cfg_.height_offset);
        Path segment;
        if (!search(cursor, center, true, segment, deadline)) return false;
        cursor = segment.back();
        seeds.push_back(cursor);
        problem.target_prediction.push_back(target);
        problem.target_sample_times.push_back(t);
    }
    // Elastic drops the last visible seed/target before corridor generation.
    if (seeds.size() > 2) {
        seeds.pop_back();
        problem.target_prediction.pop_back();
        problem.target_sample_times.pop_back();
    }
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        traj_opt::TrackingVisibleRegion region;
        region.t = problem.target_sample_times[i];
        const Vec center = problem.target_prediction[i].position+Vec(0, 0, cfg_.height_offset);
        // Elastic generate_visible_regions never aborts planning. A missing
        // fan only drops the visibility cost for that sample.
        if (cfg_.use_visible_region)
            visibleRegion(center, seeds[i], region, deadline);
        problem.visible_regions.push_back(region);
    }
    Path way_pts = seeds;
    way_pts.insert(way_pts.begin(), head.col(0));
    pts2path(way_pts, problem.guide_path, deadline);
    problem.guide_t.clear();
    problem.guide_t.reserve(problem.guide_path.size());
    const double path_horizon = std::max(horizon, cfg_.nominal_horizon);
    for (std::size_t i = 0; i < problem.guide_path.size(); ++i)
        problem.guide_t.push_back(path_horizon * static_cast<double>(i) /
                                  std::max<std::size_t>(1, problem.guide_path.size() - 1));
    problem.head_pvaj = head;
    problem.tail_pvaj.col(0) = problem.guide_path.back();
    // Elastic pins the terminal velocity to the current target velocity.
    problem.tail_pvaj.col(1) = prediction.front().velocity;
    if (problem.tail_pvaj.col(1).norm() > cfg_.max_speed)
        problem.tail_pvaj.col(1) *= cfg_.max_speed/problem.tail_pvaj.col(1).norm();
    problem.viewpoints = seeds;
    problem.tracking_distance = cfg_.tracking_distance;
    problem.distance_tolerance = cfg_.distance_tolerance;
    problem.height_offset = cfg_.height_offset;
    problem.height_tolerance = cfg_.height_tolerance;
    problem.visibility_angle_clearance = cfg_.visibility_angle_clearance;
    problem.min_total_duration = cfg_.nominal_horizon;
    problem.trusted_horizon = horizon;
    problem.use_visible_region = cfg_.use_visible_region;
    return true;
}
} // namespace general_planner
