#include "general_core/tracking/tracking_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace general_planner {
using general_utils::Vec3f;
using general_utils::vec_E;

TrackingFrontend::TrackingFrontend(const Config &cfg, const MapManager::Ptr &map)
    : cfg_(cfg), map_manager_(map) {}

bool TrackingFrontend::safe(const Vec3f &p) const {
    if (!p.allFinite()) return false;
    if (!map_manager_ || !map_manager_->ready()) return true;
    const auto type = map_manager_->getInfGridType(p);
    if (type == general_utils::OCCUPIED || type == general_utils::OUT_OF_MAP) return false;
    if (cfg_.unknown_as_occupied && type != general_utils::KNOWN_FREE) return false;
    if (!map_manager_->hasESDF()) return true;
    double distance=0.0; Vec3f gradient;
    return map_manager_->evaluateESDF(p,distance,gradient) && distance>=cfg_.safe_distance;
}

bool TrackingFrontend::visible(const Vec3f &p, const Vec3f &target) const {
    if (!safe(p)) return false;
    if (!map_manager_ || !map_manager_->ready()) return true;
    // Visibility uses raw occupancy; vehicle clearance belongs to the flight
    // corridor. End at the target's physical surface: its occupied body is
    // not an occluder between the camera and the observed target.
    const Vec3f ray = target - p;
    const double distance = ray.norm();
    if (distance < 1.e-6) return false;
    const Vec3f direction = ray / distance;
    const Vec3f half_size(std::max(0.01, cfg_.target_half_width),
                         std::max(0.01, cfg_.target_half_width),
                         std::max(0.01, cfg_.target_half_height));
    double surface_distance = distance;
    for (int axis = 0; axis < 3; ++axis)
        if (std::abs(direction(axis)) > 1.e-6)
            surface_distance = std::min(surface_distance, half_size(axis) / std::abs(direction(axis)));
    const Vec3f end = target - direction * std::min(0.9 * distance,
        surface_distance + map_manager_->getResolution());
    return map_manager_->isLineFree(p, end, false, cfg_.unknown_as_occupied);
}

bool TrackingFrontend::connectRegion(const Vec3f &start,
                                     const traj_opt::DynamicTargetState &target,
                                     const Vec3f &preferred, vec_E<Vec3f> &path,
                                     const std::chrono::steady_clock::time_point &deadline) const {
    path.clear();
    const auto line = [&](const Vec3f &a, const Vec3f &b) {
        return !map_manager_ || !map_manager_->ready() ||
            map_manager_->isLineFree(a, b, true, cfg_.unknown_as_occupied);
    };
    if (visible(preferred, target.position) && line(start, preferred)) {
        path = {start, preferred}; return true;
    }
    if (!cfg_.use_astar || !safe(start)) return false;
    const double step = map_manager_ && map_manager_->ready()
        ? std::max(0.15, map_manager_->getInfResolution()) : 0.25;
    const double lower = std::max(0.2, cfg_.tracking_distance - cfg_.distance_lower_tolerance);
    const double upper = cfg_.tracking_distance + cfg_.distance_upper_tolerance;
    const double height = target.position.z() + cfg_.height_offset;
    const auto heuristic = [&](const Vec3f &p) {
        const double d = (p - target.position).head<2>().norm();
        return std::hypot(std::max({0.0, lower - d, d - upper}),
                          std::max(0.0, std::abs(p.z() - height) - cfg_.height_tolerance));
    };
    struct Key { int x, y, z; bool operator==(const Key &b) const { return x==b.x && y==b.y && z==b.z; } };
    struct Hash { std::size_t operator()(const Key &k) const {
        return std::hash<int>{}(k.x) ^ (std::hash<int>{}(k.y) << 1) ^ (std::hash<int>{}(k.z) << 2);
    }};
    struct Node { Key key; double g; int parent; };
    struct Entry { double f, g; int id; bool operator<(const Entry &b) const { return f > b.f; } };
    std::vector<Node> nodes;
    nodes.reserve(4096);
    std::unordered_map<Key, int, Hash> visited;
    std::priority_queue<Entry> queue;
    const auto point = [&](const Key &k) { return Vec3f(start + step * Vec3f(k.x, k.y, k.z)); };
    nodes.push_back({{0,0,0}, 0.0, -1}); visited.emplace(nodes[0].key, 0);
    queue.push({heuristic(start), 0.0, 0});
    while (!queue.empty() && nodes.size() < 8192) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        const Entry entry = queue.top(); queue.pop();
        const Node node = nodes[entry.id];
        if (entry.g > node.g + 1.e-9) continue;
        const Vec3f p = point(node.key);
        if (heuristic(p) < 1.e-6 && visible(p, target.position)) {
            for (int id = entry.id; id >= 0; id = nodes[id].parent) path.push_back(point(nodes[id].key));
            std::reverse(path.begin(), path.end()); return true;
        }
        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
            if (dx==0 && dy==0 && dz==0) continue;
            Key key{node.key.x+dx, node.key.y+dy, node.key.z+dz};
            const Vec3f q = point(key);
            if ((q-start).norm() > cfg_.searching_horizon || !safe(q) || !line(p,q)) continue;
            const double g = node.g + step * std::sqrt(dx*dx+dy*dy+dz*dz);
            auto found = visited.find(key);
            int id;
            if (found == visited.end()) {
                id = nodes.size(); visited.emplace(key,id); nodes.push_back({key,g,entry.id});
            } else {
                id = found->second;
                if (g >= nodes[id].g) continue;
                nodes[id].g=g; nodes[id].parent=entry.id;
            }
            queue.push({g + heuristic(q) + 0.02*(q-preferred).norm(), g, id});
        }
    }
    return false;
}

traj_opt::TrackingVisibleRegion TrackingFrontend::visibleRegion(
        const traj_opt::DynamicTargetState &target, const Vec3f &seed) const {
    traj_opt::TrackingVisibleRegion region;
    region.t = target.t; region.target_position = target.position;
    region.visible_point = seed;
    const Vec3f delta = seed - target.position;
    const double radius = delta.head<2>().norm();
    if (radius < 0.1 || !visible(seed, target.position)) return region;
    const double angle = std::atan2(delta.y(), delta.x());
    constexpr int rays = 32;
    constexpr double pi = 3.14159265358979323846;
    const double step = pi / rays;
    double left = angle, right = angle;
    const auto point = [&](double a) { return Vec3f(target.position + Vec3f(radius*std::cos(a), radius*std::sin(a),delta.z())); };
    for (int i=1; i<=rays && visible(point(angle-i*step),target.position); ++i) left=angle-i*step;
    for (int i=1; i<=rays && visible(point(angle+i*step),target.position); ++i) right=angle+i*step;
    region.visible_point = point(0.5*(left+right));
    region.theta = 0.5*(right-left);
    region.confidence = std::min(1.0,region.theta/std::max(0.05,cfg_.visibility_angle_clearance));
    region.valid = region.theta > 0.01;
    return region;
}

bool TrackingFrontend::buildProblem(const general_utils::StatePVAJ &head,
                                    const traj_opt::DynamicTargetStates &prediction,
                                    traj_opt::TrackingProblem &problem,
                                    const Vec3f *reference_viewpoint,
                                    const traj_opt::DynamicTargetState *reference_target) const {
    problem = traj_opt::TrackingProblem{};
    if (!head.allFinite() || prediction.size()<2 || !safe(head.col(0))) return false;
    for (std::size_t i=0;i<prediction.size();++i)
        if (!prediction[i].position.allFinite() || !prediction[i].velocity.allFinite() ||
            !std::isfinite(prediction[i].t) || (i && prediction[i].t<=prediction[i-1].t)) return false;
    const double trusted = prediction.back().t;
    if (trusted < 0.15) return false;
    const double horizon = std::min(std::max(0.8,cfg_.nominal_horizon), trusted+std::max(0.0,cfg_.max_extrapolation));
    problem.head_pvaj=head; problem.trusted_horizon=trusted;
    problem.safe_distance=cfg_.safe_distance;
    problem.tracking_distance=cfg_.tracking_distance;
    problem.distance_tolerance=cfg_.distance_tolerance;
    problem.height_offset=cfg_.height_offset; problem.height_tolerance=cfg_.height_tolerance;
    problem.target_half_width=cfg_.target_half_width; problem.target_half_height=cfg_.target_half_height;
    problem.od_h_lower=std::max(0.1,cfg_.tracking_distance-cfg_.distance_lower_tolerance);
    problem.od_h_upper=cfg_.tracking_distance+cfg_.distance_upper_tolerance;
    problem.od_v_lower=cfg_.height_offset-cfg_.height_tolerance;
    problem.od_v_upper=cfg_.height_offset+cfg_.height_tolerance;
    problem.use_visible_region=cfg_.use_visible_region;
    problem.reacquire_mode=(head.col(0)-prediction.front().position).head<2>().norm()>problem.od_h_upper;
    problem.closing_gain=cfg_.closing_gain; problem.max_closing_speed=cfg_.max_closing_speed;
    // Extrapolation is explicit, bounded and downweighted by the cost manager.
    const int count=std::max(2,static_cast<int>(std::ceil(horizon/std::max(0.1,cfg_.sample_dt))));
    for (int i=0;i<=count;++i) problem.target_prediction.push_back(traj_opt::sampleTrackingTarget(prediction,horizon*i/count));
    problem.guide_path.push_back(head.col(0));
    problem.guide_t.push_back(0.0);
    Vec3f seed=head.col(0);
    if (head.col(1).norm() > 0.1) {
        const double dt = std::min(0.2, horizon / count);
        const Vec3f forward = seed + dt * head.col(1) + 0.5 * dt * dt * head.col(2);
        if (!safe(forward) || (map_manager_ && map_manager_->ready() &&
            !map_manager_->isLineFree(seed, forward, true, cfg_.unknown_as_occupied))) return false;
        problem.guide_path.push_back(forward);
        seed = forward;
    }
    Vec3f bearing=seed-prediction.front().position;
    if (reference_viewpoint && reference_target && reference_viewpoint->allFinite())
        bearing=*reference_viewpoint-reference_target->position;
    bearing.z()=0.0;
    if (bearing.norm()<1.e-4) bearing=Vec3f(-1,0,0);
    bearing.normalize();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(0.001,cfg_.search_budget_seconds)));
    for (int i=1;i<=count;++i) {
        const auto &target=problem.target_prediction[i];
        Vec3f preferred=target.position+cfg_.tracking_distance*bearing;
        preferred.z()=target.position.z()+cfg_.height_offset;
        vec_E<Vec3f> segment;
        if (!connectRegion(seed,target,preferred,segment,deadline)) return false;
        for (std::size_t j=1;j<segment.size();++j)
            if ((segment[j]-problem.guide_path.back()).norm()>1.e-4) problem.guide_path.push_back(segment[j]);
        seed=segment.back();
        problem.viewpoints.push_back(seed); problem.target_sample_times.push_back(target.t);
        if (cfg_.use_visible_region) problem.visible_regions.push_back(visibleRegion(target,seed));
        bearing=seed-target.position; bearing.z()=0.0;
        if (bearing.norm()>1.e-4) bearing.normalize(); else bearing=Vec3f(-1,0,0);
        if (std::chrono::steady_clock::now()>=deadline) return false;
    }
    // A distant target need not be reached in this horizon. Bound the geometric
    // prefix while preserving the original observation horizon and moving tail.
    const double budget=std::min(cfg_.max_speed*horizon,
        head.col(1).norm()*horizon+0.35*cfg_.max_acc*horizon*horizon);
    vec_E<Vec3f> path{head.col(0)};
    double length=0.0;
    for (std::size_t i=1;i<problem.guide_path.size() && length<budget;++i) {
        const Vec3f d=problem.guide_path[i]-path.back();
        const double distance=d.norm(); if (distance<1.e-6) continue;
        const double travel=std::min(distance,budget-length);
        path.push_back(path.back()+d*(travel/distance)); length+=travel;
    }
    if (path.size()==1) path.push_back(path.front());
    problem.guide_path=std::move(path); problem.guide_t.assign(problem.guide_path.size(),0.0);
    double arc=0.0;
    for (std::size_t i=1;i<problem.guide_path.size();++i) {
        arc+=(problem.guide_path[i]-problem.guide_path[i-1]).norm();
        problem.guide_t[i]=length>1.e-6 ? horizon*arc/length : horizon*i/(problem.guide_path.size()-1);
    }
    problem.tail_pvaj.setZero(); problem.tail_pvaj.col(0)=problem.guide_path.back();
    Vec3f velocity=problem.target_prediction.back().velocity;
    const double distance=(problem.tail_pvaj.col(0)-problem.target_prediction.back().position).head<2>().norm();
    if (distance>problem.od_h_upper && length>1.e-6) {
        const Vec3f direction=(problem.guide_path.back()-problem.guide_path[problem.guide_path.size()-2]).normalized();
        velocity=direction*std::min(cfg_.max_speed,velocity.norm()+std::min(cfg_.max_closing_speed,
                     cfg_.closing_gain*(distance-cfg_.tracking_distance)));
    }
    // A velocity reachable under acceleration alone can still require more
    // distance than this prefix once jerk and a zero-acceleration tail matter.
    // Use the symmetric jerk-limited speed transition, reserving 10% for the
    // smooth polynomial joins. This only seeds the terminal boundary; the
    // optimizer and commit still enforce the complete dynamic constraints.
    const double initial_speed = velocity.norm() > 1.e-6
        ? std::max(0.0, head.col(1).dot(velocity.normalized())) : head.col(1).norm();
    const double acceleration = std::max(0.1, 0.9 * cfg_.max_acc);
    const double jerk = std::max(0.1, 0.9 * cfg_.max_jerk);
    // A bang-bang transition alone is too aggressive for the fixed PVAJ
    // polynomial boundary. A quintic velocity ramp has derivative maxima
    // 1.875 * dv / T and (10/sqrt(3)) * dv / T^2.
    double lower = std::min(initial_speed, cfg_.max_speed);
    double upper = std::min({cfg_.max_speed,
        initial_speed + acceleration * horizon / 1.875,
        initial_speed + jerk * horizon * horizon / (10.0 / std::sqrt(3.0))});
    for (int i = 0; i < 24; ++i) {
        const double speed = 0.5 * (lower + upper);
        const double change = std::max(0.0, speed - initial_speed);
        const double transition = change < acceleration * acceleration / jerk
            ? 2.0 * std::sqrt(change / jerk) : change / acceleration + acceleration / jerk;
        if (transition <= horizon && 0.5 * (initial_speed + speed) * transition <= length)
            lower = speed;
        else upper = speed;
    }
    const double reachable=lower;
    if (velocity.norm()>reachable && velocity.norm()>1.e-6) velocity*=reachable/velocity.norm();
    problem.tail_pvaj.col(1)=velocity;
    problem.min_total_duration=horizon;
    problem.max_total_duration=horizon+0.8;
    return true;
}

} // namespace general_planner
