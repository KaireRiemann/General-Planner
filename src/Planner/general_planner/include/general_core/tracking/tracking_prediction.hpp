#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

#include <traj_opt/tracking_problem.hpp>
#include <utils/header/eigen_alias.hpp>

namespace general_planner {
struct TrackingPredictionSettings {
    double dt{0.2};
    double horizon{3.0};
    double accel{3.0};
    double vmax{4.0};
    double rho_accel{1.0};
    double max_time{0.1};
    double height_offset{1.0};
};

inline void buildConstantVelocityTrackingPrediction(
        const general_utils::Vec3f &p,
        const general_utils::Vec3f &v,
        const double pose_yaw,
        const TrackingPredictionSettings &cfg,
        traj_opt::DynamicTargetStates &prediction) {
    const double dt = std::max(0.05, cfg.dt);
    const double horizon = std::max(dt, cfg.horizon);
    const int sample_num = std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);
    const double base_yaw = v.head<2>().norm() > 1.0e-3 ? std::atan2(v.y(), v.x()) : pose_yaw;
    prediction.clear();
    prediction.reserve(static_cast<std::size_t>(sample_num));
    for (int i = 0; i < sample_num; ++i) {
        const double t = static_cast<double>(i) * dt;
        traj_opt::DynamicTargetState target;
        target.t = t;
        target.position = p + v * t;
        target.velocity = v;
        target.acceleration = general_utils::Vec3f::Zero();
        target.yaw = base_yaw;
        target.yaw_rate = 0.0;
        prediction.emplace_back(target);
    }
}

// Elastic-Tracker greedy xy acceleration search: a ∈ {-A,0,A}², z held, occupied
// rejected. Falls back to constant velocity when the search times out.
inline bool buildKinodynamicTrackingPrediction(
        const general_utils::Vec3f &p,
        const general_utils::Vec3f &v,
        const double pose_yaw,
        const TrackingPredictionSettings &cfg,
        const std::function<bool(const general_utils::Vec3f &, const general_utils::Vec3f &)> &is_valid,
        traj_opt::DynamicTargetStates &prediction) {
    const double dt = std::max(0.05, cfg.dt);
    const double horizon = std::max(dt, cfg.horizon);
    const double acc = std::max(0.0, cfg.accel);
    if (acc <= 1.0e-6 || !is_valid) return false;

    struct PredictNode {
        general_utils::Vec3f p{general_utils::Vec3f::Zero()};
        general_utils::Vec3f v{general_utils::Vec3f::Zero()};
        general_utils::Vec3f a{general_utils::Vec3f::Zero()};
        double t{0.0};
        double score{0.0};
        int parent{-1};
    };

    std::vector<PredictNode> nodes;
    nodes.reserve(512);
    nodes.push_back(PredictNode{p, v, general_utils::Vec3f::Zero(), 0.0, 0.0, -1});
    const general_utils::Vec3f nominal_end = p + v * horizon;
    auto heuristic = [&](const PredictNode &node) {
        return 0.001 * (node.p - nominal_end).norm();
    };
    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_set;
    int cur = 0;
    const double dt2_2 = 0.5 * dt * dt;
    const double max_wall_time = std::max(0.001, cfg.max_time);
    const auto start_wall = std::chrono::steady_clock::now();
    const int max_nodes = 1 << 16;
    const std::array<double, 3> acc_samples{-acc, 0.0, acc};

    while (nodes[static_cast<std::size_t>(cur)].t + 0.5 * dt < horizon) {
        const PredictNode cur_node = nodes[static_cast<std::size_t>(cur)];
        for (const double ax : acc_samples) {
            for (const double ay : acc_samples) {
                const general_utils::Vec3f input(ax, ay, 0.0);
                const general_utils::Vec3f next_p = cur_node.p + cur_node.v * dt + input * dt2_2;
                const general_utils::Vec3f next_v = cur_node.v + input * dt;
                if (cfg.vmax > 0.0 && next_v.norm() > cfg.vmax) continue;
                if (!is_valid(next_p, next_v)) continue;
                if (static_cast<int>(nodes.size()) >= max_nodes) return false;
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall).count() >
                    max_wall_time) {
                    return false;
                }
                PredictNode next;
                next.p = next_p;
                next.v = next_v;
                next.a = input;
                next.t = cur_node.t + dt;
                next.score = cur_node.score + cfg.rho_accel * input.norm();
                next.parent = cur;
                nodes.emplace_back(next);
                open_set.emplace(next.score + heuristic(next), static_cast<int>(nodes.size()) - 1);
            }
        }
        if (open_set.empty()) return false;
        cur = open_set.top().second;
        open_set.pop();
    }

    std::vector<int> ids;
    for (int id = cur; id >= 0; id = nodes[static_cast<std::size_t>(id)].parent)
        ids.emplace_back(id);
    std::reverse(ids.begin(), ids.end());
    if (ids.size() < 2) return false;

    prediction.clear();
    prediction.reserve(ids.size());
    double last_yaw = pose_yaw;
    for (const int id : ids) {
        const PredictNode &node = nodes[static_cast<std::size_t>(id)];
        traj_opt::DynamicTargetState target;
        target.t = node.t;
        target.position = node.p;
        target.velocity = node.v;
        target.acceleration = node.a;
        if (node.v.head<2>().norm() > 1.0e-3) {
            target.yaw = std::atan2(node.v.y(), node.v.x());
            target.yaw = last_yaw + std::atan2(std::sin(target.yaw - last_yaw),
                                               std::cos(target.yaw - last_yaw));
        } else {
            target.yaw = last_yaw;
        }
        target.yaw_rate = 0.0;
        last_yaw = target.yaw;
        prediction.emplace_back(target);
    }
    for (std::size_t i = 1; i < prediction.size(); ++i) {
        const double local_dt = std::max(0.05, prediction[i].t - prediction[i - 1].t);
        prediction[i - 1].yaw_rate = (prediction[i].yaw - prediction[i - 1].yaw) / local_dt;
    }
    prediction.back().yaw_rate = prediction.size() > 1
                                     ? prediction[prediction.size() - 2].yaw_rate
                                     : 0.0;
    return true;
}
} // namespace general_planner
