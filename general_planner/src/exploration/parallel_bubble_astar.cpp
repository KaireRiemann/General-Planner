#include "exploration/parallel_bubble_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace general_planner {
namespace exploration {

namespace {
struct QueueItem {
    double f{0.0};
    Eigen::Vector3i idx{Eigen::Vector3i::Zero()};
    bool operator<(const QueueItem &other) const {
        return f > other.f;
    }
};
}  // namespace

ParallelBubbleAstar::ParallelBubbleAstar(Config cfg, MapManager::Ptr map_manager)
        : cfg_(std::move(cfg)),
          map_manager_(std::move(map_manager)) {
    cfg_.resolution = std::max(0.05, cfg_.resolution);
    cfg_.safe_distance = std::max(0.0, cfg_.safe_distance);
    cfg_.lambda_heu = std::max(0.0, cfg_.lambda_heu);
    cfg_.max_nodes = std::max(16, cfg_.max_nodes);
}

int ParallelBubbleAstar::search(const super_utils::Vec3f &start,
                                const super_utils::Vec3f &goal,
                                super_utils::vec_E<super_utils::Vec3f> &path,
                                const double timeout,
                                const bool only_raycast,
                                const super_utils::Vec3f &bbox_min,
                                const super_utils::Vec3f &bbox_max) const {
    path.clear();
    if (!isNodeSafe(start, bbox_min, bbox_max)) {
        return START_FAIL;
    }
    if (!isNodeSafe(goal, bbox_min, bbox_max)) {
        return END_FAIL;
    }
    if (lineSafe(start, goal, bbox_min, bbox_max)) {
        path.push_back(start);
        path.push_back(goal);
        return REACH_END;
    }
    if (only_raycast) {
        return NO_PATH;
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto timedOut = [&]() {
        if (timeout <= 0.0) {
            return false;
        }
        const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time;
        return elapsed.count() > timeout;
    };

    const GridIndex start_idx = posToIndex(start);
    const GridIndex goal_idx = posToIndex(goal);
    std::priority_queue<QueueItem> open;
    std::unordered_map<GridIndex, double, VoxelKeyHash, VoxelKeyEqual> g_score;
    std::unordered_map<GridIndex, GridIndex, VoxelKeyHash, VoxelKeyEqual> parent;
    std::unordered_map<GridIndex, char, VoxelKeyHash, VoxelKeyEqual> closed;

    auto heuristic = [&](const GridIndex &idx) {
        return cfg_.lambda_heu * (indexToPos(idx) - goal).norm();
    };
    g_score[start_idx] = 0.0;
    open.push(QueueItem{heuristic(start_idx), start_idx});

    int expanded = 0;
    const int neighbor_bound = 1;
    while (!open.empty()) {
        if (timedOut()) {
            return TIME_OUT;
        }
        const QueueItem current = open.top();
        open.pop();
        if (closed[current.idx] != 0) {
            continue;
        }
        closed[current.idx] = 1;
        if (current.idx == goal_idx || lineSafe(indexToPos(current.idx), goal, bbox_min, bbox_max)) {
            GridIndex trace = current.idx;
            while (trace != start_idx) {
                path.push_back(indexToPos(trace));
                const auto pit = parent.find(trace);
                if (pit == parent.end()) {
                    path.clear();
                    return NO_PATH;
                }
                trace = pit->second;
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            if (path.empty() || (path.back() - goal).norm() > 1.0e-6) {
                path.push_back(goal);
            }
            if (cfg_.shorten_path) {
                shortenPath(path, bbox_min, bbox_max);
            }
            return REACH_END;
        }
        if (++expanded > cfg_.max_nodes) {
            return NO_PATH;
        }

        for (int dx = -neighbor_bound; dx <= neighbor_bound; ++dx) {
            for (int dy = -neighbor_bound; dy <= neighbor_bound; ++dy) {
                for (int dz = -neighbor_bound; dz <= neighbor_bound; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const GridIndex next_idx = current.idx + GridIndex(dx, dy, dz);
                    if (closed[next_idx] != 0) {
                        continue;
                    }
                    const super_utils::Vec3f next_pos = indexToPos(next_idx);
                    if (!isNodeSafe(next_pos, bbox_min, bbox_max)) {
                        continue;
                    }
                    const double step_cost = (next_pos - indexToPos(current.idx)).norm();
                    const double tentative_g = g_score[current.idx] + step_cost;
                    const auto git = g_score.find(next_idx);
                    if (git != g_score.end() && tentative_g >= git->second) {
                        continue;
                    }
                    g_score[next_idx] = tentative_g;
                    parent[next_idx] = current.idx;
                    open.push(QueueItem{tentative_g + heuristic(next_idx), next_idx});
                }
            }
        }
    }
    return NO_PATH;
}

void ParallelBubbleAstar::calculatePathCost(const super_utils::vec_E<super_utils::Vec3f> &path,
                                            double &cost) const {
    cost = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        cost += (path[i] - path[i - 1]).norm();
    }
}

ParallelBubbleAstar::GridIndex ParallelBubbleAstar::posToIndex(const super_utils::Vec3f &pos) const {
    return (pos / cfg_.resolution).array().floor().cast<int>();
}

super_utils::Vec3f ParallelBubbleAstar::indexToPos(const GridIndex &idx) const {
    return (idx.cast<double>() + super_utils::Vec3f::Constant(0.5)) * cfg_.resolution;
}

bool ParallelBubbleAstar::isNodeSafe(const super_utils::Vec3f &pos,
                                     const super_utils::Vec3f &bbox_min,
                                     const super_utils::Vec3f &bbox_max) const {
    if (map_manager_ == nullptr || !pos.allFinite()) {
        return false;
    }
    if ((pos.array() < bbox_min.array()).any() || (pos.array() > bbox_max.array()).any()) {
        return false;
    }
    if (map_manager_->insideLocalMap(pos) &&
        !map_manager_->isStateValid(pos, MapBackend::ROG, true, true)) {
        return false;
    }
    const double dist = map_manager_->getDisToOcc(pos.cast<float>());
    return !std::isfinite(dist) || dist >= cfg_.safe_distance;
}

bool ParallelBubbleAstar::lineSafe(const super_utils::Vec3f &start,
                                   const super_utils::Vec3f &goal,
                                   const super_utils::Vec3f &bbox_min,
                                   const super_utils::Vec3f &bbox_max) const {
    const super_utils::Vec3f delta = goal - start;
    const double length = delta.norm();
    if (length < 1.0e-6) {
        return true;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(0.05, cfg_.resolution * 0.5))));
    for (int i = 0; i <= samples; ++i) {
        const super_utils::Vec3f p = start + delta * (static_cast<double>(i) / static_cast<double>(samples));
        if (!isNodeSafe(p, bbox_min, bbox_max)) {
            return false;
        }
    }
    return true;
}

bool ParallelBubbleAstar::shortenPath(super_utils::vec_E<super_utils::Vec3f> &path,
                                      const super_utils::Vec3f &bbox_min,
                                      const super_utils::Vec3f &bbox_max) const {
    if (path.size() <= 2U) {
        return false;
    }
    super_utils::vec_E<super_utils::Vec3f> shortened;
    shortened.push_back(path.front());
    std::size_t anchor = 0;
    while (anchor + 1U < path.size()) {
        std::size_t next = path.size() - 1U;
        while (next > anchor + 1U &&
               !lineSafe(path[anchor], path[next], bbox_min, bbox_max)) {
            --next;
        }
        shortened.push_back(path[next]);
        anchor = next;
    }
    const bool changed = shortened.size() < path.size();
    path.swap(shortened);
    return changed;
}

}  // namespace exploration
}  // namespace general_planner
