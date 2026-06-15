#include <general_core/exploration_region_graph.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace general_planner {
namespace {
std::size_t mixHash(std::size_t seed, const int value) {
    const std::size_t h = std::hash<int>{}(value);
    return seed ^ (h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}
}  // namespace

std::size_t ExplorationRegionGraph::RegionKeyHasher::operator()(const RegionKey &key) const {
    std::size_t seed = 0;
    seed = mixHash(seed, key.x);
    seed = mixHash(seed, key.y);
    seed = mixHash(seed, key.z);
    return seed;
}

ExplorationRegionGraph::ExplorationRegionGraph()
        : ExplorationRegionGraph(Config{}) {
}

ExplorationRegionGraph::ExplorationRegionGraph(const Config &cfg)
        : cfg_(cfg) {
    cfg_.region_resolution = std::max(0.5, cfg_.region_resolution);
    cfg_.connectivity_radius = std::max(cfg_.region_resolution, cfg_.connectivity_radius);
}

void ExplorationRegionGraph::reset() {
    regions_.clear();
    key_to_region_id_.clear();
    region_id_to_key_.clear();
    frontier_to_region_.clear();
    visited_keys_.clear();
    current_region_id_ = -1;
    last_stamp_ = 0.0;
}

ExplorationRegionGraph::RegionKey ExplorationRegionGraph::makeKey(const super_utils::Vec3f &pos) const {
    return RegionKey{static_cast<int>(std::floor(pos.x() / cfg_.region_resolution)),
                     static_cast<int>(std::floor(pos.y() / cfg_.region_resolution)),
                     static_cast<int>(std::floor(pos.z() / cfg_.region_resolution))};
}

super_utils::Vec3f ExplorationRegionGraph::keyCenter(const RegionKey &key) const {
    return (super_utils::Vec3f(key.x, key.y, key.z) +
            super_utils::Vec3f::Constant(0.5)) * cfg_.region_resolution;
}

int ExplorationRegionGraph::nearestRegion(const super_utils::Vec3f &pos) const {
    double best_dist = std::numeric_limits<double>::infinity();
    int best_id = -1;
    for (const auto &kv : regions_) {
        const double dist = (kv.second.center - pos).squaredNorm();
        if (dist < best_dist ||
            (std::abs(dist - best_dist) < 1.0e-9 && kv.first < best_id)) {
            best_dist = dist;
            best_id = kv.first;
        }
    }
    return best_id;
}

void ExplorationRegionGraph::update(const ExplorationMemoryGrid &memory,
                                    const FrontierDatabase &frontier_db,
                                    const super_utils::Vec3f &robot_pos,
                                    const double stamp) {
    regions_.clear();
    key_to_region_id_.clear();
    region_id_to_key_.clear();
    frontier_to_region_.clear();
    last_stamp_ = stamp;

    std::unordered_map<RegionKey, int, RegionKeyHasher> known_counts;
    super_utils::vec_E<CompleteFrontierCell> known_free;
    memory.getKnownFreeCells(known_free);
    for (const auto &cell : known_free) {
        ++known_counts[makeKey(cell.position)];
    }

    for (const auto &frontier : frontier_db.getActiveFrontiers()) {
        known_counts.emplace(makeKey(frontier.center), 0);
    }

    std::vector<RegionKey> keys;
    keys.reserve(known_counts.size());
    for (const auto &kv : known_counts) {
        keys.push_back(kv.first);
    }
    std::sort(keys.begin(), keys.end());
    if (cfg_.max_region_num > 0 &&
        static_cast<int>(keys.size()) > cfg_.max_region_num) {
        const RegionKey robot_key = makeKey(robot_pos);
        std::sort(keys.begin(), keys.end(), [&](const RegionKey &lhs, const RegionKey &rhs) {
            const double dl = (keyCenter(lhs) - keyCenter(robot_key)).squaredNorm();
            const double dr = (keyCenter(rhs) - keyCenter(robot_key)).squaredNorm();
            if (std::abs(dl - dr) > 1.0e-9) return dl < dr;
            return lhs < rhs;
        });
        keys.resize(static_cast<std::size_t>(cfg_.max_region_num));
        std::sort(keys.begin(), keys.end());
    }

    int next_id = 1;
    const double voxel_volume = std::pow(memory.resolution(), 3.0);
    for (const auto &key : keys) {
        ExplorationRegion region;
        region.id = next_id++;
        region.center = keyCenter(key);
        region.bbox_min = region.center - super_utils::Vec3f::Constant(0.5 * cfg_.region_resolution);
        region.bbox_max = region.center + super_utils::Vec3f::Constant(0.5 * cfg_.region_resolution);
        region.known_free_volume = static_cast<double>(known_counts[key]) * voxel_volume;
        region.visited = visited_keys_.find(key) != visited_keys_.end();
        regions_[region.id] = region;
        key_to_region_id_[key] = region.id;
        region_id_to_key_[region.id] = key;
    }

    for (const auto &frontier : frontier_db.getActiveFrontiers()) {
        int region_id = -1;
        const RegionKey key = makeKey(frontier.center);
        const auto key_it = key_to_region_id_.find(key);
        if (key_it != key_to_region_id_.end()) {
            region_id = key_it->second;
        } else {
            region_id = nearestRegion(frontier.center);
        }
        if (region_id > 0) {
            regions_[region_id].frontier_ids.push_back(frontier.id);
            regions_[region_id].unknown_volume += frontier.estimated_gain * voxel_volume;
            regions_[region_id].coverage_priority += frontier.estimated_gain;
            frontier_to_region_[frontier.id] = region_id;
        }
    }

    for (auto &kv : regions_) {
        auto &region = kv.second;
        if (region.visited) {
            region.coverage_priority -= cfg_.revisit_penalty;
        }
        region.active = !region.frontier_ids.empty() ||
                        region.unknown_volume >= cfg_.min_region_unknown_volume;
        std::sort(region.frontier_ids.begin(), region.frontier_ids.end());
    }

    std::vector<int> ids;
    ids.reserve(regions_.size());
    for (const auto &kv : regions_) {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    const double edge_radius_sq = cfg_.connectivity_radius * cfg_.connectivity_radius;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const int a = ids[i];
            const int b = ids[j];
            if ((regions_[a].center - regions_[b].center).squaredNorm() <= edge_radius_sq) {
                regions_[a].neighbor_region_ids.push_back(b);
                regions_[b].neighbor_region_ids.push_back(a);
            }
        }
    }
    for (auto &kv : regions_) {
        std::sort(kv.second.neighbor_region_ids.begin(), kv.second.neighbor_region_ids.end());
    }
    current_region_id_ = nearestRegion(robot_pos);
}

int ExplorationRegionGraph::currentRegion(const super_utils::Vec3f &robot_pos) const {
    if (current_region_id_ > 0) {
        return current_region_id_;
    }
    return nearestRegion(robot_pos);
}

int ExplorationRegionGraph::regionOfFrontier(const int frontier_id) const {
    const auto it = frontier_to_region_.find(frontier_id);
    return it == frontier_to_region_.end() ? -1 : it->second;
}

bool ExplorationRegionGraph::getRegion(const int id, ExplorationRegion &out) const {
    const auto it = regions_.find(id);
    if (it == regions_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

std::vector<int> ExplorationRegionGraph::activeRegionIds() const {
    std::vector<int> ids;
    for (const auto &kv : regions_) {
        if (kv.second.active) {
            ids.push_back(kv.first);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<ExplorationRegion> ExplorationRegionGraph::regions() const {
    std::vector<ExplorationRegion> out;
    out.reserve(regions_.size());
    for (const auto &kv : regions_) {
        out.push_back(kv.second);
    }
    std::sort(out.begin(), out.end(), [](const ExplorationRegion &lhs,
                                         const ExplorationRegion &rhs) {
        return lhs.id < rhs.id;
    });
    return out;
}

double ExplorationRegionGraph::regionCoveragePriority(const int region_id) const {
    const auto it = regions_.find(region_id);
    return it == regions_.end() ? 0.0 : it->second.coverage_priority;
}

double ExplorationRegionGraph::graphDistanceCost(const int from_region, const int to_region) const {
    if (from_region == to_region && from_region > 0) {
        return 0.0;
    }
    if (regions_.find(from_region) == regions_.end() ||
        regions_.find(to_region) == regions_.end()) {
        return 1.0e6;
    }

    struct Node {
        int id{-1};
        double cost{0.0};
        bool operator>(const Node &other) const {
            return cost > other.cost;
        }
    };
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<int, double> dist;
    dist[from_region] = 0.0;
    open.push(Node{from_region, 0.0});
    while (!open.empty()) {
        const Node node = open.top();
        open.pop();
        if (node.id == to_region) {
            return node.cost;
        }
        if (node.cost > dist[node.id] + 1.0e-9) {
            continue;
        }
        const auto rit = regions_.find(node.id);
        if (rit == regions_.end()) {
            continue;
        }
        for (const int neighbor : rit->second.neighbor_region_ids) {
            const auto nit = regions_.find(neighbor);
            if (nit == regions_.end()) {
                continue;
            }
            const double edge = (rit->second.center - nit->second.center).norm();
            const double new_cost = node.cost + edge;
            const auto dit = dist.find(neighbor);
            if (dit == dist.end() || new_cost < dit->second) {
                dist[neighbor] = new_cost;
                open.push(Node{neighbor, new_cost});
            }
        }
    }
    const auto fit = regions_.find(from_region);
    const auto tit = regions_.find(to_region);
    return 1.0e5 + (fit->second.center - tit->second.center).norm();
}

void ExplorationRegionGraph::markVisited(const int region_id) {
    const auto kit = region_id_to_key_.find(region_id);
    if (kit != region_id_to_key_.end()) {
        visited_keys_[kit->second] = true;
    }
    const auto rit = regions_.find(region_id);
    if (rit != regions_.end()) {
        rit->second.visited = true;
    }
}

}  // namespace general_planner
