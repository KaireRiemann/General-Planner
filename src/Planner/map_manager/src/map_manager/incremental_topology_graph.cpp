#include <map_manager/incremental_topology_graph.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace general_planner {
namespace {

template <typename T>
T clampValue(const T value, const T low, const T high) {
    return std::max(low, std::min(value, high));
}

bool samePosition(const rog_map::Vec3f &lhs, const rog_map::Vec3f &rhs) {
    return (lhs - rhs).squaredNorm() <= 1.0e-12;
}

} // namespace

std::size_t IncrementalTopologyGraph::RegionKeyHash::operator()(
    const RegionKey &key) const {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
}

IncrementalTopologyGraph::Config IncrementalTopologyGraph::sanitized(
    const Config &input) {
    Config cfg = input;
    cfg.region_size = std::max(0.2, cfg.region_size);
    cfg.sample_spacing = clampValue(cfg.sample_spacing, 0.05, cfg.region_size);
    cfg.min_clearance = std::max(0.0, cfg.min_clearance);
    cfg.max_clearance = std::max(cfg.min_clearance, cfg.max_clearance);
    cfg.candidate_separation = std::max(cfg.sample_spacing, cfg.candidate_separation);
    cfg.stable_match_distance = std::max(0.0, cfg.stable_match_distance);
    cfg.connection_radius = std::max(cfg.candidate_separation, cfg.connection_radius);
    cfg.edge_sample_spacing = std::max(0.02, cfg.edge_sample_spacing);
    cfg.dirty_padding = std::max(0.0, cfg.dirty_padding);
    cfg.bubble_overlap_margin = std::max(0.0, cfg.bubble_overlap_margin);
    cfg.max_nodes_per_region = std::max<std::size_t>(1, cfg.max_nodes_per_region);
    cfg.max_bubbles_per_region = std::max<std::size_t>(1, cfg.max_bubbles_per_region);
    cfg.max_neighbors = std::max<std::size_t>(1, cfg.max_neighbors);
    cfg.max_regions_per_update = std::max<std::size_t>(1, cfg.max_regions_per_update);
    return cfg;
}

IncrementalTopologyGraph::IncrementalTopologyGraph()
    : IncrementalTopologyGraph(Config{}) {}

IncrementalTopologyGraph::IncrementalTopologyGraph(const Config &config)
    : config_(sanitized(config)) {}

IncrementalTopologyGraph::Config IncrementalTopologyGraph::config() const {
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);
    return config_;
}

void IncrementalTopologyGraph::configure(const Config &config) {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    {
        std::unique_lock<std::shared_mutex> lock(graph_mutex_);
        config_ = sanitized(config);
        nodes_.clear();
        regions_.clear();
        next_node_id_ = 1;
        revision_ = 0;
        rebuilt_region_count_ = 0;
    }
    std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
    dirty_regions_.clear();
    observed_route_regions_.clear();
}

void IncrementalTopologyGraph::clear() {
    const Config current = config();
    configure(current);
}

IncrementalTopologyGraph::RegionKey IncrementalTopologyGraph::regionOf(
    const rog_map::Vec3f &position) const {
    const double size = config_.region_size;
    return {static_cast<int>(std::floor(position.x() / size)),
            static_cast<int>(std::floor(position.y() / size)),
            static_cast<int>(std::floor(position.z() / size))};
}

rog_map::Vec3f IncrementalTopologyGraph::regionMin(const RegionKey &region) const {
    return config_.region_size * rog_map::Vec3f(region.x, region.y, region.z);
}

void IncrementalTopologyGraph::markDirty(const rog_map::Vec3f &position) {
    if (!position.allFinite()) {
        return;
    }
    Config cfg;
    RegionKey center;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        cfg = config_;
        if (!cfg.enabled) {
            return;
        }
        center = regionOf(position);
    }
    const int padding = static_cast<int>(std::ceil(cfg.dirty_padding / cfg.region_size));
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    for (int x = -padding; x <= padding; ++x) {
        for (int y = -padding; y <= padding; ++y) {
            for (int z = -padding; z <= padding; ++z) {
                dirty_regions_.insert({center.x + x, center.y + y, center.z + z});
            }
        }
    }
}

void IncrementalTopologyGraph::markDirtyBox(const rog_map::Vec3f &box_min,
                                             const rog_map::Vec3f &box_max) {
    if (!box_min.allFinite() || !box_max.allFinite() ||
        (box_max.array() < box_min.array()).any()) {
        return;
    }
    Config cfg;
    RegionKey first;
    RegionKey last;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        cfg = config_;
        if (!cfg.enabled) {
            return;
        }
        const rog_map::Vec3f padding = rog_map::Vec3f::Constant(cfg.dirty_padding);
        first = regionOf(box_min - padding);
        last = regionOf(box_max + padding);
    }
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    for (int x = first.x; x <= last.x; ++x) {
        for (int y = first.y; y <= last.y; ++y) {
            for (int z = first.z; z <= last.z; ++z) {
                dirty_regions_.insert({x, y, z});
            }
        }
    }
}

void IncrementalTopologyGraph::markDirtyVoxels(
    const std::vector<rog_map::Vec3i> &indices, const double resolution) {
    if (indices.empty() || !std::isfinite(resolution) || resolution <= 0.0) {
        return;
    }
    Config cfg;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        cfg = config_;
        if (!cfg.enabled) {
            return;
        }
    }
    const int padding = static_cast<int>(std::ceil(cfg.dirty_padding / cfg.region_size));
    std::unordered_set<RegionKey, RegionKeyHash> changed_regions;
    changed_regions.reserve(indices.size());
    for (const rog_map::Vec3i &index : indices) {
        const rog_map::Vec3f position =
            (index.cast<double>() + rog_map::Vec3f::Constant(0.5)) * resolution;
        changed_regions.insert({
            static_cast<int>(std::floor(position.x() / cfg.region_size)),
            static_cast<int>(std::floor(position.y() / cfg.region_size)),
            static_cast<int>(std::floor(position.z() / cfg.region_size))});
    }
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    for (const RegionKey &center : changed_regions) {
        for (int x = -padding; x <= padding; ++x) {
            for (int y = -padding; y <= padding; ++y) {
                for (int z = -padding; z <= padding; ++z) {
                    dirty_regions_.insert({center.x + x, center.y + y, center.z + z});
                }
            }
        }
    }
}

void IncrementalTopologyGraph::observePlannedPath(
    const rog_map::vec_Vec3f &path) {
    if (path.empty()) {
        return;
    }
    Config cfg;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        cfg = config_;
        if (!cfg.enabled) {
            return;
        }
    }

    std::unordered_set<RegionKey, RegionKeyHash> route_regions;
    const auto routeRegion = [&cfg](const rog_map::Vec3f &position) {
        return RegionKey{
            static_cast<int>(std::floor(position.x() / cfg.region_size)),
            static_cast<int>(std::floor(position.y() / cfg.region_size)),
            static_cast<int>(std::floor(position.z() / cfg.region_size))};
    };
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!path[i].allFinite()) {
            continue;
        }
        if (i == 0) {
            route_regions.insert(routeRegion(path[i]));
            continue;
        }
        const rog_map::Vec3f delta = path[i] - path[i - 1];
        const double length = delta.norm();
        const int samples = std::max(1, static_cast<int>(std::ceil(
            length / std::max(0.1, 0.5 * cfg.region_size))));
        for (int sample = 1; sample <= samples; ++sample) {
            const double ratio = static_cast<double>(sample) /
                                 static_cast<double>(samples);
            route_regions.insert(routeRegion(path[i - 1] + ratio * delta));
        }
    }

    std::lock_guard<std::mutex> lock(dirty_mutex_);
    for (const RegionKey &region : route_regions) {
        if (observed_route_regions_.insert(region).second) {
            dirty_regions_.insert(region);
        }
    }
}

bool IncrementalTopologyGraph::lineTraversable(const rog_map::Vec3f &start,
                                                const rog_map::Vec3f &goal,
                                                const TopologyMapView &map_view) const {
    if (!start.allFinite() || !goal.allFinite()) {
        return false;
    }
    const double distance = (goal - start).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(
        distance / config_.edge_sample_spacing)));
    for (int i = 0; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(steps);
        if (!map_view.isTraversable(start + ratio * (goal - start))) {
            return false;
        }
    }
    return true;
}

double IncrementalTopologyGraph::estimateClearance(
    const rog_map::Vec3f &position, const TopologyMapView &map_view) const {
    double clearance = 0.0;
    if (map_view.getClearance(position, clearance) &&
        std::isfinite(clearance)) {
        return clampValue(clearance, 0.0, config_.max_clearance);
    }

    static const rog_map::Vec3f directions[] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
        {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
        {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
        {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
        {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};
    const double step = std::min(config_.sample_spacing * 0.5,
                                 config_.edge_sample_spacing);
    double minimum = config_.max_clearance;
    for (const rog_map::Vec3f &raw_direction : directions) {
        const rog_map::Vec3f direction = raw_direction.normalized();
        double hit_distance = config_.max_clearance;
        for (double distance = step; distance <= config_.max_clearance; distance += step) {
            if (!map_view.isTraversable(position + distance * direction)) {
                hit_distance = distance;
                break;
            }
        }
        minimum = std::min(minimum, hit_distance);
    }
    return minimum;
}

std::vector<IncrementalTopologyGraph::Node,
            Eigen::aligned_allocator<IncrementalTopologyGraph::Node>>
IncrementalTopologyGraph::generateCandidates(const RegionKey &region,
                                              const TopologyMapView &map_view) const {
    std::vector<Node, Eigen::aligned_allocator<Node>> candidates;

    struct Box {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        rog_map::Vec3f minimum;
        rog_map::Vec3f maximum;
    };
    struct Bubble {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        rog_map::Vec3f center;
        double radius{0.0};
    };

    const rog_map::Vec3f minimum = regionMin(region);
    std::vector<Box, Eigen::aligned_allocator<Box>> pending;
    pending.push_back({minimum,
                       minimum + rog_map::Vec3f::Constant(config_.region_size)});
    std::vector<Bubble, Eigen::aligned_allocator<Bubble>> bubbles;
    bubbles.reserve(config_.max_bubbles_per_region);

    while (!pending.empty() && bubbles.size() < config_.max_bubbles_per_region) {
        const Box box = pending.back();
        pending.pop_back();
        const rog_map::Vec3f center = 0.5 * (box.minimum + box.maximum);
        const rog_map::Vec3f half_size = 0.5 * (box.maximum - box.minimum);
        const double half_diagonal = half_size.norm();

        bool covered = false;
        for (const Bubble &bubble : bubbles) {
            if ((center - bubble.center).norm() + half_diagonal <= bubble.radius) {
                covered = true;
                break;
            }
        }
        if (covered) {
            continue;
        }

        if (map_view.isTraversable(center)) {
            const double clearance = estimateClearance(center, map_view);
            if (clearance >= config_.min_clearance) {
                bubbles.push_back({center, clearance});
                if (clearance + 1.0e-9 >= half_diagonal) {
                    continue;
                }
            }
        }

        if ((box.maximum - box.minimum).maxCoeff() <=
            config_.sample_spacing + 1.0e-9) {
            continue;
        }
        const rog_map::Vec3f midpoint = center;
        for (int mask = 0; mask < 8; ++mask) {
            Box child;
            for (int axis = 0; axis < 3; ++axis) {
                const bool upper = (mask & (1 << axis)) != 0;
                child.minimum[axis] = upper ? midpoint[axis] : box.minimum[axis];
                child.maximum[axis] = upper ? box.maximum[axis] : midpoint[axis];
            }
            pending.push_back(child);
        }
    }

    if (bubbles.empty()) {
        return candidates;
    }

    // Bubble overlap is a local free-space connectivity relation. Keeping one
    // representative for each union preserves separate passages within the
    // same dirty region, unlike clearance-ranked uniform sampling.
    std::vector<std::size_t> parent(bubbles.size());
    for (std::size_t i = 0; i < parent.size(); ++i) {
        parent[i] = i;
    }
    auto root = [&parent](std::size_t index) {
        while (parent[index] != index) {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        return index;
    };
    for (std::size_t i = 0; i < bubbles.size(); ++i) {
        for (std::size_t j = i + 1; j < bubbles.size(); ++j) {
            const double overlap = bubbles[i].radius + bubbles[j].radius -
                                   config_.bubble_overlap_margin;
            if (overlap > 0.0 &&
                (bubbles[i].center - bubbles[j].center).norm() <= overlap) {
                const std::size_t root_i = root(i);
                const std::size_t root_j = root(j);
                if (root_i != root_j) {
                    parent[root_j] = root_i;
                }
            }
        }
    }

    std::unordered_map<std::size_t, std::size_t> representative;
    const rog_map::Vec3f region_center =
        minimum + rog_map::Vec3f::Constant(0.5 * config_.region_size);
    for (std::size_t i = 0; i < bubbles.size(); ++i) {
        const std::size_t component = root(i);
        const auto found = representative.find(component);
        if (found == representative.end()) {
            representative.emplace(component, i);
            continue;
        }
        const Bubble &current = bubbles[found->second];
        const double current_score = (current.center - region_center).norm() -
                                     0.25 * current.radius;
        const double candidate_score = (bubbles[i].center - region_center).norm() -
                                       0.25 * bubbles[i].radius;
        if (candidate_score < current_score) {
            found->second = i;
        }
    }
    for (const auto &entry : representative) {
        Node node;
        node.position = bubbles[entry.second].center;
        node.clearance = bubbles[entry.second].radius;
        candidates.push_back(node);
    }

    std::sort(candidates.begin(), candidates.end(), [](const Node &lhs, const Node &rhs) {
        if (lhs.clearance != rhs.clearance) {
            return lhs.clearance > rhs.clearance;
        }
        if (lhs.position.x() != rhs.position.x()) return lhs.position.x() < rhs.position.x();
        if (lhs.position.y() != rhs.position.y()) return lhs.position.y() < rhs.position.y();
        return lhs.position.z() < rhs.position.z();
    });

    if (candidates.size() > config_.max_nodes_per_region) {
        candidates.resize(config_.max_nodes_per_region);
    }
    return candidates;
}

bool IncrementalTopologyGraph::popDirtyRegion(RegionKey &region,
                                               const rog_map::Vec3f *focus) {
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    if (dirty_regions_.empty()) {
        return false;
    }
    auto iterator = dirty_regions_.begin();
    if (focus != nullptr && focus->allFinite()) {
        double best_distance = std::numeric_limits<double>::infinity();
        for (auto candidate = dirty_regions_.begin();
             candidate != dirty_regions_.end(); ++candidate) {
            const rog_map::Vec3f center = config_.region_size *
                rog_map::Vec3f(candidate->x + 0.5,
                               candidate->y + 0.5,
                               candidate->z + 0.5);
            const double distance = (center - *focus).squaredNorm();
            if (distance < best_distance) {
                best_distance = distance;
                iterator = candidate;
            }
        }
    }
    region = *iterator;
    dirty_regions_.erase(iterator);
    return true;
}

void IncrementalTopologyGraph::rebuildRegion(const RegionKey &region,
                                             const TopologyMapView &map_view) {
    auto candidates = generateCandidates(region, map_view);
    std::vector<Node, Eigen::aligned_allocator<Node>> old_nodes;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        const auto region_it = regions_.find(region);
        if (region_it != regions_.end()) {
            old_nodes.reserve(region_it->second.size());
            for (const NodeId id : region_it->second) {
                const auto node_it = nodes_.find(id);
                if (node_it != nodes_.end()) {
                    old_nodes.push_back(node_it->second.node);
                }
            }
        }
    }

    std::unordered_set<NodeId> matched;
    for (Node &candidate : candidates) {
        double best_distance = config_.stable_match_distance;
        NodeId best_id = 0;
        for (const Node &old_node : old_nodes) {
            if (matched.count(old_node.id) != 0U) {
                continue;
            }
            const double distance = (candidate.position - old_node.position).norm();
            if (distance <= best_distance) {
                best_distance = distance;
                best_id = old_node.id;
            }
        }
        if (best_id != 0) {
            candidate.id = best_id;
            matched.insert(best_id);
        }
    }

    std::vector<NodeId> affected_ids;
    {
        std::unique_lock<std::shared_mutex> lock(graph_mutex_);
        const auto old_region_it = regions_.find(region);
        if (old_region_it != regions_.end()) {
            for (const NodeId old_id : old_region_it->second) {
                const auto old_node_it = nodes_.find(old_id);
                if (old_node_it == nodes_.end()) {
                    continue;
                }
                for (const auto &neighbor : old_node_it->second.neighbors) {
                    const auto neighbor_it = nodes_.find(neighbor.first);
                    if (neighbor_it != nodes_.end()) {
                        neighbor_it->second.neighbors.erase(old_id);
                    }
                }
                nodes_.erase(old_node_it);
            }
            regions_.erase(old_region_it);
        }

        ++revision_;
        for (Node &candidate : candidates) {
            if (candidate.id == 0) {
                candidate.id = next_node_id_++;
            }
            candidate.revision = revision_;
            NodeRecord record;
            record.node = candidate;
            record.region = region;
            nodes_.emplace(candidate.id, std::move(record));
            affected_ids.push_back(candidate.id);
        }
        if (!affected_ids.empty()) {
            regions_[region] = affected_ids;
        }
        ++rebuilt_region_count_;
    }

    // A new obstacle can cut a long edge even when neither endpoint belongs to
    // this region. Include endpoints of every spatially crossing edge so those
    // stored visibility constraints are revalidated incrementally as well.
    const rog_map::Vec3f dirty_min = regionMin(region) -
        rog_map::Vec3f::Constant(config_.edge_sample_spacing);
    const rog_map::Vec3f dirty_max = regionMin(region) +
        rog_map::Vec3f::Constant(config_.region_size + config_.edge_sample_spacing);
    std::unordered_set<NodeId> affected_set(affected_ids.begin(), affected_ids.end());
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        for (const auto &entry : nodes_) {
            for (const auto &neighbor : entry.second.neighbors) {
                if (entry.first >= neighbor.first) {
                    continue;
                }
                const auto neighbor_it = nodes_.find(neighbor.first);
                if (neighbor_it == nodes_.end()) {
                    continue;
                }
                const rog_map::Vec3f segment_min =
                    entry.second.node.position.cwiseMin(neighbor_it->second.node.position);
                const rog_map::Vec3f segment_max =
                    entry.second.node.position.cwiseMax(neighbor_it->second.node.position);
                if ((segment_max.array() >= dirty_min.array()).all() &&
                    (segment_min.array() <= dirty_max.array()).all()) {
                    affected_set.insert(entry.first);
                    affected_set.insert(neighbor.first);
                }
            }
        }
    }
    affected_ids.assign(affected_set.begin(), affected_set.end());
    rebuildIncidentEdges(affected_ids, map_view);
}

void IncrementalTopologyGraph::rebuildIncidentEdges(
    const std::vector<NodeId> &source_ids, const TopologyMapView &map_view) {
    struct CandidateEdge {
        NodeId from{0};
        NodeId to{0};
        rog_map::Vec3f from_position{rog_map::Vec3f::Zero()};
        rog_map::Vec3f to_position{rog_map::Vec3f::Zero()};
        double distance{0.0};
    };

    std::vector<CandidateEdge> candidate_edges;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        for (const NodeId source_id : source_ids) {
            const auto source_it = nodes_.find(source_id);
            if (source_it == nodes_.end()) {
                continue;
            }
            std::vector<CandidateEdge> local;
            for (const auto &entry : nodes_) {
                if (entry.first == source_id) {
                    continue;
                }
                const double distance =
                    (entry.second.node.position - source_it->second.node.position).norm();
                if (distance <= config_.connection_radius) {
                    local.push_back({source_id, entry.first,
                                     source_it->second.node.position,
                                     entry.second.node.position, distance});
                }
            }
            std::sort(local.begin(), local.end(), [](const CandidateEdge &lhs,
                                                     const CandidateEdge &rhs) {
                return lhs.distance < rhs.distance;
            });
            if (local.size() > config_.max_neighbors) {
                local.resize(config_.max_neighbors);
            }
            candidate_edges.insert(candidate_edges.end(), local.begin(), local.end());
        }
    }

    std::vector<CandidateEdge> valid_edges;
    valid_edges.reserve(candidate_edges.size());
    for (const CandidateEdge &edge : candidate_edges) {
        if (lineTraversable(edge.from_position, edge.to_position, map_view)) {
            valid_edges.push_back(edge);
        }
    }

    std::unique_lock<std::shared_mutex> lock(graph_mutex_);
    for (const NodeId source_id : source_ids) {
        const auto source_it = nodes_.find(source_id);
        if (source_it == nodes_.end()) {
            continue;
        }
        for (const auto &neighbor : source_it->second.neighbors) {
            const auto neighbor_it = nodes_.find(neighbor.first);
            if (neighbor_it != nodes_.end()) {
                neighbor_it->second.neighbors.erase(source_id);
            }
        }
        source_it->second.neighbors.clear();
    }
    for (const CandidateEdge &edge : valid_edges) {
        const auto from_it = nodes_.find(edge.from);
        const auto to_it = nodes_.find(edge.to);
        if (from_it == nodes_.end() || to_it == nodes_.end() ||
            !samePosition(from_it->second.node.position, edge.from_position) ||
            !samePosition(to_it->second.node.position, edge.to_position)) {
            continue;
        }
        from_it->second.neighbors[edge.to] = edge.distance;
        to_it->second.neighbors[edge.from] = edge.distance;
    }
}

std::size_t IncrementalTopologyGraph::update(const TopologyMapView &map_view,
                                             std::size_t max_regions,
                                             const rog_map::Vec3f *focus) {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        if (!config_.enabled) {
            return 0;
        }
        if (max_regions == 0) {
            max_regions = config_.max_regions_per_update;
        }
    }
    std::size_t rebuilt = 0;
    RegionKey region;
    while (rebuilt < max_regions && popDirtyRegion(region, focus)) {
        rebuildRegion(region, map_view);
        ++rebuilt;
    }
    return rebuilt;
}

IncrementalTopologyGraph::Snapshot IncrementalTopologyGraph::snapshot() const {
    Snapshot output;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        output.nodes.reserve(nodes_.size());
        for (const auto &entry : nodes_) {
            output.nodes.push_back(entry.second.node);
            for (const auto &neighbor : entry.second.neighbors) {
                if (entry.first < neighbor.first) {
                    output.edges.push_back({entry.first, neighbor.first, neighbor.second});
                }
            }
        }
        output.revision = revision_;
    }
    std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
    output.dirty_region_count = dirty_regions_.size();
    return output;
}

IncrementalTopologyGraph::Stats IncrementalTopologyGraph::stats() const {
    Stats output;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        output.node_count = nodes_.size();
        output.region_count = regions_.size();
        for (const auto &entry : nodes_) {
            output.edge_count += entry.second.neighbors.size();
        }
        output.edge_count /= 2U;
        output.revision = revision_;
        output.rebuilt_region_count = rebuilt_region_count_;
    }
    std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
    output.dirty_region_count = dirty_regions_.size();
    return output;
}

bool IncrementalTopologyGraph::findPath(const rog_map::Vec3f &start,
                                        const rog_map::Vec3f &goal,
                                        const TopologyMapView &map_view,
                                        rog_map::vec_Vec3f &path,
                                        double attach_radius) const {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    path.clear();
    if (!start.allFinite() || !goal.allFinite() ||
        !map_view.isTraversable(start) || !map_view.isTraversable(goal)) {
        return false;
    }
    if ((goal - start).norm() <= config_.edge_sample_spacing) {
        path = {start, goal};
        return true;
    }
    if (lineTraversable(start, goal, map_view)) {
        path = {start, goal};
        return true;
    }

    struct SearchNode {
        rog_map::Vec3f position{rog_map::Vec3f::Zero()};
        std::unordered_map<NodeId, double> neighbors;
    };
    std::unordered_map<NodeId, SearchNode> graph;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        attach_radius = attach_radius > 0.0 ? attach_radius : config_.connection_radius;
        graph.reserve(nodes_.size());
        for (const auto &entry : nodes_) {
            graph.emplace(entry.first,
                          SearchNode{entry.second.node.position,
                                     entry.second.neighbors});
        }
    }
    if (graph.empty()) {
        return false;
    }

    std::vector<std::pair<double, NodeId>> start_candidates;
    std::unordered_map<NodeId, double> goal_links;
    for (const auto &entry : graph) {
        const double start_distance = (entry.second.position - start).norm();
        if (start_distance <= attach_radius &&
            lineTraversable(start, entry.second.position, map_view)) {
            start_candidates.emplace_back(start_distance, entry.first);
        }
        const double goal_distance = (entry.second.position - goal).norm();
        if (goal_distance <= attach_radius &&
            lineTraversable(entry.second.position, goal, map_view)) {
            goal_links.emplace(entry.first, goal_distance);
        }
    }
    if (start_candidates.empty() || goal_links.empty()) {
        return false;
    }
    std::sort(start_candidates.begin(), start_candidates.end());
    const std::size_t max_attach = std::min(config_.max_neighbors,
                                             start_candidates.size());

    using QueueEntry = std::pair<double, NodeId>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        std::greater<QueueEntry>> queue;
    std::unordered_map<NodeId, double> distance;
    std::unordered_map<NodeId, NodeId> parent;
    for (std::size_t i = 0; i < max_attach; ++i) {
        distance[start_candidates[i].second] = start_candidates[i].first;
        parent[start_candidates[i].second] = 0;
        queue.push(start_candidates[i]);
    }

    double best_goal_cost = std::numeric_limits<double>::infinity();
    NodeId best_goal_parent = 0;
    while (!queue.empty()) {
        const QueueEntry current = queue.top();
        queue.pop();
        const auto distance_it = distance.find(current.second);
        if (distance_it == distance.end() || current.first > distance_it->second + 1.0e-12) {
            continue;
        }
        if (current.first >= best_goal_cost) {
            break;
        }
        const auto goal_it = goal_links.find(current.second);
        if (goal_it != goal_links.end() && current.first + goal_it->second < best_goal_cost) {
            best_goal_cost = current.first + goal_it->second;
            best_goal_parent = current.second;
        }
        const auto graph_it = graph.find(current.second);
        if (graph_it == graph.end()) {
            continue;
        }
        for (const auto &neighbor : graph_it->second.neighbors) {
            if (graph.count(neighbor.first) == 0U) {
                continue;
            }
            const double proposed = current.first + neighbor.second;
            const auto known = distance.find(neighbor.first);
            if (known == distance.end() || proposed < known->second) {
                distance[neighbor.first] = proposed;
                parent[neighbor.first] = current.second;
                queue.emplace(proposed, neighbor.first);
            }
        }
    }
    if (best_goal_parent == 0) {
        return false;
    }

    std::vector<NodeId> reverse_ids;
    for (NodeId id = best_goal_parent; id != 0;) {
        reverse_ids.push_back(id);
        const auto parent_it = parent.find(id);
        if (parent_it == parent.end()) {
            return false;
        }
        id = parent_it->second;
    }
    path.push_back(start);
    for (auto iterator = reverse_ids.rbegin(); iterator != reverse_ids.rend(); ++iterator) {
        path.push_back(graph.at(*iterator).position);
    }
    path.push_back(goal);
    return true;
}

} // namespace general_planner
