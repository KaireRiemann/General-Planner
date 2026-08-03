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
    if (cfg.dense_known_free) {
        // A persistent graph must never turn absence of observation into
        // durable free-space evidence.
        cfg.unknown_as_free = false;
    }
    cfg.region_size = std::max(0.2, cfg.region_size);
    if (!std::isfinite(cfg.navigation_altitude)) {
        cfg.navigation_altitude = 0.0;
    }
    cfg.sample_spacing = clampValue(cfg.sample_spacing, 0.05, cfg.region_size);
    cfg.dense_evidence_vertical_tolerance =
        std::max(0.0, cfg.dense_evidence_vertical_tolerance);
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
    cfg.update_period = std::max(0.02, cfg.update_period);
    cfg.publish_period = std::max(0.05, cfg.publish_period);
    return cfg;
}

IncrementalTopologyGraph::IncrementalTopologyGraph()
    : IncrementalTopologyGraph(Config{}) {}

IncrementalTopologyGraph::IncrementalTopologyGraph(const Config &config)
    : config_(sanitized(config)), active_(config_.enabled) {
    auto snapshot = std::make_shared<SearchSnapshot>();
    snapshot->config = config_;
    search_snapshot_ = snapshot;
}

IncrementalTopologyGraph::Config IncrementalTopologyGraph::config() const {
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);
    return config_;
}

void IncrementalTopologyGraph::configure(const Config &config) {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    const Config sanitized_config = sanitized(config);
    {
        std::unique_lock<std::shared_mutex> lock(graph_mutex_);
        config_ = sanitized_config;
        nodes_.clear();
        regions_.clear();
        next_node_id_ = 1;
        revision_ = 0;
        rebuilt_region_count_ = 0;
        empty_region_count_ = 0;
        last_candidate_diagnostics_ = CandidateDiagnostics{};
    }
    {
        std::unique_lock<std::shared_mutex> evidence_lock(
            dense_evidence_mutex_);
        dense_evidence_.clear();
    }
    publishSearchSnapshot();
    {
        std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
        dirty_regions_.clear();
        observed_route_regions_.clear();
    }
    active_.store(sanitized_config.enabled, std::memory_order_release);
    {
        std::lock_guard<std::mutex> focus_lock(focus_mutex_);
        has_update_focus_ = false;
    }
}

void IncrementalTopologyGraph::clear() {
    const Config current = config();
    configure(current);
}

bool IncrementalTopologyGraph::active() const {
    return active_.load(std::memory_order_acquire);
}

void IncrementalTopologyGraph::setActive(const bool active) {
    const bool configured = config().enabled;
    active_.store(configured && active, std::memory_order_release);
}

void IncrementalTopologyGraph::requestUpdateFocus(const rog_map::Vec3f &focus) {
    if (!active() || !focus.allFinite()) {
        return;
    }
    std::lock_guard<std::mutex> lock(focus_mutex_);
    update_focus_ = focus;
    has_update_focus_ = true;
}

IncrementalTopologyGraph::RegionKey IncrementalTopologyGraph::regionOf(
    const rog_map::Vec3f &position) const {
    const double size = config_.region_size;
    return {static_cast<int>(std::floor(position.x() / size)),
            static_cast<int>(std::floor(position.y() / size)),
            config_.planar_mode
                ? 0
                : static_cast<int>(std::floor(position.z() / size))};
}

rog_map::Vec3f IncrementalTopologyGraph::regionMin(const RegionKey &region) const {
    rog_map::Vec3f minimum =
        config_.region_size * rog_map::Vec3f(region.x, region.y, region.z);
    if (config_.planar_mode) {
        minimum.z() = config_.navigation_altitude;
    }
    return minimum;
}

IncrementalTopologyGraph::RegionKey IncrementalTopologyGraph::denseCellOf(
    const rog_map::Vec3f &position) const {
    const double spacing = config_.sample_spacing;
    return {static_cast<int>(std::floor(position.x() / spacing)),
            static_cast<int>(std::floor(position.y() / spacing)),
            config_.planar_mode
                ? 0
                : static_cast<int>(std::floor(position.z() / spacing))};
}

bool IncrementalTopologyGraph::denseEvidenceTraversable(
    const rog_map::Vec3f &position) const {
    if (!position.allFinite()) {
        return false;
    }
    const RegionKey cell = denseCellOf(position);
    std::shared_lock<std::shared_mutex> lock(dense_evidence_mutex_);
    const auto found = dense_evidence_.find(cell);
    return found != dense_evidence_.end() &&
           found->second.free_count > 0 &&
           found->second.occupied_count == 0;
}

bool IncrementalTopologyGraph::constructionTraversable(
    const rog_map::Vec3f &position,
    const TopologyMapView &map_view) const {
    return config_.dense_known_free
        ? denseEvidenceTraversable(position)
        : map_view.isTraversable(position);
}

void IncrementalTopologyGraph::markDirty(const rog_map::Vec3f &position) {
    if (!active() || !position.allFinite()) {
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
    const int z_padding = cfg.planar_mode ? 0 : padding;
    for (int x = -padding; x <= padding; ++x) {
        for (int y = -padding; y <= padding; ++y) {
            for (int z = -z_padding; z <= z_padding; ++z) {
                dirty_regions_.insert({center.x + x, center.y + y, center.z + z});
            }
        }
    }
}

void IncrementalTopologyGraph::markDirtyBox(const rog_map::Vec3f &box_min,
                                             const rog_map::Vec3f &box_max) {
    if (!active() || !box_min.allFinite() || !box_max.allFinite() ||
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
    if (!active() || indices.empty() || !std::isfinite(resolution) || resolution <= 0.0) {
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
            cfg.planar_mode
                ? 0
                : static_cast<int>(std::floor(position.z() / cfg.region_size))});
    }
    const int z_padding = cfg.planar_mode ? 0 : padding;
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    for (const RegionKey &center : changed_regions) {
        for (int x = -padding; x <= padding; ++x) {
            for (int y = -padding; y <= padding; ++y) {
                for (int z = -z_padding; z <= z_padding; ++z) {
                    dirty_regions_.insert({center.x + x, center.y + y, center.z + z});
                }
            }
        }
    }
}

void IncrementalTopologyGraph::integrateDenseEvidence(
    const std::vector<VoxelEvidenceDelta> &deltas,
    const double resolution) {
    if (deltas.empty() || !std::isfinite(resolution) ||
        resolution <= 0.0) {
        return;
    }

    Config cfg;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        cfg = config_;
    }
    if (!cfg.enabled || !cfg.dense_known_free) {
        return;
    }

    std::unordered_map<RegionKey, DenseEvidence, RegionKeyHash> batch;
    batch.reserve(deltas.size());
    for (const VoxelEvidenceDelta &delta : deltas) {
        if (delta.free_delta == 0 && delta.occupied_delta == 0) {
            continue;
        }
        const rog_map::Vec3f position =
            (delta.index.cast<double>() +
             rog_map::Vec3f::Constant(0.5)) * resolution;
        if (cfg.planar_mode &&
            std::abs(position.z() - cfg.navigation_altitude) >
                cfg.dense_evidence_vertical_tolerance + 1.0e-9) {
            continue;
        }
        const RegionKey cell{
            static_cast<int>(std::floor(
                position.x() / cfg.sample_spacing)),
            static_cast<int>(std::floor(
                position.y() / cfg.sample_spacing)),
            cfg.planar_mode
                ? 0
                : static_cast<int>(std::floor(
                      position.z() / cfg.sample_spacing))};
        DenseEvidence &sum = batch[cell];
        sum.free_count += delta.free_delta;
        sum.occupied_count += delta.occupied_delta;
    }
    if (batch.empty()) {
        return;
    }

    std::vector<RegionKey> changed_cells;
    changed_cells.reserve(batch.size());
    {
        std::unique_lock<std::shared_mutex> lock(dense_evidence_mutex_);
        for (const auto &entry : batch) {
            auto found = dense_evidence_.find(entry.first);
            const bool was_traversable =
                found != dense_evidence_.end() &&
                found->second.free_count > 0 &&
                found->second.occupied_count == 0;
            if (found == dense_evidence_.end()) {
                found = dense_evidence_
                    .emplace(entry.first, DenseEvidence{})
                    .first;
            }
            DenseEvidence &evidence = found->second;
            evidence.free_count = std::max<std::int64_t>(
                0, evidence.free_count + entry.second.free_count);
            evidence.occupied_count = std::max<std::int64_t>(
                0, evidence.occupied_count +
                       entry.second.occupied_count);
            const bool is_traversable =
                evidence.free_count > 0 &&
                evidence.occupied_count == 0;
            if (was_traversable != is_traversable) {
                changed_cells.push_back(entry.first);
            }
            if (evidence.free_count == 0 &&
                evidence.occupied_count == 0) {
                dense_evidence_.erase(found);
            }
        }
    }
    if (changed_cells.empty()) {
        return;
    }

    const int padding = static_cast<int>(std::ceil(
        cfg.dirty_padding / cfg.region_size));
    const int z_padding = cfg.planar_mode ? 0 : padding;
    std::unordered_set<RegionKey, RegionKeyHash> changed_regions;
    changed_regions.reserve(changed_cells.size());
    for (const RegionKey &cell : changed_cells) {
        rog_map::Vec3f center(
            (static_cast<double>(cell.x) + 0.5) *
                cfg.sample_spacing,
            (static_cast<double>(cell.y) + 0.5) *
                cfg.sample_spacing,
            cfg.planar_mode
                ? cfg.navigation_altitude
                : (static_cast<double>(cell.z) + 0.5) *
                      cfg.sample_spacing);
        changed_regions.insert({
            static_cast<int>(std::floor(
                center.x() / cfg.region_size)),
            static_cast<int>(std::floor(
                center.y() / cfg.region_size)),
            cfg.planar_mode
                ? 0
                : static_cast<int>(std::floor(
                      center.z() / cfg.region_size))});
    }
    std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
    for (const RegionKey &center : changed_regions) {
        for (int x = -padding; x <= padding; ++x) {
            for (int y = -padding; y <= padding; ++y) {
                for (int z = -z_padding; z <= z_padding; ++z) {
                    dirty_regions_.insert(
                        {center.x + x, center.y + y, center.z + z});
                }
            }
        }
    }
}

void IncrementalTopologyGraph::observePlannedPath(
    const rog_map::vec_Vec3f &path) {
    if (!active() || path.empty()) {
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
    if (cfg.dense_known_free) {
        // A planned path is not sensor evidence. Dense persistent construction
        // is driven exclusively by the initial observed window and discrete
        // occupancy-state changes.
        return;
    }

    std::unordered_set<RegionKey, RegionKeyHash> route_regions;
    const auto routeRegion = [&cfg](const rog_map::Vec3f &position) {
        return RegionKey{
            static_cast<int>(std::floor(position.x() / cfg.region_size)),
            static_cast<int>(std::floor(position.y() / cfg.region_size)),
            cfg.planar_mode
                ? 0
                : static_cast<int>(std::floor(position.z() / cfg.region_size))};
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
                                                const TopologyMapView &map_view,
                                                double sample_spacing,
                                                const bool use_dense_evidence) const {
    if (!start.allFinite() || !goal.allFinite()) {
        return false;
    }
    const double distance = (goal - start).norm();
    if (!std::isfinite(sample_spacing) || sample_spacing <= 0.0) {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        sample_spacing = config_.edge_sample_spacing;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(
        distance / sample_spacing)));
    for (int i = 0; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(steps);
        const rog_map::Vec3f sample =
            start + ratio * (goal - start);
        if (use_dense_evidence
                ? !denseEvidenceTraversable(sample)
                : !map_view.isTraversable(sample)) {
            return false;
        }
    }
    return true;
}

double IncrementalTopologyGraph::estimateClearance(
    const rog_map::Vec3f &position, const TopologyMapView &map_view) const {
    double clearance = 0.0;
    // A 3D ESDF legitimately includes the virtual floor and ceiling. In
    // planar state2state mode those bounds have already selected the flight
    // layer, so using the ESDF minimum again would reject every horizontal
    // bubble in a narrow altitude band.
    if (!config_.dense_known_free && !config_.planar_mode &&
        map_view.getClearance(position, clearance) &&
        std::isfinite(clearance)) {
        return clampValue(clearance, 0.0, config_.max_clearance);
    }

    static const rog_map::Vec3f planar_directions[] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}};
    static const rog_map::Vec3f spatial_directions[] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
        {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
        {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
        {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
        {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};
    const double step = std::min(config_.sample_spacing * 0.5,
                                 config_.edge_sample_spacing);
    double minimum = config_.max_clearance;
    const auto scan_direction = [&](const rog_map::Vec3f &raw_direction) {
        const rog_map::Vec3f direction = raw_direction.normalized();
        double hit_distance = config_.max_clearance;
        for (double distance = step; distance <= config_.max_clearance; distance += step) {
            if (!constructionTraversable(
                    position + distance * direction, map_view)) {
                hit_distance = distance;
                break;
            }
        }
        minimum = std::min(minimum, hit_distance);
    };
    if (config_.planar_mode) {
        for (const rog_map::Vec3f &direction : planar_directions) {
            scan_direction(direction);
        }
    } else {
        for (const rog_map::Vec3f &direction : spatial_directions) {
            scan_direction(direction);
        }
    }
    return minimum;
}

std::vector<IncrementalTopologyGraph::Node,
            Eigen::aligned_allocator<IncrementalTopologyGraph::Node>>
IncrementalTopologyGraph::generateCandidates(const RegionKey &region,
                                              const TopologyMapView &map_view,
                                              CandidateDiagnostics &diagnostics) const {
    std::vector<Node, Eigen::aligned_allocator<Node>> candidates;
    const rog_map::Vec3f minimum = regionMin(region);

    if (config_.dense_known_free) {
        // Align samples to one global lattice rather than to each rebuilt
        // region. This makes node positions deterministic across incremental
        // updates and across negative/positive region boundaries.
        const double spacing = config_.sample_spacing;
        const auto firstIndex = [spacing](const double lower) {
            return static_cast<long long>(
                std::ceil(lower / spacing - 0.5 - 1.0e-9));
        };
        const auto lastIndex = [spacing](const double upper) {
            return static_cast<long long>(
                std::ceil(upper / spacing - 0.5 - 1.0e-9)) - 1LL;
        };
        const rog_map::Vec3f maximum =
            minimum + rog_map::Vec3f::Constant(config_.region_size);
        const long long first_x = firstIndex(minimum.x());
        const long long last_x = lastIndex(maximum.x());
        const long long first_y = firstIndex(minimum.y());
        const long long last_y = lastIndex(maximum.y());
        const long long first_z = config_.planar_mode
            ? 0LL : firstIndex(minimum.z());
        const long long last_z = config_.planar_mode
            ? 0LL : lastIndex(maximum.z());

        for (long long iz = first_z; iz <= last_z; ++iz) {
            for (long long iy = first_y; iy <= last_y; ++iy) {
                for (long long ix = first_x; ix <= last_x; ++ix) {
                    rog_map::Vec3f position(
                        (static_cast<double>(ix) + 0.5) * spacing,
                        (static_cast<double>(iy) + 0.5) * spacing,
                        config_.planar_mode
                            ? config_.navigation_altitude
                            : (static_cast<double>(iz) + 0.5) * spacing);
                    ++diagnostics.sampled_center_count;
                    if (!constructionTraversable(position, map_view)) {
                        continue;
                    }
                    ++diagnostics.traversable_center_count;
                    const double clearance =
                        estimateClearance(position, map_view);
                    if (clearance + 1.0e-9 < config_.min_clearance) {
                        ++diagnostics.clearance_rejected_count;
                        continue;
                    }
                    Node node;
                    node.position = position;
                    node.clearance = clearance;
                    candidates.push_back(node);
                    if (candidates.size() >=
                        config_.max_nodes_per_region) {
                        return candidates;
                    }
                }
            }
        }
        return candidates;
    }

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

    std::vector<Box, Eigen::aligned_allocator<Box>> pending;
    rog_map::Vec3f maximum =
        minimum + rog_map::Vec3f::Constant(config_.region_size);
    if (config_.planar_mode) {
        maximum.z() = minimum.z();
    }
    pending.push_back({minimum, maximum});
    std::vector<Bubble, Eigen::aligned_allocator<Bubble>> bubbles;
    bubbles.reserve(config_.max_bubbles_per_region);

    while (!pending.empty() && bubbles.size() < config_.max_bubbles_per_region) {
        const Box box = pending.back();
        pending.pop_back();
        const rog_map::Vec3f center = 0.5 * (box.minimum + box.maximum);
        const rog_map::Vec3f half_size = 0.5 * (box.maximum - box.minimum);
        const double half_diagonal = half_size.norm();
        ++diagnostics.sampled_center_count;

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

        if (constructionTraversable(center, map_view)) {
            ++diagnostics.traversable_center_count;
            const double clearance = estimateClearance(center, map_view);
            if (clearance >= config_.min_clearance) {
                bubbles.push_back({center, clearance});
                if (clearance + 1.0e-9 >= half_diagonal) {
                    continue;
                }
            } else {
                ++diagnostics.clearance_rejected_count;
            }
        }

        if ((box.maximum - box.minimum).maxCoeff() <=
            config_.sample_spacing + 1.0e-9) {
            continue;
        }
        const rog_map::Vec3f midpoint = center;
        const int split_axes = config_.planar_mode ? 2 : 3;
        const int child_count = 1 << split_axes;
        for (int mask = 0; mask < child_count; ++mask) {
            Box child;
            child.minimum = box.minimum;
            child.maximum = box.maximum;
            for (int axis = 0; axis < split_axes; ++axis) {
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
            rog_map::Vec3f center = config_.region_size *
                rog_map::Vec3f(candidate->x + 0.5,
                               candidate->y + 0.5,
                               candidate->z + 0.5);
            if (config_.planar_mode) {
                center.z() = config_.navigation_altitude;
            }
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
    CandidateDiagnostics diagnostics;
    auto candidates = generateCandidates(region, map_view, diagnostics);
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
        last_candidate_diagnostics_ = diagnostics;
        if (candidates.empty()) {
            ++empty_region_count_;
        }
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
    // this region. Only endpoints within one connection radius can own such an
    // edge, so use the region index instead of scanning the complete global
    // graph.
    const rog_map::Vec3f dirty_min = regionMin(region) -
        rog_map::Vec3f::Constant(config_.edge_sample_spacing);
    const rog_map::Vec3f dirty_max = regionMin(region) +
        rog_map::Vec3f::Constant(config_.region_size + config_.edge_sample_spacing);
    std::unordered_set<NodeId> affected_set(affected_ids.begin(), affected_ids.end());
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        const int radius = static_cast<int>(std::ceil(
            config_.connection_radius / config_.region_size)) + 1;
        const int z_radius = config_.planar_mode ? 0 : radius;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -z_radius; dz <= z_radius; ++dz) {
                    const auto region_it = regions_.find(
                        {region.x + dx, region.y + dy, region.z + dz});
                    if (region_it == regions_.end()) {
                        continue;
                    }
                    for (const NodeId id : region_it->second) {
                        const auto entry = nodes_.find(id);
                        if (entry == nodes_.end()) {
                            continue;
                        }
                        for (const auto &neighbor : entry->second.neighbors) {
                            if (entry->first >= neighbor.first) {
                                continue;
                            }
                            const auto neighbor_it = nodes_.find(neighbor.first);
                            if (neighbor_it == nodes_.end()) {
                                continue;
                            }
                            const rog_map::Vec3f segment_min =
                                entry->second.node.position.cwiseMin(
                                    neighbor_it->second.node.position);
                            const rog_map::Vec3f segment_max =
                                entry->second.node.position.cwiseMax(
                                    neighbor_it->second.node.position);
                            if ((segment_max.array() >= dirty_min.array()).all() &&
                                (segment_min.array() <= dirty_max.array()).all()) {
                                affected_set.insert(entry->first);
                                affected_set.insert(neighbor.first);
                            }
                        }
                    }
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
            const int radius = static_cast<int>(std::ceil(
                config_.connection_radius / config_.region_size));
            const int z_radius = config_.planar_mode ? 0 : radius;
            const RegionKey source_region = source_it->second.region;
            for (int dx = -radius; dx <= radius; ++dx) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dz = -z_radius; dz <= z_radius; ++dz) {
                        const auto region_it = regions_.find(
                            {source_region.x + dx, source_region.y + dy,
                             source_region.z + dz});
                        if (region_it == regions_.end()) {
                            continue;
                        }
                        for (const NodeId candidate_id : region_it->second) {
                            if (candidate_id == source_id) {
                                continue;
                            }
                            const auto candidate_it = nodes_.find(candidate_id);
                            if (candidate_it == nodes_.end()) {
                                continue;
                            }
                            const double distance =
                                (candidate_it->second.node.position -
                                 source_it->second.node.position).norm();
                            if (distance <= config_.connection_radius) {
                                local.push_back(
                                    {source_id, candidate_id,
                                     source_it->second.node.position,
                                     candidate_it->second.node.position,
                                     distance});
                            }
                        }
                    }
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
        if (lineTraversable(edge.from_position, edge.to_position, map_view,
                            0.0, config_.dense_known_free)) {
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

void IncrementalTopologyGraph::publishSearchSnapshot() {
    auto next = std::make_shared<SearchSnapshot>();
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        next->config = config_;
        next->revision = revision_;
        next->graph.reserve(nodes_.size());
        for (const auto &entry : nodes_) {
            next->graph.emplace(entry.first,
                                SearchNode{entry.second.node,
                                           entry.second.neighbors});
        }
    }
    std::atomic_store_explicit(
        &search_snapshot_, SearchSnapshotPtr(std::move(next)),
        std::memory_order_release);
}

std::size_t IncrementalTopologyGraph::update(const TopologyMapView &map_view,
                                             std::size_t max_regions,
                                             const rog_map::Vec3f *focus) {
    if (!active()) {
        return 0;
    }
    rog_map::Vec3f requested_focus = rog_map::Vec3f::Zero();
    if (focus == nullptr) {
        std::lock_guard<std::mutex> focus_lock(focus_mutex_);
        if (has_update_focus_) {
            requested_focus = update_focus_;
            focus = &requested_focus;
        }
    }
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        if (!config_.enabled || !active()) {
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
    if (rebuilt > 0 && config_.snapshot_every_update) {
        publishSearchSnapshot();
    }
    return rebuilt;
}

void IncrementalTopologyGraph::refreshSnapshot() {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
    publishSearchSnapshot();
}

IncrementalTopologyGraph::Snapshot IncrementalTopologyGraph::snapshot() const {
    Snapshot output;
    const auto search = acquireSearchSnapshot();
    if (search) {
        output.dense_known_free = search->config.dense_known_free;
        output.nodes.reserve(search->graph.size());
        for (const auto &entry : search->graph) {
            output.nodes.push_back(entry.second.node);
            for (const auto &neighbor : entry.second.neighbors) {
                if (entry.first < neighbor.first) {
                    output.edges.push_back({entry.first, neighbor.first, neighbor.second});
                }
            }
        }
        output.revision = search->revision;
    }
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        output.empty_region_count = empty_region_count_;
        output.last_sampled_center_count =
            last_candidate_diagnostics_.sampled_center_count;
        output.last_traversable_center_count =
            last_candidate_diagnostics_.traversable_center_count;
        output.last_clearance_rejected_count =
            last_candidate_diagnostics_.clearance_rejected_count;
    }
    {
        std::shared_lock<std::shared_mutex> evidence_lock(
            dense_evidence_mutex_);
        for (const auto &entry : dense_evidence_) {
            output.dense_evidence_cell_count +=
                entry.second.free_count > 0 &&
                entry.second.occupied_count == 0 ? 1U : 0U;
        }
    }
    std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
    output.dirty_region_count = dirty_regions_.size();
    return output;
}

IncrementalTopologyGraph::SearchSnapshotPtr
IncrementalTopologyGraph::acquireSearchSnapshot() const {
    return std::atomic_load_explicit(&search_snapshot_,
                                     std::memory_order_acquire);
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
        output.empty_region_count = empty_region_count_;
        output.last_sampled_center_count =
            last_candidate_diagnostics_.sampled_center_count;
        output.last_traversable_center_count =
            last_candidate_diagnostics_.traversable_center_count;
        output.last_clearance_rejected_count =
            last_candidate_diagnostics_.clearance_rejected_count;
    }
    {
        std::shared_lock<std::shared_mutex> evidence_lock(
            dense_evidence_mutex_);
        for (const auto &entry : dense_evidence_) {
            output.dense_evidence_cell_count +=
                entry.second.free_count > 0 &&
                entry.second.occupied_count == 0 ? 1U : 0U;
        }
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
    return findPath(acquireSearchSnapshot(), start, goal, map_view,
                    path, attach_radius);
}

bool IncrementalTopologyGraph::findPath(
    const SearchSnapshotPtr &snapshot,
    const rog_map::Vec3f &start,
    const rog_map::Vec3f &goal,
    const TopologyMapView &map_view,
    rog_map::vec_Vec3f &path,
    double attach_radius) const {
    path.clear();
    if (!active() || !snapshot || !start.allFinite() || !goal.allFinite() ||
        !map_view.isTraversable(start) || !map_view.isTraversable(goal)) {
        return false;
    }
    const auto &graph = snapshot->graph;
    const Config &query_config = snapshot->config;
    attach_radius = attach_radius > 0.0
        ? attach_radius : query_config.connection_radius;
    if ((goal - start).norm() <= query_config.edge_sample_spacing) {
        path = {start, goal};
        return true;
    }
    if (lineTraversable(start, goal, map_view,
                        query_config.edge_sample_spacing)) {
        path = {start, goal};
        return true;
    }
    if (graph.empty()) {
        return false;
    }

    // USS-Nav mounts query points onto a small nearest-neighbor set before
    // running graph A*. Do the same here: never ray-check every global node in
    // the real-time planner as the persistent graph grows.
    std::vector<std::pair<double, NodeId>> nearest_start;
    std::vector<std::pair<double, NodeId>> nearest_goal;
    nearest_start.reserve(graph.size());
    nearest_goal.reserve(graph.size());
    for (const auto &entry : graph) {
        const double start_distance =
            (entry.second.node.position - start).norm();
        if (start_distance <= attach_radius) {
            nearest_start.emplace_back(start_distance, entry.first);
        }
        const double goal_distance =
            (entry.second.node.position - goal).norm();
        if (goal_distance <= attach_radius) {
            nearest_goal.emplace_back(goal_distance, entry.first);
        }
    }
    std::sort(nearest_start.begin(), nearest_start.end());
    std::sort(nearest_goal.begin(), nearest_goal.end());

    std::vector<std::pair<double, NodeId>> start_candidates;
    std::unordered_map<NodeId, double> goal_links;
    const std::size_t attachment_checks = std::max<std::size_t>(
        8, 4 * query_config.max_neighbors);
    for (std::size_t i = 0;
         i < nearest_start.size() && i < attachment_checks &&
         start_candidates.size() < query_config.max_neighbors; ++i) {
        const auto node = graph.find(nearest_start[i].second);
        if (node != graph.end() &&
            lineTraversable(start, node->second.node.position, map_view,
                            query_config.edge_sample_spacing)) {
            start_candidates.push_back(nearest_start[i]);
        }
    }
    for (std::size_t i = 0;
         i < nearest_goal.size() && i < attachment_checks &&
         goal_links.size() < query_config.max_neighbors; ++i) {
        const auto node = graph.find(nearest_goal[i].second);
        if (node != graph.end() &&
            lineTraversable(node->second.node.position, goal, map_view,
                            query_config.edge_sample_spacing)) {
            goal_links.emplace(nearest_goal[i].second, nearest_goal[i].first);
        }
    }
    if (start_candidates.empty() || goal_links.empty()) {
        return false;
    }
    const std::size_t max_attach = std::min(query_config.max_neighbors,
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
        path.push_back(graph.at(*iterator).node.position);
    }
    path.push_back(goal);
    return true;
}

} // namespace general_planner
