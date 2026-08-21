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

const char *IncrementalTopologyGraph::constructionModeName(
    const ConstructionMode mode) {
    switch (mode) {
        case ConstructionMode::DENSE_KNOWN_FREE_DEBUG:
            return "dense_known_free";
        case ConstructionMode::PERSISTENT_BUBBLE_SKELETON:
        default:
            return "persistent_bubble_skeleton";
    }
}

IncrementalTopologyGraph::ConstructionMode
IncrementalTopologyGraph::constructionModeFromString(const std::string &name) {
    if (name == "dense_known_free" || name == "dense_known_free_debug") {
        return ConstructionMode::DENSE_KNOWN_FREE_DEBUG;
    }
    // Keep the old parameter value as a compatible alias.
    return ConstructionMode::PERSISTENT_BUBBLE_SKELETON;
}

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
    // Neither persistent skeletons nor diagnostic dense roadmaps may turn
    // absence of observation into durable free-space evidence.
    cfg.unknown_as_free = false;
    cfg.region_size = std::max(0.2, cfg.region_size);
    if (!std::isfinite(cfg.navigation_altitude)) {
        cfg.navigation_altitude = 0.0;
    }
    cfg.sample_spacing = clampValue(cfg.sample_spacing, 0.05, cfg.region_size);
    cfg.min_clearance = std::max(0.0, cfg.min_clearance);
    cfg.max_clearance = std::max(cfg.min_clearance, cfg.max_clearance);
    cfg.candidate_separation = std::max(cfg.sample_spacing, cfg.candidate_separation);
    cfg.stable_match_distance = std::max(0.0, cfg.stable_match_distance);
    cfg.connection_radius =
        std::max(cfg.candidate_separation, cfg.connection_radius);
    cfg.edge_sample_spacing = std::max(0.02, cfg.edge_sample_spacing);
    cfg.dirty_padding = std::max(0.0, cfg.dirty_padding);
    cfg.bubble_overlap_margin = std::max(0.0, cfg.bubble_overlap_margin);
    cfg.public_component_attach_radius = std::max(
        0.0, cfg.public_component_attach_radius);
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
        initialized_regions_.clear();
        dense_node_index_.clear();
        next_node_id_ = 1;
        revision_ = 0;
        rebuilt_region_count_ = 0;
        empty_region_count_ = 0;
        pending_node_count_ = 0;
        isolated_node_count_ = 0;
        connected_component_count_ = 0;
        public_topology_revision_ = 0;
        public_topology_initialized_ = false;
        public_topology_focus_.setZero();
        has_public_topology_focus_ = false;
        last_candidate_diagnostics_ = CandidateDiagnostics{};
    }
    publishSearchSnapshot();
    {
        std::lock_guard<std::mutex> dirty_lock(dirty_mutex_);
        dirty_regions_.clear();
        dirty_dense_cells_.clear();
        dirty_evidence_seeds_.clear();
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

bool IncrementalTopologyGraph::constructionTraversable(
    const rog_map::Vec3f &position,
    const TopologyMapView &map_view) const {
    return map_view.isTraversable(position);
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
    std::unordered_map<
        RegionKey,
        std::unordered_set<RegionKey, RegionKeyHash>,
        RegionKeyHash> changed_cells;
    std::unordered_map<RegionKey, rog_map::vec_Vec3f, RegionKeyHash>
        evidence_seeds;
    changed_regions.reserve(indices.size());
    changed_cells.reserve(indices.size());
    for (const rog_map::Vec3i &index : indices) {
        const rog_map::Vec3f position =
            (index.cast<double>() + rog_map::Vec3f::Constant(0.5)) * resolution;
        const RegionKey region{
            static_cast<int>(std::floor(position.x() / cfg.region_size)),
            static_cast<int>(std::floor(position.y() / cfg.region_size)),
            cfg.planar_mode
                ? 0
                : static_cast<int>(std::floor(position.z() / cfg.region_size))};
        changed_regions.insert(region);
        evidence_seeds[region].push_back(position);
        if (cfg.construction_mode == ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
            changed_cells[region].insert({
                static_cast<int>(std::floor(position.x() / cfg.sample_spacing)),
                static_cast<int>(std::floor(position.y() / cfg.sample_spacing)),
                cfg.planar_mode
                    ? 0
                    : static_cast<int>(
                          std::floor(position.z() / cfg.sample_spacing))});
        }
    }
    const int z_padding = cfg.planar_mode ? 0 : padding;
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    for (auto &entry : changed_cells) {
        auto &destination = dirty_dense_cells_[entry.first];
        destination.insert(entry.second.begin(), entry.second.end());
    }
    const std::size_t max_seed_count =
        std::max<std::size_t>(64, 8 * cfg.max_bubbles_per_region);
    for (auto &entry : evidence_seeds) {
        auto &destination = dirty_evidence_seeds_[entry.first];
        const std::size_t remaining = destination.size() < max_seed_count
            ? max_seed_count - destination.size() : 0;
        const std::size_t count = std::min(remaining, entry.second.size());
        destination.insert(destination.end(), entry.second.begin(),
                           entry.second.begin() +
                               static_cast<std::ptrdiff_t>(count));
    }
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
    if (cfg.construction_mode == ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
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
                                                double sample_spacing) const {
    return lineEvidence(start, goal, map_view, sample_spacing) ==
           TopologyMapView::EvidenceState::KNOWN_FREE;
}

TopologyMapView::EvidenceState IncrementalTopologyGraph::lineEvidence(
    const rog_map::Vec3f &start,
    const rog_map::Vec3f &goal,
    const TopologyMapView &map_view,
    double sample_spacing) const {
    using EvidenceState = TopologyMapView::EvidenceState;
    if (!start.allFinite() || !goal.allFinite()) {
        return EvidenceState::OCCUPIED;
    }
    const double distance = (goal - start).norm();
    if (!std::isfinite(sample_spacing) || sample_spacing <= 0.0) {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        sample_spacing = config_.edge_sample_spacing;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(
        distance / sample_spacing)));
    EvidenceState result = EvidenceState::KNOWN_FREE;
    for (int i = 0; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(steps);
        const rog_map::Vec3f sample =
            start + ratio * (goal - start);
        const EvidenceState state = map_view.evidenceState(sample);
        if (state == EvidenceState::OCCUPIED) {
            return EvidenceState::OCCUPIED;
        }
        if (state == EvidenceState::UNKNOWN) {
            result = EvidenceState::UNKNOWN;
        }
    }
    return result;
}

double IncrementalTopologyGraph::estimateClearance(
    const rog_map::Vec3f &position, const TopologyMapView &map_view) const {
    double clearance = 0.0;
    // A 3D ESDF legitimately includes the virtual floor and ceiling. In
    // planar state2state mode those bounds have already selected the flight
    // layer, so using the ESDF minimum again would reject every horizontal
    // bubble in a narrow altitude band.
    if (config_.construction_mode != ConstructionMode::DENSE_KNOWN_FREE_DEBUG &&
        !config_.planar_mode &&
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
                                              CandidateDiagnostics &diagnostics,
                                              const rog_map::vec_Vec3f &evidence_seeds,
                                              const bool evidence_only) const {
    std::vector<Node, Eigen::aligned_allocator<Node>> candidates;
    const rog_map::Vec3f minimum = regionMin(region);

    if (config_.construction_mode == ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
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
                    // Dense coverage is driven by known-free evidence only.
                    // Clearance / max_nodes_per_region must not punch holes.
                    Node node;
                    node.position = position;
                    // Dense roadmap nodes are occupancy samples, not ESDF
                    // bubbles. Their clearance is neither used for retention
                    // nor for edge cost, so a 26-direction clearance sweep here
                    // only multiplies ROG queries without changing the graph.
                    node.clearance = 0.0;
                    candidates.push_back(node);
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

    if (config_.construction_mode ==
        ConstructionMode::PERSISTENT_BUBBLE_SKELETON) {
        const double seed_separation = std::max(
            config_.edge_sample_spacing, 0.40 * config_.sample_spacing);
        for (const rog_map::Vec3f &seed : evidence_seeds) {
            if (bubbles.size() >= config_.max_bubbles_per_region) {
                break;
            }
            ++diagnostics.sampled_center_count;
            if (!constructionTraversable(seed, map_view)) {
                continue;
            }
            ++diagnostics.traversable_center_count;
            bool duplicate = false;
            for (const Bubble &bubble : bubbles) {
                if ((bubble.center - seed).norm() < seed_separation) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }
            const double clearance = estimateClearance(seed, map_view);
            if (clearance + 1.0e-9 < config_.min_clearance) {
                ++diagnostics.clearance_rejected_count;
                continue;
            }
            bubbles.push_back({seed, clearance});
        }
    }

    while (!evidence_only && !pending.empty() &&
           bubbles.size() < config_.max_bubbles_per_region) {
        const Box box = pending.back();
        pending.pop_back();
        const rog_map::Vec3f center = 0.5 * (box.minimum + box.maximum);
        const rog_map::Vec3f half_size = 0.5 * (box.maximum - box.minimum);
        const double half_diagonal = half_size.norm();
        ++diagnostics.sampled_center_count;

        rog_map::Vec3f candidate_center = center;
        bool candidate_free = constructionTraversable(candidate_center, map_view);
        const bool leaf_box = (box.maximum - box.minimum).maxCoeff() <=
                              config_.sample_spacing + 1.0e-9;
        if (!candidate_free && leaf_box && !config_.planar_mode) {
            // Sensor rays may leave a thin KNOWN_FREE band which does not
            // contain the mathematical octree center. Probe a small,
            // deterministic stencil inside the leaf and snap only to a point
            // explicitly classified KNOWN_FREE. UNKNOWN is never promoted.
            std::vector<rog_map::Vec3f,
                        Eigen::aligned_allocator<rog_map::Vec3f>> probes;
            probes.reserve(7);
            if (config_.navigation_altitude > box.minimum.z() &&
                config_.navigation_altitude < box.maximum.z()) {
                rog_map::Vec3f altitude_probe = center;
                altitude_probe.z() = config_.navigation_altitude;
                probes.push_back(altitude_probe);
            }
            for (int axis = 0; axis < 3; ++axis) {
                for (int sign : {-1, 1}) {
                    rog_map::Vec3f probe = center;
                    probe[axis] += static_cast<double>(sign) *
                                   0.90 * half_size[axis];
                    probes.push_back(probe);
                }
            }
            for (const rog_map::Vec3f &probe : probes) {
                ++diagnostics.sampled_center_count;
                if (constructionTraversable(probe, map_view)) {
                    candidate_center = probe;
                    candidate_free = true;
                    break;
                }
            }
        }

        bool covered = false;
        for (const Bubble &bubble : bubbles) {
            if ((candidate_center - bubble.center).norm() + half_diagonal <=
                bubble.radius) {
                covered = true;
                break;
            }
        }
        if (covered) {
            continue;
        }

        if (candidate_free) {
            ++diagnostics.traversable_center_count;
            const double clearance = estimateClearance(candidate_center, map_view);
            if (clearance >= config_.min_clearance) {
                bubbles.push_back({candidate_center, clearance});
                if (clearance + 1.0e-9 >= half_diagonal) {
                    continue;
                }
            } else {
                ++diagnostics.clearance_rejected_count;
            }
        }

        if (leaf_box) {
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

    // Index by union root rather than iterating an unordered container. The
    // deterministic order prevents a capped region from selecting a different
    // set of components on equivalent rebuilds.
    std::vector<std::vector<std::size_t>> components(bubbles.size());
    for (std::size_t i = 0; i < bubbles.size(); ++i) {
        components[root(i)].push_back(i);
    }

    const rog_map::Vec3f region_center =
        minimum + rog_map::Vec3f::Constant(0.5 * config_.region_size);
    struct SelectedBubble {
        std::size_t index{0};
        std::uint8_t portal_mask{0};
    };
    std::vector<SelectedBubble> selected;
    selected.reserve(config_.max_nodes_per_region);
    const auto appendSelected = [&](const std::size_t index,
                                    const std::uint8_t portal_mask = 0U) {
        const auto existing = std::find_if(
            selected.begin(), selected.end(), [index](const SelectedBubble &item) {
                return item.index == index;
            });
        if (existing != selected.end()) {
            existing->portal_mask |= portal_mask;
            return;
        }
        if (selected.size() >= config_.max_nodes_per_region) {
            return;
        }
        selected.push_back({index, portal_mask});
    };

    for (const auto &members : components) {
        if (selected.size() >= config_.max_nodes_per_region) {
            break;
        }
        if (members.empty()) {
            continue;
        }

        // Keep a maximum-clearance core for the component. It represents open
        // volume while portal nodes below preserve turns, doors and vertical
        // connections which a single component representative would erase.
        std::size_t core = members.front();
        for (const std::size_t index : members) {
            const Bubble &candidate = bubbles[index];
            const Bubble &current = bubbles[core];
            if (candidate.radius > current.radius + 1.0e-9 ||
                (std::abs(candidate.radius - current.radius) <= 1.0e-9 &&
                 (candidate.center - region_center).squaredNorm() <
                     (current.center - region_center).squaredNorm())) {
                core = index;
            }
        }
        appendSelected(core);

        const int axes = config_.planar_mode ? 2 : 3;
        for (int axis = 0; axis < axes; ++axis) {
            for (int side = 0; side < 2; ++side) {
                std::size_t portal = members.front();
                double best_face_gap = std::numeric_limits<double>::infinity();
                bool reaches_face = false;
                for (const std::size_t index : members) {
                    const Bubble &bubble = bubbles[index];
                    const double face = side == 0
                        ? minimum[axis]
                        : minimum[axis] + config_.region_size;
                    const double center_gap = std::abs(bubble.center[axis] - face);
                    const double surface_gap = std::max(0.0, center_gap - bubble.radius);
                    const bool reaches = surface_gap <= config_.sample_spacing + 1.0e-9;
                    if ((reaches && !reaches_face) ||
                        (reaches == reaches_face &&
                         surface_gap < best_face_gap - 1.0e-9) ||
                        (reaches == reaches_face &&
                         std::abs(surface_gap - best_face_gap) <= 1.0e-9 &&
                         bubble.radius > bubbles[portal].radius)) {
                        portal = index;
                        best_face_gap = surface_gap;
                        reaches_face = reaches;
                    }
                }
                if (reaches_face) {
                    bool separated = true;
                    for (const SelectedBubble &item : selected) {
                        if ((bubbles[item.index].center - bubbles[portal].center).norm() <
                            0.5 * config_.candidate_separation) {
                            separated = false;
                            break;
                        }
                    }
                    const std::uint8_t face_mask = static_cast<std::uint8_t>(
                        1U << static_cast<unsigned int>(axis * 2 + side));
                    const bool already_selected = std::any_of(
                        selected.begin(), selected.end(),
                        [portal](const SelectedBubble &item) {
                            return item.index == portal;
                        });
                    if (already_selected || separated) {
                        appendSelected(portal, face_mask);
                    }
                }
            }
        }
    }

    for (const SelectedBubble &item : selected) {
        const std::size_t index = item.index;
        Node node;
        node.position = bubbles[index].center;
        node.clearance = bubbles[index].radius;
        node.state = NodeState::ACTIVE;
        node.portal_mask = item.portal_mask;
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

bool IncrementalTopologyGraph::popDirtyRegion(
    RegionKey &region,
    std::vector<RegionKey> &changed_dense_cells,
    rog_map::vec_Vec3f &evidence_seeds,
    const rog_map::Vec3f *focus) {
    std::lock_guard<std::mutex> lock(dirty_mutex_);
    changed_dense_cells.clear();
    evidence_seeds.clear();
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
    const auto changed = dirty_dense_cells_.find(region);
    if (changed != dirty_dense_cells_.end()) {
        changed_dense_cells.assign(changed->second.begin(),
                                   changed->second.end());
        dirty_dense_cells_.erase(changed);
    }
    const auto seeds = dirty_evidence_seeds_.find(region);
    if (seeds != dirty_evidence_seeds_.end()) {
        evidence_seeds = std::move(seeds->second);
        dirty_evidence_seeds_.erase(seeds);
    }
    return true;
}

void IncrementalTopologyGraph::rebuildRegion(
    const RegionKey &region,
    const std::vector<RegionKey> &changed_dense_cells,
    const rog_map::vec_Vec3f &evidence_seeds,
    const TopologyMapView &map_view) {
    CandidateDiagnostics diagnostics;
    std::vector<Node, Eigen::aligned_allocator<Node>> old_nodes;
    std::vector<rog_map::Vec3f,
                Eigen::aligned_allocator<rog_map::Vec3f>> nearby_external_nodes;
    bool region_initialized = false;
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        region_initialized = initialized_regions_.count(region) != 0U;
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
        if (config_.construction_mode ==
            ConstructionMode::PERSISTENT_BUBBLE_SKELETON) {
            const int reach = static_cast<int>(std::ceil(
                config_.candidate_separation / config_.region_size)) + 1;
            const int z_reach = config_.planar_mode ? 0 : reach;
            for (int dx = -reach; dx <= reach; ++dx) {
                for (int dy = -reach; dy <= reach; ++dy) {
                    for (int dz = -z_reach; dz <= z_reach; ++dz) {
                        const RegionKey neighbor_region{
                            region.x + dx, region.y + dy, region.z + dz};
                        if (neighbor_region == region) {
                            continue;
                        }
                        const auto neighbor_it = regions_.find(neighbor_region);
                        if (neighbor_it == regions_.end()) {
                            continue;
                        }
                        for (const NodeId id : neighbor_it->second) {
                            const auto node_it = nodes_.find(id);
                            if (node_it != nodes_.end()) {
                                nearby_external_nodes.push_back(
                                    node_it->second.node.position);
                            }
                        }
                    }
                }
            }
        }
    }
    auto candidates = generateCandidates(
        region, map_view, diagnostics, evidence_seeds,
        config_.construction_mode ==
                ConstructionMode::PERSISTENT_BUBBLE_SKELETON &&
            region_initialized);

    std::unordered_set<NodeId> matched;
    if (config_.construction_mode ==
        ConstructionMode::PERSISTENT_BUBBLE_SKELETON) {
        // Committed topology is append-only for the lifetime of the task.
        // Re-observation may refresh state, but it must not move or replace an
        // existing node. Only an explicit OCCUPIED observation invalidates it.
        std::vector<Node, Eigen::aligned_allocator<Node>> committed;
        committed.reserve(old_nodes.size() + candidates.size());
        for (const Node &old_node : old_nodes) {
            const TopologyMapView::EvidenceState state =
                map_view.evidenceState(old_node.position);
            if (state == TopologyMapView::EvidenceState::OCCUPIED) {
                continue;
            }
            Node retained = old_node;
            retained.state = state == TopologyMapView::EvidenceState::KNOWN_FREE
                ? NodeState::ACTIVE : NodeState::HISTORICAL;
            committed.push_back(retained);
            matched.insert(retained.id);
        }

        const auto tooCloseToCommitted = [&](const rog_map::Vec3f &position) {
            for (const Node &node : committed) {
                if ((node.position - position).norm() + 1.0e-9 <
                    config_.candidate_separation) {
                    return true;
                }
            }
            for (const rog_map::Vec3f &external : nearby_external_nodes) {
                if ((external - position).norm() + 1.0e-9 <
                    config_.candidate_separation) {
                    return true;
                }
            }
            return false;
        };
        for (Node &candidate : candidates) {
            if (committed.size() >= config_.max_nodes_per_region) {
                break;
            }
            if (tooCloseToCommitted(candidate.position)) {
                continue;
            }
            candidate.state = NodeState::ACTIVE;
            committed.push_back(candidate);
        }
        candidates = std::move(committed);
    } else {
        for (Node &candidate : candidates) {
            double best_distance = config_.stable_match_distance;
            NodeId best_id = 0;
            for (const Node &old_node : old_nodes) {
                if (matched.count(old_node.id) != 0U) {
                    continue;
                }
                if (!(denseCellOf(candidate.position) ==
                      denseCellOf(old_node.position))) {
                    continue;
                }
                const double distance =
                    (candidate.position - old_node.position).norm();
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

        // Dense debug mode retains committed samples when the rolling map can
        // no longer classify them, but removes samples confirmed occupied.
        for (const Node &old_node : old_nodes) {
            if (matched.count(old_node.id) != 0U) {
                continue;
            }
            if (map_view.evidenceState(old_node.position) ==
                TopologyMapView::EvidenceState::UNKNOWN) {
                Node historical = old_node;
                historical.state = NodeState::HISTORICAL;
                candidates.push_back(historical);
                matched.insert(old_node.id);
            }
        }
    }

    std::vector<NodeId> affected_ids;
    {
        std::unique_lock<std::shared_mutex> lock(graph_mutex_);
        const auto old_region_it = regions_.find(region);
        if (old_region_it != regions_.end()) {
            for (const NodeId old_id : old_region_it->second) {
                // Matched/committed nodes retain their ID and position. Exact
                // state changes below decide whether incident edges need
                // validation; only unmatched (confirmed occupied) nodes die.
                if (matched.count(old_id) != 0U) {
                    continue;
                }
                const auto old_node_it = nodes_.find(old_id);
                if (old_node_it == nodes_.end()) {
                    continue;
                }
                if (config_.construction_mode ==
                    ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
                    dense_node_index_.erase(
                        denseCellOf(old_node_it->second.node.position));
                }
                for (const auto &neighbor : old_node_it->second.neighbors) {
                    const auto neighbor_it = nodes_.find(neighbor.first);
                    if (neighbor_it != nodes_.end()) {
                        neighbor_it->second.neighbors.erase(old_id);
                    }
                }
                nodes_.erase(old_node_it);
            }
        }

        ++revision_;
        initialized_regions_.insert(region);
        last_candidate_diagnostics_ = diagnostics;
        if (candidates.empty()) {
            ++empty_region_count_;
        }
        for (Node &candidate : candidates) {
            if (candidate.id == 0) {
                candidate.id = next_node_id_++;
            }
            candidate.revision = revision_;
            if (candidate.state == NodeState::ACTIVE) {
                candidate.last_observed_revision = revision_;
            }
            const auto existing = nodes_.find(candidate.id);
            if (existing != nodes_.end()) {
                existing->second.node = candidate;
                continue;
            }
            NodeRecord record;
            record.node = candidate;
            record.region = region;
            nodes_.emplace(candidate.id, std::move(record));
            if (config_.construction_mode ==
                ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
                dense_node_index_[denseCellOf(candidate.position)] = candidate.id;
            }
            affected_ids.push_back(candidate.id);
        }
        if (!candidates.empty()) {
            std::vector<NodeId> region_ids;
            region_ids.reserve(candidates.size());
            for (const Node &candidate : candidates) {
                region_ids.push_back(candidate.id);
            }
            regions_[region] = std::move(region_ids);
        } else {
            regions_.erase(region);
        }
        ++rebuilt_region_count_;
    }

    // A new obstacle can cut an edge even when neither endpoint belongs to
    // this region. Dense mode can find all possible owners by direct lattice
    // lookup. The old region/edge scan remains for non-lattice bubble mode.
    const rog_map::Vec3f dirty_min = regionMin(region) -
        rog_map::Vec3f::Constant(config_.edge_sample_spacing);
    const rog_map::Vec3f dirty_max = regionMin(region) +
        rog_map::Vec3f::Constant(config_.region_size + config_.edge_sample_spacing);
    std::unordered_set<NodeId> affected_set(affected_ids.begin(), affected_ids.end());
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        if (config_.construction_mode ==
                ConstructionMode::DENSE_KNOWN_FREE_DEBUG &&
            !changed_dense_cells.empty()) {
            // Only edges with an endpoint close to a genuinely changed ROG
            // cell can have changed validity. This avoids rebuilding a whole
            // 4 m region when most 1 m topology samples are unchanged.
            const int reach = static_cast<int>(std::ceil(
                config_.connection_radius / config_.sample_spacing));
            const int z_reach = config_.planar_mode ? 0 : reach;
            for (const RegionKey &cell : changed_dense_cells) {
                for (int dx = -reach; dx <= reach; ++dx) {
                    for (int dy = -reach; dy <= reach; ++dy) {
                        for (int dz = -z_reach; dz <= z_reach; ++dz) {
                            const auto found = dense_node_index_.find(
                                {cell.x + dx, cell.y + dy, cell.z + dz});
                            if (found != dense_node_index_.end()) {
                                affected_set.insert(found->second);
                            }
                        }
                    }
                }
            }
        } else if (config_.construction_mode ==
                       ConstructionMode::DENSE_KNOWN_FREE_DEBUG &&
                   old_nodes.empty()) {
            // First construction only needs the newly inserted sources;
            // undirected insertion also updates already existing neighbors.
        } else if (config_.construction_mode ==
                   ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
            const double spacing = config_.sample_spacing;
            const rog_map::Vec3f minimum = regionMin(region);
            const rog_map::Vec3f maximum =
                minimum + rog_map::Vec3f::Constant(config_.region_size);
            const auto firstIndex = [spacing](const double lower) {
                return static_cast<int>(
                    std::ceil(lower / spacing - 0.5 - 1.0e-9));
            };
            const auto lastIndex = [spacing](const double upper) {
                return static_cast<int>(
                    std::ceil(upper / spacing - 0.5 - 1.0e-9)) - 1;
            };
            // A lattice endpoint farther than this many cells cannot own an
            // edge intersecting the rebuilt region.
            const int reach = static_cast<int>(std::floor(
                config_.connection_radius / spacing + 1.0e-9));
            const int first_x = firstIndex(minimum.x()) - reach;
            const int last_x = lastIndex(maximum.x()) + reach;
            const int first_y = firstIndex(minimum.y()) - reach;
            const int last_y = lastIndex(maximum.y()) + reach;
            const int first_z = config_.planar_mode
                ? 0 : firstIndex(minimum.z()) - reach;
            const int last_z = config_.planar_mode
                ? 0 : lastIndex(maximum.z()) + reach;
            for (int x = first_x; x <= last_x; ++x) {
                for (int y = first_y; y <= last_y; ++y) {
                    for (int z = first_z; z <= last_z; ++z) {
                        const auto found = dense_node_index_.find({x, y, z});
                        if (found != dense_node_index_.end()) {
                            affected_set.insert(found->second);
                        }
                    }
                }
            }
        } else {
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
        bool existed{false};
        bool crosses_region{false};
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
            if (config_.construction_mode ==
                ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
                const RegionKey source_cell =
                    denseCellOf(source_it->second.node.position);
                const int radius = static_cast<int>(std::floor(
                    config_.connection_radius / config_.sample_spacing +
                    1.0e-9));
                const int z_radius = config_.planar_mode ? 0 : radius;
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dz = -z_radius; dz <= z_radius; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            const auto indexed = dense_node_index_.find(
                                {source_cell.x + dx, source_cell.y + dy,
                                 source_cell.z + dz});
                            if (indexed == dense_node_index_.end()) {
                                continue;
                            }
                            const NodeId candidate_id = indexed->second;
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
                                     distance,
                                     source_it->second.neighbors.count(
                                         candidate_id) != 0U,
                                     false});
                            }
                        }
                    }
                }
            } else {
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
                                         distance,
                                         source_it->second.neighbors.count(
                                             candidate_id) != 0U,
                                         !(candidate_it->second.region ==
                                           source_region)});
                                }
                            }
                        }
                    }
                }
            }
            candidate_edges.insert(candidate_edges.end(), local.begin(), local.end());
        }
    }

    std::vector<CandidateEdge> valid_edges;
    valid_edges.reserve(candidate_edges.size());
    struct UndirectedEdgeKey {
        NodeId low{0};
        NodeId high{0};
        bool operator==(const UndirectedEdgeKey &other) const {
            return low == other.low && high == other.high;
        }
    };
    struct UndirectedEdgeKeyHash {
        std::size_t operator()(const UndirectedEdgeKey &key) const {
            std::size_t seed = std::hash<NodeId>{}(key.low);
            seed ^= std::hash<NodeId>{}(key.high) + 0x9e3779b9U +
                    (seed << 6U) + (seed >> 2U);
            return seed;
        }
    };
    std::unordered_map<UndirectedEdgeKey,
                       TopologyMapView::EvidenceState,
                       UndirectedEdgeKeyHash>
        validation_cache;
    validation_cache.reserve(candidate_edges.size());
    for (const CandidateEdge &edge : candidate_edges) {
        const UndirectedEdgeKey key{
            std::min(edge.from, edge.to), std::max(edge.from, edge.to)};
        auto cached = validation_cache.find(key);
        if (cached == validation_cache.end()) {
            const TopologyMapView::EvidenceState state = lineEvidence(
                edge.from_position, edge.to_position, map_view);
            cached = validation_cache.emplace(key, state).first;
        }
        if (cached->second == TopologyMapView::EvidenceState::KNOWN_FREE ||
            (cached->second == TopologyMapView::EvidenceState::UNKNOWN &&
             edge.existed)) {
            valid_edges.push_back(edge);
        }
    }

    // Cap neighbors only after collision/evidence validation. Otherwise an
    // invalid group of close candidates can consume the complete budget and
    // disconnect otherwise reachable height layers.
    std::unordered_map<NodeId, std::vector<CandidateEdge>> valid_by_source;
    valid_by_source.reserve(source_ids.size());
    for (const CandidateEdge &edge : valid_edges) {
        valid_by_source[edge.from].push_back(edge);
    }
    valid_edges.clear();
    const auto nearestFirst = [](const CandidateEdge &lhs,
                                 const CandidateEdge &rhs) {
        if (lhs.existed != rhs.existed) {
            return lhs.existed;
        }
        // Reserve connectivity across region boundaries before spending the
        // degree budget on redundant links inside one dense local cluster.
        if (lhs.crosses_region != rhs.crosses_region) {
            return lhs.crosses_region;
        }
        if (std::abs(lhs.distance - rhs.distance) > 1.0e-9) {
            return lhs.distance < rhs.distance;
        }
        return lhs.to < rhs.to;
    };
    for (const NodeId source_id : source_ids) {
        auto found = valid_by_source.find(source_id);
        if (found == valid_by_source.end()) {
            continue;
        }
        std::vector<CandidateEdge> &local = found->second;
        std::sort(local.begin(), local.end(), nearestFirst);
        if (config_.construction_mode !=
            ConstructionMode::DENSE_KNOWN_FREE_DEBUG) {
            // Existing connectivity is committed memory. Retain every still
            // valid historic edge and use the budget only for new additions.
            std::size_t new_edge_count = 0;
            for (const CandidateEdge &edge : local) {
                if (edge.existed || new_edge_count < config_.max_neighbors) {
                    valid_edges.push_back(edge);
                    if (!edge.existed) {
                        ++new_edge_count;
                    }
                }
            }
            continue;
        }

        // A dense lattice must never lose its local connectivity because of a
        // visualization/performance neighbor budget. Keep every valid
        // 8-neighbor (2D) / 26-neighbor (3D) lattice edge first, then use the
        // configured budget only for longer shortcut edges.
        std::vector<CandidateEdge> selected;
        selected.reserve(std::max(config_.max_neighbors, local.size()));
        const auto alreadySelected = [&selected](const NodeId id) {
            return std::any_of(
                selected.begin(), selected.end(),
                [id](const CandidateEdge &edge) { return edge.to == id; });
        };
        for (const CandidateEdge &edge : local) {
            const rog_map::Vec3f delta =
                (edge.to_position - edge.from_position).cwiseAbs();
            const bool adjacent_xy =
                delta.x() <= config_.sample_spacing + 1.0e-9 &&
                delta.y() <= config_.sample_spacing + 1.0e-9;
            const bool adjacent_z = config_.planar_mode
                ? delta.z() <= 1.0e-9
                : delta.z() <= config_.sample_spacing + 1.0e-9;
            if (adjacent_xy && adjacent_z) {
                selected.push_back(edge);
            }
        }

        // Also retain the closest valid portal above and below when known-free
        // layers are farther apart than one lattice cell.
        const auto appendBestVertical = [&](const bool above) {
            const CandidateEdge *best = nullptr;
            double best_horizontal = std::numeric_limits<double>::infinity();
            double best_vertical = std::numeric_limits<double>::infinity();
            for (const CandidateEdge &edge : local) {
                const rog_map::Vec3f delta =
                    edge.to_position - edge.from_position;
                if ((above && delta.z() <= 1.0e-9) ||
                    (!above && delta.z() >= -1.0e-9)) {
                    continue;
                }
                const double horizontal = delta.head<2>().squaredNorm();
                const double vertical = std::abs(delta.z());
                if (horizontal < best_horizontal - 1.0e-9 ||
                    (std::abs(horizontal - best_horizontal) <= 1.0e-9 &&
                     (vertical < best_vertical - 1.0e-9 ||
                      (std::abs(vertical - best_vertical) <= 1.0e-9 &&
                       (!best || edge.to < best->to))))) {
                    best = &edge;
                    best_horizontal = horizontal;
                    best_vertical = vertical;
                }
            }
            if (best && !alreadySelected(best->to)) {
                selected.push_back(*best);
            }
        };
        if (!config_.planar_mode) {
            appendBestVertical(true);
            appendBestVertical(false);
        }
        const std::size_t neighbor_budget =
            std::max(config_.max_neighbors, selected.size());
        for (const CandidateEdge &edge : local) {
            if (selected.size() >= neighbor_budget) {
                break;
            }
            if (!alreadySelected(edge.to)) {
                selected.push_back(edge);
            }
        }
        valid_edges.insert(valid_edges.end(), selected.begin(), selected.end());
    }

    // Apply existing edges first. New undirected edges are admitted only when
    // both endpoint degrees have capacity, so max_neighbors is a real graph
    // degree bound rather than a per-source candidate bound.
    std::sort(valid_edges.begin(), valid_edges.end(), nearestFirst);
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
        if (config_.construction_mode ==
                ConstructionMode::PERSISTENT_BUBBLE_SKELETON &&
            !edge.existed &&
            (from_it->second.neighbors.size() >= config_.max_neighbors ||
             to_it->second.neighbors.size() >= config_.max_neighbors)) {
            continue;
        }
        from_it->second.neighbors[edge.to] = edge.distance;
        to_it->second.neighbors[edge.from] = edge.distance;
    }
}

bool IncrementalTopologyGraph::refreshPublicTopology(
    const TopologyMapView &map_view, const rog_map::Vec3f *focus) {
    struct Component {
        std::vector<NodeId> nodes;
        NodeId smallest_id{std::numeric_limits<NodeId>::max()};
    };
    struct AttachmentCandidate {
        double distance{0.0};
        std::size_t component_index{0};
        NodeId node_id{0};
    };

    std::unique_lock<std::shared_mutex> lock(graph_mutex_);
    const bool has_focus = focus != nullptr && focus->allFinite();
    // A smaller movement cannot alter a collision-validated attachment at the
    // graph sampling resolution.  This keeps an idle world-lifetime graph
    // from paying O(V + E) every 0.1 s merely because odometry is high-rate.
    const double focus_reclass_distance =
        std::max(0.05, config_.edge_sample_spacing);
    const bool focus_changed =
        has_focus != has_public_topology_focus_ ||
        (has_focus && (!public_topology_initialized_ ||
                       (public_topology_focus_ - *focus).norm() >=
                           focus_reclass_distance));
    if (public_topology_initialized_ &&
        public_topology_revision_ == revision_ && !focus_changed) {
        return false;
    }
    const auto finish = [this, has_focus, focus]() {
        public_topology_revision_ = revision_;
        public_topology_initialized_ = true;
        has_public_topology_focus_ = has_focus;
        if (has_focus) {
            public_topology_focus_ = *focus;
        } else {
            public_topology_focus_.setZero();
        }
    };
    for (auto &entry : nodes_) {
        entry.second.public_node = false;
    }
    pending_node_count_ = nodes_.size();
    isolated_node_count_ = 0;
    connected_component_count_ = 0;
    if (nodes_.empty()) {
        finish();
        return true;
    }

    // The edge maintenance code writes an undirected graph.  Treat a missing
    // reverse edge as invalid nevertheless: a partially updated edge must not
    // make an otherwise isolated candidate publicly visible.
    const auto hasValidNeighbor = [this](const NodeId from,
                                         const NodeId to) {
        const auto target = nodes_.find(to);
        return target != nodes_.end() &&
               target->second.neighbors.count(from) != 0U;
    };

    std::unordered_set<NodeId> visited;
    visited.reserve(nodes_.size());
    std::vector<Component> components;
    components.reserve(nodes_.size());
    for (const auto &entry : nodes_) {
        const NodeId start = entry.first;
        if (visited.count(start) != 0U) {
            continue;
        }
        Component component;
        std::vector<NodeId> frontier{start};
        visited.insert(start);
        while (!frontier.empty()) {
            const NodeId current = frontier.back();
            frontier.pop_back();
            const auto current_it = nodes_.find(current);
            if (current_it == nodes_.end()) {
                continue;
            }
            component.nodes.push_back(current);
            component.smallest_id = std::min(component.smallest_id, current);
            for (const auto &neighbor : current_it->second.neighbors) {
                if (!hasValidNeighbor(current, neighbor.first) ||
                    visited.count(neighbor.first) != 0U) {
                    continue;
                }
                visited.insert(neighbor.first);
                frontier.push_back(neighbor.first);
            }
        }
        if (component.nodes.size() < 2U) {
            ++isolated_node_count_;
            continue;
        }
        components.push_back(std::move(component));
    }
    connected_component_count_ = components.size();
    if (components.empty()) {
        finish();
        return true;
    }

    std::vector<std::size_t> public_components;
    if (!config_.publish_connected_component_only) {
        public_components.reserve(components.size());
        for (std::size_t index = 0; index < components.size(); ++index) {
            public_components.push_back(index);
        }
    } else if (has_focus) {
        const double attach_radius = config_.public_component_attach_radius > 0.0
            ? config_.public_component_attach_radius
            : config_.connection_radius;
        std::vector<AttachmentCandidate> candidates;
        candidates.reserve(components.size());
        for (std::size_t index = 0; index < components.size(); ++index) {
            AttachmentCandidate best;
            best.distance = std::numeric_limits<double>::infinity();
            best.component_index = index;
            for (const NodeId id : components[index].nodes) {
                const auto node_it = nodes_.find(id);
                if (node_it == nodes_.end()) {
                    continue;
                }
                const double distance =
                    (node_it->second.node.position - *focus).norm();
                if (distance <= attach_radius &&
                    (distance < best.distance - 1.0e-9 ||
                     (std::abs(distance - best.distance) <= 1.0e-9 &&
                      id < best.node_id))) {
                    best.distance = distance;
                    best.node_id = id;
                }
            }
            if (best.node_id != 0) {
                candidates.push_back(best);
            }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const AttachmentCandidate &lhs,
                     const AttachmentCandidate &rhs) {
                      if (std::abs(lhs.distance - rhs.distance) > 1.0e-9) {
                          return lhs.distance < rhs.distance;
                      }
                      return lhs.node_id < rhs.node_id;
                  });
        for (const AttachmentCandidate &candidate : candidates) {
            const auto node_it = nodes_.find(candidate.node_id);
            if (node_it != nodes_.end() && lineTraversable(
                    *focus, node_it->second.node.position, map_view,
                    config_.edge_sample_spacing)) {
                public_components.push_back(candidate.component_index);
                break;
            }
        }
        // A focus with no KNOWN_FREE attachment is intentionally published as
        // an empty graph.  Selecting a remote component would let a planner
        // believe it had a route through unknown/blocked space.
    } else {
        // Standalone users/tests may not provide odometry.  Keep one stable
        // component in that case, never an arbitrary disconnected union.
        std::size_t selected = 0;
        for (std::size_t index = 1; index < components.size(); ++index) {
            if (components[index].nodes.size() > components[selected].nodes.size() ||
                (components[index].nodes.size() ==
                     components[selected].nodes.size() &&
                 components[index].smallest_id <
                     components[selected].smallest_id)) {
                selected = index;
            }
        }
        public_components.push_back(selected);
    }

    std::size_t public_count = 0;
    for (const std::size_t index : public_components) {
        for (const NodeId id : components[index].nodes) {
            const auto node_it = nodes_.find(id);
            if (node_it != nodes_.end()) {
                node_it->second.public_node = true;
                ++public_count;
            }
        }
    }
    pending_node_count_ = nodes_.size() - public_count;
    finish();
    return true;
}

void IncrementalTopologyGraph::publishSearchSnapshot() {
    auto next = std::make_shared<SearchSnapshot>();
    {
        std::shared_lock<std::shared_mutex> lock(graph_mutex_);
        next->config = config_;
        next->revision = revision_;
        next->graph.reserve(nodes_.size());
        for (const auto &entry : nodes_) {
            if (!entry.second.public_node) {
                continue;
            }
            Node node = entry.second.node;
            node.expansion_mask = 0U;
            std::unordered_map<NodeId, double> public_neighbors;
            public_neighbors.reserve(entry.second.neighbors.size());
            for (const auto &neighbor : entry.second.neighbors) {
                const auto neighbor_it = nodes_.find(neighbor.first);
                if (neighbor_it == nodes_.end() ||
                    !neighbor_it->second.public_node ||
                    neighbor_it->second.neighbors.count(entry.first) == 0U) {
                    continue;
                }
                public_neighbors.emplace(neighbor.first, neighbor.second);
            }
            // refreshPublicTopology() already filters zero-degree candidates.
            // Retain this guard as a publication boundary invariant.
            if (public_neighbors.empty()) {
                continue;
            }
            if (node.state == NodeState::ACTIVE) {
                std::uint8_t connected_faces = 0U;
                for (const auto &neighbor : public_neighbors) {
                    const auto neighbor_it = nodes_.find(neighbor.first);
                    if (neighbor_it == nodes_.end() ||
                        neighbor_it->second.node.state != NodeState::ACTIVE) {
                        continue;
                    }
                    const RegionKey &source_region = entry.second.region;
                    const RegionKey &target_region = neighbor_it->second.region;
                    if (target_region.x < source_region.x) {
                        connected_faces |= PORTAL_NEG_X;
                    } else if (target_region.x > source_region.x) {
                        connected_faces |= PORTAL_POS_X;
                    }
                    if (target_region.y < source_region.y) {
                        connected_faces |= PORTAL_NEG_Y;
                    } else if (target_region.y > source_region.y) {
                        connected_faces |= PORTAL_POS_Y;
                    }
                    if (target_region.z < source_region.z) {
                        connected_faces |= PORTAL_NEG_Z;
                    } else if (target_region.z > source_region.z) {
                        connected_faces |= PORTAL_POS_Z;
                    }
                }
                node.expansion_mask = static_cast<std::uint8_t>(
                    node.portal_mask & static_cast<std::uint8_t>(~connected_faces));
            }
            next->graph.emplace(entry.first,
                                SearchNode{node, std::move(public_neighbors)});
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
    std::vector<RegionKey> changed_dense_cells;
    rog_map::vec_Vec3f evidence_seeds;
    while (rebuilt < max_regions &&
           popDirtyRegion(region, changed_dense_cells, evidence_seeds, focus)) {
        rebuildRegion(region, changed_dense_cells, evidence_seeds, map_view);
        ++rebuilt;
    }
    // Raw candidates may be retained across map-window movement, but only a
    // connected, collision-attached component may be used by readers.
    // Re-evaluate even with no dirty region so a moved robot focus changes
    // the public root immediately at the worker cadence.
    const bool public_topology_changed =
        refreshPublicTopology(map_view, focus);
    if (config_.snapshot_every_update &&
        (rebuilt > 0 || public_topology_changed)) {
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
        output.construction_mode = search->config.construction_mode;
        output.nodes.reserve(search->graph.size());
        for (const auto &entry : search->graph) {
            output.nodes.push_back(entry.second.node);
            for (const auto &neighbor : entry.second.neighbors) {
                if (entry.first < neighbor.first) {
                    Edge edge;
                    edge.from = entry.first;
                    edge.to = neighbor.first;
                    edge.cost = neighbor.second;
                    edge.validated_revision = search->revision;
                    const auto target = search->graph.find(neighbor.first);
                    if (target != search->graph.end()) {
                        edge.polyline = {entry.second.node.position,
                                         target->second.node.position};
                    }
                    output.edges.push_back(std::move(edge));
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
        output.raw_node_count = nodes_.size();
        output.pending_node_count = pending_node_count_;
        output.isolated_node_count = isolated_node_count_;
        output.connected_component_count = connected_component_count_;
    }
    output.known_free_cell_count = output.nodes.size();
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
        output.public_node_count = 0;
        output.public_edge_count = 0;
        output.pending_node_count = pending_node_count_;
        output.isolated_node_count = isolated_node_count_;
        output.connected_component_count = connected_component_count_;
        for (const auto &entry : nodes_) {
            if (!entry.second.public_node) {
                continue;
            }
            ++output.public_node_count;
            for (const auto &neighbor : entry.second.neighbors) {
                const auto neighbor_it = nodes_.find(neighbor.first);
                if (neighbor_it != nodes_.end() &&
                    neighbor_it->second.public_node &&
                    neighbor_it->second.neighbors.count(entry.first) != 0U) {
                    ++output.public_edge_count;
                }
            }
        }
        output.public_edge_count /= 2U;
    }
    output.known_free_cell_count = output.node_count;
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

    struct QueueEntry {
        double estimated_total_cost{0.0};
        double path_cost{0.0};
        NodeId node_id{0};
    };
    // Edge cost is Euclidean node distance, so Euclidean distance to the goal
    // is admissible and consistent.  The tie-breaker keeps route selection
    // deterministic when two nodes have the same f score.
    const auto lowerEstimatedCost = [](const QueueEntry &lhs,
                                       const QueueEntry &rhs) {
        if (std::abs(lhs.estimated_total_cost - rhs.estimated_total_cost) > 1.0e-12) {
            return lhs.estimated_total_cost > rhs.estimated_total_cost;
        }
        if (std::abs(lhs.path_cost - rhs.path_cost) > 1.0e-12) {
            return lhs.path_cost > rhs.path_cost;
        }
        return lhs.node_id > rhs.node_id;
    };
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        decltype(lowerEstimatedCost)> queue(lowerEstimatedCost);
    std::unordered_map<NodeId, double> distance;
    std::unordered_map<NodeId, NodeId> parent;
    for (std::size_t i = 0; i < max_attach; ++i) {
        const NodeId start_id = start_candidates[i].second;
        const double path_cost = start_candidates[i].first;
        distance[start_id] = path_cost;
        parent[start_id] = 0;
        queue.push({path_cost + (graph.at(start_id).node.position - goal).norm(),
                    path_cost, start_id});
    }

    double best_goal_cost = std::numeric_limits<double>::infinity();
    NodeId best_goal_parent = 0;
    while (!queue.empty()) {
        const QueueEntry current = queue.top();
        queue.pop();
        const auto distance_it = distance.find(current.node_id);
        if (distance_it == distance.end() ||
            current.path_cost > distance_it->second + 1.0e-12) {
            continue;
        }
        if (current.estimated_total_cost >= best_goal_cost) {
            break;
        }
        const auto goal_it = goal_links.find(current.node_id);
        if (goal_it != goal_links.end() &&
            current.path_cost + goal_it->second < best_goal_cost) {
            best_goal_cost = current.path_cost + goal_it->second;
            best_goal_parent = current.node_id;
        }
        const auto graph_it = graph.find(current.node_id);
        if (graph_it == graph.end()) {
            continue;
        }
        for (const auto &neighbor : graph_it->second.neighbors) {
            if (graph.count(neighbor.first) == 0U) {
                continue;
            }
            const double proposed = current.path_cost + neighbor.second;
            const auto known = distance.find(neighbor.first);
            if (known == distance.end() || proposed < known->second) {
                distance[neighbor.first] = proposed;
                parent[neighbor.first] = current.node_id;
                queue.push({proposed +
                                (graph.at(neighbor.first).node.position - goal).norm(),
                            proposed, neighbor.first});
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
