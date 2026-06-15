#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <general_core/frontier_database.hpp>

namespace general_planner {

struct ExplorationRegion {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};
    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};

    std::vector<int> frontier_ids;
    std::vector<int> neighbor_region_ids;

    double unknown_volume{0.0};
    double known_free_volume{0.0};
    double coverage_priority{0.0};

    bool visited{false};
    bool active{true};
};

class ExplorationRegionGraph {
public:
    using Ptr = std::shared_ptr<ExplorationRegionGraph>;

    struct Config {
        double region_resolution{2.0};
        double connectivity_radius{3.0};
        double min_region_unknown_volume{1.0};
        double revisit_penalty{5.0};
        int max_region_num{256};
    };

    ExplorationRegionGraph();
    explicit ExplorationRegionGraph(const Config &cfg);

    void reset();

    void update(const ExplorationMemoryGrid &memory,
                const FrontierDatabase &frontier_db,
                const super_utils::Vec3f &robot_pos,
                double stamp);

    int currentRegion(const super_utils::Vec3f &robot_pos) const;
    int regionOfFrontier(int frontier_id) const;

    bool getRegion(int id, ExplorationRegion &out) const;
    std::vector<int> activeRegionIds() const;
    std::vector<ExplorationRegion> regions() const;

    double regionCoveragePriority(int region_id) const;
    double graphDistanceCost(int from_region, int to_region) const;

    void markVisited(int region_id);

private:
    struct RegionKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const RegionKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
        bool operator<(const RegionKey &other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return z < other.z;
        }
    };

    struct RegionKeyHasher {
        std::size_t operator()(const RegionKey &key) const;
    };

    RegionKey makeKey(const super_utils::Vec3f &pos) const;
    super_utils::Vec3f keyCenter(const RegionKey &key) const;
    int nearestRegion(const super_utils::Vec3f &pos) const;

    Config cfg_;
    std::unordered_map<int, ExplorationRegion> regions_;
    std::unordered_map<RegionKey, int, RegionKeyHasher> key_to_region_id_;
    std::unordered_map<int, RegionKey> region_id_to_key_;
    std::unordered_map<int, int> frontier_to_region_;
    std::unordered_map<RegionKey, bool, RegionKeyHasher> visited_keys_;
    int current_region_id_{-1};
    double last_stamp_{0.0};
};

}  // namespace general_planner
