#pragma once

#include <mutex>
#include <unordered_map>
#include <vector>

#include <map_manager/global_exploration_map.hpp>
#include <super_utils/type_utils.hpp>

namespace general_planner {

struct ExplorationRegion {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};

    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};

    int observed_free_count{0};
    int occupied_count{0};
    int frontier_count{0};
    int unknown_count_estimate{0};

    bool active{false};
    bool explored{false};

    double information_gain_estimate{0.0};
    double last_visit_time{-1.0};

    std::vector<int> frontier_cluster_ids;
};

struct GlobalRegionGridConfig {
    bool enable{true};
    double region_size_xy{4.0};
    double region_size_z{2.0};
    int min_frontier_count{10};
    double explored_ratio_threshold{0.85};
};

class GlobalRegionGrid {
public:
    using Ptr = std::shared_ptr<GlobalRegionGrid>;

    explicit GlobalRegionGrid(const GlobalRegionGridConfig &cfg);

    void updateFromGlobalMap(const GlobalExplorationMap &global_map,
                             const std::vector<FrontierCluster> &clusters);

    void getActiveRegions(std::vector<ExplorationRegion> &regions) const;

    bool getRegion(int id, ExplorationRegion &region) const;

    int positionToRegionId(const super_utils::Vec3f &p) const;

    void reset();

private:
    void initializeRegionGeometry(const super_utils::Vec3f &p, ExplorationRegion &region) const;

private:
    GlobalRegionGridConfig cfg_;
    mutable std::mutex mutex_;
    std::unordered_map<int, ExplorationRegion> regions_;
};

}  // namespace general_planner
