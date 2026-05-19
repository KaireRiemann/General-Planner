#pragma once

#include <vector>

#include <map_manager/global_region_grid.hpp>
#include <super_utils/type_utils.hpp>

namespace general_planner {

struct CoverageGuideConfig {
    bool enable{true};
    int max_active_regions{30};

    double weight_distance{1.0};
    double weight_height{1.5};
    double weight_gain{2.0};
    double weight_revisit{0.5};
    double weight_switch_region{0.5};

    bool use_astar_cost{false};
    int max_route_length{10};
};

struct CoverageNode {
    int region_id{-1};
    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    int frontier_count{0};
    double information_gain{0.0};
    bool active{false};
    bool explored{false};
};

class CoverageGuidePlanner {
public:
    explicit CoverageGuidePlanner(const CoverageGuideConfig &cfg);

    bool buildGuidePath(const super_utils::Vec3f &robot_pos,
                        const std::vector<ExplorationRegion> &active_regions,
                        std::vector<int> &ordered_region_ids);

    bool selectPriorityRegions(const std::vector<int> &ordered_region_ids,
                               int max_regions,
                               std::vector<int> &selected_region_ids) const;

    void reset();

private:
    double estimateRegionCost(const super_utils::Vec3f &from,
                              const ExplorationRegion &to) const;

private:
    CoverageGuideConfig cfg_;
    std::vector<int> last_ordered_region_ids_;
};

}  // namespace general_planner
