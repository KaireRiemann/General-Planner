#include <map_manager/coverage_guide_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace general_planner {

CoverageGuidePlanner::CoverageGuidePlanner(const CoverageGuideConfig &cfg)
        : cfg_(cfg) {
    cfg_.max_active_regions = std::max(1, cfg_.max_active_regions);
    cfg_.max_route_length = std::max(1, cfg_.max_route_length);
}

bool CoverageGuidePlanner::buildGuidePath(const super_utils::Vec3f &robot_pos,
                                          const std::vector<ExplorationRegion> &active_regions,
                                          std::vector<int> &ordered_region_ids) {
    ordered_region_ids.clear();
    if (!cfg_.enable || active_regions.empty() || !robot_pos.allFinite()) {
        return false;
    }

    std::vector<ExplorationRegion> remaining;
    remaining.reserve(std::min(static_cast<int>(active_regions.size()), cfg_.max_active_regions));
    for (const auto &region : active_regions) {
        if (!region.active || region.explored || !region.center.allFinite()) {
            continue;
        }
        remaining.push_back(region);
        if (static_cast<int>(remaining.size()) >= cfg_.max_active_regions) {
            break;
        }
    }
    if (remaining.empty()) {
        return false;
    }

    super_utils::Vec3f current = robot_pos;
    while (!remaining.empty() &&
           static_cast<int>(ordered_region_ids.size()) < cfg_.max_route_length) {
        int best_index = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(remaining.size()); ++i) {
            const double cost = estimateRegionCost(current, remaining[static_cast<size_t>(i)]);
            if (cost < best_cost) {
                best_cost = cost;
                best_index = i;
            }
        }
        if (best_index < 0) {
            break;
        }
        const ExplorationRegion selected = remaining[static_cast<size_t>(best_index)];
        ordered_region_ids.push_back(selected.id);
        current = selected.center;
        remaining.erase(remaining.begin() + best_index);
    }

    last_ordered_region_ids_ = ordered_region_ids;
    return !ordered_region_ids.empty();
}

bool CoverageGuidePlanner::selectPriorityRegions(const std::vector<int> &ordered_region_ids,
                                                 const int max_regions,
                                                 std::vector<int> &selected_region_ids) const {
    selected_region_ids.clear();
    if (ordered_region_ids.empty() || max_regions <= 0) {
        return false;
    }
    selected_region_ids.reserve(static_cast<size_t>(max_regions));
    std::unordered_set<int> inserted;
    for (const int id : ordered_region_ids) {
        if (inserted.find(id) != inserted.end()) {
            continue;
        }
        selected_region_ids.push_back(id);
        inserted.insert(id);
        if (static_cast<int>(selected_region_ids.size()) >= max_regions) {
            break;
        }
    }
    return !selected_region_ids.empty();
}

void CoverageGuidePlanner::reset() {
    last_ordered_region_ids_.clear();
}

double CoverageGuidePlanner::estimateRegionCost(const super_utils::Vec3f &from,
                                                const ExplorationRegion &to) const {
    const double distance = (to.center - from).norm();
    const double distance_norm = distance / 10.0;
    const double height = std::abs(to.center.z() - from.z());
    const double gain_norm =
            std::clamp(std::log1p(std::max(0.0, to.information_gain_estimate)) / std::log1p(300.0),
                       0.0,
                       1.0);
    const double revisit_penalty = to.last_visit_time >= 0.0 ? 1.0 : 0.0;
    return cfg_.weight_distance * distance_norm +
           cfg_.weight_height * height -
           cfg_.weight_gain * gain_norm +
           cfg_.weight_revisit * revisit_penalty;
}

}  // namespace general_planner
