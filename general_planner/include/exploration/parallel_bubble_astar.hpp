#pragma once

#include <limits>
#include <memory>

#include <map_manager/map_manager.hpp>

#include "exploration/exploration_types.hpp"

namespace general_planner {
namespace exploration {

class ParallelBubbleAstar {
public:
    using Ptr = std::shared_ptr<ParallelBubbleAstar>;

    enum RetCode {
        REACH_END = 1,
        NO_PATH = 2,
        START_FAIL = 3,
        END_FAIL = 4,
        TIME_OUT = 5
    };

    struct Config {
        double resolution{0.5};
        double safe_distance{0.45};
        double lambda_heu{1.0};
        int max_nodes{8000};
        bool shorten_path{true};
    };

    ParallelBubbleAstar(Config cfg, MapManager::Ptr map_manager);

    int search(const super_utils::Vec3f &start,
               const super_utils::Vec3f &goal,
               super_utils::vec_E<super_utils::Vec3f> &path,
               double timeout,
               bool only_raycast = false,
               const super_utils::Vec3f &bbox_min = super_utils::Vec3f::Constant(-std::numeric_limits<double>::max()),
               const super_utils::Vec3f &bbox_max = super_utils::Vec3f::Constant(std::numeric_limits<double>::max())) const;

    void calculatePathCost(const super_utils::vec_E<super_utils::Vec3f> &path,
                           double &cost) const;

private:
    using GridIndex = Eigen::Vector3i;

    GridIndex posToIndex(const super_utils::Vec3f &pos) const;
    super_utils::Vec3f indexToPos(const GridIndex &idx) const;

    bool isNodeSafe(const super_utils::Vec3f &pos,
                    const super_utils::Vec3f &bbox_min,
                    const super_utils::Vec3f &bbox_max) const;
    bool lineSafe(const super_utils::Vec3f &start,
                  const super_utils::Vec3f &goal,
                  const super_utils::Vec3f &bbox_min,
                  const super_utils::Vec3f &bbox_max) const;
    bool shortenPath(super_utils::vec_E<super_utils::Vec3f> &path,
                     const super_utils::Vec3f &bbox_min,
                     const super_utils::Vec3f &bbox_max) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
};

}  // namespace exploration
}  // namespace general_planner
