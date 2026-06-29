#include <general_core/exploration/coverage_grid.hpp>

#include <algorithm>
#include <cmath>

namespace general_planner {

CoverageGrid::CoverageGrid()
    : CoverageGrid(Config{})
{
}

CoverageGrid::CoverageGrid(Config config)
    : config_(config)
{
}

void CoverageGrid::reset()
{
    cells_.clear();
    total_visits_ = 0;
}

void CoverageGrid::observePose(const general_utils::Vec3f &position,
                               const double stamp)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }
    CoverageCellRecord &record = cells_[keyForPosition(position)];
    if (record.visits == 0) {
        record.first_visit_stamp = stamp;
    }
    record.last_visit_stamp = stamp;
    ++record.visits;
    ++total_visits_;
    prune();
}

bool CoverageGrid::recentlyVisited(const general_utils::Vec3f &position,
                                   const double stamp) const
{
    if (!config_.enable || !position.allFinite()) {
        return false;
    }
    return revisitPenalty(position, stamp) > 0.0;
}

double CoverageGrid::revisitPenalty(const general_utils::Vec3f &position,
                                    const double stamp) const
{
    if (!config_.enable || !position.allFinite()) {
        return 0.0;
    }

    const Key center = keyForPosition(position);
    const double resolution = std::max(1.0e-3, config_.resolution);
    const int radius_steps =
            std::max(0, static_cast<int>(std::ceil(config_.revisit_radius / resolution)));
    const double time_window = std::max(0.0, config_.revisit_time_window);
    double penalty = 0.0;

    for (int dx = -radius_steps; dx <= radius_steps; ++dx) {
        for (int dy = -radius_steps; dy <= radius_steps; ++dy) {
            for (int dz = -radius_steps; dz <= radius_steps; ++dz) {
                const Key key{center.x + dx, center.y + dy, center.z + dz};
                const auto it = cells_.find(key);
                if (it == cells_.end()) {
                    continue;
                }
                const CoverageCellRecord &record = it->second;
                if (time_window > 0.0 && stamp - record.last_visit_stamp > time_window) {
                    continue;
                }
                penalty += 1.0 + 0.25 * static_cast<double>(std::max(0, record.visits - 1));
            }
        }
    }
    return penalty;
}

int CoverageGrid::visitedCellCount() const
{
    return static_cast<int>(cells_.size());
}

int CoverageGrid::totalVisitCount() const
{
    return total_visits_;
}

std::size_t CoverageGrid::KeyHasher::operator()(const Key &key) const
{
    std::size_t seed = 0;
    const auto mix = [&seed](const int value) {
        const std::size_t h = std::hash<int>{}(value);
        seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
}

CoverageGrid::Key CoverageGrid::keyForPosition(const general_utils::Vec3f &position) const
{
    const double resolution = std::max(1.0e-3, config_.resolution);
    return Key{
            static_cast<int>(std::floor(position.x() / resolution)),
            static_cast<int>(std::floor(position.y() / resolution)),
            static_cast<int>(std::floor(position.z() / resolution))};
}

void CoverageGrid::prune()
{
    const int max_cells = std::max(1, config_.max_cells);
    while (static_cast<int>(cells_.size()) > max_cells) {
        auto oldest = cells_.begin();
        for (auto it = cells_.begin(); it != cells_.end(); ++it) {
            if (it->second.last_visit_stamp < oldest->second.last_visit_stamp) {
                oldest = it;
            }
        }
        total_visits_ = std::max(0, total_visits_ - oldest->second.visits);
        cells_.erase(oldest);
    }
}

} // namespace general_planner
