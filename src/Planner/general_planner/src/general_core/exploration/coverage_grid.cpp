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
    ++record.observed_free_count;
    record.stale = false;
    if (record.visits >= std::max(1, config_.covered_visit_threshold) &&
        record.observed_unknown_count == 0 &&
        record.observed_frontier_count == 0) {
        record.covered = true;
    }
    ++total_visits_;
    prune();
}

void CoverageGrid::observeFrontierEvidence(const general_utils::Vec3f &position,
                                           const double stamp,
                                           const int unknown_count,
                                           const int frontier_count,
                                           const double information_gain)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }
    CoverageCellRecord &record = cells_[keyForPosition(position)];
    if (record.visits == 0 && record.first_visit_stamp <= 0.0) {
        record.first_visit_stamp = stamp;
    }
    record.last_visit_stamp = stamp;
    record.last_information_gain_stamp = stamp;
    record.observed_unknown_count += std::max(0, unknown_count);
    record.observed_frontier_count += std::max(0, frontier_count);
    const double alpha = std::clamp(config_.information_gain_alpha, 0.0, 1.0);
    const double gain = std::max(0.0, information_gain);
    record.information_gain_ema =
            record.information_gain_ema <= 0.0
                    ? gain
                    : alpha * gain + (1.0 - alpha) * record.information_gain_ema;
    record.covered = record.observed_unknown_count == 0 &&
                     record.observed_frontier_count == 0 &&
                     record.visits >= std::max(1, config_.covered_visit_threshold);
    if (record.observed_unknown_count > 0 ||
        record.observed_frontier_count > 0 ||
        record.information_gain_ema > 1.0e-3) {
        record.covered = false;
        record.stale = false;
        record.no_progress_basin = false;
    }
    prune();
}

void CoverageGrid::markCoveredNear(const general_utils::Vec3f &position,
                                   const double stamp,
                                   const double radius)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }
    forEachCellNear(position, radius, [stamp](CoverageCellRecord &record) {
        record.covered = true;
        record.stale = false;
        record.last_visit_stamp = std::max(record.last_visit_stamp, stamp);
        record.observed_unknown_count = 0;
        record.observed_frontier_count = 0;
        record.information_gain_ema *= 0.5;
    });
}

void CoverageGrid::markNoProgressBasin(const general_utils::Vec3f &position,
                                       const double stamp,
                                       const double radius)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }
    forEachCellNear(position, radius, [stamp](CoverageCellRecord &record) {
        record.no_progress_basin = true;
        record.last_visit_stamp = std::max(record.last_visit_stamp, stamp);
    });
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
                if (recordStale(record, stamp) ||
                    (time_window > 0.0 && stamp - record.last_visit_stamp > time_window)) {
                    continue;
                }
                double cell_penalty =
                        1.0 + 0.25 * static_cast<double>(std::max(0, record.visits - 1));
                if (record.covered) {
                    cell_penalty += 1.0;
                }
                if (record.no_progress_basin) {
                    cell_penalty += 2.0;
                }
                const bool low_information_gain =
                        record.information_gain_ema < 1.0 &&
                        record.observed_unknown_count == 0 &&
                        record.observed_frontier_count == 0;
                if (low_information_gain) {
                    cell_penalty += 0.75;
                }
                const double evidence =
                        std::min(1.0,
                                 record.information_gain_ema / 20.0 +
                                         static_cast<double>(record.observed_frontier_count) / 20.0 +
                                         static_cast<double>(record.observed_unknown_count) / 40.0);
                cell_penalty *= (1.0 - 0.45 * evidence);
                penalty += std::max(0.0, cell_penalty);
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

int CoverageGrid::coveredCellCount() const
{
    int count = 0;
    for (const auto &entry : cells_) {
        if (entry.second.covered) {
            ++count;
        }
    }
    return count;
}

int CoverageGrid::noProgressBasinCount(const double stamp) const
{
    int count = 0;
    for (const auto &entry : cells_) {
        if (entry.second.no_progress_basin && !recordStale(entry.second, stamp)) {
            ++count;
        }
    }
    return count;
}

double CoverageGrid::recentInformationGain(const double stamp) const
{
    const double window = std::max(0.0, config_.revisit_time_window);
    double gain = 0.0;
    for (const auto &entry : cells_) {
        const CoverageCellRecord &record = entry.second;
        if (record.last_information_gain_stamp <= 0.0 ||
            (window > 0.0 && stamp - record.last_information_gain_stamp > window)) {
            continue;
        }
        gain += record.information_gain_ema;
    }
    return gain;
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

bool CoverageGrid::recordStale(const CoverageCellRecord &record,
                               const double stamp) const
{
    const double stale_time = std::max(0.0, config_.stale_time);
    return stale_time > 0.0 && stamp - record.last_visit_stamp > stale_time;
}

template <typename Fn>
void CoverageGrid::forEachCellNear(const general_utils::Vec3f &position,
                                   const double radius,
                                   Fn &&fn)
{
    const double resolution = std::max(1.0e-3, config_.resolution);
    const int radius_steps =
            std::max(0, static_cast<int>(std::ceil(std::max(0.0, radius) / resolution)));
    const Key center = keyForPosition(position);
    for (int dx = -radius_steps; dx <= radius_steps; ++dx) {
        for (int dy = -radius_steps; dy <= radius_steps; ++dy) {
            for (int dz = -radius_steps; dz <= radius_steps; ++dz) {
                const Key key{center.x + dx, center.y + dy, center.z + dz};
                auto it = cells_.find(key);
                if (it == cells_.end()) {
                    continue;
                }
                fn(it->second);
            }
        }
    }
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
