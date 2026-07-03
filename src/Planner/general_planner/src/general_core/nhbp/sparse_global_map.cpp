#include <general_core/nhbp/sparse_global_map.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace general_planner::nhbp {

SparseGlobalMap::SparseGlobalMap()
    : SparseGlobalMap(Config{})
{
}

SparseGlobalMap::SparseGlobalMap(Config config)
    : config_(config)
{
}

void SparseGlobalMap::configure(Config config)
{
    config_ = config;
    if (!config_.enable) {
        reset();
    }
}

void SparseGlobalMap::reset()
{
    records_.clear();
}

void SparseGlobalMap::observeBoundaryCell(const general_utils::Vec3f &position,
                                          const SparseCellState state,
                                          const double stamp)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }
    SparseCellRecord &record = records_[key(position)];
    if (record.observations == 0) {
        record.position = position;
    } else {
        const double n = static_cast<double>(std::max(1, record.observations));
        record.position = (record.position * n + position) / (n + 1.0);
    }
    record.state = state;
    record.stamp = stamp;
    ++record.observations;
    record.confidence = std::min(1.0,
                                 0.25 + 0.15 * static_cast<double>(record.observations));
    prune(stamp);
}

SparseCellState SparseGlobalMap::query(const general_utils::Vec3f &position,
                                       const double stamp) const
{
    if (!config_.enable || !position.allFinite()) {
        return SparseCellState::UNKNOWN;
    }
    const auto it = records_.find(key(position));
    if (it == records_.end() || stale(it->second, stamp)) {
        return SparseCellState::UNKNOWN;
    }
    return it->second.state;
}

SparseQueryResult SparseGlobalMap::queryDetailed(const general_utils::Vec3f &position,
                                                 const double stamp) const
{
    SparseQueryResult result;
    if (!config_.enable || !position.allFinite()) {
        return result;
    }
    const auto it = records_.find(key(position));
    if (it == records_.end()) {
        return result;
    }
    const SparseCellRecord &record = it->second;
    result.observed = true;
    result.stale = stale(record, stamp);
    result.age = std::max(0.0, stamp - record.stamp);
    result.observations = record.observations;
    result.confidence = result.stale ? 0.0 : record.confidence;
    result.state = result.stale ? SparseCellState::UNKNOWN : record.state;
    return result;
}

std::vector<SparseCellRecord> SparseGlobalMap::frontierRecords(
        const general_utils::Vec3f &center,
        const double radius,
        const double stamp,
        const int max_count) const
{
    std::vector<SparseCellRecord> out;
    if (!config_.enable || !center.allFinite() || radius <= 0.0 || max_count == 0) {
        return out;
    }
    const double radius_sq = radius * radius;
    for (const auto &entry : records_) {
        const SparseCellRecord &record = entry.second;
        if (record.state != SparseCellState::FRONTIER_BOUNDARY ||
            stale(record, stamp) ||
            (record.position - center).squaredNorm() > radius_sq) {
            continue;
        }
        out.push_back(record);
    }
    std::sort(out.begin(), out.end(), [&](const SparseCellRecord &lhs,
                                          const SparseCellRecord &rhs) {
        const double lhs_score = (lhs.position - center).squaredNorm() -
                                 0.25 * lhs.confidence;
        const double rhs_score = (rhs.position - center).squaredNorm() -
                                 0.25 * rhs.confidence;
        return lhs_score < rhs_score;
    });
    if (max_count > 0 && static_cast<int>(out.size()) > max_count) {
        out.resize(max_count);
    }
    return out;
}

int SparseGlobalMap::recordCount() const
{
    return static_cast<int>(records_.size());
}

int SparseGlobalMap::frontierCount(const double stamp) const
{
    int count = 0;
    for (const auto &entry : records_) {
        if (entry.second.state == SparseCellState::FRONTIER_BOUNDARY &&
            !stale(entry.second, stamp)) {
            ++count;
        }
    }
    return count;
}

std::string SparseGlobalMap::key(const general_utils::Vec3f &position) const
{
    const double resolution = std::max(1.0e-3, config_.resolution);
    const auto q = [resolution](const double value) {
        return static_cast<long long>(std::llround(value / resolution));
    };
    std::ostringstream oss;
    oss << q(position.x()) << ":" << q(position.y()) << ":" << q(position.z());
    return oss.str();
}

bool SparseGlobalMap::stale(const SparseCellRecord &record, const double stamp) const
{
    return config_.stale_time > 0.0 && stamp - record.stamp > config_.stale_time;
}

void SparseGlobalMap::prune(const double stamp)
{
    if (!config_.enable) {
        reset();
        return;
    }
    for (auto it = records_.begin(); it != records_.end();) {
        if (stale(it->second, stamp)) {
            it = records_.erase(it);
        } else {
            ++it;
        }
    }
    const int max_records = std::max(1, config_.max_records);
    while (static_cast<int>(records_.size()) > max_records) {
        auto oldest = records_.begin();
        for (auto it = records_.begin(); it != records_.end(); ++it) {
            if (it->second.stamp < oldest->second.stamp) {
                oldest = it;
            }
        }
        records_.erase(oldest);
    }
}

const char *toString(const SparseCellState state)
{
    switch (state) {
        case SparseCellState::UNKNOWN:
            return "UNKNOWN";
        case SparseCellState::FREE_BOUNDARY:
            return "FREE_BOUNDARY";
        case SparseCellState::OCCUPIED_BOUNDARY:
            return "OCCUPIED_BOUNDARY";
        case SparseCellState::FRONTIER_BOUNDARY:
            return "FRONTIER_BOUNDARY";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
