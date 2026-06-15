#include <exploration_manager/global_exploration_planner.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace general_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfCost = 1.0e9;

struct LkhProblemFiles {
    std::string tsp_path;
    std::string par_path;
    std::string out_path;
    std::string log_path;
    int scale{100};
};

double wrapAngleDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

double pathLength(const super_utils::vec_E<super_utils::Vec3f> &path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

bool isUnknownLike(const rog_map::GridType type) {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}

bool isOccupiedLike(const rog_map::GridType type) {
    return type == rog_map::GridType::OCCUPIED ||
           type == rog_map::GridType::OUT_OF_MAP;
}

std::string shellQuote(const std::string &value) {
    std::string quoted = "'";
    for (const char c : value) {
        if (c == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(c);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

std::string findExecutableOnPath(const std::string &name) {
    const char *path_env = std::getenv("PATH");
    if (path_env == nullptr || name.empty()) {
        return {};
    }

    std::stringstream path_stream(path_env);
    std::string directory;
    while (std::getline(path_stream, directory, ':')) {
        if (directory.empty()) {
            continue;
        }
        const std::filesystem::path candidate =
                std::filesystem::path(directory) / name;
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec) &&
            !std::filesystem::is_directory(candidate, ec)) {
            return candidate.string();
        }
    }
    return {};
}

std::string resolveBundledLkhBinary() {
#ifdef GENERAL_PLANNER_BUNDLED_LKH_BINARY
    const std::string bundled_path = GENERAL_PLANNER_BUNDLED_LKH_BINARY;
    if (!bundled_path.empty()) {
        std::error_code ec;
        if (std::filesystem::exists(bundled_path, ec) &&
            !std::filesystem::is_directory(bundled_path, ec)) {
            return bundled_path;
        }
    }
#endif
    return findExecutableOnPath("general_planner_lkh");
}

std::string resolveLkhBinary(const std::string &configured_path) {
    if (!configured_path.empty()) {
        std::error_code ec;
        if (std::filesystem::exists(configured_path, ec) &&
            !std::filesystem::is_directory(configured_path, ec)) {
            return configured_path;
        }
        const std::string path_candidate = findExecutableOnPath(configured_path);
        if (!path_candidate.empty()) {
            return path_candidate;
        }
        return {};
    }

    const std::string bundled_path = resolveBundledLkhBinary();
    if (!bundled_path.empty()) {
        return bundled_path;
    }

    for (const char *candidate : {"LKH", "lkh", "LKH3", "lkh3"}) {
        const std::string path_candidate = findExecutableOnPath(candidate);
        if (!path_candidate.empty()) {
            return path_candidate;
        }
    }
    return {};
}

double viewpointReward(const ExplorationViewpoint &viewpoint) {
    return 0.02 * std::max(0.0, viewpoint.score);
}

bool writeLkhProblemFiles(const LkhAtspSolver::Config &cfg,
                          const Eigen::MatrixXd &cost_matrix,
                          LkhProblemFiles &files,
                          std::string &reason) {
    std::error_code ec;
    std::filesystem::create_directories(cfg.work_dir, ec);
    if (ec) {
        reason = "failed to create LKH work dir";
        return false;
    }

    const std::string base = cfg.work_dir + "/" + cfg.problem_name;
    files.tsp_path = base + ".tsp";
    files.par_path = base + ".par";
    files.out_path = base + ".txt";
    files.log_path = base + ".log";
    files.scale = std::max(1, cfg.scale);

    std::filesystem::remove(files.out_path, ec);
    ec.clear();
    std::filesystem::remove(files.log_path, ec);

    const int dim = static_cast<int>(cost_matrix.rows());
    {
        std::ofstream problem(files.tsp_path);
        if (!problem.is_open()) {
            reason = "failed to write LKH problem file";
            return false;
        }
        problem << "NAME : " << cfg.problem_name << "\n"
                << "TYPE : ATSP\n"
                << "DIMENSION : " << dim << "\n"
                << "EDGE_WEIGHT_TYPE : EXPLICIT\n"
                << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
                << "EDGE_WEIGHT_SECTION\n";
        for (int r = 0; r < dim; ++r) {
            for (int c = 0; c < dim; ++c) {
                const double cost = std::isfinite(cost_matrix(r, c))
                                            ? std::max(0.0, cost_matrix(r, c))
                                            : 1.0e6;
                problem << static_cast<int>(std::round(cost * files.scale)) << " ";
            }
            problem << "\n";
        }
        problem << "EOF\n";
    }

    {
        std::ofstream params(files.par_path);
        if (!params.is_open()) {
            reason = "failed to write LKH parameter file";
            return false;
        }
        params << "PROBLEM_FILE = " << files.tsp_path << "\n"
               << "OUTPUT_TOUR_FILE = " << files.out_path << "\n"
               << "GAIN23 = NO\n"
               << "RUNS = 1\n"
               << "TIME_LIMIT = 0.2\n"
               << "TRACE_LEVEL = 0\n";
    }

    return true;
}

bool readLkhResult(const Eigen::MatrixXd &cost_matrix,
                   const LkhProblemFiles &files,
                   std::vector<int> &tour,
                   double &total_cost,
                   std::string &reason) {
    const int dim = static_cast<int>(cost_matrix.rows());
    std::ifstream result(files.out_path);
    if (!result.is_open()) {
        reason = "LKH result file missing";
        return false;
    }

    std::string line;
    bool in_tour = false;
    total_cost = 0.0;
    tour.clear();
    while (std::getline(result, line)) {
        if (line.find("COMMENT : Length") != std::string::npos) {
            const std::size_t pos = line.find_last_of(' ');
            if (pos != std::string::npos) {
                total_cost = static_cast<double>(std::stoi(line.substr(pos + 1))) /
                             static_cast<double>(std::max(1, files.scale));
            }
        }
        if (line == "TOUR_SECTION") {
            in_tour = true;
            continue;
        }
        if (!in_tour) {
            continue;
        }
        int one_based_id = 0;
        try {
            one_based_id = std::stoi(line);
        } catch (...) {
            continue;
        }
        if (one_based_id == -1) {
            break;
        }
        const int id = one_based_id - 1;
        if (id >= 0 && id < dim) {
            tour.push_back(id);
        }
    }

    if (tour.empty()) {
        reason = "LKH returned empty tour";
        return false;
    }

    const auto zero_it = std::find(tour.begin(), tour.end(), 0);
    if (zero_it != tour.end()) {
        std::rotate(tour.begin(), zero_it, tour.end());
    } else {
        tour.insert(tour.begin(), 0);
    }

    if (total_cost <= 0.0) {
        for (std::size_t i = 1; i < tour.size(); ++i) {
            total_cost += cost_matrix(tour[i - 1], tour[i]);
        }
    }
    return true;
}
}  // namespace

LkhAtspSolver::LkhAtspSolver()
        : LkhAtspSolver(Config{}) {
}

LkhAtspSolver::LkhAtspSolver(Config cfg)
        : cfg_(std::move(cfg)) {
}

bool LkhAtspSolver::solve(const Eigen::MatrixXd &cost_matrix,
                          std::vector<int> &tour,
                          double &total_cost,
                          bool &used_lkh,
                          std::string &reason) const {
    tour.clear();
    total_cost = 0.0;
    used_lkh = false;
    reason.clear();

    if (cost_matrix.rows() != cost_matrix.cols() || cost_matrix.rows() <= 0) {
        reason = "invalid ATSP cost matrix";
        return false;
    }
    if (cost_matrix.rows() == 1) {
        tour = {0};
        reason = "single-node ATSP solved without LKH";
        return true;
    }
    if (cost_matrix.rows() == 2) {
        const bool solved = solveFallback(cost_matrix, tour, total_cost, reason);
        if (solved) {
            reason = "two-node ATSP solved without LKH";
        }
        return solved;
    }

    const std::string lkh_binary = resolveLkhBinary(cfg_.binary_path);
    if (!lkh_binary.empty()) {
        used_lkh = solveWithExternalLkh(cost_matrix, lkh_binary, tour, total_cost, reason);
        if (used_lkh) {
            return true;
        }
    }

    if (!cfg_.allow_fallback) {
        if (reason.empty()) {
            reason = "LKH binary unavailable and fallback disabled";
        }
        return false;
    }

    used_lkh = false;
    return solveFallback(cost_matrix, tour, total_cost, reason);
}

bool LkhAtspSolver::solveWithExternalLkh(const Eigen::MatrixXd &cost_matrix,
                                         const std::string &binary_path,
                                         std::vector<int> &tour,
                                         double &total_cost,
                                         std::string &reason) const {
    LkhProblemFiles files;
    if (!writeLkhProblemFiles(cfg_, cost_matrix, files, reason)) {
        return false;
    }

    const std::string command =
            shellQuote(binary_path) + " " + shellQuote(files.par_path) +
            " > " + shellQuote(files.log_path) + " 2>&1";
    const int ret = std::system(command.c_str());
    if (ret != 0) {
        reason = "external LKH call failed";
        return false;
    }

    if (!readLkhResult(cost_matrix, files, tour, total_cost, reason)) {
        return false;
    }

    reason = "external LKH solved";
    return true;
}

bool LkhAtspSolver::solveFallback(const Eigen::MatrixXd &cost_matrix,
                                  std::vector<int> &tour,
                                  double &total_cost,
                                  std::string &reason) const {
    const int dim = static_cast<int>(cost_matrix.rows());
    std::vector<char> used(static_cast<std::size_t>(dim), 0);
    tour.clear();
    tour.reserve(static_cast<std::size_t>(dim));
    tour.push_back(0);
    used[0] = 1;
    int current = 0;

    for (int step = 1; step < dim; ++step) {
        double best_cost = std::numeric_limits<double>::infinity();
        int best_id = -1;
        for (int candidate = 1; candidate < dim; ++candidate) {
            if (used[static_cast<std::size_t>(candidate)] != 0) {
                continue;
            }
            const double cost = cost_matrix(current, candidate);
            if (cost < best_cost) {
                best_cost = cost;
                best_id = candidate;
            }
        }
        if (best_id < 0) {
            break;
        }
        used[static_cast<std::size_t>(best_id)] = 1;
        tour.push_back(best_id);
        current = best_id;
    }

    bool improved = true;
    while (improved && tour.size() > 3) {
        improved = false;
        for (std::size_t i = 1; i + 2 < tour.size(); ++i) {
            for (std::size_t j = i + 1; j + 1 < tour.size(); ++j) {
                const double old_cost =
                        cost_matrix(tour[i - 1], tour[i]) +
                        cost_matrix(tour[j], tour[j + 1]);
                const double new_cost =
                        cost_matrix(tour[i - 1], tour[j]) +
                        cost_matrix(tour[i], tour[j + 1]);
                if (new_cost + 1.0e-6 < old_cost) {
                    std::reverse(tour.begin() + static_cast<long>(i),
                                 tour.begin() + static_cast<long>(j + 1));
                    improved = true;
                }
            }
        }
    }

    total_cost = 0.0;
    for (std::size_t i = 1; i < tour.size(); ++i) {
        total_cost += cost_matrix(tour[i - 1], tour[i]);
    }
    reason = "LKH unavailable, used deterministic ATSP fallback";
    return tour.size() == static_cast<std::size_t>(dim);
}

ExplorationCostEvaluator::ExplorationCostEvaluator(Config cfg,
                                                   MapManager::Ptr map_manager,
                                                   path_search::Astar::Ptr astar)
        : cfg_(cfg),
          map_manager_(std::move(map_manager)),
          astar_(std::move(astar)) {
}

double ExplorationCostEvaluator::computeCost(const super_utils::Vec3f &from,
                                             const super_utils::Vec3f &to,
                                             const double from_yaw,
                                             const double to_yaw,
                                             const super_utils::Vec3f &from_vel,
                                             const bool allow_unknown_motion,
                                             const bool hard_fail,
                                             super_utils::vec_E<super_utils::Vec3f> *path) const {
    if (path != nullptr) {
        path->clear();
    }
    if (map_manager_ == nullptr || !from.allFinite() || !to.allFinite()) {
        return hard_fail ? kInfCost : 1.0e3;
    }

    const double distance = (to - from).norm();
    super_utils::vec_E<super_utils::Vec3f> local_path;
    bool path_found = false;

    const bool unknown_as_occ =
            !allow_unknown_motion && cfg_.unknown_as_occupied_for_motion;
    if (map_manager_->insideLocalMap(from) &&
        map_manager_->insideLocalMap(to) &&
        map_manager_->isLineFree(from, to, true, unknown_as_occ)) {
        local_path = {from, to};
        path_found = true;
    }

    if (!path_found &&
        cfg_.use_astar &&
        astar_ != nullptr &&
        distance <= std::max(1.0, cfg_.hybrid_search_radius) &&
        map_manager_->insideLocalMap(from) &&
        map_manager_->insideLocalMap(to)) {
        const int flag = allow_unknown_motion
                                 ? (path_search::ON_INF_MAP |
                                    path_search::UNKNOWN_AS_FREE)
                                 : (path_search::ON_PROB_MAP |
                                    path_search::UNKNOWN_AS_OCCUPIED |
                                    path_search::DONT_USE_INF_NEIGHBOR);
        const double horizon = std::max(cfg_.hybrid_search_radius, distance * 1.8 + 2.0);
        const super_utils::RET_CODE ret = astar_->pointToPointPathSearch(from,
                                                                         to,
                                                                         flag,
                                                                         horizon,
                                                                         local_path,
                                                                         0.02);
        path_found = ret == super_utils::REACH_GOAL && !local_path.empty();
    }

    if (!path_found) {
        if (hard_fail) {
            return kInfCost;
        }
        local_path = {from, to};
    }

    if (path != nullptr) {
        *path = local_path;
    }

    const double length = path_found ? estimatePathLength(local_path) : distance;
    double pos_time = length / std::max(0.1, cfg_.max_vel);
    super_utils::Vec3f to_goal = to - from;
    if (from_vel.norm() > 0.1 && to_goal.norm() > 1.0e-3) {
        const double projected_vel = from_vel.dot(to_goal.normalized());
        if (projected_vel < 0.0) {
            pos_time += std::min(2.0, -projected_vel / std::max(0.1, cfg_.max_acc));
        }
    }

    double cost = std::max(pos_time, yawTime(from_yaw, to_yaw));
    if (!path_found) {
        cost += 1000.0 + distance;
    }
    if (allow_unknown_motion && path_found) {
        bool touches_unknown = false;
        const double step = std::max(0.1, map_manager_->getResolution());
        for (std::size_t i = 1; i < local_path.size() && !touches_unknown; ++i) {
            const super_utils::Vec3f segment = local_path[i] - local_path[i - 1];
            const int steps = std::max(1, static_cast<int>(std::ceil(segment.norm() / step)));
            for (int s = 0; s <= steps; ++s) {
                const super_utils::Vec3f query =
                        local_path[i - 1] + segment * (static_cast<double>(s) /
                                                       static_cast<double>(steps));
                if (map_manager_->insideLocalMap(query) &&
                    isUnknownLike(map_manager_->getGridType(query))) {
                    touches_unknown = true;
                    break;
                }
            }
        }
        if (touches_unknown) {
            cost *= std::max(1.0, cfg_.unknown_penalty_factor);
        }
    }
    return cost;
}

double ExplorationCostEvaluator::estimatePathLength(
        const super_utils::vec_E<super_utils::Vec3f> &path) const {
    return pathLength(path);
}

double ExplorationCostEvaluator::yawTime(const double from_yaw,
                                         const double to_yaw) const {
    const double diff = std::abs(wrapAngleDiff(to_yaw, from_yaw));
    return diff / std::max(0.1, cfg_.max_yaw_rate);
}

ViewpointSelector::ViewpointSelector(Config cfg, MapManager::Ptr map_manager)
        : cfg_(cfg),
          map_manager_(std::move(map_manager)) {
}

void ViewpointSelector::selectViewpoints(const FrontierCluster &cluster,
                                         const super_utils::Vec3f &robot_pos,
                                         super_utils::vec_E<ExplorationViewpoint> &viewpoints) const {
    viewpoints.clear();
    if (map_manager_ == nullptr || cluster.cells.empty()) {
        return;
    }

    super_utils::vec_E<ExplorationViewpoint> raw_viewpoints;
    const int radius_num = std::max(1, cfg_.radius_sample_num);
    const int yaw_num = std::max(4, cfg_.yaw_sample_num);
    const double min_radius = std::max(0.1, cfg_.min_radius);
    const double max_radius = std::max(min_radius, cfg_.max_radius);

    for (int ri = 0; ri < radius_num; ++ri) {
        const double alpha = radius_num == 1
                                     ? 0.0
                                     : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
        const double radius = min_radius + alpha * (max_radius - min_radius);
        for (int yi = 0; yi < yaw_num; ++yi) {
            const double angle = -kPi + 2.0 * kPi * static_cast<double>(yi) /
                                        static_cast<double>(yaw_num);
            super_utils::Vec3f sample =
                    cluster.center + radius * super_utils::Vec3f(std::cos(angle),
                                                                 std::sin(angle),
                                                                 0.0);
            sample.z() = cluster.center.z() + cfg_.height_offset;
            if ((sample - robot_pos).norm() < 0.2 || !isViewpointSafe(sample)) {
                continue;
            }

            super_utils::Vec3f dir_sum = super_utils::Vec3f::Zero();
            const int stride = std::max(1, static_cast<int>(cluster.cells.size()) / 48);
            for (std::size_t i = 0; i < cluster.cells.size(); i += static_cast<std::size_t>(stride)) {
                super_utils::Vec3f dir = cluster.cells[i].position - sample;
                dir.z() = 0.0;
                if (dir.norm() > 1.0e-3) {
                    dir_sum += dir.normalized();
                }
            }
            if (dir_sum.norm() < 1.0e-3) {
                dir_sum = cluster.center - sample;
            }
            const double yaw = wrapYaw(std::atan2(dir_sum.y(), dir_sum.x()));
            const int visible = countVisibleCells(sample, yaw, cluster);
            if (visible < std::max(1, cfg_.min_visible_cells)) {
                continue;
            }

            ExplorationViewpoint viewpoint;
            viewpoint.frontier_cluster_id = cluster.id;
            viewpoint.position = sample;
            viewpoint.yaw = yaw;
            viewpoint.visible_frontier_cells = visible;
            viewpoint.unknown_gain = estimateUnknownGain(sample, yaw);
            viewpoint.score = static_cast<double>(visible) + 0.1 * viewpoint.unknown_gain;
            raw_viewpoints.push_back(viewpoint);
        }
    }

    std::sort(raw_viewpoints.begin(),
              raw_viewpoints.end(),
              [](const ExplorationViewpoint &lhs, const ExplorationViewpoint &rhs) {
                  if (lhs.visible_frontier_cells != rhs.visible_frontier_cells) {
                      return lhs.visible_frontier_cells > rhs.visible_frontier_cells;
                  }
                  return lhs.unknown_gain > rhs.unknown_gain;
              });

    if (raw_viewpoints.empty()) {
        return;
    }

    const int best_visible = std::max(1, raw_viewpoints.front().visible_frontier_cells);
    const int max_count = std::max(1, cfg_.top_view_num);
    for (const auto &viewpoint : raw_viewpoints) {
        if (static_cast<int>(viewpoints.size()) >= max_count) {
            break;
        }
        if (viewpoint.visible_frontier_cells <
            static_cast<int>(std::ceil(static_cast<double>(best_visible) *
                                       std::clamp(cfg_.max_decay, 0.0, 1.0)))) {
            break;
        }
        viewpoints.push_back(viewpoint);
    }
    if (viewpoints.empty()) {
        viewpoints.push_back(raw_viewpoints.front());
    }
}

bool ViewpointSelector::isViewpointSafe(const super_utils::Vec3f &pos) const {
    return map_manager_ != nullptr &&
           map_manager_->isKnownFreeForViewpoint(pos, cfg_.safe_distance) &&
           !isNearUnknown(pos);
}

bool ViewpointSelector::isNearUnknown(const super_utils::Vec3f &pos) const {
    if (cfg_.unknown_clearance <= 1.0e-6) {
        return false;
    }
    const double res = std::max(1.0e-3, map_manager_->getResolution());
    const int bound = std::max(1, static_cast<int>(std::ceil(cfg_.unknown_clearance / res)));
    for (int dx = -bound; dx <= bound; ++dx) {
        for (int dy = -bound; dy <= bound; ++dy) {
            for (int dz = -bound; dz <= bound; ++dz) {
                const super_utils::Vec3f query = pos + res * super_utils::Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(query) ||
                    isUnknownLike(map_manager_->getGridType(query))) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ViewpointSelector::isNearOccupied(const super_utils::Vec3f &pos) const {
    const double res = std::max(1.0e-3, map_manager_->getResolution());
    const int bound = std::max(0, static_cast<int>(std::floor(cfg_.occupied_clearance / res)));
    for (int dx = -bound; dx <= bound; ++dx) {
        for (int dy = -bound; dy <= bound; ++dy) {
            for (int dz = -bound; dz <= bound; ++dz) {
                const super_utils::Vec3f query = pos + res * super_utils::Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(query) ||
                    isOccupiedLike(map_manager_->getInfGridType(query))) {
                    return true;
                }
            }
        }
    }
    return false;
}

int ViewpointSelector::countVisibleCells(const super_utils::Vec3f &viewpoint,
                                         const double yaw,
                                         const FrontierCluster &cluster) const {
    if (isNearOccupied(viewpoint)) {
        return 0;
    }

    int visible = 0;
    const int max_rays = 96;
    const int stride = std::max(1, static_cast<int>(cluster.cells.size()) / max_rays);
    for (std::size_t i = 0; i < cluster.cells.size(); i += static_cast<std::size_t>(stride)) {
        const super_utils::Vec3f &target = cluster.cells[i].position;
        if ((target - viewpoint).norm() > std::max(0.5, cfg_.sensor_range) ||
            !insideYawFov(viewpoint, yaw, target)) {
            continue;
        }
        if (map_manager_->isLineFree(viewpoint, target, true, false)) {
            ++visible;
        }
    }
    return visible;
}

double ViewpointSelector::estimateUnknownGain(const super_utils::Vec3f &viewpoint,
                                              const double yaw) const {
    const double res = std::max(0.1, cfg_.map_resolution);
    const double range = std::max(0.5, cfg_.sensor_range);
    const int bound = static_cast<int>(std::ceil(range / res));
    int unknown_count = 0;
    int checked = 0;
    for (int dx = -bound; dx <= bound; ++dx) {
        for (int dy = -bound; dy <= bound; ++dy) {
            const super_utils::Vec3f query =
                    viewpoint + res * super_utils::Vec3f(dx, dy, 0.0);
            const double dist = (query - viewpoint).norm();
            if (dist < res || dist > range ||
                !insideYawFov(viewpoint, yaw, query) ||
                !map_manager_->insideLocalMap(query)) {
                continue;
            }
            ++checked;
            const auto type = map_manager_->getGridType(query);
            if (isUnknownLike(type) &&
                map_manager_->isLineFree(viewpoint, query, true, false)) {
                ++unknown_count;
            }
        }
    }
    if (checked == 0) {
        return 0.0;
    }
    return static_cast<double>(unknown_count);
}

bool ViewpointSelector::insideYawFov(const super_utils::Vec3f &viewpoint,
                                     const double yaw,
                                     const super_utils::Vec3f &target) const {
    super_utils::Vec3f dir = target - viewpoint;
    dir.z() = 0.0;
    if (dir.norm() < 1.0e-3) {
        return true;
    }
    const double target_yaw = std::atan2(dir.y(), dir.x());
    const double half_fov = std::max(1.0, cfg_.horizontal_fov_deg) * kPi / 360.0;
    return std::abs(wrapAngleDiff(target_yaw, yaw)) <= half_fov;
}

double ViewpointSelector::wrapYaw(const double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

CoverageGridManager::CoverageGridManager()
        : CoverageGridManager(Config{}) {
}

CoverageGridManager::CoverageGridManager(Config cfg)
        : cfg_(cfg) {
}

void CoverageGridManager::reset() {
    active_nodes_.clear();
}

void CoverageGridManager::update(
        const rog_map::vec_E<FrontierCluster> &clusters,
        const std::unordered_map<int, super_utils::vec_E<ExplorationViewpoint>> &viewpoints_by_cluster,
        const super_utils::Vec3f &robot_pos) {
    active_nodes_.clear();
    std::map<std::tuple<int, int, int>, int> key_to_node;

    for (const auto &cluster : clusters) {
        const auto view_it = viewpoints_by_cluster.find(cluster.id);
        if (view_it == viewpoints_by_cluster.end() || view_it->second.empty()) {
            continue;
        }
        const super_utils::Vec3f anchor = view_it->second.front().position;
        const super_utils::Vec3i key = makeKey(anchor);
        const auto tuple_key = std::make_tuple(key.x(), key.y(), key.z());
        int node_index = -1;
        const auto key_it = key_to_node.find(tuple_key);
        if (key_it == key_to_node.end()) {
            ExplorationCoverageNode node;
            node.id = static_cast<int>(active_nodes_.size());
            node.key = key;
            const super_utils::Vec3f cell_min =
                    cfg_.cell_size * key.cast<double>();
            node.bbox_min = cell_min;
            node.bbox_max = cell_min + super_utils::Vec3f::Constant(cfg_.cell_size);
            active_nodes_.push_back(node);
            node_index = static_cast<int>(active_nodes_.size()) - 1;
            key_to_node.emplace(tuple_key, node_index);
        } else {
            node_index = key_it->second;
        }

        ExplorationCoverageNode &node = active_nodes_[static_cast<std::size_t>(node_index)];
        node.cluster_ids.push_back(cluster.id);
        node.frontier_cell_count += std::max(1, cluster.size);
        node.center += anchor;
    }

    for (auto &node : active_nodes_) {
        if (!node.cluster_ids.empty()) {
            node.center /= static_cast<double>(node.cluster_ids.size());
        } else {
            node.center = 0.5 * (node.bbox_min + node.bbox_max);
        }
    }

    std::sort(active_nodes_.begin(),
              active_nodes_.end(),
              [&robot_pos](const ExplorationCoverageNode &lhs,
                           const ExplorationCoverageNode &rhs) {
                  if (lhs.frontier_cell_count != rhs.frontier_cell_count) {
                      return lhs.frontier_cell_count > rhs.frontier_cell_count;
                  }
                  return (lhs.center - robot_pos).squaredNorm() <
                         (rhs.center - robot_pos).squaredNorm();
              });
    if (static_cast<int>(active_nodes_.size()) > std::max(1, cfg_.max_active_nodes)) {
        active_nodes_.resize(static_cast<std::size_t>(cfg_.max_active_nodes));
    }
    for (int i = 0; i < static_cast<int>(active_nodes_.size()); ++i) {
        active_nodes_[static_cast<std::size_t>(i)].id = i;
    }
}

const super_utils::vec_E<ExplorationCoverageNode> &CoverageGridManager::activeNodes() const {
    return active_nodes_;
}

super_utils::Vec3i CoverageGridManager::makeKey(const super_utils::Vec3f &pos) const {
    const double cell_size = std::max(0.5, cfg_.cell_size);
    return super_utils::Vec3i(static_cast<int>(std::floor(pos.x() / cell_size)),
                              static_cast<int>(std::floor(pos.y() / cell_size)),
                              static_cast<int>(std::floor(pos.z() / cell_size)));
}

GlobalCoveragePlanner::GlobalCoveragePlanner(Config cfg,
                                             MapManager::Ptr map_manager,
                                             path_search::Astar::Ptr astar)
        : cfg_(std::move(cfg)),
          coverage_grid_(cfg_.grid),
          cost_evaluator_(cfg_.cost, map_manager, astar),
          atsp_solver_(cfg_.lkh),
          map_manager_(std::move(map_manager)),
          astar_(std::move(astar)) {
}

void GlobalCoveragePlanner::reset() {
    coverage_grid_.reset();
}

bool GlobalCoveragePlanner::plan(
        const super_utils::StatePVAJ &robot_state,
        const double current_yaw,
        const rog_map::vec_E<FrontierCluster> &clusters,
        const std::unordered_map<int, super_utils::vec_E<ExplorationViewpoint>> &viewpoints_by_cluster,
        ExplorationCoveragePlan &plan) {
    plan = ExplorationCoveragePlan{};
    if (clusters.empty() || viewpoints_by_cluster.empty()) {
        plan.reason = "no clusters or viewpoints for coverage planning";
        return false;
    }

    const super_utils::Vec3f robot_pos = robot_state.col(0);
    coverage_grid_.update(clusters, viewpoints_by_cluster, robot_pos);
    if (coverage_grid_.activeNodes().empty()) {
        plan.reason = "no active coverage node";
        return false;
    }

    if (!buildCoverageTour(robot_state, current_yaw, plan)) {
        return false;
    }
    if (!refineLocalViewpoints(robot_state, current_yaw, viewpoints_by_cluster, plan)) {
        return false;
    }

    plan.success = true;
    return true;
}

const super_utils::vec_E<ExplorationCoverageNode> &GlobalCoveragePlanner::activeCoverageNodes() const {
    return coverage_grid_.activeNodes();
}

bool GlobalCoveragePlanner::buildCoverageTour(const super_utils::StatePVAJ &robot_state,
                                              const double current_yaw,
                                              ExplorationCoveragePlan &plan) {
    const auto &nodes = coverage_grid_.activeNodes();
    const int node_num =
            std::min(static_cast<int>(nodes.size()), std::max(1, cfg_.max_tour_nodes));
    const int dim = node_num + 1;
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(dim, dim);
    const super_utils::Vec3f robot_pos = robot_state.col(0);
    const super_utils::Vec3f robot_vel = robot_state.col(1);

    for (int i = 1; i < dim; ++i) {
        const auto &node = nodes[static_cast<std::size_t>(i - 1)];
        const double target_yaw = std::atan2(node.center.y() - robot_pos.y(),
                                             node.center.x() - robot_pos.x());
        cost_matrix(0, i) = nodeEdgeCost(robot_pos,
                                         node.center,
                                         current_yaw,
                                         target_yaw,
                                         robot_vel,
                                         true);
        cost_matrix(i, 0) = 0.0;
    }

    for (int i = 1; i < dim; ++i) {
        for (int j = 1; j < dim; ++j) {
            if (i == j) {
                cost_matrix(i, j) = 0.0;
                continue;
            }
            const auto &from = nodes[static_cast<std::size_t>(i - 1)];
            const auto &to = nodes[static_cast<std::size_t>(j - 1)];
            const double yaw = std::atan2(to.center.y() - from.center.y(),
                                          to.center.x() - from.center.x());
            cost_matrix(i, j) = nodeEdgeCost(from.center,
                                             to.center,
                                             yaw,
                                             yaw,
                                             super_utils::Vec3f::Zero(),
                                             false);
        }
    }

    std::vector<int> raw_tour;
    std::string reason;
    if (!atsp_solver_.solve(cost_matrix,
                            raw_tour,
                            plan.tour_cost,
                            plan.used_lkh,
                            reason)) {
        plan.reason = reason;
        return false;
    }

    plan.node_tour.clear();
    plan.coverage_path.clear();
    plan.coverage_path.push_back(robot_pos);
    for (const int idx : raw_tour) {
        if (idx <= 0 || idx >= dim) {
            continue;
        }
        plan.node_tour.push_back(idx - 1);
        plan.coverage_path.push_back(nodes[static_cast<std::size_t>(idx - 1)].center);
    }
    if (plan.node_tour.empty()) {
        plan.reason = "empty coverage tour";
        return false;
    }
    plan.cluster_tour = orderedClustersFromTour(plan);
    if (plan.cluster_tour.empty()) {
        plan.reason = "coverage tour has no assigned frontier cluster";
        return false;
    }
    plan.reason = reason;
    return true;
}

bool GlobalCoveragePlanner::refineLocalViewpoints(
        const super_utils::StatePVAJ &robot_state,
        const double current_yaw,
        const std::unordered_map<int, super_utils::vec_E<ExplorationViewpoint>> &viewpoints_by_cluster,
        ExplorationCoveragePlan &plan) {
    const super_utils::Vec3f robot_pos = robot_state.col(0);
    const super_utils::Vec3f robot_vel = robot_state.col(1);
    std::vector<int> refined_clusters;
    refined_clusters.reserve(static_cast<std::size_t>(std::max(1, cfg_.refined_num)));
    for (const int cluster_id : plan.cluster_tour) {
        const auto it = viewpoints_by_cluster.find(cluster_id);
        if (it == viewpoints_by_cluster.end() || it->second.empty()) {
            continue;
        }
        if (!refined_clusters.empty() &&
            (it->second.front().position - robot_pos).norm() > cfg_.refined_radius) {
            break;
        }
        refined_clusters.push_back(cluster_id);
        if (static_cast<int>(refined_clusters.size()) >= std::max(1, cfg_.refined_num)) {
            break;
        }
    }
    if (refined_clusters.empty()) {
        plan.reason = "no cluster left for local viewpoint refinement";
        return false;
    }

    std::vector<super_utils::vec_E<ExplorationViewpoint>> layers;
    layers.reserve(refined_clusters.size());
    for (const int cluster_id : refined_clusters) {
        const auto it = viewpoints_by_cluster.find(cluster_id);
        if (it != viewpoints_by_cluster.end() && !it->second.empty()) {
            layers.push_back(it->second);
        }
    }
    if (layers.empty()) {
        plan.reason = "local refinement layers empty";
        return false;
    }

    std::vector<std::vector<double>> dp(layers.size());
    std::vector<std::vector<int>> parent(layers.size());
    for (std::size_t layer = 0; layer < layers.size(); ++layer) {
        dp[layer].assign(layers[layer].size(), kInfCost);
        parent[layer].assign(layers[layer].size(), -1);
    }

    for (std::size_t j = 0; j < layers.front().size(); ++j) {
        const auto &view = layers.front()[j];
        const double travel = cost_evaluator_.computeCost(robot_pos,
                                                          view.position,
                                                          current_yaw,
                                                          view.yaw,
                                                          robot_vel,
                                                          false,
                                                          true);
        if (travel >= kInfCost) {
            continue;
        }
        dp[0][j] = travel - viewpointReward(view);
    }

    for (std::size_t layer = 1; layer < layers.size(); ++layer) {
        for (std::size_t j = 0; j < layers[layer].size(); ++j) {
            const auto &view = layers[layer][j];
            for (std::size_t i = 0; i < layers[layer - 1].size(); ++i) {
                if (dp[layer - 1][i] >= kInfCost) {
                    continue;
                }
                const auto &prev = layers[layer - 1][i];
                const double edge = cost_evaluator_.computeCost(prev.position,
                                                                view.position,
                                                                prev.yaw,
                                                                view.yaw,
                                                                super_utils::Vec3f::Zero(),
                                                                true,
                                                                false);
                const double score =
                        dp[layer - 1][i] + edge - viewpointReward(view);
                if (score < dp[layer][j]) {
                    dp[layer][j] = score;
                    parent[layer][j] = static_cast<int>(i);
                }
            }
        }
    }

    std::size_t best_layer = 0;
    int best_idx = -1;
    double best_score = kInfCost;
    for (std::size_t layer = 0; layer < layers.size(); ++layer) {
        for (std::size_t j = 0; j < layers[layer].size(); ++j) {
            double score = dp[layer][j];
            if (score >= kInfCost) {
                continue;
            }
            if (plan.coverage_path.size() > 1) {
                const super_utils::Vec3f anchor =
                        plan.coverage_path[std::min<std::size_t>(layer + 1,
                                                                  plan.coverage_path.size() - 1)];
                score += 0.1 * (layers[layer][j].position - anchor).norm();
            }
            if (score < best_score) {
                best_score = score;
                best_layer = layer;
                best_idx = static_cast<int>(j);
            }
        }
    }

    if (best_idx < 0) {
        plan.reason = "no reachable viewpoint in local refinement";
        return false;
    }

    plan.local_viewpoint_sequence.clear();
    int idx = best_idx;
    for (int layer = static_cast<int>(best_layer); layer >= 0; --layer) {
        plan.local_viewpoint_sequence.push_back(layers[static_cast<std::size_t>(layer)]
                                                      [static_cast<std::size_t>(idx)]);
        idx = parent[static_cast<std::size_t>(layer)][static_cast<std::size_t>(idx)];
        if (idx < 0 && layer > 0) {
            break;
        }
    }
    std::reverse(plan.local_viewpoint_sequence.begin(), plan.local_viewpoint_sequence.end());

    if (plan.local_viewpoint_sequence.empty()) {
        plan.reason = "local refinement produced empty sequence";
        return false;
    }

    super_utils::vec_E<super_utils::Vec3f> guide_path;
    const auto &first_view = plan.local_viewpoint_sequence.front();
    const double travel = cost_evaluator_.computeCost(robot_pos,
                                                      first_view.position,
                                                      current_yaw,
                                                      first_view.yaw,
                                                      robot_vel,
                                                      false,
                                                      true,
                                                      &guide_path);
    if (travel >= kInfCost) {
        plan.reason = "selected viewpoint not reachable";
        return false;
    }
    plan.guide_path = guide_path;
    return true;
}

std::vector<int> GlobalCoveragePlanner::orderedClustersFromTour(
        const ExplorationCoveragePlan &plan) const {
    const auto &nodes = coverage_grid_.activeNodes();
    std::vector<int> cluster_ids;
    std::unordered_set<int> inserted;
    for (const int node_id : plan.node_tour) {
        if (node_id < 0 || node_id >= static_cast<int>(nodes.size())) {
            continue;
        }
        const auto &node = nodes[static_cast<std::size_t>(node_id)];
        for (const int cluster_id : node.cluster_ids) {
            if (inserted.insert(cluster_id).second) {
                cluster_ids.push_back(cluster_id);
            }
        }
    }
    return cluster_ids;
}

double GlobalCoveragePlanner::nodeEdgeCost(const super_utils::Vec3f &from,
                                           const super_utils::Vec3f &to,
                                           const double from_yaw,
                                           const double to_yaw,
                                           const super_utils::Vec3f &from_vel,
                                           const bool from_robot) const {
    const bool allow_unknown = !from_robot;
    return cost_evaluator_.computeCost(from,
                                       to,
                                       from_yaw,
                                       to_yaw,
                                       from_vel,
                                       allow_unknown,
                                       false);
}

}  // namespace general_planner
