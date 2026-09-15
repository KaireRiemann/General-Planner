#include <fsm/config.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace {
void check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
bool near(double a, double b) { return std::abs(a - b) < 1.e-9; }
void save(const std::filesystem::path &path, const YAML::Node &node) {
    std::ofstream file(path);
    file << node;
    if (!file) throw std::runtime_error("Cannot write test fixture");
}
template <typename Fn> void mustThrow(Fn fn, const char *message) {
    bool threw = false;
    try { fn(); } catch (const std::exception &) { threw = true; }
    check(threw, message);
}
}  // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: tracking_config_self_test NAVIGATION_YAML TRACKING_YAML\n";
        return 2;
    }
    const auto dir = std::filesystem::temp_directory_path() /
        ("gp_tracking_config_test_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir / "profiles");
    try {
        YAML::Node nav = YAML::LoadFile(argv[1]);
        YAML::Node tracking = YAML::LoadFile(argv[2]);
        // Test the real loaders with deliberately conflicting navigation data.
        nav["tracking_config"] = "profiles/tracking.yaml";
        nav["traj_opt"]["exp_traj"]["convex_hull_en"] = false;
        nav["fsm"]["task_mode"] = "state2state";
        nav["fsm"]["replan_rate"] = 3.0;
        nav["fsm"]["task_timeout"] = 9.0;
        nav["fsm"]["tracking_prediction_horizon"] = 99.0;
        nav["general_planner"]["tracking"]["max_vel"] = 99.0;
        nav["general_planner"]["yaw_dot_max"] = 0.2;
        nav["general_planner"]["planning_horizon"] = 77.0;
        nav["traj_opt"]["boundary"]["max_vel"] = 7.0;
        nav["traj_opt"]["esdf_traj"]["penna_t"] = 777.0;
        tracking["fsm"]["replan_rate"] = 7.0;
        tracking["fsm"]["task_timeout"] = 1.3;
        tracking["fsm"]["tracking_prediction_horizon"] = 2.0;
        tracking["general_planner"]["tracking"]["max_vel"] = 5.0;
        tracking["general_planner"]["tracking"]["yaw_rate_limit"] = 1.2;
        tracking["general_planner"]["tracking"]["planning_horizon"] = 8.0;
        tracking["general_planner"]["tracking"]["use_snap"] = true;
        tracking["traj_opt"]["tracking_traj"]["penna_t"] = 123.0;
        const auto nav_path = dir / "navigation.yaml";
        const auto tracking_path = dir / "profiles/tracking.yaml";
        save(nav_path, nav);
        save(tracking_path, tracking);

        general_planner::Config planner(nav_path.string());
        fsm::Config state(nav_path.string());
        check(near(planner.esdf_traj_cfg.max_vel, 7.0), "navigation limit changed");
        check(near(planner.esdf_traj_cfg.penna_t, 777.0), "navigation weight changed");
        check(near(planner.tracking_traj_cfg.max_vel, 5.0), "tracking inherited navigation limit");
        check(near(planner.tracking_traj_cfg.penna_t, 123.0), "tracking inherited navigation weight");
        check(near(planner.tracking_yaw_rate_limit, 1.2), "tracking yaw clamped by navigation");
        check(near(planner.tracking_planning_horizon, 8.0), "tracking frontend inherited navigation horizon");
        check(near(state.tracking_prediction_horizon, 2.0), "tracking prediction inherited navigation");
        check(near(state.tracking_task_timeout, 1.3), "tracking timeout inherited navigation");
        check(near(state.task_timeout, 9.0), "non-tracking timeout changed");
        check(state.tracking_use_snap, "tracking backend selection ignored profile");
        check(near(state.replanRate(), 3.0), "state2state frequency changed");
        state.task_mode = fsm::TaskMode::TRACKING;
        check(near(state.replanRate(), 7.0), "tracking frequency not selected");
        state.task_mode = fsm::TaskMode::TRACKING_PERCHING;
        check(near(state.replanRate(), 7.0), "tracking-perching frequency not selected");
        state.task_mode = fsm::TaskMode::STATE_TO_STATE;
        check(near(state.replanRate(), 3.0), "navigation frequency not restored");

        // Editing only the separate file must change tracking on the next load.
        tracking["general_planner"]["tracking"]["max_vel"] = 4.0;
        tracking["traj_opt"]["tracking_traj"]["penna_t"] = 17.0;
        tracking["fsm"]["replan_rate"] = 11.0;
        save(tracking_path, tracking);
        general_planner::Config edited(nav_path.string());
        fsm::Config edited_state(nav_path.string());
        check(near(edited.tracking_traj_cfg.max_vel, 4.0), "profile edit was not loaded");
        check(near(edited.tracking_traj_cfg.penna_t, 17.0), "profile weight edit was not loaded");
        check(near(edited.esdf_traj_cfg.max_vel, 7.0), "tracking edit leaked into navigation");
        check(near(edited_state.tracking_replan_rate, 11.0), "profile rate edit was not loaded");

        // Explicit startup override wins over a missing reference in navigation.
        nav["tracking_config"] = "missing.yaml";
        save(nav_path, nav);
        general_planner::Config overridden(nav_path.string(), tracking_path.string());
        check(near(overridden.tracking_traj_cfg.max_vel, 4.0), "explicit override was ignored");
        mustThrow([&] { general_planner::Config bad(nav_path.string()); },
                  "missing explicit profile silently fell back to navigation");

        // Legacy full profiles remain self-contained when no reference exists.
        nav.remove("tracking_config");
        save(nav_path, nav);
        general_planner::Config legacy(nav_path.string());
        fsm::Config legacy_state(nav_path.string());
        check(legacy.tracking_config_path.empty(), "legacy profile unexpectedly externalized");
        check(near(legacy.tracking_traj_cfg.max_vel, 99.0), "legacy tracking override changed");
        check(near(legacy.tracking_traj_cfg.penna_t, 777.0), "legacy optimizer fallback changed");
        check(near(legacy_state.tracking_replan_rate, 3.0), "legacy replan frequency changed");
        check(near(legacy_state.tracking_task_timeout, 9.0), "legacy timeout changed");

        tracking["fsm"]["replan_rate"] = 0.0;
        save(tracking_path, tracking);
        mustThrow([&] { fsm::Config bad(nav_path.string(), tracking_path.string()); },
                  "zero tracking frequency accepted");
        tracking["traj_opt"].remove("tracking_traj");
        save(tracking_path, tracking);
        mustThrow([&] { general_planner::Config bad(nav_path.string(), tracking_path.string()); },
                  "malformed tracking profile accepted");
        std::filesystem::remove_all(dir);
        std::cout << "tracking_config_self_test PASS\n";
        return 0;
    } catch (const std::exception &error) {
        std::filesystem::remove_all(dir);
        std::cerr << "tracking_config_self_test FAIL: " << error.what() << '\n';
        return 1;
    }
}
