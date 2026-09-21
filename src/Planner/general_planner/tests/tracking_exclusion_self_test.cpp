#include <general_core/tracking/tracking_map_query.hpp>
#include <general_core/tracking/tracking_prediction.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
}

int main() {
    try {
        auto &mask = general_planner::trackingOccupancyExclusion();
        mask.clear();
        require(!mask.valid() && !mask.contains(general_utils::Vec3f(0, 0, 0)),
                "cleared mask must ignore every point");

        general_planner::setTrackingOccupancyExclusion(general_utils::Vec3f(1.0, 2.0, 0.5), 0.8, 0.8);
        require(mask.valid(), "cylinder was not installed");
        require(mask.contains(general_utils::Vec3f(1.0, 2.0, 0.5)), "center must be excluded");
        require(mask.contains(general_utils::Vec3f(1.5, 2.0, 0.5)), "body xy must be excluded");
        require(!mask.contains(general_utils::Vec3f(2.0, 2.0, 0.5)), "outside xy must stay occupied");
        require(!mask.contains(general_utils::Vec3f(1.0, 2.0, 1.5)), "outside z must stay occupied");
        require(!general_planner::trackingInflatedOccupied(nullptr, general_utils::Vec3f(1.0, 2.0, 0.5)),
                "null map plus exclusion must treat the body as free");

        general_planner::TrackingPredictionSettings cfg;
        cfg.horizon = 1.0;
        cfg.dt = 0.2;
        cfg.accel = 3.0;
        cfg.vmax = 4.0;
        cfg.max_time = 0.08;
        traj_opt::DynamicTargetStates prediction;
        general_planner::buildConstantVelocityTrackingPrediction(
            general_utils::Vec3f(0, 0, 1), general_utils::Vec3f(1, 0, 0), 0.0, cfg, prediction);
        require(prediction.size() >= 2 && std::abs(prediction.back().t - 1.0) < 1.0e-9,
                "constant-velocity horizon mismatch");
        require(std::abs(prediction.back().position.x() - 1.0) < 1.0e-9,
                "constant-velocity position mismatch");

        prediction.clear();
        const bool ok = general_planner::buildKinodynamicTrackingPrediction(
            general_utils::Vec3f(0, 0, 1), general_utils::Vec3f(1, 0, 0), 0.0, cfg,
            [](const general_utils::Vec3f &, const general_utils::Vec3f &) { return true; },
            prediction);
        require(ok && prediction.size() >= 2, "unconstrained kinodynamic search failed");
        require(std::abs(prediction.front().t) < 1.0e-9, "prediction must start at t=0");

        prediction.clear();
        require(!general_planner::buildKinodynamicTrackingPrediction(
                    general_utils::Vec3f(0, 0, 1), general_utils::Vec3f(1, 0, 0), 0.0, cfg,
                    [](const general_utils::Vec3f &, const general_utils::Vec3f &) { return false; },
                    prediction),
                "fully blocked search must fail closed");

        mask.clear();
        require(!mask.valid(), "mask must be reversible for other modes");
        std::cout << "tracking_exclusion_self_test: PASS\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "tracking_exclusion_self_test FAIL: " << e.what() << '\n';
        return 1;
    }
}
