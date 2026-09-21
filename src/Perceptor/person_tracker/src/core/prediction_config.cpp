#include "core/mapping_manager.h"

#include <cmath>
#include <stdexcept>

namespace person_tracker
{
void MappingRos::configurePredictionModel()
{
  const std::string backend = readParameter<std::string>("prediction/backend", "analytic");
  MmkNetConfig config;
  TargetEkf::defaultUpstreamMode() = (backend=="imm_mot" || backend=="imm_mot_ctra" || backend=="imm_mot_r" || backend=="imm_mot_rt" || backend=="imm_mot_fixed" || backend=="imm_mot_fixed_q") ? backend : "";
  if (!TargetEkf::defaultUpstreamMode().empty()) {
    ImmMotFilter::configure(readParameter<std::string>("prediction/imm_mot/adapter", ""),
      readParameter<std::string>("prediction/imm_mot/repository", ""),
      readParameter<std::string>("prediction/imm_mot/dependencies", "/opt/imm_mot_deps"),
      readParameter<double>("prediction/imm_mot/position_std",params_.measurement_noise),
      readParameter<double>("prediction/imm_mot/acceleration_psd",6.25),
      readParameter<double>("prediction/imm_mot/jerk_psd",16.0),
      readParameter<double>("prediction/imm_mot/turn_acceleration_psd",0.36),
      readParameter<double>("prediction/imm_mot/vertical_position_psd",0.01));
  }
  TargetEkf::defaultImmEnabled() = backend == "imm";
  auto & imm = TargetEkf::defaultImmConfig();
  imm.jerk_noise = readParameter<double>("prediction/imm/jerk_noise",4.0);
  imm.turn_noise = readParameter<double>("prediction/imm/turn_noise",0.6);
  imm.stay_probability = readParameter<double>("prediction/imm/stay_probability",0.94);
  if (backend == "imm") {
    ImmFilter validation;
    validation.initialize(Eigen::Vector2d::Zero(),params_.acceleration_noise,params_.measurement_noise,imm);
  }
  if (backend == "mmknet") {
    const std::string path = readParameter<std::string>("prediction/mmknet/weights", "");
    config.sample_period = readParameter<double>("prediction/mmknet/sample_period", 0.1);
    config.period_tolerance = readParameter<double>("prediction/mmknet/period_tolerance", 0.35);
    config.maximum_turn_rate = readParameter<double>("prediction/mmknet/maximum_turn_rate", 1.0);
    if (!std::isfinite(config.sample_period) || config.sample_period <= 0.0 ||
        !std::isfinite(config.period_tolerance) || config.period_tolerance < 0.0 ||
        config.period_tolerance >= 1.0 || !std::isfinite(config.maximum_turn_rate) ||
        config.maximum_turn_rate <= 0.0 || config.maximum_turn_rate > 1.0) {
      throw std::runtime_error("Invalid prediction/mmknet parameters");
    }
    config.network = std::make_shared<MmkNet>(path);
    ROS_WARN("MMKNet experimental backend: 8-frame XY features, expected dt=%.3f s. "
      "Bundled simulation weights used dt=1 s and are not calibrated for pedestrian tracking. "
      "Warm-up and observation gaps use CV; ground height remains unchanged.", config.sample_period);
  } else if (backend != "analytic" && backend != "imm" && backend != "imm_mot" && backend != "imm_mot_ctra" && backend != "imm_mot_r" && backend != "imm_mot_rt" && backend != "imm_mot_fixed" && backend != "imm_mot_fixed_q") {
    throw std::runtime_error("prediction/backend must be analytic, mmknet, imm, imm_mot_ctra or imm_mot");
  }
  TargetEkf::defaultMmkNetConfig() = config;
  ROS_INFO("Target prediction backend: %s", backend.c_str());
}
}  // namespace person_tracker
