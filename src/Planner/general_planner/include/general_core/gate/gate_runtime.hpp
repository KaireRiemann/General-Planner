#pragma once
#include <general_core/gate/gate_frontend.hpp>
#include <general_core/planner_runtime/planner_command_gateway.hpp>
#include <map_manager/map_manager.hpp>
#include <memory>

namespace general_planner::gate {
enum class Phase { IDLE, OBSERVING, PLANNING, READY, EXECUTING, SETTLING, SUCCEEDED, FAILED };
struct Status { Phase phase{Phase::IDLE}; std::uint64_t epoch{0}; std::string reason; };
// Environment callbacks also allow deterministic tests without a second world map.
struct Environment { std::function<bool()> ready; BodyClear body_clear; };
class Runtime {
public:
  Runtime(ros::NodeHandle nh, planner_runtime::PlannerCommandGateway &gateway,
          MapManager::Ptr map, Environment environment = {});
  ~Runtime();
  void start(std::uint64_t epoch);
  void cancel();
  bool execute(std::uint64_t epoch);
  Status status() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace general_planner::gate
