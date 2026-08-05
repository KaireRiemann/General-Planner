#include <iostream>
#include <string>

#include "task_planner/config.hpp"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: mission_task_config_self_test <config.yaml>" << std::endl;
    return 2;
  }

  const task_planner::TaskPlannerConfig config(argv[1]);
  if (config.task_mode_topic != "/mission/task_mode" ||
      config.navigation_task_mode_topic != "/planning/navigation_task_mode" ||
      config.exploration_command_topic != "/planning/exploration/command" ||
      config.exploration_status_topic != "/planning/exploration/status") {
    std::cerr << "mission topics were not parsed correctly" << std::endl;
    return 1;
  }
  if (config.tasks.size() != 5 ||
      config.tasks[0].mode != task_planner::ManagedTaskMode::STATE_TO_STATE ||
      config.tasks[1].mode != task_planner::ManagedTaskMode::EXPLORATION ||
      config.tasks[2].mode != task_planner::ManagedTaskMode::WAIT ||
      config.tasks[3].mode != task_planner::ManagedTaskMode::STATE_TO_STATE ||
      config.tasks[4].mode != task_planner::ManagedTaskMode::EXPLORATION) {
    std::cerr << "mission task order or mode parsing failed" << std::endl;
    return 1;
  }
  if (config.tasks[1].region_id != "region_a" ||
      config.tasks[4].region_id != "region_b" ||
      config.tasks[2].hold_duration != 2.0) {
    std::cerr << "exploration region or wait duration parsing failed" << std::endl;
    return 1;
  }

  std::cout << "mission_task_config_self_test passed" << std::endl;
  return 0;
}
