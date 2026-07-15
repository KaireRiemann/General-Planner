#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_guidance_manager.h>

#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "coverage_guidance_self_test",
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler);
  std::string error;
  if (!fast_planner::CoverageGuidanceManager::runDeterministicSelfTest(
          &error)) {
    std::cerr << "coverage guidance self-test failed: " << error << std::endl;
    return 1;
  }
  std::cout << "coverage guidance self-test passed" << std::endl;
  return 0;
}
