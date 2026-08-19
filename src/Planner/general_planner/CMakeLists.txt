cmake_minimum_required(VERSION 3.5)
project(general_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=unused-but-set-variable)
set(CMAKE_CXX_STANDARD 17)

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

message("Using ROS1")
# add a marco for ros1
add_definitions(-DUSE_ROS1)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        sensor_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        cv_bridge
        map_manager
        lkh_tsp_solver
        traj_utils
        yaml-cpp
        tf
)

add_message_files(
        FILES
        FrontierCluster.msg
        FrontierClusterArray.msg
        ExplorationTaskRequest.msg
        PlannerModeRequest.msg
        PlannerStatus.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex iostreams)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES general_planner_core general_planner_exploration_utils general_planner_exploration
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
                       visualization_msgs message_filters cv_bridge map_manager
                       lkh_tsp_solver traj_utils tf message_runtime
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${CMAKE_CURRENT_BINARY_DIR}/generated
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        #        general_planner
        yaml-cpp
        -ldw
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
list(FILTER srcs EXCLUDE REGEX ".*/(src|include)/general_core/exploration/(highspeed|exploration_utils)/.*$")
list(FILTER srcs EXCLUDE REGEX ".*/src/general_core/planner_runtime/.*$")
list(FILTER srcs EXCLUDE REGEX ".*/src/traj_opt/exp_traj_optimizer_s4\\.cpp$")
list(FILTER srcs EXCLUDE REGEX ".*/src/traj_opt/backup_traj_optimizer_s4\\.cpp$")
list(FILTER srcs EXCLUDE REGEX ".*/src/traj_opt/yaw_traj_opt\\.cpp$")
list(FILTER srcs EXCLUDE REGEX ".*/src/utils/minco\\.cpp$")
list(REMOVE_DUPLICATES srcs)
add_library(general_planner_core STATIC ${srcs})
target_include_directories(general_planner_core PUBLIC
        include)
add_dependencies(general_planner_core
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(general_planner_core PUBLIC
)

set(GENERAL_PLANNER_EXPLORATION_DIR
        src/general_core/exploration/highspeed)
set(GENERAL_PLANNER_EXPLORATION_UTILS_DIR
        src/general_core/exploration/exploration_utils)

add_library(general_planner_exploration_utils STATIC
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/lidar_map/ikd_Tree.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/lidar_map/lidar_map_interface.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/lidar_map/sim_lio.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/pointcloud_topo/parallel_bubble_astar.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/pointcloud_topo/topo_skeleton_graph.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/pointcloud_topo/historical_odom_graph.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/pointcloud_topo/bubble_split.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/path_searching/bubble_astar.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/frontier_manager/frontier_manager.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/frontier_manager/global_planning.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/frontier_manager/raycast.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/frontier_manager/frontier_visualizer.cpp
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/coverage_guidance/coverage_guidance_manager.cpp
)
add_dependencies(general_planner_exploration_utils
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_include_directories(general_planner_exploration_utils PUBLIC include)
target_compile_options(general_planner_exploration_utils PRIVATE
        -Wno-error=unused-variable
        -Wno-error=unused-but-set-variable
)
target_link_libraries(general_planner_exploration_utils PUBLIC
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        OpenMP::OpenMP_CXX
)

add_library(general_planner_exploration STATIC
        ${GENERAL_PLANNER_EXPLORATION_DIR}/fast_exploration_manager.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/general_planner_adapter.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/swarm_exploration_coordinator.cpp
)
add_dependencies(general_planner_exploration
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_include_directories(general_planner_exploration PUBLIC include)
target_compile_options(general_planner_exploration PRIVATE
        -Wno-error=unused-variable
        -Wno-error=unused-but-set-variable
)
target_link_libraries(general_planner_exploration PUBLIC
        general_planner_core
        general_planner_exploration_utils
        ${THIRD_PARTY}
        OpenMP::OpenMP_CXX
)

add_executable(exploration_node
        ${GENERAL_PLANNER_EXPLORATION_DIR}/exploration_node.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/dynamic_bounding_box_selector.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/fast_exploration_fsm.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/fsm_utils.cpp
)
add_dependencies(exploration_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_compile_options(exploration_node PRIVATE
        -Wno-error=unused-variable
        -Wno-error=unused-but-set-variable
)
target_link_libraries(exploration_node
        general_planner_exploration
        ${THIRD_PARTY}
        OpenMP::OpenMP_CXX
)

add_executable(highspeed_traj_server
        ${GENERAL_PLANNER_EXPLORATION_DIR}/traj_server.cpp
)
add_dependencies(highspeed_traj_server
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_compile_options(highspeed_traj_server PRIVATE
        -Wno-error=unused-variable
        -Wno-error=unused-but-set-variable
)
target_link_libraries(highspeed_traj_server
        general_planner_core
        ${THIRD_PARTY}
)

add_executable(mission_command_mux
        ${GENERAL_PLANNER_EXPLORATION_DIR}/mission_command_mux.cpp
)
add_dependencies(mission_command_mux
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mission_command_mux
        ${THIRD_PARTY}
)

add_executable(planner_runtime_node
        Apps/planner_runtime_node_ros1.cpp
        src/general_core/planner_runtime/global_map_runtime.cpp
        src/general_core/planner_runtime/planner_command_gateway.cpp
        src/general_core/planner_runtime/planner_supervisor.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/dynamic_bounding_box_selector.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/fast_exploration_fsm.cpp
        ${GENERAL_PLANNER_EXPLORATION_DIR}/fsm_utils.cpp
)
add_dependencies(planner_runtime_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_include_directories(planner_runtime_node PRIVATE include)
target_link_libraries(planner_runtime_node
        general_planner_core
        general_planner_exploration
        ${THIRD_PARTY}
        OpenMP::OpenMP_CXX
)

add_executable(planner_status_self_test
        tests/planner_status_self_test.cpp
)
target_include_directories(planner_status_self_test PRIVATE include)

add_executable(planner_command_gateway_policy_self_test
        tests/planner_command_gateway_policy_self_test.cpp
)
target_include_directories(planner_command_gateway_policy_self_test PRIVATE include)

add_executable(planner_command_gateway_behavior_self_test
        tests/planner_command_gateway_behavior_self_test.cpp
        src/general_core/planner_runtime/planner_command_gateway.cpp
)
add_dependencies(planner_command_gateway_behavior_self_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_include_directories(planner_command_gateway_behavior_self_test PRIVATE include)
target_link_libraries(planner_command_gateway_behavior_self_test
        ${catkin_LIBRARIES}
)

add_executable(target_directed_exploration_self_test
        tests/target_directed_exploration_self_test.cpp
)
target_include_directories(target_directed_exploration_self_test PRIVATE include)

add_executable(state2state_topology_route_self_test
        tests/state2state_topology_route_self_test.cpp
)
target_include_directories(state2state_topology_route_self_test PRIVATE include)

add_executable(coverage_guidance_self_test
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/coverage_guidance/coverage_guidance_self_test.cpp
)
add_dependencies(coverage_guidance_self_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(coverage_guidance_self_test
        general_planner_exploration_utils
        ${THIRD_PARTY}
)

add_executable(coverage_recovery_identity_self_test
        tests/coverage_recovery_identity_self_test.cpp
)
target_include_directories(coverage_recovery_identity_self_test PRIVATE include)

add_executable(yaw_candidate_selector_self_test
        ${GENERAL_PLANNER_EXPLORATION_UTILS_DIR}/frontier_manager/yaw_candidate_selector_self_test.cpp
)
target_include_directories(yaw_candidate_selector_self_test PRIVATE include)

add_executable(goal_height_policy_self_test
        tests/goal_height_policy_self_test.cpp
)
target_include_directories(goal_height_policy_self_test PRIVATE include)

add_executable(convex_hull_self_test
        Tests/convex_hull_self_test.cpp
)
target_include_directories(convex_hull_self_test PRIVATE include)
target_link_libraries(convex_hull_self_test
        general_planner_core
)

add_executable(bezier_certificate_self_test
        Tests/bezier_certificate_self_test.cpp
)
target_include_directories(bezier_certificate_self_test PRIVATE include)
target_link_libraries(bezier_certificate_self_test
        general_planner_core
)

add_executable(phase2_corrector_self_test
        Tests/phase2_corrector_self_test.cpp
)
target_include_directories(phase2_corrector_self_test PRIVATE include)
target_link_libraries(phase2_corrector_self_test
        general_planner_core
)

add_executable(flatness_convex_hull_self_test
        Tests/flatness_convex_hull_self_test.cpp
)
target_include_directories(flatness_convex_hull_self_test PRIVATE include)
target_link_libraries(flatness_convex_hull_self_test
        general_planner_core
)

add_executable(fast_lbfgs_self_test
        Tests/fast_lbfgs_self_test.cpp
)
target_include_directories(fast_lbfgs_self_test PRIVATE include)
target_link_libraries(fast_lbfgs_self_test
        general_planner_core
)

add_executable(guide_z_lower_bound_self_test
        Tests/guide_z_lower_bound_self_test.cpp
)
target_include_directories(guide_z_lower_bound_self_test PRIVATE include)

add_executable(convex_hull_convergence_benchmark
        Tests/convex_hull_convergence_benchmark.cpp
)
target_include_directories(convex_hull_convergence_benchmark PRIVATE include)
target_link_libraries(convex_hull_convergence_benchmark
        general_planner_core
)

add_executable(fsm_node
        Apps/fsm_node_ros1.cpp
)
add_dependencies(fsm_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fsm_node
        general_planner_core        ${THIRD_PARTY}
)

set(GENERAL_PLANNER_RUNTIME_PRESET_HEADER
        "${CMAKE_CURRENT_BINARY_DIR}/generated/general_planner_runtime_preset.hpp")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_CONFIG
        "${CMAKE_CURRENT_SOURCE_DIR}/config/tracking_tracker_drone1_ros1.yaml")
set(GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_CONFIG
        "${CMAKE_CURRENT_SOURCE_DIR}/config/click_smooth_ros1.yaml")
set(GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_CONFIG
        "${CMAKE_CURRENT_SOURCE_DIR}/config/click_esdf_ros1.yaml")
set(GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_CONFIG
        "${CMAKE_CURRENT_SOURCE_DIR}/config/click_plain_ros1.yaml")

file(READ "${GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_CONFIG}"
        GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_YAML)
file(READ "${GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_CONFIG}"
        GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_YAML)
file(READ "${GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_CONFIG}"
        GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_YAML)
file(READ "${GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_CONFIG}"
        GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_YAML)

get_filename_component(GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_NAME
        "${GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_CONFIG}" NAME)
get_filename_component(GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_NAME
        "${GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_CONFIG}" NAME)
get_filename_component(GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_NAME
        "${GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_CONFIG}" NAME)
get_filename_component(GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_NAME
        "${GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_CONFIG}" NAME)

file(WRITE "${GENERAL_PLANNER_RUNTIME_PRESET_HEADER}"
        "#pragma once\n"
        "namespace general_planner_runtime_preset {\n"
        "static constexpr const char *kTrackingPresetName = \"${GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_NAME}\";\n"
        "static constexpr const char *kTrackingPresetYaml = R\"GPTRACK(\n"
        "${GENERAL_PLANNER_RUNTIME_TRACKING_PRESET_YAML}\n"
        ")GPTRACK\";\n"
        "static constexpr const char *kState2StateCorridorPresetName = \"${GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_NAME}\";\n"
        "static constexpr const char *kState2StateCorridorPresetYaml = R\"GPCORRID(\n"
        "${GENERAL_PLANNER_RUNTIME_STATE2STATE_CORRIDOR_PRESET_YAML}\n"
        ")GPCORRID\";\n"
        "static constexpr const char *kState2StateEsdfPresetName = \"${GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_NAME}\";\n"
        "static constexpr const char *kState2StateEsdfPresetYaml = R\"GPESDF(\n"
        "${GENERAL_PLANNER_RUNTIME_STATE2STATE_ESDF_PRESET_YAML}\n"
        ")GPESDF\";\n"
        "static constexpr const char *kState2StatePlainPresetName = \"${GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_NAME}\";\n"
        "static constexpr const char *kState2StatePlainPresetYaml = R\"GPPLAIN(\n"
        "${GENERAL_PLANNER_RUNTIME_STATE2STATE_PLAIN_PRESET_YAML}\n"
        ")GPPLAIN\";\n"
        "}\n")

add_executable(general_planner_runtime_node
        Apps/general_planner_runtime_node_ros1.cpp
        "${GENERAL_PLANNER_RUNTIME_PRESET_HEADER}"
)
add_dependencies(general_planner_runtime_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(general_planner_runtime_node
        general_planner_core        ${THIRD_PARTY}
)

add_executable(read_replan_log
        Apps/read_replan_log.cpp
)
add_dependencies(read_replan_log
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(read_replan_log
        general_planner_core        -lncurses
        ${THIRD_PARTY}
)

add_executable(traj_opt_tuning
        Apps/traj_opt_tuning.cpp
)
add_dependencies(traj_opt_tuning
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(traj_opt_tuning
        general_planner_core        -lncurses
        ${THIRD_PARTY}
)

add_executable(tracking_perching_demo
        Apps/tracking_perching_demo.cpp
)
add_dependencies(tracking_perching_demo
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tracking_perching_demo
        general_planner_core        ${THIRD_PARTY}
)

add_executable(task_target_source
        Apps/task_target_source.cpp
)
add_dependencies(task_target_source
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(task_target_source
        general_planner_core        ${THIRD_PARTY}
)

add_executable(se3_gcopter_gate_planning
        Apps/se3_gcopter_gate_planning.cpp
)
add_dependencies(se3_gcopter_gate_planning
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(se3_gcopter_gate_planning
        general_planner_core        ${THIRD_PARTY}
)

add_executable(tracking_target_relay
        Apps/tracking_target_relay.cpp
)
add_dependencies(tracking_target_relay
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tracking_target_relay
        general_planner_core        ${THIRD_PARTY}
)

add_executable(tracking_sim_monitor
        Apps/tracking_sim_monitor.cpp
)
add_dependencies(tracking_sim_monitor
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tracking_sim_monitor
        general_planner_core        ${THIRD_PARTY}
)

catkin_install_python(PROGRAMS
        scripts/click_state2state_benchmark.py
        scripts/corridor_benchmark.py
        scripts/se3_so3_command_monitor.py
        scripts/planner_serial_handover.py
        scripts/planner_rviz_switcher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
