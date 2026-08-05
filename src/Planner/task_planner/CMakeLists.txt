cmake_minimum_required(VERSION 2.8.3)
project(task_planner)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DUSE_ROS1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        quadrotor_msgs
        visualization_msgs
        message_filters
        mavros_msgs
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
        )

add_executable(waypoint_mission
        Apps/ros1_waypoint_mission.cpp
        )
add_dependencies(waypoint_mission
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(waypoint_mission
        ${THIRD_PARTY}
        )

add_executable(task_planner
        Apps/ros1_task_planner.cpp
        )
add_dependencies(task_planner
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(task_planner
        ${THIRD_PARTY}
        )

add_executable(mission_task_config_self_test
        Tests/mission_task_config_self_test.cpp
        )
add_dependencies(mission_task_config_self_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(mission_task_config_self_test
        ${THIRD_PARTY}
        )

add_executable(swarm_task_source
        Apps/swarm_task_source.cpp
        )
add_dependencies(swarm_task_source
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(swarm_task_source
        ${THIRD_PARTY}
        )

add_executable(swarm_goal_relay
        Apps/swarm_goal_relay.cpp
        )
add_dependencies(swarm_goal_relay
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(swarm_goal_relay
        ${THIRD_PARTY}
        )

add_executable(formulation_task
        Apps/formulation_task.cpp
        )
add_dependencies(formulation_task
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(formulation_task
        ${THIRD_PARTY}
        )
