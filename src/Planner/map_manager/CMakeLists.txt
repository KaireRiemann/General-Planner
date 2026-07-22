cmake_minimum_required(VERSION 3.5)
project(map_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

# Keep the established ROG voxel convention and public configuration paths.
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DUSE_ROS1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        message_filters
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES map_manager
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(MAP_MANAGER_THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
)

# ROGMap is the local dense sliding backend. BoundaryMap and MapManager are
# built into this same library so map ownership has one package boundary.
file(GLOB_RECURSE MAP_MANAGER_SOURCES CONFIGURE_DEPENDS
        src/*.cpp
        include/rog_map/*.cpp
)
add_library(map_manager STATIC ${MAP_MANAGER_SOURCES})
target_include_directories(map_manager PUBLIC include)
target_link_libraries(map_manager PUBLIC ${MAP_MANAGER_THIRD_PARTY})
add_dependencies(map_manager ${catkin_EXPORTED_TARGETS})

add_executable(boundary_map_self_test tests/boundary_map_self_test.cpp)
target_include_directories(boundary_map_self_test PRIVATE include)
target_link_libraries(boundary_map_self_test map_manager)
add_dependencies(boundary_map_self_test ${catkin_EXPORTED_TARGETS})

add_executable(incremental_topology_self_test tests/incremental_topology_self_test.cpp)
target_include_directories(incremental_topology_self_test PRIVATE include)
target_link_libraries(incremental_topology_self_test map_manager)
add_dependencies(incremental_topology_self_test ${catkin_EXPORTED_TARGETS})
