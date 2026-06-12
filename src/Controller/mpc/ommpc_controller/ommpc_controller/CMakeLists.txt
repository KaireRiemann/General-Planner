cmake_minimum_required(VERSION 3.0.2)
project(ommpc_controller)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  traj_utils
)

find_package(Eigen3 REQUIRED)
find_path(OSQP_INCLUDE_DIR osqp/osqp.h
  PATHS
    /opt/ros/noetic/include
    /usr/local/include
    /usr/include
)
find_library(OSQP_LIBRARY osqp
  PATHS
    /opt/ros/noetic/lib
    /usr/local/lib
    /usr/lib
)

if(NOT OSQP_INCLUDE_DIR OR NOT OSQP_LIBRARY)
  message(FATAL_ERROR "OSQP library/header not found. Install OSQP or set OSQP_INCLUDE_DIR and OSQP_LIBRARY.")
endif()

generate_dynamic_reconfigure_options(
  cfg/state.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    mavros_msgs
    nav_msgs
    quadrotor_msgs
    roscpp
    sensor_msgs
    std_msgs
    traj_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OSQP_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
  src/ommpc_example.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OSQP_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
