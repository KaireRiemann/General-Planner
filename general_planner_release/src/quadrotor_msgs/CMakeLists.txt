cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
)

file(GLOB QUADROTOR_MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "msg/*.msg")
add_message_files(
  DIRECTORY msg
  FILES ${QUADROTOR_MSG_FILES}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
)

