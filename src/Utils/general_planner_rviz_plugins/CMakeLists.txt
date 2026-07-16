cmake_minimum_required(VERSION 3.0.2)
project(general_planner_rviz_plugins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pluginlib
  roscpp
  rviz
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/goal_3d_tool.cpp
  src/goal_3d_tool.hpp
  src/pose_3d_tool.cpp
  src/pose_3d_tool.hpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  LICENSE
  README.md
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
