cmake_minimum_required(VERSION 3.0.2)
project(aperture_detector)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy geometry_msgs nav_msgs quadrotor_msgs sensor_msgs std_msgs pcl_ros pcl_conversions visualization_msgs cv_bridge rosbag tf tf2 tf2_ros tf2_geometry_msgs message_generation)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
add_message_files(FILES ApertureObservation.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES detection_planning_bridge_lib
  CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs quadrotor_msgs sensor_msgs std_msgs pcl_ros pcl_conversions visualization_msgs cv_bridge rosbag tf tf2 tf2_ros tf2_geometry_msgs message_runtime)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(polygon_hole_step_viz src/polygon_hole_step_viz.cpp)
add_dependencies(polygon_hole_step_viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(polygon_hole_step_viz ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_library(detection_planning_bridge_lib src/detection_planning_bridge.cpp)
add_dependencies(detection_planning_bridge_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(detection_planning_bridge_lib ${catkin_LIBRARIES})
add_executable(detection_planning_bridge src/detection_planning_bridge_node.cpp)
target_link_libraries(detection_planning_bridge detection_planning_bridge_lib ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS scripts/windowtec_vlm_box_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS polygon_hole_step_viz detection_planning_bridge detection_planning_bridge_lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY launch config rviz_cfg scripts msg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY tests DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
