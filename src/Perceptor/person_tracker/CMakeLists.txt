cmake_minimum_required(VERSION 3.0.2)
project(person_tracker)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs std_msgs std_srvs tracking_detector pcl_conversions message_generation)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)
find_package(OpenMP QUIET)
add_message_files(FILES
  TrackedObject.msg
  TrackedObjectArray.msg
  TargetStatus.msg
  FusionStatus.msg
  GroundEstimate.msg
  PredictionStatus.msg
  ObservationDebug.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs std_msgs std_srvs tracking_detector pcl_conversions message_runtime)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(person_tracker_projection STATIC src/yolo/yolo_projection.cpp)
target_link_libraries(person_tracker_projection ${PCL_LIBRARIES})
add_library(person_tracker_ground STATIC src/lidar/ground_height_estimator.cpp)
target_link_libraries(person_tracker_ground ${PCL_LIBRARIES})
add_executable(person_tracker_node
  src/core/mapping_node.cpp
  src/core/mapping_manager.cpp
  src/core/parameters.cpp
  src/core/prediction_config.cpp
  src/tracking/mmknet.cpp
  src/tracking/imm_filter.cpp
  src/tracking/imm_mot_filter.cpp
  src/lidar/cloud_preprocessing.cpp
  src/lidar/voxel_map.cpp
  src/lidar/background_model.cpp
  src/tracking/ground_tracking.cpp
  src/lidar/cluster_detection.cpp
  src/tracking/track_management.cpp
  src/tracking/crossing_geometry.cpp
  src/tracking/support_memory.cpp
  src/tracking/auxiliary_tracking.cpp
  src/tracking/identity_recovery.cpp
  src/tracking/crossing_tracking.cpp
  src/tracking/selected_target.cpp
  src/output/visualization.cpp
  src/common/tracking_geometry.cpp
  src/yolo/yolo_initialization.cpp
  src/fusion/fusion_integration.cpp
  src/output/target_output.cpp
  src/output/target_path.cpp
  src/tracking/merge_support.cpp
  src/third_party/Hungarian.cpp
)
add_dependencies(person_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(person_tracker_node person_tracker_projection person_tracker_ground
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(person_tracker_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS person_tracker_node person_tracker_projection person_tracker_ground
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY tools/imm_mot DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tools)
