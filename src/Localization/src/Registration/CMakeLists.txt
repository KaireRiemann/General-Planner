cmake_minimum_required(VERSION 3.0.2)
project(registration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
)


find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pc_registered
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library( registration 
    src/registration.cpp 
    )

target_link_libraries( registration
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  


# add_executable(icp_test_node app/icp_test_node.cpp )
# target_link_libraries(icp_test_node
#   ${catkin_LIBRARIES}  
#   ${PCL_LIBRARIES}
# )

# add_executable(visual_lidar_test_node app/visual_lidar_registration_test.cpp )
# target_link_libraries(visual_lidar_test_node
#   registration
#   ${catkin_LIBRARIES}  
#   ${PCL_LIBRARIES}
# )

add_executable(lidar2lidar_registration_node app/l2l_registration_node.cpp )
target_link_libraries(lidar2lidar_registration_node
  registration
  ${catkin_LIBRARIES}  
  ${PCL_LIBRARIES}
)

# add_executable(visual2lidar_registration_node app/v2l_registration_node.cpp )
# target_link_libraries(visual2lidar_registration_node
#   ${catkin_LIBRARIES}  
#   ${PCL_LIBRARIES}
# )

# add_executable(show_pcd_node app/show_pcd.cpp )
# target_link_libraries(show_pcd_node
#   ${catkin_LIBRARIES}  
#   ${PCL_LIBRARIES}
# )



# add_executable(reg_vis app/visual_correspondence.cpp )
# target_link_libraries(reg_vis
#   ${catkin_LIBRARIES}  
#   ${PCL_LIBRARIES}
# )