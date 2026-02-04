cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

## Compile options
add_compile_options(-std=c++14 -O3 -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  octomap_ros
  message_generation
  message_runtime
  cv_bridge
  message_filters
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

## System dependencies
find_package(Eigen3 REQUIRED)

# catkin_package() expects these variable names
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen3_LIBRARIES "")
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)
add_definitions(${PCL_DEFINITIONS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  CheckCollision.srv
  RandomSample.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp rospy
    std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
    octomap_ros
    cv_bridge message_filters image_transport
    tf2 tf2_ros tf2_geometry_msgs
    pcl_ros pcl_conversions
    message_runtime
  DEPENDS PCL OpenCV Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/occupancyMap.cpp
  include/${PROJECT_NAME}/raycast.cpp
  include/${PROJECT_NAME}/dynamicMap.cpp
  include/${PROJECT_NAME}/detector/kalman_filter.cpp
  include/${PROJECT_NAME}/detector/uv_detector.cpp
)

add_executable(dynamic_map_node src/dynamic_map_node.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dynamic_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(dynamic_map_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} dynamic_map_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch cfg rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
