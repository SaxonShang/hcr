cmake_minimum_required(VERSION 3.0.2)
project(p3at_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2_ros
  robot_state_publisher
  joint_state_publisher
  xacro
  gazebo_ros
  gazebo_plugins
)

catkin_package()

install(DIRECTORY launch urdf worlds rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
