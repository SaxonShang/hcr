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

catkin_install_python(PROGRAMS
  scripts/move_target_box.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf worlds rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
