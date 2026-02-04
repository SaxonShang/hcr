cmake_minimum_required(VERSION 3.0.2)
project(target_follower)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/follow_target.py
  scripts/target_pos_mux.py
  scripts/target_candidate_selector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
