cmake_minimum_required(VERSION 2.8.3)
project(marker_downsampler)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  rospy
  std_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS scripts/app.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
