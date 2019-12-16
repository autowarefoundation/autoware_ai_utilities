cmake_minimum_required(VERSION 2.8.3)
project(pc2_downsampler)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/app.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
