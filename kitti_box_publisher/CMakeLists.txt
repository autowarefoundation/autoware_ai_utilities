cmake_minimum_required(VERSION 2.8.3)
project(kitti_box_publisher)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_recognition_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/kitti_box_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
