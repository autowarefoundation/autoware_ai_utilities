cmake_minimum_required(VERSION 2.8.12)
project(autoware_camera_lidar_calibrator)

include(FindPkgConfig)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  cv_bridge
  geometry_msgs
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL)

catkin_package()

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#camera lidar node
add_executable(autoware_camera_lidar_calibration_node
  src/autoware_camera_lidar_calibration/camera_lidar_calibration_node.cpp
)
target_link_libraries(autoware_camera_lidar_calibration_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(autoware_camera_lidar_calibration_node ${catkin_EXPORTED_TARGETS})

#camera intrinsics node
catkin_python_setup()

catkin_install_python(
  PROGRAMS
    nodes/cameracalibrator.py
    nodes/cameracheck.py
    scripts/tarfile_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  autoware_camera_lidar_calibration_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
  PATTERN ".svn" EXCLUDE
)
