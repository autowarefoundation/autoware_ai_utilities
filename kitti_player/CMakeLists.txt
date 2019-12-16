cmake_minimum_required(VERSION 2.8.3)
project(kitti_player)

SET(CMAKE_CXX_FLAGS "-O3")

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  std_msgs
  stereo_msgs
  tf
  tf2
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Core REQUIRED)

generate_dynamic_reconfigure_options(cfg/kitti_player.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_LIBS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kitti_player src/kitti_player.cpp)

target_link_libraries(kitti_player
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(kitti_player ${catkin_EXPORTED_TARGETS})

install(TARGETS kitti_player
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  PATTERN ".svn" EXCLUDE
)
