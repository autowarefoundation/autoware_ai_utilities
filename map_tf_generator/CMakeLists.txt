cmake_minimum_required(VERSION 2.8.3)
project(map_tf_generator)

add_compile_options(-std=c++11)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  map_tf_generator nodes/map_tf_generator.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  map_tf_generator ${catkin_LIBRARIES}
)

install(TARGETS map_tf_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
