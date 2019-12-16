cmake_minimum_required(VERSION 2.8.3)
project(tablet_socket)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_can_msgs
  autoware_msgs
  geometry_msgs
  gnss
  roscpp
  tablet_socket_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tablet_receiver nodes/tablet_receiver/tablet_receiver.cpp)
target_link_libraries(tablet_receiver ${catkin_LIBRARIES})
add_dependencies(tablet_receiver ${catkin_EXPORTED_TARGETS})

add_executable(tablet_sender nodes/tablet_sender/tablet_sender.cpp)
target_link_libraries(tablet_sender ${catkin_LIBRARIES})
add_dependencies(tablet_sender ${catkin_EXPORTED_TARGETS})

install(TARGETS tablet_receiver tablet_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
