cmake_minimum_required(VERSION 2.8.3)
project(twist2odom)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  geometry_msgs
  nav_msgs
  roscpp
  roslint
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(${PROJECT_NAME}
  src/twist2odom.cpp
  src/twist2odom_node.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()

  find_package(rostest REQUIRED)
  add_rostest_gtest(
    ${PROJECT_NAME}_test
    test/twist2odom_test.test
    test/twist2odom_test.cpp
    src/twist2odom.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
  )
endif()
