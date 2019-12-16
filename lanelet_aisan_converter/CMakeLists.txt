cmake_minimum_required(VERSION 2.8.3)
project(lanelet_aisan_converter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  geodesy
  geometry_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  map_file
  roscpp
  roslib
  roslint
  vector_map
  vector_map_msgs
)

# Find boost geometry
find_package(Boost REQUIRED COMPONENTS
  filesystem
)

# Register as a catkin package
catkin_package(
  DEPENDS
    Boost
)

# Set up roslint
set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(aisan_map STATIC
  lib/aisan_map/aisan_map.cpp
)

add_dependencies(aisan_map ${catkin_EXPORTED_TARGETS})

target_link_libraries(aisan_map
  ${catkin_LIBRARIES}
)

# Target: lanelet_helper
add_library(lanelet_helper STATIC
  lib/lanelet_helper/lanelet_helper.cpp
)

add_dependencies(lanelet_helper ${catkin_EXPORTED_TARGETS})

target_link_libraries(lanelet_helper
  ${catkin_LIBRARIES}
)

# Target: lanelet2aisan_converter
add_library(lanelet2aisan_converter STATIC
  lib/lanelet2aisan_converter/create_basic_shape.cpp
  lib/lanelet2aisan_converter/create_feature.cpp
  lib/lanelet2aisan_converter/overwrite.cpp
  lib/lanelet2aisan_converter/lanelet2aisan_converter.cpp
)

add_dependencies(lanelet2aisan_converter ${catkin_EXPORTED_TARGETS})

target_link_libraries(lanelet2aisan_converter
  aisan_map
  lanelet_helper
  ${catkin_LIBRARIES}
)

# Target: lanelet2aisan
add_executable(lanelet2aisan
  nodes/lanelet2aisan/lanelet2aisan.cpp
)

add_dependencies(lanelet2aisan ${catkin_EXPORTED_TARGETS})

target_link_libraries(lanelet2aisan
  lanelet2aisan_converter
  ${catkin_LIBRARIES}
)

# Install targets
install(
  TARGETS
    lanelet2aisan
    lanelet_helper
    lanelet2aisan_converter
    aisan_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install scripts
install(
  PROGRAMS scripts/launch_vector_map_loader.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Lint
  roslint_add_test()

  # Target: test_lanelet2aisan_converter
  add_rostest_gtest(test_lanelet2aisan_converter
    test/test_lanelet2aisan_converter.test
    test/src/test_lanelet2aisan_converter.cpp
  )

  target_link_libraries(test_lanelet2aisan_converter
    lanelet2aisan_converter
    ${catkin_LIBRARIES}
  )
endif ()
