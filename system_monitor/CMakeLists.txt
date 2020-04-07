cmake_minimum_required(VERSION 2.8.3)
project(system_monitor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roslint
  roscpp
  std_msgs
)

find_package(CUDA)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES system_monitor
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if (CUDA_FOUND)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
endif()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/system_monitor_node.cpp)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(CMAKE_CPU_PLATFORM "intel")
  ADD_DEFINITIONS(-D_CPU_INTEL_)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  if (CMAKE_HOST_SYSTEM_VERSION MATCHES ".*raspi.*")
    set(CMAKE_CPU_PLATFORM "raspi")
    ADD_DEFINITIONS(-D_CPU_RASPI_)
  elseif (CMAKE_HOST_SYSTEM_VERSION MATCHES ".*tegra.*")
    set(CMAKE_CPU_PLATFORM "tegra")
    ADD_DEFINITIONS(-D_CPU_TEGRA_)
  else()
    set(CMAKE_CPU_PLATFORM "arm")
    ADD_DEFINITIONS(-D_CPU_ARM_)
  endif()
else()
  set(CMAKE_CPU_PLATFORM "unknown")
endif()

find_library(NVML NAMES nvidia-ml) 
if (NVML)
  set(CMAKE_GPU_PLATFORM "nvml")
  ADD_DEFINITIONS(-D_GPU_NVML_)
  set(GPU_LIBRARY nvidia-ml)
else()
  if (CMAKE_CPU_PLATFORM STREQUAL "tegra")
    set(CMAKE_GPU_PLATFORM "tegra")
    ADD_DEFINITIONS(-D_GPU_TEGRA_)
  else()
    set(CMAKE_GPU_PLATFORM "unknown")
  endif()
endif()

message(STATUS "CUDA_FOUND: " ${CUDA_FOUND})
message(STATUS "HOST_SYSTEM_VERSION: " ${CMAKE_HOST_SYSTEM_VERSION})
message(STATUS "SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR})
message(STATUS "CPU PLATFORM: " ${CMAKE_CPU_PLATFORM})
message(STATUS "GPU PLATFORM: " ${CMAKE_GPU_PLATFORM})

set(CPU_MONITOR_SOURCE
  src/cpu_monitor/cpu_monitor_base.cpp
  src/cpu_monitor/${CMAKE_CPU_PLATFORM}_cpu_monitor.cpp
)

add_executable(cpu_monitor
  src/cpu_monitor/cpu_monitor_node.cpp
  ${CPU_MONITOR_SOURCE}
)

add_executable(hdd_monitor
  src/hdd_monitor/hdd_monitor_node.cpp
  src/hdd_monitor/hdd_monitor.cpp
)
add_executable(mem_monitor
  src/mem_monitor/mem_monitor_node.cpp
  src/mem_monitor/mem_monitor.cpp
)
add_executable(net_monitor
  src/net_monitor/net_monitor_node.cpp
  src/net_monitor/net_monitor.cpp
)
add_executable(ntp_monitor
  src/ntp_monitor/ntp_monitor_node.cpp
  src/ntp_monitor/ntp_monitor.cpp
)
add_executable(process_monitor
  src/process_monitor/process_monitor_node.cpp
  src/process_monitor/process_monitor.cpp
)

set(GPU_MONITOR_SOURCE
  src/gpu_monitor/gpu_monitor_base.cpp
  src/gpu_monitor/${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
)
add_executable(gpu_monitor
  src/gpu_monitor/gpu_monitor_node.cpp
  ${GPU_MONITOR_SOURCE}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cpu_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(hdd_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(mem_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(net_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(ntp_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(process_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(gpu_monitor ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(cpu_monitor ${catkin_LIBRARIES})
target_link_libraries(hdd_monitor ${catkin_LIBRARIES})
target_link_libraries(mem_monitor ${catkin_LIBRARIES})
target_link_libraries(net_monitor ${catkin_LIBRARIES})
target_link_libraries(ntp_monitor ${catkin_LIBRARIES})
target_link_libraries(process_monitor ${catkin_LIBRARIES})
target_link_libraries(gpu_monitor ${catkin_LIBRARIES} ${GPU_LIBRARY})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS cpu_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS hdd_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS mem_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS net_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ntp_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS process_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS gpu_monitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

# Install launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_system_monitor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)
  
  add_rostest_gtest(test_cpu_monitor
    test/test_cpu_monitor.test
    test/src/cpu_monitor/test_${CMAKE_CPU_PLATFORM}_cpu_monitor.cpp
    ${CPU_MONITOR_SOURCE}
  )
  target_link_libraries(test_cpu_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_hdd_monitor
    test/test_hdd_monitor.test
    test/src/hdd_monitor/test_hdd_monitor.cpp
    src/hdd_monitor/hdd_monitor.cpp
  )
  target_link_libraries(test_hdd_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_mem_monitor
    test/test_mem_monitor.test
    test/src/mem_monitor/test_mem_monitor.cpp
    src/mem_monitor/mem_monitor.cpp
  )
  target_link_libraries(test_mem_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_net_monitor
    test/test_net_monitor.test
    test/src/net_monitor/test_net_monitor.cpp
    src/net_monitor/net_monitor.cpp
  )
  target_link_libraries(test_net_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_ntp_monitor
    test/test_ntp_monitor.test
    test/src/ntp_monitor/test_ntp_monitor.cpp
    src/ntp_monitor/ntp_monitor.cpp
  )
  target_link_libraries(test_ntp_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_process_monitor
    test/test_process_monitor.test
    test/src/process_monitor/test_process_monitor.cpp
    src/process_monitor/process_monitor.cpp
  )
  target_link_libraries(test_process_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_gpu_monitor
    test/test_gpu_monitor.test
    test/src/gpu_monitor/test_${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
    ${GPU_MONITOR_SOURCE}
  )
  target_link_libraries(test_gpu_monitor ${catkin_LIBRARIES} ${GPU_LIBRARY})

  # Dummy executables
  add_executable(mpstat1 test/src/cpu_monitor/mpstat1.cpp)
  add_executable(mpstat2 test/src/cpu_monitor/mpstat2.cpp)
  add_executable(df1 test/src/hdd_monitor/df1.cpp)
  add_executable(free1 test/src/mem_monitor/free1.cpp)
  add_executable(iw1 test/src/net_monitor/iw1.cpp)
  add_executable(ntpdate1 test/src/ntp_monitor/ntpdate1.cpp)
  add_executable(top1 test/src/process_monitor/top1.cpp)
  add_executable(top2 test/src/process_monitor/top2.cpp)
  add_executable(top3 test/src/process_monitor/top3.cpp)
  add_executable(echo1 test/src/process_monitor/sort1.cpp)
  add_executable(sort1 test/src/process_monitor/sort1.cpp)
  add_executable(sed1 test/src/process_monitor/sed1.cpp)
  
  install(TARGETS mpstat1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS mpstat2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS df1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS free1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS iw1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS ntpdate1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS top1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS top2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS top3 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS echo1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS sort1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS sed1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()