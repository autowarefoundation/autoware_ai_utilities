cmake_minimum_required(VERSION 2.8.3)
project(mqtt_socket)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_can_msgs
  autoware_msgs
  roscpp
  roslib
  std_msgs
  tablet_socket_msgs
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Mosquitto REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

if(MOSQUITTO_FOUND)
  set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${MOSQUITTO_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
  )

  add_executable(mqtt_sender nodes/mqtt_sender/mqtt_sender.cpp)
  target_link_libraries(mqtt_sender ${MOSQUITTO_LIBRARIES} yaml-cpp ${catkin_LIBRARIES})
  add_dependencies(mqtt_sender ${catkin_EXPORTED_TARGETS})

  add_executable(mqtt_receiver nodes/mqtt_receiver/mqtt_receiver.cpp)
  target_link_libraries(mqtt_receiver ${MOSQUITTO_LIBRARIES} yaml-cpp ${catkin_LIBRARIES})
  add_dependencies(mqtt_receiver ${catkin_EXPORTED_TARGETS})

  install(TARGETS mqtt_receiver mqtt_sender
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message("'libmosquitto-dev' is not installed. 'mqtt_sender' and 'mqtt_receiver' will not be built.")
endif()
