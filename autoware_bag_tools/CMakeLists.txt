cmake_minimum_required(VERSION 2.8.3)
project(autoware_bag_tools)

find_package(catkin REQUIRED COMPONENTS
  nmea_msgs
  rosbag
  roslib
  rospy
)

find_package(cmake_modules REQUIRED)

find_package(TinyXML REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(
  PROGRAMS
    scripts/change_frame_id.py
    scripts/replace_msg_time_with_hdr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(nmea2kml nodes/nmea2kml/nmea2kml.cpp nodes/nmea2kml/ReadNMEASentence.cpp)
target_link_libraries(nmea2kml ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

add_dependencies(nmea2kml ${catkin_EXPORTED_TARGETS})

install(TARGETS nmea2kml
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
