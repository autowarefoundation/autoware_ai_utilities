cmake_minimum_required(VERSION 2.8.3)
project(synchronization)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  lidar_tracker
  points2image
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sync_drivers sync_drivers.cpp)
target_link_libraries(sync_drivers ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(sync_drivers ${catkin_EXPORTED_TARGETS})

add_executable(sync_range_fusion computing/perception/detection/packages/cv_tracker/nodes/range_fusion/sync_range_fusion.cpp)
target_link_libraries(sync_range_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sync_track computing/perception/detection/packages/cv_tracker/nodes/kf_track/sync_track.cpp)
target_link_libraries(sync_track ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sync_obj_reproj computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/sync_obj_reproj.cpp)
target_link_libraries(sync_obj_reproj ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sync_obj_fusion computing/perception/detection/packages/lidar_tracker/nodes/obj_fusion/sync_obj_fusion.cpp)
target_link_libraries(sync_obj_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(time_monitor time_monitor.cpp)
target_link_libraries(time_monitor ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(time_monitor ${catkin_EXPORTED_TARGETS})

        
install(
  TARGETS
    sync_drivers
    sync_obj_fusion
    sync_obj_reproj
    sync_range_fusion
    sync_track
    time_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
        
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
