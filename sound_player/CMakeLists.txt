cmake_minimum_required(VERSION 2.8.3)
project(sound_player)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
find_package(sound_play REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/sound_player.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/sound_player.yaml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY wavs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
