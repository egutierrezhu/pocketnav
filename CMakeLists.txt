cmake_minimum_required(VERSION 2.8.3)
project(pocketnav)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS nodes/recognizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS nodes/voice_nav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
