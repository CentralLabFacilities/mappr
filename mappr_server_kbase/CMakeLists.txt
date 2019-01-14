cmake_minimum_required(VERSION 2.8.3)
project(mappr_server_kbase)

## Catkin
find_package(catkin REQUIRED COMPONENTS
  rospy
  mappr_msgs
)

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS rospy mappr_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src" DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
CATKIN_INSTALL_PYTHON(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/src/server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/mappr_server_kbase)
