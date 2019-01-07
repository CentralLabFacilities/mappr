cmake_minimum_required(VERSION 2.8.3)
project(mappr_server)

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