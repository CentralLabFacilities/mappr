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


#############
## Testing ##
#############

## Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test)
endif()