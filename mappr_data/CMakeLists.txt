cmake_minimum_required(VERSION 2.8.3)
project(mappr_data)

find_package(catkin REQUIRED)

##################################
## CATKIN
##################################
catkin_package(INCLUDE_DIRS
        LIBRARIES
               
        CATKIN_DEPENDS 
                
        DEPENDS
                
        )

##############################
##  Build
##############################
include_directories(${catkin_INCLUDE_DIRS})

##############################
## Install rules
##############################
install(DIRECTORY rviz_config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
