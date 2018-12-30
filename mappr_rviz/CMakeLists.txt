cmake_minimum_required(VERSION 3.1)
project(mappr_rviz)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Catkin
find_package(catkin REQUIRED
        COMPONENTS
            rviz
            mappr_msgs
            
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mappr_msgs
)

include_directories(${catkin_INCLUDE_DIRS} )
link_directories(${catkin_LIBRARY_DIRS})

## QT
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)
#set(CMAKE_AUTOMOC ON)

## Etc
include_directories(include)

###########
## Build ##
###########
set(src_files
  src/location_display.cpp
  src/location_array_display.cpp
  src/viewpoint_array_display.cpp
  src/location_visual.cpp
  src/interactive_node.cpp
  src/viewpoint.cpp
  )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
qt_wrap_cpp(${PROJECT_NAME} 
  moc_files 
  include/mappr_rviz/topic_display.h 
  include/mappr_rviz/location_display.h 
  include/mappr_rviz/location_array_display.h
  include/mappr_rviz/viewpoint_array_display.h
  include/mappr_rviz/interactive_node.h
  include/mappr_rviz/viewpoint.h
  )

add_library(${PROJECT_NAME} ${src_files} ${moc_files})

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES} ${catkin_LIBRARIES}
 )

#############
## Install ##
#############
# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark config files for installation
install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
