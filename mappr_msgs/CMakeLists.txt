cmake_minimum_required(VERSION 2.8.3)
project(mappr_msgs)

set(MESSAGE_DEPS
		std_msgs
		geometry_msgs
    )

find_package(catkin REQUIRED 
	COMPONENTS 
		message_generation 
		${MESSAGE_DEPS}
)

add_message_files(
	FILES
		Location.msg
		LocationArray.msg
		Viewpoint.msg
		ViewpointArray.msg
)

add_service_files(
	FILES
		UpdateLocation.srv
		UpdateViewpoint.srv
)

generate_messages(
	DEPENDENCIES 
		${MESSAGE_DEPS}
)

catkin_package(
	CATKIN_DEPENDS 
		message_runtime 
		${MESSAGE_DEPS}
)
