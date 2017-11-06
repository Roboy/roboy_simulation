cmake_minimum_required(VERSION 2.8.3)
project(roboy_simulation)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosbag
	controller_manager
	gazebo_ros_control
	std_msgs
	geometry_msgs
	message_generation
	rviz
	tf
	interactive_markers
	visualization_msgs
	common_utilities
)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
message(STATUS ${GAZEBO_INCLUDE_DIRS})

find_package(PkgConfig)

add_message_files(
        DIRECTORY
        msgs

        FILES
        MotorControl.msg
		PIDControl.msg
)

add_service_files(
        DIRECTORY
        srvs

        FILES
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
	DEPENDS
	GAZEBO
	CATKIN_DEPENDS
		rosbag
		message_runtime
		message_generation
		common_utilities
		flexrayusbinterface
)

include_directories(
		include/
		include/muscle
        ${catkin_INCLUDE_DIRS}
		${Qt5Widgets_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================
add_library(muscle_plugin SHARED 
	src/RobotSimulation/ModelController.cpp
	src/RobotSimulation/ModelViz.cpp	
	src/RobotSimulation/pid.cpp	
	src/RobotSimulation/muscle/IMuscle.cpp
	src/RobotSimulation/muscle/CylindricalWrapping.cpp
	src/RobotSimulation/muscle/IActuator.cpp
	src/RobotSimulation/muscle/ISee.cpp
	src/RobotSimulation/muscle/IViaPoints.cpp
	src/RobotSimulation/muscle/MeshWrapping.cpp
	src/RobotSimulation/muscle/SphericalWrapping.cpp
	src/RobotSimulation/muscle/StateMachine.cpp
	)
target_link_libraries(muscle_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        muscle_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        package.xml
        muscle_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
