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
	rviz
	tf
	interactive_markers
	visualization_msgs
	roboy_communication_middleware
	roboy_communication_simulation
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

catkin_package( DEPENDS GAZEBO CATKIN_DEPENDS rosbag message_runtime message_generation roboy_communication_middleware
		roboy_communication_simulation INCLUDE_DIRS include
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
add_library(muscle_plugin SHARED src/muscle/IMuscle.cpp src/muscle/CylindricalWrapping.cpp src/muscle/IActuator.cpp
        src/muscle/ISee.cpp src/muscle/IViaPoints.cpp src/muscle/MeshWrapping.cpp src/muscle/SphericalWrapping.cpp
        src/muscle/StateMachine.cpp)
target_link_libraries(muscle_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})

add_library(dummy_muscle_plugin SHARED src/muscle/DummyMusclePlugin.cpp)
target_link_libraries(dummy_muscle_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

qt5_wrap_cpp(MOC_FILES include/roboy_simulation/walking_plugin.hpp)
add_library(roboy_rviz src/walking_plugin.cpp ${MOC_FILES})
add_dependencies(roboy_rviz roboy_communication_simulation_gencpp)
target_link_libraries(roboy_rviz ${QT_LIBRARIES} ${catkin_LIBRARIES})

qt5_wrap_cpp(MOC_FILES include/roboy_simulation/balancing_plugin.hpp)
add_library(roboy_rviz_balancing src/balancing_plugin.cpp ${MOC_FILES})
add_dependencies(roboy_rviz_balancing roboy_communication_simulation_gencpp)
target_link_libraries(roboy_rviz_balancing ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(contact_plugin SHARED src/ContactPlugin.cpp)
target_link_libraries(contact_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(force_torque_plugin SHARED src/ForceTorquePlugin.cpp)
add_dependencies(force_torque_plugin roboy_communication_simulation_gencpp)
target_link_libraries(force_torque_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(force_torque_plugin_halterung SHARED src/ForceTorquePluginHalterung.cpp)
add_dependencies(force_torque_plugin_halterung roboy_communication_simulation_gencpp)
target_link_libraries(force_torque_plugin_halterung ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(force_compensating_halterung SHARED src/ForceCompensatingHalterung.cpp)
add_dependencies(force_compensating_halterung roboy_communication_middleware_gencpp roboy_communication_simulation_gencpp)
target_link_libraries(force_compensating_halterung ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(WalkController SHARED src/walkController.cpp src/walkVisualization.cpp src/helperClasses.cpp)
add_dependencies(WalkController roboy_communication_middleware_gencpp roboy_communication_simulation_gencpp)
target_link_libraries(WalkController ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(ForceJointPlugin SHARED src/ForceJointPlugin.cpp)
target_link_libraries(ForceJointPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(roboySim src/main.cpp src/simulationControl.cpp)
add_dependencies(roboySim roboy_communication_middleware_gencpp roboy_communication_simulation_gencpp)
target_link_libraries(roboySim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(VRRoboy src/VRRoboy.cpp src/simulationControl.cpp)
add_dependencies(VRRoboy roboy_communication_middleware_gencpp roboy_communication_simulation_gencpp)
target_link_libraries(VRRoboy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(PaBiDanceSimulator src/PaBiDanceSimulator.cpp)
add_dependencies(roboy_rviz_balancing roboy_communication_middleware_gencpp)
target_link_libraries(PaBiDanceSimulator ${catkin_LIBRARIES})

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        muscle_plugin
        dummy_muscle_plugin
        roboy_rviz
        roboy_rviz_balancing
        contact_plugin
        force_torque_plugin
		WalkController
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
        roboySim
        VRRoboy
	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        package.xml
		dummy_muscle_plugin.xml
        muscle_plugin.xml
        walking_plugin.xml
        balancing_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
