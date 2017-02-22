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
	flexrayusbinterface
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
        Tendon.msg
        VisualizationControl.msg
        RecordingControl.msg
        ForceTorque.msg
        IMU.msg
        LegState.msg
        ControllerParameters.msg
        Abortion.msg
        MotorControl.msg
        Joint.msg
        BodyPart.msg
        COM.msg
        Input.msg
)

add_service_files(
        DIRECTORY
        srvs

        FILES
        UpdateControllerParameters.srv
        Energies.srv
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
add_library(muscle_plugin SHARED src/muscle/IMuscle.cpp src/muscle/CylindricalWrapping.cpp src/muscle/IActuator.cpp
        src/muscle/ISee.cpp src/muscle/IViaPoints.cpp src/muscle/MeshWrapping.cpp src/muscle/SphericalWrapping.cpp
        src/muscle/StateMachine.cpp)
target_link_libraries(muscle_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})

add_library(dummy_muscle_plugin SHARED src/muscle/DummyMusclePlugin.cpp)
target_link_libraries(dummy_muscle_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

qt5_wrap_cpp(MOC_FILES include/roboy_simulation/walking_plugin.hpp)
add_library(roboy_rviz src/walking_plugin.cpp ${MOC_FILES})
add_dependencies(roboy_rviz roboy_simulation_gencpp)
target_link_libraries(roboy_rviz ${QT_LIBRARIES} ${catkin_LIBRARIES})

qt5_wrap_cpp(MOC_FILES include/roboy_simulation/balancing_plugin.hpp)
add_library(roboy_rviz_balancing src/balancing_plugin.cpp ${MOC_FILES})
add_dependencies(roboy_rviz_balancing roboy_simulation_gencpp)
target_link_libraries(roboy_rviz_balancing ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(contact_plugin SHARED src/ContactPlugin.cpp)
target_link_libraries(contact_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(force_torque_plugin SHARED src/ForceTorquePlugin.cpp)
add_dependencies(force_torque_plugin roboy_simulation_gencpp)
target_link_libraries(force_torque_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(force_torque_plugin_halterung SHARED src/ForceTorquePluginHalterung.cpp)
add_dependencies(force_torque_plugin_halterung roboy_simulation_gencpp)
target_link_libraries(force_torque_plugin_halterung ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(force_compensating_halterung SHARED src/ForceCompensatingHalterung.cpp)
add_dependencies(force_compensating_halterung common_utilities_gencpp roboy_simulation_gencpp)
target_link_libraries(force_compensating_halterung ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(WalkController SHARED src/walkController.cpp src/walkVisualization.cpp src/helperClasses.cpp)
add_dependencies(WalkController common_utilities_gencpp roboy_simulation_gencpp)
target_link_libraries(WalkController ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(roboySim src/main.cpp src/simulationControl.cpp)
add_dependencies(roboySim common_utilities_gencpp roboy_simulation_gencpp)
target_link_libraries(roboySim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(VRRoboy src/VRRoboy.cpp src/simulationControl.cpp)
add_dependencies(VRRoboy common_utilities_gencpp roboy_simulation_gencpp)
target_link_libraries(VRRoboy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )
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
