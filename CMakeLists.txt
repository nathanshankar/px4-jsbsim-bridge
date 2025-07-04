cmake_minimum_required(VERSION 3.5)
project(jsbsim_bridge VERSION 0.1.0 DESCRIPTION "JSBSim bridge for PX4 and ROS 2")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(MAVLink REQUIRED)
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CURL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)

# JSBSim detection (manual)
if(NOT JSBSIM_FOUND)
	if(DEFINED ENV{JSBSIM_ROOT_DIR})
		set(JSBSIM_ROOT_DIR "$ENV{JSBSIM_ROOT_DIR}")
	endif()

	find_path(JSBSIM_INCLUDE_DIR
		NAMES FGFDMExec.h
		PATHS
			${JSBSIM_ROOT_DIR}/include/JSBSim
			/usr/include/JSBSim
			/usr/local/include/JSBSim
	)

	find_library(JSBSIM_LIBRARY
		NAMES JSBSim
		PATHS
			${JSBSIM_ROOT_DIR}/lib
			/usr/lib/
			/usr/local/lib
	)

	if(JSBSIM_INCLUDE_DIR AND JSBSIM_LIBRARY)
		set(JSBSIM_FOUND TRUE)
	endif()

	if(NOT JSBSIM_FOUND)
		message(FATAL_ERROR "Could not find JSBSIM")
	endif()
endif()

add_definitions(-DJSBSIM_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(
	-Wno-address-of-packed-member   # MAVLink annoyances
	--warn-uninitialized
)

include_directories(include)
include_directories(/opt/ros/jazzy/include/mavlink/v2.0)


add_executable(jsbsim_bridge
	src/main.cpp
	src/configuration_parser.cpp
	src/jsbsim_bridge.cpp
	src/geo_mag_declination.cpp
	src/mavlink_interface.cpp
	src/actuator_plugin.cpp
	src/sensor_plugin.cpp
	src/sensor_airspeed_plugin.cpp
	src/sensor_baro_plugin.cpp
	src/sensor_imu_plugin.cpp
	src/sensor_gps_plugin.cpp
	src/sensor_mag_plugin.cpp
)

target_include_directories(jsbsim_bridge
	PUBLIC
		${MAVLINK_INCLUDE_DIRS}
		${JSBSIM_INCLUDE_DIR}
		${Boost_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${TinyXML_INCLUDE_DIRS}
)

target_link_libraries(jsbsim_bridge
	${JSBSIM_LIBRARY}
	${Boost_SYSTEM_LIBRARY_RELEASE}
	${Boost_THREAD_LIBRARY_RELEASE}
	${Boost_TIMER_LIBRARY_RELEASE}
	${TinyXML_LIBRARIES}
)

# ROS 2 node
add_executable(jsbsim_bridge_node
	src/jsbsim_bridge_node.cpp
	src/jsbsim_bridge_ros.cpp
	src/configuration_parser.cpp
	src/jsbsim_bridge.cpp
	src/geo_mag_declination.cpp
	src/mavlink_interface.cpp
	src/actuator_plugin.cpp
	src/sensor_plugin.cpp
	src/sensor_airspeed_plugin.cpp
	src/sensor_baro_plugin.cpp
	src/sensor_imu_plugin.cpp
	src/sensor_gps_plugin.cpp
	src/sensor_mag_plugin.cpp
)

target_include_directories(jsbsim_bridge_node
	PUBLIC
		include
		/opt/ros/jazzy/include/mavlink/v2.0
		${MAVLINK_INCLUDE_DIRS}
		${JSBSIM_INCLUDE_DIR}
		${Boost_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${TinyXML_INCLUDE_DIRS}
)

ament_target_dependencies(jsbsim_bridge_node
	rclcpp
	std_msgs
	mavlink
)

target_link_libraries(jsbsim_bridge_node
	${JSBSIM_LIBRARY}
	${Boost_SYSTEM_LIBRARY_RELEASE}
	${Boost_THREAD_LIBRARY_RELEASE}
	${Boost_TIMER_LIBRARY_RELEASE}
	${TinyXML_LIBRARIES}
)

add_executable(jsbsim_stream
	src/fgfs_cockpit_publisher.cpp
)

ament_target_dependencies(jsbsim_stream
	rclcpp
	std_msgs
	cv_bridge
	tf2_ros
)

add_executable(cam_sync
	src/sync_cam_msg.cpp)

ament_target_dependencies(cam_sync
	rclcpp
	sensor_msgs
	message_filters
	image_transport
	cv_bridge
)

target_link_libraries(cam_sync
	${OpenCV_LIBS}
)


target_link_libraries(jsbsim_stream
	${OpenCV_LIBS}
	CURL::libcurl
)
install(TARGETS jsbsim_bridge jsbsim_bridge_node jsbsim_stream cam_sync
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
	DESTINATION include/
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(jsbsim_bridge)
ament_export_dependencies(rclcpp std_msgs MAVLink Boost TinyXML Eigen3)

ament_package()
