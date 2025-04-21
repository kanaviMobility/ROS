cmake_minimum_required(VERSION 3.8)
project(kanavi_vl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_definitions( -fPIC )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{ROS_VERSION})
    if($ENV{ROS_VERSION} STREQUAL "1")  # ROS1
        # set(ROS1 TRUE)
        message(STATUS "Detected ROS1 environment")
		add_definitions(-DROS1)
		set(ROS_MACRO ROS1)  # ROS1 매크로
    elseif($ENV{ROS_VERSION} STREQUAL "2")  # ROS2
        # set(ROS2 TRUE)
        message(STATUS "Detected ROS2 environment")
		add_definitions(-DROS2)
		set(ROS_MACRO ROS2)  # ROS1 매크로
    else()
        message(FATAL_ERROR "Unsupported ROS_VERSION: $ENV{ROS_VERSION}")
    endif()
else()
    message(FATAL_ERROR "ROS_VERSION is not set. Please source your ROS environment.")
endif()

if($ENV{ROS_VERSION} STREQUAL "1")

	message("ROS1 BUILD")

	find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	pcl_conversions
	pcl_ros
	visualization_msgs
	)

 	catkin_package()
	
	set(LIBRARIES
		kanavi_node
		kanavi_udp
		kanavi_lidar
	)

	foreach(LIBRARY ${LIBRARIES})
		set(LIB_OBJS ${LIB_OBJS} $<TARGET_OBJECTS:${LIBRARY}>)
	endforeach()

	add_library(kanavi_node
	src/node_ros1/kanavi_node.cpp)

	add_library(kanavi_udp
	src/udp/udp.cpp)

	add_library(kanavi_lidar
	src/lidar/kanavi_lidar.cpp)

	
###########
## Build ##
###########

	include_directories(
		include
		include/${PROJECT_NAME}
		include/${PROJECT_NAME}/ros1
		${catkin_INCLUDE_DIRS}
	)

	#----define R270 node 
	add_executable(R270 
		src/R270/main.cpp
		${LIB_OBJS}
	)

	target_link_libraries(R270
		kanavi_node
		kanavi_udp
		kanavi_lidar
		${catkin_LIBRARIES}
	)

	#----define R4 node 
	add_executable(R4 
		src/R4/main.cpp
		${LIB_OBJS}
	)

	target_link_libraries(R4
		kanavi_node
		kanavi_udp
		kanavi_lidar
		${catkin_LIBRARIES}
	)
	
#############
## Install ##
#############

#--------------ROS2----------------------

elseif($ENV{ROS_VERSION} STREQUAL "2")
	message("ROS2 BUILD...")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
	ament_cmake
	rclcpp
	pcl_conversions
	sensor_msgs
	std_msgs
)

# # set user defined Libs
set(LIBRARIES 	
	kanavi_node
	kanavi_udp
	kanavi_lidar
)

foreach(LIBRARY ${LIBRARIES})
	set(LIB_OBJS ${LIB_OBJS} $<TARGET_OBJECTS:${LIBRARY}>)
endforeach()

include_directories(
	${PCL_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${rclcpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${pcl_conversions_INCLUDE_DIRS}
	include
	include/${PROJECT_NAME}
)

# # add sub dir.
add_subdirectory(src/node_ros2)
add_subdirectory(src/udp)
add_subdirectory(src/lidar)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

#----define R2 node 
add_executable(R2 
	src/R2/main.cpp
	${LIB_OBJS}
)
ament_target_dependencies(R2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
#-----------------------------------------------------------

#----define R4 node 
add_executable(R4 
	src/R4/main.cpp
	${LIB_OBJS}
)
ament_target_dependencies(R4 ${THIS_PACKAGE_INCLUDE_DEPENDS})
#-----------------------------------------------------------

#----define R270 node 
add_executable(R270 
	src/R270/main.cpp
	${LIB_OBJS}
)
ament_target_dependencies(R270 ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(R270 
	${PCL_LIBRARIES}
	${EIGEN_LIBRARIES}
)
#-----------------------------------------------------------

install(TARGETS R2 R4 R270
		DESTINATION lib/${PROJECT_NAME})

ament_package()

endif()
