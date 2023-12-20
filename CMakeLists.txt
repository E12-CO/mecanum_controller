cmake_minimum_required(VERSION 3.8)
project(mecanum_controller)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(mecanum_dependencies
	rclcpp 
	rclcpp_components 
	tf2
	tf2_ros 
	tf2_geometry_msgs 
	tf2_sensor_msgs 
	tf2_eigen 
	geometry_msgs 
	nav_msgs
	std_srvs
	std_msgs
)

add_executable(mecanum_controller src/mecanum_controller.cpp)
target_link_libraries(
	mecanum_controller
)

ament_target_dependencies(
	mecanum_controller
	${mecanum_dependencies}	
)

install(
	TARGETS	mecanum_controller
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY launch params
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
