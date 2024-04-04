cmake_minimum_required(VERSION 3.8)
project(slam_main)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(dependencies)

# find ament dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# PCL
#find_package(PCL REQUIRED COMPONENTS)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

file(GLOB_RECURSE SRCS src/*.cpp)

include_directories(include/slam_pkg)
add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBS})
ament_target_dependencies(${PROJECT_NAME}
	rclcpp
	sensor_msgs
	std_msgs
	nav_msgs
	geometry_msgs
	tf2
	tf2_ros
)

if (BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# comment the line when a copyright and license is added to all source files
	set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# comment the line when this package is in a git repo and when
	# a copyright and license is added to all source files
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
