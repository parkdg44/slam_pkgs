cmake_minimum_required(VERSION 3.8)
project(slam_main)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(dependencies)
include(ros2)

MESSAGE(STATUS "----------------------------")
MESSAGE(STATUS "LIBS             = ${LIBS}")
MESSAGE(STATUS "TARGET_LINK_LIBS = ${TARGET_LINK_LIBS}")
MESSAGE(STATUS "ROS2_LIBS        = ${ROS2_LIBS}")
MESSAGE(STATUS "----------------------------")

include_directories(include)
file(GLOB_RECURSE SRCS src/*.cpp)

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${TARGET_LINK_LIBS})
ament_target_dependencies(${PROJECT_NAME} ${ROS2_LIBS})

install(TARGETS
	${PROJECT_NAME}
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
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
