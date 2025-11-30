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

# Function to add an app with option
function(add_app APP_NAME)
	string(TOUPPER ${APP_NAME} APP_NAME_UPPER)
	option(BUILD_APP_${APP_NAME_UPPER} "Build ${APP_NAME} app" ON)

	if (BUILD_APP_${APP_NAME_UPPER})
		message(STATUS "Building app: ${APP_NAME}")
		add_subdirectory(app/${APP_NAME})
	else ()
		message(STATUS "Skipping app: ${APP_NAME}")
	endif ()
endfunction()

# Add apps here
add_app(ros2)

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

# Include Google Test configuration
include(gtest)

ament_package()
