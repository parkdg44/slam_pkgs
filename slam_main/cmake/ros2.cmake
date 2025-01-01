
# find ROS2 dependencies
list(APPEND ROS2_LIBS ament_cmake)
list(APPEND ROS2_LIBS rclcpp)
list(APPEND ROS2_LIBS sensor_msgs)
list(APPEND ROS2_LIBS std_msgs)
list(APPEND ROS2_LIBS nav_msgs)
list(APPEND ROS2_LIBS geometry_msgs)
list(APPEND ROS2_LIBS tf2)
list(APPEND ROS2_LIBS tf2_ros)
list(APPEND ROS2_LIBS cv_bridge)

foreach(ROS2_LIB ${ROS2_LIBS})
	find_package(${ROS2_LIB} REQUIRED)
endforeach()