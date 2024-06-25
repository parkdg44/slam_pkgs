
# find ROS2 dependencies
set(ROS2_LIBS ${ROS2_LIBS} ament_cmake)
set(ROS2_LIBS ${ROS2_LIBS} rclcpp)
set(ROS2_LIBS ${ROS2_LIBS} sensor_msgs)
set(ROS2_LIBS ${ROS2_LIBS} std_msgs)
set(ROS2_LIBS ${ROS2_LIBS} nav_msgs)
set(ROS2_LIBS ${ROS2_LIBS} geometry_msgs)
set(ROS2_LIBS ${ROS2_LIBS} tf2)
set(ROS2_LIBS ${ROS2_LIBS} tf2_ros)
set(ROS2_LIBS ${ROS2_LIBS} cv_bridge)

foreach(ROS2_LIB ${ROS2_LIBS})
	find_package(${ROS2_LIB} REQUIRED)
endforeach()