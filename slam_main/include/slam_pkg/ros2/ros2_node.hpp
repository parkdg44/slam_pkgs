//
// Created by park on 24. 4. 2.
//

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros2/ros2_converter.hpp"

namespace Slam::ros2 {

class Node : public rclcpp::Node {
 public:
  Node() : rclcpp::Node("slam_main") { RCLCPP_INFO(get_logger(), "aaa"); }

 private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
};

}  // namespace Slam::ros2
