//
// Created by park on 24. 2. 14.
//

#include "rclcpp/rclcpp.hpp"
#include "ros2_node.hpp"
#include "slam_pkg/type/types.hpp"
#include "slam_pkg/util/utils.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam::ros2::SlamNode>());
  rclcpp::shutdown();

  return 0;
}
