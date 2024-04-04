//
// Created by park on 24. 2. 14.
//

#include "ros2/ros2_node.hpp"
#include "type/types.hpp"
#include "util/utils.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam::ros2::Node>());
  rclcpp::shutdown();
  return 0;
}
